#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include <dart/dart.hpp>
#include <Eigen/Dense>

#include "robot_config.hpp" // Usa tus constantes exactas

using namespace dart::dynamics;
using Eigen::Vector3d;

// ============================================================
// 1. CONSTRUCTOR AL ESTILO JOSH PIEPER
// ============================================================
struct LegDimensions {
    Vector3d shoulder_offset;
    Vector3d femur_len;
    Vector3d tibia_len;
};

void MakeLeg(SkeletonPtr skel, BodyNode* chasis, int leg_idx, const std::string& prefix) {
    bool es_derecha = (leg_idx == 1 || leg_idx == 3);
    LegDimensions dims;
    dims.femur_len = Vector3d(0.0, 0.0, L2);
    dims.tibia_len = Vector3d(0.0, 0.0, L3);
    dims.shoulder_offset = Vector3d(0.0, es_derecha ? L1 : -L1, 0.0);

    RevoluteJoint::Properties s_props;
    s_props.mName = "shoulder_" + prefix;
    s_props.mAxis = Vector3d::UnitX(); 
    
    Eigen::Isometry3d T_hip = Eigen::Isometry3d::Identity();
    T_hip.translation() = HIP_OFFSETS[leg_idx];
    T_hip.linear() = Eigen::AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
    
    s_props.mT_ParentBodyToJoint = T_hip;
    s_props.mT_ChildBodyToJoint.translation() = -dims.shoulder_offset;
    
    auto [shoulder_joint, shoulder_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        chasis, s_props, BodyNode::AspectProperties("shoulder_body_" + prefix));

    RevoluteJoint::Properties f_props;
    f_props.mName = "femur_" + prefix;
    f_props.mAxis = Vector3d::UnitY();
    f_props.mT_ChildBodyToJoint.translation() = -dims.femur_len; 

    auto [femur_joint, femur_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        shoulder_body, f_props, BodyNode::AspectProperties("femur_body_" + prefix));

    RevoluteJoint::Properties t_props;
    t_props.mName = "tibia_" + prefix;
    t_props.mAxis = Vector3d::UnitY();
    t_props.mT_ChildBodyToJoint.translation() = -dims.tibia_len;

    auto [tibia_joint, tibia_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        femur_body, t_props, BodyNode::AspectProperties("tibia_body_" + prefix));

    WeldJoint::Properties foot_props;
    foot_props.mName = "foot_joint_" + prefix;
    auto [foot_joint, foot_body] = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia_body, foot_props, BodyNode::AspectProperties("foot_" + prefix));
}

SkeletonPtr CreateFullRobotSkeleton() {
    SkeletonPtr skel = Skeleton::create("ATOM-51");
    
    // IMPORTANTE: Para aislar el cálculo de fuerza y que sea una comparativa justa con J^T * F,
    // apagamos la gravedad momentáneamente. J^T * F no incluye compensación de gravedad
    // por defecto a menos que se le sume un término G(q) extra.
    skel->setGravity(Vector3d(0, 0, 0)); 

    FreeJoint::Properties chasis_joint_props;
    chasis_joint_props.mName = "base_joint";
    auto [base_joint, chasis_body] = skel->createJointAndBodyNodePair<FreeJoint>(
        nullptr, chasis_joint_props, BodyNode::AspectProperties("chassis"));

    const std::string leg_names[4] = {"FL", "FR", "BL", "BR"};
    for (int i = 0; i < 4; ++i) { MakeLeg(skel, chasis_body, i, leg_names[i]); }
    return skel;
}

// ============================================================
// 2. LA VERDADERA COMPARATIVA: INVERSE DYNAMICS vs J^T*F
// ============================================================
int main() {
    SkeletonPtr robot = CreateFullRobotSkeleton();
    auto chasis = robot->getJoint("base_joint");
    chasis->setPositions(Eigen::Vector6d::Zero());

    auto coxa  = robot->getJoint("shoulder_FL");
    auto femur = robot->getJoint("femur_FL");
    auto tibia = robot->getJoint("tibia_FL");
    auto foot  = robot->getBodyNode("foot_FL");

    std::cout << "=================================================================\n";
    std::cout << "   DART INVERSE DYNAMICS vs ENFOQUE HIBRIDO (J^T * F)            \n";
    std::cout << "=================================================================\n";
    std::cout << "Fuerza Deseada (El robot empujando el suelo): 5N (X), 25N (Z)\n\n";

    double th_coxa = 0.0, th_femur = 30.0 * (M_PI/180.0), th_tibia = -60.0 * (M_PI/180.0);
    
    coxa->setPosition(0, th_coxa);
    femur->setPosition(0, th_femur);
    tibia->setPosition(0, th_tibia);
    
    // Aseguramos que el robot esté estático (v = 0, a = 0)
    robot->computeForwardKinematics();
    robot->computeForwardDynamics(); 

    Vector3d F_deseada(5.0, 0.0, 25.0); 

    // -----------------------------------------------------------------
    // MÉTODO 1: DART NATIVE INVERSE DYNAMICS
    // -----------------------------------------------------------------
    // Newton: Si el pie empuja el suelo con F_deseada, el suelo reacciona con -F_deseada
    foot->addExtForce(-F_deseada); 
    
    // DART calcula qué torques se necesitan en las articulaciones para equilibrar esta fuerza
    robot->computeInverseDynamics(true);
    
    Vector3d torques_DART(coxa->getForce(0), femur->getForce(0), tibia->getForce(0));
    
    // Limpiamos las fuerzas para el siguiente método
    foot->clearExternalForces(); 

    // -----------------------------------------------------------------
    // MÉTODO 2: HÍBRIDO (Extraer Jacobiano Lineal de DART y usar J^T * F)
    // -----------------------------------------------------------------
    // Le pedimos a DART el Jacobiano Lineal (3xN) del pie respecto al chasis
    dart::math::LinearJacobian J_dart = foot->getLinearJacobian(robot->getBodyNode("chassis"));
    
    // Extraemos solo las columnas que corresponden a los motores de nuestra pata FL
    Eigen::Matrix3d J_pierna;
    J_pierna.col(0) = J_dart.col(coxa->getIndexInSkeleton(0));
    J_pierna.col(1) = J_dart.col(femur->getIndexInSkeleton(0));
    J_pierna.col(2) = J_dart.col(tibia->getIndexInSkeleton(0));

    // La gloriosa fórmula de control de fuerza
    Vector3d torques_Hibrido = J_pierna.transpose() * F_deseada;

    // -----------------------------------------------------------------
    // RESULTADOS
    // -----------------------------------------------------------------
    std::cout << std::fixed << std::setprecision(5);
    std::cout << ">> 1. TORQUES NATIVOS (DART Inverse Dynamics): \n";
    std::cout << "   Coxa : " << std::setw(9) << torques_DART(0) << " Nm\n";
    std::cout << "   Femur: " << std::setw(9) << torques_DART(1) << " Nm\n";
    std::cout << "   Tibia: " << std::setw(9) << torques_DART(2) << " Nm\n\n";

    std::cout << ">> 2. TORQUES HÍBRIDOS (J^T * F con Jacobiano de DART): \n";
    std::cout << "   Coxa : " << std::setw(9) << torques_Hibrido(0) << " Nm\n";
    std::cout << "   Femur: " << std::setw(9) << torques_Hibrido(1) << " Nm\n";
    std::cout << "   Tibia: " << std::setw(9) << torques_Hibrido(2) << " Nm\n\n";

    std::cout << "DIFERENCIA ABSOLUTA: " 
              << (torques_DART - torques_Hibrido).norm() << " Nm\n";

    return 0;
}