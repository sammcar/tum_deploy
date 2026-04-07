#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <string>
#include <boost/assert.hpp>

#include <dart/dart.hpp>
#include <Eigen/Dense>

#include "robot_config.hpp" // Usa tus constantes exactas

using namespace dart::dynamics;
using Eigen::Matrix4d;
using Eigen::Vector3d;

// ============================================================
// 1. CINEMÁTICA ANALÍTICA (DH) - Tu código original
// ============================================================
inline Matrix4d get_dh_matrix(double theta, double d, double a, double alpha) {
    Matrix4d T;
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0,   sa,       ca,      d,
         0,   0,        0,       1;
    return T;
}

Vector3d solve_FK_local(double theta1, double theta2, double theta3, bool es_pata_derecha) {
    double l1_side = es_pata_derecha ? -L1 : L1;
    Matrix4d T01 = get_dh_matrix(M_PI/2.0, 0, 0, M_PI/2.0);
    Matrix4d T12 = get_dh_matrix(theta1, 0, l1_side, M_PI);
    Matrix4d T23 = get_dh_matrix(M_PI/2.0, 0, 0, -M_PI/2.0);
    Matrix4d T34 = get_dh_matrix(theta2, 0, L2, 0);
    Matrix4d T45 = get_dh_matrix(theta3, 0, L3, 0);
    Matrix4d m_foot = T01 * T12 * T23 * T34 * T45;
    return m_foot.block<3,1>(0,3);
}

// ============================================================
// 2. CONSTRUCTOR AL ESTILO JOSH PIEPER (mjmech)
// ============================================================

struct LegDimensions {
    Vector3d shoulder_offset;
    Vector3d femur_len;
    Vector3d tibia_len;
};

// Función modular que ensambla una extremidad directamente al chasis
void MakeLeg(SkeletonPtr skel, BodyNode* chasis, int leg_idx, const std::string& prefix) {
    bool es_derecha = (leg_idx == 1 || leg_idx == 3);

    LegDimensions dims;
    // Las longitudes deben ser estrictamente +Z para pasar los Asserts de Josh
    dims.femur_len = Vector3d(0.0, 0.0, L2);
    dims.tibia_len = Vector3d(0.0, 0.0, L3);
    
    // Al rotar la pata 180° más adelante, el eje Y local se invierte respecto al mundo.
    // Compensamos esto enviando el offset en dirección contraria:
    dims.shoulder_offset = Vector3d(0.0, es_derecha ? L1 : -L1, 0.0);

    // 1. Sanity checks exactos de Josh
    BOOST_ASSERT(dims.femur_len.x() == 0.0);
    BOOST_ASSERT(dims.femur_len.y() == 0.0);
    BOOST_ASSERT(dims.femur_len.z() > 0.0);
    BOOST_ASSERT(dims.tibia_len.x() == 0.0);
    BOOST_ASSERT(dims.tibia_len.y() == 0.0);
    BOOST_ASSERT(dims.tibia_len.z() > 0.0);

    // --- A. SHOULDER (Coxa) ---
    RevoluteJoint::Properties s_props;
    s_props.mName = "shoulder_" + prefix;
    s_props.mAxis = Vector3d::UnitX(); 
    
    Eigen::Isometry3d T_hip = Eigen::Isometry3d::Identity();
    T_hip.translation() = HIP_OFFSETS[leg_idx];
    
    // ¡EL TRUCO DE JOSH! Rotar 180° en X voltea la pata hacia el piso (-Z)
    // y naturalmente convierte el UnitY() de los motores en -UnitY() del mundo real.
    T_hip.linear() = Eigen::AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
    s_props.mT_ParentBodyToJoint = T_hip;

    s_props.mT_ChildBodyToJoint.translation() = -dims.shoulder_offset;
    
    auto [shoulder_joint, shoulder_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        chasis, s_props, BodyNode::AspectProperties("shoulder_body_" + prefix));

    // --- B. FEMUR ---
    RevoluteJoint::Properties f_props;
    f_props.mName = "femur_" + prefix;
    f_props.mAxis = Vector3d::UnitY(); // Estilo Josh: siempre positivo
    f_props.mT_ChildBodyToJoint.translation() = -dims.femur_len; 

    auto [femur_joint, femur_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        shoulder_body, f_props, BodyNode::AspectProperties("femur_body_" + prefix));

    // --- C. TIBIA ---
    RevoluteJoint::Properties t_props;
    t_props.mName = "tibia_" + prefix;
    t_props.mAxis = Vector3d::UnitY(); // Estilo Josh: siempre positivo
    t_props.mT_ChildBodyToJoint.translation() = -dims.tibia_len;

    auto [tibia_joint, tibia_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        femur_body, t_props, BodyNode::AspectProperties("tibia_body_" + prefix));

    // --- D. FOOT ---
    WeldJoint::Properties foot_props;
    foot_props.mName = "foot_joint_" + prefix;
    
    auto [foot_joint, foot_body] = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia_body, foot_props, BodyNode::AspectProperties("foot_" + prefix));
        
    // Truco de masa masiva para el efector final
    foot_body->setMass(1e6); 
}

// Ensamblador Maestro
SkeletonPtr CreateFullRobotSkeleton() {
    SkeletonPtr skel = Skeleton::create("ATOM-51");
    skel->setGravity(Vector3d(0, 0, -9.81)); 

    FreeJoint::Properties chasis_joint_props;
    chasis_joint_props.mName = "base_joint";
    
    auto [base_joint, chasis_body] = skel->createJointAndBodyNodePair<FreeJoint>(
        nullptr, chasis_joint_props, BodyNode::AspectProperties("chassis"));
    
    // Masa central
    chasis_body->setMass(5.65);

    const std::string leg_names[4] = {"FL", "FR", "BL", "BR"};
    
    // Instanciar y anclar las 4 patas
    for (int i = 0; i < 4; ++i) {
        MakeLeg(skel, chasis_body, i, leg_names[i]);
    }

    return skel;
}

// ============================================================
// 3. MAIN DE VALIDACIÓN COMPLETA
// ============================================================
int main() {
    std::cout << "==================================================\n";
    std::cout << "    DEBUGGER: CUERPO COMPLETO DART VS ANALITICA   \n";
    std::cout << "==================================================\n";

    SkeletonPtr robot = CreateFullRobotSkeleton();
    
    auto chasis = robot->getJoint("base_joint");
    Eigen::Vector6d base_pos = Eigen::Vector6d::Zero();
    chasis->setPositions(base_pos);

    std::vector<std::pair<std::string, Vector3d>> pruebas = {
        {"Stand", Vector3d(0.0, 30.0, -60.0)},
        {"Paso Adelante", Vector3d(0.0, 45.0, -45.0)},
        {"Abduccion", Vector3d(15.0, 30.0, -60.0)}
    };

    const std::string leg_names[4] = {"FL", "FR", "BL", "BR"};

    for (const auto& prueba : pruebas) {
        std::cout << "\n>>> TEST: " << prueba.first << " (Angulos: " 
                  << prueba.second.transpose() << " deg)\n";
        
        Vector3d ang_rad = prueba.second * (M_PI / 180.0);

        for (int i = 0; i < 4; ++i) {
            bool es_derecha = (i == 1 || i == 3);
            std::string prefijo = leg_names[i];

            robot->getJoint("shoulder_" + prefijo)->setPosition(0, ang_rad(0));
            robot->getJoint("femur_" + prefijo)->setPosition(0, ang_rad(1));
            robot->getJoint("tibia_" + prefijo)->setPosition(0, ang_rad(2));
            robot->computeForwardKinematics();

            Vector3d pos_dart = robot->getBodyNode("foot_" + prefijo)->getTransform().translation();

            Vector3d pos_local_analitica = solve_FK_local(ang_rad(0), ang_rad(1), ang_rad(2), es_derecha);
            Vector3d pos_global_analitica = HIP_OFFSETS[i] + pos_local_analitica;

            std::cout << "  Leg " << leg_names[i] << ":\n";
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "    DART  : " << pos_dart.transpose() << "\n";
            std::cout << "    ANALIT: " << pos_global_analitica.transpose() << "\n";
            std::cout << "    ERROR : " << (pos_dart - pos_global_analitica).norm() * 1000.0 << " mm\n";
        }
        std::cout << "--------------------------------------------------\n";
    }

    return 0;
}