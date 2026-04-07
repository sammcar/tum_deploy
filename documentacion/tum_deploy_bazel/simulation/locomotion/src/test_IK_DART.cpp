#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>

#include <dart/dart.hpp>
#include <Eigen/Dense>

using namespace dart::dynamics;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;

// ============================================================
// 1. CONSTANTES GLOBALES
// ============================================================
inline const double L1 = 0.093;
inline const double L2 = 0.147;
inline const double L3 = 0.230;

// ============================================================
// 2. ESTRUCTURAS
// ============================================================
struct LegAngles {
    double th1, th2, th3;
    bool valid;
};

struct LegDimensions {
    Vector3d shoulder_offset = {0.0, L1, 0.0};
    Vector3d femur_len = {0.0, 0.0, -L2};
    Vector3d tibia_len = {0.0, 0.0, -L3};
};

// ============================================================
// 3. CINEMÁTICA ANALÍTICA (DH)
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

Vector3d solve_FK(double theta1, double theta2, double theta3, bool es_pata_derecha) {
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
// 4. MODELO DART CORREGIDO
// ============================================================
// ============================================================
// 4. MODELO DART CORREGIDO
// ============================================================
inline SkeletonPtr CreateLegSkeleton(const LegDimensions& dims) {
    SkeletonPtr skel = Skeleton::create("pata_TUM");
    skel->setGravity(Vector3d(0, 0, 0)); 

    // 1. SHOULDER (Coxa)
    RevoluteJoint::Properties s_props;
    s_props.mName = "shoulder_joint";
    s_props.mAxis = Vector3d::UnitX(); 
    // La articulación está en (0,0,0). El centro del hombro está hacia +Y.
    s_props.mT_ChildBodyToJoint.translation() = -dims.shoulder_offset;
    auto [shoulder_joint, shoulder_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, s_props, BodyNode::AspectProperties("shoulder"));
    shoulder_body->setMass(0.01);

    // 2. FEMUR
    RevoluteJoint::Properties f_props;
    f_props.mName = "femur_joint";
    f_props.mAxis = -Vector3d::UnitY(); 
    
    // Nace exactamente en el centro del BodyNode del hombro
    f_props.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity(); 
    
    // El fémur crece hacia -Z. Por lo tanto, la articulación (vista desde el fémur) 
    // está en sentido contrario (+Z). Usamos directamente la traslación pura.
    f_props.mT_ChildBodyToJoint.translation() = -dims.femur_len; 

    auto [femur_joint, femur_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        shoulder_body, f_props, BodyNode::AspectProperties("femur"));
    femur_body->setMass(0.01);

    // 3. TIBIA
    RevoluteJoint::Properties t_props;
    t_props.mName = "tibia_joint";
    t_props.mAxis = -Vector3d::UnitY(); 
    
    // Nace exactamente al final del fémur
    t_props.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
    
    // La tibia crece hacia -Z.
    t_props.mT_ChildBodyToJoint.translation() = -dims.tibia_len;

    auto [tibia_joint, tibia_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        femur_body, t_props, BodyNode::AspectProperties("tibia"));
    tibia_body->setMass(0.01);

    // 4. FOOT
    WeldJoint::Properties foot_props;
    foot_props.mName = "foot_joint";
    
    // El pie se suelda rígidamente al final de la tibia sin desplazamientos
    foot_props.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
    foot_props.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();
    
    auto [foot_joint, foot_body] = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia_body, foot_props, BodyNode::AspectProperties("foot"));
    foot_body->setMass(1e6); // Masa alta solo para debug

    return skel;
}

inline Vector3d calcular_FK_posicion(Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node, const Vector3d& angulos_rad) {
    coxa->setPosition(0, angulos_rad(0));
    femur->setPosition(0, angulos_rad(1));
    tibia->setPosition(0, angulos_rad(2));
    pie_node->getSkeleton()->computeForwardKinematics();
    return pie_node->getTransform().translation();
}

// ============================================================
// 5. MAIN CON SEGURIDAD
// ============================================================
int main() {
    std::cout << "==================================================\n";
    std::cout << "    DEBUGGER GEOMETRICO DART VS ANALITICA         \n";
    std::cout << "==================================================\n";

    LegDimensions dims; 
    SkeletonPtr pata_dart = CreateLegSkeleton(dims);
    
    // Verificación de punteros para evitar SegFault
    auto coxa_j = pata_dart->getJoint("shoulder_joint");
    auto femur_j = pata_dart->getJoint("femur_joint");
    auto tibia_j = pata_dart->getJoint("tibia_joint");
    auto pie_node = pata_dart->getBodyNode("foot");

    if (!coxa_j || !femur_j || !tibia_j || !pie_node) {
        std::cerr << "CRITICAL ERROR: No se encontro algun componente del esqueleto!\n";
        return 1;
    }

    std::vector<std::pair<std::string, Vector3d>> pruebas = {
        {"1. Todo Cero", Vector3d(0.0, 0.0, 0.0)},
        {"2. Femur 45 deg", Vector3d(0.0, 45.0, 0.0)},
        {"3. Tibia 45 deg", Vector3d(0.0, 0.0, 45.0)},
        {"4. Stand Tipico", Vector3d(0.0, 30.0, -60.0)}
    };

    for (const auto& prueba : pruebas) {
        Vector3d ang_rad = prueba.second * (M_PI / 180.0);

        Vector3d pos_dart = calcular_FK_posicion(coxa_j, femur_j, tibia_j, pie_node, ang_rad);
        Vector3d pos_analitica = solve_FK(ang_rad(0), ang_rad(1), ang_rad(2), false);

        std::cout << "TEST: " << prueba.first << "\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "DART: " << pos_dart.transpose() << "\n";
        std::cout << "ANLIT: " << pos_analitica.transpose() << "\n";
        std::cout << "ERROR: " << (pos_dart - pos_analitica).norm()*1000.0 << " mm\n";
        std::cout << "--------------------------------------------------\n";
    }

    return 0;
}