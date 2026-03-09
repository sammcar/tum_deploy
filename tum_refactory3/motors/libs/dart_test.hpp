#ifndef CONTROL_DART_HPP
#define CONTROL_DART_HPP

#include <dart/dart.hpp>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace dart::dynamics;
using Eigen::Vector3d;


// --- 1. CONSTANTES GLOBALES (Default) ---
// Nota: Se definen como 'inline' para evitar errores de redefinición
inline Vector3d kd_base{0.1, 0.1, 0.1}; //inline Vector3d kd_base{2.0, 2.0, 5.0};
inline Vector3d kp_base{0.0, 0.0, 0.0}; //inline Vector3d kp_base{500.0, 500.0, 200.0}; 
inline double masa_pierna_def = 1.0;

// --- 2. ESTRUCTURAS ---
struct EstadoRealMotores {
    Vector3d angulos_rad;
    Vector3d velocidades_rad_s;
};

struct EstadoDeseadoCartesiano {
    Vector3d posicion;
    Vector3d velocidad;
    Vector3d aceleracion;
    double stance_actual = 0.0;
    double kp_scale = 1.0;
    double kd_scale = 1.0;
};

struct ComandosMotor {
    Vector3d velocidades_rad_s;
    Vector3d torques_Nm;
};

struct LegDimensions {
    Vector3d shoulder_offset = {0.0, 0.093, 0.0}; 
    Vector3d femur_len = {0.0, 0.0, -0.147};      
    Vector3d tibia_len = {0.0, 0.0, -0.230};      
};

// --- 3. FUNCIONES MODULARES ---
inline SkeletonPtr CreateLegSkeleton(const LegDimensions& dims) {
    SkeletonPtr skel = Skeleton::create("pata_TUM");
    skel->setGravity(Vector3d(0, 0, 0)); 

    // 1. SHOULDER (Coxa)
    RevoluteJoint::Properties s_props;
    s_props.mName = "shoulder_joint";
    s_props.mAxis = Vector3d::UnitX(); 
    s_props.mT_ChildBodyToJoint.translation() = -dims.shoulder_offset;
    auto [shoulder_joint, shoulder_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, s_props, BodyNode::AspectProperties("shoulder"));
    shoulder_body->setMass(0.01);

    // 2. FEMUR
    RevoluteJoint::Properties f_props;
    f_props.mName = "femur_joint";
    f_props.mAxis = -Vector3d::UnitY(); 
    Eigen::Isometry3d f_joint_pos = Eigen::Isometry3d::Identity();
    f_joint_pos.translation() = -dims.femur_len;
    f_joint_pos.rotate(Eigen::AngleAxisd(M_PI/2.0, Vector3d::UnitY())); 
    f_props.mT_ChildBodyToJoint = f_joint_pos;

    auto [femur_joint, femur_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        shoulder_body, f_props, BodyNode::AspectProperties("femur"));
    femur_body->setMass(0.01);

    // 3. TIBIA
    RevoluteJoint::Properties t_props;
    t_props.mName = "tibia_joint";
    t_props.mAxis = -Vector3d::UnitY(); 
    Eigen::Isometry3d t_joint_pos = Eigen::Isometry3d::Identity();
    t_joint_pos.translation() = -dims.tibia_len;
    t_props.mT_ChildBodyToJoint = t_joint_pos;

    auto [tibia_joint, tibia_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        femur_body, t_props, BodyNode::AspectProperties("tibia"));
    tibia_body->setMass(0.01);

    // 4. FOOT (¡Aquí estaba el error de SegFault!)
    WeldJoint::Properties foot_props;
    foot_props.mName = "foot_joint";
    auto [foot_joint, foot_body] = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia_body, foot_props, BodyNode::AspectProperties("foot"));
    foot_body->setMass(1e6);

    return skel;
}

inline Vector3d calcular_FK_posicion(Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node, const Vector3d& angulos_rad) {
    coxa->setPosition(0, angulos_rad(0));
    femur->setPosition(0, angulos_rad(1));
    tibia->setPosition(0, angulos_rad(2));
    pie_node->getSkeleton()->computeForwardKinematics();
    return pie_node->getTransform().translation();
}

inline Vector3d calcular_FK_velocidad(Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node, const Vector3d& velocidades_rad_s) {
    coxa->setVelocity(0, velocidades_rad_s(0));
    femur->setVelocity(0, velocidades_rad_s(1));
    tibia->setVelocity(0, velocidades_rad_s(2));
    pie_node->getSkeleton()->computeForwardKinematics();
    return pie_node->getCOMLinearVelocity();
}

inline Vector3d calcular_F_P(const Vector3d& pos_deseada, const Vector3d& pos_real, const Vector3d& Kp) {
    return Kp.cwiseProduct(pos_deseada - pos_real);
}

inline Vector3d calcular_F_D(const Vector3d& vel_deseada, const Vector3d& vel_real, const Vector3d& Kd) {
    return Kd.cwiseProduct(vel_deseada - vel_real);
}

inline Vector3d calcular_F_accel(const Vector3d& accel_deseada, double mp, double mc, double fa, double es) {
    double masa_dinamica = mp + es * ((mc * fa) - mp);
    return accel_deseada * masa_dinamica;
}

inline Eigen::Matrix3d calcular_jacobiano_lineal(SkeletonPtr pata, BodyNode* pie_node) {
    return pata->getLinearJacobian(pie_node).block<3, 3>(0, 0);
}

inline Eigen::Matrix3d calcular_jacobiano_fuerza(SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node) {
    Eigen::Matrix3d J_fuerza;
    coxa->setForce(0, 0.0); femur->setForce(0, 0.0); tibia->setForce(0, 0.0);
    pata->computeForwardDynamics();
    Vector3d baseline = pie_node->getCOMLinearAcceleration();

    for (int axis = 0; axis < 3; axis++) {
        coxa->setForce(0,  axis == 0 ? 1.0 : 0.0);
        femur->setForce(0, axis == 1 ? 1.0 : 0.0);
        tibia->setForce(0, axis == 2 ? 1.0 : 0.0);
        pata->computeForwardDynamics();
        J_fuerza.col(axis) = (pie_node->getCOMLinearAcceleration() - baseline);
    }
    return J_fuerza;
}

// --- 4. FUNCIÓN MAESTRA ---
inline ComandosMotor calcular_comandos_motores(
    SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node,
    const EstadoRealMotores& real, const EstadoDeseadoCartesiano& deseado)
{
    Vector3d pos_real = calcular_FK_posicion(coxa, femur, tibia, pie_node, real.angulos_rad);
    Vector3d vel_real = calcular_FK_velocidad(coxa, femur, tibia, pie_node, real.velocidades_rad_s);

    Vector3d Kp_final = kp_base * deseado.kp_scale;
    Vector3d Kd_final = kd_base * deseado.kd_scale;

    Vector3d F_P = 
    (deseado.posicion, pos_real, Kp_final);
    Vector3d F_D = calcular_F_D(deseado.velocidad, vel_real, Kd_final);
    Vector3d F_accel = calcular_F_accel(deseado.aceleracion, masa_pierna_def, 8.0, 0.5, deseado.stance_actual);
    
    Vector3d fuerza_total = F_P + F_D + F_accel;

    Eigen::Matrix3d J_lineal = calcular_jacobiano_lineal(pata, pie_node);
    Eigen::Matrix3d J_fuerza = calcular_jacobiano_fuerza(pata, coxa, femur, tibia, pie_node);

    ComandosMotor comandos;
    comandos.velocidades_rad_s = J_lineal.inverse() * deseado.velocidad;
    comandos.torques_Nm = (J_fuerza.inverse() * fuerza_total) * 1e-6;

    return comandos;
}

#endif // CONTROL_DART_HPP
