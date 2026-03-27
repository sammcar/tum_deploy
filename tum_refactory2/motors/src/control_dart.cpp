#include "control_dart.hpp" // Incluye el menú que acabamos de crear
#include <iostream>
#include <cmath>

// --- 1. INICIALIZACIÓN DE CONSTANTES GLOBALES ---
// Aquí se les asigna su valor real en memoria.
Vector3d kd_base{1.0, 1.0, 2.0}; 
Vector3d kp_base{200.0, 200.0, 400.0}; 
double masa_pierna_def = 1.4;


// --- 2. IMPLEMENTACIÓN DE FUNCIONES ---


// --- NUEVO CREADOR PRINCIPAL ---
SkeletonPtr CreateLegSkeleton(const LegDimensions& dims, bool is_right) {
    SkeletonPtr skel = Skeleton::create(is_right ? "pata_derecha" : "pata_izquierda");
    skel->setGravity(Vector3d(0, 0, 0)); 

    // --- CORRECCIÓN CRÍTICA: FACTOR DE SIMETRÍA ---
    // Si es derecha, invertimos el eje Y para el montaje
    double y_sign = is_right ? -1.0 : 1.0;

    // 1. SHOULDER (Coxa)
    RevoluteJoint::Properties s_props;
    s_props.mName = "shoulder_joint";
    s_props.mAxis = Vector3d::UnitX(); 
    
    // Aplicamos el espejo al offset del hombro
    Vector3d actual_shoulder_offset = dims.shoulder_offset;
    actual_shoulder_offset.y() *= y_sign; 
    s_props.mT_ChildBodyToJoint.translation() = -actual_shoulder_offset;
    
    auto [shoulder_joint, shoulder_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, s_props, BodyNode::AspectProperties("shoulder"));
    shoulder_body->setMass(0.01);

    // 2. FEMUR
    RevoluteJoint::Properties f_props;
    f_props.mName = "femur_joint";
    // Espejo en el eje de rotación: izquierda usa -Y, derecha usa +Y
    f_props.mAxis = (is_right ? 1.0 : -1.0) * Vector3d::UnitY();
    
    f_props.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity(); 
    f_props.mT_ChildBodyToJoint.translation() = -dims.femur_len; 

    auto [femur_joint, femur_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        shoulder_body, f_props, BodyNode::AspectProperties("femur"));
    femur_body->setMass(0.01);

    // 3. TIBIA
    RevoluteJoint::Properties t_props;
    t_props.mName = "tibia_joint";
    // Espejo en el eje de rotación: izquierda usa -Y, derecha usa +Y
    t_props.mAxis = (is_right ? 1.0 : -1.0) * Vector3d::UnitY();
    
    t_props.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
    t_props.mT_ChildBodyToJoint.translation() = -dims.tibia_len;

    auto [tibia_joint, tibia_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        femur_body, t_props, BodyNode::AspectProperties("tibia"));
    tibia_body->setMass(0.01);

    // 4. FOOT
    WeldJoint::Properties foot_props;
    foot_props.mName = "foot_joint";
    
    foot_props.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
    foot_props.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();
    
    auto [foot_joint, foot_body] = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia_body, foot_props, BodyNode::AspectProperties("foot"));
    foot_body->setMass(1e6); // Masa alta solo para debug

    return skel;
}

EstadoCartesianoReal calcular_FK_estado(Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node, 
                                        const Vector3d& angulos_rad, const Vector3d& velocidades_rad_s) 
{
    coxa->setPosition(0, angulos_rad(0));
    femur->setPosition(0, angulos_rad(1));
    tibia->setPosition(0, angulos_rad(2));

    coxa->setVelocity(0, velocidades_rad_s(0));
    femur->setVelocity(0, velocidades_rad_s(1));
    tibia->setVelocity(0, velocidades_rad_s(2));

    coxa->setForce(0, 0.0);
    femur->setForce(0, 0.0);
    tibia->setForce(0, 0.0);

    auto skeleton = pie_node->getSkeleton();
    skeleton->computeForwardKinematics();
    skeleton->computeForwardDynamics();

    EstadoCartesianoReal resultado;
    resultado.posicion = pie_node->getCOM(); 
    resultado.velocidad = pie_node->getCOMLinearVelocity();

    return resultado;
}

Vector3d calcular_F_P(const Vector3d& pos_deseada, const Vector3d& pos_real, const Vector3d& Kp) {
    return Kp.cwiseProduct(pos_deseada - pos_real);
}

Vector3d calcular_F_D(const Vector3d& vel_deseada, const Vector3d& vel_real, const Vector3d& Kd) {
    return Kd.cwiseProduct(vel_deseada - vel_real);
}

Vector3d calcular_F_accel(const Vector3d& accel_deseada, double mp, double mc, double fa, double es) {
    double masa_dinamica = mp + es * ((mc * fa) - mp);
    Vector3d gravedad_compensacion(0.0, 0.0, 9.81); 
    Vector3d accel_total = accel_deseada + gravedad_compensacion;
    return accel_total * masa_dinamica;
}

Eigen::Matrix3d calcular_jacobiano_lineal(BodyNode* pie_node) {
    return pie_node->getLinearJacobian(); 
}

Eigen::Matrix3d calcular_jacobiano_fuerza(SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node) {
    Eigen::Matrix3d J_fuerza;
    coxa->setForce(0, 0.0); femur->setForce(0, 0.0); tibia->setForce(0, 0.0);
    pata->computeForwardDynamics();
    pata->computeForwardDynamics();
    Vector3d baseline = pie_node->getCOMLinearAcceleration();

    for (int axis = 0; axis < 3; axis++) {
        coxa->setForce(0,  axis == 0 ? 1.0 : 0.0);
        femur->setForce(0, axis == 1 ? 1.0 : 0.0);
        tibia->setForce(0, axis == 2 ? 1.0 : 0.0);
        pata->computeForwardDynamics();
        J_fuerza.col(axis) = (pie_node->getCOMLinearAcceleration() - baseline);
    }

    coxa->setForce(0, 0.0); 
    femur->setForce(0, 0.0); 
    tibia->setForce(0, 0.0);

    return J_fuerza;
}

ComandosMotor calcular_comandos_motores(SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node,
                                        const EstadoRealMotores& real, const EstadoDeseadoCartesiano& deseado)
{
    EstadoCartesianoReal estado_real = calcular_FK_estado(coxa, femur, tibia, pie_node, real.angulos_rad, real.velocidades_rad_s);
    Vector3d pos_real = estado_real.posicion;
    Vector3d vel_real = estado_real.velocidad;

    Vector3d Kp_final = kp_base * deseado.kp_scale;
    Vector3d Kd_final = kd_base * deseado.kd_scale;

    Vector3d F_P = calcular_F_P(deseado.posicion, pos_real, Kp_final);
    Vector3d F_D = calcular_F_D(deseado.velocidad, vel_real, Kd_final);
    Vector3d F_accel = calcular_F_accel(deseado.aceleracion, masa_pierna_def, 8.0, 0.5, deseado.stance_actual);
    
    Vector3d fuerza_total = F_P + F_D + F_accel;

    coxa->setVelocity(0, 0.0);
    femur->setVelocity(0, 0.0);
    tibia->setVelocity(0, 0.0);

    coxa->setForce(0, 0.0);
    femur->setForce(0, 0.0);
    tibia->setForce(0, 0.0);

    pata->computeForwardKinematics(); 
    pata->computeForwardDynamics(); 

    Eigen::Matrix3d J_lineal = calcular_jacobiano_lineal(pie_node);
    Eigen::Matrix3d J_fuerza = calcular_jacobiano_fuerza(pata, coxa, femur, tibia, pie_node);

    ComandosMotor comandos;
    comandos.velocidades_rad_s = J_lineal.inverse() * deseado.velocidad;
    comandos.torques_Nm = (J_fuerza.inverse() * fuerza_total) * 1e-6; 

    return comandos;
}
