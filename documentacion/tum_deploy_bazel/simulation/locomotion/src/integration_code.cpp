#include <dart/dart.hpp>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace dart::dynamics;
using Eigen::Vector3d;

// ============================================================
// MÓDULO DE PLANIFICACIÓN DE MARCHA (GAIT PLANNER)
// ============================================================

struct CartesianState {
    Eigen::Vector3d P_des; 
    Eigen::Vector3d V_des; 
    Eigen::Vector3d A_des; 
    double kp_scale; 
    double kd_scale; 
    bool is_stance; 
};

class LegGaitPlanner {
private:
    int leg_id_;
    Eigen::Vector3d origin_;
    Eigen::Vector3d hip_offset_;
    
    double gait_offset_;
    double duty_factor_;
    double step_duration_;
    double step_h_;

    // Parámetros de ajuste de trayectoria (Swing)
    double world_blend_ = 0.15;
    double damp_start_phase_ = 0.85;
    double damp_scale_kp_ = 0.6;
    double damp_scale_kd_ = 0.4;

    void EvaluateBezierCubic(double t, const Eigen::Vector3d& P0, const Eigen::Vector3d& P1, 
                             const Eigen::Vector3d& P2, const Eigen::Vector3d& P3,
                             Eigen::Vector3d& P_out, Eigen::Vector3d& V_out, Eigen::Vector3d& A_out) {
        double u = 1.0 - t;
        double tt = t * t; double uu = u * u;
        double uuu = uu * u; double ttt = tt * t;

        P_out = uuu * P0 + 3 * uu * t * P1 + 3 * u * tt * P2 + ttt * P3;
        V_out = 3 * uu * (P1 - P0) + 6 * u * t * (P2 - P1) + 3 * tt * (P3 - P2);
        A_out = 6 * u * (P2 - 2 * P1 + P0) + 6 * t * (P3 - 2 * P2 + P1);
    }

public:
    LegGaitPlanner() {}

    void Initialize(int leg_id, const Eigen::Vector3d& origin, const Eigen::Vector3d& hip_offset,
                    double gait_offset, double duty_factor, double step_duration, double step_h) {
        leg_id_ = leg_id;
        origin_ = origin;
        hip_offset_ = hip_offset;
        gait_offset_ = gait_offset;
        duty_factor_ = duty_factor;
        step_duration_ = step_duration;
        step_h_ = step_h;
    }

    CartesianState Update(double global_phase, double vx, double vy, double wz) {
        CartesianState state;
        
        double leg_phase = fmod(global_phase + gait_offset_, 1.0);
        double stance_portion = duty_factor_;
        double swing_portion = 1.0 - duty_factor_;
        
        state.is_stance = (leg_phase >= swing_portion);

        double t_stance = step_duration_ * stance_portion;
        Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0,0,wz).cross(hip_offset_);
        
        Eigen::Vector3d p_start = origin_ - v_total * (t_stance / 2.0);
        Eigen::Vector3d p_target = origin_ + v_total * (t_stance / 2.0);

        if (state.is_stance) {
            double t = (leg_phase - swing_portion) / stance_portion;
            
            state.P_des = p_target + t * (p_start - p_target);
            state.P_des.z() = origin_.z();
            state.V_des = -v_total; 
            state.A_des.setZero();
            
            state.kp_scale = 1.0;
            state.kd_scale = 1.0;
        } 
        else {
            double phi = leg_phase / swing_portion; 
            double flight_duration = step_duration_ * swing_portion;

            Eigen::Vector3d P, V, A;
            P.setZero(); V.setZero(); A.setZero();

            // Eje Z
            double z_max = p_start.z() + step_h_;
            if (phi < 0.5) {
                double phi_z = phi * 2.0; 
                P.z() = p_start.z() + (z_max - p_start.z()) * std::sin(phi_z * M_PI / 2.0);
                V.z() = (z_max - p_start.z()) * std::cos(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            } else {
                double phi_z = (phi - 0.5) * 2.0;
                P.z() = z_max - (z_max - p_target.z()) * (1.0 - std::cos(phi_z * M_PI / 2.0));
                V.z() = -(z_max - p_target.z()) * std::sin(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            }

            // Ejes XY
            Eigen::Vector3d xy_start(p_start.x(), p_start.y(), 0.0);
            Eigen::Vector3d xy_target(p_target.x(), p_target.y(), 0.0);
            Eigen::Vector3d V_world_xy(-v_total.x(), -v_total.y(), 0.0);

            if (phi < world_blend_) {
                double t_local = phi / world_blend_;
                V.head<2>() = V_world_xy.head<2>() * (1.0 - t_local);
                P.head<2>() = xy_start.head<2>() + (V_world_xy.head<2>() * (phi * flight_duration)); 
            } else if (phi < 1.0 - world_blend_) {
                double t_local = (phi - world_blend_) / (1.0 - 2.0 * world_blend_);
                Eigen::Vector3d P1 = xy_start + (xy_target - xy_start) * 0.2;
                Eigen::Vector3d P2 = xy_start + (xy_target - xy_start) * 0.8;
                EvaluateBezierCubic(t_local, xy_start, P1, P2, xy_target, P, V, A);
                V.head<2>() = V.head<2>() / (flight_duration * (1.0 - 2.0 * world_blend_));
            } else {
                double t_local = (phi - (1.0 - world_blend_)) / world_blend_;
                P.head<2>() = xy_target.head<2>(); 
                V.head<2>() = V_world_xy.head<2>() * t_local; 
            }

            state.P_des = P;
            state.V_des = V;
            state.A_des = A;

            // Amortiguación final del vuelo
            if (phi > damp_start_phase_) {
                state.kp_scale = damp_scale_kp_;
                state.kd_scale = damp_scale_kd_;
            } else {
                state.kp_scale = 1.0;
                state.kd_scale = 1.0;
            }
        }

        return state;
    }
};

// --- 1. CONSTANTES GLOBALES ---
Vector3d kd_cartesiano{2.0, 2.0, 5.0}; 
Vector3d kp_cartesiano{500.0, 500.0, 200.0}; 
double masa_pierna_kg = 1.4;

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
    // Escaladores como números simples (double)
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

SkeletonPtr CreateLegSkeleton(const LegDimensions& dims) {
    SkeletonPtr skel = Skeleton::create("pata_TUM");
    skel->setGravity(Vector3d(0, 0, 0)); 

    // --- COXA / HOMBRO ---
    RevoluteJoint::Properties s_props;
    s_props.mName = "shoulder_joint";
    s_props.mAxis = Vector3d::UnitX(); 
    s_props.mT_ChildBodyToJoint.translation() = -dims.shoulder_offset;
    auto [shoulder_joint, shoulder_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, s_props, BodyNode::AspectProperties("shoulder"));
    shoulder_body->setMass(0.01); 

    // --- FÉMUR ---
    RevoluteJoint::Properties f_props;
    f_props.mName = "femur_joint";
    f_props.mAxis = Vector3d::UnitY(); 
    f_props.mT_ChildBodyToJoint.translation() = -dims.femur_len;
    auto [femur_joint, femur_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        shoulder_body, f_props, BodyNode::AspectProperties("femur"));
    femur_body->setMass(0.01); 

    // --- TIBIA ---
    RevoluteJoint::Properties t_props;
    t_props.mName = "tibia_joint";
    t_props.mAxis = Vector3d::UnitY();
    t_props.mT_ChildBodyToJoint.translation() = -dims.tibia_len;
    auto [tibia_joint, tibia_body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        femur_body, t_props, BodyNode::AspectProperties("tibia"));
    tibia_body->setMass(0.01); 

    // --- PIE (Masa ficticia para Jacobiano de fuerza) ---
    WeldJoint::Properties foot_props;
    foot_props.mName = "foot_joint";
    auto foot_body = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia_body, foot_props, BodyNode::AspectProperties("foot")).second;
    foot_body->setMass(1e6); 

    return skel;
}

Vector3d calcular_FK_posicion(Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node, const Vector3d& angulos_rad) {
    coxa->setPosition(0, angulos_rad(0));
    femur->setPosition(0, -1.0 * angulos_rad(1));
    tibia->setPosition(0, -1.0 * angulos_rad(2));
    pie_node->getSkeleton()->computeForwardKinematics();
    return pie_node->getTransform().translation();
}

Vector3d calcular_FK_velocidad(Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node, const Vector3d& velocidades_rad_s) {
    coxa->setVelocity(0, velocidades_rad_s(0));
    femur->setVelocity(0, velocidades_rad_s(1));
    tibia->setVelocity(0, velocidades_rad_s(2));
    pie_node->getSkeleton()->computeForwardKinematics();
    return pie_node->getCOMLinearVelocity();
}

Vector3d calcular_F_P(const Vector3d& pos_deseada, const Vector3d& pos_real, const Vector3d& Kp) {
    return Kp.cwiseProduct(pos_deseada - pos_real);
}

Vector3d calcular_F_D(const Vector3d& vel_deseada, const Vector3d& vel_real, const Vector3d& Kd) {
    return Kd.cwiseProduct(vel_deseada - vel_real);
}

Vector3d calcular_F_accel(const Vector3d& accel_deseada, double mp, double mc, double fa, double es) {
    double masa_dinamica = mp + es * ((mc * fa) - mp);
    return accel_deseada * masa_dinamica;
}

Eigen::Matrix3d calcular_jacobiano_lineal(SkeletonPtr pata, BodyNode* pie_node) {
    return pata->getLinearJacobian(pie_node).block<3, 3>(0, 0);
}

Eigen::Matrix3d calcular_jacobiano_fuerza(SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node) {
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
ComandosMotor calcular_comandos_motores(
    SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node,
    const EstadoRealMotores& real, const EstadoDeseadoCartesiano& deseado)
{
    Vector3d pos_real = calcular_FK_posicion(coxa, femur, tibia, pie_node, real.angulos_rad);
    Vector3d vel_real = calcular_FK_velocidad(coxa, femur, tibia, pie_node, real.velocidades_rad_s);

    Vector3d Kp_final = kp_cartesiano * deseado.kp_scale;
    Vector3d Kd_final = kd_cartesiano * deseado.kd_scale;

    Vector3d F_P = calcular_F_P(deseado.posicion, pos_real, Kp_final);
    Vector3d F_D = calcular_F_D(deseado.velocidad, vel_real, Kd_final);
    Vector3d F_accel = calcular_F_accel(deseado.aceleracion, masa_pierna_kg, 8.0, 0.5, deseado.stance_actual);
    
    Vector3d fuerza_total = F_P + F_D + F_accel;

    Eigen::Matrix3d J_lineal = calcular_jacobiano_lineal(pata, pie_node);
    Eigen::Matrix3d J_fuerza = calcular_jacobiano_fuerza(pata, coxa, femur, tibia, pie_node);

    ComandosMotor comandos;
    comandos.velocidades_rad_s = J_lineal.inverse() * deseado.velocidad;
    comandos.torques_Nm = (J_fuerza.inverse() * fuerza_total) * 1e-6;

    return comandos;
}

// --- 5. MAIN ---
int main() {
    LegDimensions dims;
    SkeletonPtr pata = CreateLegSkeleton(dims); 

    auto coxa = pata->getJoint("shoulder_joint");
    auto femur = pata->getJoint("femur_joint");
    auto tibia = pata->getJoint("tibia_joint");
    auto pie = pata->getBodyNode("foot");

    EstadoRealMotores telemetria;
    telemetria.angulos_rad = {0.0, 0.78, -0.78};
    telemetria.velocidades_rad_s = {0.0, 0.0, 0.0};


    LegGaitPlanner planner;
    Vector3d origen_pata(0.0, 0.093, -0.3); // Posición estática nominal de la pata
    Vector3d offset_cadera = dims.shoulder_offset;
    double duty_factor = 0.5; // 50% stance, 50% swing
    double step_duration = 1.0; // 1 segundo por paso
    double step_h = 0.05; // 5 cm de altura máxima al volar

    planner.Initialize(0, origen_pata, offset_cadera, 0.0, duty_factor, step_duration, step_h);

    // 2. Simulamos un instante de tiempo (Ej. 45% del paso)
    // Con duty_factor=0.5, la pata vuela desde 0.0 hasta 0.5. 
    // En global_phase = 0.45, la pata completó el 90% de su vuelo, ¡por lo que está a punto de aterrizar!
    double global_phase = 0.45; 
    
    // Velocidad de avance de prueba (10 cm/s hacia adelante)
    double vx = 0.1, vy = 0.0, wz = 0.0;

    CartesianState ideal = planner.Update(global_phase, vx, vy, wz);

    std::cout << "--- SALIDA DEL GAIT PLANNER (Fase: " << global_phase << ") ---" << std::endl;
    std::cout << "Posicion (P_des): \n" << ideal.P_des.transpose() << std::endl;
    std::cout << "Kp scale: " << ideal.kp_scale << " | Kd scale: " << ideal.kd_scale << std::endl;
    std::cout << "Estado Stance: " << (ideal.is_stance ? "Verdadero" : "Falso (En vuelo)") << "\n\n";

    // 4. Llenamos la estructura del controlador
    EstadoDeseadoCartesiano objetivo;
    objetivo.posicion = ideal.P_des;
    objetivo.velocidad = ideal.V_des;
    objetivo.aceleracion = ideal.A_des;
    objetivo.kp_scale = ideal.kp_scale; 
    objetivo.kd_scale = ideal.kd_scale;
    objetivo.stance_actual = ideal.is_stance ? 1.0 : 0.0; // Conversión booleano a double para DART

    ComandosMotor m_cmd = calcular_comandos_motores(pata, coxa, femur, tibia, pie, telemetria, objetivo);

    std::cout << "\n=== RESULTADOS DEL CONTROLADOR ===" << std::endl;
    std::cout << "Torque FF calculado (Nm): " << m_cmd.torques_Nm.transpose() << std::endl;
    std::cout << "Velocidad deseada (rad/s): " << m_cmd.velocidades_rad_s.transpose() << std::endl;

    return 0;
}