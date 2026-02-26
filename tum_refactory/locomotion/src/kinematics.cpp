#include "kinematics.hpp"
#include "robot_config.hpp" // Para L1, L2, L3, HIP_OFFSETS
#include <cmath>
#include <algorithm>

// Agregamos estas líneas para que el compilador reconozca los tipos de Eigen
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;

// ============================================================
// AUXILIAR: Matriz de Transformación DH
// ============================================================
inline Matrix4d get_dh_matrix(double theta, double d, double a, double alpha) {
    Matrix4d T;
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0,   sa,       ca,      d,
         0,   0,        0,       1;
    return T;
}

// ============================================================
// CINEMÁTICA DIRECTA (FK) - SOLO POSICIÓN DEL PIE
// ============================================================
/**
 * Calcula la posición (x, y, z) del pie respecto a la cadera.
 * @param theta1, theta2, theta3 Ángulos en RADIANES.
 */
Vector3d solve_FK(double theta1, double theta2, double theta3, bool es_pata_derecha) {
    double l1_side = es_pata_derecha ? -L1 : L1;

    // Encadenamos las transformaciones DH según tu modelo
    Matrix4d T01 = get_dh_matrix(M_PI/2.0, 0, 0, M_PI/2.0);
    Matrix4d T12 = get_dh_matrix(theta1, 0, l1_side, M_PI/2.0);
    Matrix4d T23 = get_dh_matrix(M_PI/2.0, 0, 0, -M_PI/2.0);
    Matrix4d T34 = get_dh_matrix(theta2, 0, L2, 0);
    Matrix4d T45 = get_dh_matrix(theta3, 0, L3, 0);

    // Multiplicamos todas las matrices para obtener la transformación final del pie
    Matrix4d m_foot = T01 * T12 * T23 * T34 * T45;

    // Retornamos solo la columna de traslación (los componentes X, Y, Z)
    return m_foot.block<3,1>(0,3);
}

// ============================================================
// 3. MATEMÁTICAS (IK + BÉZIER) - Implementaciones
// ============================================================

Vector3d ComputeWholeBodyIK(Vector3d foot_pos_local_flat, int leg_idx, 
                            Vector3d body_pos, Vector3d body_rpy) {
    
    Matrix3d R_body;
    R_body = AngleAxisd(body_rpy.z(), Vector3d::UnitZ()) *
             AngleAxisd(body_rpy.y(), Vector3d::UnitY()) *
             AngleAxisd(body_rpy.x(), Vector3d::UnitX());

    Vector3d hip_offset_static = HIP_OFFSETS[leg_idx];
    Vector3d foot_pos_global = hip_offset_static + foot_pos_local_flat;
    Vector3d hip_pos_new = body_pos + (R_body * hip_offset_static);
    Vector3d vec_world = foot_pos_global - hip_pos_new;
    Vector3d vec_local_ik = R_body.transpose() * vec_world;

    return vec_local_ik;
}

double wrap_to_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha) {
    double y_local = es_pata_derecha ? -y : y;
    double dist_yz = sqrt(pow(y_local, 2) + pow(z, 2));
    double D = std::max(dist_yz, L1 + 1e-6);
    
    // Clamp para evitar acos fuera de dominio [-1, 1]
    double val_c1 = std::clamp(L1 / D, -1.0, 1.0);
    double theta1 = atan2(z, y_local) + acos(val_c1);

    double R_term = pow(dist_yz, 2) - pow(L1, 2);
    double R = -sqrt(std::max(0.0, R_term)); 
    double H = sqrt(pow(x, 2) + pow(R, 2));
    
    double cos_phi1 = std::clamp((pow(L2, 2) + pow(L3, 2) - pow(H, 2)) / (2 * L2 * L3), -1.0, 1.0);
    double phi1 = acos(cos_phi1);
    
    double theta3 = M_PI - phi1; 
    double phi2 = atan2(R, x);
    
    double sin_phi3 = std::clamp((L3 * sin(phi1)) / (H + 1e-9), -1.0, 1.0);
    double phi3 = asin(sin_phi3);
    
    double theta2 = phi2 - phi3;

    return {
        wrap_to_pi(theta1) * 180.0 / M_PI,
        wrap_to_pi(theta2 + M_PI/2.0) * 180.0 / M_PI,
        wrap_to_pi(theta3) * 180.0 / M_PI,
        true
    };
}

double lerp(double a, double b, double t) { return a + t * (b - a); }

TrajectoryPoint calculate_bezier(double t, double duration, 
                                 double x0, double z0,
                                 double x1, double z1,
                                 double x2, double z2,
                                 double x3, double z3) 
{
    // Asegurar que t esté entre 0 y 1
    t = std::clamp(t, 0.0, 1.0);
    
    double t2 = t * t; 
    double t3 = t2 * t;
    double u = 1.0 - t; 
    double u2 = u * u; 
    double u3 = u2 * u;

    TrajectoryPoint res;
    res.x = u3*x0 + 3*u2*t*x1 + 3*u*t2*x2 + t3*x3;
    res.z = u3*z0 + 3*u2*t*z1 + 3*u*t2*z2 + t3*z3;

    double d2xdt2 = 6*u*(x2 - 2*x1 + x0) + 6*t*(x3 - 2*x2 + x1);
    double d2zdt2 = 6*u*(z2 - 2*z1 + z0) + 6*t*(z3 - 2*z2 + z1);

    if (duration > 1e-5) {
        res.ax = d2xdt2 / (duration * duration);
        res.az = d2zdt2 / (duration * duration);
    } else { 
        res.ax = 0; 
        res.az = 0; 
    }

    return res;
}

TrajectoryPoint get_step_trajectory(double phase, double step_period, 
                                    double origin_x, double origin_z, 
                                    double step_len, double step_height) {
    
    phase = phase - std::floor(phase);
    double x_start = origin_x - step_len / 2.0;
    double x_end   = origin_x + step_len / 2.0;
    double z_ground = origin_z;

    if (phase >= 0.5) {
        double t = (phase - 0.5) * 2.0;
        TrajectoryPoint p;
        p.x = lerp(x_end, x_start, t); 
        p.z = z_ground;
        p.ax = 0; 
        p.az = 0;
        return p;
    }

    double t = phase * 2.0;
    double lift_bias = 0.2; 
    
    return calculate_bezier(t, step_period * 0.5,
        x_start, z_ground,
        x_start + (step_len * lift_bias), z_ground + (step_height * 1.3),
        x_end - (step_len * lift_bias), z_ground + (step_height * 1.3),
        x_end, z_ground
    );
}

TrajectoryPoint3D generate_bezier_swing(double phase, const Eigen::Vector3d& p_neutral,
                                     const Eigen::Vector3d& hip_offset, double vx, double vy,
                                     double omega_z, double t_stance, double t_swing,
                                     double step_height) {
    
    Eigen::Vector3d v_linear(vx, vy, 0.0);
    Eigen::Vector3d omega_vec(0, 0, omega_z);
    Eigen::Vector3d v_rotation = omega_vec.cross(hip_offset);

    Eigen::Vector3d p_target = p_neutral + (v_linear + v_rotation) * (t_stance / 2.0);
    Eigen::Vector3d p_start = p_neutral - (v_linear + v_rotation) * (t_stance / 2.0);

    double t = std::clamp(phase * 2.0, 0.0, 1.0); 
    
    Eigen::Vector3d p0 = p_start;
    Eigen::Vector3d p1 = p_start; p1.z() += step_height * 1.5;
    Eigen::Vector3d p2 = p_target; p2.z() += step_height * 1.5;
    Eigen::Vector3d p3 = p_target;

    double u = 1.0 - t;
    TrajectoryPoint3D res;
    res.pos = (u*u*u)*p0 + 3*(u*u)*t*p1 + 3*u*(t*t)*p2 + (t*t*t)*p3;
    
    if (t_swing > 1e-5) {
        res.acc = (6 * u * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1)) / (t_swing * t_swing);
    } else {
        res.acc = Eigen::Vector3d::Zero();
    }
    
    return res;
}
