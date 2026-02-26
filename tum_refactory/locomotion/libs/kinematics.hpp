#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>
#include "robot_types.hpp" // Asegúrate de que aquí estén LegAngles, TrajectoryPoint y TrajectoryPoint3D

using Eigen::Vector3d;

// ============================================================
// 3. MATEMÁTICAS (IK + FK + BÉZIER) - Declaraciones
// ============================================================

// --- CINEMÁTICA DIRECTA (FK) ---
/**
 * Calcula la posición (x, y, z) del pie respecto a la cadera.
 * @param theta1, theta2, theta3 Ángulos en RADIANES.
 */
Vector3d solve_FK(double theta1, double theta2, double theta3, bool es_pata_derecha);


// --- CINEMÁTICA INVERSA (IK) ---
/**
 * Transforma coordenadas de cuerpo a coordenadas locales de la pata considerando RPY.
 */
Vector3d ComputeWholeBodyIK(Vector3d foot_pos_local_flat, int leg_idx, 
                            Vector3d body_pos, Vector3d body_rpy);

/**
 * Resuelve los ángulos de los motores para una posición (x, y, z) dada.
 * Retorna los ángulos en GRADOS.
 */
LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha);


// --- UTILIDADES ---
double wrap_to_pi(double angle);
double lerp(double a, double b, double t);


// --- TRAYECTORIAS (BÉZIER) ---

/**
 * Curva de Bézier Cúbica estándar en 2D (X, Z).
 */
TrajectoryPoint calculate_bezier(double t, double duration, 
                                 double x0, double z0,
                                 double x1, double z1,
                                 double x2, double z2,
                                 double x3, double z3);

/**
 * Generador de trayectoria 2D para el ciclo de marcha.
 */
TrajectoryPoint get_step_trajectory(double phase, double step_period, 
                                    double origin_x, double origin_z, 
                                    double step_len, double step_height);

/**
 * Generador de trayectoria 3D avanzado para swing omnidireccional.
 */
TrajectoryPoint3D generate_bezier_swing(double phase, const Vector3d& p_neutral,
                                     const Vector3d& hip_offset, double vx, double vy,
                                     double omega_z, double t_stance, double t_swing,
                                     double step_height);

#endif // KINEMATICS_HPP
