#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <Eigen/Dense>
#include "robot_types.hpp" // Para LegAngles y TrajectoryPoint

using Eigen::Vector3d;

// ============================================================
// 3. MATEMÁTICAS (IK + BÉZIER) - Declaraciones
// ============================================================

Vector3d ComputeWholeBodyIK(Vector3d foot_pos_local_flat, int leg_idx, 
                            Vector3d body_pos, Vector3d body_rpy);

// Utilidad para envolver ángulos (-PI a PI)
double wrap_to_pi(double angle);

// IK proporcionada por ti
LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha);

double lerp(double a, double b, double t);

// Curva de Bézier Cúbica para Swing
TrajectoryPoint calculate_bezier(double t, double duration, 
                                 double x0, double z0,
                                 double x1, double z1,
                                 double x2, double z2,
                                 double x3, double z3);

// Generador de Trayectoria relativo al centro de la pata (origin_x, origin_z)
TrajectoryPoint get_step_trajectory(double phase, double step_period, 
                                    double origin_x, double origin_z, 
                                    double step_len, double step_height);

TrajectoryPoint3D generate_bezier_swing(double phase, double duty_factor, const Eigen::Vector3d& p_neutral,
                                     const Eigen::Vector3d& hip_offset, double vx, double vy,
                                     double omega_z, double t_stance, double t_swing,
                                     double step_height);

#endif // KINEMATICS_HPP