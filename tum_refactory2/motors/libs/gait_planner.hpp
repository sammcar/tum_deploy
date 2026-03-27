#pragma once

#include <Eigen/Dense>
#include "utils_motors.hpp" // Para TrajectoryOutput

struct TrajectoryPoint3D {
    Eigen::Vector3d pos;
    Eigen::Vector3d acc;
};

// Declaración de la función de Bézier/Cicloide
TrajectoryPoint3D generate_bezier_swing(double phase, double duty_factor, const Eigen::Vector3d &p_neutral,
                                        const Eigen::Vector3d &hip_offset, double vx, double vy,
                                        double omega_z, double t_stance, double t_swing,
                                        double step_height);

// Declaración de la clase Planner
class Atom51GaitPlanner {
public:
    Atom51GaitPlanner();

    TrajectoryOutput update(double dt, double elapsed_time, int leg_id,
                            double vx, double vy, double wz,
                            double step_duration, double step_h,
                            double duty_factor, const double gait_offsets[4]);

private:
    Eigen::Vector3d last_pos_[4];
};
