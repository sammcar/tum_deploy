#ifndef SETUP_HPP
#define SETUP_HPP

#include <Eigen/Dense>
#include "robot_types.hpp"

// ============================================================
// FUNCIÓN DE CONFIGURACIÓN (FASES A y B)
// ============================================================
bool setup_robot(
    double& duracion_test, // <--- AHORA ES DOUBLE
    double& step_duration,
    double& vx,
    double& vy,
    double& wz,
    double& step_h,
    IMUData*& imu_ptr,
    CommandData*& shm_ptr,
    TelemetryData*& tel_ptr,
    ContactData*& contact_ptr,
    Eigen::Vector3d& target_body_pos,
    Eigen::Vector3d& target_body_rpy,
    double (&gait_offsets)[4],
    Eigen::Vector3d (&start_foot_positions)[4],
    bool require_real_hardware // <--- NO OLVIDAR ESTE
);

#endif // SETUP_HPP
