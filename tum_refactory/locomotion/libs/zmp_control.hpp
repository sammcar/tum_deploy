#ifndef ZMP_CONTROL_HPP
#define ZMP_CONTROL_HPP

#include "robot_types.hpp"
#include <Eigen/Dense>

/**
 * @brief Calcula el desplazamiento (offset) del Centro de Masa (CoM) para corregir el ZMP.
 * * @param require_real_hardware Booleano para saber si hay hardware (si es false, retorna 0,0).
 * @param contact_ptr Puntero a la memoria compartida de contactos.
 * @param imu_ptr Puntero a la memoria compartida de la IMU.
 * @param global_phase Fase global actual de la marcha (0.0 a 1.0).
 * @param gait_offsets Arreglo con los desfases de cada pata.
 * @param duty_factor Proporción de la marcha donde la pata está en stance.
 * @param step_duration Duración de un ciclo completo en segundos.
 * @param vx, vy, wz, step_h Velocidades y altura del paso.
 * @param h_com Altura dinámica actual del Centro de Masa.
 * @param Kp Ganancia Proporcional (Fuerza de corrección de posición).
 * @param Kd Ganancia Derivativa (Amortiguamiento por velocidad del cuerpo).
 * @return Eigen::Vector2d Vector con las correcciones [u_x, u_y] para el chasis.
 */
Eigen::Vector2d compute_zmp_offset(
    bool require_real_hardware,
    ContactData* contact_ptr,
    IMUData* imu_ptr,
    double global_phase,
    const double (&gait_offsets)[4],
    double duty_factor,
    double step_duration,
    double vx,
    double vy,
    double wz,
    double step_h,
    double h_com,
    double Kp,
    double Kd
);

#endif // ZMP_CONTROL_HPP