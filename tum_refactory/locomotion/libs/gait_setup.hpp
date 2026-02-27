#ifndef GAIT_SETUP_HPP
#define GAIT_SETUP_HPP

#include "robot_types.hpp"
#include <Eigen/Dense>

/**
 * @brief Configura la postura inicial y los parámetros de marcha del robot.
 * * @param dont_ask Si es true, usa valores por defecto y no pausa esperando al usuario.
 * @param duracion_test Referencia para guardar la duración total.
 * @param step_duration Referencia para guardar el periodo del paso.
 * @param vx Referencia para guardar velocidad X.
 * @param vy Referencia para guardar velocidad Y.
 * @param wz Referencia para guardar velocidad rotacional Z.
 * @param step_h Referencia para guardar altura del paso.
 * @param shm_ptr Puntero a la memoria de comandos de los motores.
 * @param target_body_pos Referencia para la posición final del cuerpo.
 * @param target_body_rpy Referencia para la rotación final del cuerpo.
 * @param duty_factor Proporción del ciclo en la que la pata toca el suelo.
 * @param gait_offsets Arreglo de desfases para cada pata.
 * @param start_foot_positions Arreglo donde se guardarán las posiciones iniciales calculadas.
 * @return true si la configuración fue exitosa.
 */
bool init_gait_posture(
    bool dont_ask,
    double& duracion_test,
    double& step_duration,
    double& vx,
    double& vy,
    double& wz,
    double& step_h,
    CommandData* shm_ptr,
    Eigen::Vector3d& target_body_pos,
    Eigen::Vector3d& target_body_rpy,
    double duty_factor,
    const double (&gait_offsets)[4],
    Eigen::Vector3d (&start_foot_positions)[4]
);

#endif // GAIT_SETUP_HPP