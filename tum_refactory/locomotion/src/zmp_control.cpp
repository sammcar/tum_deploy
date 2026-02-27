#include "zmp_control.hpp"
#include "kinematics.hpp"
#include "robot_config.hpp"
#include <cmath>

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
    double Kd)
{
    // Si no hay hardware real, el ZMP no se puede calcular. Se asume movimiento perfecto.
    if (!require_real_hardware || contact_ptr == nullptr || imu_ptr == nullptr) {
        return Eigen::Vector2d(0.0, 0.0);
    }

    double zmp_numerador_x = 0.0;
    double zmp_numerador_y = 0.0;
    double zmp_denominador = 0.0;
    double zmp_desired_sum_x = 0.0;
    double zmp_desired_sum_y = 0.0;
    int patas_en_suelo = 0;

    Eigen::Vector3d current_foot_pos_global[4];
    double stance_portion = duty_factor;
    double swing_portion = 1.0 - duty_factor;

    // 1. Reconstruir la posición cinemática de las patas
    for (int p = 0; p < 4; p++) {
        double leg_phase = fmod(global_phase + gait_offsets[p], 1.0);
        Eigen::Vector3d origin = LEGS_STAND_XYZ[p];
        bool stance = (leg_phase >= swing_portion);

        TrajectoryPoint3D tp;
        if (stance) {
            double t = (leg_phase - swing_portion) / stance_portion;
            double t_stance = step_duration * stance_portion;
            Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0,0,wz).cross(HIP_OFFSETS[p]);
            Eigen::Vector3d p_start = origin - v_total * (t_stance / 2.0);
            Eigen::Vector3d p_target = origin + v_total * (t_stance / 2.0);
            
            tp.pos = p_target + t * (p_start - p_target);
            tp.pos.z() = origin.z();
        } else {
            double t_stance = step_duration * stance_portion;
            double t_swing = step_duration * swing_portion;
            
            tp = generate_bezier_swing(leg_phase, duty_factor, origin, HIP_OFFSETS[p], vx, vy, wz, 
                                      t_stance, t_swing, step_h);
        }
        
        current_foot_pos_global[p] = HIP_OFFSETS[p] + tp.pos;
        
        // Sumatorias para el ZMP usando la lectura del sensor
        bool c_i = contact_ptr->is_contact[p];
        double Fz_i = contact_ptr->fz_r[p];
        
        if (c_i) {
            zmp_numerador_x += Fz_i * current_foot_pos_global[p].x();
            zmp_numerador_y += Fz_i * current_foot_pos_global[p].y();
            zmp_denominador += Fz_i;

            zmp_desired_sum_x += current_foot_pos_global[p].x();
            zmp_desired_sum_y += current_foot_pos_global[p].y();
            patas_en_suelo++;
        }
    }

    // 2. Calcular ZMP Real (pxr, pyr) y ZMP Deseado (pxd, pyd)
    if (zmp_denominador < 1e-5) zmp_denominador = 1e-5; // Evitar división por cero
    
    double pxr = zmp_numerador_x / zmp_denominador;
    double pyr = zmp_numerador_y / zmp_denominador;
    double pxd = 0.0;
    double pyd = 0.0;

    if (patas_en_suelo > 0) {
        pxd = zmp_desired_sum_x / patas_en_suelo;
        pyd = zmp_desired_sum_y / patas_en_suelo;
    }

    // 3. Obtener velocidades del CoM desde el giroscopio
    double gyro_x_rad = imu_ptr->gyro[0] * M_PI / 180.0;
    double gyro_y_rad = imu_ptr->gyro[1] * M_PI / 180.0;
    double v_com_x = h_com * gyro_y_rad; 
    double v_com_y = -h_com * gyro_x_rad;

    // 4. Ley de Control PD
    double u_x = Kp * (pxd - pxr) - Kd * v_com_x;
    double u_y = Kp * (pyd - pyr) - Kd * v_com_y;

    return Eigen::Vector2d(u_x, u_y);
}