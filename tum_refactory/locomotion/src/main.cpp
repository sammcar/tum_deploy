#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <csignal>

// Inclusión de nuestros nuevos módulos
#include "robot_types.hpp"
#include "robot_config.hpp"
#include "kinematics.hpp"
#include "utils.hpp"
#include "setup.hpp"

// Definición de las variables globales declaradas en utils.hpp
std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
    std::signal(SIGINT, SignalHandler);

    // 1. Declaramos las variables que necesita el bucle dinámico
    int max_ciclos;
    double step_duration, step_len, step_h;
    IMUData* imu_ptr = nullptr;
    CommandData* shm_ptr = nullptr;
    TelemetryData* tel_ptr = nullptr;
    ContactData* contact_ptr = nullptr;
    Eigen::Vector3d target_body_pos, target_body_rpy;
    double gait_offsets[4] = {0.0, 0.5, 0.5, 0.0};
    Eigen::Vector3d start_foot_positions[4];

    // 2. Llamamos al setup pasándole las referencias (Fases A y B)
    if (!setup_robot(max_ciclos, step_duration, step_len, step_h, 
                     imu_ptr, shm_ptr, tel_ptr, contact_ptr, 
                     target_body_pos, target_body_rpy, 
                     gait_offsets, start_foot_positions)) {
        std::cerr << "[FATAL] Error en la inicialización del robot." << std::endl;
        return 1; 
    }

    // --- PUNTO DE CONTROL DE MARCHA ---
    std::cout << "\n>>> PRESIONA ENTER PARA INICIAR MARCHA <<<" << std::endl;
    std::cin.get(); // Espera al usuario
    std::cout << ">> Iniciando marcha..." << std::endl;

    std::cout << "\nTorque Femur FL: " << tel_ptr->measured_torques[0][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia FL: " << tel_ptr->measured_torques[0][2] << " Nm      " << std::endl;
    std::cout << "\nTorque Femur FR: " << tel_ptr->measured_torques[1][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia FR: " << tel_ptr->measured_torques[1][2] << " Nm      " << std::endl;
    std::cout << "\nTorque Femur BL: " << tel_ptr->measured_torques[2][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia BL: " << tel_ptr->measured_torques[2][2] << " Nm      " << std::endl;
    std::cout << "\nTorque Femur BR: " << tel_ptr->measured_torques[3][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia BR: " << tel_ptr->measured_torques[3][2] << " Nm      " << std::endl;

    // ============================================================
    // FASE C: BUCLE DINÁMICO (Trote + Postura Mantenida)
    // ============================================================
    auto start_time = std::chrono::steady_clock::now();
    double loop_dt = 0.004;
    double prev_angles[4][3] = {0};

    std::vector<LogData> historial;
    historial.reserve(20000);
    
    // Inicializamos prev_angles con lo que tenemos en SHM (la postura ya aplicada)
    for(int p=0; p<4; ++p) {
        prev_angles[p][0] = shm_ptr->angles[p][0];
        prev_angles[p][1] = shm_ptr->angles[p][1];
        prev_angles[p][2] = shm_ptr->angles[p][2];
    }
    bool first_run = true;
    bool leg_finished[4] = {false, false, false, false};
    Eigen::Vector3d final_foot_pos_3d[4];

    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();

        int c[4] = {0, 0, 0, 0};            
        double fz_medido[4] = {0.0, 0.0, 0.0, 0.0}; 
        
        bool tiempo_agotado = (elapsed > (max_ciclos * step_duration));
        bool todo_aterrizado = true; 
        double global_phase = fmod(elapsed / step_duration, 1.0);

        double zmp_numerador_x = 0.0;
        double zmp_numerador_y = 0.0;
        double zmp_denominador = 0.0;
        double zmp_desired_sum_x = 0.0;
        double zmp_desired_sum_y = 0.0;
        int patas_en_suelo = 0;

        Eigen::Vector3d current_foot_pos_global[4];

        for (int p = 0; p < 4; p++) {
            double leg_phase = fmod(global_phase + gait_offsets[p], 1.0);
            Eigen::Vector3d origin = LEGS_STAND_XYZ[p];
            TrajectoryPoint pt = get_step_trajectory(leg_phase, step_duration, origin.x(), origin.z(), step_len, step_h);
            
            current_foot_pos_global[p] = HIP_OFFSETS[p] + Eigen::Vector3d(pt.x, origin.y(), pt.z);
            
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

        double pxr = 0.0;
        double pyr = 0.0;

        if (zmp_denominador > 1e-5) {zmp_denominador += 1e-5;}

        pxr = zmp_numerador_x / zmp_denominador;
        pyr = zmp_numerador_y / zmp_denominador;

        double pxd = 0.0;
        double pyd = 0.0;

        if (patas_en_suelo > 0) {
            pxd = zmp_desired_sum_x / patas_en_suelo;
            pyd = zmp_desired_sum_y / patas_en_suelo;
        }

        double h_com = std::abs(-0.296 + target_body_pos.z());
        double gyro_x_rad = imu_ptr->gyro[0] * M_PI / 180.0;
        double gyro_y_rad = imu_ptr->gyro[1] * M_PI / 180.0;

        double v_com_x = h_com * gyro_y_rad; 
        double v_com_y = -h_com * gyro_x_rad;

        for (int p = 0; p < 4; p++) {
            bool is_right = (p == 1 || p == 3);
            
            if (leg_finished[p]) {
                shm_ptr->desired_accel[p][0] = 0; shm_ptr->desired_accel[p][1] = 0; shm_ptr->desired_accel[p][2] = 0;
                shm_ptr->is_stance[p] = true;
                
                Eigen::Vector3d final_ik_input = ComputeWholeBodyIK(final_foot_pos_3d[p], p, target_body_pos, target_body_rpy);
                LegAngles ik = solve_IK(final_ik_input.x(), final_ik_input.y(), final_ik_input.z(), is_right);
                
                if (ik.valid) {
                    shm_ptr->angles[p][0] = ik.th1;
                    shm_ptr->angles[p][1] = ik.th2;
                    shm_ptr->angles[p][2] = ik.th3;
                    shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
                }
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                
                continue; 
            }

            double leg_phase = fmod(global_phase + gait_offsets[p], 1.0);
            bool stance = (leg_phase >= 0.5);

            if (!tiempo_agotado) {
                historial.push_back({
                    p, leg_phase, stance, tel_ptr->measured_torques[p][1], tel_ptr->measured_torques[p][2],
                    pxr, pyr, pxd, pyd, v_com_x, v_com_y
                });
            }

            Eigen::Vector3d origin = LEGS_STAND_XYZ[p];
            TrajectoryPoint point = get_step_trajectory(leg_phase, step_duration, origin.x(), origin.z(), step_len, step_h);

            if (tiempo_agotado) {
                if (stance) {
                    final_foot_pos_3d[p] = Eigen::Vector3d(point.x, origin.y(), point.z);
                    leg_finished[p] = true; 
                    shm_ptr->is_stance[p] = true;
                    continue; 
                } else {
                    todo_aterrizado = false;
                }
            }

            Eigen::Vector3d foot_local_flat(point.x, origin.y(), point.z);
            Eigen::Vector3d ik_input = ComputeWholeBodyIK(foot_local_flat, p, target_body_pos, target_body_rpy);
            LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), is_right);

            if (ik.valid) {
                shm_ptr->angles[p][0] = ik.th1;
                shm_ptr->angles[p][1] = ik.th2;
                shm_ptr->angles[p][2] = ik.th3;

                if (!first_run) {
                    shm_ptr->velocities[p][0] = (ik.th1 - prev_angles[p][0]) / loop_dt;
                    shm_ptr->velocities[p][1] = (ik.th2 - prev_angles[p][1]) / loop_dt;
                    shm_ptr->velocities[p][2] = (ik.th3 - prev_angles[p][2]) / loop_dt;
                }
                prev_angles[p][0] = ik.th1; prev_angles[p][1] = ik.th2; prev_angles[p][2] = ik.th3;

                shm_ptr->desired_accel[p][0] = point.ax;
                shm_ptr->desired_accel[p][1] = 0.0;
                shm_ptr->desired_accel[p][2] = point.az;
                shm_ptr->is_stance[p] = stance;

                double kp_val = stance ? 1.0 : 0.6;
                double kd_val = stance ? 1.0 : 0.4;
                for(int m=0; m<3; ++m) {
                    shm_ptr->kp_scale[p][m] = kp_val;
                    shm_ptr->kd_scale[p][m] = kd_val;
                }
            }
        }
        
        if (tiempo_agotado && todo_aterrizado) {
            break;
        }
        first_run = false;
        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    // ============================================================
    // FASE D: HOLD FINAL (Post-Caminata con Postura)
    // ============================================================
    std::cout << "\n>> Ciclo terminado. Todas las patas aterrizadas." << std::endl;
    std::cout << ">> MANTENIENDO POSTURA FINAL." << std::endl;
    std::cout << ">>> PRESIONA ENTER PARA FINALIZAR Y APAGAR <<<" << std::endl;
    
    std::cin.get(); 

    for(int p=0; p<4; ++p) {
        for(int m=0; m<3; ++m) {
            shm_ptr->angles[p][m] = 0.0;
            shm_ptr->velocities[p][m] = 2.0; 
            shm_ptr->kp_scale[p][m] = 1.0;
            shm_ptr->kd_scale[p][m] = 1.0;
            shm_ptr->desired_accel[p][m] = 0.0;
        }
    }
    shm_ptr->transition_time = 3.0; 
    shm_ptr->is_walking = false;

    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    std::memset(shm_ptr, 0, sizeof(CommandData));
    shm_ptr->is_walking = false; 
    
    std::cout << "\n✅ Finalizado. Robot en posición segura." << std::endl;

    std::cout << ">> Guardando datos de torques, ZMP y Velocidad CoM en 'torques_log.csv'..." << std::endl;
    std::ofstream log_file("torques_log.csv");
    if (log_file.is_open()) {
        log_file << "Leg_ID,Phase,Is_Stance,Torque_Femur,Torque_Tibia,ZMP_R_X,ZMP_R_Y,ZMP_D_X,ZMP_D_Y,V_COM_X,V_COM_Y\n";
        for (const auto& dato : historial) {
            log_file << dato.leg_id << "," << dato.phase << "," << dato.is_stance << "," 
                     << dato.torque_femur << "," << dato.torque_tibia << ","
                     << dato.zmp_x << "," << dato.zmp_y << "," << dato.zmp_d_x << "," << dato.zmp_d_y << ","
                     << dato.v_com_x << "," << dato.v_com_y << "\n";
        }
        log_file.close();
        std::cout << "✅ Archivo 'torques_log.csv' guardado correctamente." << std::endl;
    } else {
        std::cerr << "❌ Error: No se pudo crear el archivo CSV." << std::endl;
    }
    
    return 0;
}