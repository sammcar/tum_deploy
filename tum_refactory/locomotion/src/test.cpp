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
#include <cmath>

// Inclusión de módulos del proyecto ATOM-51
#include "robot_types.hpp"
#include "robot_config.hpp"
#include "kinematics.hpp"
#include "memory_setup.hpp"
#include "gait_setup.hpp"

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
    std::signal(SIGINT, SignalHandler);

    // Las variables ahora se declaran aquí, pero se llenan dentro de setup_robot
    double duracion_test, step_duration, vx, vy, wz, step_h;
    bool require_real_hardware = false; // Cambia a false para simular sin hardware real
    bool dont_ask = false; // Cambia a true para usar valores fijos sin preguntar en consola

    IMUData* imu_ptr = nullptr;
    CommandData* shm_ptr = nullptr;
    TelemetryData* tel_ptr = nullptr;
    ContactData* contact_ptr = nullptr;
    Eigen::Vector3d target_body_pos, target_body_rpy;
    double duty_factor = 0.75; // Proporción de tiempo en stance (0.5 = 50% stance, 50% swing)
    // double gait_offsets[4] = {0.0, 0.5, 0.5, 0.0}; // Trot
    // double gait_offsets[4] = {0.0, 0.5, 0.0, 0.0}; // Pace
    // double gait_offsets[4] = {0.0, 0.0, 0.5, 0.5}; // Bound
    // double gait_offsets[4] = {0.0, 0.5, 0.75, 0.25}; // Amble
    double gait_offsets[4] = {0.0, 0.5, 0.75, 0.25}; // Crawl, cambiar duty_factor por 0.75
    Eigen::Vector3d start_foot_positions[4];

    // ============================================================
    // FASE 1: INICIALIZACIÓN DE MEMORIA Y SENSORES
    // ============================================================
    if (!init_shared_memory(require_real_hardware, imu_ptr, shm_ptr, tel_ptr, contact_ptr)) {
        std::cerr << "[FATAL] Fallo al inicializar la memoria. Saliendo..." << std::endl;
        return 1;
    }

    // ============================================================
    // FASE 2: CONFIGURACIÓN DE POSTURA Y MARCHA
    // ============================================================
    if (!init_gait_posture(dont_ask, duracion_test, step_duration, vx, vy, wz, step_h, 
                           shm_ptr, target_body_pos, target_body_rpy, 
                           duty_factor, gait_offsets, start_foot_positions)) {
        std::cerr << "[FATAL] Error en la configuración de la marcha." << std::endl;
        return 1; 
    }

    std::cout << "\n>>> PRESIONA ENTER PARA INICIAR MARCHA <<<" << std::endl;
    std::cin.get();

    auto start_time = std::chrono::steady_clock::now();
    const double loop_dt = 0.004; 
    double prev_angles[4][3] = {0};
    
    // Inicializamos ángulos previos con la postura actual para la derivada de velocidad
    for(int p=0; p<4; ++p) {
        for(int m=0; m<3; ++m) prev_angles[p][m] = shm_ptr->angles[p][m];
    }

    bool first_run = true;
    bool leg_finished[4] = {false, false, false, false};
    Eigen::Vector3d final_foot_pos_3d[4];

    // ============================================================
    // FASE C: BUCLE DINÁMICO DE MARCHA
    // ============================================================
    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        
        bool tiempo_agotado = (elapsed > duracion_test);
        bool todo_aterrizado = true; // Empieza en true, si una pata está en el aire se vuelve false
        double global_phase = fmod(elapsed / step_duration, 1.0);

        // ZMP 
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
            
            // Calculamos proporciones de tiempo reales para el ZMP
            double stance_portion = duty_factor;
            double swing_portion = 1.0 - duty_factor;
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
            
            // Actualizamos la posición global usando tp.pos que es un Vector3d
            current_foot_pos_global[p] = HIP_OFFSETS[p] + tp.pos;
            
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

        // Calcular la altura base del CoM como el promedio de la componente Z de las 4 patas
        double z_sum = 0.0;
        for (int i = 0; i < 4; ++i) {
            z_sum += LEGS_STAND_XYZ[i].z();
        }
        double avg_stand_z = z_sum / 4.0;
        
        // Calcular h_com dinámico
        double h_com = std::abs(avg_stand_z + target_body_pos.z());
        double gyro_x_rad = imu_ptr->gyro[0] * M_PI / 180.0;
        double gyro_y_rad = imu_ptr->gyro[1] * M_PI / 180.0;

        double v_com_x = h_com * gyro_y_rad; 
        double v_com_y = -h_com * gyro_x_rad;

        // =========================================================
        // ESQUELETO DE LA LEY DE CONTROL (PARA FUTURA IMPLEMENTACIÓN)
        // =========================================================
        double Kp_zmp = 0.1; // Ejemplo
        double Kd_zmp = 0.05; // Ejemplo

        double error_zmp_x = pxd - pxr;
        double error_zmp_y = pyd - pyr;

        // Ley PD: Posición deseada + Kp(Error Posición) + Kd(Error Velocidad)
        double u_x = Kp_zmp *(pxd - pxr) - Kd_zmp * v_com_x;
        double u_y = Kp_zmp *(pyd - pyr) - Kd_zmp * v_com_y;

        //target_body_pos.x() = original_bx + u_x;
        //target_body_pos.y() = original_by + u_y;
        

        for (int p = 0; p < 4; p++) {
            
            // --- ATERRIZAJE SUAVE: Si la pata ya terminó, congelarla ---
            if (leg_finished[p]) {
                shm_ptr->desired_accel[p][0] = 0; shm_ptr->desired_accel[p][1] = 0; shm_ptr->desired_accel[p][2] = 0;
                shm_ptr->is_stance[p] = true;
                
                Eigen::Vector3d final_ik_input = ComputeWholeBodyIK(final_foot_pos_3d[p], p, target_body_pos, target_body_rpy);
                LegAngles ik = solve_IK(final_ik_input.x(), final_ik_input.y(), final_ik_input.z(), (p==1 || p==3));
                
                if (ik.valid) {
                    shm_ptr->angles[p][0] = ik.th1;
                    shm_ptr->angles[p][1] = ik.th2;
                    shm_ptr->angles[p][2] = ik.th3;
                    shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
                }
                
                // Mantiene rigidez alta en la postura estática final
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                
                continue; 
            }

            // --- TRAYECTORIA NORMAL ---
            double leg_phase = fmod(global_phase + gait_offsets[p], 1.0);
            
            // Calculamos proporciones de tiempo reales
            double stance_portion = duty_factor;
            double swing_portion = 1.0 - duty_factor;
            
            // Si la fase superó la porción de vuelo, la pata ya está en el suelo
            bool stance = (leg_phase >= swing_portion); 
            
            Eigen::Vector3d origin = LEGS_STAND_XYZ[p];
            TrajectoryPoint3D tp; 

            if (stance) {
                // Normalizar 't' (0.0 a 1.0) para la fase de stance
                double t = (leg_phase - swing_portion) / stance_portion;
                
                // Las velocidades y arrastres dependen de cuánto tiempo dura el stance
                double t_stance = step_duration * stance_portion;
                Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0,0,wz).cross(HIP_OFFSETS[p]);
                
                // p_start y p_target se separan según la distancia recorrida en ese t_stance
                Eigen::Vector3d p_start = origin - v_total * (t_stance / 2.0);
                Eigen::Vector3d p_target = origin + v_total * (t_stance / 2.0);
                
                tp.pos = p_target + t * (p_start - p_target);
                tp.pos.z() = origin.z();
                tp.acc = Eigen::Vector3d::Zero();
            } else {
                // Generador de Bézier con los tiempos y factor correctos
                double t_stance = step_duration * stance_portion;
                double t_swing = step_duration * swing_portion;
                
                tp = generate_bezier_swing(leg_phase, duty_factor, origin, HIP_OFFSETS[p], vx, vy, wz, 
                                          t_stance, t_swing, step_h);
            }

            // --- GATILLO DE PARADA POR TIEMPO ---
            if (tiempo_agotado) {
                if (stance) {
                    final_foot_pos_3d[p] = tp.pos; // Se guarda la posición donde tocó el suelo
                    leg_finished[p] = true; 
                    shm_ptr->is_stance[p] = true;
                    continue; 
                } else {
                    todo_aterrizado = false; // Hay una pata en el aire, no podemos romper el while
                }
            }

            Eigen::Vector3d ik_input = ComputeWholeBodyIK(tp.pos, p, target_body_pos, target_body_rpy);
            LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), (p==1 || p==3));

            if (ik.valid) {
                shm_ptr->angles[p][0] = ik.th1;
                shm_ptr->angles[p][1] = ik.th2;
                shm_ptr->angles[p][2] = ik.th3;

                // Cálculo de velocidad con protección de pico inicial
                if (!first_run) {
                    shm_ptr->velocities[p][0] = (ik.th1 - prev_angles[p][0]) / loop_dt;
                    shm_ptr->velocities[p][1] = (ik.th2 - prev_angles[p][1]) / loop_dt;
                    shm_ptr->velocities[p][2] = (ik.th3 - prev_angles[p][2]) / loop_dt;
                } else {
                    shm_ptr->velocities[p][0] = 0.0;
                    shm_ptr->velocities[p][1] = 0.0;
                    shm_ptr->velocities[p][2] = 0.0;
                }
                prev_angles[p][0] = ik.th1; prev_angles[p][1] = ik.th2; prev_angles[p][2] = ik.th3;

                shm_ptr->desired_accel[p][0] = tp.acc.x();
                shm_ptr->desired_accel[p][2] = tp.acc.z();
                shm_ptr->is_stance[p] = stance;

                double k_scale = stance ? 1.0 : 0.6;
                for(int m=0; m<3; m++) {
                    shm_ptr->kp_scale[p][m] = k_scale;
                    shm_ptr->kd_scale[p][m] = stance ? 1.0 : 0.4;
                }
            }
        }

        // Romper el bucle SOLO si el tiempo se agotó Y las 4 patas tocaron el suelo
        if (tiempo_agotado && todo_aterrizado) break;

        first_run = false;
        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    // ============================================================
    // FASE D: HOLD FINAL (Post-Caminata con Postura)
    // ============================================================
    std::cout << "\n>> Ciclo terminado. Todas las patas aterrizadas de forma segura." << std::endl;
    std::cout << ">> MANTENIENDO POSTURA FINAL." << std::endl;
    std::cout << ">>> PRESIONA ENTER PARA FINALIZAR Y APAGAR <<<" << std::endl;
    
    std::cin.get(); 

    // Bajar suavemente a la postura 0.0 a lo largo de 3 segundos
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

    // Esperar a que los motores terminen de interpolar hasta abajo
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Limpieza final de la memoria
    std::memset(shm_ptr, 0, sizeof(CommandData));
    shm_ptr->is_walking = false; 
    
    std::cout << "\n✅ Finalizado. Robot en posición segura." << std::endl;

    return 0;
}