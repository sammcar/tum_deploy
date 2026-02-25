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
#include "utils.hpp"
#include "setup.hpp"

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
    std::signal(SIGINT, SignalHandler);

    // Las variables ahora se declaran aquí, pero se llenan dentro de setup_robot
    double duracion_test, step_duration, vx, vy, wz, step_h;
    bool require_real_hardware = true; // Cambia a false para simular sin hardware real

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

    // Llamamos al setup actualizado (Fase A y B unificadas con la velocidad de arranque)
    if (!setup_robot(duracion_test, step_duration, vx, vy, wz, step_h, 
                     imu_ptr, shm_ptr, tel_ptr, contact_ptr, 
                     target_body_pos, target_body_rpy, 
                     gait_offsets, start_foot_positions, require_real_hardware)) {
        std::cerr << "[FATAL] Error en la inicialización del robot." << std::endl;
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
            bool stance = (leg_phase >= duty_factor); // Si el ciclo de la pata es mayor que el duty factor, está en stance
            Eigen::Vector3d origin = LEGS_STAND_XYZ[p];
            TrajectoryPoint3D tp; 

            if (stance) {
                double t = (leg_phase - 0.5) * 2.0;
                Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0,0,wz).cross(HIP_OFFSETS[p]);
                Eigen::Vector3d p_start = origin - v_total * (step_duration / 4.0);
                Eigen::Vector3d p_target = origin + v_total * (step_duration / 4.0);
                
                tp.pos = p_target + t * (p_start - p_target);
                tp.pos.z() = origin.z();
                tp.acc = Eigen::Vector3d::Zero();
            } else {
                tp = generate_bezier_swing(leg_phase, origin, HIP_OFFSETS[p], vx, vy, wz, 
                                          step_duration/2.0, step_duration/2.0, step_h);
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
