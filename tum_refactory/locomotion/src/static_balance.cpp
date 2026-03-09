#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <cmath>

#include "robot_types.hpp"
#include "robot_config.hpp"
#include "kinematics.hpp"
#include "memory_setup.hpp"
#include "gait_setup.hpp"
#include "zmp_control.hpp"

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
    std::signal(SIGINT, SignalHandler);

    bool require_real_hardware = true; 
    bool dont_ask = true; // Cambia a false si quieres meter la postura a mano en consola
    bool ask_pid = true;  // Cambia a false si quieres usar las ganancias por defecto abajo

    IMUData* imu_ptr = nullptr;
    CommandData* shm_ptr = nullptr;
    TelemetryData* tel_ptr = nullptr;
    ContactData* contact_ptr = nullptr;

    if (!init_shared_memory(require_real_hardware, imu_ptr, shm_ptr, tel_ptr, contact_ptr)) {
        return 1;
    }

    // Variables que gait_setup requiere
    double duracion_test, step_duration, vx, vy, wz, step_h;
    Eigen::Vector3d target_body_pos, target_body_rpy;
    
    // Forzamos al robot a creer que siempre está con las 4 patas en el suelo
    double duty_factor = 1.0; 
    double gait_offsets[4] = {0.0, 0.0, 0.0, 0.0}; 
    Eigen::Vector3d start_foot_positions[4];

    // --- FASE 1: APLICAR POSTURA INICIAL Y TIEMPOS ---
    if (!init_gait_posture(dont_ask, duracion_test, step_duration, vx, vy, wz, step_h, 
                           shm_ptr, target_body_pos, target_body_rpy, 
                           duty_factor, gait_offsets, start_foot_positions)) {
        return 1; 
    }

    // --- FASE 2: SINTONIZACIÓN DE GANANCIAS ---
    double Kp_zmp, Kd_zmp;
    if (ask_pid) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "   MODO SINTONIZACIÓN: BALANCEO ESTÁTICO" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << ">> Kp (Proporcional - Rigidez): "; std::cin >> Kp_zmp;
        std::cout << ">> Kd (Derivativo - Freno): "; std::cin >> Kd_zmp;
        std::cin.get(); // Limpiar el buffer del Enter
    } else {
        Kp_zmp = 0.02; // Ganancia segura sugerida
        Kd_zmp = 0.2;  // Amortiguador alto
    }

    std::cout << "\n>>> PRESIONA ENTER PARA ACTIVAR EL CONTROLADOR ZMP <<<" << std::endl;
    std::cin.get();

    Eigen::Vector3d base_body_pos = target_body_pos; // Guardamos la postura que gait_setup calculó
    bool enable_zmp_debug = true; 
    ZMPDebugData zmp_debug_info;

    std::thread debug_thread([&]() {
        while (g_running) {
            if (enable_zmp_debug) {
                // Evaluamos si hay alguna corrección activa (mayor a 0.00001 para ignorar ruido de flotantes)
                bool hay_accion = (std::abs(zmp_debug_info.u_x_filtered) > 1e-5) || 
                                  (std::abs(zmp_debug_info.u_y_filtered) > 1e-5);

                if (hay_accion) {
                    std::cout << std::fixed << std::setprecision(4)
                              << "[ZMP] Ux: " << zmp_debug_info.u_x_filtered
                              << " | Uy: " << zmp_debug_info.u_y_filtered
                              << " || KpX: " << zmp_debug_info.kp_term_x
                              << " | KdX: " << zmp_debug_info.kd_term_x 
                              << " | KpY: " << zmp_debug_info.kp_term_y
                              << " | KdY: " << zmp_debug_info.kd_term_y << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    auto start_time = std::chrono::steady_clock::now();
    const double loop_dt = 0.005; 

    // ============================================================
    // FASE 3: BUCLE DE CONTROL ESTÁTICO (SIN MARCHA)
    // ============================================================
    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        
        bool tiempo_agotado = (elapsed > duracion_test);

        double z_sum = 0.0;
        for (int i=0; i<4; ++i) z_sum += LEGS_STAND_XYZ[i].z();
        double h_com = std::abs((z_sum / 4.0) + target_body_pos.z());

        // Al ZMP le pasamos velocidad 0.0 para anular la inercia calculada de la caminata
        Eigen::Vector2d u_zmp = compute_zmp_offset(require_real_hardware, contact_ptr, imu_ptr, 0.0, 
                                                   gait_offsets, duty_factor, step_duration, 0.0, 0.0, 0.0, 0.0, 
                                                   h_com, Kp_zmp, Kd_zmp, &zmp_debug_info);

        target_body_pos.x() = base_body_pos.x() + u_zmp.x();
        target_body_pos.y() = base_body_pos.y() + u_zmp.y();

        for (int p = 0; p < 4; p++) {
            // Se fuerza el pie a quedarse anclado en la posición que calculó el gait_setup
            Eigen::Vector3d origin = start_foot_positions[p];
            Eigen::Vector3d ik_input = ComputeWholeBodyIK(origin, p, target_body_pos, target_body_rpy);
            LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), (p==1 || p==3));

            if (ik.valid) {
                shm_ptr->angles[p][0] = ik.th1;
                shm_ptr->angles[p][1] = ik.th2;
                shm_ptr->angles[p][2] = ik.th3;

                // Motores con rigidez estática máxima
                for(int m=0; m<3; m++) {
                    shm_ptr->kp_scale[p][m] = 1.0;
                    shm_ptr->kd_scale[p][m] = 1.0;
                    shm_ptr->velocities[p][m] = 0.0;
                    shm_ptr->desired_accel[p][m] = 0.0;
                }
                shm_ptr->is_stance[p] = true;
            }
        }

        // Si se nos acaba el tiempo definido en duracion_test, rompemos el bucle
        if (tiempo_agotado) break;

        shm_ptr->is_walking = false; 
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    // --- Silenciamos la consola para no interferir con las instrucciones ---
    enable_zmp_debug = false; 
    std::cout << "\n";

    // ============================================================
    // FASE D: HOLD FINAL (Post-Control con Postura)
    // ============================================================
    std::cout << "\n>> Ciclo terminado de forma segura." << std::endl;
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

    // Apagar el bucle del hilo y esperar a que termine
    g_running = false;
    if (debug_thread.joinable()) {
        debug_thread.join();
    }
    
    std::cout << "\n✅ Finalizado. Robot en posición segura." << std::endl;
    return 0;
}
