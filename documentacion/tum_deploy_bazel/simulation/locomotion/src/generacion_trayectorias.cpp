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
#include <fstream>
#include <csignal>
#include <cmath>
#include <Eigen/Dense>

// Inclusión de módulos del proyecto ATOM-51
#include "robot_types.hpp"
#include "robot_config.hpp"
#include "kinematics.hpp"
#include "memory_setup.hpp"
#include "gait_setup.hpp"
#include "zmp_control.hpp"

// ============================================================
// MÓDULO DE PLANIFICACIÓN DE MARCHA (GAIT PLANNER)
// ============================================================

struct CartesianState {
    Eigen::Vector3d P_des; 
    Eigen::Vector3d V_des; 
    Eigen::Vector3d A_des; 
    double kp_scale; 
    double kd_scale; 
    bool is_stance; 
};

class LegGaitPlanner {
private:
    int leg_id_;
    Eigen::Vector3d origin_;
    Eigen::Vector3d hip_offset_;
    
    double gait_offset_;
    double duty_factor_;
    double step_duration_;
    double step_h_;

    // Parámetros de ajuste de trayectoria (Swing)
    double world_blend_ = 0.15;
    double damp_start_phase_ = 0.85;
    double damp_scale_kp_ = 0.6;
    double damp_scale_kd_ = 0.4;

    void EvaluateBezierCubic(double t, const Eigen::Vector3d& P0, const Eigen::Vector3d& P1, 
                             const Eigen::Vector3d& P2, const Eigen::Vector3d& P3,
                             Eigen::Vector3d& P_out, Eigen::Vector3d& V_out, Eigen::Vector3d& A_out) {
        double u = 1.0 - t;
        double tt = t * t; double uu = u * u;
        double uuu = uu * u; double ttt = tt * t;

        P_out = uuu * P0 + 3 * uu * t * P1 + 3 * u * tt * P2 + ttt * P3;
        V_out = 3 * uu * (P1 - P0) + 6 * u * t * (P2 - P1) + 3 * tt * (P3 - P2);
        A_out = 6 * u * (P2 - 2 * P1 + P0) + 6 * t * (P3 - 2 * P2 + P1);
    }

public:
    LegGaitPlanner() {}

    void Initialize(int leg_id, const Eigen::Vector3d& origin, const Eigen::Vector3d& hip_offset,
                    double gait_offset, double duty_factor, double step_duration, double step_h) {
        leg_id_ = leg_id;
        origin_ = origin;
        hip_offset_ = hip_offset;
        gait_offset_ = gait_offset;
        duty_factor_ = duty_factor;
        step_duration_ = step_duration;
        step_h_ = step_h;
    }

    CartesianState Update(double global_phase, double vx, double vy, double wz) {
        CartesianState state;
        
        double leg_phase = fmod(global_phase + gait_offset_, 1.0);
        double stance_portion = duty_factor_;
        double swing_portion = 1.0 - duty_factor_;
        
        state.is_stance = (leg_phase >= swing_portion);

        double t_stance = step_duration_ * stance_portion;
        Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0,0,wz).cross(hip_offset_);
        
        Eigen::Vector3d p_start = origin_ - v_total * (t_stance / 2.0);
        Eigen::Vector3d p_target = origin_ + v_total * (t_stance / 2.0);

        if (state.is_stance) {
            double t = (leg_phase - swing_portion) / stance_portion;
            
            state.P_des = p_target + t * (p_start - p_target);
            state.P_des.z() = origin_.z();
            state.V_des = -v_total; 
            state.A_des.setZero();
            
            state.kp_scale = 1.0;
            state.kd_scale = 1.0;
        } 
        else {
            double phi = leg_phase / swing_portion; 
            double flight_duration = step_duration_ * swing_portion;

            Eigen::Vector3d P, V, A;
            P.setZero(); V.setZero(); A.setZero();

            // Eje Z
            double z_max = p_start.z() + step_h_;
            if (phi < 0.5) {
                double phi_z = phi * 2.0; 
                P.z() = p_start.z() + (z_max - p_start.z()) * std::sin(phi_z * M_PI / 2.0);
                V.z() = (z_max - p_start.z()) * std::cos(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            } else {
                double phi_z = (phi - 0.5) * 2.0;
                P.z() = z_max - (z_max - p_target.z()) * (1.0 - std::cos(phi_z * M_PI / 2.0));
                V.z() = -(z_max - p_target.z()) * std::sin(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            }

            // Ejes XY
            Eigen::Vector3d xy_start(p_start.x(), p_start.y(), 0.0);
            Eigen::Vector3d xy_target(p_target.x(), p_target.y(), 0.0);
            Eigen::Vector3d V_world_xy(-v_total.x(), -v_total.y(), 0.0);

            if (phi < world_blend_) {
                double t_local = phi / world_blend_;
                V.head<2>() = V_world_xy.head<2>() * (1.0 - t_local);
                P.head<2>() = xy_start.head<2>() + (V_world_xy.head<2>() * (phi * flight_duration)); 
            } else if (phi < 1.0 - world_blend_) {
                double t_local = (phi - world_blend_) / (1.0 - 2.0 * world_blend_);
                Eigen::Vector3d P1 = xy_start + (xy_target - xy_start) * 0.2;
                Eigen::Vector3d P2 = xy_start + (xy_target - xy_start) * 0.8;
                EvaluateBezierCubic(t_local, xy_start, P1, P2, xy_target, P, V, A);
                V.head<2>() = V.head<2>() / (flight_duration * (1.0 - 2.0 * world_blend_));
            } else {
                double t_local = (phi - (1.0 - world_blend_)) / world_blend_;
                P.head<2>() = xy_target.head<2>(); 
                V.head<2>() = V_world_xy.head<2>() * t_local; 
            }

            state.P_des = P;
            state.V_des = V;
            state.A_des = A;

            // Amortiguación final del vuelo
            if (phi > damp_start_phase_) {
                state.kp_scale = damp_scale_kp_;
                state.kd_scale = damp_scale_kd_;
            } else {
                state.kp_scale = 1.0;
                state.kd_scale = 1.0;
            }
        }

        return state;
    }
};

// ============================================================
// MAIN
// ============================================================

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
    std::signal(SIGINT, SignalHandler);

    double duracion_test, step_duration, vx, vy, wz, step_h;
    bool require_real_hardware = false; 
    bool dont_ask = true; 
    bool ask_pid = false; 
    double Kp_zmp, Kd_zmp;

    IMUData* imu_ptr = nullptr;
    CommandData* shm_ptr = nullptr;
    TelemetryData* tel_ptr = nullptr;
    ContactData* contact_ptr = nullptr;
    Eigen::Vector3d target_body_pos, target_body_rpy;
    
    double duty_factor = 0.75; 
    double gait_offsets[4] = {0.0, 0.5, 0.75, 0.25}; // Trot
    Eigen::Vector3d start_foot_positions[4];

    if (!init_shared_memory(require_real_hardware, imu_ptr, shm_ptr, tel_ptr, contact_ptr)) {
        std::cerr << "[FATAL] Fallo al inicializar la memoria." << std::endl;
        return 1;
    }

    if (!init_gait_posture(dont_ask, duracion_test, step_duration, vx, vy, wz, step_h, 
                           shm_ptr, target_body_pos, target_body_rpy, 
                           duty_factor, gait_offsets, start_foot_positions)) {
        std::cerr << "[FATAL] Error en la configuración de la marcha." << std::endl;
        return 1; 
    }

    if (ask_pid) {
        std::cout << "\n[INFO] Ajuste de Ganancias PID para ZMP Control." << std::endl;
        std::cout << ">> Kp: "; std::cin >> Kp_zmp;
        std::cout << ">> Kd: "; std::cin >> Kd_zmp;
    }else{
        Kp_zmp = 0.05; Kd_zmp = 0.01;
    }
    std::cin.get();

    // ------------------------------------------------------------------
    // INICIALIZACIÓN DE TUS PLANIFICADORES DE MARCHA (Se hace UNA VEZ)
    // ------------------------------------------------------------------
    LegGaitPlanner gait_planners[4];
    for (int p = 0; p < 4; p++) {
        gait_planners[p].Initialize(p, LEGS_STAND_XYZ[p], HIP_OFFSETS[p], 
                                    gait_offsets[p], duty_factor, step_duration, step_h);
    }

    std::cout << "\n>>> PRESIONA ENTER PARA INICIAR MARCHA <<<" << std::endl;
    std::cin.get();

    auto start_time = std::chrono::steady_clock::now();
    const double loop_dt = 0.005; 
    double prev_angles[4][3] = {0};
    
    for(int p=0; p<4; ++p) {
        for(int m=0; m<3; ++m) prev_angles[p][m] = shm_ptr->angles[p][m];
    }

    bool first_run = true;
    Eigen::Vector3d base_body_pos = target_body_pos;

    // Lanzar hilo de telemetría / debug
    bool enable_zmp_debug = true; 
    ZMPDebugData zmp_debug_info;
    std::thread debug_thread([&]() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    });

    // ============================================================
    // BUCLE DE CONTROL DE ALTA FRECUENCIA (200Hz)
    // ============================================================
    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        
        if (elapsed > duracion_test) break; // Terminar caminata
        
        // Fase global normalizada y cíclica (0.0 -> 1.0 -> 0.0)
        double global_phase = fmod(elapsed / step_duration, 1.0);

        // --- Compensación ZMP (Estabilidad global del chasis) ---
        double z_sum = 0.0;
        for (int i=0; i<4; ++i) z_sum += LEGS_STAND_XYZ[i].z();
        double h_com = std::abs((z_sum / 4.0) + target_body_pos.z());

        Eigen::Vector2d u_zmp = compute_zmp_offset(require_real_hardware, contact_ptr, imu_ptr, global_phase, 
                                                   gait_offsets, duty_factor, step_duration, vx, vy, wz, step_h, 
                                                   h_com, Kp_zmp, Kd_zmp, &zmp_debug_info);

        target_body_pos.x() = base_body_pos.x() + u_zmp.x();
        target_body_pos.y() = base_body_pos.y() + u_zmp.y();

        // --- Iteración por cada pata ---
        for (int p = 0; p < 4; p++) {
            
            // 1. TU PLANIFICADOR (Genera la trayectoria matemática ideal)
            CartesianState ideal = gait_planners[p].Update(global_phase, vx, vy, wz);

            // --------------------------------------------------------
            // 2. FRONTERA DEL COLEGA (AQUÍ ENTRA EL CONTROL CARTESIANO)
            // --------------------------------------------------------
            // Él usará `ideal.P_des`, `ideal.V_des`, `ideal.kp_scale`, etc.,
            // para calcular errores contra los encoders y aplicar fuerzas.
            //
            // Por ahora, usamos Cinemática Inversa pura para que el robot se mueva, 
            // pero él reemplazará o expandirá este bloque:

            Eigen::Vector3d ik_input = ComputeWholeBodyIK(ideal.P_des, p, target_body_pos, target_body_rpy);
            LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), (p==1 || p==3));

            if (ik.valid) {
                shm_ptr->angles[p][0] = ik.th1;
                shm_ptr->angles[p][1] = ik.th2;
                shm_ptr->angles[p][2] = ik.th3;

                // Cálculo derivativo numérico para la velocidad (temporal hasta que tu colega lo haga por Jacobiano)
                if (!first_run) {
                    shm_ptr->velocities[p][0] = (ik.th1 - prev_angles[p][0]) / loop_dt;
                    shm_ptr->velocities[p][1] = (ik.th2 - prev_angles[p][1]) / loop_dt;
                    shm_ptr->velocities[p][2] = (ik.th3 - prev_angles[p][2]) / loop_dt;
                } else {
                    shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
                }
                prev_angles[p][0] = ik.th1; prev_angles[p][1] = ik.th2; prev_angles[p][2] = ik.th3;

                // Escribir a memoria los perfiles de tu planificador para que los motores obedezcan
                shm_ptr->desired_accel[p][0] = ideal.A_des.x();
                shm_ptr->desired_accel[p][2] = ideal.A_des.z();
                shm_ptr->is_stance[p] = ideal.is_stance;

                // Asignar los KP y KD escalados (la magia suave del aterrizaje)
                for(int m=0; m<3; m++) {
                    shm_ptr->kp_scale[p][m] = ideal.kp_scale; 
                    shm_ptr->kd_scale[p][m] = ideal.kd_scale;
                }
            }
        }

        first_run = false;
        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    enable_zmp_debug = false; 

    // ============================================================
    // FASE DE APAGADO (Hold Final)
    // ============================================================
    std::cout << "\n>> Caminata terminada. MANTENIENDO POSTURA FINAL." << std::endl;
    std::cout << ">>> PRESIONA ENTER PARA FINALIZAR Y APAGAR <<<" << std::endl;
    std::cin.get(); 

    // Bajar suavemente a la postura 0.0
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

    g_running = false;
    if (debug_thread.joinable()) { debug_thread.join(); }
    
    std::cout << "\n✅ Finalizado. Robot apagado." << std::endl;
    return 0;
}