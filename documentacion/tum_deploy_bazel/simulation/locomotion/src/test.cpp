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
#include <cmath>

#include "robot_types.hpp"
#include "robot_config.hpp"
#include "kinematics.hpp"
#include "gait_generator.hpp" // <-- Tu nueva clase C++

// Estructura sincronizada a byte exacto con Python
#pragma pack(push, 1)
struct UICommand {
    float vx, vy, wz;
    float body_x, body_y, body_z;
    float body_roll, body_pitch, body_yaw;
    float trot_freq;
    int gait_mode;
    bool use_imu;
    char padding[19]; 
};
#pragma pack(pop)

template<typename T>
T* map_shared_memory(const char* name, bool create = false) {
    int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (fd < 0) return nullptr;
    if (create) ftruncate(fd, sizeof(T));
    T* ptr = (T*)mmap(0, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    return (ptr == MAP_FAILED) ? nullptr : ptr;
}

int main() {
    std::cout << "--- INICIANDO CEREBRO C++ NATIVO (GAIT GENERATOR) ---" << std::endl;

    UICommand* ui_cmd = map_shared_memory<UICommand>("/ui_command", true);
    IMUData* imu_ptr = map_shared_memory<IMUData>("/imu_data", false);
    CommandData* shm_ptr = map_shared_memory<CommandData>("/rex_cmd", true);

    if (!ui_cmd || !shm_ptr) {
        std::cerr << "[ERROR] Fallo al abrir memorias compartidas." << std::endl;
        return -1;
    }

    // ==========================================================
    // 1. INICIALIZAR EL NUEVO GAIT GENERATOR
    // ==========================================================
    Eigen::Vector3d default_stances[4];
    for(int i=0; i<4; i++) {
        // Extraemos las posiciones de robot_config.hpp para asegurar compatibilidad
        default_stances[i] = Eigen::Vector3d(LEGS_STAND_XYZ[i].x(), LEGS_STAND_XYZ[i].y(), LEGS_STAND_XYZ[i].z());
    }
    
    // La altura nominal es el valor absoluto de Z (ej. 0.306)
    double real_stand_height = std::abs(default_stances[0].z());
    
    // Instanciamos la clase que acabas de guardar
    NewGaitGenerator gait_gen(real_stand_height, default_stances);

    // ==========================================================
    // 2. CONFIGURACIÓN DEL BUCLE DE CONTROL
    // ==========================================================
    const double loop_dt = 0.005; // 200 Hz
    auto cycle_time = std::chrono::microseconds((int)(loop_dt * 1000000.0));
    auto next_cycle = std::chrono::steady_clock::now();

    double elapsed = 0.0;
    double prev_angles[4][3] = {0};
    bool first_run = true;

    std::cout << "🚀 Generador de Marcha corriendo a 200 Hz. Esperando comandos..." << std::endl;

    while (true) {
        next_cycle += cycle_time;
        elapsed += loop_dt;

        // --- A. LEER DESEOS DEL UI ---
        Eigen::Vector3d vel_cmd(ui_cmd->vx, ui_cmd->vy, ui_cmd->wz);
        
        std::string mode_str = "STAND";
        if (ui_cmd->gait_mode == 1) mode_str = "CRAWL";
        if (ui_cmd->gait_mode == 2) mode_str = "TROT";
        if (ui_cmd->gait_mode == 3) mode_str = "JUMP";

        double freq = std::max(0.5f, ui_cmd->trot_freq);

        Eigen::Vector3d imu_rpy(0, 0, 0);
        if (ui_cmd->use_imu && imu_ptr) {
            imu_rpy.x() = imu_ptr->roll * M_PI / 180.0;
            imu_rpy.y() = imu_ptr->pitch * M_PI / 180.0;
            imu_rpy.z() = imu_ptr->yaw * M_PI / 180.0;
        }

        // --- B. ACTUALIZAR EL GENERADOR DE MARCHA ---
        // ¡Aquí ocurre toda la magia! Nos devuelve dónde deben estar los pies y el cuerpo
        GaitOutput out = gait_gen.update(elapsed, vel_cmd, imu_rpy, mode_str, freq);

        // --- C. COMPENSACIÓN MANUAL DEL USUARIO ---
        // El UI manda body_z = 0.16 por defecto. Restamos 0.16 para no sumar altura extra
        Eigen::Vector3d user_body_pos(ui_cmd->body_x, ui_cmd->body_y, ui_cmd->body_z - 0.16);
        Eigen::Vector3d user_body_rpy(
            ui_cmd->body_roll * M_PI / 180.0,
            ui_cmd->body_pitch * M_PI / 180.0,
            ui_cmd->body_yaw * M_PI / 180.0
        );

        // Sumamos el cálculo de balance del robot + los movimientos manuales del joystick
        Eigen::Vector3d target_body_pos = out.body_shift + user_body_pos;
        Eigen::Vector3d target_body_rpy = out.body_rot + user_body_rpy;

        // --- D. RESOLVER CINEMÁTICA Y ENVIAR A MOTORES ---
        for (int p = 0; p < 4; p++) {
            
            // out.feet[p] trae la altura del pie desde el suelo (0.0 en Stance, >0 en Swing)
            Eigen::Vector3d ik_input = ComputeWholeBodyIK(out.feet[p], p, target_body_pos, target_body_rpy);
            LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), (p==1 || p==3));

            if (ik.valid) {
                shm_ptr->angles[p][0] = ik.th1;
                shm_ptr->angles[p][1] = ik.th2;
                shm_ptr->angles[p][2] = ik.th3;

                if (!first_run) {
                    shm_ptr->velocities[p][0] = (ik.th1 - prev_angles[p][0]) / loop_dt;
                    shm_ptr->velocities[p][1] = (ik.th2 - prev_angles[p][1]) / loop_dt;
                    shm_ptr->velocities[p][2] = (ik.th3 - prev_angles[p][2]) / loop_dt;
                } else {
                    shm_ptr->velocities[p][0] = 0.0;
                    shm_ptr->velocities[p][1] = 0.0;
                    shm_ptr->velocities[p][2] = 0.0;
                }
                
                prev_angles[p][0] = ik.th1;
                prev_angles[p][1] = ik.th2;
                prev_angles[p][2] = ik.th3;

                shm_ptr->desired_accel[p][0] = 0.0;
                shm_ptr->desired_accel[p][1] = 0.0;
                shm_ptr->desired_accel[p][2] = 0.0;

                // Determinamos la fase según la altura del pie (Si z < 5mm, está pisando)
                bool stance = (out.feet[p].z() < 0.005);
                shm_ptr->is_stance[p] = stance;

                // Ajuste de rigidez
                double k_scale = stance ? 1.0 : 0.6;
                double d_scale = stance ? 1.0 : 0.4;
                
                for(int m = 0; m < 3; m++) {
                    shm_ptr->kp_scale[p][m] = k_scale;
                    shm_ptr->kd_scale[p][m] = d_scale;
                }
            }
        }

        first_run = false;
        shm_ptr->is_walking = (ui_cmd->gait_mode != 0);

        std::this_thread::sleep_until(next_cycle);
    }

    return 0;
}