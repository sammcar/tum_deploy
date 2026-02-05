#include <iostream>
#include <vector>
#include <memory>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <csignal>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "moteus.h"

namespace moteus = mjbots::moteus;

// ============================================================
//    AJUSTES DE CALIBRACIÓN (INTACTOS)
// ============================================================
const double kFactorMultiplicador = 1.57; 
const double kFactorMultiplicador2 = 0.888;          

// ============================================================
//    ESTRUCTURA DE PERFIL DE MOTOR (INTACTO)
// ============================================================
struct MotorConfig {
    double kp;
    double kd;
    double max_torque;   
    double vel_limit;    
    double accel_limit;  
};

// ============================================================
//    CONFIGURACIÓN POR GRUPOS (INTACTO)
// ============================================================
const MotorConfig kCoxaConfig = { .kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0 };
const MotorConfig kFemurConfig = { .kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0 };
const MotorConfig kTibiaConfig = { .kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0 };

const std::vector<int> kMotorIds = {1,2,3,4,5,6,7,8,9,10,11,12}; 

// ============================================================
//    ESTRUCTURAS DE SISTEMA
// ============================================================

struct SharedData {
    double angles[4][3]; 
    double times[4][3];  
    bool emergency_stop;
};

struct TrajectoryState {
    double start_pos_rev;   
    double target_pos_rev;  
    double start_time_s;    
    double duration_s;      
    bool is_moving;         
};

struct MotorTelemetry {
    int id;
    std::string type_name;
    double target_deg; 
    double raw_deg;    
    double pos_deg;
    double vel_deg_s;
    double tor_nm;
    double temp_c;
    bool online;
    double cmd_pos_now; 
    double target_time; 
};

// ============================================================
//    FUNCIÓN MATEMÁTICA: MINIMUM JERK
// ============================================================
double CalculateMinimumJerk(double t_elapsed, double t_total, double p_start, double p_end) {
    if (t_total < 0.001) return p_end; 
    if (t_elapsed >= t_total) return p_end;
    if (t_elapsed <= 0) return p_start;

    double u = t_elapsed / t_total;
    double factor = (10.0 * pow(u, 3)) - (15.0 * pow(u, 4)) + (6.0 * pow(u, 5));

    return p_start + (p_end - p_start) * factor;
}

class MemoryManager {
public:
    const char* shm_name = "rex_shm";
    int shm_fd = -1;
    void* ptr = MAP_FAILED;
    SharedData* data = nullptr;
    bool is_creator = false;

    MemoryManager() {
        shm_fd = shm_open(shm_name, O_RDWR, 0666);
        if (shm_fd == -1) {
            shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
            ftruncate(shm_fd, sizeof(SharedData));
            is_creator = true;
        }
        ptr = mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        data = static_cast<SharedData*>(ptr);
        if (is_creator) std::memset(ptr, 0, sizeof(SharedData));
    }
    ~MemoryManager() {
        if (ptr != MAP_FAILED) munmap(ptr, sizeof(SharedData));
        if (shm_fd != -1) close(shm_fd);
    }
    bool is_valid() { return data != nullptr; }
};

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    moteus::Controller::DefaultArgProcess(argc, argv);

    MemoryManager memory;
    if (!memory.is_valid()) return 1;

    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (int id : kMotorIds) {
        moteus::Controller::Options options;
        options.id = id;
        options.position_format.kp_scale = moteus::kFloat;
        options.position_format.kd_scale = moteus::kFloat;
        options.position_format.velocity_limit = moteus::kFloat;
        options.position_format.accel_limit = moteus::kFloat;
        options.position_format.maximum_torque = moteus::kFloat;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::vector<MotorTelemetry> telemetry_data(kMotorIds.size());
    std::vector<TrajectoryState> motor_states(kMotorIds.size());
    
    for(auto& s : motor_states) { s.is_moving = false; s.target_pos_rev = 0.0; }
    for(auto& t : telemetry_data) t.pos_deg = 0.0;

    auto start_time = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double t_global_sec = std::chrono::duration<double>(now - start_time).count();

        for (size_t i = 0; i < controllers.size(); ++i) {
            int id = kMotorIds[i]; 
            
            int joint_type = (id - 1) % 3; 
            const MotorConfig* config = nullptr;
            std::string type_str;

            if (joint_type == 0) { config = &kCoxaConfig; type_str = "COXA"; }
            else if (joint_type == 1) { config = &kFemurConfig; type_str = "FEMUR"; }
            else { config = &kTibiaConfig; type_str = "TIBIA"; }

            int memory_index = id - 1; 
            int row = memory_index / 3; 
            int col = memory_index % 3; 
            
            double raw_target = memory.data->angles[row][col]; 
            double time_target_s = memory.data->times[row][col]; 

            double final_target = raw_target;

            if (id == 3 || id == 6) final_target = raw_target * kFactorMultiplicador;
            else if (id == 9 || id == 12) final_target = raw_target*kFactorMultiplicador2;

            double final_target_rev = final_target / 360.0;

            // ============================================================
            //    LÓGICA DE INTERPOLACIÓN (MINIMUM JERK)
            // ============================================================
            TrajectoryState& state = motor_states[i];

            if (std::abs(final_target_rev - state.target_pos_rev) > 0.0001) {
                if (telemetry_data[i].online) {
                    state.start_pos_rev = telemetry_data[i].pos_deg / 360.0;
                } else {
                    state.start_pos_rev = final_target_rev; 
                }
                state.target_pos_rev = final_target_rev;
                state.duration_s = time_target_s;
                state.start_time_s = t_global_sec;
                state.is_moving = true;
            }

            double cmd_position_rev = final_target_rev; 

            if (state.is_moving) {
                double t_elapsed = t_global_sec - state.start_time_s;
                
                if (state.duration_s > 0.01) {
                    cmd_position_rev = CalculateMinimumJerk(
                        t_elapsed, 
                        state.duration_s, 
                        state.start_pos_rev, 
                        state.target_pos_rev
                    );
                }

                if (t_elapsed >= state.duration_s) {
                    state.is_moving = false; 
                }
            }

            // --- ENVÍO DE COMANDO ---
            moteus::PositionMode::Command cmd;
            cmd.position = cmd_position_rev; 
            cmd.velocity = 0.0;
            
            // ============================================================
            //    MODIFICACIÓN KP ESPECÍFICO PARA MOTOR 11
            // ============================================================
            if (id == 11) {
                cmd.kp_scale = 0.5; // VALOR PERSONALIZADO PARA EL MOTOR 11
                cmd.kd_scale = 0.5; // VALOR PERSONALIZADO PARA EL MOTOR 11
            } else {
                cmd.kp_scale = config->kp; // Valor normal para el resto
            }
            // ============================================================

            cmd.kd_scale = config->kd;
            cmd.maximum_torque = config->max_torque;
            cmd.velocity_limit = config->vel_limit; 
            cmd.accel_limit = config->accel_limit; 
            
            cmd.stop_position = std::numeric_limits<double>::quiet_NaN();

            const auto maybe_result = controllers[i]->SetPosition(cmd);

            // --- TELEMETRÍA ---
            telemetry_data[i].id = id;
            telemetry_data[i].type_name = type_str;
            telemetry_data[i].target_deg = final_target;
            telemetry_data[i].target_time = time_target_s;
            telemetry_data[i].cmd_pos_now = cmd_position_rev * 360.0; 
            
            if (maybe_result) {
                telemetry_data[i].online = true;
                telemetry_data[i].pos_deg = maybe_result->values.position * 360.0;
                telemetry_data[i].vel_deg_s = maybe_result->values.velocity * 360.0;
                telemetry_data[i].tor_nm = maybe_result->values.torque;
                telemetry_data[i].temp_c = maybe_result->values.temperature;
            } else {
                telemetry_data[i].online = false;
            }
        }

        // --- VISUALIZACIÓN ---
        static int print_cnt = 0;
        if (print_cnt++ % 10 == 0) { 
            std::cout << "\033[2J\033[H"; 
            
            std::cout << "========= MINIMUM JERK (MOTOR 11: KP 10.0) =========\n";
            std::cout << " ID | TARGET(°) | TIME(s) |  CMD(°)  |  REAL(°) | TOR(Nm)\n"; 
            std::cout << "----|-----------|---------|----------|----------|--------\n";

            for (const auto& data : telemetry_data) {
                if (data.online) {
                    double error = std::abs(data.cmd_pos_now - data.pos_deg);
                    std::string color = (error > 5.0) ? "\033[1;31m" : "\033[0m";

                    std::cout << " " << std::setw(2) << data.id << " | "
                              << std::setw(9) << std::fixed << std::setprecision(1) << data.target_deg << " | "
                              << std::setw(7) << std::setprecision(2) << data.target_time << " | " 
                              << std::setw(8) << data.cmd_pos_now << " | " 
                              << color << std::setw(8) << data.pos_deg << "\033[0m | "
                              << std::setw(7) << std::setprecision(2) << data.tor_nm << "\n";
                } else {
                    std::cout << " " << std::setw(2) << data.id << " | OFFLINE\n";
                }
            }
            std::cout << "--------------------------------------------------\n";
            std::cout << std::flush;
        }

        ::usleep(10000); // 100Hz
    }

    std::cout << "\n\nDeteniendo..." << std::endl;
    for (auto& controller : controllers) controller->SetStop();
    return 0;
}
