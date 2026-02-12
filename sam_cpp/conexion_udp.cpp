#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <unistd.h>
#include <future>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <limits>

#include "moteus.h"

namespace moteus = mjbots::moteus;

// ============================================================
//    VARIABLES GLOBALES Y SEÑALES
// ============================================================
std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

// ============================================================
//    AJUSTES MECÁNICOS Y DE SEGURIDAD
// ============================================================
const double kFactorMultiplicador = 1.57;
const double kFactorMultiplicador2 = 0.888;

struct MotorConfig {
    double kp;
    double kd;
    double max_torque;
    double vel_limit;
    double accel_limit;
};

// Configuración unificada
const MotorConfig kCoxaConfig =  {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 6.0, .accel_limit = 10.0};
const MotorConfig kFemurConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 6.0, .accel_limit = 10.0};
const MotorConfig kTibiaConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 6.0, .accel_limit = 10.0};

const std::vector<int> kMotorIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

struct SharedData {
    double angles[4][3];
    double times[4][3];
    bool is_walking; 
};

struct TrajectoryState {
    double current_cmd_pos = 0.0;  
    double start_pos_rev = 0.0;
    double target_pos_rev = 0.0;
    double start_time_s = 0.0;
    double duration_s = 0.0;
    double theoretical_vel = 0.0;  
    bool is_moving = false;
};

// ============================================================
//    GESTOR DE MEMORIA COMPARTIDA
// ============================================================
class MemoryManager {
public:
    const char *shm_name = "/rex_shm";
    int shm_fd = -1;
    void *ptr = MAP_FAILED;
    SharedData *data = nullptr;

    MemoryManager() {
        shm_fd = shm_open(shm_name, O_RDWR | O_CREAT, 0666);
        if (shm_fd == -1) return;

        struct stat shm_stat;
        fstat(shm_fd, &shm_stat);
        
        bool is_new = (shm_stat.st_size == 0);
        if (is_new) ftruncate(shm_fd, sizeof(SharedData));

        ptr = mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        data = static_cast<SharedData *>(ptr);

        if (is_new) std::memset(data, 0, sizeof(SharedData));
    }

    ~MemoryManager() {
        if (ptr != MAP_FAILED) munmap(ptr, sizeof(SharedData));
        if (shm_fd != -1) close(shm_fd);
    }
    
    bool is_valid() { return data != nullptr; }
};

// ============================================================
//    MATEMÁTICAS
// ============================================================
double CalculateMinimumJerk(double t_elapsed, double t_total, double p_start, double p_end) {
    if (t_total < 0.001 || t_elapsed >= t_total) return p_end;
    if (t_elapsed <= 0) return p_start;
    double u = t_elapsed / t_total;
    double poly = (u * u * u) * (10.0 + u * (-15.0 + 6.0 * u));
    return p_start + (p_end - p_start) * poly;
}

double CalculateMinimumJerkVelocity(double t_elapsed, double t_total, double p_start, double p_end) {
    if (t_total < 0.001 || t_elapsed >= t_total || t_elapsed <= 0) return 0.0;
    double u = t_elapsed / t_total;
    double poly_deriv = 30.0 * (u * u) - 60.0 * (u * u * u) + 30.0 * (u * u * u * u);
    return ((p_end - p_start) * poly_deriv) / t_total;
}

const MotorConfig* GetMotorConfig(int id) {
    int type = (id - 1) % 3; 
    if (type == 0) return &kCoxaConfig;
    if (type == 1) return &kFemurConfig;
    return &kTibiaConfig;
}

double GetCalibratedTarget(int id, double target_deg) {
    double final_target = target_deg;
    if (id == 3 || id == 6) final_target *= kFactorMultiplicador;
    else if (id == 9 || id == 12) final_target *= kFactorMultiplicador2;
    return final_target;
}

// ============================================================
//    COMUNICACIÓN
// ============================================================
void SafeTransportCycle(std::shared_ptr<moteus::Transport> transport, 
                        const std::vector<moteus::CanFdFrame>& send_frames, 
                        std::vector<moteus::CanFdFrame>* receive_frames = nullptr) {
    std::promise<void> cycle_done;
    transport->Cycle(send_frames.data(), send_frames.size(), receive_frames, [&cycle_done](int) { cycle_done.set_value(); });
    cycle_done.get_future().wait(); 
}

// ============================================================
//    MAIN
// ============================================================
int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    MemoryManager memory;
    if (!memory.is_valid()) return 1;

    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    std::vector<TrajectoryState> motor_states(13); 

    for (int id : kMotorIds) {
        moteus::Controller::Options options;
        options.id = id;
        options.transport = transport;
        options.position_format.kp_scale = moteus::kFloat;
        options.position_format.kd_scale = moteus::kFloat;
        options.position_format.velocity_limit = moteus::kFloat;
        options.position_format.accel_limit = moteus::kFloat;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::cout << "\033[2J"; 

    // --- LECTURA INICIAL (NECESARIA PARA HOMING) ---
    std::cout << ">> LEYENDO POSICIONES INICIALES..." << std::endl;
    std::vector<moteus::CanFdFrame> initial_receive;
    std::vector<moteus::CanFdFrame> initial_send;
    for (auto& c : controllers) initial_send.push_back(c->MakePosition({}));
    
    SafeTransportCycle(transport, initial_send, &initial_receive);

    for (const auto& frame : initial_receive) {
        if (frame.source >= 1 && frame.source <= 12) {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            motor_states[frame.source].current_cmd_pos = res.position;
            motor_states[frame.source].target_pos_rev = res.position;
            motor_states[frame.source].start_pos_rev = res.position; // <--- CORRECCIÓN 1: Guardar el punto de partida real
        }
    }

    // --- HOMING SEGURA ---
    auto startup_start = std::chrono::steady_clock::now();
    double startup_duration = 3.0;
    std::cout << ">> EJECUTANDO HOMING A 0.0..." << std::endl;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - startup_start).count();
        if (elapsed > startup_duration) break;

        std::vector<moteus::CanFdFrame> startup_frames;
        for (auto& c : controllers) {
            int id = c->options().id;
            const MotorConfig* cfg = GetMotorConfig(id);
            
            // <--- CORRECCIÓN 2: Usar start_pos_rev (fijo) en lugar de current_cmd_pos
            double pos = CalculateMinimumJerk(elapsed, startup_duration, motor_states[id].start_pos_rev, 0.0);
            double vel = CalculateMinimumJerkVelocity(elapsed, startup_duration, motor_states[id].start_pos_rev, 0.0);
            
            // Actualizamos estado interno
            motor_states[id].current_cmd_pos = pos;

            moteus::PositionMode::Command cmd;
            cmd.position = pos;
            cmd.velocity = vel;
            cmd.velocity_limit = cfg->vel_limit;
            cmd.accel_limit = cfg->accel_limit;
            // Torque suave para homing
            cmd.kp_scale = 0.5; 
            cmd.kd_scale = 0.5;
            cmd.maximum_torque = 5.0; 
            
            startup_frames.push_back(c->MakePosition(cmd));
        }
        SafeTransportCycle(transport, startup_frames, nullptr);
        usleep(10000); // 100Hz
    }

    // --- RESET FINAL DE ESTADO ---
    for (int id : kMotorIds) {
        motor_states[id].current_cmd_pos = 0.0;
        motor_states[id].start_pos_rev = 0.0;
        motor_states[id].target_pos_rev = 0.0;
        motor_states[id].is_moving = false;
    }

    // --- BUCLE PRINCIPAL ---
    std::cout << ">> INICIANDO LAZO ABIERTO (100Hz)..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double t_global_sec = std::chrono::duration<double>(now - start_time).count();
        bool is_walking_mode = memory.data->is_walking;
        std::vector<moteus::CanFdFrame> send_frames;

        for (size_t i = 0; i < controllers.size(); ++i) {
            int id = kMotorIds[i];
            const MotorConfig* cfg = GetMotorConfig(id);
            int row = (id - 1) / 3;
            int col = (id - 1) % 3;

            double target_deg = GetCalibratedTarget(id, memory.data->angles[row][col]);
            double move_time = memory.data->times[row][col];
            double target_rev = target_deg / 360.0;

            TrajectoryState &s = motor_states[id];
            double cmd_pos, cmd_vel;

            if (is_walking_mode) {
                if (std::abs(target_rev - s.target_pos_rev) > 0.00001) {
                    s.start_pos_rev = s.current_cmd_pos; 
                    s.target_pos_rev = target_rev;
                    s.duration_s = (move_time < 0.001) ? 0.02 : move_time;
                    s.start_time_s = t_global_sec;
                    s.is_moving = true;
                    s.theoretical_vel = (s.target_pos_rev - s.start_pos_rev) / s.duration_s;
                }

                if (s.is_moving) {
                    double t_elap = t_global_sec - s.start_time_s;
                    if (t_elap >= s.duration_s) {
                        cmd_pos = s.target_pos_rev;
                        cmd_vel = 0.0;
                        s.is_moving = false;
                    } else {
                        cmd_pos = s.start_pos_rev + s.theoretical_vel * t_elap;
                        cmd_vel = s.theoretical_vel;
                    }
                } else {
                    cmd_pos = s.target_pos_rev;
                    cmd_vel = 0.0;
                }
            } else {
                cmd_pos = s.target_pos_rev;
                cmd_vel = 0.0;
                if (std::abs(target_rev - s.target_pos_rev) > 0.0001) s.target_pos_rev = target_rev;
            }

            s.current_cmd_pos = cmd_pos;

            moteus::PositionMode::Command cmd;
            cmd.position = cmd_pos;
            cmd.velocity = cmd_vel;
            cmd.kp_scale = (id == 10 || id == 11) ? 0.8 : cfg->kp;
            cmd.kd_scale = (id == 10 || id == 11) ? 0.8 : cfg->kd;
            cmd.velocity_limit = cfg->vel_limit;
            cmd.accel_limit = cfg->accel_limit;
            cmd.maximum_torque = cfg->max_torque;
            
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        SafeTransportCycle(transport, send_frames, nullptr);
        
        // Debug (latido)
        static int hb = 0;
        if (hb++ % 100 == 0) std::cout << "." << std::flush;

        usleep(5000); // <--- CORRECCIÓN 3: 10000us = 10ms = 100Hz
    }

    std::cout << "\n>> APAGADO SEGURO..." << std::endl;
    for (auto& c : controllers) c->SetStop();
    usleep(50000); 
    return 0;
}
