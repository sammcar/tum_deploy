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
#include <thread> // Necesario para sleep_until

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

const MotorConfig kCoxaConfig =  {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 10.0, .accel_limit = 20.0};
const MotorConfig kFemurConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 10.0, .accel_limit = 20.0};
const MotorConfig kTibiaConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 10.0, .accel_limit = 20.0};

const std::vector<int> kMotorIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

struct SharedData {
    double angles[4][3];      // Posición [Grados]
    double velocities[4][3];  // Velocidad [Grados/s]
    bool is_walking;      
};

struct SafetyState {
    double current_pos = 0.0;
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
    std::vector<SafetyState> motor_states(13); 

    for (int id : kMotorIds) {
        moteus::Controller::Options options;
        options.id = id;
        options.transport = transport;
        options.position_format.position = moteus::kFloat; 
        options.position_format.velocity = moteus::kFloat; 
        options.position_format.kp_scale = moteus::kInt16;
        options.position_format.kd_scale = moteus::kInt16;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::cout << "\033[2J"; 

    // 1. LECTURA INICIAL
    std::vector<moteus::CanFdFrame> initial_receive;
    std::vector<moteus::CanFdFrame> initial_send;
    for (auto& c : controllers) initial_send.push_back(c->MakePosition({}));
    SafeTransportCycle(transport, initial_send, &initial_receive);

    for (const auto& frame : initial_receive) {
        if (frame.source >= 1 && frame.source <= 12) {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            motor_states[frame.source].current_pos = res.position;
        }
    }

    // 2. HOMING INICIAL (3 segundos)
    auto homing_start = std::chrono::steady_clock::now();
    double homing_duration = 3.0;
    std::cout << ">> EJECUTANDO HOMING INICIAL A 0.0..." << std::endl;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - homing_start).count();
        if (elapsed > homing_duration) break;

        std::vector<moteus::CanFdFrame> frames;
        for (auto& c : controllers) {
            int id = c->options().id;
            double pos = CalculateMinimumJerk(elapsed, homing_duration, motor_states[id].current_pos, 0.0);
            
            moteus::PositionMode::Command cmd;
            cmd.position = pos;
            cmd.kp_scale = 0.5; cmd.kd_scale = 0.5; cmd.maximum_torque = 5.0;
            frames.push_back(c->MakePosition(cmd));
        }
        SafeTransportCycle(transport, frames, nullptr);
        usleep(10000); 
    }

    for (int id : kMotorIds) motor_states[id].current_pos = 0.0;

    // 3. BUCLE PRINCIPAL CON CONTROL DE TIEMPO DE ALTA PRECISIÓN
    std::cout << ">> SISTEMA LISTO. ESPERANDO IS_WALKING..." << std::endl;
    
    auto safety_home_start = std::chrono::steady_clock::now();
    bool was_walking = false; 

    // Configuración del metrónomo (200Hz = 5000 microsegundos)
    const auto period = std::chrono::microseconds(5000);
    auto next_cycle = std::chrono::steady_clock::now();

    while (g_running) {
        // Establecer el momento exacto en el que debe despertar el próximo ciclo
        next_cycle += period;

        bool is_walking_mode = memory.data->is_walking;
        std::vector<moteus::CanFdFrame> send_frames;

        if (was_walking && !is_walking_mode) {
            std::cout << "\n>> PARADA DETECTADA: Regresando a Home suavemente..." << std::endl;
            safety_home_start = std::chrono::steady_clock::now();
        }
        was_walking = is_walking_mode;

        for (size_t i = 0; i < controllers.size(); ++i) {
            int id = kMotorIds[i];
            const MotorConfig* cfg = GetMotorConfig(id);
            int row = (id - 1) / 3;
            int col = (id - 1) % 3;

            double cmd_pos_rev = 0.0;
            double cmd_vel_rev_s = 0.0;

            if (is_walking_mode) {
                double target_deg = GetCalibratedTarget(id, memory.data->angles[row][col]);
                double target_vel_deg = GetCalibratedTarget(id, memory.data->velocities[row][col]);
                
                cmd_pos_rev = target_deg / 360.0;
                cmd_vel_rev_s = target_vel_deg / 360.0;
                motor_states[id].current_pos = cmd_pos_rev;
            } else {
                if (std::abs(motor_states[id].current_pos) > 0.0001) {
                    auto now_safety = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now_safety - safety_home_start).count();
                    double duration = 2.0;

                    cmd_pos_rev = CalculateMinimumJerk(elapsed, duration, motor_states[id].current_pos, 0.0);
                    if (elapsed > duration) motor_states[id].current_pos = 0.0;
                } else {
                    cmd_pos_rev = 0.0;
                }
                cmd_vel_rev_s = 0.0;
            }

            moteus::PositionMode::Command cmd;
            cmd.position = cmd_pos_rev;
            cmd.velocity = cmd_vel_rev_s;
            cmd.kp_scale = (id == 10 || id == 11) ? 0.8 : 1.0;
            cmd.kd_scale = (id == 10 || id == 11) ? 0.8 : 1.0;
            cmd.velocity_limit = cfg->vel_limit;
            cmd.accel_limit = cfg->accel_limit;
            cmd.maximum_torque = cfg->max_torque;
            
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        SafeTransportCycle(transport, send_frames, nullptr);

        // Control de salud del ciclo: Si nos pasamos del tiempo, resincronizar
        auto now = std::chrono::steady_clock::now();
        if (now > next_cycle) {
            next_cycle = now; // Saltamos la espera para intentar recuperar el tiempo
        } else {
            // Dormir con precisión hasta el inicio del siguiente periodo de 5ms
            std::this_thread::sleep_until(next_cycle);
        }
    }

    for (auto& c : controllers) c->SetStop();
    return 0;
}
