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

const double kSafetyVelLimit = 30.0;  
const double kSafetyAccLimit = 150.0; 

struct MotorConfig {
    double kp;
    double kd;
    double max_torque;
    double vel_limit;
    double accel_limit;
};

const MotorConfig kCoxaConfig =  {.kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0};
const MotorConfig kFemurConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0};
const MotorConfig kTibiaConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0};

const std::vector<int> kMotorIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

struct SharedData {
    double angles[4][3];
    double times[4][3];
    bool is_walking; 
};

struct TrajectoryState {
    double current_pos_rev = 0.0;
    double start_pos_rev = 0.0;
    double target_pos_rev = 0.0;
    double start_time_s = 0.0;
    double duration_s = 0.0;
    bool is_moving = false;
};

// ============================================================
//    GESTOR DE MEMORIA COMPARTIDA
// ============================================================
class MemoryManager {
public:
    const char *shm_name = "rex_shm";
    int shm_fd = -1;
    void *ptr = MAP_FAILED;
    SharedData *data = nullptr;

    MemoryManager() {
        shm_fd = shm_open(shm_name, O_RDWR, 0666);
        if (shm_fd != -1) {
            ptr = mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
            data = static_cast<SharedData *>(ptr);
        }
    }
    ~MemoryManager() {
        if (ptr != MAP_FAILED) munmap(ptr, sizeof(SharedData));
        if (shm_fd != -1) close(shm_fd);
    }
    bool is_valid() { return data != nullptr; }
};

// ============================================================
//    MATEMÁTICAS: JERK LIMITADO (CURVA S)
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
//    VISUALIZACIÓN ORDENADA (IDs FIJOS)
// ============================================================

void PrintTelemetry(const std::vector<moteus::CanFdFrame>& receive_frames, bool is_walking, std::string status_msg) {
    static int counter = 0;
    if (counter++ % 10 != 0) return; 

    // Estructura para organizar los datos por ID
    struct MotorInfo {
        bool online = false;
        double pos = 0.0;
        double torque = 0.0;
    };
    MotorInfo table[13]; // Índices 1-12

    // Mapear los frames recibidos a su posición fija en la tabla
    for (const auto& frame : receive_frames) {
        if (frame.source >= 1 && frame.source <= 12) {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            table[frame.source].online = true;
            table[frame.source].pos = res.position * 360.0;
            table[frame.source].torque = res.torque;
        }
    }

    std::cout << "\033[H"; 
    std::cout << "==================================================================\n";
    std::cout << " STATUS: " << status_msg << "\n";
    std::cout << " MODO: " << (is_walking ? "\033[1;32mWALKING\033[0m" : "\033[1;34mSTATIC (S-CURVE)\033[0m") << "\n";
    std::cout << "==================================================================\n";
    std::cout << " ID | POS (deg) | TORQUE (Nm) | STATUS | CONFIG (Kp/Kd) \n";
    std::cout << "----|-----------|-------------|--------|----------------\n";

    // Iterar siempre del 1 al 12 para mantener el orden visual
    for (int id : kMotorIds) {
        double show_kp = (id == 10 || id == 11) ? 0.8 : GetMotorConfig(id)->kp;
        double show_kd = (id == 10 || id == 11) ? 0.8 : GetMotorConfig(id)->kd;

        if (table[id].online) {
            printf(" %2d | %9.2f | %11.2f |   OK   |  %.1f / %.1f \n", 
                    id, table[id].pos, table[id].torque, show_kp, show_kd);
        } else {
            printf(" %2d |   --.--   |    --.--    | \033[1;31mOFFLINE\033[0m |  %.1f / %.1f \n", 
                    id, show_kp, show_kd);
        }
        if (id % 3 == 0 && id < 12) std::cout << "----|-----------|-------------|--------|----------------\n";
    }
    std::cout << "==================================================================\n" << std::flush;
}



void SafeTransportCycle(std::shared_ptr<moteus::Transport> transport, 
                        const std::vector<moteus::CanFdFrame>& send_frames, 
                        std::vector<moteus::CanFdFrame>* receive_frames) {
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

    // ------------------------------------------------------------
    // INICIO SEGURO (Homing) con LÍMITES RESTRICTIVOS
    // ------------------------------------------------------------
    std::cout << ">> INICIANDO TRANSICIÓN SEGURA (LÍMITES MOTOR) ..." << std::endl;
    std::vector<moteus::CanFdFrame> initial_receive;
    std::vector<moteus::CanFdFrame> initial_send;
    for (auto& c : controllers) initial_send.push_back(c->MakePosition({}));
    SafeTransportCycle(transport, initial_send, &initial_receive);

    for (const auto& frame : initial_receive) {
        if (frame.source >= 1 && frame.source <= 12) {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            motor_states[frame.source].current_pos_rev = res.position;
        }
    }

    auto startup_start = std::chrono::steady_clock::now();
    double startup_duration = 3.0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - startup_start).count();
        if (elapsed > startup_duration) break;

        std::vector<moteus::CanFdFrame> startup_frames;
        for (auto& c : controllers) {
            int id = c->options().id;
            const MotorConfig* cfg = GetMotorConfig(id);
            double pos = CalculateMinimumJerk(elapsed, startup_duration, motor_states[id].current_pos_rev, 0.0);
            double vel = CalculateMinimumJerkVelocity(elapsed, startup_duration, motor_states[id].current_pos_rev, 0.0);
            
            moteus::PositionMode::Command cmd;
            cmd.position = pos;
            cmd.velocity = vel;
            cmd.velocity_limit = cfg->vel_limit; // Límite real del motor para homing
            cmd.accel_limit = cfg->accel_limit;
            startup_frames.push_back(c->MakePosition(cmd));
        }
        std::vector<moteus::CanFdFrame> rec;
        SafeTransportCycle(transport, startup_frames, &rec);
        PrintTelemetry(rec, false, "\033[1;33mHOMING (RESTRICTED)\033[0m");
        usleep(10000);
    }

    // ------------------------------------------------------------
    // BUCLE PRINCIPAL (Control por Memoria con LÍMITES TECHO)
    // ------------------------------------------------------------
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

            double cmd_pos, cmd_vel;
            bool high_safety = false;

            if (is_walking_mode) {
                cmd_pos = target_rev;
                cmd_vel = 0.0;
                motor_states[id].is_moving = false;
                motor_states[id].target_pos_rev = target_rev;
            } else {
                TrajectoryState &s = motor_states[id];
                if (std::abs(target_rev - s.target_pos_rev) > 0.0001) {
                    s.start_pos_rev = s.current_pos_rev;
                    s.target_pos_rev = target_rev;
                    s.duration_s = (move_time < 0.01) ? 1.0 : move_time;
                    s.start_time_s = t_global_sec;
                    s.is_moving = true;
                }

                if (s.is_moving) {
                    double t_elap = t_global_sec - s.start_time_s;
                    cmd_pos = CalculateMinimumJerk(t_elap, s.duration_s, s.start_pos_rev, s.target_pos_rev);
                    cmd_vel = CalculateMinimumJerkVelocity(t_elap, s.duration_s, s.start_pos_rev, s.target_pos_rev);
                    high_safety = true; 
                    if (t_elap >= s.duration_s) s.is_moving = false;
                } else {
                    cmd_pos = s.target_pos_rev;
                    cmd_vel = 0.0;
                }
            }
            motor_states[id].current_pos_rev = cmd_pos;

            moteus::PositionMode::Command cmd;
            cmd.position = cmd_pos;
            cmd.velocity = cmd_vel;
            cmd.kp_scale = (id == 10 || id == 11) ? 0.8 : cfg->kp;
            cmd.kd_scale = (id == 10 || id == 11) ? 0.8 : cfg->kd;
            
            cmd.velocity_limit = high_safety ? kSafetyVelLimit : cfg->vel_limit;
            cmd.accel_limit = high_safety ? kSafetyAccLimit : cfg->accel_limit;
            cmd.maximum_torque = cfg->max_torque;
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        std::vector<moteus::CanFdFrame> receive_frames;
        SafeTransportCycle(transport, send_frames, &receive_frames);
        PrintTelemetry(receive_frames, is_walking_mode, "\033[1;32mRUNNING (ACTIVE)\033[0m");
        usleep(10000); 
    }

    // ------------------------------------------------------------
    // APAGADO SEGURO
    // ------------------------------------------------------------
    std::cout << "\n>> APAGADO SEGURO: PONIENDO MOTORES EN IDLE..." << std::endl;
    for (auto& c : controllers) {
        c->SetStop(); 
    }
    usleep(50000); 
    std::cout << ">> SISTEMA DETENIDO CORRECTAMENTE." << std::endl;

    return 0;
}
