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
#include <thread>
#include "moteus.h"

namespace moteus = mjbots::moteus;

// ============================================================
//    CONSTANTES FÍSICAS (NUEVO)
// ============================================================
// Ajusta estas longitudes (en metros) a las reales de tu robot
const double L1 = 0.093; 
const double L2 = 0.147; 
const double L3 = 0.230;

double CHASIS_MASS = 5.65;
double LEG_MASS = 1.46; // Masa de una pata completa
double TOTAL_ROBOT_MASS = CHASIS_MASS + LEG_MASS * 4;

// ============================================================
//    FUNCIÓN MATEMÁTICA DE TORQUES (JACOBIANA)
// ============================================================
struct LegTorques {
    double t1_abad;
    double t2_hip;
    double t3_knee;
};

LegTorques ComputeTorques(double q1, double q2, double q3, 
                          double fx, double fy, double fz,
                          double l1, double l2, double l3, 
                          bool es_pata_derecha) {
    
    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s2 = std::sin(q2), c2 = std::cos(q2);
    
    // Simplificación trigonométrica
    double s23 = std::sin(q2 + q3);
    double c23 = std::cos(q2 + q3);
    
    double side = es_pata_derecha ? -1.0 : 1.0; 

    // --- MATRIZ JACOBIANA (Analítica) ---
    double j11 = 0;
    double j12 = l2*c2 + l3*c23;
    double j13 = l3*c23;
    
    double j21 = -l1*side*s1 + (l2*c2 + l3*c23)*c1;
    double j22 = -(l2*s2 + l3*s23)*s1;
    double j23 = -l3*s23*s1; 
    
    double j31 = l1*side*c1 + (l2*c2 + l3*c23)*s1;
    double j32 = (l2*s2 + l3*s23)*c1;
    double j33 = l3*s23*c1;

    // --- CÁLCULO FINAL (Tau = J_transpuesta * F) ---
    LegTorques tau;
    tau.t1_abad = j11*fx + j21*fy + j31*fz;
    tau.t2_hip  = j12*fx + j22*fy + j32*fz;
    tau.t3_knee = j13*fx + j23*fy + j33*fz;

    return tau;
}

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

// ============================================================
//    AJUSTES MECÁNICOS
// ============================================================
const double kFactorMultiplicador = 1.00;
const double kFactorMultiplicador2 = 1.00;

struct MotorConfig {
    double kp;
    double kd;
    double max_torque;
    double vel_limit;
    double accel_limit;
};

// Configuración sugerida para dinámica (KP más bajos, Torque alto disponible)
const MotorConfig kCoxaConfig =  {.kp = 1.0, .kd = 1.0, .max_torque = 4.0, .vel_limit = 20.0, .accel_limit = 50.0};
const MotorConfig kFemurConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 4.0, .vel_limit = 20.0, .accel_limit = 50.0};
const MotorConfig kTibiaConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 4.0, .vel_limit = 20.0, .accel_limit = 50.0};

const std::vector<int> kMotorIds = {1, 2, 3};

// ============================================================
//    MEMORIA COMPARTIDA (MODIFICADA)
// ============================================================
struct SharedData {
    double angles[4][3];        // Posición deseada [Grados]
    double velocities[4][3];    // Velocidad deseada [Grados/s]
    double desired_accel[4][3]; // NUEVO: Aceleración cartesiana deseada [m/s^2] (X, Y, Z)
    double kp_scale[4][3];      // Ganancia KP dinámica
    double kd_scale[4][3];      // Ganancia KD dinámica
    bool is_stance[4];          // NUEVO: Estado de la pata (true=Suelo, false=Aire)
    bool is_walking;      
};

struct SafetyState {
    double current_pos = 0.0;
};

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
        options.position_format.accel_limit = moteus::kFloat;
        options.position_format.maximum_torque = moteus::kFloat;
        options.position_format.feedforward_torque = moteus::kFloat; // IMPORTANTE: Habilitar esto
        options.position_format.kp_scale = moteus::kFloat; // Cambiado a kFloat para mayor precisión
        options.position_format.kd_scale = moteus::kFloat;
        
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::cout << "\033[2J"; 

    // --- 1. LECTURA INICIAL ---
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

    // --- 2. HOMING INICIAL ---
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
        usleep(100); 
    }

    for (int id : kMotorIds) motor_states[id].current_pos = 0.0;

    // --- 3. BUCLE DE CONTROL DINÁMICO ---
    std::cout << ">> SISTEMA LISTO. CONTROL DINÁMICO ACTIVADO." << std::endl;
    
    auto safety_home_start = std::chrono::steady_clock::now();
    bool was_walking = false; 
    const auto period = std::chrono::microseconds(5000); // 200Hz
    auto next_cycle = std::chrono::steady_clock::now();

    while (g_running) {
        next_cycle += period;
        bool is_walking_mode = memory.data->is_walking;
        std::vector<moteus::CanFdFrame> send_frames;

        if (was_walking && !is_walking_mode) {
            std::cout << "\n>> PARADA DETECTADA: Regresando a Home..." << std::endl;
            safety_home_start = std::chrono::steady_clock::now();
        }
        was_walking = is_walking_mode;

        for (size_t i = 0; i < controllers.size(); ++i) {
            int id = kMotorIds[i];
            const MotorConfig* cfg = GetMotorConfig(id);
            int row = (id - 1) / 3; // Pata (0-3)
            int col = (id - 1) % 3; // Articulación (0-2)

            double cmd_pos_rev = 0.0;
            double cmd_vel_rev_s = 0.0;
            double cmd_torque_nm = 0.0;

            if (is_walking_mode) {
                // A. Obtener Cinemática Deseada
                double target_deg = GetCalibratedTarget(id, memory.data->angles[row][col]);
                double target_vel_deg = GetCalibratedTarget(id, memory.data->velocities[row][col]);
                
                cmd_pos_rev = target_deg / 360.0;
                cmd_vel_rev_s = target_vel_deg / 360.0;
                motor_states[id].current_pos = cmd_pos_rev;

                // ============================================================
                // B. CÁLCULO DE DINÁMICA (NUEVO)
                // ============================================================
                
                // 1. Determinar Masa y Fase
                bool stance = memory.data->is_stance[row];
                double mass_active = stance ? (TOTAL_ROBOT_MASS / 4.0) : LEG_MASS;

                // 2. Obtener Aceleración Deseada (Debe venir del SharedData)
                double ax = memory.data->desired_accel[row][0];
                double ay = memory.data->desired_accel[row][1];
                double az = memory.data->desired_accel[row][2];

                // 3. Calcular Fuerzas Físicas (F = m * (a + g))
                // Nota: Asumimos gravedad = 9.81 en Z positivo para compensarla
                double fz_force = mass_active * (9.81 + az);
                double fx_force = mass_active * ax;
                double fy_force = mass_active * ay;

                // 4. Obtener ángulos actuales de la pata para la Jacobiana (En Radianes)
                double q1 = memory.data->angles[row][0] * (M_PI / 180.0);
                double q2 = memory.data->angles[row][1] * (M_PI / 180.0);
                double q3 = memory.data->angles[row][2] * (M_PI / 180.0);

                // 5. Calcular Torques usando Jacobiana Transpuesta
                bool is_right_leg = (row == 1 || row == 3); // 0=FL, 1=FR, 2=BL, 3=BR (Ajustar según tu mapa)
                
                LegTorques leg_tau = ComputeTorques(q1, q2, q3, fx_force, fy_force, fz_force, L1, L2, L3, is_right_leg);

                // 6. Seleccionar torque para ESTE motor
                if (col == 0) cmd_torque_nm = leg_tau.t1_abad;
                else if (col == 1) cmd_torque_nm = leg_tau.t2_hip;
                else if (col == 2) cmd_torque_nm = leg_tau.t3_knee;

                // Opcional: Aplicar reducción de engranajes si existe
                // cmd_torque_nm = cmd_torque_nm / GEAR_RATIO;

            } else {
                // Modo Seguridad / Home
                cmd_torque_nm = 0.0; // Sin FF en home
                if (std::abs(motor_states[id].current_pos) > 0.0001) {
                    auto now_safety = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now_safety - safety_home_start).count();
                    cmd_pos_rev = CalculateMinimumJerk(elapsed, 2.0, motor_states[id].current_pos, 0.0);
                    if (elapsed > 2.0) motor_states[id].current_pos = 0.0;
                } else {
                    cmd_pos_rev = 0.0;
                }
                cmd_vel_rev_s = 0.0;
            }

            moteus::PositionMode::Command cmd;
            cmd.position = cmd_pos_rev;
            cmd.velocity = cmd_vel_rev_s;
            
            // --- ENVÍO DE PARÁMETROS DINÁMICOS ---
            cmd.feedforward_torque = cmd_torque_nm; 
            
            if (is_walking_mode) {
                // Usar ganancias variables si están disponibles en SharedData
                cmd.kp_scale = memory.data->kp_scale[row][col];
                cmd.kd_scale = memory.data->kd_scale[row][col];
            } else {
                cmd.kp_scale = 1.0;
                cmd.kd_scale = 1.0;
            }

            cmd.velocity_limit = cfg->vel_limit;
            cmd.accel_limit = cfg->accel_limit;
            cmd.maximum_torque = cfg->max_torque;
            
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        SafeTransportCycle(transport, send_frames, nullptr);

        auto now = std::chrono::steady_clock::now();
        if (now > next_cycle) {
            next_cycle = now; 
        } else {
            std::this_thread::sleep_until(next_cycle);
        }
    }

    for (auto& c : controllers) c->SetStop();
    return 0;
}
