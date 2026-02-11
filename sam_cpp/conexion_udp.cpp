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
const double kSafetyAccLimit = 40.0;

struct MotorConfig {
    double kp;
    double kd;
    double max_torque;
    double vel_limit;
    double accel_limit;
};

const MotorConfig kCoxaConfig =  {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 5.0, .accel_limit = 10.0};
const MotorConfig kFemurConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 5.0, .accel_limit = 10.0};
const MotorConfig kTibiaConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 6.0, .vel_limit = 5.0, .accel_limit = 10.0};

const std::vector<int> kMotorIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

struct SharedData {
    double angles[4][3];
    double times[4][3];
    bool is_walking; 
};

struct TrajectoryState {
    double current_cmd_pos = 0.0;  // Posición comandada teórica
    double start_pos_rev = 0.0;
    double target_pos_rev = 0.0;
    double start_time_s = 0.0;
    double duration_s = 0.0;
    double theoretical_vel = 0.0;  // Velocidad calculada teóricamente
    bool is_moving = false;
};

// ============================================================
//    GESTOR DE MEMORIA COMPARTIDA
// ============================================================
// ============================================================
//    GESTOR DE MEMORIA COMPARTIDA (ACTUALIZADO)
// ============================================================
class MemoryManager {
public:
    const char *shm_name = "/rex_shm"; // Nota: Se recomienda empezar con '/'
    int shm_fd = -1;
    void *ptr = MAP_FAILED;
    SharedData *data = nullptr;

    MemoryManager() {
        // 1. Intentar abrir o crear la memoria compartida
        // O_CREAT: La crea si no existe.
        // O_RDWR: Lectura y escritura.
        shm_fd = shm_open(shm_name, O_RDWR | O_CREAT, 0666);
        
        if (shm_fd == -1) {
            std::cerr << "❌ Error crítico: No se pudo abrir/crear SHM: " << strerror(errno) << std::endl;
            return;
        }

        // 2. Verificar el tamaño actual de la memoria
        struct stat shm_stat;
        if (fstat(shm_fd, &shm_stat) == -1) {
            std::cerr << "❌ Error al verificar tamaño de SHM" << std::endl;
            return;
        }

        // 3. Si el tamaño es 0 (acaba de ser creada), ajustamos el tamaño
        bool is_new = (shm_stat.st_size == 0);
        if (is_new) {
            if (ftruncate(shm_fd, sizeof(SharedData)) == -1) {
                std::cerr << "❌ Error al definir tamaño de SHM" << std::endl;
                return;
            }
        }

        // 4. Mapear el bloque a la memoria del proceso
        ptr = mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        
        if (ptr == MAP_FAILED) {
            std::cerr << "❌ Error al mapear SHM (mmap)" << std::endl;
            return;
        }

        data = static_cast<SharedData *>(ptr);

        // 5. Inicializar en 0.0 si es nueva o si queremos asegurar un estado limpio
        if (is_new) {
            std::cout << "✨ SHM no existía. Inicializando datos en 0.0..." << std::endl;
            std::memset(data, 0, sizeof(SharedData));
        } else {
            std::cout << "✅ SHM encontrada y conectada." << std::endl;
        }
    }

    ~MemoryManager() {
        if (ptr != MAP_FAILED) munmap(ptr, sizeof(SharedData));
        if (shm_fd != -1) close(shm_fd);
    }
    
    bool is_valid() { return data != nullptr; }
};

// ============================================================
//    MATEMÁTICAS: TRAYECTORIAS
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
//    TELEMETRÍA (SOLO DEBUGGING)
// ============================================================

void PrintTelemetry(const std::vector<moteus::CanFdFrame>& receive_frames, bool is_walking, std::string status_msg) {
    static int counter = 0;
    if (counter++ % 10 != 0) return; 

    struct MotorInfo { bool online = false; double pos = 0.0; double torque = 0.0; };
    MotorInfo table[13];

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
    std::cout << " MODO: " << (is_walking ? "\033[1;32mWALKING (THEORETICAL VEL)\033[0m" : "\033[1;34mSTATIC (S-CURVE)\033[0m") << "\n";
    std::cout << "==================================================================\n";
    std::cout << " ID | POS_REAL (deg) | TORQUE (Nm) | STATUS | CONFIG (Kp/Kd) \n";
    std::cout << "----|----------------|-------------|--------|----------------\n";

    for (int id : kMotorIds) {
        double show_kp = (id == 10 || id == 11) ? 0.8 : GetMotorConfig(id)->kp;
        double show_kd = (id == 10 || id == 11) ? 0.8 : GetMotorConfig(id)->kd;

        if (table[id].online) {
            printf(" %2d | %14.2f | %11.2f |   OK   |  %.1f / %.1f \n", 
                    id, table[id].pos, table[id].torque, show_kp, show_kd);
        } else {
            printf(" %2d |     --.--      |    --.--    | \033[1;31mOFFLINE\033[0m |  %.1f / %.1f \n", 
                    id, show_kp, show_kd);
        }
        if (id % 3 == 0 && id < 12) std::cout << "----|----------------|-------------|--------|----------------\n";
    }
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
           // 2. (Opcional) Si quieres limitar la fuerza máxima dinámicamente
        options.position_format.maximum_torque = moteus::kFloat;
        options.transport = transport;
        options.position_format.kp_scale = moteus::kFloat;
        options.position_format.kd_scale = moteus::kFloat;
        options.position_format.velocity_limit = moteus::kFloat;
        options.position_format.accel_limit = moteus::kFloat;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::cout << "\033[2J"; 

    // --- HOMING SEGURA ---
    std::cout << ">> LEYENDO POSICIONES INICIALES PARA HOMING..." << std::endl;
    std::vector<moteus::CanFdFrame> initial_receive;
    std::vector<moteus::CanFdFrame> initial_send;
    for (auto& c : controllers) initial_send.push_back(c->MakePosition({}));
    SafeTransportCycle(transport, initial_send, &initial_receive);

    for (const auto& frame : initial_receive) {
        if (frame.source >= 1 && frame.source <= 12) {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            motor_states[frame.source].current_cmd_pos = res.position;
            motor_states[frame.source].target_pos_rev = res.position;
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
            double pos = CalculateMinimumJerk(elapsed, startup_duration, motor_states[id].current_cmd_pos, 0.0);
            double vel = CalculateMinimumJerkVelocity(elapsed, startup_duration, motor_states[id].current_cmd_pos, 0.0);
            
            moteus::PositionMode::Command cmd;
            cmd.position = pos; cmd.velocity = vel;
            cmd.velocity_limit = cfg->vel_limit; cmd.accel_limit = cfg->accel_limit;
            startup_frames.push_back(c->MakePosition(cmd));
        }
        std::vector<moteus::CanFdFrame> rec;
        SafeTransportCycle(transport, startup_frames, &rec);
        PrintTelemetry(rec, false, "\033[1;33mHOMING\033[0m");
        usleep(10000);
    }
    // --- INSERTAR ESTO JUSTO DESPUÉS DE SALIR DEL BUCLE DE HOMING ---
    for (int id : kMotorIds) {
        motor_states[id].current_cmd_pos = 0.0; // Estamos físicamente en 0
        motor_states[id].start_pos_rev = 0.0;   // El nuevo inicio es 0
        motor_states[id].target_pos_rev = 0.0;  // El objetivo actual es 0
        motor_states[id].is_moving = false;     // Detener cualquier trayectoria previa
    }
    // --- BUCLE PRINCIPAL ---
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
            bool high_safety = false;
            
        if (is_walking_mode) {
            // 1. DETECTAR NUEVO OBJETIVO DESDE SHM
            if (std::abs(target_rev - s.target_pos_rev) > 0.00001) {
                // Guardamos de dónde venimos y a dónde vamos
                s.start_pos_rev = s.current_cmd_pos; // IMPORTANTE: empezar desde donde estamos REALMENTE ahora
                s.target_pos_rev = target_rev;
                
                // Ajuste de tiempo para evitar divisiones por cero
                s.duration_s = (move_time < 0.001) ? 0.02 : move_time;
                s.start_time_s = t_global_sec;
                s.is_moving = true;

                // Calculamos la velocidad necesaria para cubrir la distancia en ese tiempo
                s.theoretical_vel = (s.target_pos_rev - s.start_pos_rev) / s.duration_s;
            }

            // 2. EJECUTAR EL MOVIMIENTO FLUIDO
            if (s.is_moving) {
                double t_elap = t_global_sec - s.start_time_s;

                if (t_elap >= s.duration_s) {
                    // Llegamos al objetivo
                    cmd_pos = s.target_pos_rev;
                    cmd_vel = 0.0;
                    s.is_moving = false;
                } else {
                    // Avanzamos un paso pequeño proporcional al tiempo transcurrido
                    cmd_pos = s.start_pos_rev + s.theoretical_vel * t_elap;
                    cmd_vel = s.theoretical_vel;
                }
            } else {
                // Mantener posición si no hay movimiento nuevo
                cmd_pos = s.target_pos_rev;
                cmd_vel = 0.0;
            }
        }

            // Actualizamos la posición comandada teórica (sin usar feedback real)
            s.current_cmd_pos = cmd_pos;

            moteus::PositionMode::Command cmd;
            cmd.position = cmd_pos;
            cmd.velocity = cmd_vel; // Este es el valor clave para la fluidez
            cmd.kp_scale = (id == 10 || id == 11) ? 0.8 : cfg->kp;
            cmd.kd_scale = (id == 10 || id == 11) ? 0.8 : cfg->kd;
            cmd.velocity_limit = high_safety ? kSafetyVelLimit : cfg->vel_limit;
            cmd.accel_limit = high_safety ? kSafetyAccLimit : cfg->accel_limit;
            cmd.maximum_torque = cfg->max_torque;
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        std::vector<moteus::CanFdFrame> receive_frames;
        SafeTransportCycle(transport, send_frames, &receive_frames);
        
        // La telemetría lee los frames de respuesta, pero estos NO afectan los cálculos de arriba
        PrintTelemetry(receive_frames, is_walking_mode, "\033[1;32mRUNNING\033[0m");
        
        usleep(10000); // Bucle a ~100Hz
    }

    std::cout << "\n>> APAGADO SEGURO: MOTORES EN IDLE..." << std::endl;
    for (auto& c : controllers) c->SetStop();
    usleep(50000); 
    return 0;
}
