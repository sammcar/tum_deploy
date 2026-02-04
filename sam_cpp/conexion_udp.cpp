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
#include <iomanip> // Necesario para la tabla
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "moteus.h"

namespace moteus = mjbots::moteus;

// ================= CONFIGURACIÓN =================
// IDs a controlar (Pata BL: 7, 8, 9)
const std::vector<int> kMotorIds = {1,2,3,4,5,6,7,8,9,10,11,12}; 

const double kMaxTorque = 12.0; 
const double kKp = 1.0;   
const double kKd = 1.0; 

// --- PERFILES DE MOVIMIENTO ---
const double kVelLimit_Std   = 0.1;  // rev/s
const double kAccelLimit_Std = 0.5; 

const double kVelLimit_Tibia   = 0.05;  // rev/s (Lento para tibias)
const double kAccelLimit_Tibia = 0.1; 

// --- ESTRUCTURAS ---

// 1. Memoria Compartida
struct SharedData {
    double angles[4][3]; 
    bool emergency_stop;
};

// 2. Telemetría para el Dashboard
struct MotorTelemetry {
    int id;
    double target_deg; // Agregado para ver qué pide Python
    double pos_deg;
    double vel_deg_s;
    double tor_nm;
    double temp_c;
    bool online;
};

// --- CLASE GESTOR DE MEMORIA ---
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

    // Inicializar controladores
    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (int id : kMotorIds) {
        moteus::Controller::Options options;
        options.id = id;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    // Vector para guardar datos del ciclo actual
    std::vector<MotorTelemetry> telemetry_data(kMotorIds.size());

    auto start_time = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double t_sec = std::chrono::duration<double>(now - start_time).count();

        // --- BUCLE DE CONTROL ---
        for (size_t i = 0; i < controllers.size(); ++i) {
            int id = kMotorIds[i]; 
            
            // Lógica de mapeo de memoria
            int memory_index = id - 1; 
            int row = memory_index / 3; 
            int col = memory_index % 3; 

            double target_deg = memory.data->angles[row][col]; 
            bool is_tibia = (id == 3 || id == 6 || id == 9 || id == 12);

            moteus::PositionMode::Command cmd;
            cmd.position = target_deg / 360.0;
            cmd.velocity = 0.0;
            cmd.kp_scale = kKp;
            cmd.kd_scale = kKd;
            cmd.maximum_torque = kMaxTorque;
            cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
            
            if (is_tibia) {
                cmd.velocity_limit = kVelLimit_Tibia; 
                cmd.accel_limit = kAccelLimit_Tibia;
            } else {
                cmd.velocity_limit = kVelLimit_Std;
                cmd.accel_limit = kAccelLimit_Std;
            }

            const auto maybe_result = controllers[i]->SetPosition(cmd);

            // Guardar datos para el Dashboard
            telemetry_data[i].id = id;
            telemetry_data[i].target_deg = target_deg;
            
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

        // --- VISUALIZACIÓN (DASHBOARD) ---
        // Actualizamos cada 100ms (cada 10 ciclos de 10ms)
        static int print_cnt = 0;
        if (print_cnt++ % 10 == 0) {
            std::cout << "\033[2J\033[H"; // Limpiar Pantalla
            
            std::cout << "========= SHM CONTROL MONITOR =========\n";
            std::cout << " Tiempo: " << std::fixed << std::setprecision(2) << t_sec << "s\n";
            std::cout << "---------------------------------------\n";
            // He agregado la columna TARGET para que compares
            std::cout << " ID | TARGET(°) |  POS(°)  |  VEL(d/s) | TOR(Nm) | TEMP\n";
            std::cout << "----|-----------|----------|-----------|---------|-----\n";

            for (const auto& data : telemetry_data) {
                if (data.online) {
                    // Si el error es grande (> 5 grados), pintamos la posición en rojo
                    double error = std::abs(data.target_deg - data.pos_deg);
                    std::string color = (error > 5.0) ? "\033[1;31m" : "\033[0m";

                    std::cout << " " << std::setw(2) << data.id << " | "
                              << std::setw(9) << std::fixed << std::setprecision(1) << data.target_deg << " | "
                              << color << std::setw(8) << data.pos_deg << "\033[0m | "
                              << std::setw(9) << std::setprecision(1) << data.vel_deg_s << " | "
                              << std::setw(7) << std::setprecision(2) << data.tor_nm << " | "
                              << std::setw(4) << std::setprecision(1) << data.temp_c << "\n";
                } else {
                    std::cout << " " << std::setw(2) << data.id << " | "
                              << std::setw(9) << data.target_deg << " | "
                              << "  OFFLINE |    ---    |   ---   |  ---\n";
                }
            }
            std::cout << "---------------------------------------\n";
            std::cout << " [Ctrl+C] para detener.\n";
            std::cout << std::flush;
        }

        ::usleep(10000); // 100Hz
    }

    std::cout << "\n\nDeteniendo..." << std::endl;
    for (auto& controller : controllers) controller->SetStop();
    return 0;
}
