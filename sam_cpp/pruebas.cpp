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

// --- CONFIGURACIÓN DE MOTORES ---
const std::vector<int> kMotorIds = {7, 8, 9}; 

const double kMaxTorque = 3.0; 
const double kKp = 1.0; 
const double kKd = 1.0; 

// --- LÍMITES DE SEGURIDAD ---
const double kVelLimit   = 15.0;   
const double kAccelLimit = 12.0;   

// --- ESTRUCTURA DE MEMORIA ---
struct SharedData {
    double angles[4][3]; 
    bool emergency_stop; 
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
            std::cout << "⚠️ Memoria no encontrada. CREANDO nueva memoria..." << std::endl;
            shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
            if (shm_fd == -1) {
                perror("❌ Error fatal creando SHM");
                return;
            }
            if (ftruncate(shm_fd, sizeof(SharedData)) == -1) {
                perror("❌ Error en ftruncate");
                return;
            }
            is_creator = true;
        } else {
            std::cout << "✅ Memoria encontrada. Conectando..." << std::endl;
        }

        ptr = mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (ptr == MAP_FAILED) {
            perror("❌ Error en mmap");
            return;
        }
        data = static_cast<SharedData*>(ptr);

        if (is_creator) {
            std::memset(ptr, 0, sizeof(SharedData));
            std::cout << "🧹 Memoria inicializada a 0.0" << std::endl;
        }
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
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::cout << "--- MONITOR DE ÁNGULOS (Target vs Real) ---" << std::endl;
    std::cout << "Formato: ID [Objetivo -> Real]" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();

    // Contador para controlar la frecuencia de impresión
    int cycle_count = 0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;
        double t_sec = elapsed.count();
        
        // Stringstream para construir la línea de texto
        std::stringstream status_line;

        // Solo preparamos el texto si toca imprimir (Optimización)
        bool should_print = (cycle_count % 50 == 0); // 50 ciclos = ~0.5 segundos

        if (should_print) {
            status_line << std::fixed << std::setprecision(1) << "T:" << t_sec << "s | ";
        }

        for (size_t i = 0; i < controllers.size(); ++i) {
            int id = kMotorIds[i];
            double target_deg = memory.data->angles[0][i]; 

            moteus::PositionMode::Command cmd;
            cmd.position = target_deg / 360.0; 
            cmd.velocity = 0.0;
            cmd.kp_scale = kKp;
            cmd.kd_scale = kKd;
            cmd.maximum_torque = kMaxTorque;
            cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
            cmd.velocity_limit = kVelLimit / 360.0; 
            cmd.accel_limit = kAccelLimit / 360.0;

            const auto maybe_result = controllers[i]->SetPosition(cmd);

            // Solo guardamos datos para imprimir si toca imprimir
            if (maybe_result && should_print) {
                double real_pos = maybe_result->values.position * 360.0;
                
                // Formato limpio: M7[ 10.0 ->  9.8]
                status_line << "M" << id << "[" 
                            << std::setw(5) << std::fixed << std::setprecision(2) << target_deg 
                            << "->" 
                            << std::setw(5) << std::fixed << std::setprecision(2) << real_pos 
                            << "] ";
            }
        }

        // --- IMPRESIÓN PERIÓDICA ---
        if (should_print) {
            // Usa "\r" para sobreescribir la línea (Dashboard)
            // Usa "\n" si quieres que baje línea por línea (Historial)
            std::cout << "\r" << status_line.str() << "      " << std::flush;
        }

        cycle_count++;
        ::usleep(10000); // 100Hz
    }

    std::cout << "\n\nDeteniendo..." << std::endl;
    for (auto& controller : controllers) controller->SetStop();
    return 0;
}
