#pragma once
#include <atomic>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>

struct SharedTelemetry {
    std::atomic<double> position[14];
    std::atomic<double> velocity[14];
    std::atomic<double> torque[14];
    std::atomic<double> cycle_time_ms{0.0};
};

// Declaramos que existe, pero se instancia en el main
extern SharedTelemetry global_telemetry; 
extern std::atomic<bool> g_running;

inline void MonitorLoop(const std::vector<int>& motor_ids) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    while (g_running) {
        std::cout << "\033[H"; 
        std::cout << "=== ATOM-51 TELEMETRY MONITOR ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2); // Bajé a 2 decimales para leer mejor los grados
        
        for (int id : motor_ids) {
            // Conversión: Vueltas * 360 = Grados
            double pos_deg = global_telemetry.position[id].load(std::memory_order_relaxed) * 360.0;
            
            // Opcional: Si quieres ver la velocidad en grados/segundo, descomenta la siguiente línea:
            // double vel_deg_s = global_telemetry.velocity[id].load(std::memory_order_relaxed) * 360.0;
            double vel_raw = global_telemetry.velocity[id].load(std::memory_order_relaxed);

            std::cout << "ID [" << id << "] Pos: " << std::setw(6) << pos_deg << " deg" 
                      << " | Vel: " << std::setw(6) << vel_raw 
                      << " | Trq: " << std::setw(6) << global_telemetry.torque[id].load(std::memory_order_relaxed) << std::endl;
        }
        std::cout << "Cycle: " << global_telemetry.cycle_time_ms.load(std::memory_order_relaxed) << "ms    " << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
