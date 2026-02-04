#include <iostream>
#include <vector>
#include <memory>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <csignal>
#include <atomic>
#include <algorithm>
#include <iomanip>

#include "moteus.h"

namespace moteus = mjbots::moteus;

// ================= CONFIGURACIÓN DE USUARIO =================

// 1. DEFINICIÓN DEL HARDWARE (12 Motores)
// Ordenados típicamente: [FL, FR, RR, RL] o según tu cableado
const std::vector<int> kAllMotorIds = {
    1, 2, 3,    // Pata 1 (Ej. Frontal Izq)
    4, 5, 6,    // Pata 2 (Ej. Frontal Der)
    7, 8, 9,    // Pata 3 (Ej. Trasera Der)
    10, 11, 12  // Pata 4 (Ej. Trasera Izq)
};

// 2. MOTORES ACTIVOS (¿Quiénes hacen la secuencia?)
// Ejemplo: {1, 2, 3} -> Mueve solo Pata 1
// Ejemplo: {3, 6, 9, 12} -> Mueve solo las rodillas
// Ejemplo: kAllMotorIds -> Mueve TODO el robot
const std::vector<int> kActiveIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}; 

// 3. CONTROL Y SEGURIDAD
const double kMaxTorque  = 3.0;   // Nm
const double kKp         = 20.0;  // Rigidez
const double kKd         = 1.0;   // Amortiguación

// Límites de perfil (Suavizado)
const double kVelLimit   = 90.0;  // deg/s
const double kAccelLimit = 200.0; // deg/s^2

// ============================================================

struct MotorTelemetry {
    int id;
    double pos_deg;
    double vel_deg_s;
    double tor_nm;
    double temp_c;
    bool online;
};

std::atomic<bool> g_running{true};

void SignalHandler(int) {
    g_running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    
    // Configuración automática (detecta --pi3hat-cfg)
    moteus::Controller::DefaultArgProcess(argc, argv);

    // Inicializar controladores
    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (int id : kAllMotorIds) {
        moteus::Controller::Options options;
        options.id = id;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    // Estructura para datos
    std::vector<MotorTelemetry> telemetry_data(kAllMotorIds.size());

    std::cout << "Iniciando control de 12 motores..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();

    while (g_running) {
        // --- 1. Lógica de Tiempo y Secuencia ---
        auto now = std::chrono::steady_clock::now();
        double t_sec = std::chrono::duration<double>(now - start_time).count();

        double seq_target = 0.0;
        std::string phase = "";

        if (t_sec < 2.0) { seq_target = 0.0; phase = "ZEROING"; }
        else if (t_sec < 4.0) { seq_target = 10.0; phase = "TARGET 10"; }
        else if (t_sec < 6.0) { seq_target = 0.0; phase = "BACK TO 0"; }
        else { seq_target = 0.0; phase = "HOLDING"; }

        // --- 2. Bucle de Control (Enviar/Recibir) ---
        for (size_t i = 0; i < controllers.size(); ++i) {
            auto& controller = controllers[i];
            int current_id = controller->options().id;
            
            // Verificar si este ID está en la lista de activos
            bool is_active = false;
            for (int active_id : kActiveIds) {
                if (active_id == current_id) { is_active = true; break; }
            }

            double target_deg = is_active ? seq_target : 0.0;

            moteus::PositionMode::Command cmd;
            cmd.position = target_deg / 360.0;
            cmd.velocity = 0.0;
            cmd.kp_scale = kKp;
            cmd.kd_scale = kKd;
            cmd.maximum_torque = kMaxTorque;
            cmd.velocity_limit = kVelLimit / 360.0;
            cmd.accel_limit = kAccelLimit / 360.0;
            cmd.stop_position = std::numeric_limits<double>::quiet_NaN();

            const auto maybe_result = controller->SetPosition(cmd);
            
            // Guardar datos
            telemetry_data[i].id = current_id;
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

        // --- 3. Dashboard Visual (Cada 100ms) ---
        static int print_cnt = 0;
        if (print_cnt++ % 10 == 0) {
            std::cout << "\033[2J\033[H"; // Limpiar pantalla
            std::cout << "========== ROBOT MONITOR (12 DOF) ==========\n";
            std::cout << " T: " << std::fixed << std::setprecision(2) << t_sec 
                      << "s | Fase: " << phase << " | Activos: " << kActiveIds.size() << "\n";
            std::cout << "--------------------------------------------\n";
            std::cout << " ID |   POS°   |   VEL°/s |  TOR(Nm) | TEMP\n";
            std::cout << "----|----------|----------|----------|------\n";

            int counter = 0;
            for (const auto& data : telemetry_data) {
                if (data.online) {
                    // Colorear si el torque es alto (simple debug visual)
                    std::string tor_color = (std::abs(data.tor_nm) > 1.5) ? "\033[1;31m" : "\033[0m"; // Rojo si > 1.5Nm

                    std::cout << " " << std::setw(2) << data.id << " | "
                              << std::setw(8) << std::fixed << std::setprecision(2) << data.pos_deg << " | "
                              << std::setw(8) << std::setprecision(1) << data.vel_deg_s << " | "
                              << tor_color << std::setw(8) << std::setprecision(2) << data.tor_nm << "\033[0m | "
                              << std::setw(4) << std::setprecision(1) << data.temp_c << "\n";
                } else {
                    std::cout << " " << std::setw(2) << data.id << " | "
                              << " OFFLINE  |   ----   |   ----   |  -- \n";
                }

                // Separador visual cada 3 motores (cada pata)
                counter++;
                if (counter % 3 == 0 && counter < 12) {
                    std::cout << "----+----------+----------+----------+------\n";
                }
            }
            std::cout << "--------------------------------------------\n";
            std::cout << " [Ctrl+C] Stop & Relax\n" << std::flush;
        }

        ::usleep(10000); // 100Hz Loop
    }

    // --- APAGADO ---
    std::cout << "\n\nDeteniendo Robot..." << std::endl;
    for (auto& controller : controllers) {
        controller->SetStop();
    }
    ::usleep(50000);
    std::cout << "Motores desenergizados.\n";

    return 0;
}
