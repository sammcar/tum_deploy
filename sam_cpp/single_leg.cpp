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

#include "moteus.h"

namespace moteus = mjbots::moteus;

// --- CONFIGURACIÓN ---
const std::vector<int> kMotorIds = {4, 5, 6}; 
const int kActiveMotorId = 6; // EL ÚNICO QUE SE MOVERÁ

const double kMaxTorque = 3.0; 
const double kKp = 1.0; 
const double kKd = 1.0; 

std::atomic<bool> g_running{true};

void SignalHandler(int) {
    g_running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    moteus::Controller::DefaultArgProcess(argc, argv);

    // Crear controladores
    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (int id : kMotorIds) {
        moteus::Controller::Options options;
        options.id = id;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::cout << "--- INICIANDO ---" << std::endl;
    std::cout << "Motores fijos en 0: 4, 5" << std::endl;
    std::cout << "Motor activo (secuencia): " << kActiveMotorId << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();

    while (g_running) {
        // A. Calcular tiempo
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;
        double t_sec = elapsed.count();

        // B. Calcular el target SOLO para el motor activo
        double seq_target = 0.0;
        std::string phase = "";

        if (t_sec < 2.0) {
            seq_target = 0.0; phase = "ZERO";
        } else if (t_sec < 4.0) {
            seq_target = 20.0; phase = "UP 20";
        } else if (t_sec < 6.0) {
            seq_target = 0.0; phase = "DOWN 0";
        } else {
            seq_target = 0.0; phase = "HOLD";
        }

        // C. Preparar String de estado
        std::stringstream status_line;
        status_line.precision(1);
        status_line << std::fixed << "[" << t_sec << "s] " << phase << " | ";

        // D. Bucle para enviar a cada motor individualmente
        for (auto& controller : controllers) {
            int current_id = controller->options().id;
            
            // Decidir posición: ¿Es el motor activo o uno fijo?
            double position_deg = (current_id == kActiveMotorId) ? seq_target : 0.0;

            moteus::PositionMode::Command cmd;
            cmd.position = position_deg / 360.0;
            cmd.velocity = 0.0;
            cmd.kp_scale = kKp;
            cmd.kd_scale = kKd;
            cmd.maximum_torque = kMaxTorque;
            cmd.stop_position = std::numeric_limits<double>::quiet_NaN();

            const auto maybe_result = controller->SetPosition(cmd);
            
            if (maybe_result) {
                double pos = maybe_result->values.position * 360.0;
                status_line << "M" << current_id << ":" << pos << " ";
            }
        }

        // E. Imprimir
        static int print_cnt = 0;
        if (print_cnt++ % 10 == 0) {
            std::cout << "\r" << status_line.str() << "      " << std::flush;
        }

        ::usleep(10000); 
    }

    // --- APAGADO ---
    std::cout << "\n\nDeteniendo motores..." << std::endl;
    for (auto& controller : controllers) {
        controller->SetStop();
    }
    ::usleep(50000); 
    std::cout << "Listo." << std::endl;

    return 0;
}
