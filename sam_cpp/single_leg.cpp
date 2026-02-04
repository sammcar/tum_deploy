#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>
#include <iomanip>
#include "moteus.h"

namespace moteus = mjbots::moteus;

// ================= CONFIGURACIÓN DE CONTROL =================
const int kMotorId = 12;
const double kMaxTorque = 12.0;   // Torque Máximo
const double kKp = 1.0;
const double kKd = 1.5;
const double kVelLimit = 0.5;    // Velocidad Límite
const double kAccelLimit = 1.1;  // Aceleración Límite
const double kToleranceDeg = 1.0;

// ================= CONFIGURACIÓN DE DATOS =================
const std::string kFileName = "datos_calibracion.csv";

// Frecuencia de guardado (Hz). 
const double kLogFreqHz = 50.0; 

// ============================================================

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

struct MotorData {
    double timestamp;
    double target_pos;
    double actual_pos;
    double velocity;
    double torque;
};

// Función auxiliar para mover el motor
void EjecutarMovimiento(moteus::Controller& controller, 
                        double target_deg, 
                        double time_limit_s, 
                        bool wait_until_arrival,
                        std::vector<MotorData>& log_buffer,
                        std::chrono::steady_clock::time_point global_start,
                        double& last_log_timestamp) { 
    
    auto phase_start = std::chrono::steady_clock::now();
    double target_rev = target_deg / 360.0;
    bool target_reached = false;
    double log_period = 1.0 / kLogFreqHz; 

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double phase_elapsed = std::chrono::duration<double>(now - phase_start).count();
        double total_elapsed = std::chrono::duration<double>(now - global_start).count();

        // 1. CONTROL
        moteus::PositionMode::Command cmd;
        cmd.position = target_rev;
        cmd.velocity = 0.0;
        cmd.kp_scale = kKp;
        cmd.kd_scale = kKd;
        cmd.maximum_torque = kMaxTorque;
        cmd.velocity_limit = kVelLimit;
        cmd.accel_limit = kAccelLimit;

        const auto result = controller.SetPosition(cmd);

        if (result) {
            double current_pos_deg = result->values.position * 360.0;
            double error = std::abs(current_pos_deg - target_deg);

            // 2. LOGGING
            if ((total_elapsed - last_log_timestamp) >= log_period) {
                MotorData data;
                data.timestamp = total_elapsed;
                data.target_pos = target_deg;
                data.actual_pos = current_pos_deg;
                data.velocity = result->values.velocity * 360.0;
                data.torque = result->values.torque;
                log_buffer.push_back(data);

                last_log_timestamp = total_elapsed; 
                
                std::cout << " -> T: " << std::fixed << std::setprecision(2) << total_elapsed 
                          << "s | Err: " << std::setprecision(1) << error << "°\r" << std::flush;
            }

            // 3. LÓGICA DE SALIDA
            if (wait_until_arrival) {
                if (error < kToleranceDeg) target_reached = true;
                if (target_reached && phase_elapsed > 0.5) break; 
                if (phase_elapsed > 5.0) {
                    std::cout << "\n⚠️ Timeout." << std::endl;
                    break;
                }
            } else {
                if (phase_elapsed >= time_limit_s) break;
            }
        }
        ::usleep(1000); 
    }
    std::cout << std::endl; 
}

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    moteus::Controller::DefaultArgProcess(argc, argv);

    moteus::Controller::Options options;
    options.id = kMotorId;
    // Habilitar el envío de límites de velocidad y aceleración
    options.position_format.velocity_limit = moteus::kFloat;
    options.position_format.accel_limit = moteus::kFloat;
    // También puede habilitar el torque máximo si desea variarlo dinámicamente
    options.position_format.maximum_torque = moteus::kFloat;

    moteus::Controller controller(options);

    std::vector<MotorData> flight_recorder;
    flight_recorder.reserve(5000); 

    double angulo_usuario = 0.0;
    std::cout << "\033[2J\033[H"; 
    std::cout << "=== CALIBRADOR MOTEUS PRO ===" << std::endl;
    std::cout << "Frecuencia de muestreo: " << kLogFreqHz << " Hz" << std::endl;
    std::cout << "Parametros actuales: Kp=" << kKp << ", Kd=" << kKd << std::endl;
    std::cout << "Límites: Vel=" << kVelLimit << ", Acc=" << kAccelLimit << ", Torq=" << kMaxTorque << std::endl;
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Introduce el ángulo deseado: ";
    if (!(std::cin >> angulo_usuario)) return 1;

    std::cout << "\n--- INICIANDO ---" << std::endl;
    auto global_start = std::chrono::steady_clock::now();
    double last_log_time = -1.0; 

    // Secuencia
    std::cout << "[1/3] Viajando..." << std::endl;
    EjecutarMovimiento(controller, angulo_usuario, 0.0, true, flight_recorder, global_start, last_log_time);

    std::cout << "[2/3] Manteniendo (1.5s)..." << std::endl;
    // MODIFICADO: Ahora espera 1.5 segundos en lugar de 3.0
    EjecutarMovimiento(controller, angulo_usuario, 1.5, false, flight_recorder, global_start, last_log_time);

    std::cout << "[3/3] Regresando..." << std::endl;
    EjecutarMovimiento(controller, 0.0, 0.0, true, flight_recorder, global_start, last_log_time);

    controller.SetStop();

    // ==========================================
    // GUARDAR CON METADATOS ACTUALIZADOS
    // ==========================================
    std::cout << "\n💾 Guardando en '" << kFileName << "'..." << std::endl;
    std::ofstream file(kFileName);
    
    if (file.is_open()) {
        // 1. ESCRIBIR METADATOS (Inician con #)
        file << "# DATOS DE CALIBRACION MOTEUS\n";
        file << "# Motor ID: " << kMotorId << "\n";
        file << "# Kp: " << kKp << "\n";
        file << "# Kd: " << kKd << "\n";
        // MODIFICADO: Agregados los parámetros solicitados
        file << "# Max Torque: " << kMaxTorque << " Nm\n";
        file << "# Vel Limit: " << kVelLimit << "\n";
        file << "# Accel Limit: " << kAccelLimit << "\n";
        file << "# Sampling Freq: " << kLogFreqHz << " Hz\n";
        file << "# ---------------------------\n";

        // 2. ESCRIBIR CABECERA DE DATOS
        file << "Time_s,Target_deg,Actual_deg,Velocity_dps,Torque_Nm\n";
        
        // 3. ESCRIBIR DATOS
        for (const auto& d : flight_recorder) {
            file << d.timestamp << "," 
                 << d.target_pos << "," 
                 << d.actual_pos << "," 
                 << d.velocity << "," 
                 << d.torque << "\n";
        }
        file.close();
        std::cout << "✅ Guardado. Datos reducidos a " << flight_recorder.size() << " puntos." << std::endl;
    } else {
        std::cerr << "❌ Error al crear archivo." << std::endl;
    }

    return 0;
}

