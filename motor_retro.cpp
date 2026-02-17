#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <iomanip>

#include "moteus.h"

// ==========================================
// 1. CONFIGURACIÓN
// ==========================================
// Motores a reiniciar al principio
const std::vector<int> kResetIds = {1, 2, 3}; 

const int kMotorActive = 2;      // El motor que hará el seno después
const int kControlFreqHz = 200; 
const double kFreqSine = 0.5;    

// CONFIGURACIÓN DE RANGO [0.0 a 0.2]
const double kMaxPos = 0.2;            
const double kAmplitude = kMaxPos / 2.0; // 0.1
const double kOffset = kMaxPos / 2.0;    // 0.1

// ==========================================
// 2. TELEMETRÍA (Thread-Safe)
// ==========================================
struct SharedTelemetry {
    std::atomic<double> position{0.0};
    std::atomic<double> velocity{0.0};
    std::atomic<double> torque{0.0};
    std::atomic<double> temp{0.0};
    std::atomic<double> cycle_time_ms{0.0};
    std::atomic<int> phase{0}; // 0=Inicio, 1=Seno
};

SharedTelemetry global_telemetry;
std::atomic<bool> keep_running{true};

void signal_handler(int) { keep_running = false; }

// ==========================================
// 3. HILO DE VISUALIZACIÓN
// ==========================================
void PrintLoop() {
    std::cout << "--- HILO MONITOR INICIADO ---" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (keep_running) {
        double p = global_telemetry.position.load(std::memory_order_relaxed);
        double v = global_telemetry.velocity.load(std::memory_order_relaxed);
        double t = global_telemetry.torque.load(std::memory_order_relaxed);
        int phase = global_telemetry.phase.load(std::memory_order_relaxed);
        
        std::string phase_str = (phase == 0) ? "RESET (1,2,3)" : "SENO (ID 2)";

        std::cout << "\r[" << phase_str << "] ID " << kMotorActive << " "
                  << "Pos: " << std::fixed << std::setprecision(3) << p << " | "
                  << "Trq: " << std::setprecision(2) << t << " Nm | "
                  << "Vel: " << std::setprecision(2) << v << "   " 
                  << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ==========================================
// 4. HILO DE CONTROL (REAL-TIME)
// ==========================================
int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    auto transport = mjbots::moteus::Transport::MakeSingleton();
    
    // Creamos controladores mapeados (opcional, pero útil si quieres optimizar)
    // Aquí usaremos MakePosition genérico para simplicidad en el bucle
    auto controller = std::make_shared<mjbots::moteus::Controller>();

    std::vector<mjbots::moteus::CanFrame> send_frames;
    std::vector<mjbots::moteus::CanFrame> receive_frames;
    // Reservamos espacio para 3 motores
    send_frames.reserve(kResetIds.size());
    receive_frames.reserve(kResetIds.size());

    std::thread printer_thread(PrintLoop);

    const auto kCycleDuration = std::chrono::microseconds(1000000 / kControlFreqHz);

    // =========================================================
    // FASE 1: INICIO SEGURO (RESET A 0.0)
    // =========================================================
    std::cout << "\n\n>>> INICIANDO RESET SEGURO (3 Segundos) <<<" << std::endl;
    global_telemetry.phase.store(0);
    
    auto start_reset = std::chrono::steady_clock::now();
    
    while (keep_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_reset = std::chrono::duration<double>(now - start_reset).count();
        
        if (elapsed_reset > 3.0) break; // Salir después de 3 segundos

        auto cycle_start = std::chrono::steady_clock::now();
        send_frames.clear();
        receive_frames.clear();

        // Enviar comando 0.0 a TODOS los IDs (1, 2, 3)
        for (int id : kResetIds) {
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = 0.0; // Ir a cero
            cmd.velocity = 0.0;
            cmd.feedforward_torque = 0.0;
            cmd.kp_scale = 1.0; 
            cmd.kd_scale = 0.5; // Un poco más suave
            cmd.maximum_torque = 2.0;
            cmd.watchdog_timeout = 0.1;
            
            // Solo pedimos respuesta (query) al ID 2 para pintar en pantalla
            // para no saturar el bus si no es necesario ver los otros
            cmd.query = (id == kMotorActive); 

            auto frame = controller->MakePosition(cmd);
            frame.source = 0;
            frame.dest = id;
            send_frames.push_back(frame);
        }

        transport->Cycle(send_frames.data(), send_frames.size(), &receive_frames);

        // Telemetría (Solo leemos del ID 2 para el monitor)
        for (const auto& rx : receive_frames) {
            if (rx.source == kMotorActive) {
                const auto res = controller->Parse(rx);
                global_telemetry.position.store(res.values.position, std::memory_order_relaxed);
                global_telemetry.velocity.store(res.values.velocity, std::memory_order_relaxed);
                global_telemetry.torque.store(res.values.torque, std::memory_order_relaxed);
            }
        }

        // Timing
        std::this_thread::sleep_for(kCycleDuration - (std::chrono::steady_clock::now() - cycle_start));
    }

    // =========================================================
    // FASE 2: MOVIMIENTO SENOIDAL (SOLO ID 2)
    // =========================================================
    std::cout << "\n\n>>> INICIANDO MOVIMIENTO SENOIDAL (ID " << kMotorActive << ") <<<" << std::endl;
    global_telemetry.phase.store(1);

    auto start_time_sine = std::chrono::steady_clock::now();

    while (keep_running) {
        auto cycle_start = std::chrono::steady_clock::now();
        send_frames.clear();
        receive_frames.clear();

        // 1. Cálculo de Trayectoria
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time_sine).count();

        // TRUCO MATEMÁTICO: Usamos -COS para empezar en 0.0
        // Offset (0.1) - Amplitud (0.1) * cos(0) = 0.0 -> ¡Transición Suave!
        double target_pos = kOffset - kAmplitude * std::cos(2.0 * M_PI * kFreqSine * t);
        
        // Derivada de -cos es sin
        double target_vel = kAmplitude * (2.0 * M_PI * kFreqSine) * std::sin(2.0 * M_PI * kFreqSine * t);

        // 2. Comando
        mjbots::moteus::PositionMode::Command cmd;
        cmd.position = target_pos;
        cmd.velocity = target_vel;
        cmd.feedforward_torque = 0.0;
        cmd.kp_scale = 1.0;
        cmd.kd_scale = 0.5;
        cmd.maximum_torque = 2.0;
        cmd.watchdog_timeout = 0.1;
        cmd.query = true;

        auto frame = controller->MakePosition(cmd);
        frame.source = 0;
        frame.dest = kMotorActive;
        send_frames.push_back(frame);

        // Nota: Los motores 1 y 3 entrarán en timeout y se apagarán (idle) 
        // porque dejamos de enviarles comandos. Si quieres que mantengan el 0 rígido,
        // tendrías que enviarles comandos a ellos también aquí.
        
        transport->Cycle(send_frames.data(), send_frames.size(), &receive_frames);

        // 3. Telemetría
        for (const auto& rx : receive_frames) {
            if (rx.source == kMotorActive) {
                const auto res = controller->Parse(rx);
                global_telemetry.position.store(res.values.position, std::memory_order_relaxed);
                global_telemetry.velocity.store(res.values.velocity, std::memory_order_relaxed);
                global_telemetry.torque.store(res.values.torque, std::memory_order_relaxed);
            }
        }

        // 4. Timing
        auto elapsed = std::chrono::steady_clock::now() - cycle_start;
        auto sleep_time = kCycleDuration - elapsed;
        if (sleep_time.count() > 0) std::this_thread::sleep_for(sleep_time);
    }

    // =========================================================
    // CIERRE
    // =========================================================
    if (printer_thread.joinable()) printer_thread.join();

    std::cout << "\nDeteniendo todos los motores..." << std::endl;
    send_frames.clear();
    
    // Parada para TODOS
    for (int id : kResetIds) {
        mjbots::moteus::PositionMode::Command stop_cmd;
        stop_cmd.mode = mjbots::moteus::Mode::kStopped;
        stop_cmd.query = false;
        auto stop_frame = controller->MakePosition(stop_cmd);
        stop_frame.dest = id;
        send_frames.push_back(stop_frame);
    }
    
    transport->Cycle(send_frames.data(), send_frames.size(), nullptr);

    return 0;
}
