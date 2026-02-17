#include <iostream>
#include <vector>
#include <memory>
#include <csignal>
#include <thread>
#include <cmath>
#include <algorithm> // Para std::max, std::min
#include "moteus.h"

// --- INCLUSIONES PROPIAS ---
#include "robot_config.hpp"
#include "telemetry.hpp"
#include "kinematics.hpp"
#include "shared_memory.hpp"
#include "utils.hpp"

// --- GLOBALES ---
SharedTelemetry global_telemetry;
std::atomic<bool> g_running{true};

void SignalHandler(int) { 
    g_running = false; 
}

// Estado local de seguridad para interpolación
struct SafetyState { 
    double current_pos = 0.0; 
};

// --- MAIN ---
int main(int argc, char** argv) {
    // 1. Configuración Básica
    std::signal(SIGINT, SignalHandler);
    mjbots::moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = mjbots::moteus::Controller::MakeSingletonTransport({});

    // 2. Memoria Compartida
    MemoryManager memory;
    if (!memory.is_valid()) {
        std::cerr << "[ERROR] No se pudo acceder a la memoria compartida (/rex_cmd o /rex_tel)." << std::endl;
        return 1;
    }

    // 3. Inicialización de Controladores
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    std::vector<SafetyState> motor_states(14); // Índice directo por ID (0-13)

    for (int id : kMotorIds) {
        mjbots::moteus::Controller::Options options;
        options.id = id;
        options.transport = transport;
        
        // Formatos de comando
        options.position_format.position = mjbots::moteus::kFloat; 
        options.position_format.velocity = mjbots::moteus::kFloat;
        options.position_format.feedforward_torque = mjbots::moteus::kFloat;
        options.position_format.kp_scale = mjbots::moteus::kFloat;
        options.position_format.kd_scale = mjbots::moteus::kFloat;
        
        // Formatos de respuesta (Query)
        options.query_format.position = mjbots::moteus::kFloat;
        options.query_format.velocity = mjbots::moteus::kFloat;
        options.query_format.torque = mjbots::moteus::kFloat;
        options.query_format.temperature = mjbots::moteus::kInt8; // Agregado para monitoreo
        
        controllers.push_back(std::make_shared<mjbots::moteus::Controller>(options));
        controllers.back()->SetStop(); 
    }

    // 4. Iniciar Hilo de Telemetría (Consola)
    std::cout << "\033[2J"; 
    std::thread monitor_thread(MonitorLoop, kMotorIds); 

    // --- FASE 1: LECTURA INICIAL ---
    {
        std::vector<mjbots::moteus::CanFdFrame> rx, tx;
        for (auto& c : controllers) tx.push_back(c->MakePosition({})); 
        SafeTransportCycle(transport, tx, &rx);

        for (const auto& frame : rx) {
            if (frame.source >= 1 && frame.source <= 13) {
                const auto res = mjbots::moteus::Query::Parse(frame.data, frame.size);
                motor_states[frame.source].current_pos = res.position;
            }
        }
    }

    // --- FASE 2: HOMING SUAVE ---
    auto homing_start = std::chrono::steady_clock::now();
    double homing_duration = 3.0;
    
    std::cout << "[INFO] Iniciando Homing..." << std::endl;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - homing_start).count();
        if (elapsed > homing_duration) break;

        std::vector<mjbots::moteus::CanFdFrame> send_frames, rec_frames;
        
        for (auto& c : controllers) {
            int id = c->options().id;
            double pos = CalculateMinimumJerk(elapsed, homing_duration, motor_states[id].current_pos, 0.0);
            
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = pos;
            cmd.velocity = 0.0;
            cmd.kp_scale = 0.5; 
            cmd.kd_scale = 0.5;
            cmd.maximum_torque = 5.0; 
            
            send_frames.push_back(c->MakePosition(cmd));
        }
        
        SafeTransportCycle(transport, send_frames, &rec_frames);
        
        // Telemetría durante Homing
        for (const auto& rx : rec_frames) {
            const auto res = mjbots::moteus::Query::Parse(rx.data, rx.size);
            global_telemetry.position[rx.source].store(res.position, std::memory_order_relaxed);
            global_telemetry.velocity[rx.source].store(res.velocity, std::memory_order_relaxed);
            global_telemetry.torque[rx.source].store(res.torque, std::memory_order_relaxed);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // --- FASE 3: RESINCRONIZACIÓN ---
    {
        std::vector<mjbots::moteus::CanFdFrame> sync_rx, sync_tx;
        for (auto& c : controllers) sync_tx.push_back(c->MakePosition({})); 
        SafeTransportCycle(transport, sync_tx, &sync_rx);

        for (const auto& frame : sync_rx) {
            if (frame.source >= 1 && frame.source <= 13) {
                const auto res = mjbots::moteus::Query::Parse(frame.data, frame.size);
                motor_states[frame.source].current_pos = res.position; 
            }
        }
    }
    std::cout << "[INFO] Control Dinámico Iniciado." << std::endl;

    // --- FASE 4: BUCLE DE CONTROL DINÁMICO ---
    auto safety_home_start = std::chrono::steady_clock::now();
    bool was_walking = false; 
    const auto period = std::chrono::microseconds(5000); 
    auto next_cycle = std::chrono::steady_clock::now();

    while (g_running) {
        auto cycle_start = std::chrono::steady_clock::now();
        next_cycle += period;
        
        // --- LEER COMANDOS (Input de Python) ---
        // Usamos memory.cmd en lugar de memory.data
        bool is_walking_mode = memory.cmd->is_walking; 
        
        if (was_walking && !is_walking_mode) {
            safety_home_start = std::chrono::steady_clock::now();
        }
        was_walking = is_walking_mode;

        std::vector<mjbots::moteus::CanFdFrame> send_frames, receive_frames;

        for (size_t i = 0; i < controllers.size(); ++i) {
            int id = kMotorIds[i];
            const MotorConfig* cfg = GetMotorConfig(id);
            int row = (id - 1) / 3; 
            int col = (id - 1) % 3; 

            double cmd_pos_rev = 0.0;
            double cmd_vel_rev_s = 0.0;
            double cmd_torque_nm = 0.0;
            double cmd_kp_scale = 1.0;
            double cmd_kd_scale = 1.0;

            if (is_walking_mode) {
                // Leer desde memory.cmd
                double q1 = memory.cmd->angles[row][0] * (M_PI / 180.0);
                double q2 = memory.cmd->angles[row][1] * (M_PI / 180.0);
                double q3 = memory.cmd->angles[row][2] * (M_PI / 180.0);

                bool stance = memory.cmd->is_stance[row];
                double mass_active = stance ? (TOTAL_ROBOT_MASS / 4.0) : LEG_MASS;
                
                double fx = mass_active * memory.cmd->desired_accel[row][0];
                double fy = mass_active * memory.cmd->desired_accel[row][1];
                double fz = mass_active * (9.81 + memory.cmd->desired_accel[row][2]);

                LegTorques tau = ComputeTorques(q1, q2, q3, fx, fy, fz, (row == 1 || row == 3));

                if (col == 0) cmd_torque_nm = tau.t1_abad;
                else if (col == 1) cmd_torque_nm = tau.t2_hip;
                else if (col == 2) cmd_torque_nm = tau.t3_knee;

                double target_deg = GetCalibratedTarget(id, memory.cmd->angles[row][col]);
                double target_vel_deg = GetCalibratedTarget(id, memory.cmd->velocities[row][col]);

                cmd_pos_rev = target_deg / 360.0;
                cmd_vel_rev_s = target_vel_deg / 360.0;
                motor_states[id].current_pos = cmd_pos_rev;

                cmd_kp_scale = memory.cmd->kp_scale[row][col];
                cmd_kd_scale = memory.cmd->kd_scale[row][col];

            } else {
                // Standby
                if (std::abs(motor_states[id].current_pos) > 0.0001) {
                    double elap = std::chrono::duration<double>(std::chrono::steady_clock::now() - safety_home_start).count();
                    cmd_pos_rev = CalculateMinimumJerk(elap, 2.0, motor_states[id].current_pos, 0.0);
                } else {
                    cmd_pos_rev = 0.0;
                }
                cmd_torque_nm = 0.0;
            }

            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = cmd_pos_rev; 
            cmd.velocity = cmd_vel_rev_s; 
            cmd.feedforward_torque = cmd_torque_nm;
            cmd.kp_scale = cmd_kp_scale;
            cmd.kd_scale = cmd_kd_scale;
            cmd.velocity_limit = cfg->vel_limit; 
            cmd.maximum_torque = cfg->max_torque;

            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        SafeTransportCycle(transport, send_frames, &receive_frames);

        // --- PUBLICAR TELEMETRÍA (Output para Python/UI) ---
        for (const auto& rx : receive_frames) {
            if (rx.source >= 1 && rx.source <= 13) {
                const auto res = mjbots::moteus::Query::Parse(rx.data, rx.size);
                
                // 1. Actualizar Telemetría Local (Consola)
                global_telemetry.position[rx.source].store(res.position, std::memory_order_relaxed);
                global_telemetry.velocity[rx.source].store(res.velocity, std::memory_order_relaxed);
                global_telemetry.torque[rx.source].store(res.torque, std::memory_order_relaxed);

                // 2. Actualizar Memoria Compartida (Shared Memory)
                int row = (rx.source - 1) / 3;
                int col = (rx.source - 1) % 3;
                
                // NOTA: Convertimos Vueltas a Grados para facilitar la lectura en la UI
                memory.tel->measured_angles[row][col] = res.position * 360.0;
                memory.tel->measured_velocities[row][col] = res.velocity * 360.0;
                memory.tel->measured_torques[row][col] = res.torque;
                memory.tel->temperature[row][col] = res.temperature;
            }
        }

        // Actualizar Timestamp
        auto cycle_end = std::chrono::steady_clock::now();
        double cycle_dur_ms = std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count();
        
        global_telemetry.cycle_time_ms.store(cycle_dur_ms, std::memory_order_relaxed);
        
        // Guardar timestamp en microsegundos para latencia en UI
        memory.tel->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_end.time_since_epoch()
        ).count();

        std::this_thread::sleep_until(next_cycle);
    }

    if (monitor_thread.joinable()) monitor_thread.join();
    for (auto& c : controllers) c->SetStop();
    
    std::cout << "Programa finalizado correctamente." << std::endl;
    return 0;
}