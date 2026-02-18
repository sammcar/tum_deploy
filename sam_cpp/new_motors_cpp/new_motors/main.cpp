#include <iostream>
#include <vector>
#include <memory>
#include <csignal>
#include <thread>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstring>
#include "moteus.h"

// --- INCLUSIONES PROPIAS ---
#include "robot_config.hpp"
#include "telemetry.hpp"
#include "dynamics.hpp"
#include "shared_memory.hpp"
#include "utils.hpp"

// --- GLOBALES ---
SharedTelemetry global_telemetry;
std::atomic<bool> g_running{true};

const bool kSchizoMode = false;
const double kSchizoThresholdDeg = 100.0;
const double kStallTorqueThreshold = 3.0;
const double kStallVelThreshold = 0.1;
const double kStallTimeLimit = 0.1;

void SignalHandler(int) { g_running = false; }

struct SafetyState
{
    double current_pos = 0.0;
    double current_vel = 0.0;
    double current_torque = 0.0;
    bool potential_stall = false;
    std::chrono::time_point<std::chrono::steady_clock> stall_start_time;
};

int main(int argc, char **argv)
{
    std::signal(SIGINT, SignalHandler);
    mjbots::moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = mjbots::moteus::Controller::MakeSingletonTransport({});

    MemoryManager memory;
    if (!memory.is_valid())
    {
        std::cerr << "[ERROR] Memoria compartida no valida." << std::endl;
        return 1;
    }

    std::memset(memory.cmd, 0, sizeof(CommandData));
    std::memset(memory.tel, 0, sizeof(TelemetryData));
    std::cout << "[INFO] Memoria compartida reseteada." << std::endl;

    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    std::vector<SafetyState> motor_states(14);

    for (int id : kMotorIds)
    {
        mjbots::moteus::Controller::Options options;
        options.id = id;
        options.transport = transport;
        options.position_format.position = mjbots::moteus::kFloat;
        options.position_format.velocity = mjbots::moteus::kFloat;
        options.position_format.accel_limit = mjbots::moteus::kFloat;
        options.position_format.feedforward_torque = mjbots::moteus::kFloat;
        options.position_format.kp_scale = mjbots::moteus::kFloat;
        options.position_format.kd_scale = mjbots::moteus::kFloat;
        options.query_format.position = mjbots::moteus::kFloat;
        options.query_format.velocity = mjbots::moteus::kFloat;
        options.query_format.temperature = mjbots::moteus::kInt8;
        controllers.push_back(std::make_shared<mjbots::moteus::Controller>(options));
        controllers.back()->SetStop();
    }

    std::cout << "\033[2J";
    std::thread monitor_thread(MonitorLoop, kMotorIds);

    // --- FASE 1: LECTURA INICIAL ---
    {
        std::vector<mjbots::moteus::CanFdFrame> rx, tx;
        for (auto &c : controllers)
            tx.push_back(c->MakeQuery());
        SafeTransportCycle(transport, tx, &rx);
        for (const auto &frame : rx)
        {
            if (frame.source >= 1 && frame.source <= 13)
            {
                const auto res = mjbots::moteus::Query::Parse(frame.data, frame.size);
                motor_states[frame.source].current_pos = res.position;
                int row = (frame.source - 1) / 3;
                int col = (frame.source - 1) % 3;
                memory.tel->measured_angles[row][col] = res.position * 360.0;
            }
        }
    }

    // --- FASE 2: HOMING SUAVE A 0.0 ---
    auto homing_start = std::chrono::steady_clock::now();
    double homing_duration = 3.0;
    while (g_running)
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - homing_start).count();
        if (elapsed > homing_duration)
            break;
        std::vector<mjbots::moteus::CanFdFrame> send_frames;
        for (auto &c : controllers)
        {
            int id = c->options().id;
            double pos = CalculateMinimumJerk(elapsed, homing_duration, motor_states[id].current_pos, 0.0);
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = pos;
            cmd.kp_scale = 0.5;
            cmd.kd_scale = 0.5;
            cmd.maximum_torque = 1.5;
            send_frames.push_back(c->MakePosition(cmd));
        }
        SafeTransportCycle(transport, send_frames);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // --- FASE 3: BUCLE DINÁMICO ---
    auto safety_home_start = std::chrono::steady_clock::now();
    bool was_walking = false;
    bool first_static_run = true;
    double last_target_angles[4][3] = {0};
    const auto period = std::chrono::microseconds(5000);
    auto next_cycle = std::chrono::steady_clock::now();
    std::vector<double> standby_start_pos(14, 0.0);

    const double kSchizoThresholdRev = kSchizoThresholdDeg / 360.0;
    bool leg_panic_state[4] = {false, false, false, false};

    while (g_running)
    {
        auto cycle_start = std::chrono::steady_clock::now();
        next_cycle += period;

        bool is_walking_mode = memory.cmd->is_walking;

        // --- LÓGICA DE DETECCIÓN DE CAMBIO DE OBJETIVO ---
        if (is_walking_mode)
        {
            first_static_run = true;
            was_walking = true;
        }
        else
        {
            bool targets_changed = false;
            for (int p = 0; p < 4; ++p)
            {
                for (int m = 0; m < 3; ++m)
                {
                    if (std::abs(memory.cmd->angles[p][m] - last_target_angles[p][m]) > 0.01)
                    {
                        targets_changed = true;
                        break;
                    }
                }
                if (targets_changed)
                    break;
            }

            if (was_walking || (first_static_run && memory.cmd->angles[0][0] != 0) || targets_changed)
            {
                safety_home_start = std::chrono::steady_clock::now();
                for (int id : kMotorIds)
                    standby_start_pos[id] = motor_states[id].current_pos;
                for (int p = 0; p < 4; ++p)
                    for (int m = 0; m < 3; ++m)
                        last_target_angles[p][m] = memory.cmd->angles[p][m];

                was_walking = false;
                first_static_run = false;
                std::cout << "[INFO] Nueva pose estatica/cambio detectado. Interpolando..." << std::endl;
            }
        }

        std::vector<mjbots::moteus::CanFdFrame> send_frames, receive_frames;

        for (size_t i = 0; i < controllers.size(); ++i)
        {
            int id = kMotorIds[i];
            const MotorConfig *cfg = GetMotorConfig(id);
            int row = (id - 1) / 3;
            int col = (id - 1) % 3;

            double cmd_pos_rev = 0.0;
            double cmd_vel_rev_s = 0.0;
            double cmd_torque_nm = 0.0;
            double cmd_kp_scale = 1.0;
            double cmd_kd_scale = 1.0;

            if (is_walking_mode)
            {
                double q1 = memory.cmd->angles[row][0] * (M_PI / 180.0);
                double q2 = memory.cmd->angles[row][1] * (M_PI / 180.0);
                double q3 = memory.cmd->angles[row][2] * (M_PI / 180.0);
                bool stance = memory.cmd->is_stance[row];
                double mass_active = stance ? (TOTAL_ROBOT_MASS / 4.0) : LEG_MASS;
                double fx = mass_active * memory.cmd->desired_accel[row][0];
                double fy = mass_active * memory.cmd->desired_accel[row][1];
                double fz = mass_active * (9.81 + memory.cmd->desired_accel[row][2]);
                LegTorques tau = ComputeTorques(q1, q2, q3, fx, fy, fz, (row == 1 || row == 3));

                if (col == 0)
                    cmd_torque_nm = tau.t1_abad;
                else if (col == 1)
                    cmd_torque_nm = tau.t2_hip;
                else if (col == 2)
                    cmd_torque_nm = tau.t3_knee;

                cmd_pos_rev = GetCalibratedTarget(id, memory.cmd->angles[row][col]) / 360.0;
                cmd_vel_rev_s = GetCalibratedTarget(id, memory.cmd->velocities[row][col]) / 360.0;
                cmd_kp_scale = memory.cmd->kp_scale[row][col];
                cmd_kd_scale = memory.cmd->kd_scale[row][col];
            }
            else
            {
                double target_rev = GetCalibratedTarget(id, memory.cmd->angles[row][col]) / 360.0;
                double t_limit = std::max(0.01, memory.cmd->transition_time);
                double elap = std::chrono::duration<double>(std::chrono::steady_clock::now() - safety_home_start).count();

                cmd_pos_rev = CalculateMinimumJerk(elap, t_limit, standby_start_pos[id], target_rev);
                cmd_vel_rev_s = 0.0; // En estatico la velocidad de feedforward es 0
                cmd_torque_nm = 0.0;
                cmd_kp_scale = 1.0;
                cmd_kd_scale = 1.0;
            }

           // -- -REINSERCIÓN DE LÓGICA DE SEGURIDAD-- -
                if (leg_panic_state[row])
            {
                // Si la pata está en pánico, mandamos torque 0 y ganancias 0
                cmd_kp_scale = 0.0;
                cmd_kd_scale = 0.0;
                cmd_torque_nm = 0.0;
            }
            else if (kSchizoMode)
            {
                double real_pos = motor_states[id].current_pos;
                double real_vel = motor_states[id].current_vel;
                double real_torque = motor_states[id].current_torque;

                // 1. Error de Posición (Schizo)
                double error = std::abs(cmd_pos_rev - real_pos);
                if (error > kSchizoThresholdRev)
                {
                    leg_panic_state[row] = true;
                    std::cout << "\033[1;31m[PANIC] Pata " << row << " ID " << id
                              << " Error Pos: " << (error * 360.0) << " deg\033[0m" << std::endl;
                }

                // 2. Detección de Atascamiento (Stall)
                bool high_torque = std::abs(real_torque) > kStallTorqueThreshold;
                bool low_velocity = std::abs(real_vel) < kStallVelThreshold;

                if (high_torque && low_velocity)
                {
                    if (!motor_states[id].potential_stall)
                    {
                        motor_states[id].potential_stall = true;
                        motor_states[id].stall_start_time = std::chrono::steady_clock::now();
                    }
                    else
                    {
                        auto now = std::chrono::steady_clock::now();
                        if (std::chrono::duration<double>(now - motor_states[id].stall_start_time).count() > kStallTimeLimit)
                        {
                            leg_panic_state[row] = true;
                            std::cout << "\033[1;31m[STALL] Pata " << row << " ID " << id << " BLOQUEADA!\033[0m" << std::endl;
                        }
                    }
                }
                else
                {
                    motor_states[id].potential_stall = false;
                }
            }

            // Aplicar corrección IDs específicos
            if (id == 6 || id == 10 || id == 11)
            {
                cmd_kp_scale = std::max(0.0, cmd_kp_scale - 0.2);
                cmd_kd_scale = std::max(0.0, cmd_kd_scale - 0.2);
            }

            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = cmd_pos_rev;
            cmd.velocity = cmd_vel_rev_s;
            cmd.feedforward_torque = cmd_torque_nm;
            cmd.kp_scale = cmd_kp_scale;
            cmd.kd_scale = cmd_kd_scale;

            double v_limit_shm = std::abs(GetCalibratedTarget(id, memory.cmd->velocities[row][col]) / 360.0);
            cmd.velocity_limit = (is_walking_mode) ? cfg->vel_limit : std::max(0.2, v_limit_shm);
            cmd.maximum_torque = cfg->max_torque;
            cmd.accel_limit = cfg->accel_limit;

            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        SafeTransportCycle(transport, send_frames, &receive_frames);

        for (const auto &rx : receive_frames)
        {
            if (rx.source >= 1 && rx.source <= 13)
            {
                const auto res = mjbots::moteus::Query::Parse(rx.data, rx.size);
                motor_states[rx.source].current_pos = res.position;
                motor_states[rx.source].current_vel = res.velocity;
                motor_states[rx.source].current_torque = res.torque;

                int row = (rx.source - 1) / 3;
                int col = (rx.source - 1) % 3;
                memory.tel->measured_angles[row][col] = res.position * 360.0;
                memory.tel->measured_velocities[row][col] = res.velocity * 360.0;
                memory.tel->measured_torques[row][col] = res.torque;
                // Dentro del bucle de recepción de frames
                memory.tel->temperature[row][col] = res.temperature;
                memory.tel->fault_code[row] = leg_panic_state[row]; // Útil para saber desde Python si una pata falló
            }
        }

        auto cycle_end = std::chrono::steady_clock::now();
        global_telemetry.cycle_time_ms.store(std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count());
        std::this_thread::sleep_until(next_cycle);
    }

    if (monitor_thread.joinable())
        monitor_thread.join();
    for (auto &c : controllers)
        c->SetStop();
    return 0;
}
