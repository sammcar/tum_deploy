#pragma once
#include <cmath>
#include <future>
#include <vector>
#include "moteus.h"
#include "robot_config.hpp"

void SignalHandler(int) { g_running = false; }

void inicializar_robot(Leg patas[4], 
                       std::shared_ptr<mjbots::moteus::Transport> transport,
                       std::vector<std::shared_ptr<mjbots::moteus::Controller>>& controllers) 
{
    for (int p = 0; p < 4; ++p) {
        MotorData* motores_pata[3] = { &patas[p].coxa, &patas[p].femur, &patas[p].tibia };

        for (int m = 0; m < 3; ++m) {
            int id = (p * 3) + m + 1;
            motores_pata[m]->id = id;

            // Configuración de software por defecto
            motores_pata[m]->kp = 1.0;
            motores_pata[m]->kd = 1.0;
            motores_pata[m]->accel_lim = 5.0;
            motores_pata[m]->max_trq = 3.0;
            motores_pata[m]->target_pos = 0.0;
            motores_pata[m]->target_vel = 0.0;
            motores_pata[m]->ff_torque = 0.0;

            // Configuración de Hardware
            mjbots::moteus::Controller::Options options;
            options.id = id;
            options.transport = transport;

            options.position_format.position = mjbots::moteus::kFloat;
            options.position_format.velocity = mjbots::moteus::kFloat;
            options.position_format.feedforward_torque = mjbots::moteus::kFloat;
            options.position_format.kp_scale = mjbots::moteus::kFloat;
            options.position_format.kd_scale = mjbots::moteus::kFloat;
            options.position_format.maximum_torque = mjbots::moteus::kFloat;
            options.position_format.accel_limit = mjbots::moteus::kFloat;

            options.query_format.position = mjbots::moteus::kFloat;
            options.query_format.velocity = mjbots::moteus::kFloat;
            options.query_format.torque = mjbots::moteus::kFloat;
            options.query_format.temperature = mjbots::moteus::kInt8;

            auto controller = std::make_shared<mjbots::moteus::Controller>(options);
            controllers.push_back(controller);
            
            controller->SetStop();
        }
    }
    std::cout << "✅ ATOM-51 Inicializado: 12 motores vinculados a la estructura de control." << std::endl;
}


// --- OPTIMIZACIÓN PI4 ---
void optimizar_recursos_pi4()
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    struct sched_param param;
    param.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &param);
    mlockall(MCL_CURRENT | MCL_FUTURE);
}

inline double CalculateMinimumJerk(double t_elapsed, double t_total, double p_start, double p_end) {
    if (t_total < 0.001 || t_elapsed >= t_total) return p_end;
    if (t_elapsed <= 0) return p_start;
    double u = t_elapsed / t_total;
    double poly = (u * u * u) * (10.0 + u * (-15.0 + 6.0 * u));
    return p_start + (p_end - p_start) * poly;
}

inline void SafeTransportCycle(std::shared_ptr<mjbots::moteus::Transport> transport, 
                        const std::vector<mjbots::moteus::CanFdFrame>& send_frames, 
                        std::vector<mjbots::moteus::CanFdFrame>* receive_frames = nullptr) {
    std::promise<void> cycle_done;
    transport->Cycle(send_frames.data(), send_frames.size(), receive_frames, [&cycle_done](int) { cycle_done.set_value(); });
    cycle_done.get_future().wait(); 
}
