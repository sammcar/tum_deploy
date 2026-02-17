#pragma once
#include <cmath>
#include <future>
#include <vector>
#include "moteus.h"
#include "robot_config.hpp"

inline double CalculateMinimumJerk(double t_elapsed, double t_total, double p_start, double p_end) {
    if (t_total < 0.001 || t_elapsed >= t_total) return p_end;
    if (t_elapsed <= 0) return p_start;
    double u = t_elapsed / t_total;
    double poly = (u * u * u) * (10.0 + u * (-15.0 + 6.0 * u));
    return p_start + (p_end - p_start) * poly;
}

inline double GetCalibratedTarget(int id, double target_deg) {
    double final_target = target_deg;
    if (id == 3 || id == 6) final_target *= kFactorMultiplicador;
    else if (id == 9 || id == 12) final_target *= kFactorMultiplicador2;
    return final_target;
}

inline void SafeTransportCycle(std::shared_ptr<mjbots::moteus::Transport> transport, 
                        const std::vector<mjbots::moteus::CanFdFrame>& send_frames, 
                        std::vector<mjbots::moteus::CanFdFrame>* receive_frames = nullptr) {
    std::promise<void> cycle_done;
    transport->Cycle(send_frames.data(), send_frames.size(), receive_frames, [&cycle_done](int) { cycle_done.set_value(); });
    cycle_done.get_future().wait(); 
}