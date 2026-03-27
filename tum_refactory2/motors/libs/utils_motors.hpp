#pragma once

#include <cmath>
#include <future>
#include <vector>
#include <iostream>
#include <atomic>
#include "moteus.h"
#include "robot_config.hpp"
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/signal_set.hpp>
#include <sys/mman.h>  // Para mlockall, MCL_CURRENT, MCL_FUTURE
#include <sched.h>     // Para sched_setscheduler, SCHED_FIFO
#include <pthread.h>
#include <Eigen/Dense>
// --- VARIABLES GLOBALES DEL SISTEMA ---
extern std::atomic<bool> g_running;
extern bool usar_watchdog_seguridad;
extern const double WATCHDOG_MAX_TORQUE;
extern const double WATCHDOG_MAX_ERROR_POS;
extern const double WATCHDOG_MIN_VEL_STALL;
extern const int8_t WATCHDOG_MAX_TEMP;
extern const int PATA_ACTIVA_DEBUG;

// --- ESTRUCTURA PARA EL PLANIFICADOR ---
struct TrajectoryOutput {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    bool is_stance;
};

// --- DECLARACIONES DE FUNCIONES (Sin cuerpo, sin 'inline') ---

void SignalHandler(int);

void inicializar_robot(Leg patas[4], 
                       std::shared_ptr<mjbots::moteus::Transport> transport,
                       std::vector<std::shared_ptr<mjbots::moteus::Controller>>& controllers);

void optimizar_recursos_pi4();

double CalculateMinimumJerk(double t_elapsed, double t_total, double p_start, double p_end);

void SafeTransportCycle(std::shared_ptr<mjbots::moteus::Transport> transport, 
                        const std::vector<mjbots::moteus::CanFdFrame>& send_frames, 
                        std::vector<mjbots::moteus::CanFdFrame>* receive_frames = nullptr);

// Funciones de Monitor y Seguridad (Libres de shared_memory)
void ProcesarTelemetria(const std::vector<mjbots::moteus::CanFdFrame>& rx_frames_global, Leg patas[4]);

bool VerificarSeguridad(Leg patas[4]);
