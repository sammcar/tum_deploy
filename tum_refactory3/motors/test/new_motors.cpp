#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <chrono>
#include "moteus.h"

using namespace mjbots;

// =========================================================
// ESTRUCTURAS DE DATOS (Integradas para que compile directo)
// =========================================================
struct MotorData {
    int id;
    double current_pos;
    double current_vel;
    double current_torque;
    double target_pos;
    double target_vel;
    double ff_torque;
    double kp;
    double kd;
    double accel_lim;
    double max_trq;
};

struct Leg {
    MotorData coxa;
    MotorData femur;
    MotorData tibia;
};

// =========================================================
// GLOBALES
// =========================================================
boost::asio::steady_timer* timer_global;
std::vector<moteus::CanFdFrame> tx_frames_global;
std::vector<moteus::CanFdFrame> rx_frames_global;
std::vector<std::shared_ptr<moteus::Controller>> controllers;

Leg patas[4]; // Usaremos solo patas[0] para los IDs 1, 2, 3

enum class RobotState { HOMING, DONE };
RobotState estado_actual = RobotState::HOMING;

double start_pos[4] = {0.0};   // Posiciones iniciales (el índice 0 no se usa)
double tiempo_homing = 0.0;
const double DURACION_HOMING = 3.0; // Segundos
const double DT = 0.0025; // 400Hz (2.5 milisegundos)

void HandleStatus(std::shared_ptr<moteus::Transport> transport);

// =========================================================
// FUNCIÓN DE INICIALIZACIÓN (1 pata / 3 motores)
// =========================================================
void inicializar_robot_prueba(Leg patas[4], 
                              std::shared_ptr<mjbots::moteus::Transport> transport,
                              std::vector<std::shared_ptr<mjbots::moteus::Controller>>& controllers) 
{
    int p = 0; 
    MotorData* motores_pata[3] = { &patas[p].coxa, &patas[p].femur, &patas[p].tibia };

    for (int m = 0; m < 3; ++m) {
        int id = m + 1; // IDs 1, 2, 3
        motores_pata[m]->id = id;

        motores_pata[m]->kp = 1.0;
        motores_pata[m]->kd = 1.0;
        motores_pata[m]->target_pos = 0.0;
        motores_pata[m]->target_vel = 0.0;
        motores_pata[m]->ff_torque = 0.0;

        mjbots::moteus::Controller::Options options;
        options.id = id;
        options.transport = transport;

        options.position_format.position = mjbots::moteus::kFloat;
        options.position_format.velocity = mjbots::moteus::kFloat;
        options.position_format.feedforward_torque = mjbots::moteus::kFloat;
        options.position_format.kp_scale = mjbots::moteus::kFloat;
        options.position_format.kd_scale = mjbots::moteus::kFloat;
        options.position_format.maximum_torque = mjbots::moteus::kFloat; 
        
        options.query_format.position = mjbots::moteus::kFloat;
        options.query_format.velocity = mjbots::moteus::kFloat;
        options.query_format.torque = mjbots::moteus::kFloat;
        options.query_format.temperature = mjbots::moteus::kInt8;

        auto controller = std::make_shared<mjbots::moteus::Controller>(options);
        controllers.push_back(controller);
        
        controller->SetStop(); 
    }
    std::cout << "✅ ATOM-51: 3 motores inicializados con formatos CAN configurados." << std::endl;
}

// =========================================================
// EL METRÓNOMO (400 Hz)
// =========================================================
void HandleTimer(const boost::system::error_code& ec, std::shared_ptr<moteus::Transport> transport) 
{
    if (ec) return;
    
    // Reprogramar para exactamente 2.5 milisegundos en el futuro
    timer_global->expires_at(timer_global->expiry() + std::chrono::microseconds(2500));
    timer_global->async_wait([transport](const boost::system::error_code& e){ HandleTimer(e, transport); });

    // Enviar frames empaquetados en el ciclo anterior y pedir nuevos
    transport->Cycle(
        tx_frames_global.data(), tx_frames_global.size(), &rx_frames_global, 
        [transport](int) { HandleStatus(transport); }
    );
}

// =========================================================
// LÓGICA DE CONTROL (Homing y Ejecución)
// =========================================================
void HandleStatus(std::shared_ptr<moteus::Transport> transport) 
{
    // 1. Leer feedback real
    for (const auto& frame : rx_frames_global) {
        const auto res = moteus::Query::Parse(frame.data, frame.size);
        int id = frame.source;
        if (id >= 1 && id <= 3) {
            MotorData* motores[3] = {&patas[0].coxa, &patas[0].femur, &patas[0].tibia};
            motores[id-1]->current_pos = res.position;
            motores[id-1]->current_vel = res.velocity;
            motores[id-1]->current_torque = res.torque;
        }
    }
    
    // Limpiar el buffer de envío para prepararlo
    tx_frames_global.clear();

    // 2. Máquina de estados
    if (estado_actual == RobotState::HOMING) 
    {
        tiempo_homing += DT; 
        double t = std::clamp(tiempo_homing / DURACION_HOMING, 0.0, 1.0);

        for (int id = 1; id <= 3; ++id) {
            double target = start_pos[id] + t * (0.0 - start_pos[id]);

            moteus::PositionMode::Command cmd;
            cmd.position = target;
            cmd.velocity = 0.0;
            cmd.kp_scale = 1.0;
            cmd.kd_scale = 1.0;
            cmd.maximum_torque = 1.5; // Límite de seguridad
            
            tx_frames_global.push_back(controllers[id - 1]->MakePosition(cmd));
        }

        if (t >= 1.0) {
            estado_actual = RobotState::DONE;
            std::cout << "[SISTEMA] Homing completado. Manteniendo posición 0." << std::endl;
        }
    } 
    else if (estado_actual == RobotState::DONE) 
    {
        for (int id = 1; id <= 3; ++id) {
            moteus::PositionMode::Command cmd;
            cmd.position = 0.0; 
            cmd.kp_scale = 1.0;
            cmd.kd_scale = 1.0;
            cmd.maximum_torque = 1.5; // Límite de seguridad permanente
            tx_frames_global.push_back(controllers[id - 1]->MakePosition(cmd));
        }
    }
}

// =========================================================
// MAIN
// =========================================================
int main(int argc, char **argv) {
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    // 1. Inicialización
    inicializar_robot_prueba(patas, transport, controllers);

    // 2. Sincronizar (Foto Inicial)
    std::cout << "Sincronizando posiciones iniciales..." << std::endl;
    std::vector<moteus::CanFdFrame> tx_init, rx_init;
    for (auto& c : controllers) {
        tx_init.push_back(c->MakeQuery());
    }
    
    // Llamada bloqueante para leer el estado antes de arrancar el timer
    transport->Cycle(tx_init.data(), tx_init.size(), &rx_init);

    for (const auto& frame : rx_init) {
        const auto res = moteus::Query::Parse(frame.data, frame.size);
        int id = frame.source;
        if (id >= 1 && id <= 3) {
            start_pos[id] = res.position;
            std::cout << "Motor " << id << " arranca en: " << start_pos[id] << " rev" << std::endl;
        }
    }

    // 3. Preparar Primer Frame
    for (int id = 1; id <= 3; ++id) {
        moteus::PositionMode::Command cmd;
        cmd.position = start_pos[id]; 
        cmd.maximum_torque = 1.5; 
        tx_frames_global.push_back(controllers[id - 1]->MakePosition(cmd));
    }

    // 4. Iniciar Bucle Asíncrono
    boost::asio::io_context io_context;
    boost::asio::steady_timer timer(io_context);
    timer_global = &timer;

    // Arrancar el timer en 1ms
    timer.expires_after(std::chrono::milliseconds(1));
    timer.async_wait([transport](const boost::system::error_code& e){ HandleTimer(e, transport); });

    std::cout << "Iniciando bucle de control asíncrono a 400Hz..." << std::endl;
    
    // Esto mantendrá el programa corriendo y gestionará los callbacks
    io_context.run(); 

    return 0;
}
