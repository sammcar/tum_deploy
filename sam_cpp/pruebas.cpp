#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <unistd.h>
#include <future>
#include <iomanip>
#include <csignal> // NECESARIA PARA CAPTURAR CTRL+C
#include <atomic>  // NECESARIA PARA LA BANDERA SEGURA
#include "moteus.h"

namespace moteus = mjbots::moteus;

// ============================================================
//    VARIABLES GLOBALES DE CONTROL
// ============================================================
std::atomic<bool> g_running{true};

void SignalHandler(int) {
    g_running = false; // Rompe los bucles suavemente al presionar Ctrl+C
}

// ============================================================
//    TU ESTRUCTURA DE DATOS Y CONSTANTES
// ============================================================
const double kFactorMultiplicador = 1.57;
const double kFactorMultiplicador2 = 0.888;

struct MotorConfig {
    double kp;
    double kd;
    double max_torque;
    double vel_limit;
    double accel_limit;
};

// Configuraciones específicas por articulación
const MotorConfig kCoxaConfig =  {.kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0};
const MotorConfig kFemurConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0};
const MotorConfig kTibiaConfig = {.kp = 1.0, .kd = 1.0, .max_torque = 12.0, .vel_limit = 4.0, .accel_limit = 5.0};

const std::vector<int> kMotorIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

struct SharedData {
    double angles[4][3];
    double times[4][3];
    bool is_walking; 
};

// ============================================================
//    HELPERS DE LÓGICA MECATRÓNICA
// ============================================================

const MotorConfig* GetMotorConfig(int id) {
    int type = (id - 1) % 3; // 0=Coxa, 1=Femur, 2=Tibia
    if (type == 0) return &kCoxaConfig;
    if (type == 1) return &kFemurConfig;
    return &kTibiaConfig;
}

double GetCalibratedTarget(int id, double target_deg) {
    double final_target = target_deg;
    if (id == 3 || id == 6) final_target *= kFactorMultiplicador;
    else if (id == 9 || id == 12) final_target *= kFactorMultiplicador2;
    return final_target;
}

// ============================================================
//    SISTEMA DE VISUALIZACIÓN Y TRANSPORTE
// ============================================================

void PrintTelemetry(const std::vector<moteus::CanFdFrame>& receive_frames) {
    static int counter = 0;
    if (counter++ % 10 != 0) return; // 10Hz refresh

    std::vector<double> positions(13, 0.0);
    std::vector<double> torques(13, 0.0);
    std::vector<bool> online(13, false);

    for (const auto& frame : receive_frames) {
        if (frame.source >= 1 && frame.source <= 12) {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            positions[frame.source] = res.position * 360.0;
            torques[frame.source] = res.torque;
            online[frame.source] = true;
        }
    }

    std::cout << "\033[H"; 
    std::cout << "==================================================================\n";
    std::cout << "             MONITOR DE ESTADO (TOM) - CALIBRADO                  \n";
    std::cout << "==================================================================\n";
    std::cout << " ID | TYPE  |  POS (deg) |  TORQUE (Nm) | STATUS  | CONFIG (Kp/Kd)\n";
    std::cout << "----|-------|------------|--------------|---------|---------------\n";

    for(int id : kMotorIds) {
        const MotorConfig* cfg = GetMotorConfig(id);
        std::string type = ((id-1)%3==0) ? "COXA " : ((id-1)%3==1) ? "FEMUR" : "TIBIA";
        
        std::cout << " " << std::setw(2) << id << " | " << type << " | ";
        
        if (online[id]) {
            std::string t_color = (std::abs(torques[id]) > cfg->max_torque * 0.8) ? "\033[1;31m" : "\033[0m";
            std::cout << std::fixed << std::setprecision(2) << std::setw(10) << positions[id] << " | "
                      << t_color << std::setw(12) << torques[id] << "\033[0m | "
                      << "\033[1;32m  OK   \033[0m | "
                      << std::fixed << std::setprecision(1) << cfg->kp << "/" << cfg->kd;
        } else {
            std::cout << "   --.-   |      --.-    | \033[1;31mOFFLINE\033[0m | --/--";
        }
        std::cout << "\n";
        if (id % 3 == 0 && id < 12) std::cout << "----+-------+------------+--------------+---------+---------------\n";
    }
    std::cout << "==================================================================\n" 
              << " Presione Ctrl+C para detener los motores de forma segura.\n" << std::flush;
}

void SafeTransportCycle(std::shared_ptr<moteus::Transport> transport, 
                        const std::vector<moteus::CanFdFrame>& send_frames, 
                        std::vector<moteus::CanFdFrame>* receive_frames) {
    std::promise<void> cycle_done;
    transport->Cycle(
        send_frames.data(),
        send_frames.size(),
        receive_frames,
        [&cycle_done](int) { cycle_done.set_value(); }
    );
    cycle_done.get_future().wait(); 
}

// ============================================================
//    FUNCIONES DE CONTROL (MOVIMIENTO Y HOLD)
// ============================================================

void HoldPosition(std::vector<std::shared_ptr<moteus::Controller>>& controllers, 
                  std::shared_ptr<moteus::Transport> transport, 
                  double target_deg, 
                  double duration_s) {
    
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;
    int steps = static_cast<int>(duration_s * 100); 

    // AÑADIDO: && g_running para poder cancelar
    for (int step = 0; step < steps && g_running; ++step) {
        send_frames.clear();
        for (auto& controller : controllers) {
            int id = controller->options().id;
            const MotorConfig* cfg = GetMotorConfig(id);
            
            double calibrated_deg = GetCalibratedTarget(id, target_deg);
            double target_rev = calibrated_deg / 360.0;

            moteus::PositionMode::Command cmd;
            cmd.position = target_rev;
            cmd.velocity = 0.0;
            cmd.kp_scale = cfg->kp;
            cmd.kd_scale = cfg->kd;
            cmd.maximum_torque = cfg->max_torque;
            cmd.velocity_limit = cfg->vel_limit;
            cmd.accel_limit = cfg->accel_limit;
            
            send_frames.push_back(controller->MakePosition(cmd));
        }
        SafeTransportCycle(transport, send_frames, &receive_frames);
        PrintTelemetry(receive_frames);
        usleep(10000); 
    }
}

void MoveToTarget(std::vector<std::shared_ptr<moteus::Controller>>& controllers, 
                  std::shared_ptr<moteus::Transport> transport, 
                  double target_deg, 
                  double duration_s) {
    
    std::vector<double> start_positions(13, 0.0);
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    send_frames.clear();
    for (auto& c : controllers) send_frames.push_back(c->MakePosition({}));
    SafeTransportCycle(transport, send_frames, &receive_frames);

    for (const auto& frame : receive_frames) {
        if (frame.source >= 1 && frame.source <= 12) {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            start_positions[frame.source] = res.position;
        }
    }

    int steps = static_cast<int>(duration_s * 100); 

    // AÑADIDO: && g_running para poder cancelar
    for (int step = 0; step <= steps && g_running; ++step) {
        double alpha = (double)step / steps;
        send_frames.clear();

        for (auto& controller : controllers) {
            int id = controller->options().id;
            const MotorConfig* cfg = GetMotorConfig(id);

            double calibrated_target_deg = GetCalibratedTarget(id, target_deg);
            double target_rev = calibrated_target_deg / 360.0;
            double start_rev = start_positions[id];

            moteus::PositionMode::Command cmd;
            cmd.position = start_rev + (target_rev - start_rev) * alpha;
            cmd.velocity = (target_rev - start_rev) / duration_s;
            cmd.kp_scale = cfg->kp;
            cmd.kd_scale = cfg->kd;
            cmd.maximum_torque = cfg->max_torque;
            cmd.velocity_limit = cfg->vel_limit;
            cmd.accel_limit = cfg->accel_limit;
            
            send_frames.push_back(controller->MakePosition(cmd));
        }

        SafeTransportCycle(transport, send_frames, &receive_frames);
        PrintTelemetry(receive_frames);
        usleep(10000); 
    }
}

// ============================================================
//    MAIN
// ============================================================
int main(int argc, char** argv) {
    // 1. REGISTRAR SEÑAL DE APAGADO
    std::signal(SIGINT, SignalHandler);

    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (int id : kMotorIds) {
        moteus::Controller::Options options;
        options.id = id;
        options.transport = transport;
        options.query_format.position = moteus::kFloat;
        options.query_format.velocity = moteus::kFloat;
        options.query_format.torque = moteus::kFloat;
        options.position_format.kp_scale = moteus::kFloat;
        options.position_format.kd_scale = moteus::kFloat;
        options.position_format.velocity_limit = moteus::kFloat;
        options.position_format.accel_limit = moteus::kFloat;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    std::cout << "\033[2J"; 

    // 2. Ir a Cero
    if (g_running) MoveToTarget(controllers, transport, 0.0, 3.0); 

    // 3. Repetir 3 veces
    for (int i = 1; i <= 3 && g_running; ++i) {
        MoveToTarget(controllers, transport, 10.0, 1.5);
        HoldPosition(controllers, transport, 10.0, 2.0);
        MoveToTarget(controllers, transport, 0.0, 1.5);
    }

    // 4. Bucle Final: Mantener 0 grados
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;
    send_frames.reserve(12);

    while (g_running) { // Se rompe cuando pulsas Ctrl+C
        send_frames.clear();
        for (auto& controller : controllers) {
            int id = controller->options().id;
            const MotorConfig* cfg = GetMotorConfig(id);

            moteus::PositionMode::Command cmd;
            cmd.position = 0.0;
            cmd.velocity = 0.0;
            cmd.kp_scale = cfg->kp;
            cmd.kd_scale = cfg->kd;
            cmd.maximum_torque = cfg->max_torque;
            cmd.velocity_limit = cfg->vel_limit;
            cmd.accel_limit = cfg->accel_limit;
            
            send_frames.push_back(controller->MakePosition(cmd));
        }

        SafeTransportCycle(transport, send_frames, &receive_frames);
        PrintTelemetry(receive_frames);
        usleep(10000);
    }
    
    // 5. APAGADO SEGURO
    std::cout << "\n\n--- DETENIENDO MOTORES (STOP MODE) ---\n";
    for (auto& controller : controllers) {
        controller->SetStop();
    }
    usleep(50000); 

    return 0;
}
