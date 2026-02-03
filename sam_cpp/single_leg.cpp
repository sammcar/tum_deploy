#include <iostream>
#include <vector>
#include <memory>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <limits>

#include "moteus.h"
// #include "moteus/pi3hat_moteus_transport.h"  <-- BORRAR ESTA LÍNEA
// #include "moteus/fdcanusb.h"                  <-- BORRAR ESTA TAMBIÉN SI ESTÁ

namespace moteus = mjbots::moteus;

// --- PARÁMETROS ---
const int kMotorId = 4;
const double kMaxTorque = 3.0; 
const double kKp = 1.0; 
const double kKd = 1.0; 

int main(int argc, char** argv) {
    // BORRAR TAMBIÉN EL TRUCO DEL DUMMY
    // volatile auto *dummy = ... 
    // (void)dummy;

    // 1. Procesar argumentos (Esto cargará el Pi3Hat automáticamente)
    moteus::Controller::DefaultArgProcess(argc, argv);

    moteus::Controller::Options options;
    options.id = kMotorId;
    
    auto controller = std::make_shared<moteus::Controller>(options);

    std::cout << "--- INICIANDO CONTROL MOTOR " << kMotorId << " ---" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    bool running = true;

    while (running) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;
        double t_sec = elapsed.count();

        double target_deg = 0.0;
        std::string phase = "";

        if (t_sec < 2.0) {
            target_deg = 0.0; phase = "ZEROING";
        } else if (t_sec < 4.0) {
            target_deg = 10.0; phase = "TARGET 10";
        } else if (t_sec < 6.0) {
            target_deg = 0.0; phase = "BACK TO 0";
        } else {
            target_deg = 0.0; phase = "HOLDING";
        }

        moteus::PositionMode::Command cmd;
        cmd.position = target_deg / 360.0;
        cmd.velocity = 0.0;
        cmd.kp_scale = kKp;
        cmd.kd_scale = kKd;
        cmd.maximum_torque = kMaxTorque;
        cmd.stop_position = std::numeric_limits<double>::quiet_NaN();

        const auto maybe_result = controller->SetPosition(cmd);

        if (maybe_result) {
            const auto& v = maybe_result->values;
            printf("\r[%5.2fs] %-10s | ID:%d Pos: %6.2f", t_sec, phase.c_str(), kMotorId, v.position * 360.0);
            fflush(stdout);
        } else {
            // Imprimir menos frecuente para no saturar si falla
            static int err_cnt = 0;
            if (err_cnt++ % 100 == 0) printf("\rEsperando respuesta..."); 
            fflush(stdout);
        }

        ::usleep(10000); 
    }
    return 0;
}
