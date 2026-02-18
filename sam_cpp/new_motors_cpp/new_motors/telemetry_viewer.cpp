#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

// Incluimos tu cabecera oficial para asegurar que la estructura sea idéntica
#include "shared_memory.hpp"

std::atomic<bool> keep_running{true};

void signal_handler(int) {
    keep_running = false;
}

int main() {
    std::signal(SIGINT, signal_handler);

    // 1. Inicializamos el gestor de memoria
    // MemoryManager ya se encarga de abrir /rex_tel y /rex_cmd
    MemoryManager memory;

    if (!memory.is_valid()) {
        std::cerr << "[ERROR] No se pudo acceder a la memoria compartida." << std::endl;
        std::cerr << "¿Está corriendo el controlador principal?" << std::endl;
        return 1;
    }

    std::cout << "--- Monitor de Telemetría REX ---" << std::endl;
    std::cout << "Presiona Ctrl+C para salir." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (keep_running) {
        // Limpiar pantalla (ANSI escape)
        std::cout << "\033[H\033[2J"; 

        auto* tel = memory.tel; // Puntero a la telemetría real

        std::cout << "Timestamp: " << tel->timestamp_us << " us" << std::endl;
        std::cout << "-----------------------------------------------------------------------" << std::endl;
        std::cout << "| PATA | MOTOR | ANGULO (deg) | VEL (deg/s) | TORQUE (Nm) | TEMP (C) |" << std::endl;
        std::cout << "-----------------------------------------------------------------------" << std::endl;

        for (int p = 0; p < 4; ++p) {
            std::string leg_name;
            if (p == 0) leg_name = "FL";
            else if (p == 1) leg_name = "FR";
            else if (p == 2) leg_name = "BL";
            else leg_name = "BR";

            for (int m = 0; m < 3; ++m) {
                // Imprimir cada motor de la pata
                std::cout << "|  " << (m == 0 ? leg_name : "  ") << "  |   " << m << "   | "
                          << std::fixed << std::setprecision(2) << std::setw(12) << tel->measured_angles[p][m] << " | "
                          << std::setw(11) << tel->measured_velocities[p][m] << " | "
                          << std::setw(11) << tel->measured_torques[p][m] << " | "
                          << std::setw(8) << (int)tel->temperature[p][m] << " |" << std::endl;
            }
            std::cout << "|------|-------|--------------|-------------|-------------|----------|" << std::endl;
        }

        // Mostrar estados de pánico (Fault Codes)
        std::cout << "\nESTADO DE PÁNICO: ";
        for (int p = 0; p < 4; ++p) {
            std::cout << "P" << p << ":[" << (tel->fault_code[p] ? "\033[1;31mMUERTA\033[0m" : "\033[1;32m OK \033[0m") << "]  ";
        }
        std::cout << std::endl;

        // Esperar 100ms para la siguiente lectura
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nCerrando monitor..." << std::endl;
    return 0;
}
