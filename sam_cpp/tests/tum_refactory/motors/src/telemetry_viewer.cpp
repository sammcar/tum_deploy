#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// Cabecera oficial para motores
#include "shared_memory.hpp"

// Estructura de la IMU (debe coincidir con tu publicador)
struct IMUData {
    double acc[3];
    double gyro[3];
    double roll;
    double pitch;
    double yaw;
    double temp;
    uint64_t timestamp;
};

struct ContactData {
    double fz_r[4]; // Fuerza vertical real en Newtons
    bool is_contact[4];  // true si supera el umbral, false si no
    double v_xyz[4][3];   // Velocidades lineales de la pata [X, Y, Z] en m/s
    double umbral_r[4];   // Umbral dinámico calculado
};

std::atomic<bool> keep_running{true};

void signal_handler(int) {
    keep_running = false;
}

int main() {
    std::signal(SIGINT, signal_handler);

    // 1. Inicializar Gestor de Memoria de Motores
    MemoryManager memory;
    if (!memory.is_valid()) {
        std::cerr << "[ERROR] No se pudo acceder a la memoria de motores." << std::endl;
        return 1;
    }

    // 2. Abrir Memoria Compartida de la IMU
    int shm_imu_fd = shm_open("/imu_data", O_RDONLY, 0666);
    if (shm_imu_fd == -1) {
        std::cerr << "[ERROR] No se pudo acceder a la memoria de la IMU. ¿Está corriendo imu_publisher?" << std::endl;
        return 1;
    }
    IMUData* imu = (IMUData*)mmap(0, sizeof(IMUData), PROT_READ, MAP_SHARED, shm_imu_fd, 0);

    // 3. --- MODIFICADO: Memoria Compartida de Contactos con Tolerancia a Fallos ---
    ContactData* contact = nullptr;
    int shm_contact_fd = shm_open("/rex_contact", O_RDONLY, 0666);
    
    if (shm_contact_fd == -1) {
        std::cerr << "[WARNING] No se encontró /rex_contact. El monitor funcionará sin datos de fuerza." << std::endl;
    } else {
        contact = (ContactData*)mmap(0, sizeof(ContactData), PROT_READ, MAP_SHARED, shm_contact_fd, 0);
        if (contact == MAP_FAILED) {
            std::cerr << "[WARNING] Fallo al mapear /rex_contact en memoria." << std::endl;
            contact = nullptr;
        }
    }

    std::cout << "--- Monitor de Telemetría REX + IMU + Contact ---" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    uint64_t last_imu_ts = 0;
    const char* leg_names[4] = {"FL", "FR", "BL", "BR"};

    while (keep_running) {
        // Limpiar pantalla
        std::cout << "\033[H\033[2J"; 

        auto* tel = memory.tel;

        // --- SECCIÓN MOTORES ---
        std::cout << "Timestamp Motores: " << tel->timestamp_us << " us" << std::endl;
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
                std::cout << "|  " << (m == 0 ? leg_name : "  ") << "  |   " << m << "   | "
                          << std::fixed << std::setprecision(2) << std::setw(12) << tel->measured_angles[p][m] << " | "
                          << std::setw(11) << tel->measured_velocities[p][m] << " | "
                          << std::setw(11) << tel->measured_torques[p][m] << " | "
                          << std::setw(8) << (int)tel->temperature[p][m] << " |" << std::endl;
            }
            std::cout << "|------|-------|--------------|-------------|-------------|----------|" << std::endl;
        }

        // --- SECCIÓN ESTADOS ---
        std::cout << "\nESTADO PÁNICO: ";
        for (int p = 0; p < 4; ++p) {
            std::cout << "P" << p << ":[" << (tel->fault_code[p] ? "\033[1;31mMUERTA\033[0m" : "\033[1;32m OK \033[0m") << "]  ";
        }
        std::cout << std::endl;

        // --- SECCIÓN DINÁMICA DE CONTACTO (AMPLIADA) ---
        std::cout << "\n---------------------------------------- DINÁMICA DE CONTACTO ----------------------------------------" << std::endl;
        std::cout << "  PATA  |  Fz (N)  | UMBRAL (N) |  Vx (m/s)  |  Vy (m/s)  |  Vz (m/s)  |  ESTADO ACTUAL" << std::endl;
        std::cout << "------------------------------------------------------------------------------------------------------" << std::endl;
        
        if (contact != nullptr) {
            for (int p = 0; p < 4; ++p) {
                std::string estado_color = contact->is_contact[p] ? "\033[1;32m[ PISANDO ]\033[0m" : "\033[1;33m[  AIRE   ]\033[0m";
                
                std::cout << "   " << leg_names[p] << "   | "
                          << std::setw(8) << contact->fz_r[p] << " | "
                          << std::setw(10) << contact->umbral_r[p] << " | "
                          << std::setw(10) << contact->v_xyz[p][0] << " | "
                          << std::setw(10) << contact->v_xyz[p][1] << " | "
                          << std::setw(10) << contact->v_xyz[p][2] << " |  "
                          << estado_color << std::endl;
            }
        } else {
            for (int p = 0; p < 4; ++p) {
                std::cout << "   " << leg_names[p] << "   | "
                          << std::setw(8) << "N/A" << " | "
                          << std::setw(10) << "N/A" << " | "
                          << std::setw(10) << "N/A" << " | "
                          << std::setw(10) << "N/A" << " | "
                          << std::setw(10) << "N/A" << " |  "
                          << "\033[1;31m[ NO DATA ]\033[0m" << std::endl;
            }
        }

        // --- SECCIÓN IMU (NUEVA) ---
        // Calcular frecuencia real de la IMU para el monitor
        double imu_hz = 0;
        if (last_imu_ts > 0 && imu->timestamp > last_imu_ts) {
            imu_hz = 1000000.0 / (imu->timestamp - last_imu_ts);
        }
        last_imu_ts = imu->timestamp;

        std::cout << "\n----------------------- ORIENTACIÓN DEL ROBOT -----------------------" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  ROLL:  " << std::setw(7) << imu->roll  << " deg  |  " 
                  << "ACC_X: " << std::setw(6) << imu->acc[0] << " g" << std::endl;
        std::cout << "  PITCH: " << std::setw(7) << imu->pitch << " deg  |  "
                  << "ACC_Y: " << std::setw(6) << imu->acc[1] << " g" << std::endl;
        std::cout << "  YAW:   " << std::setw(7) << imu->yaw   << " deg  |  "
                  << "ACC_Z: " << std::setw(6) << imu->acc[2] << " g" << std::endl;
        std::cout << "  TEMP:  " << std::setw(7) << imu->temp  << " C    |  "
                  << "FREQ:  " << std::setw(6) << imu_hz     << " Hz" << std::endl;
        std::cout << "---------------------------------------------------------------------" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nCerrando monitor..." << std::endl;
    return 0;
}
