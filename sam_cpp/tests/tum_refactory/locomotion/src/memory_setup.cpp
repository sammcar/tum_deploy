#include "memory_setup.hpp"
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

bool init_shared_memory(
    bool require_real_hardware,
    IMUData*& imu_ptr,
    CommandData*& shm_ptr,
    TelemetryData*& tel_ptr,
    ContactData*& contact_ptr)
{
    // --- 1. ABRIR MEMORIA COMPARTIDA IMU ---
    int imu_fd;
    if (require_real_hardware) {
        imu_fd = shm_open("/imu_data", O_RDONLY, 0666);
        if (imu_fd == -1) {
            std::cerr << "[ERROR] Asegúrate de ejecutar imu_publisher primero." << std::endl;
            return false;
        }
        imu_ptr = static_cast<IMUData*>(mmap(0, sizeof(IMUData), PROT_READ, MAP_SHARED, imu_fd, 0));
    } else {
        // MODO DUMMY/SIMULACIÓN: Crea la memoria localmente
        imu_fd = shm_open("/imu_data", O_CREAT | O_RDWR, 0666);
        if (imu_fd != -1) {
            ftruncate(imu_fd, sizeof(IMUData));
            imu_ptr = static_cast<IMUData*>(mmap(0, sizeof(IMUData), PROT_READ | PROT_WRITE, MAP_SHARED, imu_fd, 0));
            if (imu_ptr != MAP_FAILED) {
                std::memset(imu_ptr, 0, sizeof(IMUData)); // Todo en 0
            }
        }
    }
    if (imu_ptr == MAP_FAILED) {
        std::cerr << "Error mapeando IMU" << std::endl;
        return false;
    }

    // --- 2. ABRIR MEMORIA COMPARTIDA COMANDOS (Motores) ---
    const char* shm_name = "/rex_cmd";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(CommandData));
    shm_ptr = static_cast<CommandData*>(mmap(0, sizeof(CommandData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    if (shm_ptr == MAP_FAILED) {
        std::cerr << "[ERROR] Error accediendo a memoria compartida de comandos" << std::endl;
        return false;
    }

    // --- 3. ABRIR MEMORIA COMPARTIDA TELEMETRÍA ---
    const char* tel_name = "/rex_tel";
    int tel_fd = shm_open(tel_name, O_CREAT | O_RDWR, 0666);
    ftruncate(tel_fd, sizeof(TelemetryData));
    tel_ptr = static_cast<TelemetryData*>(mmap(0, sizeof(TelemetryData), PROT_READ | PROT_WRITE, MAP_SHARED, tel_fd, 0));

    if (tel_ptr == MAP_FAILED) {
        std::cerr << "[ERROR] Error accediendo a memoria compartida de telemetria" << std::endl;
        return false;
    }

    // --- 4. ABRIR MEMORIA COMPARTIDA CONTACTOS ---
    const char* contact_name = "/rex_contact";
    int contact_fd = shm_open(contact_name, O_CREAT | O_RDWR, 0666);
    ftruncate(contact_fd, sizeof(ContactData));
    contact_ptr = static_cast<ContactData*>(mmap(0, sizeof(ContactData), PROT_READ | PROT_WRITE, MAP_SHARED, contact_fd, 0));

    if (contact_ptr == MAP_FAILED) {
        std::cerr << "[ERROR] Error accediendo a memoria compartida de contactos" << std::endl;
        return false;
    }

    std::cout << "[OK] Todas las memorias compartidas conectadas con exito." << std::endl;
    return true;
}