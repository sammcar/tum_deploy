#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <chrono>
#include <thread>

// 1. LA ESTRUCTURA DEBE SER IDÉNTICA A LA DEL PUBLICADOR
struct IMUData {
    double acc[3];
    double gyro[3];
    double roll;
    double pitch;
    double yaw;
    double temp;
    uint64_t timestamp; // Microsegundos
};

int main() {
    // 2. ACCEDER A LA MEMORIA COMPARTIDA
    int shm_fd = shm_open("/imu_data", O_RDONLY, 0666);
    if (shm_fd == -1) {
        std::cerr << "[ERROR] No se pudo abrir la memoria. ¿Está corriendo el publicador?" << std::endl;
        return 1;
    }

    // Mapear la memoria en modo solo lectura
    IMUData* imu = (IMUData*)mmap(0, sizeof(IMUData), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (imu == MAP_FAILED) {
        std::cerr << "[ERROR] Error al mapear la memoria." << std::endl;
        return 1;
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "--- Monitor de IMU en Tiempo Real ---" << std::endl;
    std::cout << "Roll    | Pitch   | Yaw     | Temp   | Frecuencia (Hz)" << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;

    uint64_t last_ts = 0;
    
    while (true) {
        // 3. CAPTURAR DATOS ACTUALES
        double r = imu->roll;
        double p = imu->pitch;
        double y = imu->yaw;
        double t = imu->temp;
        uint64_t current_ts = imu->timestamp;

        // 4. CALCULAR FRECUENCIA REAL (Hz)
        double hz = 0;
        if (last_ts > 0 && current_ts > last_ts) {
            uint64_t diff = current_ts - last_ts;
            hz = 1000000.0 / diff; // 1,000,000 micros / diferencia
        }
        last_ts = current_ts;

        // 5. MOSTRAR RESULTADOS (Línea dinámica)
        std::cout << "\r" 
                  << std::setw(7) << r << " | "
                  << std::setw(7) << p << " | "
                  << std::setw(7) << y << " | "
                  << std::setw(5) << t << "C | "
                  << std::setw(6) << hz << " Hz" << std::flush;

        // Leemos un poco más rápido que el publicador para no perder datos
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
    }

    return 0;
}
