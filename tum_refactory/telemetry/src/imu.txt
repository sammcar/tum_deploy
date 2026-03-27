#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip> // Para formatear la salida

// 1. ESTRUCTURA DE DATOS
struct IMUData {
    double acc[3];
    double gyro[3];
    double roll;
    double pitch;
    double yaw;
    double temp;
    uint64_t timestamp;
};

int main() {
    // --- VARIABLES DE CONTROL Y DEBUG ---
    bool DEBUG_MODE = false;           // Cambia a 'false' para desactivar impresiones
    const int PRINT_INTERVAL = 100;   // Imprimir cada 100 ciclos (aprox. 1 seg si HZ=100)
    int cycle_counter = 0;

    // --- CONFIGURACIÓN DE FRECUENCIA ---
    const int TARGET_HZ = 100; 
    const auto CYCLE_TIME = std::chrono::microseconds(1000000 / TARGET_HZ);

    // 2. CONFIGURAR MEMORIA COMPARTIDA
    int shm_fd = shm_open("/imu_data", O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(IMUData));
    IMUData* shared_imu = (IMUData*)mmap(0, sizeof(IMUData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    
    if (shared_imu == MAP_FAILED) {
        std::cerr << "[ERROR] No se pudo mapear la memoria." << std::endl;
        return 1;
    }
    std::memset(shared_imu, 0, sizeof(IMUData));

    // 3. CONFIGURAR PUERTO SERIAL
    int serial_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        std::cerr << "[ERROR] No se pudo abrir el puerto serial en /dev/ttyAMA0." << std::endl;
        return 1;
    }

    struct termios tty;
    tcgetattr(serial_fd, &tty);
    cfsetospeed(&tty, B115200); 
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tcsetattr(serial_fd, TCSANOW, &tty);

    // COMANDO DE REINICIO
    std::cout << "[INFO] Enviando comando de Reinicio al IMU..." << std::endl;
    unsigned char unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    unsigned char reset_yaw[]  = {0xFF, 0xAA, 0x01, 0x03, 0x00};
    write(serial_fd, unlock_cmd, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write(serial_fd, reset_yaw, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "[OK] Sistema iniciado. Debug: " << (DEBUG_MODE ? "ON" : "OFF") << std::endl;

    unsigned char buf[11];
    auto next_cycle = std::chrono::steady_clock::now();

    while (true) {
        next_cycle += CYCLE_TIME;

        unsigned char head;
        while (read(serial_fd, &head, 1) > 0) {
            if (head == 0x55) {
                unsigned char type;
                if (read(serial_fd, &type, 1) > 0) {
                    // Leemos los 9 bytes restantes (8 de datos + 1 de checksum)
                    if (read(serial_fd, buf, 9) == 9) {
                        int16_t v[4];
                        for(int i=0; i<4; i++) v[i] = (int16_t)(buf[i*2+1] << 8 | buf[i*2]);

                        if (type == 0x51) {
                            // Acelerómetro: Aplicamos el mismo mapeo que en los ángulos
                            // X_robot = -Y_imu, Y_robot = X_imu, Z_robot = Z_imu
                            shared_imu->acc[0] = -1.0 * v[1] / 32768.0 * 16.0; 
                            shared_imu->acc[1] = v[0] / 32768.0 * 16.0;
                            shared_imu->acc[2] = v[2] / 32768.0 * 16.0;
                            shared_imu->temp   = v[3] / 100.0;
                        } 
                        else if (type == 0x52) {
                            // Giroscopio: Aplicamos el mismo mapeo que en los ángulos
                            shared_imu->gyro[0] = -1.0 * v[1] / 32768.0 * 2000.0;
                            shared_imu->gyro[1] = v[0] / 32768.0 * 2000.0;
                            shared_imu->gyro[2] = v[2] / 32768.0 * 2000.0;
                        } 
                        else if (type == 0x53) {
                            // Ángulos (Ya estaba correcto)
                            shared_imu->roll  = -1.0 * v[1] / 32768.0 * 180.0;
                            shared_imu->pitch = v[0] / 32768.0 * 180.0;
                            shared_imu->yaw   = v[2] / 32768.0 * 180.0;
                        }
                    }
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        shared_imu->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();

        // --- LÓGICA DE DEBUG ---
        if (DEBUG_MODE) {
            cycle_counter++;
            if (cycle_counter >= PRINT_INTERVAL) {
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "--- [IMU DEBUG] ---" << std::endl;
                std::cout << "Ángulos -> R: " << shared_imu->roll << " | P: " << shared_imu->pitch << " | Y: " << shared_imu->yaw << std::endl;
                std::cout << "Gyro    -> X: " << shared_imu->gyro[0] << " | Y: " << shared_imu->gyro[1] << " | Z: " << shared_imu->gyro[2] << std::endl;
                std::cout << "Acc     -> X: " << shared_imu->acc[0] << " | Y: " << shared_imu->acc[1] << " | Z: " << shared_imu->acc[2] << std::endl;
                std::cout << "Temp    -> " << shared_imu->temp << " C" << std::endl;
                std::cout << "-------------------" << std::endl << std::endl;
                cycle_counter = 0;
            }
        }

        std::this_thread::sleep_until(next_cycle);
    }

    close(serial_fd);
    return 0;
}
