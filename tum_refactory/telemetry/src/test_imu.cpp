#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>

// 1. ESTRUCTURA DE DATOS LOCAL
struct IMUData {
    double acc[3];
    double gyro[3];
    double roll;
    double pitch;
    double yaw;
    double temp;
    uint64_t timestamp;
};

// Función auxiliar para configurar el puerto serie
bool set_interface_attribs(int fd, int speed) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return false;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
    return true;
}

int main() {
    bool DEBUG_MODE = true; 
    const int PRINT_INTERVAL = 100;
    int cycle_counter = 0;
    const int TARGET_HZ = 100; 
    const auto CYCLE_TIME = std::chrono::microseconds(1000000 / TARGET_HZ);

    // INSTANCIA LOCAL DE DATOS
    IMUData imu_local;
    std::memset(&imu_local, 0, sizeof(IMUData));

    // 2. RUTINA DE CONFIGURACIÓN INICIAL (A 9600 baudios)
    std::cout << "[INFO] Iniciando negociación con IMU a 9600 baudios..." << std::endl;
    int serial_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        std::cerr << "[ERROR] No se pudo abrir /dev/ttyAMA0." << std::endl;
        return 1;
    }

    set_interface_attribs(serial_fd, B9600);

    // Comandos WitMotion
    unsigned char unlock[]     = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    unsigned char set_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
    unsigned char save[]       = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    unsigned char reset_yaw[]  = {0xFF, 0xAA, 0x01, 0x03, 0x00};

    write(serial_fd, unlock, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    write(serial_fd, set_115200, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    write(serial_fd, save, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    close(serial_fd);
    std::cout << "[OK] Sensor configurado a 115200. Reiniciando puerto..." << std::endl;

    // 3. REABRIR PUERTO A ALTA VELOCIDAD (115200)
    serial_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        std::cerr << "[ERROR] No se pudo reabrir /dev/ttyAMA0." << std::endl;
        return 1;
    }
    set_interface_attribs(serial_fd, B115200);
    
    // Reset de Yaw final a la nueva velocidad
    write(serial_fd, unlock, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write(serial_fd, reset_yaw, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "[OK] Sistema en ejecución a 115200 baudios." << std::endl;

    unsigned char buf[11];
    auto next_cycle = std::chrono::steady_clock::now();

    while (true) {
        next_cycle += CYCLE_TIME;

        unsigned char head;
        // Lectura de datos del puerto serie
        while (read(serial_fd, &head, 1) > 0) {
            if (head == 0x55) {
                unsigned char type;
                if (read(serial_fd, &type, 1) > 0) {
                    if (read(serial_fd, buf, 9) == 9) {
                        int16_t v[4];
                        for(int i=0; i<4; i++) v[i] = (int16_t)(buf[i*2+1] << 8 | buf[i*2]);

                        if (type == 0x51) { // ACELERÓMETRO
                            imu_local.acc[0] = -1.0 * v[1] / 32768.0 * 16.0; 
                            imu_local.acc[1] = v[0] / 32768.0 * 16.0;
                            imu_local.acc[2] = v[2] / 32768.0 * 16.0;
                            imu_local.temp   = v[3] / 100.0;
                        } 
                        else if (type == 0x52) { // GIROSCOPIO
                            imu_local.gyro[0] = -1.0 * v[1] / 32768.0 * 2000.0;
                            imu_local.gyro[1] = v[0] / 32768.0 * 2000.0;
                            imu_local.gyro[2] = v[2] / 32768.0 * 2000.0;
                        } 
                        else if (type == 0x53) { // ÁNGULOS
                            imu_local.roll  = -1.0 * v[1] / 32768.0 * 180.0;
                            imu_local.pitch = v[0] / 32768.0 * 180.0;
                            imu_local.yaw   = v[2] / 32768.0 * 180.0;
                        }
                    }
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        imu_local.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();

        // Mostrar datos por consola
        if (DEBUG_MODE) {
            cycle_counter++;
            if (cycle_counter >= PRINT_INTERVAL) {
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "--- Lectura IMU ---" << std::endl;
                std::cout << "R: " << imu_local.roll << " P: " << imu_local.pitch << " Y: " << imu_local.yaw;
                std::cout << " | Temp: " << imu_local.temp << "C" << std::endl;
                cycle_counter = 0;
            }
        }

        std::this_thread::sleep_until(next_cycle);
    }

    close(serial_fd);
    return 0;
}
