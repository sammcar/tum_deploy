#ifndef IMU_SENSOR_HPP
#define IMU_SENSOR_HPP

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>

struct IMUData {
    double acc[3];
    double gyro[3];
    double roll;
    double pitch;
    double yaw;
    double temp;
    uint64_t timestamp;
};

class IMUSensor {
private:
    int serial_fd = -1;
    IMUData imu_local;

    // --- VARIABLES PARA EL CERO POR SOFTWARE ---
    double offset_roll = 0.0;
    double offset_pitch = 0.0;
    double offset_yaw = 0.0;

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

public:
    IMUSensor() {
        std::memset(&imu_local, 0, sizeof(IMUData));
    }

    ~IMUSensor() {
        if (serial_fd >= 0) close(serial_fd);
    }

    bool init(const char* port_name = "/dev/ttyAMA0") {
        std::cout << "[IMU] Iniciando negociación a 9600 baudios..." << std::endl;
        serial_fd = open(port_name, O_RDWR | O_NOCTTY);
        if (serial_fd < 0) return false;

        set_interface_attribs(serial_fd, B9600);

        unsigned char unlock[]     = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
        unsigned char set_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
        unsigned char save[]       = {0xFF, 0xAA, 0x00, 0x00, 0x00};

        write(serial_fd, unlock, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        write(serial_fd, set_115200, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        write(serial_fd, save, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        close(serial_fd);
        
        serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd < 0) return false;
        set_interface_attribs(serial_fd, B115200);
        
        // Reset inicial de Yaw
        resetHardwareYaw();

        std::cout << "[IMU OK] Sistema en ejecución a 115200 baudios." << std::endl;
        return true;
    }

    // ========================================================
    // NUEVAS FUNCIONES DE SET ZERO
    // ========================================================

    // 1. SET ZERO POR SOFTWARE (Recomendado para equilibrar el robot)
    void setZero() {
        offset_roll = imu_local.roll;
        offset_pitch = imu_local.pitch;
        offset_yaw = imu_local.yaw;
        std::cout << "[IMU] Cero por software configurado." 
                  << " Offsets -> R: " << offset_roll 
                  << " P: " << offset_pitch 
                  << " Y: " << offset_yaw << std::endl;
    }

    // 2. RESET DE HARDWARE (Específico para mandar el eje Z a 0 internamente)
    void resetHardwareYaw() {
        if (serial_fd < 0) return;
        unsigned char unlock[]    = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
        unsigned char reset_yaw[] = {0xFF, 0xAA, 0x01, 0x03, 0x00};
        
        write(serial_fd, unlock, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        write(serial_fd, reset_yaw, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Al resetear el hardware, debemos limpiar el offset de software del Yaw
        offset_yaw = 0.0;
        std::cout << "[IMU] Comando de hardware enviado: Yaw centrado a 0." << std::endl;
    }

    // Opcional: Función para limpiar todos los ceros
    void clearZero() {
        offset_roll = 0.0;
        offset_pitch = 0.0;
        offset_yaw = 0.0;
        std::cout << "[IMU] Offsets limpiados. Leyendo valores absolutos." << std::endl;
    }

    // ========================================================

    void update() {
        if (serial_fd < 0) return;

        unsigned char head;
        unsigned char buf[11];

        while (read(serial_fd, &head, 1) > 0) {
            if (head == 0x55) {
                unsigned char type;
                if (read(serial_fd, &type, 1) > 0) {
                    if (read(serial_fd, buf, 9) == 9) {
                        int16_t v[4];
                        for(int i=0; i<4; i++) v[i] = (int16_t)(buf[i*2+1] << 8 | buf[i*2]);

                        if (type == 0x51) { 
                            imu_local.acc[0] = -1.0 * v[1] / 32768.0 * 16.0; 
                            imu_local.acc[1] = v[0] / 32768.0 * 16.0;
                            imu_local.acc[2] = v[2] / 32768.0 * 16.0;
                            imu_local.temp   = v[3] / 100.0;
                        } 
                        else if (type == 0x52) { 
                            imu_local.gyro[0] = -1.0 * v[1] / 32768.0 * 2000.0;
                            imu_local.gyro[1] = v[0] / 32768.0 * 2000.0;
                            imu_local.gyro[2] = v[2] / 32768.0 * 2000.0;
                        } 
                        else if (type == 0x53) { 
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
    }

    // GET DATA: Ahora aplica la matemática del offset
    IMUData getData() const {
        IMUData data_corregida = imu_local; // Copiamos la data cruda
        
        // Aplicamos los offsets
        data_corregida.roll -= offset_roll;
        data_corregida.pitch -= offset_pitch;
        data_corregida.yaw -= offset_yaw;

        // Mantenemos el Yaw en un rango de -180 a 180 grados
        if (data_corregida.yaw > 180.0) data_corregida.yaw -= 360.0;
        if (data_corregida.yaw < -180.0) data_corregida.yaw += 360.0;

        return data_corregida;
    }
};

#endif // IMU_SENSOR_HPP
