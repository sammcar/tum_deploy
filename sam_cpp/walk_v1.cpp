#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;

// 1. ESTRUCTURA DE MEMORIA COMPARTIDA
struct SharedData {
    double angles[4][3];
    double velocities[4][3];
    double desired_accel[4][3];
    double kp_scale[4][3];
    double kd_scale[4][3];
    double transition_time;
    bool is_stance[4];
    bool is_walking;
};

// 2. CONFIGURACIÓN FÍSICA (mm)
const double L1 = 93.0, L2 = 147.0, L3 = 230.0;
const Vector3d HIP_OFFSETS[4] = {
    Vector3d(187.5,  100.0, 0.0), Vector3d(187.5, -100.0, 0.0),
    Vector3d(-187.5, 100.0, 0.0), Vector3d(-187.5, -100.0, 0.0)
};

const Vector3d NEUTRAL_FEET[4] = {
    Vector3d(10.0,  103.0, -296.0), Vector3d(10.0,  -93.0, -296.0),
    Vector3d(-28.0, 103.0, -296.0), Vector3d(-28.0, -93.0, -296.0)
};

// --- CINEMÁTICA ---
Vector3d ComputeWBC(Vector3d foot_pos, int idx, Vector3d body_p, Vector3d body_rpy) {
    Matrix3d R = (AngleAxisd(body_rpy.z(), Vector3d::UnitZ()) *
                  AngleAxisd(body_rpy.y(), Vector3d::UnitY()) *
                  AngleAxisd(body_rpy.x(), Vector3d::UnitX())).toRotationMatrix();

    Vector3d global_f = HIP_OFFSETS[idx] + foot_pos;
    Vector3d new_hip = body_p + (R * HIP_OFFSETS[idx]);
    return R.transpose() * (global_f - new_hip);
}

void solve_IK(Vector3d p, bool is_right, double* out) {
    double y = is_right ? -p.y() : p.y();
    double D = std::max(sqrt(y*y + p.z()*p.z()), L1 + 0.001);
    double th1 = atan2(p.z(), y) + acos(L1 / D);
    double R_val = -sqrt(std::max(0.0, D*D - L1*L1));
    double H = sqrt(p.x()*p.x() + R_val*R_val);
    double phi1 = acos(std::max(-1.0, std::min(1.0, (L2*L2 + L3*L3 - H*H) / (2*L2*L3))));
    out[0] = th1 * 180.0 / M_PI;
    out[1] = (atan2(R_val, p.x()) - asin((L3 * sin(phi1)) / (H + 1e-9)) + M_PI/2.0) * 180.0 / M_PI;
    out[2] = (M_PI - phi1) * 180.0 / M_PI;
}

// 4. FUNCIÓN PARA ENVIAR COMANDO
void SendPose(SharedData* shm, Vector3d body_p, Vector3d body_r, const Vector3d feet[4], double seconds) {
    shm->transition_time = seconds;
    shm->is_walking = false;

    Vector3d body_r_rad = body_r * M_PI / 180.0;

    for (int i = 0; i < 4; i++) {
        Vector3d local = ComputeWBC(feet[i], i, body_p, body_r_rad);
        solve_IK(local, (i == 1 || i == 3), shm->angles[i]);
        shm->is_stance[i] = true;
    }
    
    auto start = std::chrono::steady_clock::now();
    while(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void flexion(SharedData* shm, const Vector3d feet[4]) {
    std::cout << "\n[START] Iniciando rutina de Flexiones..." << std::endl;
    for (int i = 0; i < 4; ++i) {
        SendPose(shm, Vector3d(0, 0, -70.0), Vector3d(0, 0, 0), feet, 1.0);
        SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), feet, 1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void sentarse(SharedData* shm) {
    double z_inc = 80;
    double x_inc = 50;

    Vector3d A[4] = {
        Vector3d(10.0, 103.0, -296.0), Vector3d(10.0, -93.0, -296.0),
        Vector3d(-28.0, 103.0, -296.0+z_inc), Vector3d(-28.0, -93.0, -296.0+z_inc)
    };
    SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), A, 1.5);

    Vector3d B[4] = {
        Vector3d(10.0-x_inc, 103.0, -296.0), Vector3d(10.0-x_inc, -93.0, -296.0),
        Vector3d(-28.0-x_inc, 103.0, -296.0+z_inc), Vector3d(-28.0-x_inc, -93.0, -296.0+z_inc)
    };
    SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), B, 1.5);

    x_inc = 50+80;
    Vector3d C[4] = {
        Vector3d(10.0-x_inc, 103.0, -296.0+z_inc), Vector3d(10.0-x_inc, -93.0, -296.0+z_inc),
        Vector3d(-28.0-x_inc, 103.0, -296.0+z_inc), Vector3d(-28.0-x_inc, -93.0, -296.0+z_inc)
    };
    SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), C, 1.0);

   
    Vector3d D[4] = {
        Vector3d(10.0-x_inc-28.0, 103.0, -296.0+z_inc), Vector3d(10.0-x_inc-28.0, -93.0, -296.0+z_inc),
        Vector3d(-28.0-x_inc, 103.0, -296.0+z_inc), Vector3d(-28.0-x_inc, -93.0, -296.0+z_inc)
    };
    SendPose(shm, Vector3d(30, 0, -120.0), Vector3d(0, 0, 0), D, 1.0);
}

void baile(SharedData* shm) {
    std::cout << "\n[START] Iniciando rutina de Baile..." << std::endl;
    for (int i = 0; i < 4; ++i) {
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(-10, -10, 5), NEUTRAL_FEET, 2.0);
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(10, 10, 5), NEUTRAL_FEET, 2.0);
    }
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);
    double x_inc = 30;
    Vector3d NEUTRAL_FEET_2[4] = {
        Vector3d(10.0-x_inc, 103.0, -296.0), Vector3d(10.0-x_inc, -93.0, -296.0),
        Vector3d(-28.0-x_inc, 103.0, -296.0), Vector3d(-28.0-x_inc, -93.0, -296.0)
    };
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET_2, 2.0);
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);
    for (int i = 0; i < 4; ++i) {
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, -10, 0), NEUTRAL_FEET, 2.0);
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 10, 0), NEUTRAL_FEET, 2.0);
    }
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);
}

// ==========================================
// NUEVA FUNCIÓN PARA OPCIÓN 5: ÁNGULOS CERO
// ==========================================
void sendZeroAngles(SharedData* shm, double seconds) {
    std::cout << "[WARN] Enviando 0.0 a todos los motores en " << seconds << "s..." << std::endl;
    shm->transition_time = seconds;
    shm->is_walking = false;
    for (int p = 0; p < 4; ++p) {
        shm->is_stance[p] = true;
        for (int m = 0; m < 3; ++m) {
            shm->angles[p][m] = 0.0; // Forzamos el cero absoluto
        }
    }
    // Esperar a que el driver complete el movimiento suave
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

int main() {
    int fd = shm_open("/rex_cmd", O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "Error: SHM no detectada." << std::endl;
        return 1;
    }
    SharedData* shm = (SharedData*)mmap(0, sizeof(SharedData), PROT_WRITE, MAP_SHARED, fd, 0);

    int opcion = 0;
    while (true) {
        std::cout << "\n========== MENÚ DE RUTINAS ATOM-51 ==========" << std::endl;
        std::cout << "1. Rutina de Flexiones" << std::endl;
        std::cout << "2. Rutina de Baile" << std::endl;
        std::cout << "3. Rutina de Sentarse" << std::endl;
        std::cout << "4. Posición NEUTRAL (IK)" << std::endl;
        std::cout << "5. ÁNGULOS CERO (Reset Total)" << std::endl;
        std::cout << "0. Salir" << std::endl;
        std::cout << "Selecciona una opción: ";
        std::cin >> opcion;

        if (std::cin.fail()) {
            std::cin.clear(); std::cin.ignore(1000, '\n');
            continue;
        }

        if (opcion == 1) flexion(shm, NEUTRAL_FEET);
        else if (opcion == 2) baile(shm);
        else if (opcion == 3) sentarse(shm);
        else if (opcion == 4) SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 1.0);
        else if (opcion == 5) sendZeroAngles(shm, 2.0);
        else if (opcion == 0) break;
    }
    return 0;
}
