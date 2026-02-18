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

// 1. Estructura exacta de comunicación
struct SharedData {
    double angles[4][3];
    double velocities[4][3];
    double desired_accel[4][3];
    double kp_scale[4][3];
    double kd_scale[4][3];
    bool is_stance[4];
    bool is_walking;
};

// 2. Constantes Físicas del ATOM-51
const double L1 = 0.093, L2 = 0.147, L3 = 0.230;
const Vector3d HIP_OFFSETS[4] = {
    Vector3d(0.1875, 0.10, 0), Vector3d(0.1875, -0.10, 0),
    Vector3d(-0.1875, 0.10, 0), Vector3d(-0.1875, -0.10, 0)
};
const Vector3d STAND_XYZ[4] = {
    Vector3d(0.01, 0.103, -0.296), Vector3d(0.01, -0.093, -0.296),
    Vector3d(-0.028, 0.103, -0.296), Vector3d(-0.028, -0.093, -0.296)
};

// 1. Corrección en ComputeWBC
Vector3d ComputeWBC(Vector3d foot_pos, int idx, Vector3d body_p, Vector3d body_rpy) {
    // Agregamos .toRotationMatrix() al final
    Matrix3d R = (AngleAxisd(body_rpy.z(), Vector3d::UnitZ()) *
                  AngleAxisd(body_rpy.y(), Vector3d::UnitY()) *
                  AngleAxisd(body_rpy.x(), Vector3d::UnitX())).toRotationMatrix();

    Vector3d global_f = HIP_OFFSETS[idx] + foot_pos;
    Vector3d new_hip = body_p + (R * HIP_OFFSETS[idx]);
    return R.transpose() * (global_f - new_hip);
}
void solve_IK(Vector3d p, bool is_right, double* out) {
    double y = is_right ? -p.y() : p.y();
    double D = std::max(sqrt(y*y + p.z()*p.z()), L1 + 1e-6);
    double th1 = atan2(p.z(), y) + acos(L1 / D);
    double R = -sqrt(std::max(0.0, D*D - L1*L1));
    double H = sqrt(p.x()*p.x() + R*R);
    double phi1 = acos(std::max(-1.0, std::min(1.0, (L2*L2 + L3*L3 - H*H) / (2*L2*L3))));
    out[0] = th1 * 180.0 / M_PI;
    out[1] = (atan2(R, p.x()) - asin((L3 * sin(phi1)) / (H + 1e-9)) + M_PI/2.0) * 180.0 / M_PI;
    out[2] = (M_PI - phi1) * 180.0 / M_PI;
}

// 2. Corrección en ApplyPose
void ApplyPose(SharedData* shm, Vector3d pos, Vector3d rpy, double duration) {
    // Cambiamos el punto (.) por doble punto (::) antes de now()
    auto start = std::chrono::steady_clock::now(); 
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start).count() / duration;
        if (t > 1.0) t = 1.0;
        
        // ... resto del código igual ...
        if (t >= 1.0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    int fd = shm_open("/rex_cmd", O_RDWR, 0666);
    SharedData* shm = (SharedData*)mmap(0, sizeof(SharedData), PROT_WRITE, MAP_SHARED, fd, 0);

    std::cout << "Iniciando secuencia de posturas..." << std::endl;

    // SECUENCIA:
    // 1. Agacharse (Z -50mm)
    ApplyPose(shm, Vector3d(0, 0, -0.05), Vector3d(0, 0, 0), 2.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 2. Inclinación lateral (Roll 15 deg)
    ApplyPose(shm, Vector3d(0, 0, -0.05), Vector3d(10.0 * M_PI/180.0, 0, 0), 1.5);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. Saludo/Pitch (Pitch 20 deg)
    ApplyPose(shm, Vector3d(0.04, 0, -0.03), Vector3d(0, 10.0 * M_PI/180.0, 0), 1.5);

    std::cout << "Secuencia completada." << std::endl;
    return 0;
}
