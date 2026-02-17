#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <string>

int leg_to_move = 3;

// ============================================================
// 1. ESTRUCTURA DE MEMORIA COMPARTIDA (IDÉNTICA AL MAIN)
// ============================================================
#pragma pack(push, 1)
struct SharedData {
    double angles[4][3];        // [Pata][Motor]
    double velocities[4][3];    
    double desired_accel[4][3]; // [X, Y, Z] Feedforward
    double kp_scale[4][3];      
    double kd_scale[4][3];      
    bool is_stance[4];          
    bool is_walking;      
};
#pragma pack(pop)

struct Vector3d {
    double x, y, z;
    Vector3d(double _x=0, double _y=0, double _z=0) : x(_x), y(_y), z(_z) {}
};

struct LegAngles { double th1, th2, th3; bool valid; };

struct TrajectoryPoint {
    double x, z;          
    double vx, vz;        
    double ax, az;        
};

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

// ============================================================
// 2. CONFIGURACIÓN FÍSICA
// ============================================================
const double L1 = 0.093;
const double L2 = 0.147;
const double L3 = 0.230;

// Solo definimos la PATA 0 (Front Left) para esta prueba
// Coordenada local del hombro respecto al centro (ajusta si es necesario)

Vector3d LEG_STAND_0 = { 0.02,   0.093, -0.316}; // 0: FL
Vector3d LEG_STAND_1 = { 0.02,  -0.093, -0.316}; // 1: FR
Vector3d LEG_STAND_2 = {-0.018,  0.093, -0.316}; // 2: BL
Vector3d LEG_STAND_3 = {-0.018, -0.093, -0.316}; // 3: BR

// ============================================================
// 3. MATEMÁTICAS (IK + BÉZIER)
// ============================================================

double wrap_to_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha) {
    double y_local = es_pata_derecha ? -y : y;
    double dist_yz = sqrt(pow(y_local, 2) + pow(z, 2));
    double D = std::max(dist_yz, L1 + 1e-6);
    
    // Clamp para evitar acos fuera de dominio
    double val_c1 = L1 / D;
    if (val_c1 < -1.0) val_c1 = -1.0; if (val_c1 > 1.0) val_c1 = 1.0;
    
    double theta1 = atan2(z, y_local) + acos(val_c1);

    double R_term = pow(dist_yz, 2) - pow(L1, 2);
    double R = -sqrt(std::max(0.0, R_term)); 
    double H = sqrt(pow(x, 2) + pow(R, 2));
    
    double cos_phi1 = (pow(L2, 2) + pow(L3, 2) - pow(H, 2)) / (2 * L2 * L3);
    if (cos_phi1 < -1.0) cos_phi1 = -1.0; if (cos_phi1 > 1.0) cos_phi1 = 1.0;
    double phi1 = acos(cos_phi1);
    
    double theta3 = M_PI - phi1; 
    double phi2 = atan2(R, x);
    
    double sin_phi3 = (L3 * sin(phi1)) / (H + 1e-9);
    if (sin_phi3 < -1.0) sin_phi3 = -1.0; if (sin_phi3 > 1.0) sin_phi3 = 1.0;
    double phi3 = asin(sin_phi3);
    
    double theta2 = phi2 - phi3;

    // Ajuste final para tu robot (según tu código previo) + Conversión a Grados
    // Nota: El código previo retornaba grados, la IK del main.cpp espera grados.
    return {
        wrap_to_pi(theta1) * 180.0 / M_PI,
        wrap_to_pi(theta2 + M_PI/2.0) * 180.0 / M_PI, // Offset de montaje común
        wrap_to_pi(theta3) * 180.0 / M_PI,
        true
    };
}

double lerp(double a, double b, double t) { return a + t * (b - a); }

TrajectoryPoint calculate_bezier(double t, double duration, 
                                 double x0, double z0,
                                 double x1, double z1,
                                 double x2, double z2,
                                 double x3, double z3) 
{
    if (t < 0) t = 0; if (t > 1) t = 1;
    double t2 = t * t; double t3 = t2 * t;
    double u = 1.0 - t; double u2 = u * u; double u3 = u2 * u;

    TrajectoryPoint res;
    res.x = u3*x0 + 3*u2*t*x1 + 3*u*t2*x2 + t3*x3;
    res.z = u3*z0 + 3*u2*t*z1 + 3*u*t2*z2 + t3*z3;

    double d2xdt2 = 6*u*(x2 - 2*x1 + x0) + 6*t*(x3 - 2*x2 + x1);
    double d2zdt2 = 6*u*(z2 - 2*z1 + z0) + 6*t*(z3 - 2*z2 + z1);

    if (duration > 1e-5) {
        res.ax = d2xdt2 / (duration * duration);
        res.az = d2zdt2 / (duration * duration);
    } else { res.ax = 0; res.az = 0; }

    return res;
}

TrajectoryPoint get_step_trajectory(double phase, double step_period, 
                                    double origin_x, double origin_z, 
                                    double step_len, double step_height) {
    phase = phase - std::floor(phase);
    
    double x_start = origin_x - step_len / 2.0;
    double x_end   = origin_x + step_len / 2.0;
    
    // --- STANCE (0.5 - 1.0) ---
    if (phase >= 0.5) {
        double t = (phase - 0.5) * 2.0;
        TrajectoryPoint p;
        p.x = lerp(x_end, x_start, t);
        p.z = origin_z;
        p.ax = 0; p.az = 0;
        return p;
    }

    // --- SWING (0.0 - 0.5) ---
    double t = phase * 2.0;
    double lift_bias = 0.2; 
    
    return calculate_bezier(t, step_period * 0.5,
        x_start, origin_z,
        x_start + (step_len * lift_bias), origin_z + (step_height * 1.3),
        x_end - (step_len * lift_bias), origin_z + (step_height * 1.3),
        x_end, origin_z
    );
}

template <typename T>
T pedir_dato(std::string mensaje) {
    T valor; std::cout << ">> " << mensaje << ": ";
    while (!(std::cin >> valor)) {
        std::cin.clear(); std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "❌ Error. " << mensaje << ": ";
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return valor;
}

// ============================================================
// 4. MAIN (SOLO PATA 0)
// ============================================================
int main() {
    std::signal(SIGINT, SignalHandler);

    std::cout << "========================================" << std::endl;
    std::cout << "   TEST DE TRAYECTORIA - SOLO PATA 0    " << std::endl;
    std::cout << "========================================" << std::endl;

    Vector3d LEG_STAND_XYZ;

    if (leg_to_move == 0) {LEG_STAND_XYZ = LEG_STAND_0;}
    else if (leg_to_move == 1) {LEG_STAND_XYZ = LEG_STAND_1;}
    else if (leg_to_move == 2) {LEG_STAND_XYZ = LEG_STAND_2;}
    else if (leg_to_move == 3) {LEG_STAND_XYZ = LEG_STAND_3;}
    else {
        std::cerr << "Número de pata inválido. Debe ser 0, 1, 2 o 3." << std::endl;
        return -1;
    }

    bool es_derecha = (leg_to_move == 1 || leg_to_move == 3);

    int max_ciclos = pedir_dato<int>("Ciclos totales");
    double step_duration = pedir_dato<double>("Periodo paso (s) (Recomendado 1.0 para test)"); 
    double step_len = pedir_dato<double>("Largo paso (mm) (Recomendado 50)");
    double step_h = pedir_dato<double>("Altura paso (mm) (Recomendado 30)");

    step_len /= 1000.0;
    step_h /= 1000.0;

    const char* shm_name = "/rex_shm";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(SharedData));
    SharedData* shm_ptr = static_cast<SharedData*>(mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    // Limpiamos memoria por seguridad
    std::memset(shm_ptr, 0, sizeof(SharedData));

    std::cout << "\n[INFO] Moviendo Pata " << leg_to_move << "." << std::endl;
    std::cout << "[SISTEMA LISTO] Presiona ENTER para alinear y comenzar..." << std::endl;
    std::cin.get();

    // --- FASE A: ALINEACIÓN SUAVE ---
    std::cout << ">> Alineando Pata " << leg_to_move << "..." << std::endl;
    double align_duration = 3.0;
    int align_steps = align_duration / 0.01;

    // Calculamos pose inicial
    TrajectoryPoint pt_start = get_step_trajectory(0.0, step_duration, LEG_STAND_XYZ.x, LEG_STAND_XYZ.z, step_len, step_h);
    LegAngles ang_start = solve_IK(pt_start.x, LEG_STAND_XYZ.y, pt_start.z, es_derecha);

    for (int i = 1; i <= align_steps && g_running; ++i) {
        double t = (double)i / align_steps; 
        double smooth = (1.0 - std::cos(t * M_PI)) / 2.0;

        // Solo escribimos en el índice [0]
        if (ang_start.valid) {
            shm_ptr->angles[leg_to_move][0] = ang_start.th1 * smooth;
            shm_ptr->angles[leg_to_move][1] = ang_start.th2 * smooth;
            shm_ptr->angles[leg_to_move][2] = ang_start.th3 * smooth;
            
            // Forzar Stance y ganancias altas para alinear
            shm_ptr->is_stance[leg_to_move] = true; 
            shm_ptr->kp_scale[leg_to_move][0] = 1.0; shm_ptr->kd_scale[0][0] = 1.0;
            shm_ptr->kp_scale[leg_to_move][1] = 1.0; shm_ptr->kd_scale[0][1] = 1.0;
            shm_ptr->kp_scale[leg_to_move][2] = 1.0; shm_ptr->kd_scale[0][2] = 1.0;
        }
        
        // Aseguramos que las otras patas estén quietas/apagadas
        for(int k=0; k<4; k++) {
            if (k != leg_to_move) {
                shm_ptr->is_stance[k] = true; // Decirle al controlador que están en el suelo (modo seguro)
                // Opcional: Poner sus KP/KD altos para que se queden quietas
                for(int m=0; m<3; m++) {
                    shm_ptr->kp_scale[k][m] = 1.0;
                    shm_ptr->kd_scale[k][m] = 1.0;
                }
            }
        }

        shm_ptr->is_walking = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // --- FASE B: BUCLE DINÁMICO (SOLO PATA 0) ---
    std::cout << ">> Iniciando ciclo en Pata " << leg_to_move << "..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    double loop_dt = 0.004;
    double prev_ang[3] = {ang_start.th1, ang_start.th2, ang_start.th3};

    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        if (elapsed > (max_ciclos * step_duration)) break;

        double phase = fmod(elapsed / step_duration, 1.0);

        // 1. Trayectoria
        TrajectoryPoint pt = get_step_trajectory(phase, step_duration, 
                                                 LEG_STAND_XYZ.x, LEG_STAND_XYZ.z, 
                                                 step_len, step_h);

        // 2. IK
        LegAngles ik = solve_IK(pt.x, LEG_STAND_XYZ.y, pt.z, es_derecha);

        if (ik.valid) {
            // Posición
            shm_ptr->angles[leg_to_move][0] = ik.th1;
            shm_ptr->angles[leg_to_move][1] = ik.th2;
            shm_ptr->angles[leg_to_move][2] = ik.th3;

            // Velocidad
            shm_ptr->velocities[leg_to_move][0] = (ik.th1 - prev_ang[0]) / loop_dt;
            shm_ptr->velocities[leg_to_move][1] = (ik.th2 - prev_ang[1]) / loop_dt;
            shm_ptr->velocities[leg_to_move][2] = (ik.th3 - prev_ang[2]) / loop_dt;
            
            prev_ang[0] = ik.th1; prev_ang[1] = ik.th2; prev_ang[2] = ik.th3;

            // Aceleración (Feedforward)
            shm_ptr->desired_accel[leg_to_move][0] = pt.ax;
            shm_ptr->desired_accel[leg_to_move][1] = 0.0;
            shm_ptr->desired_accel[leg_to_move][2] = pt.az;

            // Estado
            bool stance = (phase >= 0.5);
            shm_ptr->is_stance[0] = stance;

            // Ganancias variables
            double kp = stance ? 1.0 : 0.6;
            double kd = stance ? 1.0 : 0.4;
            
            for(int m=0; m<3; m++) {
                shm_ptr->kp_scale[leg_to_move][m] = kp;
                shm_ptr->kd_scale[leg_to_move][m] = kd;
            }
        }

        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    std::memset(shm_ptr, 0, sizeof(SharedData));
    shm_ptr->is_walking = false;
    std::cout << "\n✅ Test finalizado." << std::endl;
    return 0;
}
