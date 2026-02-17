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

// ============================================================
// 1. ESTRUCTURA DE MEMORIA COMPARTIDA
// ============================================================
#pragma pack(push, 1)
struct SharedData {
    double angles[4][3];      // Posición Objetivo [Grados]
    double velocities[4][3];  // Velocidad Real [Grados/s]
    bool is_walking;          
};
#pragma pack(pop)

struct LegAngles { double th1, th2, th3; bool valid; };
struct PointXZ { double x, z; };

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

const double L1 = 0.093, L2 = 0.147, L3 = 0.230;

// ============================================================
// 2. LÓGICA DE TRAYECTORIA TRIANGULAR CON DESACELERACIÓN
// ============================================================

PointXZ get_trajectory_point(double phase, double x_ini, double step_len, double z_suelo, double h_paso) {
    double x_mid = x_ini + (step_len / 2.0);
    double x_final = x_ini + step_len;
    double z_alto = z_suelo + h_paso;

    // Función de suavizado (S-curve) para esquinas
    auto ease = [](double t) {
        return (1.0 - cos(t * M_PI)) / 2.0;
    };

    if (phase <= 0.5) {
        // --- FASE VUELO (SWING) ---
        // Normalizamos la fase de vuelo de 0.0 a 1.0
        double t_vuelo = phase / 0.5;

        if (t_vuelo < 0.5) {
            // 1. SUBIDA DIAGONAL (de inicio a punto más alto)
            double t = ease(t_vuelo / 0.5);
            return { x_ini + (x_mid - x_ini) * t, z_suelo + (h_paso * t) };
        } 
        else {
            // 2. BAJADA DIAGONAL (de punto más alto a contacto final)
            double t = ease((t_vuelo - 0.5) / 0.5);
            return { x_mid + (x_final - x_mid) * t, z_alto - (h_paso * t) };
        }
    } else {
        // --- FASE APOYO (STANCE) ---
        // 3. RETROCESO LINEAL (por el suelo)
        double t_stance = (phase - 0.5) / 0.5;
        double t = ease(t_stance); 
        return { x_final + (x_ini - x_final) * t, z_suelo };
    }
}

// --- IK Y UTILIDADES ---
LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha) {
    double y_local = es_pata_derecha ? -y : y;
    double dist_yz = sqrt(pow(y_local, 2) + pow(z, 2));
    double D = std::max(dist_yz, L1 + 1e-6);
    double cos_v1 = L1 / D;
    if (std::abs(cos_v1) > 1.0) return {0,0,0, false};
    double theta1 = atan2(z, y_local) + acos(cos_v1);
    double R = -sqrt(std::max(0.0, pow(dist_yz, 2) - pow(L1, 2)));
    double H = sqrt(pow(x, 2) + pow(R, 2));
    double cos_v2 = (pow(L2, 2) + pow(L3, 2) - pow(H, 2)) / (2 * L2 * L3);
    if (std::abs(cos_v2) > 1.0) return {0,0,0, false};
    double phi1 = acos(cos_v2), phi2 = atan2(R, x);
    double sin_v3 = (L3 * sin(phi1)) / (H + 1e-9);
    if (std::abs(sin_v3) > 1.0) return {0,0,0, false};
    double phi3 = asin(sin_v3);
    auto wrap = [](double a) {
        a = fmod(a + M_PI, 2 * M_PI);
        if (a < 0) a += 2 * M_PI;
        return (a - M_PI) * 180.0 / M_PI;
    };
    if (es_pata_derecha) theta1 = -theta1;
    return {wrap(theta1), wrap(phi2 - phi3 + M_PI/2.0), wrap(M_PI - phi1), true};
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
// 4. MAIN
// ============================================================
int main() {
    std::signal(SIGINT, SignalHandler);

    int max_ciclos = pedir_dato<int>("Ciclos totales");
    double step_duration = pedir_dato<double>("Periodo del paso (s)"); 
    double x_start = pedir_dato<double>("X inicio (mm)");
    double step_len = pedir_dato<double>("Largo paso (mm)");
    double step_h = pedir_dato<double>("Altura paso (mm)");

    const char* shm_name = "/rex_shm";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(SharedData));
    SharedData* shm_ptr = static_cast<SharedData*>(mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    std::cout << "\n[SISTEMA LISTO] Presiona ENTER para alinear y comenzar...";
    std::cin.get();

    // --- FASE A: ALINEACIÓN SEGURA ---
    double align_time = 2.0;
    double align_dt = 0.01;
    int align_steps = align_time / align_dt;
    PointXZ p_start = get_trajectory_point(0.0, x_start, step_len, -330.0, step_h);
    LegAngles ik_start = solve_IK(p_start.x / 1000.0, L1, p_start.z / 1000.0, false);

    std::cout << ">> Alineando pata..." << std::endl;
    for (int i = 1; i <= align_steps && g_running; ++i) {
        auto t_frame = std::chrono::steady_clock::now();
        double t_lin = (double)i / align_steps;
        shm_ptr->angles[0][0] = ik_start.th1 * t_lin;
        shm_ptr->angles[0][1] = ik_start.th2 * t_lin;
        shm_ptr->angles[0][2] = ik_start.th3 * t_lin;
        shm_ptr->velocities[0][0] = ik_start.th1 / align_time;
        shm_ptr->velocities[0][1] = ik_start.th2 / align_time;
        shm_ptr->velocities[0][2] = ik_start.th3 / align_time;
        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_frame + std::chrono::duration<double>(align_dt));
    }

    // --- FASE B: BUCLE DINÁMICO ---
    std::cout << ">> Iniciando caminata TRIANGULAR SUAVE..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    double loop_dt = 0.005; // 200 Hz
    double lookahead = 0.001;

    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        if (elapsed > (max_ciclos * step_duration)) break;

        double total_phase = elapsed / step_duration;
        double phase = total_phase - std::floor(total_phase);
        double phase_next = (elapsed + lookahead) / step_duration;
        phase_next = phase_next - std::floor(phase_next);

        PointXZ p_curr = get_trajectory_point(phase, x_start, step_len, -330.0, step_h);
        PointXZ p_next = get_trajectory_point(phase_next, x_start, step_len, -330.0, step_h);

        LegAngles ik_curr = solve_IK(p_curr.x / 1000.0, L1, p_curr.z / 1000.0, false);
        LegAngles ik_next = solve_IK(p_next.x / 1000.0, L1, p_next.z / 1000.0, false);

        if (ik_curr.valid && ik_next.valid) {
            shm_ptr->angles[0][0] = ik_curr.th1;
            shm_ptr->angles[0][1] = ik_curr.th2;
            shm_ptr->angles[0][2] = ik_curr.th3;
            
            shm_ptr->velocities[0][0] = (ik_next.th1 - ik_curr.th1) / lookahead;
            shm_ptr->velocities[0][1] = (ik_next.th2 - ik_curr.th2) / lookahead;
            shm_ptr->velocities[0][2] = (ik_next.th3 - ik_curr.th3) / lookahead;
        }

        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    std::memset(shm_ptr, 0, sizeof(SharedData));
    shm_ptr->is_walking = false;
    std::cout << "\n✅ Finalizado con éxito." << std::endl;
    return 0;
}
