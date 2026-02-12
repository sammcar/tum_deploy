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

#pragma pack(push, 1)
struct SharedData {
    double angles[4][3];
    double velocities[4][3];
    bool is_walking;
};
#pragma pack(pop)

struct LegAngles { double th1, th2, th3; bool valid; };
struct PointXZ { double x, z; };

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

const double L1 = 0.093, L2 = 0.147, L3 = 0.230;

// ============================================================
// LÓGICA MEJORADA: TRANSICIÓN SUAVE (CROSS-FADE)
// ============================================================

PointXZ cubic_bezier(PointXZ p0, PointXZ p1, PointXZ p2, PointXZ p3, double t) {
    double u = 1.0 - t, tt = t * t, uu = u * u, uuu = uu * u, ttt = tt * t;
    return {
        (uuu * p0.x) + (3 * uu * t * p1.x) + (3 * u * tt * p2.x) + (ttt * p3.x),
        (uuu * p0.z) + (3 * uu * t * p1.z) + (3 * u * tt * p2.z) + (ttt * p3.z)
    };
}

PointXZ get_trajectory_point(double phase, double x_ini, double step_len, double z_suelo, double h_paso) {
    PointXZ P0 = {x_ini, z_suelo};
    PointXZ P3 = {x_ini + step_len, z_suelo};
    
    // Ajuste de control P2 para que la curva termine moviéndose hacia atrás (pre-sincronización)
    double h_ctrl = z_suelo + (4.0 / 3.0) * h_paso;
    PointXZ P1 = {x_ini, h_ctrl};
    PointXZ P2 = {P3.x + (P3.x - P0.x)*0.2, h_ctrl}; // P2 "empuja" hacia la dirección de stance

    if (phase <= 0.5) {
        // VUELO: 0.0 a 0.5 mapeado a 0.0 a 1.0
        double t = phase / 0.5;
        return cubic_bezier(P0, P1, P2, P3, t);
    } else {
        // APOYO: 0.5 a 1.0 mapeado a 0.0 a 1.0
        double t = (phase - 0.5) / 0.5;
        // Usamos una interpolación de coseno para que el inicio y fin del apoyo sean suaves
        // Esto evita el golpe seco al empezar a arrastrar.
        double s = (1.0 - cos(t * M_PI)) / 2.0; 
        return { P3.x + (P0.x - P3.x) * s, z_suelo };
    }
}

// --- IK ---
LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha) {
    double y_l = es_pata_derecha ? -y : y;
    double d_yz = sqrt(y_l*y_l + z*z);
    double D = std::max(d_yz, L1 + 1e-6);
    double cos1 = L1 / D;
    if (std::abs(cos1) > 1.0) return {0,0,0, false};
    double th1 = atan2(z, y_l) + acos(cos1);
    double R = -sqrt(std::max(0.0, d_yz*d_yz - L1*L1));
    double H = sqrt(x*x + R*R);
    double cos2 = (L2*L2 + L3*L3 - H*H) / (2 * L2 * L3);
    if (std::abs(cos2) > 1.0) return {0,0,0, false};
    double phi1 = acos(cos2), phi2 = atan2(R, x);
    double sin3 = (L3 * sin(phi1)) / (H + 1e-9);
    if (std::abs(sin3) > 1.0) return {0,0,0, false};
    double phi3 = asin(sin3);
    auto wrap = [](double a) {
        a = fmod(a + M_PI, 2 * M_PI);
        if (a < 0) a += 2 * M_PI;
        return (a - M_PI) * 180.0 / M_PI;
    };
    if (es_pata_derecha) th1 = -th1;
    return {wrap(th1), wrap(phi2 - phi3 + M_PI/2.0), wrap(M_PI - phi1), true};
}

// ============================================================
// MAIN CON FILTRADO DE VELOCIDAD
// ============================================================
int main() {
    std::signal(SIGINT, SignalHandler);

    int max_ciclos = 10;
    double step_duration = 1.0; 
    double x_start = -50.0, step_len = 100.0, step_h = 50.0;

    const char* shm_name = "/rex_shm";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(SharedData));
    SharedData* shm_ptr = static_cast<SharedData*>(mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    std::cout << "\nPresiona ENTER para comenzar...";
    std::cin.get();

    auto start_time = std::chrono::steady_clock::now();
    double loop_dt = 0.005; // 200 Hz
    double lookahead = 0.001;
    
    // Variables para filtro de velocidad (estilo exponential moving average)
    double v_filt[3] = {0,0,0};
    double alpha_v = 0.7; // Factor de suavizado (0.1 muy filtrado, 1.0 sin filtro)

    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        if (elapsed > (max_ciclos * step_duration)) break;

        double phase = (elapsed / step_duration) - std::floor(elapsed / step_duration);
        
        // Evitamos que el lookahead salte la frontera del 0.5 para el cálculo de velocidad
        double next_phase = phase + (lookahead / step_duration);
        if ((phase <= 0.5 && next_phase > 0.5) || (phase > 0.99)) {
            next_phase = phase; // En la frontera, mantenemos velocidad constante un instante
        }

        PointXZ p_curr = get_trajectory_point(phase, x_start, step_len, -330.0, step_h);
        PointXZ p_next = get_trajectory_point(next_phase, x_start, step_len, -330.0, step_h);

        LegAngles ik_curr = solve_IK(p_curr.x / 1000.0, L1, p_curr.z / 1000.0, false);
        LegAngles ik_next = solve_IK(p_next.x / 1000.0, L1, p_next.z / 1000.0, false);

        if (ik_curr.valid && ik_next.valid) {
            shm_ptr->angles[0][0] = ik_curr.th1;
            shm_ptr->angles[0][1] = ik_curr.th2;
            shm_ptr->angles[0][2] = ik_curr.th3;

            // Cálculo de velocidad con filtro de paso bajo
            double raw_v1 = (ik_next.th1 - ik_curr.th1) / lookahead;
            double raw_v2 = (ik_next.th2 - ik_curr.th2) / lookahead;
            double raw_v3 = (ik_next.th3 - ik_curr.th3) / lookahead;

            v_filt[0] = alpha_v * raw_v1 + (1.0 - alpha_v) * v_filt[0];
            v_filt[1] = alpha_v * raw_v2 + (1.0 - alpha_v) * v_filt[1];
            v_filt[2] = alpha_v * raw_v3 + (1.0 - alpha_v) * v_filt[2];

            shm_ptr->velocities[0][0] = v_filt[0];
            shm_ptr->velocities[0][1] = v_filt[1];
            shm_ptr->velocities[0][2] = v_filt[2];
        }

        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    std::memset(shm_ptr, 0, sizeof(SharedData));
    return 0;
}
