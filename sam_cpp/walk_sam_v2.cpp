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
    double angles[4][3];      // [Pata][Motor: 0=Abad, 1=Hip, 2=Knee]
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
// 2. LÓGICA DE TRAYECTORIA ELÍPTICA ASIMÉTRICA
// ============================================================
PointXZ get_trajectory_point(double phase, double x_ini, double step_len, double z_suelo, double h_paso, double r_z_stance) {
    double x_center = x_ini + (step_len / 2.0);
    double r_x = step_len / 2.0;
    double r_z_swing = h_paso; 

    // Wrap de fase para asegurar que esté entre 0 y 1
    phase = fmod(phase, 1.0);
    if (phase < 0) phase += 1.0;

    double angle = phase * 2.0 * M_PI;
    double current_r_z = (phase <= 0.5) ? r_z_swing : r_z_stance;

    double x = x_center - r_x * cos(angle);
    double z = z_suelo + current_r_z * sin(angle);

    return { x, z };
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
    return {wrap(th1), wrap(phi2 - phi3 + M_PI/2.0), wrap(M_PI - phi1), true};
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

int main() {
    std::signal(SIGINT, SignalHandler);

    int max_ciclos = pedir_dato<int>("Ciclos totales");
    double step_duration = pedir_dato<double>("Periodo del paso (s)"); 
    double x_start = pedir_dato<double>("X inicio (mm)");
    double step_len = pedir_dato<double>("Largo paso (mm)");
    double step_h = pedir_dato<double>("Altura paso (mm)");
    double step_pressure = pedir_dato<double>("Presión/Bajo suelo (mm)");

    const char* shm_name = "/rex_shm";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(SharedData));
    SharedData* shm_ptr = static_cast<SharedData*>(mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    std::cout << "\n[SISTEMA LISTO] Presiona ENTER para alinear las 4 patas y comenzar...";
    std::cin.get();

    // --- ALINEACIÓN INICIAL (Las 4 patas a posición de inicio de fase) ---
    for (int i = 1; i <= 100 && g_running; ++i) {
        auto t_frame = std::chrono::steady_clock::now();
        double t_lin = (double)i / 100.0;
        
        for (int p = 0; p < 4; p++) {
            // Alineamos todas a phase 0.0 (inicio de elipse)
            double p_phase = (p == 1 || p == 2) ? 0.5 : 0.0; // FR y BL empiezan desfasadas
            PointXZ p_init = get_trajectory_point(p_phase, x_start, step_len, -330.0, step_h, step_pressure);
            bool is_right = (p == 1 || p == 3);
            LegAngles ik = solve_IK(p_init.x / 1000.0, L1, p_init.z / 1000.0, is_right);
            
            shm_ptr->angles[p][0] = ik.th1 * t_lin;
            shm_ptr->angles[p][1] = ik.th2 * t_lin;
            shm_ptr->angles[p][2] = ik.th3 * t_lin;
        }
        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_frame + std::chrono::milliseconds(10));
    }

    // --- BUCLE DINÁMICO ---
    auto start_time = std::chrono::steady_clock::now();
    double loop_dt = 0.005; 
    double lookahead = 0.001;

    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        double total_time = max_ciclos * step_duration;
        if (elapsed > total_time) break;

        double base_phase = (elapsed / step_duration);
        
        double speed_factor = 1.0;
        double ramp_time = step_duration; 
        if (elapsed < ramp_time) speed_factor = (1.0 - cos((elapsed / ramp_time) * M_PI)) / 2.0;
        else if (elapsed > total_time - ramp_time) {
            double t_end = (total_time - elapsed) / ramp_time;
            speed_factor = (1.0 - cos(t_end * M_PI)) / 2.0;
        }

        // Calcular trayectoria para cada pata
        for (int p = 0; p < 4; p++) {
            // FL=0, FR=1, BL=2, BR=3
            // Parejas diagonales: (0 y 3) vs (1 y 2)
            double p_phase = base_phase;
            if (p == 1 || p == 2) p_phase += 0.5; // Desfase para la segunda diagonal
            
            double phase_curr = p_phase - std::floor(p_phase);
            double phase_next = phase_curr + (lookahead / step_duration) * speed_factor;

            PointXZ p_curr = get_trajectory_point(phase_curr, x_start, step_len, -330.0, step_h, step_pressure);
            PointXZ p_next = get_trajectory_point(phase_next, x_start, step_len, -330.0, step_h, step_pressure);

            bool is_right = (p == 1 || p == 3);
            LegAngles ik_curr = solve_IK(p_curr.x / 1000.0, L1, p_curr.z / 1000.0, is_right);
            LegAngles ik_next = solve_IK(p_next.x / 1000.0, L1, p_next.z / 1000.0, is_right);

            if (ik_curr.valid && ik_next.valid) {
                shm_ptr->angles[p][0] = ik_curr.th1;
                shm_ptr->angles[p][1] = ik_curr.th2;
                shm_ptr->angles[p][2] = ik_curr.th3;
                
                shm_ptr->velocities[p][0] = ((ik_next.th1 - ik_curr.th1) / lookahead) * speed_factor;
                shm_ptr->velocities[p][1] = ((ik_next.th2 - ik_curr.th2) / lookahead) * speed_factor;
                shm_ptr->velocities[p][2] = ((ik_next.th3 - ik_curr.th3) / lookahead) * speed_factor;
            }
        }

        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    std::memset(shm_ptr, 0, sizeof(SharedData));
    shm_ptr->is_walking = false;
    std::cout << "\n✅ Trote elíptico completado con éxito." << std::endl;
    return 0;
}
