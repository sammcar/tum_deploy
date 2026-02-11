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

// ============================================================
// 1. ESTRUCTURA COMPARTIDA
// ============================================================
#pragma pack(push, 1)
struct SharedData {
    double angles[4][3]; 
    double times[4][3];  
    bool is_walking;     
};
#pragma pack(pop)

struct LegAngles { double th1, th2, th3; };

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

const double L1 = 0.093, L2 = 0.147, L3 = 0.230;
const double MIN_DIST_MM = 0.5; 

// ============================================================
// 2. CINEMÁTICA INVERSA
// ============================================================
double wrap_to_pi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha) {
    double y_local = es_pata_derecha ? -y : y;
    double dist_yz = sqrt(pow(y_local, 2) + pow(z, 2));
    double D = std::max(dist_yz, L1 + 1e-6);
    double theta1 = atan2(z, y_local) + acos(std::max(-1.0, std::min(1.0, L1 / D)));
    double R_term = pow(dist_yz, 2) - pow(L1, 2);
    double R = -sqrt(std::max(0.0, R_term));
    double H = sqrt(pow(x, 2) + pow(R, 2));
    double cos_phi1 = (pow(L2, 2) + pow(L3, 2) - pow(H, 2)) / (2 * L2 * L3);
    double phi1 = acos(std::max(-1.0, std::min(1.0, cos_phi1)));
    double theta3 = M_PI - phi1; 
    double phi2 = atan2(R, x);
    double sin_phi3 = (L3 * sin(phi1)) / (H + 1e-9);
    double phi3 = asin(std::max(-1.0, std::min(1.0, sin_phi3)));
    double theta2 = phi2 - phi3;

    if (es_pata_derecha) theta1 = -theta1;
    return {
        wrap_to_pi(theta1) * 180.0 / M_PI,
        wrap_to_pi(theta2 + M_PI/2.0) * 180.0 / M_PI,
        wrap_to_pi(theta3) * 180.0 / M_PI
    };
}

// ============================================================
// 3. GENERADOR DE ELIPSE SEGURO (CON SUAVIZADO Z)
// ============================================================
struct PointXZ { double x, z; };

double dist_sq(const PointXZ& p1, const PointXZ& p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.z - p2.z, 2);
}

// --- NUEVA FUNCIÓN: SUAVIZADO (Smoothstep) ---
double ease_in_out(double t) {
    // Curva S cúbica: Empieza lento, acelera, termina lento
    return t * t * (3.0 - 2.0 * t);
}

std::vector<PointXZ> generar_elipse_segura(double x_ini, double x_fin, double z_suelo, double h_paso, int pts_vuelo, int pts_apoyo) {
    std::vector<PointXZ> trayectoria;
    double largo = std::abs(x_fin - x_ini);
    double radio_h = largo / 2.0;
    double centro_x = (x_ini + x_fin) / 2.0;
    
    int descartados = 0;

    auto intentar_agregar = [&](PointXZ p) {
        if (trayectoria.empty()) {
            trayectoria.push_back(p);
        } else {
            if (dist_sq(trayectoria.back(), p) >= pow(MIN_DIST_MM, 2)) {
                trayectoria.push_back(p);
            } else {
                descartados++;
            }
        }
    };

    // --- FASE 1: VUELO CON SUAVIZADO ---
    for (int i = 0; i <= pts_vuelo; ++i) { 
        double t_lin = (double)i / (double)pts_vuelo; // Tiempo lineal 0.0 -> 1.0
        
        // APLICAMOS EL SUAVIZADO AQUÍ
        // Esto hace que t_smooth avance muy poco al inicio y al final
        double t_smooth = ease_in_out(t_lin); 

        // Usamos t_smooth para calcular el ángulo
        double theta = M_PI - (M_PI * t_smooth); 
        
        PointXZ pt = { 
            centro_x + radio_h * cos(theta), 
            z_suelo + h_paso * sin(theta) 
        };
        intentar_agregar(pt);
    }

    // --- FASE 2: APOYO (LINEAL) ---
    // En el suelo mantenemos velocidad constante para tracción
    for (int i = 1; i <= pts_apoyo; ++i) {
        double t = (double)i / (double)pts_apoyo;
        PointXZ pt = { 
            x_fin - (largo * t), 
            z_suelo 
        };
        intentar_agregar(pt);
    }

    std::cout << "🔍 [INFO] Puntos totales: " << trayectoria.size() 
              << " (Descartados: " << descartados << ")" << std::endl;

    return trayectoria;
}

// ============================================================
// 4. MAIN
// ============================================================
int main() {
    std::signal(SIGINT, SignalHandler);

    int max_ciclos;
    int pts_up, pts_down;
    double x_start, step_length, step_height;
    double periodo;

    std::cout << "========================================" << std::endl;
    std::cout << "   🛠️  CONFIGURACIÓN CON SUAVIZADO Z   " << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::cout << "1. CONFIGURACIÓN DE TIEMPO:" << std::endl;
    std::cout << "   🔢 Ciclos a ejecutar: "; std::cin >> max_ciclos;
    std::cout << "   ⏱️  Periodo del paso (segundos) [Ej. 1.0]: "; std::cin >> periodo;
    
    std::cout << "\n2. DENSIDAD DE TRAYECTORIA:" << std::endl;
    std::cout << "   📈 Puntos VUELO (Arriba) [Ej. 40]: "; std::cin >> pts_up;
    std::cout << "   📉 Puntos APOYO (Abajo)  [Ej. 40]: "; std::cin >> pts_down;

    std::cout << "\n3. GEOMETRÍA:" << std::endl;
    std::cout << "   📍 X inicio (mm) [Ej. -30]: "; std::cin >> x_start;
    std::cout << "   📏 Largo (mm) [Ej. 60]: "; std::cin >> step_length;
    std::cout << "   🔼 Altura (mm) [Ej. 50]: "; std::cin >> step_height;
    std::cout << "----------------------------------------" << std::endl;

    if (periodo <= 0.1) periodo = 0.1;
    if (pts_up < 5) pts_up = 5;
    if (pts_down < 5) pts_down = 5;

    const char* shm_name = "/rex_shm";
    const size_t shm_size = sizeof(SharedData);

    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) { perror("shm_open"); return 1; }
    ftruncate(shm_fd, shm_size);
    void* ptr = mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    SharedData* shm_ptr = static_cast<SharedData*>(ptr);
    std::memset(shm_ptr, 0, shm_size);

    double x_fin = x_start + step_length;
    auto trayectoria = generar_elipse_segura(x_start, x_fin, -330.0, step_height, pts_up, pts_down);
    
    int num_pts = trayectoria.size();
    if (num_pts == 0) return 1;

    double dt = periodo / (double)num_pts;
    double freq_resultante = 1.0 / dt;

    std::cout << "🚀 DATOS CALCULADOS:" << std::endl;
    std::cout << "   🔹 Puntos Totales: " << num_pts << std::endl;
    std::cout << "   🔹 Tiempo por punto (dt): " << std::fixed << std::setprecision(4) << dt << " s" << std::endl;
    std::cout << "   🔹 Frecuencia Efectiva: " << freq_resultante << " Hz" << std::endl;

    if (freq_resultante > 200.0) std::cout << "⚠️  ADVERTENCIA: Frecuencia muy alta." << std::endl;

    std::cout << "Presiona ENTER para iniciar..." << std::endl;
    std::cin.ignore(); std::cin.get();

    int total_iteraciones = num_pts * max_ciclos;

    for (int i = 0; i < total_iteraciones && g_running; ++i) {
        auto t_start = std::chrono::steady_clock::now();
        
        int idx = i % num_pts;

        LegAngles ik = solve_IK(trayectoria[idx].x / 1000.0, 0.093, trayectoria[idx].z / 1000.0, false);

        shm_ptr->angles[0][0] = ik.th1;
        shm_ptr->angles[0][1] = ik.th2;
        shm_ptr->angles[0][2] = ik.th3;

        for (int p = 0; p < 4; ++p) {
            if (p > 0) { 
                shm_ptr->angles[p][0] = 0.0; shm_ptr->angles[p][1] = 0.0; shm_ptr->angles[p][2] = 0.0;
            }
            // Mantenemos tu corrección del 1.2x para evitar micro-frenados
            for (int m = 0; m < 3; ++m) shm_ptr->times[p][m] = dt * 1.2;
        }
        shm_ptr->is_walking = true;

        if (idx % 10 == 0 || idx == 0) {
            int ciclo_actual = (i / num_pts) + 1;
            std::string fase = (idx < pts_up) ? "VUELO" : "APOYO";
            printf("\r[%s] Ciclo: %d/%d | T: %.2fs | X: %5.1f Z: %5.1f   ", 
                   fase.c_str(), ciclo_actual, max_ciclos, (idx * dt), trayectoria[idx].x, trayectoria[idx].z);
            std::fflush(stdout);
        }

        auto t_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t_end - t_start;
        if (elapsed.count() < dt) {
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed.count()));
        }
    }

    std::memset(shm_ptr, 0, shm_size);
    shm_ptr->is_walking = false;
    munmap(ptr, shm_size);
    close(shm_fd);
    std::cout << "\n✅ Finalizado." << std::endl;
    return 0;
}
