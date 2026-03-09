#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <cmath>
#include <array>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

// --- ESTRUCTURAS DE MEMORIA COMPARTIDA ---
struct CommandData {
    double foot_positions[4][3];
    double velocities[4][3];
    double desired_accel[4][3];
    double kp_scale[4][3];
    double kd_scale[4][3];
    double transition_time;
    bool is_stance[4];
    bool is_walking;
};

struct TelemetryData {
    double measured_angles[4][3];
    double measured_velocities[4][3];
    double measured_torques[4][3];
    double temperature[4][3];
    uint64_t timestamp_us;
    bool fault_code[4];
};

struct TrajectoryPoint {
    double x, z;          // Posición
    double vx, vz;        // Velocidad
    double ax, az;        // Aceleración
};

class MemoryManager {
private:
    template <typename T>
    void open_shm(const char *name, int &fd, T *&ptr) {
        fd = shm_open(name, O_RDWR | O_CREAT, 0666);
        if (fd == -1) return;
        struct stat shm_stat;
        fstat(fd, &shm_stat);
        if (shm_stat.st_size != sizeof(T)) {
            if (ftruncate(fd, sizeof(T)) == -1) return;
        }
        void *map = mmap(0, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (map == MAP_FAILED) return;
        ptr = static_cast<T *>(map);
    }

public:
    CommandData *cmd = nullptr;
    TelemetryData *tel = nullptr;
    int fd_cmd = -1, fd_tel = -1;

    MemoryManager() {
        open_shm<CommandData>("/rex_cmd", fd_cmd, cmd);
        open_shm<TelemetryData>("/rex_tel", fd_tel, tel);
    }
    ~MemoryManager() {
        if (cmd) munmap(cmd, sizeof(CommandData));
        if (tel) munmap(tel, sizeof(TelemetryData));
        if (fd_cmd != -1) close(fd_cmd);
        if (fd_tel != -1) close(fd_tel);
    }
    bool is_valid() { return cmd != nullptr && tel != nullptr; }
};

// --- NUEVA CLASE GENERADORA DE BÉZIER ---
class BezierGait {
private:
    double lerp(double a, double b, double t) { return a + t * (b - a); }

    TrajectoryPoint calculate_bezier(double t, double duration, 
                                     double x0, double z0,
                                     double x1, double z1,
                                     double x2, double z2,
                                     double x3, double z3) 
    {
        if (t < 0) t = 0; 
        if (t > 1) t = 1;
        
        double t2 = t * t; double t3 = t2 * t;
        double u = 1.0 - t; double u2 = u * u; double u3 = u2 * u;

        TrajectoryPoint res;
        
        // Posición
        res.x = u3*x0 + 3*u2*t*x1 + 3*u*t2*x2 + t3*x3;
        res.z = u3*z0 + 3*u2*t*z1 + 3*u*t2*z2 + t3*z3;

        // Velocidad y Aceleración analítica
        if (duration > 1e-5) {
            double dxdt = 3*u2*(x1 - x0) + 6*u*t*(x2 - x1) + 3*t2*(x3 - x2);
            double dzdt = 3*u2*(z1 - z0) + 6*u*t*(z2 - z1) + 3*t2*(z3 - z2);
            res.vx = dxdt / duration;
            res.vz = dzdt / duration;

            double d2xdt2 = 6*u*(x2 - 2*x1 + x0) + 6*t*(x3 - 2*x2 + x1);
            double d2zdt2 = 6*u*(z2 - 2*z1 + z0) + 6*t*(z3 - 2*z2 + z1);
            res.ax = d2xdt2 / (duration * duration);
            res.az = d2zdt2 / (duration * duration);
        } else { 
            res.vx = 0; res.vz = 0; res.ax = 0; res.az = 0; 
        }
        return res;
    }

public:
    TrajectoryPoint get_step_trajectory(double phase, double step_period, 
                                        double origin_x, double origin_z, 
                                        double step_len, double step_height) {
        phase = phase - std::floor(phase);
        
        double x_start = origin_x - step_len / 2.0;
        double x_end   = origin_x + step_len / 2.0;
        double z_ground = origin_z;

        // --- STANCE (0.5 - 1.0) ---
        if (phase >= 0.5) {
            double t = (phase - 0.5) * 2.0;
            TrajectoryPoint p;
            p.x = lerp(x_end, x_start, t); 
            p.z = z_ground;
            p.vx = (x_start - x_end) / (step_period * 0.5); // Velocidad lineal constante
            p.vz = 0.0;
            p.ax = 0.0; p.az = 0.0;
            return p;
        }

        // --- SWING (0.0 - 0.5) ---
        double t = phase * 2.0;
        double lift_bias = 0.2; 
        
        return calculate_bezier(t, step_period * 0.5,
            x_start, z_ground,
            x_start + (step_len * lift_bias), z_ground + (step_height * 1.3),
            x_end - (step_len * lift_bias), z_ground + (step_height * 1.3),
            x_end, z_ground
        );
    }
};

// --- GLOBALES Y SEÑALES ---
std::atomic<bool> publisher_running{true};
void SigIntHandler(int) { publisher_running = false; }

struct Point3D {
    double x, y, z;
    Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

const Point3D LEGS_STAND_XYZ[4] = {
    Point3D(0.01,  0.093, -0.306), // FL
    Point3D(0.01, -0.093, -0.306), // FR
    Point3D(0.01,  0.093, -0.306), // BL
    Point3D(0.01, -0.093, -0.306)  // BR
};

int main() {
    std::signal(SIGINT, SigIntHandler);

    double periodo, step_length, step_height, duracion_test;
    std::cout << ">> Periodo (seg): "; std::cin >> periodo;
    std::cout << ">> Longitud paso (m): "; std::cin >> step_length;
    std::cout << ">> Altura paso (m): "; std::cin >> step_height;
    std::cout << ">> Duración test (seg): "; std::cin >> duracion_test;
    
    // Mantenemos tu ajuste original
    step_length = step_length / 2.0; 

    MemoryManager memory;
    if (!memory.is_valid()) return 1;

    // --- FASE DE PRE-CARGA (STAND) ---
    std::cout << "\n[SETUP] Posicionando en STAND (Transition: 2s)..." << std::endl;
    memory.cmd->is_walking = false;
    memory.cmd->transition_time = 2.0;

    for (int p = 0; p < 4; p++) {
        memory.cmd->foot_positions[p][0] = LEGS_STAND_XYZ[p].x;
        memory.cmd->foot_positions[p][1] = LEGS_STAND_XYZ[p].y;
        memory.cmd->foot_positions[p][2] = LEGS_STAND_XYZ[p].z;
        for (int j = 0; j < 3; j++) {
            memory.cmd->kp_scale[p][j] = 1.0;
            memory.cmd->kd_scale[p][j] = 1.0;
        }
        memory.cmd->is_stance[p] = true;
    }

    for (int i = 3; i > 0; i--) {
        std::cout << "Iniciando en " << i << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // --- CICLO DE MARCHA DINÁMICO ---
    const double DT = 0.005; // 200Hz
    BezierGait bezier;
    const int PATA = 1; // Front-Right (FR)

    double t_ciclo = 0.0;
    auto t_inicio = std::chrono::steady_clock::now();
    auto proximo_ciclo = t_inicio;

    std::cout << "[RUN] Marcha iniciada." << std::endl;

    while (publisher_running) {
        memory.cmd->is_walking = true;
        
        auto ahora = std::chrono::steady_clock::now();
        double total_elap = std::chrono::duration<double>(ahora - t_inicio).count();
        if (total_elap >= duracion_test) break;

        // 1. Calculamos la fase (0.0 a 1.0)
        double phase = t_ciclo / periodo;

        // 2. Obtenemos todo (posición y velocidad) de nuestra nueva clase
        TrajectoryPoint tp = bezier.get_step_trajectory(
            phase, 
            periodo, 
            LEGS_STAND_XYZ[PATA].x, 
            LEGS_STAND_XYZ[PATA].z, 
            step_length, 
            step_height
        );

        // 3. ESCRIBIMOS EN MEMORIA COMPARTIDA
        memory.cmd->foot_positions[PATA][0] = tp.x;
        memory.cmd->foot_positions[PATA][1] = LEGS_STAND_XYZ[PATA].y; // Y constante
        memory.cmd->foot_positions[PATA][2] = tp.z;

        memory.cmd->velocities[PATA][0] = tp.vx;
        memory.cmd->velocities[PATA][1] = 0.0;
        memory.cmd->velocities[PATA][2] = tp.vz;

        memory.cmd->desired_accel[PATA][0] = tp.ax;
        memory.cmd->desired_accel[PATA][1] = 0.0;
        memory.cmd->desired_accel[PATA][2] = tp.az;

        memory.cmd->is_stance[PATA] = (phase >= 0.5);

        // --- SINCRONIZACIÓN ---
        t_ciclo += DT;
        if (t_ciclo >= periodo) t_ciclo = 0.0;

        proximo_ciclo += std::chrono::microseconds(5000); // 5ms (200Hz)
        std::this_thread::sleep_until(proximo_ciclo);
    }

    memory.cmd->is_walking = false;
    std::cout << "[FIN] Robot en modo STANDBY." << std::endl;
    return 0;
}
