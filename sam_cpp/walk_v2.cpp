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
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>

using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;

// ==========================================
// 1. ESTRUCTURAS DE MEMORIA COMPARTIDA Y GLOBALES
// ==========================================
struct CommandData {
    double angles[4][3]; // [Patas][Joints]
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

class MemoryManager {
private:
    template <typename T>
    void open_shm(const char *name, int &fd, T *&ptr) {
        fd = shm_open(name, O_RDWR | O_CREAT, 0666);
        if (fd == -1) {
            std::cerr << "[SHM] Error abriendo " << name << std::endl;
            return;
        }
        struct stat shm_stat;
        fstat(fd, &shm_stat);

        if (shm_stat.st_size != sizeof(T)) {
            ftruncate(fd, sizeof(T));
        }

        void *map = mmap(0, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (map == MAP_FAILED) {
            std::cerr << "[SHM] Error mapeando " << name << std::endl;
            return;
        }
        ptr = static_cast<T *>(map);

        if (shm_stat.st_size == 0 || shm_stat.st_size != sizeof(T)) {
            std::memset(ptr, 0, sizeof(T));
        }
    }

public:
    const char *cmd_name = "/rex_cmd";
    const char *tel_name = "/rex_tel";

    int fd_cmd = -1;
    int fd_tel = -1;

    CommandData *cmd = nullptr;
    TelemetryData *tel = nullptr;

    MemoryManager() {
        open_shm<CommandData>(cmd_name, fd_cmd, cmd);
        open_shm<TelemetryData>(tel_name, fd_tel, tel);
    }

    ~MemoryManager() {
        if (cmd) munmap(cmd, sizeof(CommandData));
        if (tel) munmap(tel, sizeof(TelemetryData));
        if (fd_cmd != -1) close(fd_cmd);
        if (fd_tel != -1) close(fd_tel);
    }

    bool is_valid() { return cmd != nullptr && tel != nullptr; }
};

// --- GLOBALES PARA MONITOREO ---
std::atomic<bool> g_running{true};
std::atomic<bool> g_print_telemetry{false}; 
std::mutex g_cmd_mutex;
int g_monitor_leg = 0; // Pata a monitorear (0: Delantera Izquierda)

// ==========================================
// 2. CONSTANTES FÍSICAS (ATOM-51)
// ==========================================
const double L1 = 93.0, L2 = 147.0, L3 = 230.0;

const Vector3d HIP_OFFSETS[4] = {
    Vector3d(187.5,  100.0, 0.0), Vector3d(187.5, -100.0, 0.0),
    Vector3d(-187.5, 100.0, 0.0), Vector3d(-187.5, -100.0, 0.0)
};

const Vector3d NEUTRAL_FEET[4] = {
    Vector3d(10.0,  103.0, -296.0), Vector3d(10.0,  -93.0, -296.0),
    Vector3d(-28.0, 103.0, -296.0), Vector3d(-28.0, -93.0, -296.0)
};

// ==========================================
// 3. CINEMÁTICA E IK
// ==========================================
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

void getModifiedFeet(double x_front, double x_rear, double y_offset, double z_add_front, double z_add_rear, Vector3d feet_array[4]) {
    for(int i=0; i<4; i++) feet_array[i] = NEUTRAL_FEET[i];
    for(int i=0; i<4; i++) feet_array[i].y() += (i%2 == 0) ? y_offset : -y_offset; 

    feet_array[0].x() += x_front; feet_array[1].x() += x_front;
    feet_array[0].z() += z_add_front; feet_array[1].z() += z_add_front;
    
    feet_array[2].x() += x_rear; feet_array[3].x() += x_rear;
    feet_array[2].z() += z_add_rear; feet_array[3].z() += z_add_rear;
}

// ==========================================
// 4. ENVÍO DE COMANDOS
// ==========================================
void SendPose(CommandData* cmd, Vector3d body_p, Vector3d body_r, const Vector3d feet[4], double seconds) {
    std::lock_guard<std::mutex> lock(g_cmd_mutex); // Bloquear mientras escribimos en shm
    
    cmd->transition_time = seconds;
    cmd->is_walking = false;
    Vector3d body_r_rad = body_r * M_PI / 180.0;

    for (int i = 0; i < 4; i++) {
        Vector3d local = ComputeWBC(feet[i], i, body_p, body_r_rad);
        solve_IK(local, (i == 1 || i == 3), cmd->angles[i]);
        cmd->is_stance[i] = true;
        
        for(int m=0; m<3; m++) {
            cmd->kp_scale[i][m] = 1.0; 
            cmd->kd_scale[i][m] = 1.0;
            cmd->velocities[i][m] = 5.0; 
            cmd->desired_accel[i][m] = 0.0;
        }
    }
    
    // El sleep se hace FUERA del lock para no colgar el hilo de monitoreo
    auto start = std::chrono::steady_clock::now();
    while(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void sendZeroAngles(CommandData* cmd, double seconds) {
    std::cout << "[WARN] Enviando 0.0 a todos los motores en " << seconds << "s..." << std::endl;
    std::lock_guard<std::mutex> lock(g_cmd_mutex);
    
    cmd->transition_time = seconds;
    cmd->is_walking = false;
    
    for (int p = 0; p < 4; ++p) {
        cmd->is_stance[p] = true;
        for (int m = 0; m < 3; ++m) {
            cmd->angles[p][m] = 0.0;
            cmd->velocities[p][m] = 2.0; 
            cmd->desired_accel[p][m] = 0.0;
            cmd->kp_scale[p][m] = 1.0;   
            cmd->kd_scale[p][m] = 1.0;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

// ==========================================
// 5. RUTINAS
// ==========================================
void flexion(CommandData* cmd) {
    std::cout << "\n[START] Iniciando rutina de Flexiones..." << std::endl;
    for (int i = 0; i < 4; ++i) {
        SendPose(cmd, Vector3d(0, 0, -70.0), Vector3d(0, 0, 0), NEUTRAL_FEET, 1.0); 
        SendPose(cmd, Vector3d(0, 0, 0.0),   Vector3d(0, 0, 0), NEUTRAL_FEET, 1.0); 
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void sentarse(CommandData* cmd) {
    std::cout << "\n[START] Rutina Sentarse..." << std::endl;
    Vector3d feet[4];
    getModifiedFeet(0, 0, 0, 0, 60, feet);
    SendPose(cmd, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), feet, 1.5);
    getModifiedFeet(-50, -50, 0, 0, 60, feet);
    SendPose(cmd, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), feet, 2.0);
    getModifiedFeet(-80, -80, 0, 60, 80, feet);
    SendPose(cmd, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), feet, 2.0);
    getModifiedFeet(-110, -110, 0, 160, 190, feet);
    SendPose(cmd, Vector3d(0, 0, 0.0), Vector3d(0, -3, 0), feet, 2.0);
    SendPose(cmd, Vector3d(20, 0, -40), Vector3d(0, -3, 0), feet, 2.5);
    getModifiedFeet(-110, -110, 20, 170, 170 + 30, feet);
    SendPose(cmd, Vector3d(20, 0, -40), Vector3d(0, 0, 0), feet, 1.0);
}

void levantarse(CommandData* cmd) {
    std::cout << "\n[START] Rutina Levantarse..." << std::endl;
    Vector3d feet[4];
    double x_inc = 110.0; 
    getModifiedFeet(-x_inc, -x_inc, 0, 180, 140, feet); 
    SendPose(cmd, Vector3d(0, 0, -20), Vector3d(0, 0, 0), feet, 1.5);
    getModifiedFeet(0, 0, 0, 0, 0, feet); 
    SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, 0, 0), feet, 1.5);
}

void baile(CommandData* cmd) {
    std::cout << "\n[START] Iniciando rutina de Baile..." << std::endl;
    Vector3d feet[4];
    for (int i = 0; i < 4; ++i) {
        SendPose(cmd, Vector3d(0, 0, 0), Vector3d(-10, -10, 5), NEUTRAL_FEET, 2.0);
        SendPose(cmd, Vector3d(0, 0, 0), Vector3d(10, 10, 5),  NEUTRAL_FEET, 2.0);
    }
    SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);
    getModifiedFeet(-30, -30, 0, 0, 0, feet);
    SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, 0, 0), feet, 2.0);
    SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);
    for (int i = 0; i < 4; ++i) {
        SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, -10, 0), NEUTRAL_FEET, 2.0);
        SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, 10, 0),  NEUTRAL_FEET, 2.0);
    }
    SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);
}

// ==========================================
// 6. HILO DE MONITOREO
// ==========================================
// ==========================================
// 6. HILO DE MONITOREO (CORREGIDO)
// ==========================================
void telemetry_monitor_thread(CommandData* cmd, TelemetryData* tel) {
    while (g_running) {
        if (g_print_telemetry) {
            double des_abad = 0, des_hip = 0, des_knee = 0;
            
            { // Bloqueo rápido para leer posición deseada sin cruces
                std::lock_guard<std::mutex> lock(g_cmd_mutex);
                des_abad = cmd->angles[g_monitor_leg][0];
                des_hip  = cmd->angles[g_monitor_leg][1];
                des_knee = cmd->angles[g_monitor_leg][2];
            }

            double act_abad = tel->measured_angles[g_monitor_leg][0];
            double act_hip  = tel->measured_angles[g_monitor_leg][1];
            double act_knee = tel->measured_angles[g_monitor_leg][2];

            double err_abad = des_abad - act_abad;
            double err_hip  = des_hip - act_hip;
            double err_knee = des_knee - act_knee;

            // Imprimir de forma segura y clara en una NUEVA línea
            std::cout << "[Monitor P" << g_monitor_leg << "] "
                      << "Error Abad: " << std::fixed << std::setprecision(2) << std::setw(7) << err_abad 
                      << " | Hip: "     << std::setw(7) << err_hip  
                      << " | Knee: "    << std::setw(7) << err_knee << " deg" 
                      << std::endl; // Cambiado a endl para salto de línea seguro
        }
        
        // Frecuencia más lenta (10Hz) para que puedas leer los números mientras bajan
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
}

// ==========================================
// 7. MAIN
// ==========================================
int main() {
    MemoryManager mem;
    if (!mem.is_valid()) {
        std::cerr << "[ERROR] Memoria compartida no inicializada. Asegúrate de correr el driver CAN." << std::endl;
        return 1;
    }

    CommandData* cmd = mem.cmd;
    TelemetryData* tel = mem.tel;

    // Lanzar hilo de monitoreo
    std::thread monitor(telemetry_monitor_thread, cmd, tel);

    int opcion = 0;
    while (true) {
        g_print_telemetry = false; // Pausa el print en el menú para no ensuciar la pantalla

        std::cout << "\n========== MENÚ DE RUTINAS ATOM-51 ==========" << std::endl;
        std::cout << "1. Rutina de Flexiones" << std::endl;
        std::cout << "2. Rutina de Baile" << std::endl;
        std::cout << "3. Rutina de Sentarse" << std::endl;
        std::cout << "4. Posición NEUTRAL (IK)" << std::endl;
        std::cout << "5. ÁNGULOS CERO (Reset Total)" << std::endl;
        std::cout << "6. Levantarse" << std::endl;
        std::cout << "7. Activar/Desactivar Monitoreo en vivo (Pata " << g_monitor_leg << ")" << std::endl;
        std::cout << "0. Salir" << std::endl;
        std::cout << "Selecciona una opción: ";
        
        std::cin >> opcion;
        if (std::cin.fail()) {
            std::cin.clear(); std::cin.ignore(1000, '\n');
            continue;
        }

        switch(opcion) {
            case 1: 
                g_print_telemetry = true; 
                flexion(cmd); 
                break; 
            case 2: 
                g_print_telemetry = true; 
                baile(cmd); 
                break;
            case 3: 
                g_print_telemetry = true; 
                sentarse(cmd); 
                break;
            case 4: 
                g_print_telemetry = true; 
                SendPose(cmd, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 1.0); 
                break;
            case 5: 
                g_print_telemetry = true; 
                sendZeroAngles(cmd, 2.0); 
                break;
            case 6: 
                g_print_telemetry = true; 
                levantarse(cmd); 
                break;
            case 7: 
                g_print_telemetry = !g_print_telemetry;
                std::cout << "Monitoreo " << (g_print_telemetry ? "ACTIVADO" : "DESACTIVADO") << ".\n";
                if (g_print_telemetry) {
                    std::cout << "Presiona Enter para detener el monitoreo y volver al menú...";
                    std::cin.ignore(1000, '\n');
                    std::cin.get();
                }
                break;
            case 0: 
                g_running = false;
                monitor.join();
                return 0;
            default: std::cout << "Opción inválida." << std::endl;
        }
    }
    
    g_running = false;
    monitor.join();
    return 0;
}
