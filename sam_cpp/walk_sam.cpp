#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// ==========================================
// 1. ESTRUCTURA DE MEMORIA (COMPATIBLE CON SEBAS)
// ==========================================
#pragma pack(push, 1)
struct SharedData {
    double angles[4][3];  // [FL, FR, BL, BR][Hip, Thigh, Calf] (Grados)
    double times[4][3];   // Tiempos de interpolación (Segundos)
    bool is_walking;      // Flag de estado
};
#pragma pack(pop)

const char* SHM_NAME = "/rex_shm"; // Nombre usado en código de Sebas
const int SHM_SIZE = sizeof(SharedData);

volatile sig_atomic_t stop_signal_received = 0;
void signal_handler(int signal) { stop_signal_received = 1; }

// ==========================================
// 2. UTILIDADES MATEMÁTICAS & SUAVIZADO
// ==========================================
double wrap_to_pi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

// Curva S Cúbica (Smoothstep) - El secreto de la suavidad de Sebas
double ease_in_out(double t) {
    if (t <= 0) return 0;
    if (t >= 1) return 1;
    return t * t * (3.0 - 2.0 * t);
}

struct LegAngles { double th1, th2, th3; };

// ==========================================
// 3. CONFIGURACIÓN FÍSICA
// ==========================================
const double L1 = 0.093;
const double L2 = 0.147;
const double L3 = 0.230;

std::map<std::string, Vector3d> STAND_XYZ = {
    {"FL", Vector3d(-0.008, 0.093, -0.316)}, {"FR", Vector3d(-0.008, -0.093, -0.316)},
    {"BL", Vector3d(-0.043, 0.093, -0.316)}, {"BR", Vector3d(-0.043, -0.093, -0.316)}
};

std::map<std::string, Vector3d> INIT_XYZ = {
    {"FL", Vector3d(0.0, 0.093, -0.377)}, {"FR", Vector3d(0.0, -0.093, -0.377)},
    {"BL", Vector3d(0.0, 0.093, -0.377)}, {"BR", Vector3d(0.0, -0.093, -0.377)}
};

// ==========================================
// 4. CINEMÁTICA INVERSA (VERSIÓN SEBAS)
// ==========================================
// Retorna GRADOS directamente y maneja offsets de hardware
LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha) {
    double y_local = es_pata_derecha ? -y : y;
    double dist_yz = sqrt(pow(y_local, 2) + pow(z, 2));
    double D = std::max(dist_yz, L1 + 1e-6);
    double theta1 = atan2(z, y_local) + acos(std::max(-1.0, std::min(1.0, L1 / D)));

    double R_term = pow(dist_yz, 2) - pow(L1, 2);
    // std::max(0.0, ...) previene NaN por errores de flotante
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

    // Conversión a Grados y Offsets de Hardware (Igual que código Sebas)
    return {
        wrap_to_pi(theta1) * 180.0 / M_PI,
        wrap_to_pi(theta2 + M_PI/2.0) * 180.0 / M_PI,
        wrap_to_pi(theta3) * 180.0 / M_PI
    };
}

// ==========================================
// 5. CEREBRO DE MARCHA (V5 + SUAVIZADO)
// ==========================================
class GaitGenerator {
public:
    std::map<std::string, Vector3d> feet_state;
    std::map<std::string, Vector3d> phase_start;
    std::map<std::string, Vector3d> phase_end;
    std::map<std::string, Vector3d> default_stance; 
    
    double walking_height; 
    Vector3d body_shift;
    Vector3d target_shift_val;
    Vector3d current_vel;
    Vector3d next_vel;
    
    int swing_points = 20;   
    int shift_points = 20;   
    int stance_points = 20;  
    
    double swing_time = 1.0; 
    double shift_time = 1.5; 
    double lift_height = 0.05;
    double lean_factor = 0.015;

    double step_timer = 0.0;
    std::vector<std::string> sequence = {"FL", "BR", "FR", "BL"};
    int current_seq_idx = 0;
    int step_queue = 0;
    string state = "STAND";

    GaitGenerator() {
        double raw_h = std::abs(STAND_XYZ["FL"][2]);
        walking_height = raw_h * 0.95; // Corrección de altura V5

        for (auto const& [key, val] : STAND_XYZ) {
            Vector3d flat_pos = val;
            flat_pos[2] = 0.0; 
            default_stance[key] = flat_pos;
            feet_state[key] = flat_pos;
            phase_start[key] = flat_pos;
            phase_end[key] = flat_pos;
        }

        body_shift = Vector3d(0,0,0);
        target_shift_val = Vector3d(0,0,0);
        current_vel = Vector3d(0,0,0);
        next_vel = Vector3d(0,0,0);
    }

    // --- FUNCIÓN DE TRAYECTORIA MEJORADA (Con ease_in_out) ---
    Vector3d get_staircase_pos(Vector3d start, Vector3d end, double progress, int points_count, double lift_z = 0.0) {
        if (points_count <= 1) return start;
        
        // 1. Discretización (Lógica V5)
        int step_idx = (int)(progress * points_count);
        if (step_idx >= points_count) step_idx = points_count - 1;
        double t_linear = (double)step_idx / (double)(points_count - 1);

        // 2. Suavizado (Lógica Sebas)
        // Aplicamos la curva S al tiempo para que el movimiento no sea robótico
        double t_smooth = ease_in_out(t_linear);
        
        // 3. Interpolación
        Vector3d current_pos = start + (end - start) * t_smooth;

        // 4. Arco Z (También suavizado por t_smooth)
        if (lift_z > 0.001) {
            double z_offset = sin(t_smooth * M_PI) * lift_z;
            if (z_offset < 0) z_offset = 0;
            current_pos[2] = z_offset; 
        } else {
            current_pos[2] = 0.0;
        }
        return current_pos;
    }

    void trigger_steps(int n_steps, Vector3d velocity_override) {
        if (n_steps > 0) {
            step_queue += n_steps;
            state = "SEQUENCE_RUNNING";
            double speed_scale_lin = 0.12; 
            double speed_scale_rot = 0.4;
            double vx = velocity_override[0];
            if (velocity_override.norm() < 0.01) vx = 1.0; 
            next_vel = Vector3d(vx * speed_scale_lin, velocity_override[1] * speed_scale_lin, velocity_override[2] * speed_scale_rot);
            if (current_vel.norm() < 0.001) current_vel = next_vel;
        }
    }

    void _reset_to_stand() {
        state = "STAND";
        step_queue = 0;
        body_shift = Vector3d(0,0,0);
        current_vel = Vector3d(0,0,0);
        for (auto const& [key, val] : default_stance) {
            phase_start[key] = feet_state[key];
            phase_end[key] = default_stance[key];
        }
    }

    void update(double dt) {
        if (state == "STAND" && step_queue > 0) state = "SEQUENCE_RUNNING";
        if (state != "SEQUENCE_RUNNING") return;

        double t_total = swing_time + shift_time;
        step_timer += dt;
        std::string active_leg = sequence[current_seq_idx];
        Vector3d step_dist_vector = current_vel * t_total;

        // LATCHING (Memoria de paso)
        if (step_timer <= dt * 1.5) {
            for (auto const& [leg, val] : default_stance) {
                Vector3d home = default_stance[leg]; 
                phase_start[leg] = feet_state[leg];
                if (leg == active_leg) {
                    phase_end[leg] = home + (step_dist_vector * 1.5);
                    phase_end[leg][2] = 0.0; 
                } else {
                    phase_end[leg] = phase_start[leg] - step_dist_vector;
                    phase_end[leg][2] = 0.0;
                }
            }
        }
        
        // CALCULO CONTINUO DE BALANCEO (V5 Stable Logic)
        double sx = (active_leg == "FL" || active_leg == "FR") ? 1.0 : -1.0;
        double sy = (active_leg == "FL" || active_leg == "BL") ? 1.0 : -1.0;
        target_shift_val = Vector3d(-sx * lean_factor, -sy * lean_factor, 0.0);

        Vector3d zero(0,0,0);
        double p_stance = step_timer / t_total;
        
        // Ejecución (Usando get_staircase_pos suavizada)
        for (auto const& [leg, val] : default_stance) {
            if (leg == active_leg) continue;
            feet_state[leg] = get_staircase_pos(phase_start[leg], phase_end[leg], p_stance, stance_points, 0.0);
        }

        if (step_timer < shift_time) {
            double p_sh = step_timer / shift_time;
            body_shift = get_staircase_pos(zero, target_shift_val, p_sh, shift_points);
            feet_state[active_leg] = phase_start[active_leg];
            feet_state[active_leg][2] = 0.0;
        } else {
            double time_in_return = step_timer - shift_time;
            double p_sh = time_in_return / swing_time;
            body_shift = get_staircase_pos(target_shift_val, zero, p_sh, shift_points);
            
            double p_swing = (step_timer - shift_time) / swing_time;
            feet_state[active_leg] = get_staircase_pos(phase_start[active_leg], phase_end[active_leg], p_swing, swing_points, lift_height);
        }

        if (step_timer >= t_total) {
            step_timer = 0.0;
            step_queue -= 1;
            feet_state[active_leg] = phase_end[active_leg];
            feet_state[active_leg][2] = 0.0;
            body_shift = Vector3d(0,0,0);
            current_seq_idx = (current_seq_idx + 1) % 4;
            current_vel = next_vel;
            if (step_queue <= 0) _reset_to_stand();
        }
    }
};

// ==========================================
// 6. MAIN (EJECUCIÓN TIPO SEBAS)
// ==========================================
int main() {
    signal(SIGINT, signal_handler);
    
    // Setup SHM
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) { perror("Error SHM"); return 1; }
    ftruncate(shm_fd, SHM_SIZE);
    void* ptr = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    SharedData* shm_data = (SharedData*)ptr;
    memset(shm_data, 0, SHM_SIZE); // Limpiar memoria al inicio
    
    GaitGenerator brain;
    
    // Tiempos
    // Usamos 20ms como base para el ciclo lógico, pero ajustaremos el sleep con chrono
    const double target_dt = 0.02; 

    // --- 1. POSE INICIAL (STAND) ---
    std::cout << "=== HYBRID ROBOT CONTROLLER ===" << std::endl;
    std::cout << ">>> ESTADO: STAND (Presiona ENTER para iniciar)" << std::endl;

    auto update_shm_from_brain = [&](Vector3d override_body_shift = Vector3d(0,0,0), bool use_override = false) {
        Vector3d body_shift = use_override ? override_body_shift : brain.body_shift;
        Vector3d total_body_pos = body_shift;
        total_body_pos[2] += brain.walking_height;

        std::string leg_order[4] = {"FL", "FR", "BL", "BR"};
        for(int i=0; i<4; i++) {
            std::string leg = leg_order[i];
            Vector3d foot_local = brain.feet_state[leg];
            
            // IK Input: Pie - Cuerpo
            Vector3d ik_target = foot_local - total_body_pos;
            bool is_right = (leg == "FR" || leg == "BR");
            
            LegAngles angs = solve_IK(ik_target[0], ik_target[1], ik_target[2], is_right);
            
            shm_data->angles[i][0] = angs.th1;
            shm_data->angles[i][1] = angs.th2;
            shm_data->angles[i][2] = angs.th3;
            
            // TIEMPO DE INTERPOLACIÓN (Factor de seguridad Sebas 1.2x)
            double safe_time = target_dt * 1.2;
            shm_data->times[i][0] = safe_time;
            shm_data->times[i][1] = safe_time;
            shm_data->times[i][2] = safe_time;
        }
    };

    // Calcular pose inicial
    brain.body_shift = Vector3d(0,0,0);
    update_shm_from_brain(); 
    shm_data->is_walking = true; // Activar servos

    std::cin.get(); // Esperar Enter

    // --- 2. CAMINATA (BUCLE DE TIEMPO REAL) ---
    brain.trigger_steps(999999, Vector3d(0.4, 0.0, 0.0)); // 0.4 de velocidad para empezar suave
    std::cout << ">>> RUNNING (Precision Loop)..." << std::endl;

    while (!stop_signal_received) {
        auto t_start = std::chrono::steady_clock::now();

        // A. Actualizar Lógica
        brain.update(target_dt);

        // B. Calcular IK y Escribir en SHM
        update_shm_from_brain();
        shm_data->is_walking = true;

        // C. Feedback Visual Consola
        printf("\r[RUN] BodyX: %.3f | FL_Z: %.3f | DT: %.3fs", 
               brain.body_shift[0], brain.feet_state["FL"][2], target_dt);
        fflush(stdout);

        // D. Control de Tiempo Preciso (Chrono)
        auto t_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t_end - t_start;
        double sleep_seconds = target_dt - elapsed.count();
        
        if (sleep_seconds > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds));
        }
    }

    // --- 3. APAGADO SEGURO (INIT POSE) ---
    std::cout << "\n\n>>> DETENIENDO... INIT POSE." << std::endl;
    
    // Resetear cerebro a stand
    brain._reset_to_stand(); 
    // Usar INIT_XYZ para la pose final
    for (auto const& [name, pos] : INIT_XYZ) {
        // Truco: Ponemos el feet_state en la posición INIT y el cuerpo en 0
        // Como INIT ya es relativa al hombro, target = INIT
        brain.feet_state[name] = pos; 
        brain.feet_state[name][2] += brain.walking_height; // Compensar resta interna
    }
    // Forzamos body_shift 0 para que el calculo final sea puro
    update_shm_from_brain(Vector3d(0,0,0), true);
    
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Dar tiempo a llegar
    
    shm_data->is_walking = false; // Desactivar
    munmap(ptr, SHM_SIZE);
    shm_unlink(SHM_NAME);
    close(shm_fd);
    
    std::cout << "Sistema finalizado." << std::endl;
    return 0;
}
