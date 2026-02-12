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
// ESTRUCTURA DE MEMORIA
// ==========================================
#pragma pack(push, 1)
struct SharedData {
    double angles[4][3];
    double times[4][3];   
    bool is_walking;      
};
#pragma pack(pop)

const char* SHM_NAME = "/rex_shm"; 
const int SHM_SIZE = sizeof(SharedData);

volatile sig_atomic_t stop_signal_received = 0;
void signal_handler(int signal) { stop_signal_received = 1; }

// ==========================================
// UTILIDADES MATEMÁTICAS
// ==========================================
double wrap_to_pi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

double ease_in_out(double t) {
    if (t <= 0) return 0;
    if (t >= 1) return 1;
    return t * t * (3.0 - 2.0 * t);
}

// Perfil Trapezoidal (Sube, Mantiene, Baja) para el cuerpo
double trapezoid_profile(double t, double fade_in, double fade_out) {
    if (t < fade_in) return t / fade_in; 
    if (t > fade_out) return (1.0 - t) / (1.0 - fade_out); 
    return 1.0; 
}

struct LegAngles { double th1, th2, th3; };

// ==========================================
// CONFIGURACIÓN FÍSICA (DEFINICIONES USUARIO)
// ==========================================
const double L1 = 0.093;
const double L2 = 0.147;
const double L3 = 0.230;

// Posición de Marcha (Altura de trabajo)
std::map<std::string, Vector3d> STAND_XYZ = {
    {"FL", Vector3d(0.02, 0.093, -0.316)}, {"FR", Vector3d(0.02, -0.093, -0.316)},
    {"BL", Vector3d(-0.018, 0.093, -0.316)}, {"BR", Vector3d(-0.018, -0.093, -0.316)}
};

// std::map<std::string, Vector3d> STAND_XYZ = {
//     {"FL", Vector3d(-0.008, 0.093, -0.316)}, {"FR", Vector3d(-0.008, -0.093, -0.316)},
//     {"BL", Vector3d(-0.043, 0.093, -0.316)}, {"BR", Vector3d(-0.043, -0.093, -0.316)}
// };


// Posición de Reposo (Para apagar/descansar)
std::map<std::string, Vector3d> INIT_XYZ = {
    {"FL", Vector3d(0.0, 0.093, -0.377)}, {"FR", Vector3d(0.0, -0.093, -0.377)},
    {"BL", Vector3d(0.0, 0.093, -0.377)}, {"BR", Vector3d(0.0, -0.093, -0.377)}
};

// ==========================================
// CINEMÁTICA INVERSA
// ==========================================
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

    //if (es_pata_derecha) theta1 = -theta1;

    return {
        wrap_to_pi(theta1) * 180.0 / M_PI,
        wrap_to_pi(theta2 + M_PI/2.0) * 180.0 / M_PI,
        wrap_to_pi(theta3) * 180.0 / M_PI
    };
}

// ==========================================
// CEREBRO V7.2: SEGURIDAD + LIMP LOGIC
// ==========================================
class GaitGeneratorV7 {
public:
    std::map<std::string, Vector3d> current_feet_pos; 
    
    double step_length_x;  
    double step_height_z;  
    double period;         
    double lean_y; 
    double lean_x; 

    int pts_swing; 
    int pts_shift; 
    
    std::vector<std::string> leg_order = {"FL", "BR", "FR", "BL"};
    int seq_idx = 0;
    
    std::vector<Vector3d> body_offset_traj; 
    std::vector<Vector3d> swing_traj_abs;   
    
    int current_tick = 0;
    int total_ticks = 0;
    bool is_moving = false;

    GaitGeneratorV7() {
        step_length_x = 0.06; // Esta en metros, 0.07 son 7 cm 
        step_height_z = 0.12; 
        period = 1.0; // Segundos        
        pts_swing = 15;
        pts_shift = 15;
        
        lean_y = 0.05;  
        lean_x = 0.05; // Subir

        // INICIO SEGURO: Usamos STAND_XYZ como punto de partida
        for(auto const& [k, v] : STAND_XYZ) {
            current_feet_pos[k] = v;
        }
    }

    void plan_period() {
        current_tick = 0;
        total_ticks = pts_swing + pts_shift;
        
        body_offset_traj.clear();
        swing_traj_abs.clear();
        
        std::string active_leg = leg_order[seq_idx];

        // 1. DIRECCIÓN COMPENSACIÓN
        double shift_dir_x = 0.0;
        double shift_dir_y = 0.0;

        if (active_leg == "FL") { shift_dir_x = -1.0; shift_dir_y = -1.0; } 
        if (active_leg == "FR") { shift_dir_x = -1.0; shift_dir_y =  1.0; } 
        if (active_leg == "BL") { shift_dir_x =  1.0; shift_dir_y = -1.0; } 
        if (active_leg == "BR") { shift_dir_x =  1.0; shift_dir_y =  1.0; } 

        double body_fwd_per_period = step_length_x / 4.0;

        for(int i = 0; i < total_ticks; i++) {
            double t_total = (double)i / (double)(total_ticks - 1);
            double sway_weight = trapezoid_profile(t_total, 0.3, 0.7);
            
            Vector3d pt_body = Vector3d(0,0,0);
            
            // Avance lineal continuo
            pt_body[0] += body_fwd_per_period * t_total;
            
            // Compensación Trapezoidal (Shift)
            pt_body[0] += shift_dir_x * lean_x * sway_weight;
            pt_body[1] += shift_dir_y * lean_y * sway_weight;
            
            body_offset_traj.push_back(pt_body);
        }

        // 2. CURVA SWING (Desde posición actual en memoria)
        Vector3d start_pos = current_feet_pos[active_leg];
        Vector3d target_pos = start_pos;
        target_pos[0] += step_length_x; 

        for(int i = 0; i < pts_swing; i++) {
            double t_swing = (double)i / (double)(pts_swing - 1);
            double t_s = ease_in_out(t_swing);
            
            Vector3d pt_swing = start_pos + (target_pos - start_pos) * t_s;
            // Usamos STAND_XYZ[active_leg][2] como base Z para asegurar altura correcta
            pt_swing[2] = STAND_XYZ[active_leg][2] + sin(t_s * M_PI) * step_height_z;
            
            swing_traj_abs.push_back(pt_swing);
        }
        
        is_moving = true;
    }

    // double update() {
    //     if(!is_moving) return 0.02;

    //     std::string active_leg = leg_order[seq_idx];
        
    //     // CALCULAR DELTA 
    //     Vector3d current_body_off = body_offset_traj[current_tick];
    //     Vector3d prev_body_off = (current_tick == 0) ? Vector3d(0,0,0) : body_offset_traj[current_tick - 1];
    //     Vector3d delta_body = current_body_off - prev_body_off;

    //     // 1. APLICAR DELTA A TODAS (Arrastre)
    //     for(auto& [leg, pos] : current_feet_pos) {
    //         pos -= delta_body;
    //     }

    //     // 2. SWING OVERWRITE
    //     if(current_tick < pts_swing) {
    //         current_feet_pos[active_leg] = swing_traj_abs[current_tick] - current_body_off;
    //     }
        
    //     current_tick++;

    //     // FIN PERIODO
    //     if(current_tick >= total_ticks) {
    //         seq_idx = (seq_idx + 1) % 4; 
    //         plan_period(); 
    //         return 0.0; 
    //     }

    //     if(current_tick < pts_swing) return (period * 0.5) / pts_swing;
    //     else return (period * 0.5) / pts_shift;
    // }

    double update() {
        if(!is_moving) return 0.02;

        std::string active_leg = leg_order[seq_idx];
        
        // CALCULAR DELTA 
        Vector3d current_body_off = body_offset_traj[current_tick];
        Vector3d prev_body_off = (current_tick == 0) ? Vector3d(0,0,0) : body_offset_traj[current_tick - 1];
        Vector3d delta_body = current_body_off - prev_body_off;

        // 1. APLICAR DELTA A TODAS (Arrastre)
        for(auto& [leg, pos] : current_feet_pos) {
            pos -= delta_body;
        }
        if (current_tick < pts_shift) {
        }
        else {
            int swing_idx = current_tick - pts_shift;
            if (swing_idx < swing_traj_abs.size()) {
                current_feet_pos[active_leg] = swing_traj_abs[swing_idx] - current_body_off;
            }
        }
        
        current_tick++;

        // FIN DEL PERIODO
        if(current_tick >= total_ticks) {
            seq_idx = (seq_idx + 1) % 4; 
            plan_period(); 
            return 0.0; 
        }

        if(current_tick < pts_shift) return (period * 0.5) / pts_shift;
        else return (period * 0.5) / pts_swing;
    }

};

// ==========================================
// MAIN
// ==========================================
int main() {
    signal(SIGINT, signal_handler);
    
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) { perror("Error SHM"); return 1; }
    ftruncate(shm_fd, SHM_SIZE);
    void* ptr = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    SharedData* shm_data = (SharedData*)ptr;
    memset(shm_data, 0, SHM_SIZE);

    GaitGeneratorV7 brain;
    
    // Configuración
    // brain.period = 0.5;
    // brain.step_length_x = 0.07; 
    // brain.pts_swing = 20;
    // brain.pts_shift = 20;
    
    // brain.lean_y = 0.015; 
    // brain.lean_x = 0.008; 

    // --- FASE 1: INICIO SEGURO (STAND) ---
    std::cout << "=== REX V7.2: SAFETY START ===" << std::endl;
    std::cout << "1. Putting robot in STAND_XYZ..." << std::endl;

    // Calcular y enviar postura inicial (STAND)
    for(int i=0; i<4; i++) {
        std::string legs[4] = {"FL", "FR", "BL", "BR"};
        std::string leg = legs[i];
        
        // Usamos directamente la posición STAND definida por el usuario
        Vector3d pos = STAND_XYZ[leg];
        
        bool is_right = (leg == "FR" || leg == "BR");
        LegAngles angs = solve_IK(pos[0], pos[1], pos[2], is_right);
        
        shm_data->angles[i][0] = angs.th1;
        shm_data->angles[i][1] = angs.th2;
        shm_data->angles[i][2] = angs.th3;
        
        // Tiempo suave para levantarse (2 segundos)
        double startup_time = 2.0;
        shm_data->times[i][0] = startup_time;
        shm_data->times[i][1] = startup_time;
        shm_data->times[i][2] = startup_time;
    }
    shm_data->is_walking = true;

    std::cout << ">>> ROBOT READY IN STAND POSITION." << std::endl;
    std::cout << ">>> PRESS ENTER TO START WALKING..." << std::endl;
    std::cin.get(); // BLOQUEO HASTA ENTER

    // Planificar el primer paso desde la posición actual
    brain.plan_period();

    // --- FASE 2: BUCLE DE MARCHA ---
    std::cout << ">>> WALKING STARTED (Ctrl+C to Stop & Sit)" << std::endl;
    
    while (!stop_signal_received) {
        auto t_start = std::chrono::steady_clock::now();

        double dt_target = brain.update();
        if(dt_target == 0.0) continue;

        // int peak_swing_tick = brain.pts_shift + (brain.pts_swing / 2);

        // // Si estamos en ese tick exacto...
        // if (brain.current_tick == peak_swing_tick) {
        //     std::cout << "\n!!! FREEZE: Pata en el punto mas alto (Z Max) !!!" << std::endl;
        //     std::cout << ">>> Presiona ENTER para continuar el paso..." << std::endl;
        //     std::cin.get(); 
        //     // std::this_thread::sleep_for(std::chrono::seconds(3));
            
        //     std::cout << ">>> Resuming..." << std::endl;
        // }


        // BREAK

        if (!brain.swing_traj_abs.empty() && brain.current_tick > brain.pts_shift) {
            
            int local_peak_idx = 0;
            double max_z_found = -999.0;
            
            for(size_t k = 0; k < brain.swing_traj_abs.size(); k++) {
                if (brain.swing_traj_abs[k].z() > max_z_found) {
                    max_z_found = brain.swing_traj_abs[k].z();
                    local_peak_idx = k;
                }
            }

            int global_peak_tick = brain.pts_shift + local_peak_idx;

            if (brain.current_tick == global_peak_tick + 1) {
                std::cout << "\n!!! FREEZE EXACTO: Altura Maxima (Z=" << max_z_found << ") !!!" << std::endl;
                std::cout << ">>> Pata " << brain.leg_order[brain.seq_idx] << " en el aire." << std::endl;
                std::cout << ">>> Presiona ENTER para bajar..." << std::endl;
                std::cin.get(); 
            }
        }

        // FIN BREAK

        std::string legs[4] = {"FL", "FR", "BL", "BR"};
        for(int i=0; i<4; i++) {
            Vector3d pos = brain.current_feet_pos[legs[i]];
            bool is_right = (legs[i] == "FR" || legs[i] == "BR");
            LegAngles angs = solve_IK(pos[0], pos[1], pos[2], is_right);
            
            shm_data->angles[i][0] = angs.th1;
            shm_data->angles[i][1] = angs.th2;
            shm_data->angles[i][2] = angs.th3;
            
            double safe_time = dt_target * 1.2;
            shm_data->times[i][0] = safe_time;
            shm_data->times[i][1] = safe_time;
            shm_data->times[i][2] = safe_time;
        }
        shm_data->is_walking = true;

        auto t_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t_end - t_start;
        double sleep_s = dt_target - elapsed.count();
        if(sleep_s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
        
        // Debug
        std::string phase = (brain.current_tick < brain.pts_swing) ? "SWING" : "SHIFT";
        printf("\r[%s] Tick:%2d | FL_X:%.3f | BodyY:%.4f", 
               phase.c_str(), brain.current_tick, 
               brain.current_feet_pos["FL"][0], 
               brain.body_offset_traj[brain.current_tick][1]);
        fflush(stdout);
    }

    // --- FASE 3: APAGADO CONTROLADO (INIT POSE) ---
    std::cout << "\n\n>>> INTERRUPT DETECTED. GOING TO INIT_XYZ (SITTING DOWN)..." << std::endl;
    
    // Capturamos dónde quedaron las patas al interrumpir
    std::map<std::string, Vector3d> last_pos = brain.current_feet_pos;
    
    // Duración de la transición a reposo (segundos)
    double shutdown_duration = 2.0;
    int shutdown_steps = 50;
    double dt_shutdown = shutdown_duration / shutdown_steps;

    for(int k=0; k <= shutdown_steps; k++) {
        double t = (double)k / (double)shutdown_steps; // 0.0 a 1.0
        double t_smooth = ease_in_out(t);
        
        std::string legs[4] = {"FL", "FR", "BL", "BR"};
        for(int i=0; i<4; i++) {
            std::string leg = legs[i];
            
            // Interpolación Lineal: Last_Pos -> INIT_XYZ
            Vector3d start = last_pos[leg];
            Vector3d end = INIT_XYZ[leg];
            Vector3d current = start + (end - start) * t_smooth;
            
            bool is_right = (leg == "FR" || leg == "BR");
            LegAngles angs = solve_IK(current[0], current[1], current[2], is_right);
            
            shm_data->angles[i][0] = angs.th1;
            shm_data->angles[i][1] = angs.th2;
            shm_data->angles[i][2] = angs.th3;
            
            // Tiempo un poco mayor al ciclo para suavidad extrema
            double time_s = dt_shutdown * 1.5;
            shm_data->times[i][0] = time_s;
            shm_data->times[i][1] = time_s;
            shm_data->times[i][2] = time_s;
        }
        shm_data->is_walking = true;
        std::this_thread::sleep_for(std::chrono::duration<double>(dt_shutdown));
    }

    std::cout << ">>> ROBOT IS SITTING. CLOSING..." << std::endl;
    shm_data->is_walking = false;
    munmap(ptr, SHM_SIZE);
    shm_unlink(SHM_NAME);
    close(shm_fd);
    return 0;
}
