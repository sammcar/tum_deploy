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

// ============================================================
// 1. ESTRUCTURA DE MEMORIA COMPARTIDA
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

// Estructura vectorial simple
struct Vector3d {
    double x, y, z;
    Vector3d(double _x=0, double _y=0, double _z=0) : x(_x), y(_y), z(_z) {}
};

struct LegAngles { double th1, th2, th3; bool valid; };

struct TrajectoryPoint {
    double x, z;          // Posición
    double vx, vz;        // Velocidad
    double ax, az;        // Aceleración
};

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

// ============================================================
// 2. CONFIGURACIÓN FÍSICA (STAND_XYZ)
// ============================================================
const double L1 = 0.093;
const double L2 = 0.147;
const double L3 = 0.230;

// Mapeo de IDs a Nombres para usar tu mapa
// 0: FL, 1: FR, 2: BL, 3: BR
Vector3d LEGS_STAND_XYZ[4] = {
    { 0.02,   0.093, -0.316}, // 0: FL
    { 0.02,  -0.093, -0.316}, // 1: FR
    {-0.018,  0.093, -0.316}, // 2: BL
    {-0.018, -0.093, -0.316}  // 3: BR
};

// ============================================================
// 3. MATEMÁTICAS (IK + BÉZIER)
// ============================================================

// Utilidad para envolver ángulos (-PI a PI)
double wrap_to_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// IK proporcionada por ti
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

// Curva de Bézier Cúbica para Swing
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
    // Posición
    res.x = u3*x0 + 3*u2*t*x1 + 3*u*t2*x2 + t3*x3;
    res.z = u3*z0 + 3*u2*t*z1 + 3*u*t2*z2 + t3*z3;

    // Velocidad / Aceleración (Simplificadas para brevedad, idealmente completas)
    // Aceleración aproximada analítica para el control dinámico
    double d2xdt2 = 6*u*(x2 - 2*x1 + x0) + 6*t*(x3 - 2*x2 + x1);
    double d2zdt2 = 6*u*(z2 - 2*z1 + z0) + 6*t*(z3 - 2*z2 + z1);

    if (duration > 1e-5) {
        res.ax = d2xdt2 / (duration * duration);
        res.az = d2zdt2 / (duration * duration);
    } else { res.ax = 0; res.az = 0; }

    return res;
}

// Generador de Trayectoria relativo al centro de la pata (origin_x, origin_z)
TrajectoryPoint get_step_trajectory(double phase, double step_period, 
                                    double origin_x, double origin_z, 
                                    double step_len, double step_height) {
    
    phase = phase - std::floor(phase);
    
    // El paso se centra en la posición STAND de la pata
    double x_start = origin_x - step_len / 2.0;
    double x_end   = origin_x + step_len / 2.0;
    double z_ground = origin_z;

    // --- STANCE (0.5 - 1.0) ---
    if (phase >= 0.5) {
        double t = (phase - 0.5) * 2.0;
        TrajectoryPoint p;
        p.x = lerp(x_end, x_start, t); // Arrastre hacia atrás
        p.z = z_ground;
        p.ax = 0; p.az = 0;
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

// ============================================================
// 4. MAIN CORREGIDO (Aterrizaje Suave + Bloqueo Real)
// ============================================================
int main() {
    std::signal(SIGINT, SignalHandler);

    // --- INPUT DATOS ---
    int max_ciclos = pedir_dato<int>("Ciclos totales (ej. 20)");
    double step_duration = pedir_dato<double>("Periodo paso (s) (ej. 0.4)"); 
    double step_len = pedir_dato<double>("Largo paso (mm) (ej. 80)");
    double step_h = pedir_dato<double>("Altura paso (mm) (ej. 40)");

    // Conversión a metros
    step_len /= 1000.0;
    step_h /= 1000.0;

    const char* shm_name = "/rex_shm";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(SharedData));
    SharedData* shm_ptr = static_cast<SharedData*>(mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    if (shm_ptr == MAP_FAILED) {
        std::cerr << "Error accediendo a memoria compartida" << std::endl;
        return 1;
    }

    std::cout << "\n[INFO] Usando STAND_XYZ como origen." << std::endl;

    // --- CONFIGURACIÓN DE GAIT (Trote) ---
    double gait_offsets[4] = {0.0, 0.5, 0.5, 0.0}; 

    // ============================================================
    // FASE A: ALINEACIÓN SUAVE
    // ============================================================
    std::cout << ">> Alineando suavemente a posición de inicio..." << std::endl;
    
    double align_duration = 3.0; 
    double align_dt = 0.01;      
    int align_steps = align_duration / align_dt;

    LegAngles start_angles[4];
    for(int p=0; p<4; ++p) {
        bool is_right = (p == 1 || p == 3);
        Vector3d origin = LEGS_STAND_XYZ[p];
        double start_phase = fmod(0.0 + gait_offsets[p], 1.0);
        // Calculamos posición inicial exacta
        TrajectoryPoint pt = get_step_trajectory(start_phase, step_duration, origin.x, origin.z, step_len, step_h);
        start_angles[p] = solve_IK(pt.x, origin.y, pt.z, is_right);
    }

    for (int i = 1; i <= align_steps && g_running; ++i) {
        double t_lin = (double)i / align_steps; 
        double t_smooth = (1.0 - std::cos(t_lin * M_PI)) / 2.0;

        for(int p=0; p<4; ++p) {
            if (start_angles[p].valid) {
                shm_ptr->angles[p][0] = start_angles[p].th1 * t_smooth;
                shm_ptr->angles[p][1] = start_angles[p].th2 * t_smooth;
                shm_ptr->angles[p][2] = start_angles[p].th3 * t_smooth;
                // Velocidad estimada para que el feedforward no se vuelva loco
                shm_ptr->velocities[p][0] = (start_angles[p].th1 / align_duration) * 1.5;
                shm_ptr->velocities[p][1] = (start_angles[p].th2 / align_duration) * 1.5;
                shm_ptr->velocities[p][2] = (start_angles[p].th3 / align_duration) * 1.5;
                shm_ptr->desired_accel[p][0] = 0; shm_ptr->desired_accel[p][1] = 0; shm_ptr->desired_accel[p][2] = 0;
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                shm_ptr->is_stance[p] = true; 
            }
        }
        shm_ptr->is_walking = true;
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(align_dt * 1000)));
    }

    // ============================================================
    // FASE B: HOLD DE ESPERA
    // ============================================================
    std::cout << ">> Alineación completa. MANTENIENDO POSICIÓN." << std::endl;
    
    // Fijamos valores finales exactos
    for(int p=0; p<4; ++p) {
        if (start_angles[p].valid) {
            shm_ptr->angles[p][0] = start_angles[p].th1;
            shm_ptr->angles[p][1] = start_angles[p].th2;
            shm_ptr->angles[p][2] = start_angles[p].th3;
        }
        shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
        shm_ptr->desired_accel[p][0] = 0.0; shm_ptr->desired_accel[p][1] = 0.0; shm_ptr->desired_accel[p][2] = 0.0;
        shm_ptr->is_stance[p] = true;
    }

    std::cout << "\n>>> PRESIONA ENTER PARA INICIAR LA MARCHA <<<" << std::endl;
    // Limpieza de buffer robusta
    std::cin.clear(); 
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get(); 

    std::cout << ">> Iniciando marcha..." << std::endl;

    // ============================================================
    // FASE C: BUCLE DINÁMICO (Trote + Aterrizaje Controlado)
    // ============================================================
    auto start_time = std::chrono::steady_clock::now();
    double loop_dt = 0.004;
    double prev_angles[4][3] = {0}; 
    
    for(int p=0; p<4; ++p) {
        prev_angles[p][0] = start_angles[p].th1;
        prev_angles[p][1] = start_angles[p].th2;
        prev_angles[p][2] = start_angles[p].th3;
    }
    bool first_run = true;

    // Variables para el control de aterrizaje final
    bool leg_finished[4] = {false, false, false, false}; // Si ya aterrizó y se bloqueó
    TrajectoryPoint final_points[4]; // Guardamos la coordenada exacta donde aterrizó

    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();
        
        bool tiempo_agotado = (elapsed > (max_ciclos * step_duration));
        bool todo_aterrizado = true; 

        double global_phase = fmod(elapsed / step_duration, 1.0);

        for (int p = 0; p < 4; p++) {
            bool is_right = (p == 1 || p == 3);
            
            // Si la pata ya terminó, usamos su punto congelado y no calculamos nada más
            if (leg_finished[p]) {
                shm_ptr->desired_accel[p][0] = 0.0; shm_ptr->desired_accel[p][1] = 0.0; shm_ptr->desired_accel[p][2] = 0.0;
                shm_ptr->is_stance[p] = true;
                
                // Calculamos IK para el punto congelado (para mantener posición estricta)
                LegAngles ik = solve_IK(final_points[p].x, LEGS_STAND_XYZ[p].y, final_points[p].z, is_right);
                if (ik.valid) {
                    shm_ptr->angles[p][0] = ik.th1;
                    shm_ptr->angles[p][1] = ik.th2;
                    shm_ptr->angles[p][2] = ik.th3;
                    shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
                }
                // Ganancias al máximo para sostener
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                
                continue; // Pasamos a la siguiente pata
            }

            // Si no ha terminado, calculamos normal
            double leg_phase = fmod(global_phase + gait_offsets[p], 1.0);
            bool stance = (leg_phase >= 0.5);
            
            Vector3d origin = LEGS_STAND_XYZ[p];
            TrajectoryPoint point = get_step_trajectory(leg_phase, step_duration, origin.x, origin.z, step_len, step_h);

            // LOGICA DE TRANSICIÓN A FINAL
            if (tiempo_agotado) {
                if (stance) {
                    // ¡ATERRIZAJE DETECTADO!
                    // Guardamos la posición ACTUAL como la posición final.
                    final_points[p] = point; // Guardamos X, Z actuales
                    final_points[p].vx = 0; final_points[p].vz = 0;
                    
                    leg_finished[p] = true; // Marcamos para no volver a calcular
                    
                    // Configuramos inmediata rigidez
                    shm_ptr->is_stance[p] = true;
                    // Saltamos al siguiente ciclo para que el bloque 'if(leg_finished)' tome el control
                    continue; 
                } else {
                    // Sigue volando, permitimos que continúe su trayectoria
                    todo_aterrizado = false;
                }
            }

            // Ejecución Normal (Marcha o Vuelo Final)
            LegAngles ik = solve_IK(point.x, origin.y, point.z, is_right);

            if (ik.valid) {
                shm_ptr->angles[p][0] = ik.th1;
                shm_ptr->angles[p][1] = ik.th2;
                shm_ptr->angles[p][2] = ik.th3;

                if (!first_run) {
                    shm_ptr->velocities[p][0] = (ik.th1 - prev_angles[p][0]) / loop_dt;
                    shm_ptr->velocities[p][1] = (ik.th2 - prev_angles[p][1]) / loop_dt;
                    shm_ptr->velocities[p][2] = (ik.th3 - prev_angles[p][2]) / loop_dt;
                }
                prev_angles[p][0] = ik.th1; prev_angles[p][1] = ik.th2; prev_angles[p][2] = ik.th3;

                shm_ptr->desired_accel[p][0] = point.ax;
                shm_ptr->desired_accel[p][1] = 0.0;
                shm_ptr->desired_accel[p][2] = point.az;

                shm_ptr->is_stance[p] = stance;

                if (stance) {
                    shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                    shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                    shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                } else {
                    shm_ptr->kp_scale[p][0] = 0.6; shm_ptr->kd_scale[p][0] = 0.4;
                    shm_ptr->kp_scale[p][1] = 0.6; shm_ptr->kd_scale[p][1] = 0.4;
                    shm_ptr->kp_scale[p][2] = 0.6; shm_ptr->kd_scale[p][2] = 0.4;
                }
            }
        }
        
        if (tiempo_agotado && todo_aterrizado) {
            break;
        }

        first_run = false;
        shm_ptr->is_walking = true;
        std::this_thread::sleep_until(t_now + std::chrono::duration<double>(loop_dt));
    }

    // ============================================================
    // FASE D: HOLD FINAL (Post-Caminata)
    // ============================================================
    std::cout << "\n>> Ciclo terminado. Todas las patas aterrizadas." << std::endl;
    std::cout << ">> MANTENIENDO POSICIÓN FINAL." << std::endl;
    std::cout << ">>> PRESIONA ENTER PARA FINALIZAR Y APAGAR <<<" << std::endl;
    
    // Importante: No re-limpiamos buffer aquí porque el flujo ha sido automático.
    // Solo esperamos el input del usuario.
    std::cin.get(); 

    std::memset(shm_ptr, 0, sizeof(SharedData));
    shm_ptr->is_walking = false;
    std::cout << "\n✅ Finalizado con éxito." << std::endl;
    return 0;
}
