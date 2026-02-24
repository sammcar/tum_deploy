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
#include <Eigen/Dense>
#include <fstream> // NUEVO: Para guardar el CSV

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;

// ============================================================
// 1. ESTRUCTURA DE MEMORIA COMPARTIDA
// ============================================================
//#pragma pack(push, 1)
struct CommandData {
    double angles[4][3];        // [Pata][Motor]
    double velocities[4][3];    // Se usa como LÍMITE DE VELOCIDAD en modo estático
    double desired_accel[4][3]; // [X, Y, Z] Feedforward
    double kp_scale[4][3];      
    double kd_scale[4][3];
    double transition_time;     // NUEVO: Tiempo para interpolación en main_sebas
    bool is_stance[4];          
    bool is_walking;            // false = modo estático interpolado
};
//#pragma pack(pop)

struct TelemetryData
{
    double measured_angles[4][3];
    double measured_velocities[4][3];
    double measured_torques[4][3];
    double temperature[4][3];
    uint64_t timestamp_us;
    bool fault_code[4]; 
};

struct LogData {
    int leg_id;
    double phase;       // El 'leg_phase' continuo (0.0 a 1.0)
    bool is_stance;     // El booleano de tu algoritmo (opcional, para referencia)
    double torque_femur; // tel_ptr->measured_torques[p][1]
    double torque_tibia; // tel_ptr->measured_torques[p][2]
};

struct LegAngles { double th1, th2, th3; bool valid; };

struct ContactData {
    double fz_r[4]; // Fuerza vertical real en Newtons
    bool is_contact[4];  // true si supera el umbral, false si no
};

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
const double L_BODY = 0.375; // Largo total
const double W_BODY = 0.200; // Ancho total

// Distancia del Centro Geométrico a los hombros (Hips)
const double DX = L_BODY / 2.0;
const double DY = W_BODY / 2.0;

// Posición de cada cadera respecto al centro del robot (0,0,0)
// Orden: 0:FL, 1:FR, 2:BL, 3:BR
const Vector3d HIP_OFFSETS[4] = {
    Vector3d( DX,  DY, 0.0), // FL
    Vector3d( DX, -DY, 0.0), // FR
    Vector3d(-DX,  DY, 0.0), // BL
    Vector3d(-DX, -DY, 0.0)  // BR
};

Vector3d LEGS_STAND_XYZ[4] = {
    Vector3d( 0.01,   0.103, -0.296), // 0: FL
    Vector3d( 0.01,  -0.093, -0.296), // 1: FR
    Vector3d(-0.028,  0.103, -0.296), // 2: BL
    Vector3d(-0.028, -0.093, -0.296)  // 3: BR
};

// Vector3d LEGS_STAND_XYZ[4] = { // Original
//     { 0.02,   0.093, -0.316}, // 0: FL
//     { 0.02,  -0.093, -0.316}, // 1: FR
//     {-0.018,  0.093, -0.316}, // 2: BL
//     {-0.018, -0.093, -0.316}  // 3: BR
// };

// ============================================================
// 3. MATEMÁTICAS (IK + BÉZIER)
// ============================================================


Vector3d ComputeWholeBodyIK(Vector3d foot_pos_local_flat, int leg_idx, 
                            Vector3d body_pos, Vector3d body_rpy) {
    
    Matrix3d R_body;
    R_body = AngleAxisd(body_rpy.z(), Vector3d::UnitZ()) *
             AngleAxisd(body_rpy.y(), Vector3d::UnitY()) *
             AngleAxisd(body_rpy.x(), Vector3d::UnitX());

    Vector3d hip_offset_static = HIP_OFFSETS[leg_idx];
    Vector3d foot_pos_global = hip_offset_static + foot_pos_local_flat;
    Vector3d hip_pos_new = body_pos + (R_body * hip_offset_static);
    Vector3d vec_world = foot_pos_global - hip_pos_new;
    Vector3d vec_local_ik = R_body.transpose() * vec_world;

    return vec_local_ik;
}

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
// 4. MAIN CORREGIDO (Aterrizaje Suave + Bloqueo Real)
// ============================================================
int main() {
    std::signal(SIGINT, SignalHandler);

    // --- 1. INPUT DATOS BÁSICOS (Marcha) ---
    int max_ciclos = pedir_dato<int>("Ciclos totales (ej. 20)");
    double step_duration = pedir_dato<double>("Periodo paso (s) (ej. 0.4)"); 
    double step_len = pedir_dato<double>("Largo paso (mm) (ej. 80)");
    double step_h = pedir_dato<double>("Altura paso (mm) (ej. 40)");

    // Conversión a metros
    step_len /= 1000.0;
    step_h /= 1000.0;

    const char* shm_name = "/rex_cmd";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(CommandData));
    CommandData* shm_ptr = static_cast<CommandData*>(mmap(0, sizeof(CommandData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    if (shm_ptr == MAP_FAILED) {
        std::cerr << "Error accediendo a memoria compartida" << std::endl;
        return 1;
    }

    const char* tel_name = "/rex_tel";
    int tel_fd = shm_open(tel_name, O_CREAT | O_RDWR, 0666);
    ftruncate(tel_fd, sizeof(TelemetryData));
    TelemetryData* tel_ptr = static_cast<TelemetryData*>(mmap(0, sizeof(TelemetryData), PROT_READ | PROT_WRITE, MAP_SHARED, tel_fd, 0));

    const char* contact_name = "/rex_contact";
    int contact_fd = shm_open(contact_name, O_CREAT | O_RDWR, 0666);
    ftruncate(contact_fd, sizeof(ContactData));
    ContactData* contact_ptr = static_cast<ContactData*>(mmap(0, sizeof(ContactData), PROT_READ | PROT_WRITE, MAP_SHARED, contact_fd, 0));

    if (contact_ptr == MAP_FAILED) {
        std::cerr << "Error accediendo a memoria compartida de contactos" << std::endl;
        return 1;
    }

    if (tel_ptr == MAP_FAILED) {
        std::cerr << "Error accediendo a memoria compartida de telemetria" << std::endl;
        return 1;
    }

    std::cout << "\n[INFO] Usando STAND_XYZ como origen." << std::endl;

    // --- CONFIGURACIÓN DE GAIT (Trote) ---
    double gait_offsets[4] = {0.0, 0.5, 0.5, 0.0};
    Vector3d start_foot_positions[4]; 

    // ============================================================
    // FASE A: ALINEACIÓN SUAVE (A postura Neutra)
    // ============================================================
    std::cout << ">> Alineando suavemente a posición de inicio (Neutra)..." << std::endl;
    
    double align_duration = 3.0;     

    LegAngles start_angles[4];
    // Calculamos ángulos neutros (Sin offsets de cuerpo aún)
    std::cout << ">> Enviando comando de alineación inicial (3.0s)..." << std::endl;
    
    // 1. Calcular ángulos neutros
    for(int p=0; p<4; ++p) {
        bool is_right = (p == 1 || p == 3);
        Vector3d origin = LEGS_STAND_XYZ[p];
        double start_phase = fmod(0.0 + gait_offsets[p], 1.0);
        TrajectoryPoint pt = get_step_trajectory(start_phase, step_duration, origin.x(), origin.z(), step_len, step_h);
        start_foot_positions[p] = Vector3d(pt.x, origin.y(), pt.z);
        start_angles[p] = solve_IK(pt.x, origin.y(), pt.z, is_right);
    }

    // 2. Escribir en SHM (UNA SOLA VEZ)
    if (shm_ptr) {
        for(int p=0; p<4; ++p) {
            if (start_angles[p].valid) {
                shm_ptr->angles[p][0] = start_angles[p].th1;
                shm_ptr->angles[p][1] = start_angles[p].th2;
                shm_ptr->angles[p][2] = start_angles[p].th3;
                
                // Configuración para modo estático en main_sebas:
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                
                // Velocidad límite (importante para que no se mueva lento artificialmente)
                shm_ptr->velocities[p][0] = 5.0; 
                shm_ptr->velocities[p][1] = 5.0; 
                shm_ptr->velocities[p][2] = 5.0;
                
                shm_ptr->is_stance[p] = true;
            }
        }
        shm_ptr->transition_time = align_duration; // main_sebas usará esto para interpolar
        shm_ptr->is_walking = false;    // Activa el modo interpolación estática
    }

    // 3. Esperar a que el robot termine el movimiento físico
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << ">> Alineación completada." << std::endl;

    // ============================================================
    // FASE B: CONFIGURACIÓN Y APLICACIÓN DE POSTURA (ESTÁTICA)
    // ============================================================
    std::cout << "\n>> Alineación base completada. Manteniendo posición neutra." << std::endl;

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

    std::cout << "========================================" << std::endl;
    std::cout << "   CONFIGURACIÓN DE POSTURA CORPORAL    " << std::endl;
    std::cout << "========================================" << std::endl;

    double in_bx = pedir_dato<double>("Offset Cuerpo X (mm) [Adelante+]");
    double in_by = pedir_dato<double>("Offset Cuerpo Y (mm) [Izq+]");
    double in_bz = pedir_dato<double>("Offset Cuerpo Z (mm) [Abajo-]"); 
    
    double in_roll  = pedir_dato<double>("Roll (grados)");
    double in_pitch = pedir_dato<double>("Pitch (grados) [Nariz Abajo+]");
    double in_yaw   = pedir_dato<double>("Yaw (grados)");

    // Objetivos finales de postura
    Vector3d target_body_pos(in_bx / 1000.0, in_by / 1000.0, in_bz / 1000.0);
    Vector3d target_body_rpy(in_roll * M_PI / 180.0, in_pitch * M_PI / 180.0, in_yaw * M_PI / 180.0);

    std::cout << "\n>>> PRESIONA ENTER PARA APLICAR POSTURA (SIN CAMINAR) <<<" << std::endl;
    std::cin.get(); 

    std::cout << ">> Aplicando postura corporal..." << std::endl;

    // --- TRANSICIÓN SUAVE A LA POSTURA MODIFICADA (Implementada) ---
    // Se utiliza la misma lógica de suavizado (coseno) que en la Fase A
    double posture_duration = 2.0; // 2 segundos para acomodarse
    double posture_dt = 0.01;      // Paso de tiempo
    int posture_steps = posture_duration / posture_dt;

    std::cout << ">> Enviando comando de postura (2.0s)..." << std::endl;

    for(int p=0; p<4; ++p) {
        bool is_right = (p == 1 || p == 3);
        Vector3d foot_flat = start_foot_positions[p];
        // Calcular IK con el cuerpo rotado
        Vector3d ik_input = ComputeWholeBodyIK(foot_flat, p, target_body_pos, target_body_rpy);
        LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), is_right);
        
        if(ik.valid) {
             shm_ptr->angles[p][0] = ik.th1;
             shm_ptr->angles[p][1] = ik.th2;
             shm_ptr->angles[p][2] = ik.th3;
             
             // Asegurar ganancias fuertes y velocidad límite
             shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
             shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
             shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
             shm_ptr->velocities[p][0] = 5.0; 
             shm_ptr->velocities[p][1] = 5.0; 
             shm_ptr->velocities[p][2] = 5.0;
        }
    }
    
    shm_ptr->transition_time = 2.0; // Tiempo para acomodar el cuerpo
    shm_ptr->is_walking = false;    // Modo estático

    // Esperar transición
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << ">> Postura aplicada." << std::endl;

    std::cout << "\n>>> PRESIONA ENTER PARA INICIAR MARCHA <<<" << std::endl;
    std::cin.get(); // Espera al usuario
    std::cout << ">> Iniciando marcha..." << std::endl;

    std::cout << "\nTorque Femur FL: " << tel_ptr->measured_torques[0][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia FL: " << tel_ptr->measured_torques[0][2] << " Nm      " << std::endl;
    std::cout << "\nTorque Femur FR: " << tel_ptr->measured_torques[1][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia FR: " << tel_ptr->measured_torques[1][2] << " Nm      " << std::endl;
    std::cout << "\nTorque Femur BL: " << tel_ptr->measured_torques[2][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia BL: " << tel_ptr->measured_torques[2][2] << " Nm      " << std::endl;
    std::cout << "\nTorque Femur BR: " << tel_ptr->measured_torques[3][1] << " Nm      " << std::endl;
    std::cout << "\nTorque Tibia BR: " << tel_ptr->measured_torques[3][2] << " Nm      " << std::endl;

    // ============================================================
    // FASE C: BUCLE DINÁMICO (Trote + Postura Mantenida)
    // ============================================================
    auto start_time = std::chrono::steady_clock::now();
    double loop_dt = 0.004;
    double prev_angles[4][3] = {0};
    double fz_u[4] = {15.0, 15.0, 15.0, 15.0};

    std::vector<LogData> historial;
    historial.reserve(20000);
    
    // Inicializamos prev_angles con lo que tenemos en SHM (la postura ya aplicada)
    for(int p=0; p<4; ++p) {
        prev_angles[p][0] = shm_ptr->angles[p][0];
        prev_angles[p][1] = shm_ptr->angles[p][1];
        prev_angles[p][2] = shm_ptr->angles[p][2];
    }
    bool first_run = true;
    bool leg_finished[4] = {false, false, false, false};
    Vector3d final_foot_pos_3d[4];

    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - start_time).count();

        int c[4] = {0, 0, 0, 0};            // Aquí guardaremos tu c_i (1 si toca, 0 si no)
        double fz_medido[4] = {0.0, 0.0, 0.0, 0.0}; // Guardamos la fuerza Z de cada pata
        
        bool tiempo_agotado = (elapsed > (max_ciclos * step_duration));
        bool todo_aterrizado = true; 
        double global_phase = fmod(elapsed / step_duration, 1.0);

        for (int p = 0; p < 4; p++) {
            bool is_right = (p == 1 || p == 3);
            
            // --- Si la pata ya terminó (HOLD FINAL) ---
            if (leg_finished[p]) {
                shm_ptr->desired_accel[p][0] = 0; shm_ptr->desired_accel[p][1] = 0; shm_ptr->desired_accel[p][2] = 0;
                shm_ptr->is_stance[p] = true;
                
                // MANTENEMOS LA POSTURA MODIFICADA EN EL HOLD
                Vector3d final_ik_input = ComputeWholeBodyIK(final_foot_pos_3d[p], p, target_body_pos, target_body_rpy);
                
                LegAngles ik = solve_IK(final_ik_input.x(), final_ik_input.y(), final_ik_input.z(), is_right);
                
                if (ik.valid) {
                    shm_ptr->angles[p][0] = ik.th1;
                    shm_ptr->angles[p][1] = ik.th2;
                    shm_ptr->angles[p][2] = ik.th3;
                    shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
                }
                // Alta rigidez para mantener postura
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                
                continue; 
            }

            // --- Trayectoria Normal ---
            double leg_phase = fmod(global_phase + gait_offsets[p], 1.0);
            bool stance = (leg_phase >= 0.5);

            // --- NUEVO: GUARDAR TELEMETRÍA (Sólo Pata 0) ---
            if (!tiempo_agotado) {
                historial.push_back({
                    p,                               // leg_id
                    leg_phase,                       // phase
                    stance,                          // is_stance
                    tel_ptr->measured_torques[p][1], // Motor 1 (Femur)
                    tel_ptr->measured_torques[p][2]  // Motor 2 (Tibia)
                });
            }

            Vector3d origin = LEGS_STAND_XYZ[p];
            TrajectoryPoint point = get_step_trajectory(leg_phase, step_duration, origin.x(), origin.z(), step_len, step_h);

            // --- Chequeo de fin de ciclo ---
            if (tiempo_agotado) {
                if (stance) {
                    final_foot_pos_3d[p] = Vector3d(point.x, origin.y(), point.z);
                    leg_finished[p] = true; 
                    shm_ptr->is_stance[p] = true;
                    continue; 
                } else {
                    todo_aterrizado = false;
                }
            }

            // --- APLICACIÓN DINÁMICA DE POSTURA ---
            Vector3d foot_local_flat(point.x, origin.y(), point.z);
            
            // Aquí usamos target_body_pos/rpy que ya definimos arriba
            Vector3d ik_input = ComputeWholeBodyIK(foot_local_flat, p, target_body_pos, target_body_rpy);
            LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), is_right);

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

                // Ajuste de ganancias según fase
                double kp_val = stance ? 1.0 : 0.6;
                double kd_val = stance ? 1.0 : 0.4;
                for(int m=0; m<3; ++m) {
                    shm_ptr->kp_scale[p][m] = kp_val;
                    shm_ptr->kd_scale[p][m] = kd_val;
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
    // FASE D: HOLD FINAL (Post-Caminata con Postura)
    // ============================================================
    std::cout << "\n>> Ciclo terminado. Todas las patas aterrizadas." << std::endl;
    std::cout << ">> MANTENIENDO POSTURA FINAL." << std::endl;
    std::cout << ">>> PRESIONA ENTER PARA FINALIZAR Y APAGAR <<<" << std::endl;
    
    // Aquí el robot se queda congelado enviando el último comando del bucle while
    // que ya incluye la postura modificada.
    std::cin.get(); 

    // Enviar todo a 0.0
    for(int p=0; p<4; ++p) {
        for(int m=0; m<3; ++m) {
            shm_ptr->angles[p][m] = 0.0;
            shm_ptr->velocities[p][m] = 2.0; // Velocidad límite moderada
            shm_ptr->kp_scale[p][m] = 1.0;
            shm_ptr->kd_scale[p][m] = 1.0;
            shm_ptr->desired_accel[p][m] = 0.0;
        }
    }
    shm_ptr->transition_time = 3.0; // 3 segundos para sentarse/colgarse
    shm_ptr->is_walking = false;

    // Esperar a que baje
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Limpieza final
    std::memset(shm_ptr, 0, sizeof(CommandData));
    shm_ptr->is_walking = false; // Asegurar que quede en false al salir
    
    std::cout << "\n✅ Finalizado. Robot en posición segura." << std::endl;

    // --- NUEVO: GUARDAR EL CSV ---
    std::cout << ">> Guardando datos de torques en 'torques_log.csv'..." << std::endl;
    std::ofstream log_file("torques_log.csv");
    if (log_file.is_open()) {
        log_file << "Leg_ID,Phase,Is_Stance,Torque_Femur,Torque_Tibia\n";
        for (const auto& dato : historial) {
            log_file << dato.leg_id << ","
                     << dato.phase << "," 
                     << dato.is_stance << "," 
                     << dato.torque_femur << "," 
                     << dato.torque_tibia << "\n";
        }
        log_file.close();
        std::cout << "✅ Archivo 'torques_log.csv' guardado correctamente." << std::endl;
    } else {
        std::cerr << "❌ Error: No se pudo crear el archivo CSV." << std::endl;
    }
    
    return 0;
}
