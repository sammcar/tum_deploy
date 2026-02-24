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
#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;

// ==========================================
// 1. ESTRUCTURA DE MEMORIA Y CONSTANTES
// ==========================================
struct SharedData {
    double angles[4][3];
    double velocities[4][3];
    double desired_accel[4][3];
    double kp_scale[4][3];
    double kd_scale[4][3];
    double transition_time;
    bool is_stance[4];
    bool is_walking;
};

// Dimensiones físicas (mm)
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
// 2. FUNCIONES AUXILIARES (NUEVO)
// ==========================================

/**
 * @brief Genera una configuración de pies con offsets independientes en X y Z para adelante/atrás.
 * Esto evita tener que escribir Vector3d(...) repetidamente.
 * @param x_front Desplazamiento en X para las patas delanteras
 * @param x_rear Desplazamiento en X para las patas traseras
 * @param y_offset Desplazamiento en Y (simetría)
 * @param z_add_front Altura extra para patas delanteras (0 = altura normal)
 * @param z_add_rear Altura extra para patas traseras
 * @param feet_array Arreglo donde se guardará el resultado
 */
void getModifiedFeet(double x_front, double x_rear, double y_offset, double z_add_front, double z_add_rear, Vector3d feet_array[4]) {
    // Copiar base
    for(int i=0; i<4; i++) feet_array[i] = NEUTRAL_FEET[i];

    // Aplicar offsets generales en Y
    for(int i=0; i<4; i++) {
        feet_array[i].y() += (i%2 == 0) ? y_offset : -y_offset; // Simetría en Y
    }

    // Aplicar altura y offset X específico (Front: indices 0 y 1, Rear: indices 2 y 3)
    feet_array[0].x() += x_front;
    feet_array[1].x() += x_front;
    feet_array[0].z() += z_add_front;
    feet_array[1].z() += z_add_front;
    
    feet_array[2].x() += x_rear;
    feet_array[3].x() += x_rear;
    feet_array[2].z() += z_add_rear;
    feet_array[3].z() += z_add_rear;
}

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

// ==========================================
// 4. ENVÍO DE COMANDOS
// ==========================================
void SendPose(SharedData* shm, Vector3d body_p, Vector3d body_r, const Vector3d feet[4], double seconds) {
    shm->transition_time = seconds;
    shm->is_walking = false;

    Vector3d body_r_rad = body_r * M_PI / 180.0;

    for (int i = 0; i < 4; i++) {
        Vector3d local = ComputeWBC(feet[i], i, body_p, body_r_rad);
        solve_IK(local, (i == 1 || i == 3), shm->angles[i]);
        shm->is_stance[i] = true;
    }
    
    // Bloqueo preciso basado en tiempo
    auto start = std::chrono::steady_clock::now();
    while(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ==========================================
// 5. RUTINAS
// ==========================================

void flexion(SharedData* shm) {
    std::cout << "\n[START] Iniciando rutina de Flexiones..." << std::endl;
    // Usamos NEUTRAL_FEET directamente
    for (int i = 0; i < 4; ++i) {
        SendPose(shm, Vector3d(0, 0, -70.0), Vector3d(0, 0, 0), NEUTRAL_FEET, 1.0); // Bajar
        SendPose(shm, Vector3d(0, 0, 0.0),   Vector3d(0, 0, 0), NEUTRAL_FEET, 1.0); // Subir
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void sentarse(SharedData* shm) {
    std::cout << "\n[START] Rutina Sentarse..." << std::endl;
    Vector3d feet[4];

    // PASO A: Posición Neutra con patas traseras subiendo (z_inc = 90)
    // x_front=0, x_rear=0, y_offset=0, front_z=0, rear_z=90
    getModifiedFeet(0, 0, 0, 0, 60, feet);
    SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), feet, 1.5);

    // PASO B: Mover patas hacia atrás (x_inc = 50)
    // x_front=-50, x_rear=-50, y_offset=0, front_z=0, rear_z=90
    getModifiedFeet(-50, -50, 0, 0, 60, feet);
    SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), feet, 2.0);

    // PASO C: Sentada profunda
    // x_inc total = 130 (50+80), z_inc total = 150. Traseras +30 extra.
    getModifiedFeet(-80, -80, 0, 60, 80, feet);
    SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, 0, 0), feet, 2.0);

    // PASO C: Sentada profunda
    // x_inc total = 130 (50+80), z_inc total = 150. Traseras +30 extra.
    getModifiedFeet(-110, -110, 0, 160, 190, feet);
    SendPose(shm, Vector3d(0, 0, 0.0), Vector3d(0, -3, 0), feet, 2.0);

    // PASO D: Inclinación del cuerpo
    // Misma posición de pies, pero movemos el cuerpo
    SendPose(shm, Vector3d(20, 0, -40), Vector3d(0, -3, 0), feet, 2.5);

    // PASO E: Ajuste final (Abrir patas en Y)
    // y_inc = 20, z_inc = 170 (150+20)
    getModifiedFeet(-110, -110, 20, 170, 170 + 30, feet);
    SendPose(shm, Vector3d(20, 0, -40), Vector3d(0, 0, 0), feet, 1.0);
}

void levantarse(SharedData* shm) {
    std::cout << "\n[START] Rutina Levantarse..." << std::endl;
    Vector3d feet[4];
    
    // NOTA: Asumo que x_inc es 130.0 basado en el final de la rutina 'sentarse'
    double x_inc = 110.0; 

    // PASO E (Inverso): Posición sentada con patas abiertas PASO 1
    // getModifiedFeet(x_front, x_rear, Eje_Y, Altura_Delantera, Altura_Trasera, Variable_Destino);
    getModifiedFeet(-x_inc, -x_inc, 0, 180, 140, feet); 
    SendPose(shm, Vector3d(0, 0, -20), Vector3d(0, 0, 0), feet, 1.5);

    getModifiedFeet(0, 0, 0, 0, 0, feet); 
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), feet, 1.5);

}

void baile(SharedData* shm) {
    std::cout << "\n[START] Iniciando rutina de Baile..." << std::endl;
    Vector3d feet[4];
    
    // Balanceo lateral
    for (int i = 0; i < 4; ++i) {
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(-10, -10, 5), NEUTRAL_FEET, 2.0);
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(10, 10, 5),  NEUTRAL_FEET, 2.0);
    }
    
    // Centro
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);

    // Paso Adelante/Atrás rápido (x_inc = 30 para todos los pies)
    getModifiedFeet(-30, -30, 0, 0, 0, feet);
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), feet, 2.0);
    
    // Retorno
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);

    // Movimiento de caderas (offset Y simulado moviendo cuerpo)
    for (int i = 0; i < 4; ++i) {
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, -10, 0), NEUTRAL_FEET, 2.0);
        SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 10, 0),  NEUTRAL_FEET, 2.0);
    }
    
    // Final
    SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 2.0);
}

void sendZeroAngles(SharedData* shm, double seconds) {
    std::cout << "[WARN] Enviando 0.0 a todos los motores en " << seconds << "s..." << std::endl;
    shm->transition_time = seconds;
    shm->is_walking = false;
    
    // Resetear IK a postura de 'stance' pero con ángulos cero
    for (int p = 0; p < 4; ++p) {
        shm->is_stance[p] = true;
        for (int m = 0; m < 3; ++m) shm->angles[p][m] = 0.0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

// ==========================================
// 6. MAIN
// ==========================================
int main() {
    int fd = shm_open("/rex_cmd", O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "Error: SHM no detectada. Asegúrate de correr el driver primero." << std::endl;
        return 1;
    }
    
    // Uso reinterpret_cast para mayor corrección en C++
    SharedData* shm = reinterpret_cast<SharedData*>(mmap(0, sizeof(SharedData), PROT_WRITE, MAP_SHARED, fd, 0));

    int opcion = 0;
    while (true) {
        std::cout << "\n========== MENÚ DE RUTINAS ATOM-51 ==========" << std::endl;
        std::cout << "1. Rutina de Flexiones" << std::endl;
        std::cout << "2. Rutina de Baile" << std::endl;
        std::cout << "3. Rutina de Sentarse" << std::endl;
        std::cout << "4. Posición NEUTRAL (IK)" << std::endl;
        std::cout << "5. ÁNGULOS CERO (Reset Total)" << std::endl;
        std::cout << "6. Levantarse" << std::endl;
        std::cout << "0. Salir" << std::endl;
        std::cout << "Selecciona una opción: ";
        
        std::cin >> opcion;
        if (std::cin.fail()) {
            std::cin.clear(); std::cin.ignore(1000, '\n');
            continue;
        }

        switch(opcion) {
            case 1: flexion(shm); break; // Ya no requiere pasar NEUTRAL_FEET
            case 2: baile(shm); break;
            case 3: sentarse(shm); break;
            case 4: SendPose(shm, Vector3d(0, 0, 0), Vector3d(0, 0, 0), NEUTRAL_FEET, 1.0); break;
            case 5: sendZeroAngles(shm, 2.0); break;
            case 6: levantarse(shm); break;
            case 0: return 0;
            default: std::cout << "Opción inválida." << std::endl;
        }
    }
    return 0;
}
