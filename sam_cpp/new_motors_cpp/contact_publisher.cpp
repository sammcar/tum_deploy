#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm> // Añadido para std::clamp
#include <Eigen/Dense>

// ============================================================
// 1. ESTRUCTURAS DE DATOS (Mapeo de Memoria Compartida)
// ============================================================

// Memoria de LECTURA: Telemetría de los motores (Desde rex_tel)
struct TelemetryData {
    double measured_angles[4][3];
    double measured_velocities[4][3];
    double measured_torques[4][3];
    double temperature[4][3];
    uint64_t timestamp_us;
    bool fault_code[4]; 
};

// Memoria de ESCRITURA: Contacto y Fuerzas (Hacia rex_contact)
struct ContactData {
    double fz_r[4];       // Fuerza Z medida absoluta
    bool is_contact[4];   // Estado booleano de contacto
    double v_xyz[4][3];   // Velocidades lineales de la pata [X, Y, Z] en m/s
    double umbral_r[4];   // Umbral dinámico calculado (para depuración)
};

// Estructura interna para retornar fuerzas y velocidades
struct LegForces { 
    double fx, fy, fz;
    double vx, vy, vz; 
};

// ============================================================
// 2. CONFIGURACIÓN FÍSICA Y UMBRALES
// ============================================================
const double L1 = 0.093;
const double L2 = 0.147;
const double L3 = 0.230;

// Umbrales de contacto en Newtons para cada pata [FL, FR, BL, BR]
const double fz_u[4] = {5.0, 7.0, 4.5, 10.0};       // BASE DEL UMBRAL (Corregido a 4.0 para que escale)
const double gain_vz[4] = {1.5, 1.5, 1.5, 1.5};    // GANANCIA DE INERCIA
const double fz_u_max[4] = {35.0, 35.0, 35.0, 35.0}; // LÍMITE MÁXIMO DEL UMBRAL

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

// ============================================================
// 3. FUNCIÓN MATEMÁTICA: JACOBIANO (FUERZAS Y VELOCIDADES)
// ============================================================
LegForces ComputeForcesFromTorques(double q1, double q2, double q3, 
                                   double dq1, double dq2, double dq3,
                                   double t1_abad, double t2_hip, double t3_knee, 
                                   double L1, double L2, double L3, 
                                   bool es_pata_derecha) {
    
    // Precomputar trigonometría
    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s2 = std::sin(q2), c2 = std::cos(q2);
    double s3 = std::sin(q3), c3 = std::cos(q3);
    double s23 = std::sin(q2 + q3);
    double c23 = std::cos(q2 + q3);
    
    double side = es_pata_derecha ? -1.0 : 1.0; 
    
    // Elementos de la matriz Jacobiana
    double j11 = 0;
    double j12 = L2*c2 + L3*c23;
    double j13 = L3*c23;
    double j21 = -L1*side*s1 + (L2*c2 + L3*c23)*c1; 
    double j22 = -(L2*s2 + L3*s23)*s1;
    double j23 = -L3*s23*s1; 
    double j31 = L1*side*c1 + (L2*c2 + L3*c23)*s1;
    double j32 = (L2*s2 + L3*s23)*c1;
    double j33 = L3*s23*c1;

    // Construir la matriz J_Transpuesta
    Eigen::Matrix3d JT;
    JT << j11, j21, j31,
          j12, j22, j32,
          j13, j23, j33;
          
    Eigen::Matrix3d J = JT.transpose(); 
    
    // RESOLUCIÓN DE FUERZAS: Damped Least Squares
    Eigen::Vector3d tau(t1_abad, t2_hip, t3_knee);
    double lambda = 0.05; 
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d F = (J * JT + (lambda * lambda) * I).inverse() * J * tau;
    
    // RESOLUCIÓN DE VELOCIDADES: V = J * dq
    Eigen::Vector3d dq(dq1, dq2, dq3);
    Eigen::Vector3d V = J * dq;
    
    return {F.x(), F.y(), F.z(), V.x(), V.y(), V.z()};
}

// ============================================================
// 4. BUCLE PRINCIPAL (PUBLICADOR)
// ============================================================
int main() {
    std::signal(SIGINT, SignalHandler);

    // --- CONFIGURACIÓN DE FRECUENCIA ---
    const int TARGET_HZ = 250; // 250 Hz = 4ms por ciclo
    const auto CYCLE_TIME = std::chrono::microseconds(1000000 / TARGET_HZ);

    // --- ABRIR MEMORIA COMPARTIDA: TELEMETRÍA (Solo Lectura) ---
    int tel_fd = shm_open("/rex_tel", O_RDONLY, 0666);
    if (tel_fd == -1) {
        std::cerr << "[ERROR] No se pudo abrir /rex_tel. ¿Está corriendo el programa principal?" << std::endl;
        return 1;
    }
    TelemetryData* tel_ptr = static_cast<TelemetryData*>(mmap(0, sizeof(TelemetryData), PROT_READ, MAP_SHARED, tel_fd, 0));
    if (tel_ptr == MAP_FAILED) {
        std::cerr << "[ERROR] Falló el mapeo de la telemetría." << std::endl;
        return 1;
    }

    shm_unlink("/rex_contact"); // Borra el archivo si ya existía   
    // --- ABRIR MEMORIA COMPARTIDA: CONTACTOS (Lectura/Escritura) ---
    int contact_fd = shm_open("/rex_contact", O_CREAT | O_RDWR, 0666);
    ftruncate(contact_fd, sizeof(ContactData));
    ContactData* contact_ptr = static_cast<ContactData*>(mmap(0, sizeof(ContactData), PROT_READ | PROT_WRITE, MAP_SHARED, contact_fd, 0));
    if (contact_ptr == MAP_FAILED) {
        std::cerr << "[ERROR] Falló el mapeo de contactos." << std::endl;
        return 1;
    }
    
    // Limpiar basura residual de memoria
    std::memset(contact_ptr, 0, sizeof(ContactData));

    std::cout << "[OK] Publicador de Contacto iniciado a " << TARGET_HZ << " Hz." << std::endl;

    auto next_cycle = std::chrono::steady_clock::now();

    double fz_anterior[4] = {0.0, 0.0, 0.0, 0.0};
    bool estado_contacto[4] = {false, false, false, false};
    double escudo_memoria[4] = {0.0, 0.0, 0.0, 0.0};

    int contador_pisando[4] = {0, 0, 0, 0};
    const int CICLOS_CONFIRMACION = 8; // 5 ciclos * 4ms = 20ms de validación

    while (g_running) {
        next_cycle += CYCLE_TIME;

        // Declaración de arreglos para el ciclo
        int c[4] = {0, 0, 0, 0}; 
        double fz_medido[4] = {0.0, 0.0, 0.0, 0.0};
        double vz_medido[4][3] = {0.0};
        double vy_medido[4][3] = {0.0};
        double vx_medido[4][3] = {0.0}; 
        double u_d[4] = {0.0}; // NUEVO: Arreglo para poder pasarlo al segundo bucle

        // -------------------------------------------------------------
        // PRIMER PASO: CÁLCULO DE FUERZAS
        // -------------------------------------------------------------
        for (int p = 0; p < 4; p++) {
            bool is_right = (p == 1 || p == 3);

            // Leer ángulos actuales y convertirlos a Radianes
            double q1 = tel_ptr->measured_angles[p][0] * M_PI / 180.0;
            double q2 = tel_ptr->measured_angles[p][1] * M_PI / 180.0;
            double q3 = tel_ptr->measured_angles[p][2] * M_PI / 180.0;

            // Leer velocidades actuales y convertirlas a Radianes/s (CRÍTICO PARA EL JACOBIANO)
            double dq1 = tel_ptr->measured_velocities[p][0] * M_PI / 180.0;
            double dq2 = tel_ptr->measured_velocities[p][1] * M_PI / 180.0;
            double dq3 = tel_ptr->measured_velocities[p][2] * M_PI / 180.0;

            // Leer torques actuales
            double t1 = tel_ptr->measured_torques[p][0];
            double t2 = tel_ptr->measured_torques[p][1];
            double t3 = tel_ptr->measured_torques[p][2];

            // Calcular las fuerzas (X, Y, Z) y Velocidades Lineales
            LegForces state = ComputeForcesFromTorques(q1, q2, q3, dq1, dq2, dq3, t1, t2, t3, L1, L2, L3, is_right);
            
            fz_medido[p] = std::abs(state.fz);
            vz_medido[p][0] = state.vz; 
            vx_medido[p][1] = state.vx;
            vy_medido[p][2] = state.vy;

            double delta_fz = std::abs(fz_medido[p] - fz_anterior[p]);
            fz_anterior[p] = fz_medido[p]; // Guardar para el próximo ciclo

            // --- 1. ESCUDO OMNIDIRECCIONAL (CON PERSISTENCIA) ---
            double sum_dq = std::abs(dq1) + std::abs(dq2) + std::abs(dq3);
            double u_objetivo = fz_u[p] + (gain_vz[p] * sum_dq);
            u_objetivo = std::clamp(u_objetivo, fz_u[p], fz_u_max[p]);

            // Filtro de Decaimiento (Decay Filter)
            if (u_objetivo > escudo_memoria[p]) {
                escudo_memoria[p] = u_objetivo; // Sube instantáneamente para proteger
            } else {
                // Cae lentamente (pierde 0.5 N cada 4ms = 125 N/s)
                // Esto mantiene el escudo alto durante el Ápex
                escudo_memoria[p] -= 0.5; 
                if (escudo_memoria[p] < fz_u[p]) {
                    escudo_memoria[p] = fz_u[p];
                }
            }
            u_d[p] = escudo_memoria[p];

            // ============================================================
            // 5. MÁQUINA DE ESTADOS (LA REGLA MÁGICA)
            // ============================================================
            const double UMBRAL_LATIGAZO = 4.0; // Salto violento de 4N en solo 4 milisegundos
            const double UMBRAL_DESPEGUE[4] = {4.5, 3.5, 3.5, 9.0};
            const double BANDA_QUIETUD_VZ = 0.20; // 0.2 m/s máximo para considerar que chocó

            // if (!estado_contacto[p]) {
            //     if (fz_medido[p] > u_d[p] || (delta_fz > UMBRAL_LATIGAZO )){//&& fz_medido[p] > UMBRAL_DESPEGUE[p])) {
            //         estado_contacto[p] = true; 
            //     }
            // } else {

            //     if (fz_medido[p] < UMBRAL_DESPEGUE[p]) {
            //         estado_contacto[p] = false;
            //     }
            // }
            
            bool supero_escudo = (fz_medido[p] > u_d[p]) || (delta_fz > UMBRAL_LATIGAZO);
            bool vz_casi_cero = (std::abs(vz_medido[p][0]) < BANDA_QUIETUD_VZ);

            // Condición cruda de impacto (Aún puede tener ruido de 1 ciclo)
            bool condicion_impacto = supero_escudo && vz_casi_cero;

            if (!estado_contacto[p]) {
                // ESTADO: AIRE
                if (condicion_impacto) {
                    contador_pisando[p]++; // Acumulamos evidencia
                    if (contador_pisando[p] >= CICLOS_CONFIRMACION) {
                        estado_contacto[p] = true; // Confirmado, es el piso
                    }
                } else {
                    contador_pisando[p] = 0; // Falsa alarma, el ruido desapareció
                }
            } else {
                // ESTADO: PISANDO
                // Para salir, la pata debe dejar de cargar peso
                if (fz_medido[p] < UMBRAL_DESPEGUE[p]) {
                    estado_contacto[p] = false;
                    contador_pisando[p] = 0; // Reiniciar contador para la próxima caída
                }
            }

            c[p] = estado_contacto[p] ? 1 : 0;

            // // Aplicar la función indicadora
            // if (fz_medido[p] > u_d[p]) {
            //     c[p] = 1; // Contacto firme
            // } else {
            //     c[p] = 0; // En el aire
            // }
        }

        // -------------------------------------------------------------
        // SEGUNDO PASO: PUBLICAR ESTADO EN MEMORIA
        // -------------------------------------------------------------
        for (int p = 0; p < 4; p++) {
            contact_ptr->fz_r[p] = fz_medido[p];
            contact_ptr->is_contact[p] = (c[p] == 1);
            contact_ptr->umbral_r[p] = u_d[p];         
            contact_ptr->v_xyz[p][0] = vx_medido[p][1];
            contact_ptr->v_xyz[p][1] = vy_medido[p][2];
            contact_ptr->v_xyz[p][2] = vz_medido[p][0];
        }

        // Esperar estrictamente hasta el siguiente ciclo (mantener 250Hz exactos)
        std::this_thread::sleep_until(next_cycle);
    }

    // Limpieza al salir
    std::cout << "\n[INFO] Cerrando publicador de contacto." << std::endl;
    munmap(tel_ptr, sizeof(TelemetryData));
    munmap(contact_ptr, sizeof(ContactData));
    close(tel_fd);
    close(contact_fd);

    return 0;
}
