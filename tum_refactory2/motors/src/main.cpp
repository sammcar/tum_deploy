#include <iostream>
#include <vector>
#include <memory>
#include <csignal>
#include <thread>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <atomic>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include "moteus.h"

// --- INCLUSIONES PROPIAS ---
#include "robot_config.hpp"
#include "shared_memory.hpp"
#include "utils_motors.hpp"
#include "kinematics.hpp"

bool usar_watchdog_seguridad = true;
// --- GLOBALES ---
std::atomic<bool> g_running{true};
std::atomic<bool> g_monitor_activo{true}; // <-- El interruptor

// --- VARIABLES GLOBALES DE DEBUG (Sin tocar robot_config.hpp) ---
static double debug_sum_sq_error[4] = {0.0, 0.0, 0.0, 0.0}; // Para el RMSE
static double debug_max_error_mm[4] = {0.0, 0.0, 0.0, 0.0}; // Para el Peak Error
static int debug_ciclos_caminando[4] = {0, 0, 0, 0};

// Variables para mostrar en la pantalla (Monitor)
static double debug_last_rmse_mm[4] = {0.0, 0.0, 0.0, 0.0};
static double debug_last_peak_mm[4] = {0.0, 0.0, 0.0, 0.0};

enum class RobotState
{
    HOMING,
    READY, // Esperando comandos (Idle)
    ACTIVE,
    EMERGENCY // <--- NUEVO ESTADO PARA PROTECCIÓN   // Siguiendo consignas de la memoria compartida
};

// --- VARIABLE CENTRAL DEL ROBOT ---
Leg patas[4]; // 0:FL, 1:FR, 2:BL, 3:BR

// --- CAPA DE SEGURIDAD ACTIVA ---
bool verificar_seguridad_limites(Leg patas[4])
{
    const double MAX_TORQUE = 3.0;
    const int8_t MAX_TEMP = 65;
    const double MAX_POS_ERROR = 30 * (1.0 / 360.0);
    const double MIN_VEL_STALL = 0.03;

    for (int p = 0; p < 4; ++p)
    {
        MotorData *motors[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
        for (int m = 0; m < 3; ++m)
        {
            // 1. Protección por Torque
            if (std::abs(motors[m]->current_torque) > MAX_TORQUE)
            {
                std::cerr << "\n"
                          << std::string(45, '!') << "\n";
                std::cerr << "[FALLO: TORQUE EXCESIVO]\n";
                std::cerr << "Pata: " << p << " | ID: " << motors[m]->id << "\n";
                std::cerr << "Torque detectado: " << motors[m]->current_torque << " Nm\n";
                std::cerr << "Límite máximo:    " << MAX_TORQUE << " Nm\n";
                std::cerr << std::string(45, '!') << std::endl;
                return false;
            }

            // 2. Detección de Atasco (Stall) con reporte detallado
            double error_posicion = std::abs(motors[m]->target_pos - motors[m]->current_pos);
            double velocidad_actual = std::abs(motors[m]->current_vel);

            if ((error_posicion > MAX_POS_ERROR) && (velocidad_actual < MIN_VEL_STALL))
            {
                std::cerr << "\n"
                          << std::string(45, '-') << "\n";
                std::cerr << "[FALLO: ATASCO / STALL DETECTADO]\n";
                std::cerr << "Pata: " << p << " | ID: " << motors[m]->id << "\n";
                std::cerr << "Error de Posición: " << error_posicion * 360.0 << " grados\n";
                std::cerr << "Velocidad actual:  " << velocidad_actual << " rev/s\n";
                std::cerr << "Consigna (Target): " << motors[m]->target_pos * 360.0 << " deg\n";
                std::cerr << "Real (Actual):    " << motors[m]->current_pos * 360.0 << " deg\n";
                std::cerr << "Límite Error:      " << MAX_POS_ERROR * 360.0 << " deg\n";
                std::cerr << std::string(45, '-') << std::endl;
                return false;
            }

            // 3. Protección Térmica
            if (motors[m]->temperature > MAX_TEMP)
            {
                std::cerr << "\n"
                          << std::string(45, '*') << "\n";
                std::cerr << "[FALLO: SOBRECALENTAMIENTO]\n";
                std::cerr << "Pata: " << p << " | ID: " << motors[m]->id << "\n";
                std::cerr << "Temperatura: " << (int)motors[m]->temperature << " °C\n";
                std::cerr << "Límite:      " << (int)MAX_TEMP << " °C\n";
                std::cerr << std::string(45, '*') << std::endl;
                return false;
            }
        }
    }
    return true;
    return true;
}
// --- HILO DE MONITOREO ---
void MonitorLoop(Leg patas_ref[4])
{
    // Nombres para hacer la tabla más legible
    const char *nombres_patas[] = {"FL", "FR", "BL", "BR"};
    const char *nombres_joints[] = {"COXA", "FEMUR", "TIBIA"};

    while (g_running)
    {
        // Si el monitor está desactivado, no hacemos nada y esperamos
        if (!g_monitor_activo)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // Limpiamos pantalla y ponemos el cursor al inicio
        std::cout << "\033[2J\033[H";

        // =========================================================
        // TABLA 1: TELEMETRÍA (LO QUE EL MOTOR SIENTE)
        // =========================================================
        std::cout << "=========================================================\n";
        std::cout << "             [ MONITOR DE TELEMETRÍA ATOM-51 ]           \n";
        std::cout << "=========================================================\n";
        printf("%-4s | %-6s | %-3s | %-9s | %-8s | %-5s\n", "LEG", "JOINT", "ID", "POS(deg)", "TRQ(Nm)", "TEMP");
        std::cout << "---------------------------------------------------------\n";

        for (int p = 0; p < 4; ++p)
        {
            MotorData *m[3] = {&patas_ref[p].coxa, &patas_ref[p].femur, &patas_ref[p].tibia};

            for (int i = 0; i < 3; i++)
            {
                printf("%-4s | %-6s | %-3d | %9.2f | %8.2f | %-5d\n",
                       nombres_patas[p],
                       nombres_joints[i],
                       m[i]->id,
                       m[i]->current_pos * 360.0,
                       m[i]->current_torque,
                       m[i]->temperature);
            }
            if (p < 3)
                std::cout << "---------------------------------------------------------\n";
        }

        // =========================================================
        // TABLA 2: COMANDOS (LO QUE LE ESTAMOS ENVIANDO)
        // =========================================================
        std::cout << "\n===============================================================================\n";
        std::cout << "                        [ COMANDOS ENVIADOS (TARGET) ]                           \n";
        std::cout << "===============================================================================\n";
        // Ampliamos el encabezado para incluir Velocidad, KP y KD
        printf("%-4s | %-6s | %-3s | %-10s | %-10s | %-10s | %-5s | %-5s\n",
               "LEG", "JOINT", "ID", "TGT_POS(d)", "TGT_VEL", "FF_TRQ(Nm)", "KP", "KD");
        std::cout << "-------------------------------------------------------------------------------\n";

        for (int p = 0; p < 4; ++p)
        {
            MotorData *m[3] = {&patas_ref[p].coxa, &patas_ref[p].femur, &patas_ref[p].tibia};

            for (int i = 0; i < 3; i++)
            {
                // Añadimos target_vel, kp y kd a la impresión
                printf("%-4s | %-6s | %-3d | %10.2f | %10.2f | %10.2f | %5.2f | %5.2f\n",
                       nombres_patas[p],
                       nombres_joints[i],
                       m[i]->id,
                       m[i]->target_pos * 360.0,
                       m[i]->target_vel, // Velocidad deseada
                       m[i]->ff_torque,  // Fuerza virtual convertida a torque
                       m[i]->kp,         // Rigidez del motor
                       m[i]->kd);        // Amortiguamiento del motor
            }
            if (p < 3)
                std::cout << "-------------------------------------------------------------------------------\n";
        }
        // =========================================================
        // TABLA 3: RESULTADOS DE DIAGNÓSTICO (RMSE Y PEAK ERROR)
        // =========================================================
        std::cout << "\n===============================================================================\n";
        std::cout << "               [ RESULTADOS DEL ÚLTIMO TEST (TRAYECTORIA) ]              \n";
        std::cout << "===============================================================================\n";
        printf(" RMSE (mm) | FL: %5.2f | FR: %5.2f | BL: %5.2f | BR: %5.2f\n",
               debug_last_rmse_mm[0], debug_last_rmse_mm[1], 
               debug_last_rmse_mm[2], debug_last_rmse_mm[3]);
        printf(" PEAK (mm) | FL: %5.2f | FR: %5.2f | BL: %5.2f | BR: %5.2f\n",
               debug_last_peak_mm[0], debug_last_peak_mm[1], 
               debug_last_peak_mm[2], debug_last_peak_mm[3]);
        std::cout << "-------------------------------------------------------------------------------\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void enviar_informacion_motores(int p_idx,
                                Leg &pata,
                                MemoryManager &memory,
                                std::vector<std::shared_ptr<mjbots::moteus::Controller>> &controllers,
                                std::vector<mjbots::moteus::CanFdFrame> &tx_frames)
{
    // Agrupamos los punteros de los motores de esta pata para iterar
    MotorData *motors[3] = {&pata.coxa, &pata.femur, &pata.tibia};

    for (int m = 0; m < 3; ++m)
    {
        mjbots::moteus::PositionMode::Command cmd;

        // --- ASIGNACIÓN DE SETPOINTS ---
        cmd.position = motors[m]->target_pos;          // revoluciones
        cmd.velocity = motors[m]->target_vel;          // rev/s
        cmd.feedforward_torque = motors[m]->ff_torque; // Nm

        // --- GESTIÓN DE GANANCIAS (IMPEDANCIA) ---
        // Leemos escalas desde la memoria compartida
        double kp_mem = memory.cmd->kp_scale[p_idx][m];
        double kd_mem = memory.cmd->kd_scale[p_idx][m];

        // Aplicamos valores por defecto si la memoria viene en 0
        cmd.kp_scale = (kp_mem > 0.0) ? kp_mem : 0.7;
        cmd.kd_scale = (kd_mem > 0.0) ? kd_mem : 0.7;

        // Sincronizamos con las variables de monitoreo para el thread visual
        motors[m]->kp = cmd.kp_scale;
        motors[m]->kd = cmd.kd_scale;

        // --- GENERACIÓN DEL FRAME ---
        int controller_id = motors[m]->id;
        tx_frames.push_back(controllers[controller_id - 1]->MakePosition(cmd));
    }
}

// --- GENERADOR DE COMANDOS HOMING ---
void generar_comando_homing(double t,
                            const double start_pos[13],
                            Leg patas[4],
                            std::vector<std::shared_ptr<mjbots::moteus::Controller>> &controllers,
                            std::vector<mjbots::moteus::CanFdFrame> &tx_frames)
{
    for (int p = 0; p < 4; ++p)
    {
        MotorData *motors[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
        for (int m = 0; m < 3; ++m)
        {
            int id = motors[m]->id;

            // Calculamos el Set-point
            double p_target = start_pos[id] + t * (0.0 - start_pos[id]);
            motors[m]->target_pos = p_target; // Guardamos en la estructura para registro

            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = p_target;
            cmd.kp_scale = 0.4;
            cmd.kd_scale = 0.4;
            tx_frames.push_back(controllers[id - 1]->MakePosition(cmd));
        }
    }
}

void ejecutar_transicion_estatica(double t_proporcional, // de 0.0 a 1.0
                                  const double snapshot_inicial[13],
                                  Leg patas[4],
                                  MemoryManager &memory,
                                  std::vector<std::shared_ptr<mjbots::moteus::Controller>> &controllers,
                                  std::vector<mjbots::moteus::CanFdFrame> &tx_frames)
{
    for (int p = 0; p < 4; ++p)
    {
        MotorData *motors[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};

        for (int m = 0; m < 3; ++m)
        {
            int id = motors[m]->id;

            // 1. EL DESTINO: Ya no es 0.0, es lo que calculó la cinemática (goal_angles)
            double p_final = patas[p].goal_angles[m];
            double p_inicio = snapshot_inicial[id];

            // 2. LA MAGIA: Cambiamos la resta lineal por Minimum Jerk
            // t_proporcional hace el rol de (t_elapsed / t_total)
            double u = std::clamp(t_proporcional, 0.0, 1.0);
            double poly = (u * u * u) * (10.0 + u * (-15.0 + 6.0 * u));

            double p_target = p_inicio + (p_final - p_inicio) * poly;

            // Guardamos en la estructura para que el monitor lo vea
            motors[m]->target_pos = p_target;

            // 3. COMANDO MOTEUS
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = p_target;

            // En estático podemos subir un poco el KP para que tenga más fuerza (rigidez)
            cmd.kp_scale = 0.6;
            cmd.kd_scale = 0.6;

            tx_frames.push_back(controllers[id - 1]->MakePosition(cmd));
        }
    }
}

// --- ACTUALIZACIÓN DE TELEMETRÍA ---
void update_telemetry(const std::vector<mjbots::moteus::CanFdFrame> &rx_frames, Leg patas[4], MemoryManager &memory)
{
    for (const auto &frame : rx_frames)
    {
        const auto res = mjbots::moteus::Query::Parse(frame.data, frame.size);
        int p_idx = (frame.source - 1) / 3;
        int m_idx = (frame.source - 1) % 3;
        if (p_idx < 0 || p_idx > 3)
            continue;

        MotorData *m_ptr = (m_idx == 0) ? &patas[p_idx].coxa : (m_idx == 1) ? &patas[p_idx].femur
                                                                            : &patas[p_idx].tibia;

        // Guardamos el Feedback
        m_ptr->current_pos = res.position;
        m_ptr->current_vel = res.velocity;
        m_ptr->current_torque = res.torque;
        m_ptr->temperature = res.temperature;

        memory.tel->measured_angles[p_idx][m_idx] = res.position * 360.0;
        memory.tel->temperature[p_idx][m_idx] = res.temperature;
    }
}

// 1. AQUI CAMBIÓ A bool
bool procesar_cinematica_shm(MemoryManager &memory, Leg patas[4], double last_foot_positions[4][3])
{
    // 2. AQUI CREAMOS LA BANDERA
    bool comando_valido = false;

    for (int p = 0; p < 4; ++p)
    {
        double x = memory.cmd->foot_positions[p][0];
        double y = memory.cmd->foot_positions[p][1];
        double z = memory.cmd->foot_positions[p][2];

        // --- FILTRO ANTI-CERO (GUARDABOSQUES) ---
        if (std::abs(x) < 0.0001 && std::abs(y) < 0.0001 && std::abs(z) < 0.0001)
        {
            continue;
        }

        // 3. AQUI ACTIVAMOS LA BANDERA (Significa que Python mandó algo diferente a 0.0)
        comando_valido = true;

        bool es_derecha = (p == 1 || p == 3);

        // 1. Calculamos la cinemática inversa
        LegAngles angles = solve_IK(x, y, z, es_derecha);

        // 2. Solo actualizamos si la posición es físicamente alcanzable
        if (angles.valid)
        {
            patas[p].coxa.target_pos = angles.th1 / 360.0;
            patas[p].femur.target_pos = angles.th2 / 360.0;
            patas[p].tibia.target_pos = angles.th3 / 360.0;

            // Actualizamos goal_angles para telemetría visual
            patas[p].goal_angles[0] = patas[p].coxa.target_pos;
            patas[p].goal_angles[1] = patas[p].femur.target_pos;
            patas[p].goal_angles[2] = patas[p].tibia.target_pos;
        }
    }

    // 4. AQUI DEVOLVEMOS EL RESULTADO AL BUCLE PRINCIPAL
    return comando_valido;
}
// --- NUEVA VERSIÓN EIGEN ---
Eigen::Matrix3d calcular_jacobiano_numerico(double th1, double th2, double th3, bool es_derecha, Eigen::Vector3d p0)
{
    Eigen::Matrix3d J;
    const double epsilon = 1e-6;

    // 1. Perturbar Theta 1 (Coxa) - Llena la Columna 0
    Eigen::Vector3d p1 = solve_FK(th1 + epsilon, th2, th3, es_derecha);
    J(0, 0) = (p1.x() - p0.x()) / epsilon;
    J(1, 0) = (p1.y() - p0.y()) / epsilon;
    J(2, 0) = (p1.z() - p0.z()) / epsilon;

    // 2. Perturbar Theta 2 (Fémur) - Llena la Columna 1
    Eigen::Vector3d p2 = solve_FK(th1, th2 + epsilon, th3, es_derecha);
    J(0, 1) = (p2.x() - p0.x()) / epsilon;
    J(1, 1) = (p2.y() - p0.y()) / epsilon;
    J(2, 1) = (p2.z() - p0.z()) / epsilon;

    // 3. Perturbar Theta 3 (Tibia) - Llena la Columna 2
    Eigen::Vector3d p3 = solve_FK(th1, th2, th3 + epsilon, es_derecha);
    J(0, 2) = (p3.x() - p0.x()) / epsilon;
    J(1, 2) = (p3.y() - p0.y()) / epsilon;
    J(2, 2) = (p3.z() - p0.z()) / epsilon;

    return J;
}

void actualizar_jacobianos_robot(Leg patas[4], MemoryManager &memory)
{
    for (int p = 0; p < 4; ++p)
    {
        // 1. Moteus da revoluciones. solve_FK y Jacobiano EXIGEN RADIANES.
        double th1_rad = patas[p].coxa.current_pos * 2.0 * M_PI;
        double th2_rad = patas[p].femur.current_pos * 2.0 * M_PI;
        double th3_rad = patas[p].tibia.current_pos * 2.0 * M_PI;

        bool es_derecha = (p == 1 || p == 3);

        // 2. Calculamos la posición real actual (con radianes)
        Vector3d pos_real = solve_FK(th1_rad, th2_rad, th3_rad, es_derecha);

        // 3. Guardamos la posición en metros
        patas[p].real_xyz = pos_real;

        // 4. El Jacobiano se calcula perturbando en radianes
        patas[p].Jacobiano = calcular_jacobiano_numerico(th1_rad, th2_rad, th3_rad, es_derecha, pos_real);
    }
}

// Añadimos los parámetros a la declaración
void compute_torque_velocity(Leg patas[4], MemoryManager &memory)
{
    for (int p = 0; p < 4; ++p)
    {
        Eigen::Vector3d p_des(
            memory.cmd->foot_positions[p][0],
            memory.cmd->foot_positions[p][1],
            memory.cmd->foot_positions[p][2]);

        Eigen::Vector3d v_des(
            memory.cmd->velocities[p][0],
            memory.cmd->velocities[p][1],
            memory.cmd->velocities[p][2]);

        // --- 1. NUEVO: CÁLCULO DE VELOCIDAD ARTICULAR (TGT_VEL) ---
        // Convertimos la velocidad cartesiana (m/s) a revoluciones por segundo para Moteus
        if (std::abs(patas[p].Jacobiano.determinant()) > 1e-6)
        {
            Eigen::Vector3d dq_des = patas[p].Jacobiano.inverse() * v_des;
            
            // --- CORRECCIÓN DE SIMETRÍA FÍSICA ---
            bool es_derecha = (p == 1 || p == 3);
            
            if (es_derecha) {
                // Invertimos el signo de la coxa para las patas derechas
                patas[p].coxa.target_vel = -dq_des.x() / (2.0 * M_PI);
            } else {
                patas[p].coxa.target_vel = dq_des.x() / (2.0 * M_PI);
            }

            patas[p].femur.target_vel = dq_des.y() / (2.0 * M_PI);
            patas[p].tibia.target_vel = dq_des.z() / (2.0 * M_PI);
        }
        else
        {
            patas[p].coxa.target_vel = 0.0;
            patas[p].femur.target_vel = 0.0;
            patas[p].tibia.target_vel = 0.0;
        }

        Eigen::Vector3d p_real = patas[p].real_xyz;
        Eigen::Vector3d error_pos = p_des - p_real;

        double dq1 = patas[p].coxa.current_vel * 2.0 * M_PI;
        double dq2 = patas[p].femur.current_vel * 2.0 * M_PI;
        double dq3 = patas[p].tibia.current_vel * 2.0 * M_PI;
        Eigen::Vector3d dq(dq1, dq2, dq3);

        Eigen::Vector3d v_real = patas[p].Jacobiano * dq;

        patas[p].realVel_xyz = v_real;
        
        Eigen::Vector3d error_vel = v_des - v_real;

        // 1. Sintaxis de Eigen: Se usa paréntesis para inicializar un vector, no corchetes de array
        Eigen::Vector3d Kp(70.0, 2.0, 30.0); // Rigidez (N/m)
        Eigen::Vector3d Kd(0.0, 0.0, 0.0); // Amortiguamiento (Ns/m)

        // 2. Producto de vectores: No puedes usar '*' directo entre dos vectores 3x1.
        // Se debe usar .cwiseProduct() para multiplicar X con X, Y con Y, etc.
        Eigen::Vector3d F_virtual = Kp.cwiseProduct(error_pos) + Kd.cwiseProduct(error_vel);

        Eigen::Vector3d Torque = patas[p].Jacobiano.transpose() * F_virtual; // Transponemos el Jacobiano para multiplicar con la fuerza virtual

        // --- INICIO CÁLCULO DE DEBUG ---
        double error_mm = error_pos.norm() * 1000.0; // Distancia real en milímetros
        
        // 1. Acumulamos el error AL CUADRADO para el RMSE
        debug_sum_sq_error[p] += (error_mm * error_mm);
        
        // 2. Atrapamos el Peak Error (Si este error es el más alto hasta ahora, lo guardamos)
        if (error_mm > debug_max_error_mm[p]) {
            debug_max_error_mm[p] = error_mm;
        }
        
        debug_ciclos_caminando[p]++;
        // --- FIN CÁLCULO DE DEBUG ---

        // --- CORRECCIÓN DE SIMETRÍA PARA EL TORQUE ---
        bool es_derecha = (p == 1 || p == 3);
        if (es_derecha) {
            patas[p].coxa.ff_torque = -Torque.x(); // ¡Invertido igual que la velocidad!
        } else {
            patas[p].coxa.ff_torque = Torque.x();
        }
        
        patas[p].femur.ff_torque = Torque.y();
        patas[p].tibia.ff_torque = Torque.z();
    }
}

// FUNCION 2: Calcula la interpolación en cada paso
void ejecutar_interpolacion_estatica(TransitionState &state, Leg patas[4], MemoryManager &memory)
{
    double t_limit = std::max(0.1, memory.cmd->transition_time);
    auto ahora = std::chrono::steady_clock::now();
    
    // 1. Calculamos el tiempo transcurrido en segundos
    double elap = std::chrono::duration<double>(ahora - state.tiempo_inicio).count();

    // 2. NORMALIZAMOS: Lo convertimos en un valor de 0.0 a 1.0
    double t_proporcional = std::clamp(elap / t_limit, 0.0, 1.0);

    // 3. LA CURVA MAGICA (Minimum Jerk puro)
    double poly = (t_proporcional * t_proporcional * t_proporcional) * (10.0 + t_proporcional * (-15.0 + 6.0 * t_proporcional));

    for (int p = 0; p < 4; ++p)
    {
        MotorData *m_ptr[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};

        for (int m = 0; m < 3; ++m)
        {
            double p_start = state.pos_inicial[m_ptr[m]->id]; // Foto tomada al inicio
            double p_target = state.angulos_objetivo[p][m];   // Objetivo de la IK

            // 4. Calculamos la posición usando el polinomio
            m_ptr[m]->target_pos = p_start + (p_target - p_start) * poly;

            // En modo estático, reseteamos velocidad y torque feedforward
            m_ptr[m]->target_vel = 0.0;
            m_ptr[m]->ff_torque = 0.0;
        }
    }
}

void iniciar_transicion_estatica(TransitionState &state, Leg patas[4], MemoryManager &memory)
{
    // 1. CAPTURAR EL MOMENTO EXACTO
    // Ponemos en marcha el cronómetro para la trayectoria
    state.tiempo_inicio = std::chrono::steady_clock::now();

    for (int p = 0; p < 4; ++p)
    {
        // 2. GUARDAR POSICIÓN INICIAL (SNAPSHOT)
        // Guardamos la posición REAL (current_pos) que reporta el motor ahora mismo.
        // Usamos el ID del motor como índice para no perdernos.
        state.pos_inicial[patas[p].coxa.id] = patas[p].coxa.current_pos;
        state.pos_inicial[patas[p].femur.id] = patas[p].femur.current_pos;
        state.pos_inicial[patas[p].tibia.id] = patas[p].tibia.current_pos;

        // 3. GUARDAR POSICIÓN OBJETIVO (GOAL)
        // Estos son los ángulos que procesar_cinematica_shm calculó desde la memoria.
        // Son nuestro "Punto B".
        state.angulos_objetivo[p][0] = patas[p].goal_angles[0];
        state.angulos_objetivo[p][1] = patas[p].goal_angles[1];
        state.angulos_objetivo[p][2] = patas[p].goal_angles[2];
    }

    // 4. ACTIVAR LA TRANSICIÓN
    // Esto evita que volvamos a tomar la "foto" en el siguiente ciclo (5ms después)
    state.activo = true;

    std::cout << "[TRANSITION] Snapshot capturado. Iniciando movimiento suave del ATOM-51." << std::endl;
}

void control_robot(std::shared_ptr<mjbots::moteus::Transport> transport,
                   std::vector<std::shared_ptr<mjbots::moteus::Controller>> &controllers,
                   Leg patas[4],
                   MemoryManager &memory)
{
    RobotState estado = RobotState::HOMING;

    static TransitionState state; // Declaramos la variable de estado de transición

    bool fist_static = false;
    auto t_start = std::chrono::steady_clock::now();
    const double duracion_homing = 3.0;
    double start_pos[13] = {0.0};
    bool init_captured = false;
    double last_foot_positions[4][3] = {0.0};

    auto proximo_ciclo = std::chrono::steady_clock::now();

    while (g_running)
    {
        std::vector<mjbots::moteus::CanFdFrame> tx_frames, rx_frames;
        auto ahora = std::chrono::steady_clock::now();

        // --- 1. EVALUACIÓN DE SEGURIDAD ---
        // Usa la telemetría recibida al final del ciclo anterior
        if (usar_watchdog_seguridad && estado != RobotState::EMERGENCY)
        {
            if (!verificar_seguridad_limites(patas))
            {
                g_monitor_activo = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                estado = RobotState::EMERGENCY;

                std::cout << "\n"
                          << std::string(60, '!') << std::endl;
                std::cout << "  [ALERTA DE SEGURIDAD] WATCHDOG DISPARADO" << std::endl;
                std::cout << "  MOTIVO: Violación de límites físicos en el ATOM-51" << std::endl;
                std::cout << std::string(60, '!') << std::endl;
            }
        }

        // --- 2. MÁQUINA DE ESTADOS (Cálculo de tx_frames) ---
        if (estado == RobotState::EMERGENCY)
        {
            for (auto &c : controllers)
                tx_frames.push_back(c->MakeStop());
        }
        else if (estado == RobotState::HOMING)
        {
            double elapsed = std::chrono::duration<double>(ahora - t_start).count();
            double t = std::clamp(elapsed / duracion_homing, 0.0, 1.0);

            if (!init_captured)
            {
                // Capturamos las posiciones reales actuales para el inicio del homing
                for (int i = 1; i <= 12; i++)
                {
                    int p_idx = (i - 1) / 3;
                    int m_idx = (i - 1) % 3;
                    MotorData *m_ptr = (m_idx == 0) ? &patas[p_idx].coxa : (m_idx == 1) ? &patas[p_idx].femur
                                                                                        : &patas[p_idx].tibia;
                    start_pos[i] = m_ptr->current_pos;
                }
                init_captured = true;
            }

            generar_comando_homing(t, start_pos, patas, controllers, tx_frames);
            if (t >= 1.0)
            {
                // --- SOLUCIÓN: SINCRONIZAR AL TERMINAR EL HOMING ---
                for (int p = 0; p < 4; ++p)
                {
                    patas[p].goal_angles[0] = patas[p].coxa.current_pos;
                    patas[p].goal_angles[1] = patas[p].femur.current_pos;
                    patas[p].goal_angles[2] = patas[p].tibia.current_pos;
                }
                estado = RobotState::ACTIVE;
                std::cout << "[SYSTEM] ATOM-51 en modo ACTIVE." << std::endl;
            }
        }
        else if (estado == RobotState::ACTIVE)
        {
            // --- NUEVO: Bandera para saber cuándo termina el test ---
            static bool estaba_caminando = false; 

            // 1. RECOGEMOS EL AVISO DE LA CINEMÁTICA
            bool hay_comando = procesar_cinematica_shm(memory, patas, last_foot_positions);

            // 2. SOLO ACTUAMOS SI PYTHON MANDÓ ALGO DIFERENTE A 0.0
            if (hay_comando)
            {
                if (memory.cmd->is_walking)
                {
                    state.activo = false; // Reset al caminar
                    actualizar_jacobianos_robot(patas, memory);
                    compute_torque_velocity(patas, memory);
                    
                    // --- NUEVO: Recordamos que estamos en medio de un test ---
                    estaba_caminando = true; 
                }
                else
                {
                    // --- NUEVO: INICIO REPORTE DE DEBUG (RMSE y PEAK) ---
                    if (estaba_caminando)
                    {
                        for (int p = 0; p < 4; ++p) {
                            if (debug_ciclos_caminando[p] > 0) {
                                // Calculamos el RMSE: Raíz cuadrada del promedio de los cuadrados
                                debug_last_rmse_mm[p] = std::sqrt(debug_sum_sq_error[p] / debug_ciclos_caminando[p]);
                                
                                // Pasamos el Error Máximo atrapado
                                debug_last_peak_mm[p] = debug_max_error_mm[p];
                            }
                            // Limpiamos los acumuladores para la próxima prueba
                            debug_sum_sq_error[p] = 0.0;
                            debug_max_error_mm[p] = 0.0;
                            debug_ciclos_caminando[p] = 0;
                        }
                        estaba_caminando = false; // Apagamos la bandera para no repetir el cálculo
                    }
                    // --- FIN REPORTE DE DEBUG ---

                    if (!state.activo)
                    {
                        iniciar_transicion_estatica(state, patas, memory);
                    }
                    ejecutar_interpolacion_estatica(state, patas, memory);
                }
            }
            else
            {
                // Si la memoria tiene ceros, no bloqueamos la transición
                state.activo = false;
            }

            // 3. Mantenemos vivos los motores
            for (int p = 0; p < 4; ++p)
            {
                enviar_informacion_motores(p, patas[p], memory, controllers, tx_frames);
            }
        }
        // --- 3. TRANSACCIÓN UNIFICADA (Enviar y Recibir en un solo paso) ---
        // Esto reduce a la mitad el tráfico en el bus CAN
        SafeTransportCycle(transport, tx_frames, &rx_frames);

        // --- 4. ACTUALIZAR TELEMETRÍA PARA EL PRÓXIMO CICLO ---
        update_telemetry(rx_frames, patas, memory);

        // 5. Sincronización 200Hz
        proximo_ciclo += std::chrono::milliseconds(5);
        std::this_thread::sleep_until(proximo_ciclo);
    }
}

bool sincronizar_telemetria_inicial(std::shared_ptr<mjbots::moteus::Transport> transport,
                                    std::vector<std::shared_ptr<mjbots::moteus::Controller>> &controllers,
                                    Leg patas[4],
                                    MemoryManager &memory)
{
    const int MAX_INTENTOS = 20;
    std::cout << "\n[SETUP] Sincronizando telemetría inicial..." << std::endl;

    for (int intento = 1; intento <= MAX_INTENTOS; ++intento)
    {
        std::vector<mjbots::moteus::CanFdFrame> tx_init, rx_init;
        for (auto &c : controllers)
            tx_init.push_back(c->MakeQuery());

        SafeTransportCycle(transport, tx_init, &rx_init);
        update_telemetry(rx_init, patas, memory);

        // Verificamos si todos los motores respondieron (Temp > 0)
        int motores_listos = 0;
        for (int p = 0; p < 4; ++p)
        {
            if (patas[p].coxa.temperature > 0)
                motores_listos++;
            if (patas[p].femur.temperature > 0)
                motores_listos++;
            if (patas[p].tibia.temperature > 0)
                motores_listos++;
        }

        if (motores_listos == 12)
        {
            std::cout << "✅ Intento " << intento << ": ¡Los 12 motores están ONLINE!" << std::endl;
            return true;
        }

        std::cout << "⚠️  Intento " << intento << ": Solo " << motores_listos << "/12 motores listos. Reintentando..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cerr << "❌ ERROR CRÍTICO: No se pudo establecer comunicación con todos los motores después de " << MAX_INTENTOS << " intentos." << std::endl;
    return false;
}
// --- MAIN ---
int main(int argc, char **argv)
{
    optimizar_recursos_pi4();
    std::signal(SIGINT, SignalHandler);
    mjbots::moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = mjbots::moteus::Controller::MakeSingletonTransport({});

    MemoryManager memory;
    if (!memory.is_valid())
        return 1;

    for (int p = 0; p < 4; p++)
    {
        for (int j = 0; j < 3; j++)
        {
            memory.cmd->foot_positions[p][j] = 0.0;
            memory.cmd->velocities[p][j] = 0.0;
            memory.cmd->kp_scale[p][j] = 1.0; // Dejamos ganancias listas
            memory.cmd->kd_scale[p][j] = 1.0;
        }
        // --- AÑADE ESTO ---
        memory.cmd->is_walking = false;    // Arranca siempre en estático
        memory.cmd->transition_time = 2.0; // Tiempo de transición de seguridad
    }
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;

    inicializar_robot(patas, transport, controllers);

    // 1. Sincronización Inicial (Vital para evitar latigazos)
    // Si no se sincroniza, el programa se cierra antes de hacer nada peligroso
    if (!sincronizar_telemetria_inicial(transport, controllers, patas, memory))
    {
        std::cerr << "Abortando inicio por falla de hardware." << std::endl;
        return 1;
    }

    // --- PASO CRÍTICO: IGUALAR TARGETS A LA REALIDAD ---
    // Esto evita que el Watchdog detecte un error de posición al iniciar el bucle.
    for (int p = 0; p < 4; ++p)
    {
        // Sincronizamos targets inmediatos
        patas[p].coxa.target_pos = patas[p].coxa.current_pos;
        patas[p].femur.target_pos = patas[p].femur.current_pos;
        patas[p].tibia.target_pos = patas[p].tibia.current_pos;

        // Sincronizamos los objetivos de la cinemática
        patas[p].goal_angles[0] = patas[p].coxa.current_pos;
        patas[p].goal_angles[1] = patas[p].femur.current_pos;
        patas[p].goal_angles[2] = patas[p].tibia.current_pos;
    }
    std::cout << "✅ Estados internos sincronizados. Listos para control dinámico." << std::endl;

    // 2. Iniciar Hilo de Monitoreo
    std::thread monitor_thread(MonitorLoop, patas);

    if (g_running)
    {
        control_robot(transport, controllers, patas, memory);
    }

    // 3. Cierre ordenado
    if (monitor_thread.joinable())
        monitor_thread.join();

    for (auto &c : controllers)
        c->SetStop();
    return 0;
}
