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
#include "dart_test.hpp"

bool usar_watchdog_seguridad = true;
// --- GLOBALES ---
std::atomic<bool> g_running{true};
std::atomic<bool> g_monitor_activo{true}; // <-- El interruptor

// --- CONSTANTES DE CONFIGURACIÓN ---
static const int PATA_ACTIVA_DEBUG = 0; // -1 para todas, 0-3 para una específica

// --- VARIABLES DE TELEMETRÍA Y DEBUG ---
static double debug_sum_sq_error[4] = {0, 0, 0, 0};
static double debug_max_error_mm[4] = {0, 0, 0, 0};
static int debug_ciclos_caminando[4] = {0, 0, 0, 0};
static double debug_last_rmse_mm[4] = {0, 0, 0, 0};
static double debug_last_peak_mm[4] = {0, 0, 0, 0};

// ---> AÑADE ESTO <---
static double debug_target_x[4] = {0, 0, 0, 0};
static double debug_target_y[4] = {0, 0, 0, 0};
static double debug_target_z[4] = {0, 0, 0, 0};

struct CartesianState
{
    Eigen::Vector3d P_des;
    Eigen::Vector3d V_des;
    Eigen::Vector3d A_des;
    double kp_scale;
    double kd_scale;
    bool is_stance;
};

class LegGaitPlanner {
private:
    int leg_id_;
    Eigen::Vector3d origin_;
    Eigen::Vector3d hip_offset_;
    
    double gait_offset_;
    double duty_factor_;
    double step_duration_;
    double step_h_;

    // Parámetros de ajuste de trayectoria (Swing)
    double world_blend_ = 0.15;
    double damp_start_phase_ = 0.85;
    double damp_scale_kp_ = 0.6;
    double damp_scale_kd_ = 0.4;

    void EvaluateBezierCubic(double t, const Eigen::Vector3d& P0, const Eigen::Vector3d& P1, 
                             const Eigen::Vector3d& P2, const Eigen::Vector3d& P3,
                             Eigen::Vector3d& P_out, Eigen::Vector3d& V_out, Eigen::Vector3d& A_out) {
        double u = 1.0 - t;
        double tt = t * t; double uu = u * u;
        double uuu = uu * u; double ttt = tt * t;

        P_out = uuu * P0 + 3 * uu * t * P1 + 3 * u * tt * P2 + ttt * P3;
        V_out = 3 * uu * (P1 - P0) + 6 * u * t * (P2 - P1) + 3 * tt * (P3 - P2);
        A_out = 6 * u * (P2 - 2 * P1 + P0) + 6 * t * (P3 - 2 * P2 + P1);
    }

public:
    LegGaitPlanner() {}

    void Initialize(int leg_id, const Eigen::Vector3d& origin, const Eigen::Vector3d& hip_offset,
                    double gait_offset, double duty_factor, double step_duration, double step_h) {
        leg_id_ = leg_id;
        origin_ = origin;
        hip_offset_ = hip_offset;
        gait_offset_ = gait_offset;
        duty_factor_ = duty_factor;
        step_duration_ = step_duration;
        step_h_ = step_h;
    }

    CartesianState Update(double global_phase, double vx, double vy, double wz) {
        CartesianState state;
        
        double leg_phase = fmod(global_phase + gait_offset_, 1.0);
        double stance_portion = duty_factor_;
        double swing_portion = 1.0 - duty_factor_;
        
        state.is_stance = (leg_phase >= swing_portion);

        double t_stance = step_duration_ * stance_portion;
        Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0,0,wz).cross(hip_offset_);
        
        Eigen::Vector3d p_start = origin_ - v_total * (t_stance / 2.0);
        Eigen::Vector3d p_target = origin_ + v_total * (t_stance / 2.0);

        if (state.is_stance) {
            double t = (leg_phase - swing_portion) / stance_portion;
            
            state.P_des = p_target + t * (p_start - p_target);
            state.P_des.z() = origin_.z();
            state.V_des = -v_total; 
            state.A_des.setZero();
            
            state.kp_scale = 1.0;
            state.kd_scale = 1.0;
        } 
        else {
            double phi = leg_phase / swing_portion; 
            double flight_duration = step_duration_ * swing_portion;

            Eigen::Vector3d P, V, A;
            P.setZero(); V.setZero(); A.setZero();

            // Eje Z
            double z_max = p_start.z() + step_h_;
            if (phi < 0.5) {
                double phi_z = phi * 2.0; 
                P.z() = p_start.z() + (z_max - p_start.z()) * std::sin(phi_z * M_PI / 2.0);
                V.z() = (z_max - p_start.z()) * std::cos(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            } else {
                double phi_z = (phi - 0.5) * 2.0;
                P.z() = z_max - (z_max - p_target.z()) * (1.0 - std::cos(phi_z * M_PI / 2.0));
                V.z() = -(z_max - p_target.z()) * std::sin(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            }

            // Ejes XY
            Eigen::Vector3d xy_start(p_start.x(), p_start.y(), 0.0);
            Eigen::Vector3d xy_target(p_target.x(), p_target.y(), 0.0);
            Eigen::Vector3d V_world_xy(-v_total.x(), -v_total.y(), 0.0);

            if (phi < world_blend_) {
                double t_local = phi / world_blend_;
                V.head<2>() = V_world_xy.head<2>() * (1.0 - t_local);
                P.head<2>() = xy_start.head<2>() + (V_world_xy.head<2>() * (phi * flight_duration)); 
            } 
            else if (phi < 1.0 - world_blend_)
            {
                double t_local = (phi - world_blend_) / (1.0 - 2.0 * world_blend_);
                Eigen::Vector3d P1 = xy_start + (xy_target - xy_start) * 0.2;
                Eigen::Vector3d P2 = xy_start + (xy_target - xy_start) * 0.8;
                
                // 1. Usamos vectores temporales para la curva de Bezier
                Eigen::Vector3d P_bez, V_bez, A_bez;
                EvaluateBezierCubic(t_local, xy_start, P1, P2, xy_target, P_bez, V_bez, A_bez);
                
                // 2. Rescatamos la Z original que calculaste con el seno/coseno
                double altura_z_guardada = P.z(); 
                double velocidad_z_guardada = V.z();

                // 3. Asignamos el resultado de Bezier
                P = P_bez;
                V = V_bez / (flight_duration * (1.0 - 2.0 * world_blend_));
                A = A_bez;

                // 4. Restauramos la Z correcta (¡Salvamos al robot!)
                P.z() = altura_z_guardada;
                V.z() = velocidad_z_guardada;
            }
            else {
                double t_local = (phi - (1.0 - world_blend_)) / world_blend_;
                P.head<2>() = xy_target.head<2>(); 
                V.head<2>() = V_world_xy.head<2>() * t_local; 
            }

            state.P_des = P;
            state.V_des = V;
            state.A_des = A;

            // Amortiguación final del vuelo
            if (phi > damp_start_phase_) {
                state.kp_scale = damp_scale_kp_;
                state.kd_scale = damp_scale_kd_;
            } else {
                state.kp_scale = 1.0;
                state.kd_scale = 1.0;
            }
        }

        return state;
    }
};


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
    const double MAX_TORQUE = 4.0;
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
}
// --- HILO DE MONITOREO ---
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
            break; // Esto congela la última tabla visible.
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
        std::cout << "                        [ COMANDOS ENVIADOS (TARGET) ]                         \n";
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
        // TABLA 3: TRAYECTORIA CARTESIANA (GENERADOR DE MARCHA)
        // =========================================================
        std::cout << "\n===============================================================================\n";
        std::cout << "                 [ TRAYECTORIA CARTESIANA DESEADA (M) ]                        \n";
        std::cout << "===============================================================================\n";
        printf("%-4s | %-15s | %-15s | %-15s\n", "LEG", "X Deseado", "Y Deseado", "Z Deseado");
        std::cout << "-------------------------------------------------------------------------------\n";
        for (int p = 0; p < 4; ++p)
        {
            printf("%-4s | %15.4f | %15.4f | %15.4f\n",
                   nombres_patas[p],
                   debug_target_x[p],
                   debug_target_y[p],
                   debug_target_z[p]);
        }
        std::cout << "===============================================================================\n";

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
            cmd.kp_scale = 1.0;
            cmd.kd_scale = 1.0;
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
        state.pos_inicial[patas[p].coxa.id] = patas[p].coxa.target_pos;
        state.pos_inicial[patas[p].femur.id] = patas[p].femur.target_pos;
        state.pos_inicial[patas[p].tibia.id] = patas[p].tibia.target_pos;

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

    // 1. INICIALIZAR MODELOS DART (¡ESTO ES LO QUE FALTA!)
    LegDimensions dims;
    dart::dynamics::SkeletonPtr patas_dart[4];
    for (int i = 0; i < 4; i++)
    {
        patas_dart[i] = CreateLegSkeleton(dims);
    }

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

        if (usar_watchdog_seguridad && estado != RobotState::EMERGENCY)
        {
            if (!verificar_seguridad_limites(patas))
            {
                g_monitor_activo = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
            // 1. EL HOMING: Lleva todo a 0 para indexar
            double elapsed = std::chrono::duration<double>(ahora - t_start).count();
            double t = std::clamp(elapsed / duracion_homing, 0.0, 1.0);

            if (!init_captured)
            {
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
                // 2. PREPARAR TRANSICIÓN SUAVE
                // En lugar de saltar a ACTIVE, calculamos la postura segura de pie (Stand)
                for (int p = 0; p < 4; ++p)
                {
                    bool es_derecha = (p == 1 || p == 3);
                    // Calculamos a qué ángulos equivale Z = -0.306
                    LegAngles angles = solve_IK(0.01, (es_derecha ? -0.093 : 0.093), -0.306, es_derecha);

                    if (angles.valid)
                    {
                        patas[p].goal_angles[0] = angles.th1 / 360.0;
                        patas[p].goal_angles[1] = angles.th2 / 360.0;
                        patas[p].goal_angles[2] = angles.th3 / 360.0;
                    }
                    else
                    {
                        // Fallback de seguridad si falla la IK
                        patas[p].goal_angles[0] = 0.0;
                        patas[p].goal_angles[1] = 0.0;
                        patas[p].goal_angles[2] = 0.0;
                    }
                }

                // Iniciamos la curva suave
                iniciar_transicion_estatica(state, patas, memory);
                estado = RobotState::READY; // <--- VAMOS AL ESTADO INTERMEDIO
                std::cout << "[SYSTEM] Iniciando transición suave a postura Stand..." << std::endl;
            }
        }
        else if (estado == RobotState::READY)
        {
            // 3. ESTADO READY: Se agacha suavemente sin latigazos
            ejecutar_interpolacion_estatica(state, patas, memory);

            // Reutilizamos tu función de enviar
            for (int p = 0; p < 4; ++p)
            {
                enviar_informacion_motores(p, patas[p], memory, controllers, tx_frames);
            }

            double elap = std::chrono::duration<double>(ahora - state.tiempo_inicio).count();
            if (elap >= memory.cmd->transition_time)
            {
                estado = RobotState::ACTIVE;
                std::cout << "[SYSTEM] ATOM-51 en modo ACTIVE. ¡Listo para caminar!" << std::endl;
            }
        }
        else if (estado == RobotState::ACTIVE)
        {
            for (int p = 0; p < 4; ++p)
            {
                if (PATA_ACTIVA_DEBUG != -1 && p != PATA_ACTIVA_DEBUG)
                    continue;

                bool es_derecha = (p == 1 || p == 3);
                auto coxa_j = patas_dart[p]->getJoint("shoulder_joint");
                auto femur_j = patas_dart[p]->getJoint("femur_joint");
                auto tibia_j = patas_dart[p]->getJoint("tibia_joint");
                auto pie_node = patas_dart[p]->getBodyNode("foot");

                // --- 1. CAPTURAR FEEDBACK REAL (EL PRESENTE) ---
                EstadoRealMotores real;
                real.angulos_rad = {
                    patas[p].coxa.current_pos * 2.0 * M_PI,
                    patas[p].femur.current_pos * 2.0 * M_PI,
                    patas[p].tibia.current_pos * 2.0 * M_PI};

                real.velocidades_rad_s = {
                    patas[p].coxa.current_vel * 2.0 * M_PI,
                    patas[p].femur.current_vel * 2.0 * M_PI,
                    patas[p].tibia.current_vel * 2.0 * M_PI};

                
                // 1. Variables estáticas: Conservan su valor en cada iteración (a 200Hz) sin ser globales
                static LegGaitPlanner planners[4];
                static bool gait_initialized = false;
                static std::chrono::time_point<std::chrono::steady_clock> t_active_start;

                // Definimos los orígenes individuales de cada pata (Postura nominal)
                const Eigen::Vector3d LEGS_STAND_XYZ[4] = {
                    Eigen::Vector3d( 0.01,  0.093, -0.306), // 0: FL
                    Eigen::Vector3d( 0.01, -0.093, -0.306), // 1: FR
                    Eigen::Vector3d( 0.01,  0.093, -0.306), // 2: BL
                    Eigen::Vector3d( 0.01, -0.093, -0.306)  // 3: BR
                };

                // Parámetros de la marcha (Trot)
                const double gait_offsets[4] = {0.0, 0.5, 0.5, 0.0}; 
                double duty_factor = 0.5;   // 50% stance, 50% swing
                double step_duration = 0.8; // 1 segundo por ciclo de paso completo
                double step_h = 0.05;       // 5 cm de despeje (altura) al volar

                // 2. Inicialización de los 4 planificadores (Se ejecuta SOLO UNA VEZ al pasar a ACTIVE)
                if (!gait_initialized) {
                    for(int i = 0; i < 4; i++) {
                        // El offset de cadera se extrae del valor nominal en Y (vital para rotación wz)
                        Eigen::Vector3d offset_cadera(0.0, LEGS_STAND_XYZ[i].y(), 0.0);
                        
                        planners[i].Initialize(i, LEGS_STAND_XYZ[i], offset_cadera, 
                                               gait_offsets[i], duty_factor, step_duration, step_h);
                    }
                    t_active_start = ahora; // Capturamos el "Tiempo Cero" de la marcha
                    gait_initialized = true;
                }

                 // 3. RELOJ DINÁMICO: Calculamos la fase global cíclica (de 0.0 a 1.0)
                double elapsed_active = std::chrono::duration<double>(ahora - t_active_start).count();
                double global_phase = fmod(elapsed_active / step_duration, 1.0);

                // 4. Velocidad de avance de prueba (Aquí más adelante tu colega conectará un control remoto)
                double vx = 0.1, vy = 0.0, wz = 0.0;

                // 5. Tu clase hace toda la matemática temporal y devuelve el estado IDEAL para ESTA pata 'p'
                CartesianState ideal = planners[p].Update(global_phase, vx, vy, wz);
                
                debug_target_x[p] = ideal.P_des.x();
                debug_target_y[p] = ideal.P_des.y();
                debug_target_z[p] = ideal.P_des.z();

                EstadoDeseadoCartesiano deseado_dummy;
                deseado_dummy.posicion = ideal.P_des;
                deseado_dummy.velocidad = ideal.V_des;
                deseado_dummy.aceleracion = ideal.A_des;
                deseado_dummy.kp_scale = ideal.kp_scale;
                deseado_dummy.kd_scale = ideal.kd_scale;
                deseado_dummy.stance_actual = ideal.is_stance ? 1.0 : 0.0;

                // --- 3. CINEMÁTICA INVERSA PARA EL MOTOR (El PD Interno) ---
                LegAngles angles = solve_IK(ideal.P_des.x(), ideal.P_des.y(), ideal.P_des.z(), es_derecha);
                if (angles.valid)
                {
                    patas[p].coxa.target_pos = angles.th1 / 360.0;
                    patas[p].femur.target_pos = angles.th2 / 360.0;
                    patas[p].tibia.target_pos = angles.th3 / 360.0;
                }

                // --- 4. PREPARAR DART (Sincronizar con el PRESENTE, no con el futuro) ---
                // ✅ ESTA ES LA CORRECCIÓN CLAVE
                coxa_j->setPosition(0, real.angulos_rad(0));
                femur_j->setPosition(0, real.angulos_rad(1));
                tibia_j->setPosition(0, real.angulos_rad(2));
            
                pie_node->getSkeleton()->computeForwardKinematics(); // Refrescar el modelo

                // --- 5. CÁLCULO DEL ORÁCULO DART ---
                // DART usa el presente (real) y mira hacia el futuro (deseado_dummy)
                ComandosMotor m_cmd = calcular_comandos_motores(patas_dart[p], coxa_j, femur_j, tibia_j, pie_node,
                                                                real, deseado_dummy);

                // --- 6. ASIGNACIÓN FINAL PARA ENVÍO ---
                patas[p].coxa.target_vel = m_cmd.velocidades_rad_s(0) / (2.0 * M_PI);
                patas[p].femur.target_vel = m_cmd.velocidades_rad_s(1) / (2.0 * M_PI);
                patas[p].tibia.target_vel = m_cmd.velocidades_rad_s(2) / (2.0 * M_PI);
                

                //¡Recuerda descomentar esto cuando vayas a probar los torques reales!
                patas[p].coxa.ff_torque = m_cmd.torques_Nm(0);
                patas[p].femur.ff_torque = m_cmd.torques_Nm(1);
                patas[p].tibia.ff_torque = m_cmd.torques_Nm(2);
                
                const double MAX_FF_TORQUE = 3.5;
                //2. Extraemos y saturamos (clamp) los torques calculados por DART
                patas[p].coxa.ff_torque  = std::clamp(m_cmd.torques_Nm(0), -MAX_FF_TORQUE, MAX_FF_TORQUE);
                patas[p].femur.ff_torque = std::clamp(m_cmd.torques_Nm(1), -MAX_FF_TORQUE, MAX_FF_TORQUE);
                patas[p].tibia.ff_torque = std::clamp(m_cmd.torques_Nm(2), -MAX_FF_TORQUE, MAX_FF_TORQUE);
                
                patas[p].coxa.kp = deseado_dummy.kp_scale;
                patas[p].coxa.kd = deseado_dummy.kd_scale;
                patas[p].femur.kp = deseado_dummy.kp_scale;
                patas[p].femur.kd = deseado_dummy.kd_scale;
                patas[p].tibia.kp = deseado_dummy.kp_scale;
                patas[p].tibia.kd = deseado_dummy.kd_scale;

                // --- 7. EMPAQUETAR ---
                enviar_informacion_motores(p, patas[p], memory, controllers, tx_frames);
            }
        }
        // --- 3. TRANSACCIÓN UNIFICADA (Enviar y Recibir en un solo paso) ---
        // Esto reduce a la mitad el tráfico en el bus CAN
        SafeTransportCycle(transport, tx_frames, &rx_frames);

        // --- 4. ACTUALIZAR TELEMETRÍA PARA EL PRÓXIMO CICLO ---
        update_telemetry(rx_frames, patas, memory);

        // 5. Sincronización 1000Hz
        proximo_ciclo += std::chrono::milliseconds(1);
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
