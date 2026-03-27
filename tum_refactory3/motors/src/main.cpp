#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/signal_set.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <csignal>
#include <thread>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <future>
#include "moteus.h"

#include <sophus/se3.hpp> // <-- NUEVO: Para PropagateLeg
#include <sophus/so3.hpp> // <-- NUEVO: Para PropagateLeg

// --- INCLUSIONES PROPIAS ---
#include "robot_config.hpp"
#include "shared_memory.hpp"
#include "utils_motors.hpp"
#include "kinematics.hpp"
#include "dart_test.hpp"

using namespace mjbots;

// =========================================================
// 1. CONSTANTES GENERALIZADAS (El panel de control)
// =========================================================
constexpr int CONTROL_HZ = 400;
constexpr double DT = 1.0 / CONTROL_HZ;
constexpr int PERIOD_US = 1000000 / CONTROL_HZ;

const double DURACION_HOMING = 3.0;   // Segundos
const double TIEMPO_TRANSICION = 2.0; // Segundos para estado READY

const double MAX_TORQUE_DEFAULT = 6.0; // Nm permitidos en operación
const double max_control = 6.0;        // Nm permitidos en operación
// --- LÍMITES DEL WATCHDOG DE SEGURIDAD ---
bool usar_watchdog_seguridad = true;
const double WATCHDOG_MAX_TORQUE = 5.5;                     // Límite absoluto de emergencia
const double WATCHDOG_MAX_ERROR_POS = 30.0 * (1.0 / 360.0); // 30 grados
const double WATCHDOG_MIN_VEL_STALL = 0.03;
const int8_t WATCHDOG_MAX_TEMP = 65;

// =========================================================
// 2. GLOBALES DE ESTADO Y RED
// =========================================================
boost::asio::steady_timer *timer_global;
std::vector<moteus::CanFdFrame> tx_frames_global;
std::vector<moteus::CanFdFrame> rx_frames_global;
std::vector<std::shared_ptr<moteus::Controller>> controllers;

std::atomic<bool> g_running{true};
std::atomic<bool> g_monitor_activo{true};

Leg patas[4];                // 0:FL, 1:FR, 2:BL, 3:BR
MemoryManager memory_global; // Memoria compartida globalizada

enum class RobotState
{
    HOMING,
    READY,
    ACTIVE,
    EMERGENCY
};
RobotState estado_actual = RobotState::HOMING;

double start_pos[13] = {0.0};     // Posiciones iniciales de los 12 motores
double tiempo_estado = 0.0;       // Reloj puro matemático basado en DT
TransitionState state_transicion; // Para el estado READY

dart::dynamics::SkeletonPtr patas_dart_global[4];

static const int PATA_ACTIVA_DEBUG = 0; // -1 para todas

static double debug_target_x[4] = {0, 0, 0, 0};
static double debug_target_y[4] = {0, 0, 0, 0};
static double debug_target_z[4] = {0, 0, 0, 0};

// Declaraciones adelantadas
void HandleStatus(std::shared_ptr<moteus::Transport> transport);
void ProcesarTelemetria();
void PlanificarMovimiento();
void GenerarComandosCAN();
bool VerificarSeguridad();

// =========================================================
// 3. CLASES Y ESTRUCTURAS DE CONTROL (Actualizado con PropagateLeg)
// =========================================================
struct CartesianState
{
    Eigen::Vector3d P_des;
    Eigen::Vector3d V_des;
    Eigen::Vector3d A_des;
    double kp_scale;
    double kd_scale;
    bool is_stance;
};

// --- 3.1 CLASE PROPAGATE LEG (Para la fase de apoyo) ---
class PropagateLeg
{
public:
    PropagateLeg(const Eigen::Vector3d &v_R, const Eigen::Vector3d &w_R, double period_s)
        : v_R_(v_R), w_R_(w_R),
          pose_T2_T1_(Sophus::SO3d(Eigen::AngleAxisd(-period_s * w_R.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix()), -v_R_ * period_s) {}

    struct Result
    {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
    };

    Result operator()(const Eigen::Vector3d &position_R) const
    {
        Result result;
        result.position = pose_T2_T1_ * position_R;
        result.velocity = -v_R_ - w_R_.cross(position_R);
        return result;
    }

private:
    Eigen::Vector3d v_R_;
    Eigen::Vector3d w_R_;
    Sophus::SE3d pose_T2_T1_;
};

// --- 3.2 CLASE SWING TRAJECTORY (Para la fase de vuelo con Bézier 3D exacto) ---
class SwingTrajectory
{
public:
    struct Result
    {
        double phase;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity_s;
        Eigen::Vector3d acceleration_s2; // ¡Vital para DART!
    };

    SwingTrajectory(Eigen::Vector3d start, Eigen::Vector3d start_vel,
                    Eigen::Vector3d end, double height, double swing_time_s)
        : start_(start), start_vel_(start_vel), end_(end), height_(height),
          swing_time_s_(swing_time_s), phase_(0.0) {}

    Result Advance(double delta_s, Eigen::Vector3d world_velocity_s)
    {
        phase_ += delta_s / swing_time_s_;
        phase_ = std::clamp(phase_, 0.0, 1.0);

        Result res;
        res.phase = phase_;

        double world_blend_ = 0.15;
        double flight_duration = swing_time_s_;

        Eigen::Vector3d P, V, A;
        P.setZero();
        V.setZero();
        A.setZero();

        // === EJE Z (Movimiento Senoidal Puro) ===
        double z_max = std::max(start_.z(), end_.z()) + height_;
        if (phase_ < 0.5)
        {
            double phi_z = phase_ * 2.0;
            P.z() = start_.z() + (z_max - start_.z()) * std::sin(phi_z * M_PI / 2.0);
            V.z() = (z_max - start_.z()) * std::cos(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            A.z() = -(z_max - start_.z()) * std::sin(phi_z * M_PI / 2.0) * std::pow(M_PI / flight_duration, 2);
        }
        else
        {
            double phi_z = (phase_ - 0.5) * 2.0;
            P.z() = z_max - (z_max - end_.z()) * (1.0 - std::cos(phi_z * M_PI / 2.0));
            V.z() = -(z_max - end_.z()) * std::sin(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            A.z() = -(z_max - end_.z()) * std::cos(phi_z * M_PI / 2.0) * std::pow(M_PI / flight_duration, 2);
        }

        // === EJES X/Y (Lift -> Move Bezier -> Lower) ===
        Eigen::Vector3d xy_start(start_.x(), start_.y(), 0.0);
        Eigen::Vector3d xy_target(end_.x(), end_.y(), 0.0);

        // CORRECCIÓN 2: Usar la velocidad de inicio real para frenar la pata al levantarla
        Eigen::Vector3d V_start_xy(start_vel_.x(), start_vel_.y(), 0.0);
        Eigen::Vector3d V_world_xy(world_velocity_s.x(), world_velocity_s.y(), 0.0);

        double blend_time_s = world_blend_ * flight_duration;

        if (phase_ < world_blend_)
        {
            // LIFT: Despegue suave (Frena la velocidad que traía del suelo)
            double t_local = phase_ / world_blend_;
            double time_s = phase_ * flight_duration;

            A.head<2>() = -V_start_xy.head<2>() / blend_time_s;
            V.head<2>() = V_start_xy.head<2>() * (1.0 - t_local);

            // CORRECCIÓN 1: Integrar la posición para que coincida con la velocidad y aceleración
            P.head<2>() = xy_start.head<2>() + (V_start_xy.head<2>() * time_s) + (0.5 * A.head<2>() * time_s * time_s);
        }
        else if (phase_ < 1.0 - world_blend_)
        {
            // MOVE: Bézier Cúbico Central
            double t_local = (phase_ - world_blend_) / (1.0 - 2.0 * world_blend_);
            double duration_local = flight_duration * (1.0 - 2.0 * world_blend_);

            // CORRECCIÓN 3: El Bézier debe iniciar donde terminó LIFT y terminar donde inicia LOWER
            Eigen::Vector3d P0 = xy_start + 0.5 * blend_time_s * V_start_xy;
            Eigen::Vector3d P3 = xy_target - 0.5 * blend_time_s * V_world_xy;

            Eigen::Vector3d P1 = P0 + (P3 - P0) * 0.2;
            Eigen::Vector3d P2 = P0 + (P3 - P0) * 0.8;
            double u = 1.0 - t_local;

            // Derivadas Analíticas Exactas
            P.head<2>() = (u * u * u) * P0.head<2>() + 3 * (u * u) * t_local * P1.head<2>() + 3 * u * (t_local * t_local) * P2.head<2>() + (t_local * t_local * t_local) * P3.head<2>();
            Eigen::Vector3d vel_bezier = 3 * (u * u) * (P1 - P0) + 6 * u * t_local * (P2 - P1) + 3 * (t_local * t_local) * (P3 - P2);
            V.head<2>() = vel_bezier.head<2>() / duration_local;
            Eigen::Vector3d acc_bezier = 6 * u * (P2 - 2 * P1 + P0) + 6 * t_local * (P3 - 2 * P2 + P1);
            A.head<2>() = acc_bezier.head<2>() / (duration_local * duration_local);
        }
        else
        {
            // LOWER: Aterrizaje compensando velocidad
            double t_local = (phase_ - (1.0 - world_blend_)) / world_blend_;
            double time_in_lower_s = t_local * blend_time_s;

            A.head<2>() = V_world_xy.head<2>() / blend_time_s;
            V.head<2>() = V_world_xy.head<2>() * t_local;

            // CORRECCIÓN 1: Integrar la posición hacia el target para evitar la posición estática
            Eigen::Vector3d P_lower_start = xy_target - 0.5 * blend_time_s * V_world_xy;
            P.head<2>() = P_lower_start.head<2>() + (0.5 * A.head<2>() * time_in_lower_s * time_in_lower_s);
        }

        res.position = P;
        res.velocity_s = V;
        res.acceleration_s2 = A;
        return res;
    }

private:
    Eigen::Vector3d start_, start_vel_, end_;
    double height_, swing_time_s_, phase_;
};
// --- 3.3 ESTRUCTURA Y CÁLCULO DE TIEMPOS DE MARCHA DINÁMICOS ---
struct TrotTimes
{
    double swing_time;
    double flight_time;
    double twovleg_time;
    double onevleg_time;
};

// TrotTimes CalcularTiemposMarcha(double speed) //ORIGINAL
// {
//     TrotTimes t;
//     double limite_fase_a = 0.4;
//     if (speed < limite_fase_a)
//     {
//         t.swing_time = 0.30;
//         t.twovleg_time = 0.10; // Tiempo de anclaje al piso
//         t.flight_time = 0.0;
//     }
//     else
//     {
//         t.swing_time = 0.20;
//         t.flight_time = 0.05;
//         t.twovleg_time = 0.0;
//     }
//     return t;
// }

TrotTimes CalcularTiemposMarcha(double speed, double max_travel_dist = 0.15) {
    TrotTimes t;
    
    // Parámetros base configurables
    double max_swing_time = 0.8;
    double max_twovleg_time = 0.2;
    double max_flight_time = 0.05;

    // Evitar división por cero a velocidad muy cercana a 0
    if (speed < 0.01) speed = 0.01; 

    // FASE A: Caminata muy lenta (Pausa máxima en el suelo)
    if (speed < ((0.5 * max_travel_dist) / (max_twovleg_time + 0.5 * max_swing_time))) { 
        t.swing_time = max_swing_time;
        t.twovleg_time = max_twovleg_time;
        t.flight_time = 0.0;
    } 
    // FASE B: Transición (La pausa en el suelo se va reduciendo dinámicamente)
    else if (speed < (max_travel_dist / max_swing_time)) { 
        t.swing_time = max_swing_time;
        // La magia está aquí: A mayor velocidad, menor es el twovleg_time matemáticamente
        t.twovleg_time = (0.5 * max_travel_dist / speed) - (0.5 * max_swing_time);
        t.flight_time = 0.0;
    } 
    // FASE C: Trote rápido (Aparece el tiempo de vuelo, cero pausa)
    else if (speed < (max_travel_dist / (max_swing_time - 2 * max_flight_time))) { 
        t.swing_time = max_swing_time;
        t.twovleg_time = 0.0; // Ya no hay pausa
        // El tiempo de vuelo va creciendo poco a poco
        t.flight_time = (0.5 * t.swing_time) - (max_travel_dist / (2 * speed));
    } 
    // FASES D y E: Velocidad Máxima (Vuelo máximo, las patas se mueven más rápido)
    else {
        t.flight_time = max_flight_time;
        t.twovleg_time = 0.0;
        // Ya no podemos volar más tiempo, así que encogemos el tiempo de balanceo para ir más rápido
        t.swing_time = std::min(max_swing_time, (max_travel_dist / speed) + 2 * t.flight_time);
    }

    // Asegurar numéricamente que no haya tiempos negativos
    t.twovleg_time = std::max(0.0, t.twovleg_time);
    t.flight_time = std::max(0.0, t.flight_time);

    return t;
}

// =========================================================
// 5. EL METRÓNOMO ASÍNCRONO
// =========================================================
void HandleTimer(const boost::system::error_code &ec, std::shared_ptr<moteus::Transport> transport)
{
    if (ec || !g_running)
        return;

    timer_global->expires_at(timer_global->expiry() + std::chrono::microseconds(PERIOD_US));
    timer_global->async_wait([transport](const boost::system::error_code &e)
                             { HandleTimer(e, transport); });

    transport->Cycle(tx_frames_global.data(), tx_frames_global.size(), &rx_frames_global,
                     [transport](int)
                     { HandleStatus(transport); });
}

// =========================================================
// 6. PIPELINE PRINCIPAL Y CEREBRO
// =========================================================
void HandleStatus(std::shared_ptr<moteus::Transport> transport)
{
    if (!g_running)
        return;

    ProcesarTelemetria();

    if (usar_watchdog_seguridad && estado_actual != RobotState::EMERGENCY)
    {
        if (!VerificarSeguridad())
        {
            g_monitor_activo = false;
            estado_actual = RobotState::EMERGENCY;
        }
    }

    PlanificarMovimiento();
    GenerarComandosCAN();
}

void ProcesarTelemetria()
{
    for (const auto &frame : rx_frames_global)
    {
        const auto res = moteus::Query::Parse(frame.data, frame.size);
        int p_idx = (frame.source - 1) / 3;
        int m_idx = (frame.source - 1) % 3;
        if (p_idx < 0 || p_idx > 3)
            continue;

        MotorData *m_ptr = (m_idx == 0) ? &patas[p_idx].coxa : (m_idx == 1) ? &patas[p_idx].femur
                                                                            : &patas[p_idx].tibia;
        m_ptr->current_pos = res.position;
        m_ptr->current_vel = res.velocity;
        m_ptr->current_torque = res.torque;
        m_ptr->temperature = res.temperature;

        memory_global.tel->measured_angles[p_idx][m_idx] = res.position * 360.0;
        memory_global.tel->temperature[p_idx][m_idx] = res.temperature;
    }
}

bool VerificarSeguridad()
{
    for (int p = 0; p < 4; ++p)
    {
        MotorData *motors[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
        for (int m = 0; m < 3; ++m)
        {
            // 1. Protección por Torque
            if (std::abs(motors[m]->current_torque) > WATCHDOG_MAX_TORQUE)
            {
                std::cerr << "\n" << std::string(50, '!') << "\n";
                std::cerr << "🚨 [EMERGENCIA: TORQUE EXCESIVO]\n";
                std::cerr << "Pata: " << p << " | Motor ID: " << motors[m]->id << "\n";
                std::cerr << "   -> Torque detectado: " << motors[m]->current_torque << " Nm\n";
                std::cerr << "   -> Límite permitido: " << WATCHDOG_MAX_TORQUE << " Nm\n";
                std::cerr << std::string(50, '!') << std::endl;
                return false;
            }

            // 2. Detección de Atasco (Stall)
            double error_posicion = std::abs(motors[m]->target_pos - motors[m]->current_pos);
            double velocidad_real = std::abs(motors[m]->current_vel);

            if ((error_posicion > WATCHDOG_MAX_ERROR_POS) && (velocidad_real < WATCHDOG_MIN_VEL_STALL))
            {
                std::cerr << "\n" << std::string(50, '-') << "\n";
                std::cerr << "🚨 [EMERGENCIA: ATASCO / STALL DETECTADO]\n";
                std::cerr << "Pata: " << p << " | Motor ID: " << motors[m]->id << "\n";
                std::cerr << "   -> Error de Posición: " << (error_posicion * 360.0) << " grados\n";
                std::cerr << "      (Límite de error:  " << (WATCHDOG_MAX_ERROR_POS * 360.0) << " grados)\n";
                std::cerr << "   -> Velocidad actual:  " << velocidad_real << " rev/s\n";
                std::cerr << "      (Mínimo esperado:  " << WATCHDOG_MIN_VEL_STALL << " rev/s)\n";
                std::cerr << "   -> TARGET: " << (motors[m]->target_pos * 360.0) << " deg | REAL: " << (motors[m]->current_pos * 360.0) << " deg\n";
                std::cerr << std::string(50, '-') << std::endl;
                return false;
            }

            // 3. Protección Térmica
            if (motors[m]->temperature > WATCHDOG_MAX_TEMP)
            {
                std::cerr << "\n" << std::string(50, '*') << "\n";
                std::cerr << "🚨 [EMERGENCIA: SOBRECALENTAMIENTO]\n";
                std::cerr << "Pata: " << p << " | Motor ID: " << motors[m]->id << "\n";
                std::cerr << "   -> Temperatura actual: " << static_cast<int>(motors[m]->temperature) << " °C\n";
                std::cerr << "   -> Límite permitido:   " << static_cast<int>(WATCHDOG_MAX_TEMP) << " °C\n";
                std::cerr << std::string(50, '*') << std::endl;
                return false;
            }
        }
    }
    return true;
}

void PlanificarMovimiento()
{
    if (estado_actual == RobotState::EMERGENCY)
        return;

    tiempo_estado += DT;

    // --- ESTADO 1: HOMING ---
    if (estado_actual == RobotState::HOMING)
    {
        double t = std::clamp(tiempo_estado / DURACION_HOMING, 0.0, 1.0);

        for (int p = 0; p < 4; ++p)
        {
            MotorData *motors[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
            for (int m = 0; m < 3; ++m)
            {
                int id = motors[m]->id;
                motors[m]->target_pos = start_pos[id] + t * (0.0 - start_pos[id]);
                motors[m]->target_vel = 0.0;
                motors[m]->kp = 1.0;
                motors[m]->kd = 1.0;
            }
        }

        if (t >= 1.0)
        {
            for (int p = 0; p < 4; ++p)
            {
                bool es_derecha = (p == 1 || p == 3);
                LegAngles angles = solve_IK(0.01, (es_derecha ? -0.093 : 0.093), -0.306, es_derecha);
                if (angles.valid)
                {
                    patas[p].goal_angles[0] = angles.th1 / 360.0;
                    patas[p].goal_angles[1] = angles.th2 / 360.0;
                    patas[p].goal_angles[2] = angles.th3 / 360.0;
                }
                else
                {
                    patas[p].goal_angles[0] = 0.0;
                    patas[p].goal_angles[1] = 0.0;
                    patas[p].goal_angles[2] = 0.0;
                }
                state_transicion.pos_inicial[patas[p].coxa.id] = patas[p].coxa.target_pos;
                state_transicion.pos_inicial[patas[p].femur.id] = patas[p].femur.target_pos;
                state_transicion.pos_inicial[patas[p].tibia.id] = patas[p].tibia.target_pos;
                state_transicion.angulos_objetivo[p][0] = patas[p].goal_angles[0];
                state_transicion.angulos_objetivo[p][1] = patas[p].goal_angles[1];
                state_transicion.angulos_objetivo[p][2] = patas[p].goal_angles[2];
            }
            estado_actual = RobotState::READY;
            tiempo_estado = 0.0;
            std::cout << "[SYSTEM] Iniciando transición suave a postura Stand..." << std::endl;
        }
    }
    // --- ESTADO 2: READY (Transición Minimum Jerk basada en dt) ---
    else if (estado_actual == RobotState::READY)
    {
        double t_proporcional = std::clamp(tiempo_estado / TIEMPO_TRANSICION, 0.0, 1.0);
        double poly = (t_proporcional * t_proporcional * t_proporcional) * (10.0 + t_proporcional * (-15.0 + 6.0 * t_proporcional));

        for (int p = 0; p < 4; ++p)
        {
            MotorData *m_ptr[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
            for (int m = 0; m < 3; ++m)
            {
                double p_start = state_transicion.pos_inicial[m_ptr[m]->id];
                double p_target = state_transicion.angulos_objetivo[p][m];
                m_ptr[m]->target_pos = p_start + (p_target - p_start) * poly;
                m_ptr[m]->target_vel = 0.0;
                m_ptr[m]->ff_torque = 0.0;
            }
        }

        if (tiempo_estado >= TIEMPO_TRANSICION)
        {
            estado_actual = RobotState::ACTIVE;
            tiempo_estado = 0.0;
            std::cout << "[SYSTEM] ATOM-51 en modo ACTIVE. ¡Listo para caminar!" << std::endl;
        }
    }
    // --- ESTADO 3: ACTIVE (DART + LegGaitPlanner) ---
    else if (estado_actual == RobotState::ACTIVE)
    {
        // Variables de estado interno para el Trot
        enum LegMode
        {
            kStance,
            kSwing
        };
        static LegMode current_mode[4] = {kStance, kStance, kStance, kStance};
        static double mode_timer[4] = {0.0, 0.0, 0.0, 0.0};
        static Eigen::Vector3d foot_pos_R[4];
        static std::shared_ptr<SwingTrajectory> swing_trajs[4] = {nullptr, nullptr, nullptr, nullptr};
        static bool gait_initialized = false;

        const double step_h = 0.08;
        const Eigen::Vector3d LEGS_STAND_XYZ[4] = {
            Eigen::Vector3d(0.01, 0.093, -0.306),  // 0: FL
            Eigen::Vector3d(0.01, -0.093, -0.306), // 1: FR
            Eigen::Vector3d(0.01, 0.093, -0.306),  // 2: BL
            Eigen::Vector3d(0.01, -0.093, -0.306)  // 3: BR
        };

        // Inputs direccionales (Control remoto irá aquí)
        double vx = 0.1, vy = 0.0, wz = 0.0;
        double speed = std::sqrt(vx * vx + vy * vy);
        TrotTimes times = CalcularTiemposMarcha(speed);

        if (!gait_initialized)
        {
            for (int i = 0; i < 4; i++)
            {
                foot_pos_R[i] = LEGS_STAND_XYZ[i];
                current_mode[i] = kStance;
            }
            gait_initialized = true;
        }

        // Mantener sincronización si el robot está quieto
        if (speed < 0.01)
        {
            for (int i = 0; i < 4; i++)
            {
                if (i == 0 || i == 3)
                    mode_timer[i] = times.twovleg_time;
                else
                    mode_timer[i] = -(times.swing_time + times.twovleg_time) / 2.0;
            }
        }

        for (int p = 0; p < 4; ++p)
        {
            if (PATA_ACTIVA_DEBUG != -1 && p != PATA_ACTIVA_DEBUG)
                continue;

            bool es_derecha = (p == 1 || p == 3);
            auto coxa_j = patas_dart_global[p]->getJoint("shoulder_joint");
            auto femur_j = patas_dart_global[p]->getJoint("femur_joint");
            auto tibia_j = patas_dart_global[p]->getJoint("tibia_joint");
            auto pie_node = patas_dart_global[p]->getBodyNode("foot");

            EstadoRealMotores real;
            real.angulos_rad = {patas[p].coxa.current_pos * 2.0 * M_PI, patas[p].femur.current_pos * 2.0 * M_PI, patas[p].tibia.current_pos * 2.0 * M_PI};
            real.velocidades_rad_s = {patas[p].coxa.current_vel * 2.0 * M_PI, patas[p].femur.current_vel * 2.0 * M_PI, patas[p].tibia.current_vel * 2.0 * M_PI};

            Eigen::Vector3d offset_cadera(0.0, LEGS_STAND_XYZ[p].y(), 0.0);
            Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0, 0, wz).cross(offset_cadera);

            // ==========================================
            // MÁQUINA DE ESTADOS DINÁMICA POR PATA
            // ==========================================
            if (speed >= 0.01)
                mode_timer[p] += DT;

            if (current_mode[p] == kStance)
            {
                // Solo inicia el paso si hay velocidad de input
                if (mode_timer[p] >= times.twovleg_time && speed > 0.01)
                {
                    current_mode[p] = kSwing;
                    mode_timer[p] = 0.0;

                    Eigen::Vector3d p_target = LEGS_STAND_XYZ[p] + v_total * (times.swing_time / 2.0);
                    p_target.z() = LEGS_STAND_XYZ[p].z(); // Garantiza aterrizaje a nivel del suelo

                    swing_trajs[p] = std::make_shared<SwingTrajectory>(
                        foot_pos_R[p], -v_total, p_target, step_h, times.swing_time);
                }
            }
            else if (current_mode[p] == kSwing)
            {
                if (mode_timer[p] >= times.swing_time)
                {
                    current_mode[p] = kStance;
                    mode_timer[p] = 0.0;
                    swing_trajs[p].reset();
                }
            }

            CartesianState ideal;

            // Asignación de curvas de estado exactas
            if (current_mode[p] == kStance)
            {
                Eigen::Vector3d v_R(vx, vy, 0.0);
                Eigen::Vector3d w_R(0.0, 0.0, wz);

                PropagateLeg propagator(v_R, w_R, DT);
                PropagateLeg::Result prop_res = propagator(foot_pos_R[p]);
                foot_pos_R[p] = prop_res.position;

                ideal.P_des = prop_res.position;
                ideal.V_des = prop_res.velocity;
                ideal.A_des = Eigen::Vector3d::Zero(); // Fijo al piso
                ideal.is_stance = true;
                ideal.kp_scale = 1.0;
                ideal.kd_scale = 1.0;
            }
            else
            {
                SwingTrajectory::Result swing_res = swing_trajs[p]->Advance(DT, -v_total);
                foot_pos_R[p] = swing_res.position;

                ideal.P_des = swing_res.position;
                ideal.V_des = swing_res.velocity_s;

                // ¡AQUÍ ESTÁ LA MAGIA PARA DART! Inyección de aceleración analítica
                ideal.A_des = swing_res.acceleration_s2;
                ideal.is_stance = false;

                if (swing_res.phase > 0.85)
                {
                    ideal.kp_scale = 1.0;
                    ideal.kd_scale = 1.0;
                }
                else
                {
                    ideal.kp_scale = 1.0;
                    ideal.kd_scale = 1.0;
                }
            }

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

            LegAngles angles = solve_IK(ideal.P_des.x(), ideal.P_des.y(), ideal.P_des.z(), es_derecha);
            if (angles.valid)
            {
                patas[p].coxa.target_pos = angles.th1 / 360.0;
                patas[p].femur.target_pos = angles.th2 / 360.0;
                patas[p].tibia.target_pos = angles.th3 / 360.0;
            }

            // DART Forward Dynamics para Compensación de Fuerza
            coxa_j->setPosition(0, real.angulos_rad(0));
            femur_j->setPosition(0, real.angulos_rad(1));
            tibia_j->setPosition(0, real.angulos_rad(2));
            pie_node->getSkeleton()->computeForwardKinematics();

            ComandosMotor m_cmd = calcular_comandos_motores(patas_dart_global[p], coxa_j, femur_j, tibia_j, pie_node, real, deseado_dummy);

            // --- CÁLCULO CRUDO ---
            double v_coxa = m_cmd.velocidades_rad_s(0) / (2.0 * M_PI);
            double v_femur = m_cmd.velocidades_rad_s(1) / (2.0 * M_PI);
            double v_tibia = m_cmd.velocidades_rad_s(2) / (2.0 * M_PI);

            double t_coxa = std::clamp(m_cmd.torques_Nm(0), -max_control, max_control);
            double t_femur = std::clamp(m_cmd.torques_Nm(1), -max_control, max_control);
            double t_tibia = std::clamp(m_cmd.torques_Nm(2), -max_control, max_control);

            // --- 1. SETPOINTS DE VELOCIDAD (Con Deadband de 0.05) ---
            patas[p].coxa.target_vel = (std::abs(v_coxa) < 0.05) ? 0.0 : v_coxa;
            patas[p].femur.target_vel = (std::abs(v_femur) < 0.05) ? 0.0 : v_femur;
            patas[p].tibia.target_vel = (std::abs(v_tibia) < 0.05) ? 0.0 : v_tibia;

            // --- 2. SETPOINTS DE TORQUE (Con Deadband de 0.05) ---
            // DESCOMENTAR PARA APLICAR TORQUES DART
            patas[p].coxa.ff_torque = (std::abs(t_coxa) < 0.05) ? 0.0 : t_coxa;
            patas[p].femur.ff_torque = (std::abs(t_femur) < 0.05) ? 0.0 : t_femur;
            patas[p].tibia.ff_torque = (std::abs(t_tibia) < 0.05) ? 0.0 : t_tibia;

            patas[p].coxa.kp = deseado_dummy.kp_scale;
            patas[p].coxa.kd = deseado_dummy.kd_scale;
            patas[p].femur.kp = deseado_dummy.kp_scale;
            patas[p].femur.kd = deseado_dummy.kd_scale;
            patas[p].tibia.kp = deseado_dummy.kp_scale;
            patas[p].tibia.kd = deseado_dummy.kd_scale;
        }
    }
}

void GenerarComandosCAN()
{
    tx_frames_global.clear();
    for (int p = 0; p < 4; ++p)
    {
        MotorData *motors[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
        for (int m = 0; m < 3; ++m)
        {
            int controller_idx = motors[m]->id - 1;

            if (estado_actual == RobotState::EMERGENCY)
            {
                tx_frames_global.push_back(controllers[controller_idx]->MakeStop());
                continue;
            }

            moteus::PositionMode::Command cmd;
            cmd.position = motors[m]->target_pos;
            cmd.velocity = motors[m]->target_vel;
            cmd.feedforward_torque = motors[m]->ff_torque;

            double kp_mem = memory_global.cmd->kp_scale[p][m];
            double kd_mem = memory_global.cmd->kd_scale[p][m];

            cmd.kp_scale = (kp_mem > 0.0) ? kp_mem : motors[m]->kp;
            cmd.kd_scale = (kd_mem > 0.0) ? kd_mem : motors[m]->kd;
            cmd.maximum_torque = MAX_TORQUE_DEFAULT;

            tx_frames_global.push_back(controllers[controller_idx]->MakePosition(cmd));
        }
    }
}
//=========================================================
// 7. MONITOR VISUAL (En hilo separado)
// =========================================================
void MonitorLoop()
{
    const char *nombres_patas[] = {"FL", "FR", "BL", "BR"};
    const char *nombres_joints[] = {"COXA", "FEMUR", "TIBIA"};

    while (g_running)
    {
        if (!g_monitor_activo)
            break;
        std::cout << "\033[2J\033[H";

        // --- 1. LECTURAS REALES DEL MOTOR (FEEDBACK) ---
        std::cout << "=========================================================\n";
        std::cout << "             [ TELEMETRÍA REAL DEL MOTOR ]               \n";
        std::cout << "=========================================================\n";
        printf("%-4s | %-6s | %-3s | %-9s | %-8s | %-5s\n", "LEG", "JOINT", "ID", "POS(deg)", "TRQ(Nm)", "TEMP");
        std::cout << "---------------------------------------------------------\n";

        for (int p = 0; p < 4; ++p)
        {
            MotorData *m[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
            for (int i = 0; i < 3; i++)
            {
                printf("%-4s | %-6s | %-3d | %9.2f | %8.2f | %-5d\n",
                       nombres_patas[p], nombres_joints[i], m[i]->id,
                       m[i]->current_pos * 360.0, m[i]->current_torque, m[i]->temperature);
            }
            if (p < 3)
                std::cout << "---------------------------------------------------------\n";
        }

        // --- 2. COMANDOS ENVIADOS AL MOTOR (SETPOINTS) ---
        std::cout << "\n===============================================================================\n";
        std::cout << "                        [ COMANDOS ENVIADOS (TARGET) ]                         \n";
        std::cout << "===============================================================================\n";
        printf("%-4s | %-6s | %-3s | %-10s | %-10s | %-10s | %-5s | %-5s\n", "LEG", "JOINT", "ID", "TGT_POS(d)", "TGT_VEL", "FF_TRQ(Nm)", "KP", "KD");
        std::cout << "-------------------------------------------------------------------------------\n";

        for (int p = 0; p < 4; ++p)
        {
            MotorData *m[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
            for (int i = 0; i < 3; i++)
            {
                printf("%-4s | %-6s | %-3d | %10.2f | %10.2f | %10.2f | %5.2f | %5.2f\n",
                       nombres_patas[p], nombres_joints[i], m[i]->id,
                       m[i]->target_pos * 360.0, m[i]->target_vel, m[i]->ff_torque, m[i]->kp, m[i]->kd);
            }
            if (p < 3)
                std::cout << "-------------------------------------------------------------------------------\n";
        }

        // --- 3. METAS CARTESIANAS (DEL PLANIFICADOR) ---
        std::cout << "\n===============================================================================\n";
        std::cout << "                 [ TRAYECTORIA CARTESIANA DESEADA (M) ]                        \n";
        std::cout << "===============================================================================\n";
        printf("%-4s | %-15s | %-15s | %-15s\n", "LEG", "X Deseado", "Y Deseado", "Z Deseado");
        std::cout << "-------------------------------------------------------------------------------\n";

        for (int p = 0; p < 4; ++p)
        {
            printf("%-4s | %15.4f | %15.4f | %15.4f\n", nombres_patas[p], debug_target_x[p], debug_target_y[p], debug_target_z[p]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Refrescar pantalla a 10 Hz
    }
}
// =========================================================
// 8. ENTRY POINT (MAIN)
// =========================================================
int main(int argc, char **argv)
{
    optimizar_recursos_pi4();
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    if (!memory_global.is_valid())
        return 1;

    // Inicializar DART Skeletons
    LegDimensions dims;
    for (int i = 0; i < 4; i++)
        patas_dart_global[i] = CreateLegSkeleton(dims);

    inicializar_robot(patas, transport, controllers);

    std::cout << "[SETUP] Sincronizando los 12 motores..." << std::endl;
    std::vector<moteus::CanFdFrame> tx_init, rx_init;
    for (auto &c : controllers)
        tx_init.push_back(c->MakeQuery());

    bool motores_listos = false;
    int intentos = 0;
    while (!motores_listos && intentos < 20)
    {
        intentos++;
        rx_init.clear();
        SafeTransportCycle(transport, tx_init, &rx_init);

        int count = 0;
        for (const auto &frame : rx_init)
        {
            const auto res = moteus::Query::Parse(frame.data, frame.size);
            int id = frame.source;
            if (id >= 1 && id <= 12)
            {
                start_pos[id] = res.position;
                count++;
            }
        }
        if (count == 12)
            motores_listos = true;
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!motores_listos)
    {
        std::cerr << "❌ [ERROR CRÍTICO] Faltan motores. Abortando programa." << std::endl;
        return 1;
    }

    std::cout << "✅ Sincronización exitosa. Estados internos emparejados con la realidad." << std::endl;

    // Cebar primera planificación
    PlanificarMovimiento();
    GenerarComandosCAN();

    // Hilo de Interfaz
    std::thread monitor_thread(MonitorLoop);

    // Motor Asíncrono
    boost::asio::io_context io_context;
    boost::asio::steady_timer timer(io_context);
    timer_global = &timer;

    boost::asio::signal_set signals(io_context, SIGINT, SIGTERM);
    signals.async_wait([&](auto, auto)
                       {
        std::cout << "\n[SISTEMA] Apagado detectado. Forzando parada segura..." << std::endl;
        g_running = false; 
        g_monitor_activo = false;
        timer.cancel();  
        io_context.stop(); });

    timer.expires_after(std::chrono::milliseconds(1));
    timer.async_wait([transport](const boost::system::error_code &e)
                     { HandleTimer(e, transport); });

    io_context.run();

    // Limpieza Segura al Salir
    if (monitor_thread.joinable())
        monitor_thread.join();

    std::cout << "\n[APAGADO] Mandando comandos Freewheel a los 12 ejes..." << std::endl;
    std::vector<moteus::CanFdFrame> tx_stop, rx_stop;
    for (auto &c : controllers)
        tx_stop.push_back(c->MakeStop());
    SafeTransportCycle(transport, tx_stop, &rx_stop);

    std::cout << "✅ ATOM-51 apagado correctamente." << std::endl;
    return 0;
}
