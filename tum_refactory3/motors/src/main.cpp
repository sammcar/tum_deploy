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
constexpr int CONTROL_HZ = 400; // 200Hz ideal para procesar DART en 4 patas
constexpr double DT = 1.0 / CONTROL_HZ;
constexpr int PERIOD_US = 1000000 / CONTROL_HZ;

const double DURACION_HOMING = 3.0;    // Segundos
const double TIEMPO_TRANSICION = 2.0;  // Segundos para estado READY
const double MAX_TORQUE_DEFAULT = 3.5; // Nm permitidos en operación

// --- LÍMITES DEL WATCHDOG DE SEGURIDAD ---
bool usar_watchdog_seguridad = true;
const double WATCHDOG_MAX_TORQUE = 4.0;                     // Límite absoluto de emergencia
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

// --- CLASE DE PROPAGACIÓN DE ESTADO (Cuerpo Rígido) ---
class PropagateLeg
{
public:
    PropagateLeg(const Eigen::Vector3d &v_R,
                 const Eigen::Vector3d &w_R,
                 double period_s)
        : v_R_(v_R),
          w_R_(w_R),
          pose_T2_T1_(
              Sophus::SO3d(
                  Eigen::AngleAxisd(-period_s * w_R.z(), Eigen::Vector3d::UnitZ())
                      .toRotationMatrix()),
              -v_R_ * period_s) {}

    struct Result
    {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
    };

    Result operator()(const Eigen::Vector3d &position_R) const
    {
        Result result;
        // El pie permanece estático en el mundo; el chasis se mueve sobre él
        result.position = pose_T2_T1_ * position_R;
        result.velocity = -v_R_ - w_R_.cross(position_R);
        return result;
    }

private:
    Eigen::Vector3d v_R_;
    Eigen::Vector3d w_R_;
    Sophus::SE3d pose_T2_T1_;
};

class LegGaitPlanner
{
private:
    int leg_id_;
    Eigen::Vector3d origin_;
    Eigen::Vector3d hip_offset_;
    double gait_offset_;
    double duty_factor_;
    double step_duration_;
    double step_h_;

    // --- NUEVAS VARIABLES DE PERSISTENCIA ---
    Eigen::Vector3d foot_pos_R_;
    bool was_stance_ = false;

    double world_blend_ = 0.15;
    double damp_start_phase_ = 0.85;
    double damp_scale_kp_ = 0.6;
    double damp_scale_kd_ = 0.4;

    void EvaluateBezierCubic(double t, const Eigen::Vector3d &P0, const Eigen::Vector3d &P1,
                             const Eigen::Vector3d &P2, const Eigen::Vector3d &P3,
                             Eigen::Vector3d &P_out, Eigen::Vector3d &V_out, Eigen::Vector3d &A_out)
    {
        double u = 1.0 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;
        P_out = uuu * P0 + 3 * uu * t * P1 + 3 * u * tt * P2 + ttt * P3;
        V_out = 3 * uu * (P1 - P0) + 6 * u * t * (P2 - P1) + 3 * tt * (P3 - P2);
        A_out = 6 * u * (P2 - 2 * P1 + P0) + 6 * t * (P3 - 2 * P2 + P1);
    }

public:
    LegGaitPlanner() {}
    void Initialize(int leg_id, const Eigen::Vector3d &origin, const Eigen::Vector3d &hip_offset,
                    double gait_offset, double duty_factor, double step_duration, double step_h)
    {
        leg_id_ = leg_id;
        origin_ = origin;
        hip_offset_ = hip_offset;
        gait_offset_ = gait_offset;
        duty_factor_ = duty_factor;
        step_duration_ = step_duration;
        step_h_ = step_h;
    }

    CartesianState Update(double global_phase, double vx, double vy, double wz, double dt = 0.001)
    {
        CartesianState state;
        double leg_phase = fmod(global_phase + gait_offset_, 1.0);
        double stance_portion = duty_factor_;
        double swing_portion = 1.0 - duty_factor_;
        state.is_stance = (leg_phase >= swing_portion);

        double t_stance = step_duration_ * stance_portion;
        Eigen::Vector3d v_total = Eigen::Vector3d(vx, vy, 0.0) + Eigen::Vector3d(0, 0, wz).cross(hip_offset_);
        Eigen::Vector3d p_start = origin_ - v_total * (t_stance / 2.0);
        Eigen::Vector3d p_target = origin_ + v_total * (t_stance / 2.0);

        if (state.is_stance)
        {
            if (!was_stance_)
            {
                // Al iniciar el apoyo, establecemos la posición inicial en el Frame del Robot (R)
                foot_pos_R_ = p_start + hip_offset_;
            }

            Eigen::Vector3d v_R(vx, vy, 0.0);
            Eigen::Vector3d w_R(0.0, 0.0, wz);

            // Propagación incremental del estado
            PropagateLeg propagator(v_R, w_R, dt);
            PropagateLeg::Result res = propagator(foot_pos_R_);

            // Persistencia de la posición para el siguiente ciclo
            foot_pos_R_ = res.position;

            // Retorno al frame local (cadera) para mantener compatibilidad con IK
            state.P_des = foot_pos_R_ - hip_offset_;
            state.P_des.z() = origin_.z(); // Mantiene el pie anclado al suelo en Z

            // Velocidad dinámica compensando rotaciones y traslaciones
            state.V_des = res.velocity;
            state.A_des.setZero();

            state.kp_scale = 1.0;
            state.kd_scale = 1.0;
        }
        else
        {
            double phi = leg_phase / swing_portion;
            double flight_duration = step_duration_ * swing_portion;
            Eigen::Vector3d P, V, A;
            P.setZero();
            V.setZero();
            A.setZero();

            double z_max = p_start.z() + step_h_;
            if (phi < 0.5)
            {
                double phi_z = phi * 2.0;
                P.z() = p_start.z() + (z_max - p_start.z()) * std::sin(phi_z * M_PI / 2.0);
                V.z() = (z_max - p_start.z()) * std::cos(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            }
            else
            {
                double phi_z = (phi - 0.5) * 2.0;
                P.z() = z_max - (z_max - p_target.z()) * (1.0 - std::cos(phi_z * M_PI / 2.0));
                V.z() = -(z_max - p_target.z()) * std::sin(phi_z * M_PI / 2.0) * (M_PI / flight_duration);
            }

            Eigen::Vector3d xy_start(p_start.x(), p_start.y(), 0.0);
            Eigen::Vector3d xy_target(p_target.x(), p_target.y(), 0.0);
            Eigen::Vector3d V_world_xy(-v_total.x(), -v_total.y(), 0.0);

            if (phi < world_blend_)
            {
                double t_local = phi / world_blend_;
                V.head<2>() = V_world_xy.head<2>() * (1.0 - t_local);
                P.head<2>() = xy_start.head<2>() + (V_world_xy.head<2>() * (phi * flight_duration));
            }
            else if (phi < 1.0 - world_blend_)
            {
                double t_local = (phi - world_blend_) / (1.0 - 2.0 * world_blend_);
                Eigen::Vector3d P1 = xy_start + (xy_target - xy_start) * 0.2;
                Eigen::Vector3d P2 = xy_start + (xy_target - xy_start) * 0.8;
                Eigen::Vector3d P_bez, V_bez, A_bez;
                EvaluateBezierCubic(t_local, xy_start, P1, P2, xy_target, P_bez, V_bez, A_bez);

                double altura_z_guardada = P.z();
                double velocidad_z_guardada = V.z();
                P = P_bez;
                V = V_bez / (flight_duration * (1.0 - 2.0 * world_blend_));
                A = A_bez;
                P.z() = altura_z_guardada;
                V.z() = velocidad_z_guardada;
            }
            else
            {
                double t_local = (phi - (1.0 - world_blend_)) / world_blend_;
                P.head<2>() = xy_target.head<2>();
                V.head<2>() = V_world_xy.head<2>() * t_local;
            }

            state.P_des = P;
            state.V_des = V;
            state.A_des = A;
            if (phi > damp_start_phase_)
            {
                state.kp_scale = damp_scale_kp_;
                state.kd_scale = damp_scale_kd_;
            }
            else
            {
                state.kp_scale = 1.0;
                state.kd_scale = 1.0;
            }
        }

        was_stance_ = state.is_stance; // Guardar el estado para el próximo ciclo
        return state;
    }
};

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
            if (std::abs(motors[m]->current_torque) > WATCHDOG_MAX_TORQUE)
            {
                std::cout << "\n🚨 [EMERGENCIA] Torque excesivo en Pata " << p << " Motor " << motors[m]->id << std::endl;
                return false;
            }

            double error_posicion = std::abs(motors[m]->target_pos - motors[m]->current_pos);
            double velocidad_real = std::abs(motors[m]->current_vel);

            if ((error_posicion > WATCHDOG_MAX_ERROR_POS) && (velocidad_real < WATCHDOG_MIN_VEL_STALL))
            {
                std::cout << "\n🚨 [EMERGENCIA] Atasco detectado en Pata " << p << " Motor " << motors[m]->id << std::endl;
                return false;
            }
            if (motors[m]->temperature > WATCHDOG_MAX_TEMP)
            {
                std::cout << "\n🚨 [EMERGENCIA] Temperatura excesiva." << std::endl;
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
        static LegGaitPlanner planners[4];
        static bool gait_initialized = false;

        const double step_duration = 1.0;
        const double step_h = 0.05;
        const double duty_factor = 0.5;
        const double gait_offsets[4] = {0.0, 0.5, 0.5, 0.0};

        const Eigen::Vector3d LEGS_STAND_XYZ[4] = {
            Eigen::Vector3d(0.01, 0.093, -0.306),  // 0: FL
            Eigen::Vector3d(0.01, -0.093, -0.306), // 1: FR
            Eigen::Vector3d(0.01, 0.093, -0.306),  // 2: BL
            Eigen::Vector3d(0.01, -0.093, -0.306)  // 3: BR
        };

        if (!gait_initialized)
        {
            for (int i = 0; i < 4; i++)
            {
                Eigen::Vector3d offset_cadera(0.0, LEGS_STAND_XYZ[i].y(), 0.0);
                planners[i].Initialize(i, LEGS_STAND_XYZ[i], offset_cadera, gait_offsets[i], duty_factor, step_duration, step_h);
            }
            gait_initialized = true;
        }

        double global_phase = fmod(tiempo_estado / step_duration, 1.0);
        double vx = 0.0, vy = 0.0, wz = 0.0; // Velocidad del control remoto iría aquí

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

            // --> AQUÍ ESTÁ EL PLANIFICADOR ACTUALIZADO USANDO EL NUEVO MÉTODO UPDATE QUE RECIBE 'DT' <--
            CartesianState ideal = planners[p].Update(global_phase, vx, vy, wz, DT);

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

            // Cinemática Inversa (Posición objetivo)
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

            // 1. Setpoints de movimiento
            patas[p].coxa.target_vel = m_cmd.velocidades_rad_s(0) / (2.0 * M_PI);
            patas[p].femur.target_vel = m_cmd.velocidades_rad_s(1) / (2.0 * M_PI);
            patas[p].tibia.target_vel = m_cmd.velocidades_rad_s(2) / (2.0 * M_PI);

            //DESCOMENTAR PARA APLICAR TORQUES DART (Asegurarse de ajustar masas primero)
            patas[p].coxa.ff_torque  = std::clamp(m_cmd.torques_Nm(0), -MAX_TORQUE_DEFAULT, MAX_TORQUE_DEFAULT);
            patas[p].femur.ff_torque = std::clamp(m_cmd.torques_Nm(1), -MAX_TORQUE_DEFAULT, MAX_TORQUE_DEFAULT);
            patas[p].tibia.ff_torque = std::clamp(m_cmd.torques_Nm(2), -MAX_TORQUE_DEFAULT, MAX_TORQUE_DEFAULT);

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

// =========================================================
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
        std::cout << "=========================================================\n";
        std::cout << "             [ MONITOR DE TELEMETRÍA ATOM-51 ]           \n";
        std::cout << "=========================================================\n";
        printf("%-4s | %-6s | %-3s | %-9s | %-8s | %-5s\n", "LEG", "JOINT", "ID", "POS(deg)", "TRQ(Nm)", "TEMP");
        for (int p = 0; p < 4; ++p)
        {
            MotorData *m[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
            for (int i = 0; i < 3; i++)
            {
                printf("%-4s | %-6s | %-3d | %9.2f | %8.2f | %-5d\n", nombres_patas[p], nombres_joints[i], m[i]->id, m[i]->current_pos * 360.0, m[i]->current_torque, m[i]->temperature);
            }
        }
        std::cout << "=========================================================\n";
        for (int p = 0; p < 4; ++p)
        {
            printf("%-4s | Target_X: %6.3f | Target_Y: %6.3f | Target_Z: %6.3f\n", nombres_patas[p], debug_target_x[p], debug_target_y[p], debug_target_z[p]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
