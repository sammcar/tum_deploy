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
#include <sys/mman.h> // Para mlockall, MCL_CURRENT, MCL_FUTURE
#include <sched.h>    // Para sched_setscheduler, SCHED_FIFO
#include <pthread.h>

// --- INCLUSIONES PROPIAS ---
#include "robot_config.hpp"
#include "utils_motors.hpp"
#include "kinematics.hpp"
#include "control_dart.hpp"
#include "gait_planner.hpp"
#include "parameters_robot.hpp"

using namespace mjbots;

using Eigen::Vector3d;

// 2. GLOBALES DE ESTADO Y RED
// =========================================================
boost::asio::steady_timer *timer_global;
std::vector<moteus::CanFdFrame> tx_frames_global;
std::vector<moteus::CanFdFrame> rx_frames_global;
std::vector<std::shared_ptr<moteus::Controller>> controllers;

std::atomic<bool> g_running{true};
std::atomic<bool> g_monitor_activo{true};

Leg patas[4]; // 0:FL, 1:FR, 2:BL, 3:BR

Atom51GaitPlanner planner_global;
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

static double debug_target_x[4] = {0, 0, 0, 0};
static double debug_target_y[4] = {0, 0, 0, 0};
static double debug_target_z[4] = {0, 0, 0, 0};

// Declaraciones adelantadas
void HandleStatus(std::shared_ptr<moteus::Transport> transport);
void PlanificarMovimiento();
void GenerarComandosCAN(); // <--- ¡Faltaba esta!
void MonitorLoop();        // <--- ¡Y faltaba esta!

// =========================================================
// 3. CLASES Y ESTRUCTURAS DE CONTROL (Actualizado con PropagateLeg)
// =========================================================

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

    ProcesarTelemetria(rx_frames_global, patas);

    if (usar_watchdog_seguridad && estado_actual != RobotState::EMERGENCY)
    {
        if (!VerificarSeguridad(patas))
        {
            g_monitor_activo = false;
            estado_actual = RobotState::EMERGENCY;
        }
    }

    PlanificarMovimiento();
    GenerarComandosCAN();
}

// Estructura para guardar los punteros ya encontrados

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

        if (tiempo_estado >= 10.0)
        {
            estado_actual = RobotState::ACTIVE;
            tiempo_estado = 0.0;
            std::cout << "[SYSTEM] ATOM-51 en modo ACTIVE. ¡Listo para caminar!" << std::endl;
        }
    }
    // --- ESTADO 3: ACTIVE (DART + LegGaitPlanner) ---
    else if (estado_actual == RobotState::ACTIVE)
    {
        // --- PARÁMETROS DE MARCHA ---
        double vx = 0.05, vy = 0.0, wz = 0.0;
        double step_duration = 1.0, step_h = 0.15, duty_factor = 0.75;
        const double gait_offsets[4] = {0.0, 0.5, 0.75, 0.25};

        for (int p = 0; p < 4; ++p)
        {
            if (PATA_ACTIVA_DEBUG != -1 && p != PATA_ACTIVA_DEBUG)
                continue;
            // 1. Obtener Trayectoria Encapsulada
            TrajectoryOutput traj = planner_global.update(DT, tiempo_estado, p, vx, vy, wz, step_duration, step_h, duty_factor, gait_offsets);

            debug_target_x[p] = traj.position.x();
            debug_target_y[p] = traj.position.y();
            debug_target_z[p] = traj.position.z();

            // 2. Cinemática Inversa
            bool es_derecha = (p == 1 || p == 3);
            LegAngles angles = solve_IK(traj.position.x(), traj.position.y(), traj.position.z(), es_derecha);

            if (angles.valid)
            {
                patas[p].coxa.target_pos = angles.th1 / 360.0;
                patas[p].femur.target_pos = angles.th2 / 360.0;
                patas[p].tibia.target_pos = angles.th3 / 360.0;

                // 3. Preparar datos para DART (Dynamics)
                EstadoDeseadoCartesiano deseado;
                deseado.posicion = traj.position;
                deseado.velocidad = traj.velocity;
                deseado.aceleracion = traj.acceleration;
                deseado.kp_scale = (traj.is_stance) ? 1.0 : 1.0;
                deseado.kd_scale = (traj.is_stance) ? 1.0 : 1.0;

                EstadoRealMotores real;
                real.angulos_rad = {patas[p].coxa.current_pos * 2 * M_PI, patas[p].femur.current_pos * 2 * M_PI, patas[p].tibia.current_pos * 2 * M_PI};

                // NOTA IMPORTANTE: Faltaba pasarle las velocidades reales
                real.velocidades_rad_s = {patas[p].coxa.current_vel * 2 * M_PI, patas[p].femur.current_vel * 2 * M_PI, patas[p].tibia.current_vel * 2 * M_PI};

                // 4. EXTRAER LOS PUNTEROS DEL ESQUELETO ACTUAL
                auto esqueleto_actual = patas_dart_global[p];
                auto coxa_j = esqueleto_actual->getJoint("shoulder_joint");
                auto femur_j = esqueleto_actual->getJoint("femur_joint");
                auto tibia_j = esqueleto_actual->getJoint("tibia_joint");
                auto pie_node = esqueleto_actual->getBodyNode("foot");
                ComandosMotor m_cmd = calcular_comandos_motores(esqueleto_actual, coxa_j, femur_j, tibia_j, pie_node, real, deseado);
                // 5. Asignación final con Deadband de 0.05
                auto apply_deadband = [](double val)
                { return (std::abs(val) < 0.05) ? 0.0 : val; };

                patas[p].coxa.target_vel = apply_deadband(m_cmd.velocidades_rad_s(0) / (2.0 * M_PI));
                patas[p].coxa.ff_torque = apply_deadband(std::clamp(m_cmd.torques_Nm(0), -max_control, max_control));
                patas[p].coxa.kp = deseado.kp_scale;
                patas[p].coxa.kd = deseado.kd_scale;

                // Repetir para fémur y tibia (simplificado aquí por brevedad)
                patas[p].femur.target_vel = apply_deadband(m_cmd.velocidades_rad_s(1) / (2.0 * M_PI));
                patas[p].femur.ff_torque = apply_deadband(std::clamp(m_cmd.torques_Nm(1), -max_control, max_control));
                patas[p].femur.kp = deseado.kp_scale;
                patas[p].femur.kd = deseado.kd_scale;

                patas[p].tibia.target_vel = apply_deadband(m_cmd.velocidades_rad_s(2) / (2.0 * M_PI));
                patas[p].tibia.ff_torque = apply_deadband(std::clamp(m_cmd.torques_Nm(2), -max_control, max_control));
                patas[p].tibia.kp = deseado.kp_scale;
                patas[p].tibia.kd = deseado.kd_scale;
            }
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
            cmd.kp_scale = motors[m]->kp;
            cmd.kd_scale = motors[m]->kd;
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

    LegDimensions dims;
    for (int i = 0; i < 4; i++)
    {
        bool es_derecha = (i == 1 || i == 3); // 1 (FR) y 3 (BR) son derechas
        patas_dart_global[i] = CreateLegSkeleton(dims, es_derecha);
    }

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
