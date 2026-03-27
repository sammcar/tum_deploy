#include "utils_motors.hpp"

// Variables importadas del main.cpp (asegúrate de que estén definidas allí)
extern std::atomic<bool> g_running;
extern bool usar_watchdog_seguridad;
extern const double WATCHDOG_MAX_TORQUE;
extern const double WATCHDOG_MAX_ERROR_POS;
extern const double WATCHDOG_MIN_VEL_STALL;
extern const int8_t WATCHDOG_MAX_TEMP;
extern const int PATA_ACTIVA_DEBUG;

void SignalHandler(int) { g_running = false; }

void inicializar_robot(Leg patas[4], 
                       std::shared_ptr<mjbots::moteus::Transport> transport,
                       std::vector<std::shared_ptr<mjbots::moteus::Controller>>& controllers) 
{
    for (int p = 0; p < 4; ++p) {
        MotorData* motores_pata[3] = { &patas[p].coxa, &patas[p].femur, &patas[p].tibia };

        for (int m = 0; m < 3; ++m) {
            int id = (p * 3) + m + 1;
            motores_pata[m]->id = id;

            // Configuración de software por defecto
            motores_pata[m]->kp = 1.0;
            motores_pata[m]->kd = 1.0;
            motores_pata[m]->accel_lim = 20.0;
            motores_pata[m]->max_trq = 3.0;
            motores_pata[m]->target_pos = 0.0;
            motores_pata[m]->target_vel = 0.0;
            motores_pata[m]->ff_torque = 0.0;

            // Configuración de Hardware
            mjbots::moteus::Controller::Options options;
            options.id = id;
            options.transport = transport;

            options.position_format.position = mjbots::moteus::kFloat;
            options.position_format.velocity = mjbots::moteus::kFloat;
            options.position_format.feedforward_torque = mjbots::moteus::kFloat;
            options.position_format.kp_scale = mjbots::moteus::kFloat;
            options.position_format.kd_scale = mjbots::moteus::kFloat;
            options.position_format.maximum_torque = mjbots::moteus::kFloat;
            options.position_format.accel_limit = mjbots::moteus::kFloat;

            options.query_format.position = mjbots::moteus::kFloat;
            options.query_format.velocity = mjbots::moteus::kFloat;
            options.query_format.torque = mjbots::moteus::kFloat;
            options.query_format.temperature = mjbots::moteus::kInt8;

            auto controller = std::make_shared<mjbots::moteus::Controller>(options);
            controllers.push_back(controller);
            
            controller->SetStop();
        }
    }
    std::cout << "✅ ATOM-51 Inicializado: 12 motores vinculados a la estructura de control." << std::endl;
}

void optimizar_recursos_pi4()
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    struct sched_param param;
    param.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &param);
    mlockall(MCL_CURRENT | MCL_FUTURE);
}

double CalculateMinimumJerk(double t_elapsed, double t_total, double p_start, double p_end) {
    if (t_total < 0.001 || t_elapsed >= t_total) return p_end;
    if (t_elapsed <= 0) return p_start;
    double u = t_elapsed / t_total;
    double poly = (u * u * u) * (10.0 + u * (-15.0 + 6.0 * u));
    return p_start + (p_end - p_start) * poly;
}

void SafeTransportCycle(std::shared_ptr<mjbots::moteus::Transport> transport, 
                        const std::vector<mjbots::moteus::CanFdFrame>& send_frames, 
                        std::vector<mjbots::moteus::CanFdFrame>* receive_frames) {
    std::promise<void> cycle_done;
    transport->Cycle(send_frames.data(), send_frames.size(), receive_frames, [&cycle_done](int) { cycle_done.set_value(); });
    cycle_done.get_future().wait(); 
}

void ProcesarTelemetria(const std::vector<mjbots::moteus::CanFdFrame>& rx_frames_global, Leg patas[4])
{
    for (const auto &frame : rx_frames_global)
    {
        const auto res = mjbots::moteus::Query::Parse(frame.data, frame.size);
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
    }
}

bool VerificarSeguridad(Leg patas[4])
{
    for (int p = 0; p < 4; ++p)
    {
        // ✅ CORRECCIÓN CLAVE: Ignorar patas que no están bajo prueba
        if (PATA_ACTIVA_DEBUG != -1 && p != PATA_ACTIVA_DEBUG)
            continue;

        MotorData *motors[3] = {&patas[p].coxa, &patas[p].femur, &patas[p].tibia};
        for (int m = 0; m < 3; ++m)
        {
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
