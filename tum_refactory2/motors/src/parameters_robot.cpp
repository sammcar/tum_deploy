#include "parameters_robot.hpp"

// =========================================================
// 1. CONSTANTES GENERALIZADAS (El panel de control)
// =========================================================
const int CONTROL_HZ = 400;
const double DT = 1.0 / CONTROL_HZ;
const int PERIOD_US = 1000000 / CONTROL_HZ;

const double DURACION_HOMING = 3.0;   // Segundos
const double TIEMPO_TRANSICION = 2.0; // Segundos para estado READY

const double MAX_TORQUE_DEFAULT = 8.0; // Nm permitidos en operación
const double max_control = 4.0;        // Nm permitidos en operación

// --- LÍMITES DEL WATCHDOG DE SEGURIDAD ---
bool usar_watchdog_seguridad = false;
const double WATCHDOG_MAX_TORQUE = 7.0;                     // Límite absoluto de emergencia
const double WATCHDOG_MAX_ERROR_POS = 40.0 * (1.0 / 360.0); // 30 grados
const double WATCHDOG_MIN_VEL_STALL = 0.03;
const int8_t WATCHDOG_MAX_TEMP = 65;

// --- DEBUG ---
const int PATA_ACTIVA_DEBUG = -1; // -1 para todas
