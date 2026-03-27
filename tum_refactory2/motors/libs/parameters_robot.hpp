#pragma once
#include <cstdint>

// =========================================================
// 1. CONSTANTES GENERALIZADAS (El panel de control)
// =========================================================
extern const int CONTROL_HZ;
extern const double DT;
extern const int PERIOD_US;

extern const double DURACION_HOMING;
extern const double TIEMPO_TRANSICION;

extern const double MAX_TORQUE_DEFAULT;
extern const double max_control;

// --- LÍMITES DEL WATCHDOG DE SEGURIDAD ---
extern bool usar_watchdog_seguridad;
extern const double WATCHDOG_MAX_TORQUE;
extern const double WATCHDOG_MAX_ERROR_POS;
extern const double WATCHDOG_MIN_VEL_STALL;
extern const int8_t WATCHDOG_MAX_TEMP;

// --- DEBUG ---
extern const int PATA_ACTIVA_DEBUG;

