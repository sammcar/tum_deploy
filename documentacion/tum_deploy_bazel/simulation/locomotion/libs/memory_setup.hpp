#ifndef MEMORY_SETUP_HPP
#define MEMORY_SETUP_HPP

// Inclusión de los tipos de datos de memoria (ajusta si tus estructuras están en otro .hpp)
#include "robot_types.hpp"

/**
 * @brief Inicializa y mapea todas las memorias compartidas necesarias para el robot.
 * * @param require_real_hardware Si es true, imprime advertencia de modo real.
 * @param imu_ptr Puntero donde se mapeará la memoria de la IMU.
 * @param shm_ptr Puntero donde se mapeará la memoria de Comandos.
 * @param tel_ptr Puntero donde se mapeará la memoria de Telemetría.
 * @param contact_ptr Puntero donde se mapeará la memoria de Contactos.
 * @return true Si todas las memorias se mapearon con éxito.
 * @return false Si hubo algún error al acceder a la memoria.
 */
bool init_shared_memory(
    bool require_real_hardware,
    IMUData*& imu_ptr,
    CommandData*& shm_ptr,
    TelemetryData*& tel_ptr,
    ContactData*& contact_ptr
);

#endif // MEMORY_SETUP_HPP