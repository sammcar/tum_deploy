#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include "utils_sam.h"
#include "bezier_sam.h"  // Incluimos Bezier para usar sus funciones matemáticas
#include <vector>
#include <string>
#include <map>

// Definición de los estados de la máquina de estados
enum class GaitState {
    STAND,
    STARTUP_FL,
    STARTUP_BR,
    WALK,
    STOPPING
};

// Estructura para devolver los resultados del generador
struct GaitOutput {
    std::map<std::string, Vec3> feet_pos; // Posición deseada de cada pie
    Vec3 body_shift;                      // Desplazamiento del cuerpo (Sway + Z)
    Vec3 body_rot;                        // Rotación del cuerpo (si aplica)
};

class GaitGenerator {
public:
    // Constructor
    // stand_height: Altura base del robot
    // default_stance: Diccionario con la posición (x,y) de cada pata en reposo
    GaitGenerator(double stand_height, std::map<std::string, Vec3> default_stance);

    // Método principal de actualización (se llama en cada ciclo)
    // t_global: Tiempo actual en segundos
    // vel_cmd:  Vector de velocidad deseada (vx, vy, wz) - usaremos x,y para lineal
    // imu_rpy:  Datos del giroscopio (Roll, Pitch, Yaw)
    // modo:     String de control ("Walk", "Stand", etc.)
    GaitOutput update(double t_global, Vec3 vel_cmd, Vec3 imu_rpy, std::string modo = "Walk");

private:
    // --- Parámetros de Ajuste (Tuning) ---
    double stand_height;
    double walking_height;
    double spider_offset;
    double step_height;
    double step_length;
    double body_x_bias;
    
    double sway_y;
    double sway_x;
    double center_y_bias;

    // --- Variables de Estado ---
    double activity_level;
    double yaw_trim;
    GaitState state;
    int turn_index;        // Índice en la secuencia [0..3]
    double local_progress; // Progreso del ciclo [0.0 .. 3.0]
    double last_t_global;
    bool stop_requested;
    bool first_run;

    // --- Secuencia y Patas ---
    std::vector<std::string> sequence; // ["FL", "BR", "FR", "BL"]
    std::string active_leg;            // Pata que se está moviendo actualmente

    // --- Poses y Control de Movimiento ---
    std::map<std::string, Vec3> pose_rest;   // Posición de reposo (Z=0)
    std::map<std::string, Vec3> feet_state;  // Estado actual de los pies
    
    Vec3 swing_start;       // Inicio de la trayectoria de vuelo
    Vec3 swing_end;         // Fin de la trayectoria de vuelo
    bool lift_initialized;  // Flag: ¿Ya empezó a levantar la pata?
    bool touchdown_done;    // Flag: ¿Ya tocó el suelo?
    double swing_dx_accum;  // Compensación acumulada del treadmill

    // --- Tiempos ---
    double turn_T_min;
    double turn_T_max;
    double turn_T_stop;
    double cmd_dead;        // Zona muerta del joystick

    // --- Métodos Privados ---
    void init_rest_pose(const std::map<std::string, Vec3>& raw_stance);
    
    // Aplica el efecto de "cinta de correr" (mover las patas hacia atrás para avanzar)
    void apply_treadmill(double delta_local, bool include_active, double treadmill_per_turn, double dir_sign, bool treadmill_on);
};

#endif