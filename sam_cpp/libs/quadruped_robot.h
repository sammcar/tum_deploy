#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include "utils_sam.h"
#include "gait_generator.h"
#include "kinematics_sam.h"
#include <vector>
#include <string>
#include <map>

// Estructura limpia para reemplazar el diccionario 'cmd' de Python
struct RobotCommand {
    // Velocidad lineal y angular
    double vx = 0;
    double vy = 0;
    double wz = 0;

    // Postura del cuerpo (Offsets)
    double body_x = 0;
    double body_y = 0;
    double body_z = 0; // Altura deseada absoluta o relativa

    // Orientación del cuerpo
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    // Punto de pivote para la rotación (opcional)
    double piv_x = 0;
    double piv_y = 0;

    // Modos de patas individuales (true = fija/stance, false = normal)
    std::map<std::string, bool> leg_modes;
};

class QuadrupedRobot {
public:
    // Constructor
    // body_length: Largo del robot (X)
    // body_width:  Ancho del robot (Y)
    // l1, l2, l3:  Longitudes de los eslabones
    QuadrupedRobot(double body_length, double body_width, double l1, double l2, double l3);

    // Método principal paso a paso
    // t: Tiempo actual
    // dt: Delta tiempo
    // cmd: Estructura con comandos de control
    // sensor_data: Datos de IMU o encoders (placeholder)
    // modo: String de modo ("Walk", "Trote", etc.)
    // Retorna: Mapa con los ángulos [q1, q2, q3] para cada pata
    std::map<std::string, std::vector<double>> step(double t, double dt, RobotCommand cmd, Vec3 sensor_data, std::string modo = "Trote");

    // Calcula la postura estática por defecto (para inicialización)
    std::map<std::string, Vec3> get_default_stance();

private:
    // --- Dimensiones Físicas ---
    double lx, ly;
    double l1, l2, l3;
    double stand_height;

    // --- Componentes ---
    std::map<std::string, Vec3> hip_offsets;
    std::vector<std::string> leg_names;
    std::map<std::string, bool> leg_types; // false = dinámica, true = estática
    
    // Cerebro de Caminado (Instancia interna)
    GaitGenerator brain;

    // --- Estado Interno ---
    std::map<std::string, std::vector<double>> prev_angles; // Para unwrap
    Vec3 last_comp; // Compensación IMU anterior

    // --- Métodos Privados ---
    
    // Calcula la cinemática inversa para todas las patas dada una postura corporal
    std::map<std::string, std::vector<double>> compute_pose(Vec3 body_pos, Vec3 body_rpy, Vec3 pivote, std::map<std::string, Vec3> targets);
    
    // Evita saltos bruscos de ángulo (mantiene la continuidad > 2PI)
    std::vector<double> unwrap_angles(std::string leg_name, std::vector<double> new_angles);
};

#endif