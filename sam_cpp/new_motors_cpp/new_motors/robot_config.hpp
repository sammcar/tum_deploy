#pragma once
#include <vector>
#include <cmath>

// Dimensiones físicas
const double L1 = 0.093; 
const double L2 = 0.147; 
const double L3 = 0.230;

// Masas
const double CHASIS_MASS = 5.65;
const double LEG_MASS = 1.46; 
const double TOTAL_ROBOT_MASS = CHASIS_MASS + LEG_MASS * 4;

// Calibración
const double kFactorMultiplicador = 1.00;
const double kFactorMultiplicador2 = 1.00;

// Configuración de Motores
struct MotorConfig {
    double kp; double kd; double max_torque;
    double vel_limit; double accel_limit;
};

const MotorConfig kCoxaConfig  = {1.0, 1.0, 2.0, 20.0, 50.0};
const MotorConfig kFemurConfig = {1.0, 1.0, 2.0, 20.0, 50.0};
const MotorConfig kTibiaConfig = {1.0, 1.0, 2.0, 20.0, 50.0};

const std::vector<int> kMotorIds = {1, 2, 3}; //, 4, 5... agregar resto

// Helper para obtener config según ID
inline const MotorConfig* GetMotorConfig(int id) {
    int type = (id - 1) % 3; 
    if (type == 0) return &kCoxaConfig;
    if (type == 1) return &kFemurConfig;
    return &kTibiaConfig;
}