#ifndef ROBOT_TYPES_HPP
#define ROBOT_TYPES_HPP

#include <cstdint> // Necesario para uint64_t
#include <Eigen/Dense> // <--- NUEVO: Necesario para TrajectoryPoint3D

// ============================================================
// 1. ESTRUCTURA DE MEMORIA COMPARTIDA
// ============================================================
//#pragma pack(push, 1)
struct CommandData {
    double angles[4][3];        // [Pata][Motor]
    double velocities[4][3];    // Se usa como LÍMITE DE VELOCIDAD en modo estático
    double desired_accel[4][3]; // [X, Y, Z] Feedforward
    double kp_scale[4][3];      
    double kd_scale[4][3];
    double transition_time;     // NUEVO: Tiempo para interpolación en main_sebas
    bool is_stance[4];          
    bool is_walking;            // false = modo estático interpolado
};
//#pragma pack(pop)

struct TelemetryData
{
    double measured_angles[4][3];
    double measured_velocities[4][3];
    double measured_torques[4][3];
    double temperature[4][3];
    uint64_t timestamp_us;
    bool fault_code[4]; 
};

// NUEVO: Agregamos zmp_d_x y zmp_d_y al Log
struct LogData {
    int leg_id;
    double phase;       
    bool is_stance;     
    double torque_femur; 
    double torque_tibia; 
    double zmp_x;        
    double zmp_y;        
    double zmp_d_x;      
    double zmp_d_y;      
    double v_com_x;      // NUEVO: Velocidad CoM X calculada
    double v_com_y;      // NUEVO: Velocidad CoM Y calculada
};

struct IMUData {
    double acc[3];
    double gyro[3]; // Velocidad angular en grados/segundo
    double roll;
    double pitch;
    double yaw;
    double temp;
    uint64_t timestamp;
};

struct LegAngles { double th1, th2, th3; bool valid; };

struct ContactData {
    double fz_r[4];       
    bool is_contact[4];   
    double v_xyz[4][3];   
    double umbral_r[4];   
};

struct TrajectoryPoint {
    double x, z;          // Posición
    double vx, vz;        // Velocidad
    double ax, az;        // Aceleración
};

struct TrajectoryPoint3D {
    Eigen::Vector3d pos; 
    Eigen::Vector3d acc; 
};

#endif // ROBOT_TYPES_HPP
