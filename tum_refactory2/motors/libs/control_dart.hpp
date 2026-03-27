#ifndef CONTROL_DART_HPP
#define CONTROL_DART_HPP

#include <dart/dart.hpp>
#include <Eigen/Dense>

using namespace dart::dynamics;
using Eigen::Vector3d;

// --- 1. CONSTANTES GLOBALES (Declaradas como extern) ---
// El compilador sabrá que estas existen, pero su valor real está en el .cpp
extern Vector3d kd_base; 
extern Vector3d kp_base; 
extern double masa_pierna_def;

// --- 2. ESTRUCTURAS ---
struct EstadoRealMotores {
    Vector3d angulos_rad;
    Vector3d velocidades_rad_s;
};

struct EstadoDeseadoCartesiano {
    Vector3d posicion;
    Vector3d velocidad;
    Vector3d aceleracion;
    double stance_actual = 0.0;
    double kp_scale = 1.0;
    double kd_scale = 1.0;
};

struct ComandosMotor {
    Vector3d velocidades_rad_s;
    Vector3d torques_Nm;
};

struct LegDimensions {
    Vector3d shoulder_offset = {0.0, 0.093, 0.0}; 
    Vector3d femur_len = {0.0, 0.0, -0.147};      
    Vector3d tibia_len = {0.0, 0.0, -0.230};      
};

struct EstadoCartesianoReal {
    Vector3d posicion;
    Vector3d velocidad;
};

// --- 3. DECLARACIONES DE FUNCIONES ---
// (Ya no llevan "inline", ni llevan las llaves con el código dentro)

SkeletonPtr CreateLegSkeleton(const LegDimensions& dims, bool is_right);

EstadoCartesianoReal calcular_FK_estado(Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node, 
                                        const Vector3d& angulos_rad, const Vector3d& velocidades_rad_s);

Vector3d calcular_F_P(const Vector3d& pos_deseada, const Vector3d& pos_real, const Vector3d& Kp);
Vector3d calcular_F_D(const Vector3d& vel_deseada, const Vector3d& vel_real, const Vector3d& Kd);
Vector3d calcular_F_accel(const Vector3d& accel_deseada, double mp, double mc, double fa, double es);

Eigen::Matrix3d calcular_jacobiano_lineal(BodyNode* pie_node);
Eigen::Matrix3d calcular_jacobiano_fuerza(SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node);

// Función Maestra
ComandosMotor calcular_comandos_motores(SkeletonPtr pata, Joint* coxa, Joint* femur, Joint* tibia, BodyNode* pie_node,
                                        const EstadoRealMotores& real, const EstadoDeseadoCartesiano& deseado);

#endif // CONTROL_DART_HPP
