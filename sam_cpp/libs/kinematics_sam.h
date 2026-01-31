#ifndef KINEMATICS_SAM_H
#define KINEMATICS_SAM_H

#include "utils_sam.h"
#include <vector>

struct IKResult {
    double theta1, theta2, theta3;
};

// Cinemática Inversa: De coordenada (x,y,z) a ángulos
IKResult solve_IK(double x, double y, double z, double L1, double L2, double L3, bool es_pata_derecha, bool es_tipo_rodilla = false);

// Cinemática Directa: De ángulos a puntos en el espacio (devuelve lista de puntos: [cadera, rodilla, pie])
std::vector<Vec3> solve_FK(double theta1, double theta2, double theta3, double L1, double L2, double L3, bool es_pata_derecha);

// Transformación de coordenadas: Del mundo global al hombro local
Vec3 origen_a_cadera_local(Vec3 coord_pie, Vec3 cuerpo_rpy, Vec3 hip_offset, Vec3 cuerpo_pos = {0,0,0}, Vec3 centro_rotacion = {0,0,0});

// Utilidad matemática
double wrap_to_pi(double angle);

#endif