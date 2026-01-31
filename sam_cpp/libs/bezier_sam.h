#ifndef BEZIER_SAM_H
#define BEZIER_SAM_H

#include "utils_sam.h"
#include <vector>

// Genera un punto en el espacio para un instante t (0.0 a 2.0)
// t <= 1: Fase de vuelo (Curva Bezier)
// t > 1: Fase de apoyo (Interpolarción Lineal)
Vec3 generar_paso(Vec3 inicio, Vec3 final, double altura, double t);

// Función pura de Bezier (útil si se necesita externamente)
Vec3 matrix_bezier(Vec3 P0, Vec3 P1, Vec3 P2, Vec3 P3, double t);

#endif