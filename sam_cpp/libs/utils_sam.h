#ifndef UTILS_SAM_H
#define UTILS_SAM_H

#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    // Operadores para facilitar las matemáticas (v1 + v2, v1 * 0.5, etc.)
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    
    // Permitir: double * Vec3 (conmutatividad)
    friend Vec3 operator*(double s, const Vec3& v) {
        return {v.x * s, v.y * s, v.z * s};
    }
    
    // Norma (magnitud)
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

#endif