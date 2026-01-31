#include "bezier_sam.h"
#include <cmath>

// ==========================================
// Funciones de Ayuda (Internas)
// ==========================================

// Interpolación Lineal
Vec3 lerp(Vec3 P0, Vec3 P1, double t) {
    // ((1-t)*P0)+(t*P1)
    return ((1.0 - t) * P0) + (t * P1);
}

// Cálculo de puntos de control para el arco de Bezier
std::vector<Vec3> calcular_puntos_control_paso(Vec3 inicio, Vec3 fin, double altura_deseada) {
    Vec3 P0 = inicio;
    Vec3 P3 = fin;
    
    double z_suelo = inicio.z;
    // Factor 4/3 para asegurar la altura real en el pico de la curva Bezier
    double h_ctrl = z_suelo + (4.0 / 3.0) * altura_deseada;
    
    Vec3 P1(inicio.x, inicio.y, h_ctrl);
    Vec3 P2(fin.x, fin.y, h_ctrl);

    return {P0, P1, P2, P3};
}

// ==========================================
// Implementación de Funciones Públicas (del .h)
// ==========================================

Vec3 matrix_bezier(Vec3 P0, Vec3 P1, Vec3 P2, Vec3 P3, double t) {
    // Polinomios de Bernstein cúbicos explícitos
    double inv_t = 1.0 - t;
    double inv_t2 = inv_t * inv_t;
    double inv_t3 = inv_t2 * inv_t;
    double t2 = t * t;
    double t3 = t2 * t;

    double w0 = inv_t3;              // (1-t)^3
    double w1 = 3 * t * inv_t2;      // 3t(1-t)^2
    double w2 = 3 * t2 * inv_t;      // 3t^2(1-t)
    double w3 = t3;                  // t^3

    return (w0 * P0) + (w1 * P1) + (w2 * P2) + (w3 * P3);
}

Vec3 generar_paso(Vec3 inicio, Vec3 final, double altura, double t) {
    std::vector<Vec3> ctrls = calcular_puntos_control_paso(inicio, final, altura);
    // Extraer puntos del vector
    Vec3 P0 = ctrls[0];
    Vec3 P1 = ctrls[1];
    Vec3 P2 = ctrls[2];
    Vec3 P3 = ctrls[3];

    if (t <= 1.0) {
        // Fase de Vuelo: Curva de Bezier
        return matrix_bezier(P0, P1, P2, P3, t);
    } else {
        // Fase de Apoyo (Retorno): Interpolación lineal
        // Lerp desde P3 (fin) hasta P0 (inicio) usando (t-1)
        return lerp(P3, P0, t - 1.0);
    }
}