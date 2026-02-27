#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <Eigen/Dense>

using Eigen::Vector3d;

// ============================================================
// 2. CONFIGURACIÓN FÍSICA (STAND_XYZ)
// ============================================================
const double L1 = 0.093;
const double L2 = 0.147;
const double L3 = 0.230;
const double L_BODY = 0.375; // Largo total
const double W_BODY = 0.200; // Ancho total

// Distancia del Centro Geométrico a los hombros (Hips)
const double DX = L_BODY / 2.0;
const double DY = W_BODY / 2.0;

// Posición de cada cadera respecto al centro del robot (0,0,0)
// Orden: 0:FL, 1:FR, 2:BL, 3:BR
const Vector3d HIP_OFFSETS[4] = {
    Vector3d( DX,  DY, 0.0), // FL
    Vector3d( DX, -DY, 0.0), // FR
    Vector3d(-DX,  DY, 0.0), // BL
    Vector3d(-DX, -DY, 0.0)  // BR
};

const Vector3d LEGS_STAND_XYZ[4] = {
    Vector3d( 0.01,  0.093, -0.306), // 0: FL
    Vector3d( 0.01, -0.093, -0.306), // 1: FR
    Vector3d( 0.01,  0.093, -0.306), // 2: BL
    Vector3d( 0.01, -0.093, -0.306)  // 3: BR
};

// const Vector3d LEGS_STAND_XYZ[4] = { // Caminata
//     Vector3d( 0.01,   0.103, -0.296), // 0: FL
//     Vector3d( 0.01,  -0.093, -0.296), // 1: FR
//     Vector3d(-0.028,  0.103, -0.296), // 2: BL
//     Vector3d(-0.028, -0.093, -0.296)  // 3: BR
// };

// Vector3d LEGS_STAND_XYZ[4] = { // Original
//     { 0.02,   0.093, -0.316}, // 0: FL
//     { 0.02,  -0.093, -0.316}, // 1: FR
//     {-0.018,  0.093, -0.316}, // 2: BL
//     {-0.018, -0.093, -0.316}  // 3: BR
// };

#endif // ROBOT_CONFIG_HPP