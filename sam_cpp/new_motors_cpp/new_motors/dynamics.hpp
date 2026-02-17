#pragma once
#include <cmath>
#include "robot_config.hpp" // Necesita L1, L2, L3

struct LegTorques {
    double t1_abad;
    double t2_hip;
    double t3_knee;
};

inline LegTorques ComputeTorques(double q1, double q2, double q3, 
                          double fx, double fy, double fz,
                          bool es_pata_derecha) {
    double l1 = L1; double l2 = L2; double l3 = L3; // Usamos las constantes globales
    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s2 = std::sin(q2), c2 = std::cos(q2);
    double s23 = std::sin(q2 + q3);
    double c23 = std::cos(q2 + q3);
    double side = es_pata_derecha ? -1.0 : 1.0; 

    
    double j12 = l2*c2 + l3*c23;
    double j21 = -l1*side*s1 + (l2*c2 + l3*c23)*c1;
    double j22 = -(l2*s2 + l3*s23)*s1;
    double j23 = -l3*s23*s1; 
    double j31 = l1*side*c1 + (l2*c2 + l3*c23)*s1;
    double j32 = (l2*s2 + l3*s23)*c1;
    double j33 = l3*s23*c1;
    // j11 es 0 y j13 es l3*c23, optimizado abajo:

    LegTorques tau;
    tau.t1_abad = 0*fx + j21*fy + j31*fz;
    tau.t2_hip  = j12*fx + j22*fy + j32*fz;
    tau.t3_knee = (l3*c23)*fx + j23*fy + j33*fz;
    return tau;
}