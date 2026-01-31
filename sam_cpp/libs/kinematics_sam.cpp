#include "kinematics_sam.h"
#include <cmath>
#include <algorithm> // std::clamp, std::max, std::min

// ==========================================
// Estructuras Auxiliares Internas (Matrices)
// ==========================================
// Nota: Vec3 viene de utils_sam.h (incluido en kinematics_sam.h)

struct Mat3 {
    double m[3][3];

    Mat3() : m{{0}} {}

    Mat3(double r00, double r01, double r02,
         double r10, double r11, double r12,
         double r20, double r21, double r22) {
        m[0][0] = r00; m[0][1] = r01; m[0][2] = r02;
        m[1][0] = r10; m[1][1] = r11; m[1][2] = r12;
        m[2][0] = r20; m[2][1] = r21; m[2][2] = r22;
    }

    Vec3 operator*(const Vec3& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }

    Mat3 operator*(const Mat3& other) const {
        Mat3 res;
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                for(int k=0; k<3; k++)
                    res.m[i][j] += m[i][k] * other.m[k][j];
        return res;
    }

    Mat3 transpose() const {
        Mat3 res;
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                res.m[i][j] = m[j][i];
        return res;
    }
};

struct Mat4 {
    double m[4][4];

    Mat4() : m{{0}} {}

    Mat4(double r00, double r01, double r02, double r03,
         double r10, double r11, double r12, double r13,
         double r20, double r21, double r22, double r23,
         double r30, double r31, double r32, double r33) {
        m[0][0]=r00; m[0][1]=r01; m[0][2]=r02; m[0][3]=r03;
        m[1][0]=r10; m[1][1]=r11; m[1][2]=r12; m[1][3]=r13;
        m[2][0]=r20; m[2][1]=r21; m[2][2]=r22; m[2][3]=r23;
        m[3][0]=r30; m[3][1]=r31; m[3][2]=r32; m[3][3]=r33;
    }

    Mat4 operator*(const Mat4& other) const {
        Mat4 res;
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
                for(int k=0; k<4; k++)
                    res.m[i][j] += m[i][k] * other.m[k][j];
        return res;
    }

    Vec3 multiply_point(const Vec3& p) const {
        double x = m[0][0]*p.x + m[0][1]*p.y + m[0][2]*p.z + m[0][3]*1.0;
        double y = m[1][0]*p.x + m[1][1]*p.y + m[1][2]*p.z + m[1][3]*1.0;
        double z = m[2][0]*p.x + m[2][1]*p.y + m[2][2]*p.z + m[2][3]*1.0;
        return {x, y, z};
    }
};

// ==========================================
// Implementación de Funciones
// ==========================================

double wrap_to_pi(double angle) {
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

Mat3 get_rotation_matrix(double r, double p, double y) {
    Mat3 Rx(1, 0, 0, 0, std::cos(r), -std::sin(r), 0, std::sin(r), std::cos(r));
    Mat3 Ry(std::cos(p), 0, std::sin(p), 0, 1, 0, -std::sin(p), 0, std::cos(p));
    Mat3 Rz(std::cos(y), -std::sin(y), 0, std::sin(y), std::cos(y), 0, 0, 0, 1);
    return Rz * Ry * Rx; 
}

IKResult solve_IK(double x, double y, double z, double L1, double L2, double L3, bool es_pata_derecha, bool es_tipo_rodilla) {
    double y_local = es_pata_derecha ? -y : y;

    // --- Seguridad ---
    double dist_yz = std::sqrt(y_local*y_local + z*z);
    double min_dist_yz = L1 + 0.001;
    
    if (dist_yz < min_dist_yz) {
        double scale_yz = min_dist_yz / (dist_yz + 1e-9);
        y_local *= scale_yz;
        z *= scale_yz;
    }

    double R_check = std::sqrt(std::max(dist_yz*dist_yz - L1*L1, 0.0));
    double dist_total = std::sqrt(x*x + R_check*R_check);
    double max_reach = (L2 + L3) * 0.999;

    if (dist_total > max_reach) {
        double scale = max_reach / dist_total;
        x *= scale;
        double R_new = R_check * scale;
        double D_new = std::sqrt(R_new*R_new + L1*L1);
        double scale_yz_final = D_new / dist_yz;
        y_local *= scale_yz_final;
        z *= scale_yz_final;
    }

    // --- Cadera ---
    double D = std::max(std::sqrt(y_local*y_local + z*z), L1);
    double theta1 = 0.0;
    if (D > 1e-6) {
        theta1 = std::atan2(z, y_local) + std::acos(std::clamp(L1 / D, -1.0, 1.0));
    }

    // --- Rodilla ---
    double R = -std::sqrt(std::max(D*D - L1*L1, 0.0)); 
    double H = std::min(std::sqrt(x*x + R*R), L2 + L3 - 1e-6);

    double arg_acos = (L2*L2 + L3*L3 - H*H) / (2 * L2 * L3);
    double phi1 = std::acos(std::clamp(arg_acos, -1.0, 1.0));

    double theta3, signo_phi3;
    if (es_tipo_rodilla) {
        theta3 = phi1 - M_PI;
        signo_phi3 = 1.0;
    } else {
        theta3 = M_PI - phi1;
        signo_phi3 = -1.0;
    }

    // --- Muslo ---
    double phi2 = std::atan2(R, x);
    double arg_asin = (L3 * std::sin(phi1)) / H;
    double phi3 = std::asin(std::clamp(arg_asin, -1.0, 1.0));
    
    double theta2 = phi2 + (signo_phi3 * phi3);

    if (es_pata_derecha) theta1 = -theta1;

    return {wrap_to_pi(theta1), wrap_to_pi(theta2), wrap_to_pi(theta3)};
}

Mat4 dh_transform(double theta, double d, double a, double alpha) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    return Mat4(c, -s*ca, s*sa, a*c, s, c*ca, -c*sa, a*s, 0, sa, ca, d, 0, 0, 0, 1);
}

std::vector<Vec3> solve_FK(double theta1, double theta2, double theta3, double L1, double L2, double L3, bool es_pata_derecha) {
    double l1_eff = es_pata_derecha ? -L1 : L1;

    Mat4 T1 = dh_transform(M_PI/2, 0, 0, M_PI/2);
    Mat4 T2 = dh_transform(theta1, 0, l1_eff, M_PI/2);
    Mat4 T3 = dh_transform(M_PI/2, 0, 0, -M_PI/2);
    Mat4 T4 = dh_transform(theta2, 0, L2, 0);
    Mat4 T5 = dh_transform(theta3, 0, L3, 0);

    Vec3 P0_vec = {0, 0, 0}; 

    Mat4 M_Hip = T1 * T2;
    Vec3 p1 = M_Hip.multiply_point(P0_vec);

    Mat4 M_Knee = M_Hip * T3 * T4;
    Vec3 p2 = M_Knee.multiply_point(P0_vec);

    Mat4 M_Foot = M_Knee * T5;
    Vec3 p3 = M_Foot.multiply_point(P0_vec);

    return {P0_vec, p1, p2, p3};
}

// Nota: Los valores por defecto de cuerpo_pos y centro_rotacion están en el .h
Vec3 origen_a_cadera_local(Vec3 coord_pie, Vec3 cuerpo_rpy, Vec3 hip_offset, Vec3 cuerpo_pos, Vec3 centro_rotacion) {
    Mat3 R_cuerpo = get_rotation_matrix(cuerpo_rpy.x, cuerpo_rpy.y, cuerpo_rpy.z);
    
    Vec3 rot_center_rot = R_cuerpo * centro_rotacion;
    Vec3 compensacion_pivote = centro_rotacion - rot_center_rot;

    Vec3 rot_hip_offset = R_cuerpo * hip_offset;
    Vec3 hip_origen = cuerpo_pos + compensacion_pivote + rot_hip_offset;

    Vec3 vector_origen = coord_pie - hip_origen;
    Vec3 vector_local = R_cuerpo.transpose() * vector_origen;

    return vector_local;
}