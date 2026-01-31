#include "quadruped_robot.h"
#include <cmath>
#include <iostream>

// ==========================================
// Implementación QuadrupedRobot
// ==========================================

QuadrupedRobot::QuadrupedRobot(double body_length, double body_width, double _l1, double _l2, double _l3) 
    : l1(_l1), l2(_l2), l3(_l3), 
      // Inicializamos el GaitGenerator con valores temporales, se reconfiguran en el cuerpo del constructor
      brain(0.0, {}) 
{
    lx = body_length / 2.0;
    ly = body_width / 2.0;

    // Definir Offsets de Cadera (Del centro del robot a cada hombro)
    hip_offsets["FL"] = Vec3( lx,  ly, 0);
    hip_offsets["FR"] = Vec3( lx, -ly, 0);
    hip_offsets["BL"] = Vec3(-lx,  ly, 0);
    hip_offsets["BR"] = Vec3(-lx, -ly, 0);

    // Calcular altura de parado ideal (ligeramente flexionada)
    stand_height = std::sqrt(l2*l2 + l3*l3) - 0.05;

    leg_names = {"FL", "FR", "BL", "BR"};
    for(const auto& name : leg_names) {
        leg_types[name] = false; // Todas dinámicas por defecto
        prev_angles[name] = {0.0, 0.0, 0.0};
    }

    // Reinicializar el GaitGenerator con la altura correcta y postura base
    // Nota: Esto sobrescribe la inicialización de la lista de inicializadores
    brain = GaitGenerator(stand_height, get_default_stance());
    
    last_comp = Vec3(0,0,0);
}

std::map<std::string, Vec3> QuadrupedRobot::get_default_stance() {
    std::map<std::string, Vec3> stand_pose;
    double extra_width = 0.015;
    double shift_x = 0.025;

    for (auto const& [leg, offset] : hip_offsets) {
        double signo_y = (offset.y > 0) ? 1.0 : -1.0;
        
        // Calculamos dónde debería estar el pie en el suelo (Z negativo relativo al cuerpo)
        stand_pose[leg] = Vec3(
            offset.x + shift_x,
            offset.y + (signo_y * (l1 + extra_width)),
            -stand_height
        );
    }
    return stand_pose;
}

std::map<std::string, std::vector<double>> QuadrupedRobot::step(double t, double dt, RobotCommand cmd, Vec3 sensor_data, std::string modo) {
    // 1. Configurar modos de patas (si vienen en el comando)
    for (const auto& [leg, is_static] : cmd.leg_modes) {
        if (leg_types.find(leg) != leg_types.end()) {
            leg_types[leg] = is_static;
        }
    }

    // 2. Obtener trayectoria de pies del GaitGenerator
    // Convertimos el comando a Vec3 para el generador
    Vec3 vel_cmd(cmd.vx, cmd.vy, cmd.wz);
    
    // Llamada al cerebro
    GaitOutput gait_out = brain.update(t, vel_cmd, sensor_data, modo);
    
    // targets serán las posiciones de los pies generadas por el gait
    std::map<std::string, Vec3> targets = gait_out.feet_pos;

    // 3. Postura del Cuerpo (Cinemática del Cuerpo)
    // Sumamos el offset calculado por el Gait (Sway) más el comando manual
    double final_body_x = cmd.body_x + gait_out.body_shift.x;
    double final_body_y = cmd.body_y + gait_out.body_shift.y;
    
    // Usamos la altura comandada manualmente, ignorando la sugerencia Z del gait 
    // (o podrías sumarlas si quisieras que el gait controle la altura dinámica)
    double final_body_z = cmd.body_z; 

    Vec3 body_pos(final_body_x, final_body_y, final_body_z);
    
    // Rotación del cuerpo
    Vec3 input_rpy(cmd.roll, cmd.pitch, cmd.yaw);
    // Aquí sumaríamos la compensación de IMU si estuviera implementada
    Vec3 body_rpy = input_rpy; 

    Vec3 pivote(cmd.piv_x, cmd.piv_y, 0.0);

    // 4. Calcular Cinemática Inversa Final
    return compute_pose(body_pos, body_rpy, pivote, targets);
}

std::map<std::string, std::vector<double>> QuadrupedRobot::compute_pose(Vec3 body_pos, Vec3 body_rpy, Vec3 pivote, std::map<std::string, Vec3> targets) {
    std::map<std::string, std::vector<double>> angles;

    for (const auto& leg : leg_names) {
        Vec3 offset = hip_offsets[leg];
        Vec3 target = targets[leg];

        // 1. Transformar del mundo al sistema local de la cadera
        // Función de kinematics_sam.h
        Vec3 local = origen_a_cadera_local(target, body_rpy, offset, body_pos, pivote);

        // 2. Resolver IK
        bool is_right_leg = (leg.find("R") != std::string::npos);
        // Si la pata está marcada como 'tipo rodilla' especial (leg_types[leg]), se pasa true
        // En tu código python pasabas self.leg_types[name]
        bool knee_type = leg_types[leg];

        IKResult ik = solve_IK(local.x, local.y, local.z, l1, l2, l3, is_right_leg, knee_type);

        // 3. Empaquetar y Suavizar
        std::vector<double> raw_q = {ik.theta1, ik.theta2, ik.theta3};
        std::vector<double> clean_q = unwrap_angles(leg, raw_q);
        
        angles[leg] = clean_q;
    }

    return angles;
}

std::vector<double> QuadrupedRobot::unwrap_angles(std::string leg_name, std::vector<double> new_angles) {
    std::vector<double> prev = prev_angles[leg_name];
    std::vector<double> smoothed = new_angles;

    for (int i = 0; i < 3; i++) {
        double diff = new_angles[i] - prev[i];
        if (diff > M_PI) {
            smoothed[i] -= 2 * M_PI;
        } else if (diff < -M_PI) {
            smoothed[i] += 2 * M_PI;
        }
    }
    
    prev_angles[leg_name] = smoothed;
    return smoothed;
}