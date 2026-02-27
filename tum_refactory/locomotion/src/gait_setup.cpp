#include "gait_setup.hpp"
#include "utils.hpp"
#include "kinematics.hpp"
#include "robot_config.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

using Eigen::Vector3d;

bool init_gait_posture(
    bool dont_ask,
    double& duracion_test,
    double& step_duration,
    double& vx,
    double& vy,
    double& wz,
    double& step_h,
    CommandData* shm_ptr,
    Eigen::Vector3d& target_body_pos,
    Eigen::Vector3d& target_body_rpy,
    double duty_factor,
    const double (&gait_offsets)[4],
    Eigen::Vector3d (&start_foot_positions)[4])
{
    // --- 1. INPUT DATOS BÁSICOS (Marcha) ---
    if (dont_ask) {
        std::cout << "[INFO] Modo Automático: Usando parámetros de marcha por defecto." << std::endl;
        step_duration = 3.0; 
        step_h = 40.0 / 1000.0;
        duracion_test = 10.0;
        vx = 40.0 / 1000.0;  // <--- CORRECCIÓN AQUÍ (mm a m)
        vy = 0.0 / 1000.0; 
        wz = 0.0;

    } else {
        step_duration = pedir_dato<double>("Periodo paso (s) (ej. 0.4)"); 
        step_h = pedir_dato<double>("Altura paso (mm) (ej. 40)") / 1000.0;
        duracion_test = pedir_dato<double>("Duración test (seg) (ej. 10)");

        std::cout << ">> Velocidad X (mm/s): "; std::cin >> vx; vx /= 1000.0;
        std::cout << ">> Velocidad Y (mm/s): "; std::cin >> vy; vy /= 1000.0;
        std::cout << ">> Giro Wz (rad/s): "; std::cin >> wz;
    }

    std::cout << "\n[INFO] Usando STAND_XYZ como origen." << std::endl;

    // ============================================================
    // FASE A: ALINEACIÓN SUAVE (A postura Neutra)
    // ============================================================
    std::cout << ">> Enviando comando de alineación inicial (3.0s)..." << std::endl;
    double align_duration = 3.0;     
    LegAngles start_angles[4];

    for(int p=0; p<4; ++p) {
        bool is_right = (p == 1 || p == 3);
        Vector3d origin = LEGS_STAND_XYZ[p];
        double start_phase = fmod(0.0 + gait_offsets[p], 1.0);
        
        Vector3d pt_pos;

        // Calculamos proporciones de tiempo reales (Lógica corregida)
        double stance_portion = duty_factor;
        double swing_portion = 1.0 - duty_factor;
        
        bool stance = (start_phase >= swing_portion);

        if (stance) {
            double t = (start_phase - swing_portion) / stance_portion;
            Vector3d v_total = Vector3d(vx, vy, 0.0) + Vector3d(0,0,wz).cross(HIP_OFFSETS[p]);
            
            double t_stance = step_duration * stance_portion;
            Vector3d p_start = origin - v_total * (t_stance / 2.0);
            Vector3d p_target = origin + v_total * (t_stance / 2.0);
            
            pt_pos = p_target + t * (p_start - p_target);
            pt_pos.z() = origin.z();
        } else {
            double t_stance = step_duration * stance_portion;
            double t_swing = step_duration * swing_portion;
            
            TrajectoryPoint3D tp = generate_bezier_swing(start_phase, duty_factor, origin, HIP_OFFSETS[p], vx, vy, wz, 
                                          t_stance, t_swing, step_h);
            pt_pos = tp.pos;
        }

        start_foot_positions[p] = pt_pos;
        start_angles[p] = solve_IK(pt_pos.x(), pt_pos.y(), pt_pos.z(), is_right);
    }

    if (shm_ptr) {
        for(int p=0; p<4; ++p) {
            if (start_angles[p].valid) {
                shm_ptr->angles[p][0] = start_angles[p].th1;
                shm_ptr->angles[p][1] = start_angles[p].th2;
                shm_ptr->angles[p][2] = start_angles[p].th3;
                
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                
                shm_ptr->velocities[p][0] = 5.0; 
                shm_ptr->velocities[p][1] = 5.0; 
                shm_ptr->velocities[p][2] = 5.0;
                
                shm_ptr->is_stance[p] = true;
            }
        }
        shm_ptr->transition_time = align_duration; 
        shm_ptr->is_walking = false;    
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << ">> Alineación completada." << std::endl;

    // ============================================================
    // FASE B: CONFIGURACIÓN Y APLICACIÓN DE POSTURA
    // ============================================================
    for(int p=0; p<4; ++p) {
        shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
        shm_ptr->desired_accel[p][0] = 0.0; shm_ptr->desired_accel[p][1] = 0.0; shm_ptr->desired_accel[p][2] = 0.0;
    }

    double in_bx = 0.0, in_by = 0.0, in_bz = 0.0;
    double in_roll = 0.0, in_pitch = 0.0, in_yaw = 0.0;

    if (dont_ask) {
            // --- HARDCODEA AQUÍ TU POSTURA DE PRUEBA ---
            in_bx = 0.0;     // Offset Cuerpo X (mm) [Adelante+]
            in_by = 0.0;     // Offset Cuerpo Y (mm) [Izq+]
            in_bz = 0.0;     // Offset Cuerpo Z (mm) [Abajo-]
            in_roll = 0.0;   // Roll (grados)
            in_pitch = 1.0;  // Pitch (grados)
            in_yaw = 0.0;    // Yaw (grados)
            // -------------------------------------------
    } else {
        std::cout << "========================================" << std::endl;
        std::cout << "   CONFIGURACIÓN DE POSTURA CORPORAL    " << std::endl;
        std::cout << "========================================" << std::endl;

        in_bx = pedir_dato<double>("Offset Cuerpo X (mm) [Adelante+]");
        in_by = pedir_dato<double>("Offset Cuerpo Y (mm) [Izq+]");
        in_bz = pedir_dato<double>("Offset Cuerpo Z (mm) [Abajo-]"); 
        in_roll  = pedir_dato<double>("Roll (grados)");
        in_pitch = pedir_dato<double>("Pitch (grados) [Nariz Abajo+]");
        in_yaw   = pedir_dato<double>("Yaw (grados)");

        std::cout << "\n>>> PRESIONA ENTER PARA APLICAR POSTURA (SIN CAMINAR) <<<" << std::endl;
        // Limpiamos el buffer por si quedó un salto de línea
        std::cin.ignore(10000, '\n'); 
        std::cin.get(); 
    }

    target_body_pos = Vector3d(in_bx / 1000.0, in_by / 1000.0, in_bz / 1000.0);
    target_body_rpy = Vector3d(in_roll * M_PI / 180.0, in_pitch * M_PI / 180.0, in_yaw * M_PI / 180.0);

    std::cout << ">> Aplicando postura corporal..." << std::endl;

    for(int p=0; p<4; ++p) {
        bool is_right = (p == 1 || p == 3);
        Vector3d foot_flat = start_foot_positions[p];
        Vector3d ik_input = ComputeWholeBodyIK(foot_flat, p, target_body_pos, target_body_rpy);
        LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), is_right);
        
        if(ik.valid) {
             shm_ptr->angles[p][0] = ik.th1;
             shm_ptr->angles[p][1] = ik.th2;
             shm_ptr->angles[p][2] = ik.th3;
             
             shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
             shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
             shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
             shm_ptr->velocities[p][0] = 5.0; 
             shm_ptr->velocities[p][1] = 5.0; 
             shm_ptr->velocities[p][2] = 5.0;
        }
    }
    
    shm_ptr->transition_time = 2.0; 
    shm_ptr->is_walking = false;    

    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << ">> Postura aplicada." << std::endl;

    return true;
}
