#include "setup.hpp"
#include "utils.hpp"
#include "kinematics.hpp"
#include "robot_config.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cmath>

using Eigen::Vector3d;

bool setup_robot(
    int& max_ciclos,
    double& step_duration,
    double& step_len,
    double& step_h,
    IMUData*& imu_ptr,
    CommandData*& shm_ptr,
    TelemetryData*& tel_ptr,
    ContactData*& contact_ptr,
    Eigen::Vector3d& target_body_pos,
    Eigen::Vector3d& target_body_rpy,
    double (&gait_offsets)[4],
    Eigen::Vector3d (&start_foot_positions)[4]
) {
    // --- 1. INPUT DATOS BÁSICOS (Marcha) ---
    max_ciclos = pedir_dato<int>("Ciclos totales (ej. 20)");
    step_duration = pedir_dato<double>("Periodo paso (s) (ej. 0.4)"); 
    step_len = pedir_dato<double>("Largo paso (mm) (ej. 80)");
    step_h = pedir_dato<double>("Altura paso (mm) (ej. 40)");

    // Conversión a metros
    step_len /= 1000.0;
    step_h /= 1000.0;

    // --- NUEVO: ABRIR MEMORIA COMPARTIDA IMU ---
    int imu_fd = shm_open("/imu_data", O_RDONLY, 0666);
    if (imu_fd == -1) {
        std::cerr << "[ERROR] Asegúrate de ejecutar imu_publisher primero." << std::endl;
        return false;
    }
    imu_ptr = static_cast<IMUData*>(mmap(0, sizeof(IMUData), PROT_READ, MAP_SHARED, imu_fd, 0));
    if (imu_ptr == MAP_FAILED) {
        std::cerr << "Error mapeando IMU" << std::endl;
        return false;
    }

    const char* shm_name = "/rex_cmd";
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(CommandData));
    shm_ptr = static_cast<CommandData*>(mmap(0, sizeof(CommandData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    if (shm_ptr == MAP_FAILED) {
        std::cerr << "Error accediendo a memoria compartida" << std::endl;
        return false;
    }

    const char* tel_name = "/rex_tel";
    int tel_fd = shm_open(tel_name, O_CREAT | O_RDWR, 0666);
    ftruncate(tel_fd, sizeof(TelemetryData));
    tel_ptr = static_cast<TelemetryData*>(mmap(0, sizeof(TelemetryData), PROT_READ | PROT_WRITE, MAP_SHARED, tel_fd, 0));

    const char* contact_name = "/rex_contact";
    int contact_fd = shm_open(contact_name, O_CREAT | O_RDWR, 0666);
    ftruncate(contact_fd, sizeof(ContactData));
    contact_ptr = static_cast<ContactData*>(mmap(0, sizeof(ContactData), PROT_READ | PROT_WRITE, MAP_SHARED, contact_fd, 0));

    if (contact_ptr == MAP_FAILED) {
        std::cerr << "Error accediendo a memoria compartida de contactos" << std::endl;
        return false;
    }

    if (tel_ptr == MAP_FAILED) {
        std::cerr << "Error accediendo a memoria compartida de telemetria" << std::endl;
        return false;
    }

    std::cout << "\n[INFO] Usando STAND_XYZ como origen." << std::endl;

    // --- CONFIGURACIÓN DE GAIT (Trote) ---
    // (gait_offsets se pasa y se inicializa en el main como {0.0, 0.5, 0.5, 0.0})

    // ============================================================
    // FASE A: ALINEACIÓN SUAVE (A postura Neutra)
    // ============================================================
    std::cout << ">> Alineando suavemente a posición de inicio (Neutra)..." << std::endl;
    
    double align_duration = 3.0;     

    LegAngles start_angles[4];
    // Calculamos ángulos neutros (Sin offsets de cuerpo aún)
    std::cout << ">> Enviando comando de alineación inicial (3.0s)..." << std::endl;
    
    // 1. Calcular ángulos neutros
    for(int p=0; p<4; ++p) {
        bool is_right = (p == 1 || p == 3);
        Vector3d origin = LEGS_STAND_XYZ[p];
        double start_phase = fmod(0.0 + gait_offsets[p], 1.0);
        TrajectoryPoint pt = get_step_trajectory(start_phase, step_duration, origin.x(), origin.z(), step_len, step_h);
        start_foot_positions[p] = Vector3d(pt.x, origin.y(), pt.z);
        start_angles[p] = solve_IK(pt.x, origin.y(), pt.z, is_right);
    }

    // 2. Escribir en SHM (UNA SOLA VEZ)
    if (shm_ptr) {
        for(int p=0; p<4; ++p) {
            if (start_angles[p].valid) {
                shm_ptr->angles[p][0] = start_angles[p].th1;
                shm_ptr->angles[p][1] = start_angles[p].th2;
                shm_ptr->angles[p][2] = start_angles[p].th3;
                
                // Configuración para modo estático en main_sebas:
                shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
                shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
                shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
                
                // Velocidad límite (importante para que no se mueva lento artificialmente)
                shm_ptr->velocities[p][0] = 5.0; 
                shm_ptr->velocities[p][1] = 5.0; 
                shm_ptr->velocities[p][2] = 5.0;
                
                shm_ptr->is_stance[p] = true;
            }
        }
        shm_ptr->transition_time = align_duration; // main_sebas usará esto para interpolar
        shm_ptr->is_walking = false;    // Activa el modo interpolación estática
    }

    // 3. Esperar a que el robot termine el movimiento físico
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << ">> Alineación completada." << std::endl;

    // ============================================================
    // FASE B: CONFIGURACIÓN Y APLICACIÓN DE POSTURA (ESTÁTICA)
    // ============================================================
    std::cout << "\n>> Alineación base completada. Manteniendo posición neutra." << std::endl;

    // Fijamos valores finales exactos
    for(int p=0; p<4; ++p) {
        if (start_angles[p].valid) {
            shm_ptr->angles[p][0] = start_angles[p].th1;
            shm_ptr->angles[p][1] = start_angles[p].th2;
            shm_ptr->angles[p][2] = start_angles[p].th3;
        }
        shm_ptr->velocities[p][0] = 0.0; shm_ptr->velocities[p][1] = 0.0; shm_ptr->velocities[p][2] = 0.0;
        shm_ptr->desired_accel[p][0] = 0.0; shm_ptr->desired_accel[p][1] = 0.0; shm_ptr->desired_accel[p][2] = 0.0;
        shm_ptr->is_stance[p] = true;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "   CONFIGURACIÓN DE POSTURA CORPORAL    " << std::endl;
    std::cout << "========================================" << std::endl;

    double in_bx = pedir_dato<double>("Offset Cuerpo X (mm) [Adelante+]");
    double in_by = pedir_dato<double>("Offset Cuerpo Y (mm) [Izq+]");
    double in_bz = pedir_dato<double>("Offset Cuerpo Z (mm) [Abajo-]"); 
    
    double in_roll  = pedir_dato<double>("Roll (grados)");
    double in_pitch = pedir_dato<double>("Pitch (grados) [Nariz Abajo+]");
    double in_yaw   = pedir_dato<double>("Yaw (grados)");

    // Objetivos finales de postura
    target_body_pos = Vector3d(in_bx / 1000.0, in_by / 1000.0, in_bz / 1000.0);
    target_body_rpy = Vector3d(in_roll * M_PI / 180.0, in_pitch * M_PI / 180.0, in_yaw * M_PI / 180.0);

    std::cout << "\n>>> PRESIONA ENTER PARA APLICAR POSTURA (SIN CAMINAR) <<<" << std::endl;
    std::cin.get(); 

    std::cout << ">> Aplicando postura corporal..." << std::endl;

    // --- TRANSICIÓN SUAVE A LA POSTURA MODIFICADA (Implementada) ---
    // Se utiliza la misma lógica de suavizado (coseno) que en la Fase A
    double posture_duration = 2.0; // 2 segundos para acomodarse
    double posture_dt = 0.01;      // Paso de tiempo
    int posture_steps = posture_duration / posture_dt;

    std::cout << ">> Enviando comando de postura (2.0s)..." << std::endl;

    for(int p=0; p<4; ++p) {
        bool is_right = (p == 1 || p == 3);
        Vector3d foot_flat = start_foot_positions[p];
        // Calcular IK con el cuerpo rotado
        Vector3d ik_input = ComputeWholeBodyIK(foot_flat, p, target_body_pos, target_body_rpy);
        LegAngles ik = solve_IK(ik_input.x(), ik_input.y(), ik_input.z(), is_right);
        
        if(ik.valid) {
             shm_ptr->angles[p][0] = ik.th1;
             shm_ptr->angles[p][1] = ik.th2;
             shm_ptr->angles[p][2] = ik.th3;
             
             // Asegurar ganancias fuertes y velocidad límite
             shm_ptr->kp_scale[p][0] = 1.0; shm_ptr->kd_scale[p][0] = 1.0;
             shm_ptr->kp_scale[p][1] = 1.0; shm_ptr->kd_scale[p][1] = 1.0;
             shm_ptr->kp_scale[p][2] = 1.0; shm_ptr->kd_scale[p][2] = 1.0;
             shm_ptr->velocities[p][0] = 5.0; 
             shm_ptr->velocities[p][1] = 5.0; 
             shm_ptr->velocities[p][2] = 5.0;
        }
    }
    
    shm_ptr->transition_time = 2.0; // Tiempo para acomodar el cuerpo
    shm_ptr->is_walking = false;    // Modo estático

    // Esperar transición
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << ">> Postura aplicada." << std::endl;

    return true;
}
