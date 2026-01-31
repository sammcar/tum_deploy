#include "gait_generator.h"
#include <iostream>
#include <algorithm> // std::clamp, std::max, std::min
#include <cmath>

// ==========================================
// Implementación GaitGenerator
// ==========================================

GaitGenerator::GaitGenerator(double _stand_height, std::map<std::string, Vec3> default_stance) {
    // --- Tuning Parameters (Defaults) ---
    stand_height = _stand_height;
    walking_height = _stand_height;
    spider_offset = 0.0;
    step_height = 0.08;
    step_length = 0.05;
    body_x_bias = 0.01;
    sway_y = 0.0025;
    sway_x = 0.0015;
    center_y_bias = 0.0;
    
    turn_T_min = 0.35;
    turn_T_max = 1.20;
    turn_T_stop = 0.70;
    cmd_dead = 0.01;

    // --- State Initialization ---
    activity_level = 0.0;
    yaw_trim = 0.0;
    state = GaitState::STAND;
    turn_index = 0;
    local_progress = 0.0;
    last_t_global = 0.0;
    first_run = true;
    stop_requested = false;

    sequence = {"FL", "BR", "FR", "BL"};
    active_leg = sequence[0];

    // --- Pose Setup ---
    init_rest_pose(default_stance);
    
    // Inicializar estado de pies en reposo
    for (auto const& [leg, pos] : pose_rest) {
        feet_state[leg] = pos;
    }

    swing_start = pose_rest[active_leg];
    swing_end = pose_rest[active_leg];
    lift_initialized = false;
    touchdown_done = false;
    swing_dx_accum = 0.0;
}

void GaitGenerator::init_rest_pose(const std::map<std::string, Vec3>& raw_stance) {
    // Convertir postura del robot (donde Z suele ser negativo) a postura local plana (Z=0)
    for (auto const& [leg, pos] : raw_stance) {
        pose_rest[leg] = Vec3(pos.x, pos.y, 0.0);
    }
}

void GaitGenerator::apply_treadmill(double delta_local, bool include_active, double treadmill_per_turn, double dir_sign, bool treadmill_on) {
    if (!treadmill_on || delta_local <= 0.0) return;

    // Lógica Python: dx = treadmill_per_turn * (delta_local / 3.0)
    double dx = treadmill_per_turn * (delta_local / 3.0); 
    
    for (auto& [leg, pos] : feet_state) {
        if (include_active || (leg != active_leg)) {
            pos.x -= dx;
        }
    }
}

GaitOutput GaitGenerator::update(double t_global, Vec3 vel_cmd, Vec3 imu_rpy, std::string modo) {
    // -------------------------------
    // 1. Cálculo del Delta Time (dt)
    // -------------------------------
    double dt = 0.0;
    if (first_run) {
        first_run = false;
        dt = 0.0;
    } else {
        dt = t_global - last_t_global;
        if (dt < 0.0) dt = 0.0;
        if (dt > 0.1) dt = 0.1; // Clamp safety
    }
    last_t_global = t_global;

    // -------------------------------
    // 2. Procesar Comando (Velocidad)
    // -------------------------------
    double vx = vel_cmd.x;
    double vy = vel_cmd.y;
    double input_mag = std::sqrt(vx*vx + vy*vy);
    double dir_sign = (vx >= 0.0) ? 1.0 : -1.0;

    // -------------------------------
    // 3. Lógica de Stop Request
    // -------------------------------
    if (modo != "Walk") {
        stop_requested = true;
    } else {
        stop_requested = (input_mag <= cmd_dead);
    }

    // -------------------------------
    // 4. Activity Level (Suavizado)
    // -------------------------------
    if (!stop_requested && input_mag > cmd_dead) {
        activity_level += 0.05;
    } else {
        activity_level -= 0.01;
    }
    activity_level = std::clamp(activity_level, 0.0, 1.0);

    // -------------------------------
    // 5. Transiciones de Estado
    // -------------------------------
    
    // STAND -> STARTUP
    if (state == GaitState::STAND && !stop_requested && input_mag > cmd_dead) {
        state = GaitState::STARTUP_FL;
        turn_index = 0;
        local_progress = 0.0;
        active_leg = sequence[turn_index];
        lift_initialized = false;
        touchdown_done = false;
    }

    // FREEZE EN STAND (Retorno temprano)
    if (state == GaitState::STAND && stop_requested) {
        local_progress = 0.0;
        lift_initialized = false;
        touchdown_done = false;
        
        // Reset pies a reposo
        for (auto const& leg : sequence) {
            feet_state[leg] = pose_rest[leg];
        }
        
        GaitOutput out;
        out.feet_pos = feet_state;
        // Aplicar bias X final
        for(auto& [l, p] : out.feet_pos) p.x -= body_x_bias;
        
        out.body_shift = Vec3(0, 0, stand_height);
        out.body_rot = Vec3(0, 0, 0);
        return out;
    }

    // -------------------------------
    // 6. Duración del Turno
    // -------------------------------
    double turn_T;
    if (state == GaitState::STOPPING) {
        turn_T = turn_T_stop;
    } else {
        double s = std::clamp(input_mag, 0.0, 1.0);
        turn_T = turn_T_max - s * (turn_T_max - turn_T_min);
    }

    // -------------------------------
    // 7. Avance Progresión Local
    // -------------------------------
    double old_local = local_progress;
    if (dt > 0.0) {
        local_progress += dt * (3.0 / turn_T);
        if (local_progress > 3.0) local_progress = 3.0;
    }
    double new_local = local_progress;

    // -------------------------------
    // 8. Lógica Treadmill & Swing
    // -------------------------------
    
    // Flag Treadmill: Solo activo si caminamos y no queremos parar
    bool treadmill_on = (state == GaitState::STARTUP_BR || state == GaitState::WALK) && (!stop_requested);

    double L = step_length;
    double treadmill_per_turn = (L / 4.0) * dir_sign * 1.0;

    // Segmentación por tramos (0..1, 1..2, 2..3)
    double seg_a = old_local;
    double seg_b = new_local;

    // --- TRAMO 0..1 (Apoyo Previo) ---
    if (seg_a < 1.0) {
        double end0 = std::min(seg_b, 1.0);
        apply_treadmill(end0 - seg_a, true, treadmill_per_turn, dir_sign, treadmill_on);
        seg_a = end0;
    }

    // Determinar estado objetivo para el siguiente paso
    GaitState target_state = state;
    if (stop_requested && (state == GaitState::WALK || state == GaitState::STARTUP_FL || state == GaitState::STARTUP_BR)) {
        target_state = GaitState::STOPPING;
    }

    // --- CRUCE A SWING (Inicio Fase 1) ---
    bool crossed_into_swing = (old_local < 1.0) && (new_local >= 1.0);
    if (crossed_into_swing) {
        lift_initialized = true;
        touchdown_done = false;
        swing_dx_accum = 0.0; // Reset acumulador
        
        swing_start = feet_state[active_leg];

        double advance = 0.0;
        if (target_state == GaitState::STARTUP_FL) {
            advance = (L / 2.0) * dir_sign;
            swing_end = swing_start;
            swing_end.x = swing_start.x + advance;
            swing_end.z = 0.0;
        } else if (target_state == GaitState::STARTUP_BR) {
            advance = (L / 2.0 + L / 4.0) * dir_sign;
            swing_end = swing_start;
            swing_end.x = swing_start.x + advance;
            swing_end.z = 0.0;
        } else if (target_state == GaitState::WALK) {
            advance = L * dir_sign;
            swing_end = swing_start;
            swing_end.x = swing_start.x + advance;
            swing_end.z = 0.0;
        } else {
            // STOPPING: Recoger pata a posición de reposo
            Vec3 target = pose_rest[active_leg];
            target.z = 0.0;
            swing_end = target;
        }
    }

    // --- TRAMO 1..2 (Swing / Vuelo) ---
    if (seg_a < 2.0 && seg_b > 1.0) {
        double start1 = std::max(seg_a, 1.0);
        double end1 = std::min(seg_b, 2.0);
        
        double delta1 = end1 - start1;
        // En vuelo, el treadmill mueve SOLO las patas que están en el suelo (include_active=false)
        apply_treadmill(delta1, false, treadmill_per_turn, dir_sign, treadmill_on);

        // Acumular cuánto se "movió el piso" para compensar el aterrizaje
        if (treadmill_on && delta1 > 0.0) {
            swing_dx_accum += treadmill_per_turn * (delta1 / 3.0);
        }
        seg_a = end1;
    }

    // Actualizar posición de la pata en vuelo usando Bezier
    if (new_local >= 1.0 && new_local < 2.0 && lift_initialized) {
        double t_swing = new_local - 1.0; // Normalizar 0..1
        
        double min_h_scale = 0.5;
        double h = step_height * (min_h_scale + (1.0 - min_h_scale) * activity_level);
        
        // Usamos la función de la librería bezier_sam.h
        // generar_paso con t <= 1 usa matrix_bezier internamente
        Vec3 p = generar_paso(swing_start, swing_end, h, t_swing);
        
        // Compensación crítica de velocidad
        p.x -= swing_dx_accum;
        feet_state[active_leg] = p;
    }

    // --- TOUCHDOWN (Cruce Fase 2) ---
    bool crossed_into_trans = (old_local < 2.0) && (new_local >= 2.0) && lift_initialized;
    if (crossed_into_trans && !touchdown_done) {
        touchdown_done = true;
        swing_end.z = 0.0;
        Vec3 touchdown = swing_end;
        
        // Aterrizar en el punto compensado para evitar resbalón
        touchdown.x -= swing_dx_accum;
        feet_state[active_leg] = touchdown;
    }

    // --- TRAMO 2..3 (Apoyo Posterior) ---
    if (seg_a < 3.0 && seg_b > 2.0) {
        double start2 = std::max(seg_a, 2.0);
        double end2 = std::min(seg_b, 3.0);
        apply_treadmill(end2 - start2, true, treadmill_per_turn, dir_sign, treadmill_on);
        seg_a = end2;
    }

    // Forzar Z=0 en apoyo post-touchdown para estabilidad numérica
    if (new_local >= 2.0 && lift_initialized) {
        feet_state[active_leg].z = 0.0;
    }

    // -------------------------------
    // 9. Sway (Balanceo del Cuerpo)
    // -------------------------------
    Vec3 body_shift(0,0,0);
    bool sway_enabled = (state == GaitState::STARTUP_FL || state == GaitState::STARTUP_BR || state == GaitState::WALK) && (!stop_requested);
    double sway_gain = sway_enabled ? activity_level : 0.0;

    // Mapa de signos de balanceo (replicado de Python)
    // first = X sign, second = Y sign
    static std::map<std::string, std::pair<double, double>> sway_signs = {
        {"FL", {-1, -1}}, {"BR", {1, 1}}, {"FR", {-1, 1}}, {"BL", {1, -1}}
    };

    std::string next_leg = sequence[(turn_index + 1) % 4];
    auto s_curr = sway_signs[active_leg];
    auto s_next = sway_signs[next_leg];

    Vec3 target_shift_current(sway_x * s_curr.first, sway_y * s_curr.second, 0.0);
    Vec3 target_shift_next(sway_x * s_next.first, sway_y * s_next.second, 0.0);

    // Interpolación del balanceo basada en progreso local
    if (new_local < 1.0) {
        double sub = new_local;
        body_shift = target_shift_current * sub;
    } else if (new_local < 2.0) {
        body_shift = target_shift_current;
    } else {
        double sub = new_local - 2.0;
        // Interpolación lineal entre target actual y siguiente
        body_shift = target_shift_current * (1.0 - sub) + target_shift_next * sub;
    }
    body_shift = body_shift * sway_gain;

    // -------------------------------
    // 10. Fin de Turno & Cambio de Estado
    // -------------------------------
    if (local_progress >= 3.0 - 1e-9) {
        local_progress = 0.0;

        // Lógica de transición de fases
        if (state == GaitState::STARTUP_FL && active_leg == "FL") {
            state = GaitState::STARTUP_BR;
            turn_index = 1; // BR
        } else if (state == GaitState::STARTUP_BR && active_leg == "BR") {
            state = GaitState::WALK;
            turn_index = 2; // FR
        } else {
            // Ciclo normal Walk: siguiente pata
            turn_index = (turn_index + 1) % 4;
        }

        // Check transición a STOPPING
        if (stop_requested && (state == GaitState::WALK || state == GaitState::STARTUP_BR || state == GaitState::STARTUP_FL)) {
            state = GaitState::STOPPING;
        }

        active_leg = sequence[turn_index];
        lift_initialized = false;
        touchdown_done = false;
        swing_dx_accum = 0.0;

        // Verificación final de STOPPING: Si todos los pies están cerca del reposo, pasar a STAND
        if (state == GaitState::STOPPING) {
            bool all_legs_home = true;
            for (auto const& leg : sequence) {
                Vec3 tgt = pose_rest[leg];
                Vec3 p = feet_state[leg];
                if (std::abs(p.x - tgt.x) > 1e-3 || std::abs(p.y - tgt.y) > 1e-3 || std::abs(p.z) > 1e-3) {
                    all_legs_home = false; break;
                }
            }
            if (all_legs_home) {
                state = GaitState::STAND;
                for (auto const& leg : sequence) {
                    feet_state[leg] = pose_rest[leg];
                }
            }
        }
    }

    // -------------------------------
    // 11. Yaw Trim (Rotación In-Place)
    // -------------------------------
    if (std::abs(yaw_trim) > 0.001 && input_mag > cmd_dead && !stop_requested) {
        double c = std::cos(yaw_trim);
        double s = std::sin(yaw_trim);
        for (auto& [leg, pos] : feet_state) {
            double x_new = pos.x * c - pos.y * s;
            double y_new = pos.x * s + pos.y * c;
            pos.x = x_new;
            pos.y = y_new;
        }
    }

    // -------------------------------
    // 12. Construir Salida
    // -------------------------------
    double base_z = stand_height;
    double sway_magnitude = std::abs(body_shift.y + center_y_bias);
    // Corrección Z para mantener el COM estable durante el sway
    double z_correction = -sway_magnitude * 0.3;
    double final_z = base_z + z_correction;

    GaitOutput output;
    output.body_shift = body_shift + Vec3(0.0, center_y_bias, final_z);
    
    // Copiar mapa de pies y aplicar Bias X (Body Bias)
    output.feet_pos = feet_state;
    for (auto& [leg, pos] : output.feet_pos) {
        pos.x -= body_x_bias;
    }
    
    output.body_rot = Vec3(0,0,0); // Placeholder para rotación futura
    return output;
}