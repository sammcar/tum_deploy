#ifndef NEW_GAIT_GENERATOR_HPP
#define NEW_GAIT_GENERATOR_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>

// Estructura para empaquetar la salida del generador
struct GaitOutput {
    Eigen::Vector3d feet[4];
    Eigen::Vector3d body_shift;
    Eigen::Vector3d body_rot;
    bool is_settling;
};

class NewGaitGenerator {
public:
    // --- TUNING GENERAL ---
    double stand_height;
    double walking_height;
    double sway_y = 0.003;
    double sway_x = 0.001;

    double scale_walk_x = 1.0;
    double scale_walk_y = 1.0;
    double scale_walk_w = 1.0;
    
    // --- TUNING CRAWL ---
    double step_height_walk = 0.08;
    double step_length_max_walk = 0.08;
    double dynamic_body_shift_x = -0.02;

    // --- TUNING JUMP (PRONKING) ---
    double step_height_jump = 0.1;
    double jump_freq = 3.0;
    double jump_swing_ratio = 0.4;
    double jump_spread_y = -0.02;
    
    // --- TUNING TROT ---
    double step_height_trot = 0.06;
    double step_length_max_trot = 0.20;
    double trot_freq = 2.5;
    double trot_swing_ratio = 0.5;
    double swing_gain = 0.5;
    double stance_gain = 1.1;
    double trot_spread_y = 0.0;
    double trot_inertia_gain = 0.02; 

    double scale_trot_x = 0.3;
    double scale_trot_y = 0.2;
    double scale_trot_w = 0.15;

    double smoothed_body_shift_x = 0.0;
    double trot_static_bias_x = 0.005;
    double settle_speed = 0.2;

    // --- ESTADO COMPARTIDO ---
    double activity_level = 0.0;
    double last_t_global = -1.0;
    double cmd_dead = 0.01;
    
    Eigen::Vector3d pose_rest[4];
    Eigen::Vector3d feet_state[4];
    
    std::string state = "STAND";
    std::string active_mode = "STAND";

    // --- ESTADO CRAWL ---
    int sequence[4] = {0, 3, 1, 2}; // FL(0), BR(3), FR(1), BL(2)
    int turn_index = 0;
    double local_progress = 0.0;
    int walk_active_leg = 0;
    bool _walk_lift_init = false;
    bool _walk_touch_done = false;
    double _walk_swing_dx_accum = 0.0;
    Eigen::Vector3d walk_swing_start;
    Eigen::Vector3d walk_swing_end;

    // --- ESTADO TROT / JUMP ---
    double trot_phase = 0.0;
    struct SwingMem { Eigen::Vector3d start; Eigen::Vector3d end; };
    SwingMem trot_swing_mem[4];
    bool trot_is_swinging[4] = {false, false, false, false};

    NewGaitGenerator(double default_stand_height, const Eigen::Vector3d default_stances[4]) {
        stand_height = default_stand_height;
        walking_height = stand_height * 0.95;

        for (int i = 0; i < 4; i++) {
            // En el código Python original extraes X y Y pero fuerzas Z=0.0
            pose_rest[i] = Eigen::Vector3d(default_stances[i].x(), default_stances[i].y(), 0.0);
            feet_state[i] = pose_rest[i];
            
            trot_swing_mem[i].start = pose_rest[i];
            trot_swing_mem[i].end = pose_rest[i];
        }
        walk_swing_start = pose_rest[0];
        walk_swing_end = pose_rest[0];
    }

    GaitOutput update(double t_global, const Eigen::Vector3d& vel_cmd, const Eigen::Vector3d& imu_rpy, std::string modo, double freq) {
        // 1. DT
        double dt = 0.0;
        if (last_t_global >= 0.0) {
            dt = t_global - last_t_global;
            if (dt > 0.1) dt = 0.1;
        }
        last_t_global = t_global;

        // 2. Comandos
        double raw_vx = vel_cmd.x();
        double raw_vy = vel_cmd.y();
        double raw_w  = vel_cmd.z();

        // 3. Scalers
        double vx_scaled = 0, vy_scaled = 0, w_scaled = 0;
        if (modo == "TROT") {
            double vx_trot = (raw_vx >= 0) ? scale_trot_x : scale_trot_x;
            vx_scaled = raw_vx * vx_trot;
            vy_scaled = raw_vy * scale_trot_y;
            w_scaled  = raw_w  * scale_trot_w;
        } else if (modo == "JUMP") {
            vx_scaled = 0.0; vy_scaled = 0.0; w_scaled = 0.0;
        } else {
            vx_scaled = raw_vx * scale_walk_x;
            vy_scaled = raw_vy * scale_walk_y;
            w_scaled  = raw_w  * scale_walk_w;
        }

        // 4. Actividad
        double input_mag = std::sqrt(raw_vx*raw_vx + raw_vy*raw_vy + raw_w*raw_w);
        double target_activity = 0.0;
        
        if (modo == "TROT" || modo == "JUMP") target_activity = 1.0;
        else if (modo == "CRAWL") target_activity = (input_mag > cmd_dead) ? 1.0 : 0.0;

        // Rampa de Actividad
        double accel = (target_activity > activity_level) ? 1.5 : 2.0;
        if (activity_level < target_activity) activity_level += accel * dt;
        else if (activity_level > target_activity) activity_level -= accel * dt;
        activity_level = std::clamp(activity_level, 0.0, 1.0);

        if (modo != active_mode) active_mode = modo;

        bool is_still_trotting = (activity_level > 0.01);

        // 5. MÁQUINA DE ESTADOS
        if (modo == "TROT" || (modo == "STAND" && is_still_trotting)) {
            return _update_trot(dt, vx_scaled, vy_scaled, w_scaled, activity_level, freq);
        } else if (modo == "JUMP") {
            return _update_jump(dt, activity_level, jump_freq);
        } else if (modo == "STAND") {
            return _return_to_stand(dt);
        } else {
            return _update_crawl(dt, vx_scaled, vy_scaled, activity_level);
        }
    }

private:
    double _s_curve(double t) {
        t = std::clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }

    // Bézier Cúbica idéntica a bezier_sam.py
    Eigen::Vector3d _get_curve_point(const Eigen::Vector3d& p_start, const Eigen::Vector3d& p_end, double h, double t) {
        Eigen::Vector3d p1 = p_start; p1.z() += h * 1.5;
        Eigen::Vector3d p2 = p_end;   p2.z() += h * 1.5;
        double u = 1.0 - t;
        return (u*u*u)*p_start + 3*(u*u)*t*p1 + 3*u*(t*t)*p2 + (t*t*t)*p_end;
    }

    GaitOutput _return_to_stand(double dt) {
        GaitOutput out;
        bool moving_flags = false;

        for (int leg = 0; leg < 4; leg++) {
            Eigen::Vector3d current = feet_state[leg];
            Eigen::Vector3d target = pose_rest[leg];
            
            Eigen::Vector2d diff(target.x() - current.x(), target.y() - current.y());
            double dist_xy = diff.norm();

            if (dist_xy > 0.001) {
                double step = settle_speed * dt;
                if (step > dist_xy) step = dist_xy;
                
                Eigen::Vector2d dir = diff / dist_xy;
                out.feet[leg] = Eigen::Vector3d(current.x() + dir.x() * step, 
                                                current.y() + dir.y() * step, 
                                                0.01); // 1cm lift de seguridad
                moving_flags = true;
            } else {
                out.feet[leg] = Eigen::Vector3d(target.x(), target.y(), 0.0);
            }
            feet_state[leg] = out.feet[leg];
        }
        
        smoothed_body_shift_x += (0.0 - smoothed_body_shift_x) * 5.0 * dt;
        out.body_shift = Eigen::Vector3d(smoothed_body_shift_x, 0.0, walking_height);
        out.body_rot = Eigen::Vector3d::Zero();
        
        out.is_settling = moving_flags || (std::abs(smoothed_body_shift_x) > 0.001);
        return out;
    }

    GaitOutput _update_crawl(double dt, double vx, double vy, double activity) {
        GaitOutput out;
        double dir_sign = (vx >= 0.0) ? 1.0 : -1.0;
        double L = step_length_max_walk * _s_curve(activity);

        if (state == "STAND" && activity <= 0.01) {
            for(int i=0; i<4; i++) out.feet[i] = feet_state[i];
            out.body_shift = Eigen::Vector3d(0.0, 0.0, walking_height);
            out.body_rot = Eigen::Vector3d::Zero();
            return out;
        }

        if (state == "STAND" && activity > 0.01) {
            state = "STARTUP_FL";
            turn_index = 0;
            local_progress = 0.0;
            walk_active_leg = sequence[turn_index];
            _walk_lift_init = false;
            _walk_touch_done = false;
        }

        double turn_T_min = 0.45, turn_T_max = 1.00;
        double turn_T = turn_T_max - activity * (turn_T_max - turn_T_min);

        if (dt > 0.0) {
            local_progress += dt * (3.0 / turn_T);
            if (local_progress > 3.0) local_progress = 3.0;
        }

        double treadmill_per_turn = (L / 4.0) * dir_sign;

        auto apply_treadmill = [&](double delta_local, bool include_active) {
            if (delta_local <= 0.0) return;
            double dx = treadmill_per_turn * (delta_local / 3.0);
            for (int leg = 0; leg < 4; leg++) {
                if (include_active || (leg != walk_active_leg)) {
                    feet_state[leg].x() -= dx;
                }
            }
        };

        double old_local = std::max(0.0, local_progress - (dt * (3.0 / turn_T)));
        double new_local = local_progress;
        double seg_a = old_local, seg_b = new_local;

        // Fases Walk
        if (seg_a < 1.0) {
            double end0 = std::min(seg_b, 1.0);
            apply_treadmill(end0 - seg_a, true);
            seg_a = end0;
        }

        if ((old_local < 1.0) && (new_local >= 1.0)) {
            _walk_lift_init = true;
            _walk_touch_done = false;
            _walk_swing_dx_accum = 0.0;
            walk_swing_start = feet_state[walk_active_leg];
            walk_swing_start.z() = 0.0;
            
            double adv = (state.find("STARTUP") != std::string::npos) ? (L * 0.5 * dir_sign) : (L * dir_sign);
            if (state == "STOPPING") adv = 0.0;
            walk_swing_end = walk_swing_start;
            walk_swing_end.x() += adv;
            walk_swing_end.z() = 0.0;
        }

        if (seg_a < 2.0 && seg_b > 1.0) {
            double start1 = std::max(seg_a, 1.0);
            double end1 = std::min(seg_b, 2.0);
            double delta1 = end1 - start1;
            apply_treadmill(delta1, false);
            _walk_swing_dx_accum += treadmill_per_turn * (delta1 / 3.0);
            seg_a = end1;
        }

        if (new_local >= 1.0 && new_local < 2.0 && _walk_lift_init) {
            double raw_t = new_local - 1.0;
            double t_smooth = _s_curve(raw_t);
            double h = step_height_walk * (0.8 + 0.2 * activity);
            Eigen::Vector3d p = _get_curve_point(walk_swing_start, walk_swing_end, h, t_smooth);
            p.x() -= _walk_swing_dx_accum;
            feet_state[walk_active_leg] = p;
        }

        if ((old_local < 2.0) && (new_local >= 2.0) && _walk_lift_init) {
            if (!_walk_touch_done) {
                _walk_touch_done = true;
                Eigen::Vector3d final_touch = walk_swing_end;
                final_touch.x() -= _walk_swing_dx_accum;
                final_touch.z() = 0.0;
                feet_state[walk_active_leg] = final_touch;
            }
        }

        if (seg_a < 3.0 && seg_b > 2.0) {
            double start2 = std::max(seg_a, 2.0);
            double end2 = std::min(seg_b, 3.0);
            apply_treadmill(end2 - start2, true);
        }

        if (local_progress >= 3.0 - 1e-9) {
            local_progress = 0.0;
            if (state == "STARTUP_FL") { state = "STARTUP_BR"; turn_index = 1; }
            else if (state == "STARTUP_BR") { state = "WALK"; turn_index = 2; }
            else turn_index = (turn_index + 1) % 4;
            
            if (activity < 0.05) state = "STOPPING";
            walk_active_leg = sequence[turn_index];
            _walk_lift_init = false;
            _walk_touch_done = false;
            _walk_swing_dx_accum = 0.0;
            
            if (state == "STOPPING") {
                state = "STAND";
                for(int i=0; i<4; i++) feet_state[i] = pose_rest[i];
            }
        }

        double sway_gain = activity;
        Eigen::Vector2d sway_signs[4] = {{-1,-1}, {-1,1}, {1,-1}, {1,1}}; // FL, FR, BL, BR
        Eigen::Vector2d curr_s = sway_signs[walk_active_leg];
        Eigen::Vector3d target_curr(sway_x * curr_s.x(), sway_y * curr_s.y(), 0.0);
        
        double phase = new_local;
        double s_env = 1.0;
        if (phase < 0.5) s_env = phase / 0.5;
        else if (phase >= 2.5) s_env = 1.0 - (phase - 2.5)/0.5;
        
        Eigen::Vector3d body_shift = target_curr * s_env * sway_gain;
        double shift_x_dynamic = dynamic_body_shift_x * dir_sign * activity;
        
        for(int i=0; i<4; i++) out.feet[i] = feet_state[i];
        out.body_shift = body_shift + Eigen::Vector3d(shift_x_dynamic, 0.0, walking_height);
        out.body_rot = Eigen::Vector3d::Zero();
        
        return out;
    }

    GaitOutput _update_trot(double dt, double vx, double vy, double wz, double activity, double freq) {
        GaitOutput out;
        trot_phase += dt * freq;
        if (trot_phase > 1.0) trot_phase -= 1.0;

        double diagonals[4] = {0.0, 0.5, 0.5, 0.0}; // FL(0), FR(1), BL(2), BR(3)
        
        double swing_base_x = vx * (1.0/freq) * activity * swing_gain;
        double swing_base_y = vy * (1.0/freq) * activity * swing_gain;
        double stance_base_dx = vx * dt * stance_gain;
        double stance_base_dy = vy * dt * stance_gain;

        for (int leg = 0; leg < 4; leg++) {
            double rx = pose_rest[leg].x();
            double ry = pose_rest[leg].y();

            double rot_swing_x = (-wz * ry) * (1.0/freq) * activity * swing_gain;
            double rot_swing_y = ( wz * rx) * (1.0/freq) * activity * swing_gain;
            
            double rot_stance_dx = (-wz * ry) * dt * stance_gain;
            double rot_stance_dy = ( wz * rx) * dt * stance_gain;

            double total_swing_x = swing_base_x + rot_swing_x;
            double total_swing_y = swing_base_y + rot_swing_y;
            
            double total_stance_dx = stance_base_dx + rot_stance_dx;
            double total_stance_dy = stance_base_dy + rot_stance_dy;

            double phi = std::fmod(trot_phase + diagonals[leg], 1.0);
            double side_sign = (ry >= 0) ? 1.0 : -1.0;
            double extra_width = trot_spread_y * side_sign;

            bool is_swing_time = (phi < trot_swing_ratio);

            if (is_swing_time && (activity > 0.01 || trot_is_swinging[leg])) {
                // SWING
                double swing_progress = phi / trot_swing_ratio;
                double t_smooth = _s_curve(swing_progress);

                if (!trot_is_swinging[leg]) {
                    trot_is_swinging[leg] = true;
                    Eigen::Vector3d start_pos = feet_state[leg];
                    start_pos.z() = 0.0;
                    
                    Eigen::Vector3d end_pos = pose_rest[leg];
                    end_pos.x() += total_swing_x;
                    end_pos.y() += total_swing_y + extra_width;
                    
                    trot_swing_mem[leg].start = start_pos;
                    trot_swing_mem[leg].end = end_pos;
                }

                double h = step_height_trot;
                if (activity < 0.5) h *= (0.5 + activity);

                feet_state[leg] = _get_curve_point(trot_swing_mem[leg].start, trot_swing_mem[leg].end, h, t_smooth);
            } else {
                // STANCE
                trot_is_swinging[leg] = false;
                feet_state[leg].x() -= total_stance_dx;
                feet_state[leg].y() -= total_stance_dy;
                
                double lin_mag = std::sqrt(vx*vx + vy*vy);
                double rot_mag = std::abs(wz);
                
                if (lin_mag < 0.05 && rot_mag < 0.05 && activity > 0.01) {
                    feet_state[leg].x() += (pose_rest[leg].x() - feet_state[leg].x()) * 0.1;
                    feet_state[leg].y() += (pose_rest[leg].y() + extra_width - feet_state[leg].y()) * 0.1;
                } else if (activity > 0.01) {
                    feet_state[leg].y() += (pose_rest[leg].y() + extra_width - feet_state[leg].y()) * 0.02;
                }
                feet_state[leg].z() = 0.0;
            }
        }

        // SEGURIDAD DE PARADA
        bool any_leg_in_air = (trot_is_swinging[0] || trot_is_swinging[1] || trot_is_swinging[2] || trot_is_swinging[3]);
        if (activity <= 0.01 && !any_leg_in_air) {
            for(int i=0; i<4; i++) {
                out.feet[i] = feet_state[i];
                out.feet[i].z() = 0.0;
            }
            out.body_shift = Eigen::Vector3d(0.0, 0.0, walking_height);
            out.body_rot = Eigen::Vector3d::Zero();
            return out;
        }

        double bounce_amp = 0.005 * activity;
        double bounce = std::sin(trot_phase * 2 * M_PI * 2) * bounce_amp;
        
        double target_shift_x = vx * trot_inertia_gain * activity;
        smoothed_body_shift_x += (target_shift_x - smoothed_body_shift_x) * 1.0 * dt;
        
        for(int i=0; i<4; i++) out.feet[i] = feet_state[i];
        out.body_shift = Eigen::Vector3d(smoothed_body_shift_x, 0.0, walking_height + bounce);
        out.body_rot = Eigen::Vector3d(0.0, 0.0, wz);
        
        return out;
    }

    GaitOutput _update_jump(double dt, double activity, double freq) {
        GaitOutput out;
        trot_phase += dt * freq;
        if (trot_phase > 1.0) trot_phase -= 1.0;

        bool any_leg_in_air = (trot_is_swinging[0] || trot_is_swinging[1] || trot_is_swinging[2] || trot_is_swinging[3]);

        for (int leg = 0; leg < 4; leg++) {
            double phi = std::fmod(trot_phase, 1.0); // Offset 0.0 para todas
            bool is_swing_time = (phi < jump_swing_ratio);
            
            double fixed_x = pose_rest[leg].x();
            double side_sign = (pose_rest[leg].y() >= 0) ? 1.0 : -1.0;
            double fixed_y = pose_rest[leg].y() + (jump_spread_y * side_sign);

            if (is_swing_time && (activity > 0.01 || trot_is_swinging[leg])) {
                double swing_progress = phi / jump_swing_ratio;
                double t_smooth = _s_curve(swing_progress);

                if (!trot_is_swinging[leg]) {
                    trot_is_swinging[leg] = true;
                    Eigen::Vector3d pos(fixed_x, fixed_y, 0.0);
                    trot_swing_mem[leg].start = pos;
                    trot_swing_mem[leg].end = pos;
                }

                double h = step_height_jump * activity;
                Eigen::Vector3d new_pos = _get_curve_point(trot_swing_mem[leg].start, trot_swing_mem[leg].end, h, t_smooth);
                
                new_pos.x() = fixed_x;
                new_pos.y() = fixed_y;
                feet_state[leg] = new_pos;
            } else {
                trot_is_swinging[leg] = false;
                feet_state[leg].x() = fixed_x;
                feet_state[leg].y() = fixed_y;
                feet_state[leg].z() = 0.0;
            }
        }

        if (activity <= 0.01 && !any_leg_in_air) {
            for(int i=0; i<4; i++) {
                out.feet[i] = feet_state[i];
                out.feet[i].z() = 0.0;
            }
            out.body_shift = Eigen::Vector3d(0.0, 0.0, walking_height);
            out.body_rot = Eigen::Vector3d::Zero();
            return out;
        }

        double body_bounce_amp = 0.03 * activity;
        double bounce_z = std::sin((trot_phase + 0.25) * 2 * M_PI) * body_bounce_amp;
        
        for(int i=0; i<4; i++) out.feet[i] = feet_state[i];
        out.body_shift = Eigen::Vector3d(0.0, 0.0, walking_height + bounce_z);
        out.body_rot = Eigen::Vector3d::Zero();
        
        return out;
    }
};

#endif // NEW_GAIT_GENERATOR_HPP