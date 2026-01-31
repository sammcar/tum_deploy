import numpy as np
from . import bezier_sam as bz

class GaitGenerator:
    def __init__(self, robot):
        self.robot = robot

        # --- TUNING GENERAL ---
        self.stand_height = self.robot.stand_height
        self.walking_height = self.robot.stand_height * 0.95
        self.sway_y = 0.003 
        self.sway_x = 0.001

        self.speed_scaler_walk = 1.0   
        self.speed_scaler_trot = 0.4

        self.scale_walk_x = 1.0
        self.scale_walk_y = 1.0
        self.scale_walk_w = 1.0
        
        # --- TUNING CRAWL (Antes Walk) ---
        self.step_height_walk = 0.08
        self.step_length_max_walk = 0.08
        self.dynamic_body_shift_x = -0.02 
        
        # --- TUNING TROT ---
        self.step_height_trot = 0.06      
        self.step_length_max_trot = 0.20  
        self.trot_freq = 2.5           
        self.trot_swing_ratio = 0.5      
        self.swing_gain = 0.5
        self.stance_gain = 1.1
        self.trot_spread_y = 0.0
        self.trot_inertia_gain = 0.03 

        self.scale_trot_x = 0.3  
        self.scale_trot_y = 0.2  
        self.scale_trot_w = 0.15  

        self.trot_inertia_gain = 0.02
        self.smoothed_body_shift_x = 0.0
        self.trot_static_bias_x = 0.005
        self.settle_speed = 0.2

        # --- ESTADO COMPARTIDO ---
        self.activity_level = 0.0
        self.last_t_global = None
        self.cmd_dead = 0.01
        self.pose_rest = self._get_rest_pose()
        self.feet_state = {leg: self.pose_rest[leg].copy() for leg in self.pose_rest}
        self.state = "STAND"
        self.active_mode = "STAND" # Default

        # --- ESTADO CRAWL ---
        self.sequence = ["FL", "BR", "FR", "BL"]
        self.turn_index = 0
        self.local_progress = 0.0
        self.walk_active_leg = self.sequence[0]
        self._walk_lift_init = False
        self._walk_touch_done = False
        self._walk_swing_dx_accum = 0.0
        self.walk_swing_start = self.pose_rest["FL"].copy()
        self.walk_swing_end = self.pose_rest["FL"].copy()

        # --- ESTADO TROT ---
        self.trot_phase = 0.0
        self.trot_swing_mem = {
            leg: {"start": self.pose_rest[leg].copy(), "end": self.pose_rest[leg].copy()} 
            for leg in self.pose_rest
        }
        self.trot_is_swinging = {leg: False for leg in self.pose_rest}

    def _get_rest_pose(self):
        raw = self.robot.get_default_stance()
        pose = {}
        for leg, pos in raw.items():
            pose[leg] = np.array([pos[0], pos[1], 0.0], dtype=float)
        return pose
    
    def _return_to_stand(self, dt):

        final_feet = {}
        moving_flags = [] # Para saber si alguna pata se movió

        for leg in self.feet_state:
            current = self.feet_state[leg]
            target = self.pose_rest[leg]
            
            # Vector error (Hacia donde debe ir)
            diff = target - current
            dist_xy = np.linalg.norm(diff[:2]) # Distancia solo en el plano suelo

            # Si estamos lejos (> 1mm), nos movemos
            if dist_xy > 0.001:
                # Nos movemos hacia el target a velocidad constante
                step = self.settle_speed * dt
                if step > dist_xy: step = dist_xy 
                
                # Interpolación Lineal (Lerp) para X e Y
                direction = diff[:2] / dist_xy
                new_x = current[0] + direction[0] * step
                new_y = current[1] + direction[1] * step
                z_lift = 0.01 # 1cm de altura de seguridad
                
                final_feet[leg] = np.array([new_x, new_y, z_lift])
                moving_flags.append(True)
            else:
                # Si ya llegamos (o estamos muy cerca), aseguramos posición exacta y Z=0
                final_feet[leg] = np.array([target[0], target[1], 0.0])
                moving_flags.append(False)
        
        # Actualizamos el estado
        self.feet_state = final_feet
        
        # Cuerpo estático centrado
        # Lerp suave del cuerpo hacia 0 (por si veníamos con inercia)
        self.smoothed_body_shift_x += (0.0 - self.smoothed_body_shift_x) * 5.0 * dt
        final_shift = np.array([self.smoothed_body_shift_x, 0.0, self.walking_height])
        
        # Devolvemos True si alguna pata se movió (para saber si seguimos "acomodando")
        is_still_settling = any(moving_flags) or (abs(self.smoothed_body_shift_x) > 0.001)
        
        return final_feet, final_shift, np.array([0., 0., 0.]), is_still_settling

    def _s_curve(self, t):
        t = np.clip(t, 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)
    
    def update(self, t_global, vel_cmd, imu_rpy, modo="STAND", freq=2.5):
        # 1. DT
        if self.last_t_global is None: dt = 0.0
        else:
            dt = float(t_global - self.last_t_global)
            if dt > 0.1: dt = 0.1 
        self.last_t_global = float(t_global)

        # 2. Comandos
        raw_vx = float(vel_cmd[0]) if len(vel_cmd) > 0 else 0.0
        raw_vy = float(vel_cmd[1]) if len(vel_cmd) > 1 else 0.0
        raw_w  = float(vel_cmd[2]) if len(vel_cmd) > 2 else 0.0 

        # 3. Scalers
        if modo == "TROT":
            vx_trot = self.scale_trot_x if raw_vx >= 0 else self.scale_trot_x 
            vx_scaled = raw_vx * vx_trot
            vy_scaled = raw_vy * self.scale_trot_y
            w_scaled  = raw_w  * self.scale_trot_w
        else:
            vx_scaled = raw_vx * self.scale_walk_x
            vy_scaled = raw_vy * self.scale_walk_y
            w_scaled  = raw_w  * self.scale_walk_w

        # 4. Actividad
        input_mag = float(np.linalg.norm([raw_vx, raw_vy, raw_w]))
        
        if modo == "TROT": target_activity = 1.0
        elif modo == "CRAWL": target_activity = 1.0 if input_mag > self.cmd_dead else 0.0
        else: target_activity = 0.0

        # Rampa Actividad
        accel = 1.5 if target_activity > self.activity_level else 2.0 
        if self.activity_level < target_activity: self.activity_level += accel * dt
        elif self.activity_level > target_activity: self.activity_level -= accel * dt
        self.activity_level = float(np.clip(self.activity_level, 0.0, 1.0))

        if modo != self.active_mode: self.active_mode = modo

        # 5. LÓGICA DE TRANSICIONES
        is_still_trotting = (self.activity_level > 0.01)

        # A) MODO TROT (o inercia de trote)
        if modo == "TROT" or (modo == "STAND" and is_still_trotting):
            return self._update_trot(dt, vx_scaled, vy_scaled, w_scaled, self.activity_level, freq)
        
        # B) MODO STAND (Acomodación / Settling)
        # Si ya paramos de trotar, pero estamos en STAND, llamamos a la acomodación
        elif modo == "STAND":
            feet, shift, rot, _ = self._return_to_stand(dt)
            return feet, shift, rot
            
        # C) MODO CRAWL
        else:
            return self._update_crawl(dt, vx_scaled, vy_scaled, self.activity_level)

    # =========================================================================
    # LÓGICA DE CRAWL (Antiguo Walk)
    # =========================================================================
    def _update_crawl(self, dt, vx, vy, activity):
        dir_sign = 1.0 if vx >= 0.0 else -1.0
        L = self.step_length_max_walk * self._s_curve(activity)

        # Stand Congelado
        if self.state == "STAND" and activity <= 0.01:
            return {l: self.feet_state[l].copy() for l in self.feet_state}, \
                   np.array([0.0, 0.0, self.walking_height]), np.array([0.,0.,0.])

        # Transición Stand -> Startup
        if self.state == "STAND" and activity > 0.01:
            self.state = "STARTUP_FL"
            self.turn_index = 0
            self.local_progress = 0.0
            self.walk_active_leg = self.sequence[self.turn_index]
            self._walk_lift_init = False
            self._walk_touch_done = False

        # Tiempos
        turn_T_min, turn_T_max = 0.45, 1.00
        turn_T = turn_T_max - activity * (turn_T_max - turn_T_min)

        # Avance local
        if dt > 0.0:
            self.local_progress += dt * (3.0 / turn_T)
            if self.local_progress > 3.0: self.local_progress = 3.0

        # Treadmill
        treadmill_per_turn = (L / 4.0) * dir_sign
        
        def apply_treadmill(delta_local, include_active):
            if delta_local <= 0.0: return
            dx = treadmill_per_turn * (delta_local / 3.0)
            for leg in self.feet_state:
                if include_active or (leg != self.walk_active_leg):
                    self.feet_state[leg][0] -= dx

        old_local = self.local_progress - (dt * (3.0 / turn_T))
        if old_local < 0: old_local = 0
        new_local = self.local_progress
        seg_a, seg_b = old_local, new_local

        # Fases Walk
        if seg_a < 1.0: # Pre-swing
            end0 = min(seg_b, 1.0)
            apply_treadmill(end0 - seg_a, True)
            seg_a = end0

        if (old_local < 1.0) and (new_local >= 1.0): # Init Swing
            self._walk_lift_init = True
            self._walk_touch_done = False
            self._walk_swing_dx_accum = 0.0
            self.walk_swing_start = self.feet_state[self.walk_active_leg].copy()
            self.walk_swing_start[2] = 0.0
            adv = (L * 0.5 * dir_sign) if "STARTUP" in self.state else (L * dir_sign)
            if self.state == "STOPPING": adv = 0.0
            self.walk_swing_end = self.walk_swing_start.copy()
            self.walk_swing_end[0] += adv
            self.walk_swing_end[2] = 0.0

        if seg_a < 2.0 and seg_b > 1.0: # Swing
            start1, end1 = max(seg_a, 1.0), min(seg_b, 2.0)
            delta1 = end1 - start1
            apply_treadmill(delta1, False)
            self._walk_swing_dx_accum += treadmill_per_turn * (delta1 / 3.0)
            seg_a = end1

        if 1.0 <= new_local < 2.0 and self._walk_lift_init: # Bezier
            raw_t = float(new_local - 1.0)
            t_smooth = self._s_curve(raw_t)
            h = self.step_height_walk * (0.8 + 0.2 * activity)
            p = bz.get_curve_point(self.walk_swing_start, self.walk_swing_end, h, t_smooth)
            p[0] -= self._walk_swing_dx_accum
            self.feet_state[self.walk_active_leg] = p

        if (old_local < 2.0) and (new_local >= 2.0) and self._walk_lift_init: # Touchdown
            if not self._walk_touch_done:
                self._walk_touch_done = True
                final_touch = self.walk_swing_end.copy()
                final_touch[0] -= self._walk_swing_dx_accum
                final_touch[2] = 0.0
                self.feet_state[self.walk_active_leg] = final_touch

        if seg_a < 3.0 and seg_b > 2.0: # Post-swing
            start2, end2 = max(seg_a, 2.0), min(seg_b, 3.0)
            apply_treadmill(end2 - start2, True)
        
        # Reset ciclo
        if self.local_progress >= 3.0 - 1e-9:
            self.local_progress = 0.0
            if self.state == "STARTUP_FL": self.state = "STARTUP_BR"; self.turn_index = 1
            elif self.state == "STARTUP_BR": self.state = "WALK"; self.turn_index = 2
            else: self.turn_index = (self.turn_index + 1) % 4
            
            if activity < 0.05: self.state = "STOPPING"
            self.walk_active_leg = self.sequence[self.turn_index]
            self._walk_lift_init = False
            self._walk_touch_done = False
            self._walk_swing_dx_accum = 0.0
            if self.state == "STOPPING":
                self.state = "STAND"
                self.feet_state = {k:v.copy() for k,v in self.pose_rest.items()}

        sway_gain = activity
        sway_signs = {"FL": [-1, -1], "BR": [1, 1], "FR": [-1, 1], "BL": [1, -1]}
        curr_s = sway_signs[self.walk_active_leg]
        target_curr = np.array([self.sway_x*curr_s[0], self.sway_y*curr_s[1], 0.0])
        phase = new_local
        if phase < 0.5: s_env = phase / 0.5
        elif phase < 2.5: s_env = 1.0
        else: s_env = 1.0 - (phase - 2.5)/0.5
        body_shift = target_curr * s_env * sway_gain

        shift_x_dynamic = self.dynamic_body_shift_x * dir_sign * activity
        final_shift = body_shift + np.array([shift_x_dynamic, 0.0, self.walking_height])
        return {l: self.feet_state[l].copy() for l in self.feet_state}, final_shift, np.array([0.,0.,0.])

    # =========================================================================
    # LÓGICA DE TROT
    # =========================================================================
    def _update_trot(self, dt, vx, vy, wz, activity, freq):
        
        # 1. AVANCE DE FASE
        self.trot_phase += dt * freq
        if self.trot_phase > 1.0: self.trot_phase -= 1.0

        diagonals = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}
        swing_ratio = self.trot_swing_ratio 

        # 2. VECTORES DE MOVIMIENTO LINEAL (Base)
        # (Esto es lo que ya tenías)
        swing_base_x = vx * (1.0/freq) * activity * self.swing_gain
        swing_base_y = vy * (1.0/freq) * activity * self.swing_gain
        
        stance_base_dx = vx * dt * self.stance_gain
        stance_base_dy = vy * dt * self.stance_gain

        for leg, offset in diagonals.items():
            # Posición de reposo de esta pata (Radio de giro)
            rx, ry, _ = self.pose_rest[leg]

            # 3. CÁLCULO DE GIRO (Tangencial)
            # Fórmula: Vx = -w*y, Vy = w*x
            # Calculamos cuánto debe moverse la pata para crear el giro wz
            
            # Aporte al Swing (Hacia dónde lanzar la pata)
            rot_swing_x = (-wz * ry) * (1.0/freq) * activity * self.swing_gain
            rot_swing_y = ( wz * rx) * (1.0/freq) * activity * self.swing_gain
            
            # Aporte al Stance (Hacia dónde mover el suelo)
            rot_stance_dx = (-wz * ry) * dt * self.stance_gain
            rot_stance_dy = ( wz * rx) * dt * self.stance_gain

            # SUMA FINAL (Lineal + Rotación)
            total_swing_x = swing_base_x + rot_swing_x
            total_swing_y = swing_base_y + rot_swing_y
            
            total_stance_dx = stance_base_dx + rot_stance_dx
            total_stance_dy = stance_base_dy + rot_stance_dy

            # --- CICLO DE MARCHA ---
            phi = (self.trot_phase + offset) % 1.0
            side_sign = np.sign(ry)
            extra_width = self.trot_spread_y * side_sign 

            is_swing_time = phi < swing_ratio

            # Gatekeeper
            if is_swing_time and (activity > 0.01 or self.trot_is_swinging[leg]):
                
                # SWING
                swing_progress = phi / swing_ratio
                t_smooth = self._s_curve(swing_progress)

                if not self.trot_is_swinging[leg]:
                    self.trot_is_swinging[leg] = True
                    start_pos = self.feet_state[leg].copy()
                    start_pos[2] = 0.0
                    
                    rest_pos = self.pose_rest[leg]
                    end_pos = rest_pos.copy()
                    
                    # Usamos los totales (Lineal + Rotación)
                    end_pos[0] += total_swing_x 
                    end_pos[1] += total_swing_y + extra_width 
                    
                    self.trot_swing_mem[leg]["start"] = start_pos
                    self.trot_swing_mem[leg]["end"] = end_pos

                p_start = self.trot_swing_mem[leg]["start"]
                p_end = self.trot_swing_mem[leg]["end"]
                
                h = self.step_height_trot
                if activity < 0.5: h *= (0.5 + activity)

                new_pos = bz.get_curve_point(p_start, p_end, h, t_smooth)
                self.feet_state[leg] = new_pos
                
            else:
                # STANCE
                self.trot_is_swinging[leg] = False
                
                # Usamos los totales (Lineal + Rotación)
                self.feet_state[leg][0] -= total_stance_dx
                self.feet_state[leg][1] -= total_stance_dy
                
                # --- CORRECCIÓN DE CENTRADO ---
                # Detectamos si hay comando de movimiento (Lineal O Angular)
                lin_mag = np.linalg.norm([vx, vy])
                rot_mag = abs(wz) # Magnitud del giro
                
                # Solo centramos si NO hay velocidad lineal NI rotacional
                # Si rotamos en el sitio (lin=0, rot>0), NO entramos aquí, 
                # porque el resorte pelearía contra el giro.
                if lin_mag < 0.05 and rot_mag < 0.05 and activity > 0.01:
                    
                    target_x = self.pose_rest[leg][0]
                    current_x = self.feet_state[leg][0]
                    self.feet_state[leg][0] += (target_x - current_x) * 0.1
                    
                    target_y = self.pose_rest[leg][1] + extra_width
                    current_y = self.feet_state[leg][1]
                    self.feet_state[leg][1] += (target_y - current_y) * 0.1
                
                elif activity > 0.01:
                    # En movimiento, solo corrección suave en Y
                    target_y = self.pose_rest[leg][1] + extra_width
                    current_y = self.feet_state[leg][1]
                    self.feet_state[leg][1] += (target_y - current_y) * 0.02

                self.feet_state[leg][2] = 0.0

        # 3. SECUENCIA DE PARADA (Safety Freeze)
        any_leg_in_air = any(self.trot_is_swinging.values())
        if activity <= 0.01 and not any_leg_in_air:
            final_feet = {l: self.feet_state[l].copy() for l in self.feet_state}
            for l in final_feet: final_feet[l][2] = 0.0
            final_shift = np.array([0.0, 0.0, self.walking_height])
            return final_feet, final_shift, np.array([0., 0., 0.])

        # 4. SALIDA FINAL
        bounce_amp = 0.005 * activity
        bounce = np.sin(self.trot_phase * 2 * np.pi * 2) * bounce_amp
        
        target_shift_x = vx * self.trot_inertia_gain * activity
        lerp_speed = 1.0 
        self.smoothed_body_shift_x += (target_shift_x - self.smoothed_body_shift_x) * lerp_speed * dt
        
        body_shift = np.array([self.smoothed_body_shift_x, 0.0, self.walking_height + bounce])

        return {l: self.feet_state[l].copy() for l in self.feet_state}, \
               body_shift, \
               np.array([0.0, 0.0, wz])
