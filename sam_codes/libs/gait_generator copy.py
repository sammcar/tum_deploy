import numpy as np
from . import bezier_sam as bz

class GaitGenerator:
    def __init__(self, robot):
        self.robot = robot

        # --- TUNING ---
        self.stand_height = self.robot.stand_height
        # Bajamos un pelín el CG para ganar estabilidad lateral
        self.walking_height = self.robot.stand_height * 0.95

        self.step_height = 0.08  # Altura generosa para evitar tropiezos
        self.step_length_max = 0.055 # Longitud MÁXIMA (se escala dinámicamente)
        
        # Sway (Balanceo)
        self.sway_y = 0.003 
        self.sway_x = 0.001
        
        # --- CORRECCIÓN DINÁMICA DE CENTRO (Anti-Caída de cara) ---
        # Cuanto más rápido vaya, más echamos el cuerpo hacia atrás.
        # -0.02 significa 2cm hacia atrás a máxima velocidad.
        self.dynamic_body_shift_x = -0.02 

        # Estado
        self.activity_level = 0.0
        self.yaw_trim = 0.0

        # Posturas
        self.pose_rest = self._get_rest_pose()
        self.sequence = ["FL", "BR", "FR", "BL"]

        # Estado interno
        self.state = "STAND"
        self.turn_index = 0
        self.local_progress = 0.0
        self.last_t_global = None
        self.stop_requested = False

        self.active_leg = self.sequence[self.turn_index]
        self.swing_start = self.pose_rest[self.active_leg].copy()
        self.swing_end = self.pose_rest[self.active_leg].copy()
        self._lift_initialized = False
        self._touchdown_done = False
        self._swing_dx_accum = 0.0

        self.feet_state = {leg: self.pose_rest[leg].copy() for leg in self.pose_rest}

        # Tiempos (Ligeramente más lento para dar tiempo al control)
        self.turn_T_min = 0.45 
        self.turn_T_max = 1.00
        self.turn_T_stop = 0.60
        self.cmd_dead = 0.01

    def _get_rest_pose(self):
        raw = self.robot.get_default_stance()
        pose = {}
        for leg, pos in raw.items():
            pose[leg] = np.array([pos[0], pos[1], 0.0], dtype=float)
        return pose

    def _s_curve(self, t):
        """ Curva de suavizado para evitar golpes secos (Easing) """
        t = np.clip(t, 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)

    def update(self, t_global, vel_cmd, imu_rpy, modo="Walk"):
        # --- DT ---
        if self.last_t_global is None:
            dt = 0.0
        else:
            dt = float(t_global - self.last_t_global)
            if dt > 0.1: dt = 0.1
        self.last_t_global = float(t_global)

        # --- INPUT ---
        vx = float(vel_cmd[0]) if len(vel_cmd) > 0 else 0.0
        vy = float(vel_cmd[1]) if len(vel_cmd) > 1 else 0.0
        input_mag = float(np.linalg.norm([vx, vy]))
        dir_sign = 1.0 if vx >= 0.0 else -1.0

        # --- ACTIVITY LEVEL (RAMPA SUAVE) ---
        # Controla la transición de estar quieto a moverse.
        target_activity = 1.0 if (input_mag > self.cmd_dead and modo == "Walk") else 0.0
        
        # Subimos lento (evita arranques bruscos) y bajamos un poco más rápido
        accel = 1.5 if target_activity > self.activity_level else 3.0
        
        if self.activity_level < target_activity:
            self.activity_level += accel * dt
        elif self.activity_level > target_activity:
            self.activity_level -= accel * dt
        
        self.activity_level = float(np.clip(self.activity_level, 0.0, 1.0))
        self.stop_requested = (self.activity_level < 0.01 and target_activity == 0.0)

        # --- LONGITUD DE PASO VARIABLE (SOLUCIÓN AL ARRASTRE) ---
        # Si activity es bajo, el paso es corto. Si es alto, el paso es largo.
        # Esto evita que al arrancar el pie intente viajar lejos antes de levantarse.
        L = self.step_length_max * self._s_curve(self.activity_level)

        # --- MAQUINA DE ESTADOS ---
        # Stand -> Startup
        if self.state == "STAND" and self.activity_level > 0.01:
            self.state = "STARTUP_FL"
            self.turn_index = 0
            self.local_progress = 0.0
            self.active_leg = self.sequence[self.turn_index]
            self._lift_initialized = False
            self._touchdown_done = False

        # STAND Congelado (Solo si realmente estamos parados)
        if self.state == "STAND" and self.activity_level <= 0.01:
            body_shift = np.array([0.0, 0.0, self.walking_height], dtype=float)
            final_feet = {l: self.feet_state[l].copy() for l in self.feet_state}
            return final_feet, body_shift, np.array([0.0,0.0,0.0])

        # Duración del ciclo
        turn_T = self.turn_T_max - self.activity_level * (self.turn_T_max - self.turn_T_min)

        # Avance de tiempo local
        if dt > 0.0:
            self.local_progress += dt * (3.0 / turn_T)
            if self.local_progress > 3.0: self.local_progress = 3.0

        # --- TREADMILL (CINTA CORREDORA) ---
        # Importante: El treadmill también escala con L.
        # Si L es pequeño, el cuerpo se mueve despacio, evitando desincronización.
        treadmill_per_turn = (L / 4.0) * dir_sign 
        
        def apply_treadmill(delta_local, include_active):
            if delta_local <= 0.0: return
            dx = treadmill_per_turn * (delta_local / 3.0) 
            for leg in self.feet_state:
                if include_active or (leg != self.active_leg):
                    self.feet_state[leg][0] -= dx

        if not hasattr(self, "_swing_dx_accum"): self._swing_dx_accum = 0.0

        # Fases
        old_local = self.local_progress - (dt * (3.0 / turn_T)) # Reconstruir old
        if old_local < 0: old_local = 0
        new_local = self.local_progress

        seg_a, seg_b = old_local, new_local

        # [Fase 0-1] Pre-swing
        if seg_a < 1.0:
            end0 = min(seg_b, 1.0)
            apply_treadmill(end0 - seg_a, include_active=True)
            seg_a = end0

        # [Transición a Swing] - Inicializar trayectoria
        if (old_local < 1.0) and (new_local >= 1.0):
            self._lift_initialized = True
            self._touchdown_done = False
            self._swing_dx_accum = 0.0
            
            self.swing_start = self.feet_state[self.active_leg].copy()
            self.swing_start[2] = 0.0 

            # Calcular destino relativo al inicio
            if "STARTUP" in self.state:
                # Pasos iniciales más cortos para ganar estabilidad
                adv = (L * 0.5) * dir_sign
            elif self.state == "STOPPING":
                adv = 0.0
            else:
                adv = L * dir_sign

            self.swing_end = self.swing_start.copy()
            self.swing_end[0] += adv
            self.swing_end[2] = 0.0

        # [Fase 1-2] SWING
        if seg_a < 2.0 and seg_b > 1.0:
            start1, end1 = max(seg_a, 1.0), min(seg_b, 2.0)
            delta1 = end1 - start1
            apply_treadmill(delta1, include_active=False)
            self._swing_dx_accum += treadmill_per_turn * (delta1 / 3.0)
            seg_a = end1

        # Actualizar curva Bézier
        if 1.0 <= new_local < 2.0 and self._lift_initialized:
            raw_t = float(new_local - 1.0)
            t_smooth = self._s_curve(raw_t)
            
            # Altura dinámica: baja si vamos lento, alta si vamos rápido
            h = self.step_height * (0.8 + 0.2 * self.activity_level)
            
            p = bz.get_curve_point(self.swing_start, self.swing_end, h, t_smooth)
            p[0] -= self._swing_dx_accum 
            self.feet_state[self.active_leg] = p

        # [Transición Touchdown]
        if (old_local < 2.0) and (new_local >= 2.0) and self._lift_initialized:
            if not self._touchdown_done:
                self._touchdown_done = True
                final_touch = self.swing_end.copy()
                final_touch[0] -= self._swing_dx_accum
                final_touch[2] = 0.0 
                self.feet_state[self.active_leg] = final_touch

        # [Fase 2-3] Post-swing
        if seg_a < 3.0 and seg_b > 2.0:
            start2, end2 = max(seg_a, 2.0), min(seg_b, 3.0)
            apply_treadmill(end2 - start2, include_active=True)
            seg_a = end2
            
        # Asegurar suelo
        if new_local >= 2.0 and self._lift_initialized:
            self.feet_state[self.active_leg][2] = 0.0

        # --- SWAY ---
        body_shift = np.array([0.0, 0.0, 0.0], dtype=float)
        # Aplicamos sway solo si hay actividad suficiente
        sway_gain = self.activity_level 
        
        sway_signs = {"FL": [-1, -1], "BR": [1, 1], "FR": [-1, 1], "BL": [1, -1]}
        curr_s = sway_signs[self.active_leg]
        target_curr = np.array([self.sway_x*curr_s[0], self.sway_y*curr_s[1], 0.0])
        
        # Perfil senoidal simple para el sway (siempre hacia la pata activa)
        # 0..3 -> pico en 1.5
        phase = new_local 
        # Ventana de Hann suavizada para sway: sube en swing, baja en apoyo
        if phase < 0.5: s_env = phase / 0.5
        elif phase < 2.5: s_env = 1.0
        else: s_env = 1.0 - (phase - 2.5)/0.5
        
        body_shift = target_curr * s_env * sway_gain

        # --- CICLO ---
        if self.local_progress >= 3.0 - 1e-9:
            self.local_progress = 0.0
            
            if self.state == "STARTUP_FL": 
                self.state = "STARTUP_BR"; self.turn_index = 1
            elif self.state == "STARTUP_BR":
                self.state = "WALK"; self.turn_index = 2
            else:
                self.turn_index = (self.turn_index + 1) % 4

            if self.activity_level < 0.05: 
                self.state = "STOPPING"
            
            self.active_leg = self.sequence[self.turn_index]
            self._lift_initialized = False
            self._touchdown_done = False
            self._swing_dx_accum = 0.0
            
            if self.state == "STOPPING":
                 # Reset limpio si estamos casi parados
                 self.state = "STAND"
                 self.feet_state = {k:v.copy() for k,v in self.pose_rest.items()}

        # --- SALIDA FINAL CON CORRECCIONES ---
        
        # 1. Corrección dinámica del centro (Anti-Faceplant)
        # Movemos el CUERPO hacia atrás suavemente según velocidad
        shift_x_dynamic = self.dynamic_body_shift_x * self.activity_level
        
        final_shift = body_shift + np.array([shift_x_dynamic, 0.0, self.walking_height])
        
        final_feet = {leg: self.feet_state[leg].copy() for leg in self.feet_state}

        return final_feet, final_shift, np.array([0.0, 0.0, 0.0])