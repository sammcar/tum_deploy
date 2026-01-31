import numpy as np

try:
    from . import kinematics_sam as kt
    from .gait_generator import GaitGenerator
    #from .imu_controller import ImuController
except ImportError:
    import kinematics_sam as kt # type: ignore
    from gait_generator import GaitGenerator # type: ignore
    #from imu_controller import ImuController

class QuadrupedRobot:
    def __init__(self, body_length, body_width, l1, l2, l3):
        self.lx, self.ly = body_length / 2.0, body_width / 2.0
        self.l1, self.l2, self.l3 = l1, l2, l3
        
        self.hip_offsets = {
            'FL': np.array([ self.lx,  self.ly, 0]),
            'FR': np.array([ self.lx, -self.ly, 0]),
            'BL': np.array([-self.lx,  self.ly, 0]),
            'BR': np.array([-self.lx, -self.ly, 0])
        }
        self.stand_height = np.sqrt(l2**2 + l3**2)-0.15 #0.05 Más alto
        self.leg_names = ['FL', 'FR', 'BL', 'BR']
        self.leg_types = {name: False for name in self.leg_names}
        self.pies_fijos = None 

        self.brain = GaitGenerator(self)
        # self.imu = ImuController()
        self.last_comp = np.zeros(3)

        self.prev_angles = {}
        for leg in self.leg_names:
            self.prev_angles[leg] = np.array([0.0, 0.0, 0.0])

    def step(self, t, dt, cmd, sensor_data, modo="Trote", freq=2.5):
        # 1. Configurar patas
        for l in self.leg_names: 
            if l in cmd.get('leg_modes', {}): self.leg_types[l] = cmd['leg_modes'][l]

        # 2. Trayectoria de pies (Mundo)
        # Pasamos modo y freq para que el cerebro decida si caminar o trotar
        vel = [cmd['vx'], cmd['vy'], cmd.get('wz', 0.0)]
        feet_pos_gait, body_offset_calc, _ = self.brain.update(t, vel, [0,0,0], modo, freq)
        
        targets = self.pies_fijos if self.pies_fijos else feet_pos_gait
        self.current_targets = targets

        # 3. IMU 
        if cmd.get('use_imu', True):
            # sensor_data ya debe venir en radianes desde el visualizador
            self.last_comp = self.imu.compute_compensation(sensor_data, [0,0,0], dt)
        else:
            self.last_comp = np.zeros(3)

        # 4. Postura del Cuerpo (TODO SUMA, TODO RADIANES)
        # Sumamos offsets dinámicos (como el shift anti-caída en X)
        final_body_x = cmd['body_x'] + body_offset_calc[0]
        final_body_y = cmd['body_y'] + body_offset_calc[1]
        
        # Nota: body_offset_calc[2] trae la altura del gait + el rebote (bounce).
        # Para respetar tu control manual pero añadir el rebote del trote:
        # Calculamos solo la variación (bounce) y se la sumamos a tu comando manual.
        bounce = body_offset_calc[2] - self.brain.walking_height
        final_body_z = cmd['body_z'] + bounce

        body_pos = [final_body_x, final_body_y, final_body_z]
        self.current_body_pos = body_pos
        
        input_rpy = np.array([cmd['body_roll'], cmd['body_pitch'], cmd['body_yaw']])
        body_rpy = input_rpy + self.last_comp
        
        pivote = [cmd.get('piv_x',0), cmd.get('piv_y',0), 0]

        return self.compute_pose(body_pos, body_rpy, pivote, targets)
    
    def compute_pose(self, body_pos, body_rpy, pivote, targets):
        angles = {}
        for name, offset in self.hip_offsets.items():
            # Función original de kinematics_sam.py
            local = kt.origen_a_cadera_local(targets[name], body_rpy, offset, body_pos, pivote)
            raw_q = kt.solve_IK(local[0], local[1], local[2], self.l1, self.l2, self.l3, 'R' in name, self.leg_types[name])
            clean_q = self._unwrap_angles(name, raw_q)
            angles[name] = clean_q
        return angles
    
    def get_default_stance(self):
        """
        Calcula la posición de los pies relative al centro del cuerpo (0,0,0).
        """
        stand_pose = {}
        
        # Ajuste lateral (Y): Abre las patas un poco más que el ancho de hombros
        extra_width = 0.015
        
        # Ajuste longitudinal (X): 
        # Queremos abrir la base de sustentación (delanteras adelante, traseras atrás)
        shift_front = 0.02   # 2 cm hacia adelante del hombro
        shift_rear = -0.04   # 2 cm hacia atrás de la cadera (NEGATIVO)
        
        for leg, hip_off in self.hip_offsets.items():
            # Determinamos si la pata es izquierda (+) o derecha (-) para Y
            signo_y = 1 if hip_off[1] > 0 else -1

            # Lógica para X: Diferenciar Delanteras vs Traseras
            if 'F' in leg:
                current_shift_x = shift_front
            else:
                current_shift_x = shift_rear

            stand_pose[leg] = np.array([
                # X: Posición del hombro/cadera + su desplazamiento específico
                hip_off[0] + current_shift_x, 
                
                # Y: Posición del hombro/cadera + longitud del link 1 + un extra
                hip_off[1] + (signo_y * (self.l1 + extra_width)), 
                
                # Z: Altura deseada (negativa porque el pie está abajo)
                -self.stand_height
            ])
            
        return stand_pose
    
    def _unwrap_angles(self, leg_name, new_angles):
        """
        MÉTODO PRIVADO DE HAL:
        Compara los nuevos ángulos con los anteriores. 
        Si hay un salto de discontinuidad (+/- 2PI), lo corrige suavemente.
        """
        prev = self.prev_angles[leg_name]
        smoothed = np.copy(new_angles)
        
        for i in range(3):
            diff = new_angles[i] - prev[i]
            # Si el salto es mayor a PI (180 deg), asumimos que es un wrap matemático
            if diff > np.pi:
                smoothed[i] -= 2 * np.pi
            elif diff < -np.pi:
                smoothed[i] += 2 * np.pi
                
        # Actualizamos la memoria con el valor suavizado
        self.prev_angles[leg_name] = smoothed
        return smoothed
