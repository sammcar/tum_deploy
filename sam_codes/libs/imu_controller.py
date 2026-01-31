import numpy as np

class ImuController:
    def __init__(self, kp=0.5, ki=0.01, kd=0.1):
        # Ganancias PID para Roll y Pitch
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.error_prev = np.array([0.0, 0.0]) # [Roll, Pitch]
        self.integral = np.array([0.0, 0.0])
        
        # Offset del IMU respecto al centro del robot (X, Y, Z)
        self.imu_offset = np.array([0.05, 0.0, 0.0]) # Ejemplo: 5cm adelante

    def compute_compensation(self, imu_rpy_actual, target_rpy, dt):
        """
        Calcula la rotación necesaria para estabilizar el chasis.
        imu_rpy_actual: Lo que lee el sensor [roll, pitch, yaw]
        target_rpy: Lo que el usuario quiere (usualmente [0, 0, 0])
        """
        # Error = Meta - Actual
        error = np.array(target_rpy[:2]) - np.array(imu_rpy_actual[:2])
        
        # 2. PID
        self.integral += error * dt
        derivativo = (error - self.error_prev) / dt
        
        # Salida del PID (Ajuste angular)
        ajuste = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivativo)
        
        self.error_prev = error
        
        # Retornamos el ajuste para [Roll, Pitch, Yaw] (Yaw suele ser 0 o directo del joystick)
        return np.array([ajuste[0], ajuste[1], 0.0])