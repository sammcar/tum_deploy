import asyncio
import math
import sys
import os
import json
import mmap
import time
import numpy as np
import moteus #type:ignore
import moteus_pi3hat #type:ignore

# Intentamos importar la cinemática
try:
    from libs.robot_class import QuadrupedRobot
except ImportError:
    print("? ERROR: No se encuentra 'libs/robot_class.py'.")
    sys.exit(1)

# ==========================================
# 1. CONFIGURACIÓN FÍSICA (PRESERVADA)
# ==========================================
ROBOT_LENGTH = 0.375 
ROBOT_WIDTH  = 0.200      
L1, L2, L3 = 0.093, 0.147, 0.230 # Datos actualizados del tum_robot.py

def deg_to_rad(deg):
    return deg * math.pi / 180

BUS_MAP = {
    1: [4, 5, 6],     # FR
    2: [1, 2, 3],     # FL
    4: [7, 8, 9],     # BL
    5: [10, 11, 12],  # BR
}

# Direcciones
DIRS_L = [1.0, 1.0, 1.0]
DIRS_R = [-1.0, 1.0, 1.0]

# Offsets 
OFFSET_FL = [0.0, math.pi/2, 0.0]
OFFSET_FR = [0.0, math.pi/2, 0.0]
OFFSET_BL = [0.0, math.pi/2, 0.0] 
OFFSET_BR = [0.0, math.pi/2, 0.0]

LEGS_CONFIG = {
    'FL': {'ids': [1, 2, 3],   'dirs': DIRS_L, 'offsets': OFFSET_FL},
    'FR': {'ids': [4, 5, 6],   'dirs': DIRS_R, 'offsets': OFFSET_FR},
    'BL': {'ids': [7, 8, 9],   'dirs': DIRS_L, 'offsets': OFFSET_BL},
    'BR': {'ids': [10, 11, 12],'dirs': DIRS_R, 'offsets': OFFSET_BR},
}

ALL_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
VEL_LIMIT = 4.0   
ACCEL_LIMIT = 8.0 
TORQUE_LIMIT = 10.0 

# Control Loop Timing
DT = 0.02 # 50Hz (Coincide con asyncio.sleep(0.02))

# ==========================================
# 2. CLASE LECTORA DE MEMORIA
# ==========================================
class SharedMemoryReader:
    def __init__(self, filename='/dev/shm/tum_shared', size=4096):
        self.filename = filename
        self.size = size
        self.mem = None
        self.fd = None
        self.connected = False
        
        try:
            self.fd = os.open(self.filename, os.O_RDONLY)
            self.mem = mmap.mmap(self.fd, self.size, mmap.MAP_SHARED, mmap.PROT_READ)
            self.connected = True
            print(f"? Conectado a memoria compartida: {filename}")
        except FileNotFoundError:
            print(f"?? Esperando al proceso C++ (No existe {filename})...")
        except Exception as e:
            print(f"? Error SHM: {e}")

    def read(self):
        if not self.connected:
            try:
                self.fd = os.open(self.filename, os.O_RDONLY)
                self.mem = mmap.mmap(self.fd, self.size, mmap.MAP_SHARED, mmap.PROT_READ)
                self.connected = True
                print("? Conexión establecida ahora.")
            except:
                return None

        try:
            self.mem.seek(0)
            data_bytes = self.mem.read(self.size)
            end_idx = data_bytes.find(b'\x00')
            
            if end_idx != -1:
                json_str = data_bytes[:end_idx].decode('utf-8')
            else:
                json_str = data_bytes.decode('utf-8')

            last_brace = json_str.rfind('}')
            if last_brace != -1:
                json_str = json_str[:last_brace+1]
                return json.loads(json_str)
            return None
        except Exception:
            return None

# ==========================================
# 3. FUNCIONES DE UTILIDAD
# ==========================================
def rad_to_rev(rads, direction, offset):
    angle_corrected = (rads * direction) + offset
    return angle_corrected / (2.0 * math.pi)

# ==========================================
# 4. BUCLE PRINCIPAL
# ==========================================
async def main():
    # --- HARDWARE ---
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=BUS_MAP)
    servos = {id: moteus.Controller(id=id, transport=transport) for id in ALL_IDS}
    
    # --- MEMORIA ---
    shm = SharedMemoryReader()
    
    # --- CINEMÁTICA ---
    # Inicializamos el robot con tus medidas actualizadas
    robot = QuadrupedRobot(ROBOT_LENGTH, ROBOT_WIDTH, L1, L2, L3)
    
    # Estado inicial de control
    t_ctrl = 0.0 # Tiempo interno para el generador de marchas
    
    print("\n? TUM ROBOT MASTER V2 (GAIT ENABLED)")
    print("Esperando datos del Joystick (C++)... presiona Ctrl+C para salir.")

    last_print = time.time()
    
    try:
        while True:
            start_loop = time.time()

            # A. LEER C++
            data = shm.read()
            
            # Valores por defecto (Seguros)
            cmd_vx, cmd_vy, cmd_wz = 0.0, 0.0, 0.0
            cmd_bx, cmd_by, cmd_bz = 0.0, 0.0, 0.22 # Altura base
            cmd_roll, cmd_pitch, cmd_yaw = 0.0, 0.0, 0.0
            trot_freq = 2.5
            gait_mode = "STAND"
            use_imu = False
            
            if data:
                # Velocidades (Sistema de coordenadas del robot)
                cmd_vx = data.get('vx', 0.0)
                cmd_vy = data.get('vy', 0.0)
                cmd_wz = data.get('wz', 0.0)
                
                # Posición Cuerpo
                cmd_bx = data.get('body_x', 0.0)
                cmd_by = data.get('body_y', 0.0)
                cmd_bz = data.get('body_z', 0.22) 

                # Rotación (Grados -> Radianes)
                cmd_roll  = math.radians(data.get('body_roll', 0.0))
                cmd_pitch = math.radians(data.get('body_pitch', 0.0))
                cmd_yaw   = math.radians(data.get('body_yaw', 0.0))
                
                # Configuración
                trot_freq = data.get('trot_freq', 2.5)
                gait_mode = data.get('gait_mode', 'STAND')
                use_imu = data.get('use_imu', False)

            # B. CONSTRUIR DICCIONARIO DE COMANDO (Formato PyBullet)
            # Esto es lo que espera tu librería libs/robot_class.py
            cmd_dict = {
                'vx': cmd_vx, 
                'vy': cmd_vy, 
                'wz': cmd_wz, 
                'body_z': cmd_bz,
                'body_x': cmd_bx, 
                'body_y': cmd_by, 
                'body_roll': cmd_roll, 
                'body_pitch': cmd_pitch, 
                'body_yaw': cmd_yaw,
                'use_imu': use_imu, 
                'es_rodilla': True # Asumimos rodilla
            }

            # IMU Placeholder (Hasta que tengas el sensor real conectado)
            # Formato: [roll, pitch, yaw]
            current_imu = [0.0, 0.0, 0.0] 

            # C. CALCULAR MARCHA Y CINEMÁTICA (STEP)
            try:
                # Aquí ocurre la magia: step maneja el tiempo y genera pasos si es TROT
                angles = robot.step(t_ctrl, DT, cmd_dict, current_imu, gait_mode, trot_freq)
            except Exception as e:
                print(f"?? Error en cinemática: {e}")
                angles = {}

            # D. DEBUG UNIFICADO
            if time.time() - last_print > 0.5:
                sub = data.get('submode', '-') if data else '-'
                print(f"[{gait_mode}|{sub}] Z:{cmd_bz:.3f} | V:({cmd_vx:.2f}, {cmd_vy:.2f}) | Freq:{trot_freq}")
                last_print = time.time()

            # E. ENVIAR A MOTORES
            commands = []
            if angles: # Solo si hubo cálculo exitoso
                for leg_key, rads in angles.items():
                    if leg_key not in LEGS_CONFIG: continue
                    cfg = LEGS_CONFIG[leg_key]
                    
                    for i in range(3):
                        motor_id = cfg['ids'][i]
                        # Conversión Radianes -> Revoluciones con tus Offsets
                        rev = rad_to_rev(rads[i], cfg['dirs'][i], cfg['offsets'][i])
                        
                        cmd = servos[motor_id].make_position(
                            position=rev,
                            velocity=0.0,
                            velocity_limit=VEL_LIMIT,
                            accel_limit=ACCEL_LIMIT,
                            maximum_torque=TORQUE_LIMIT,
                            watchdog_timeout=0.2, 
                            query=False
                        )
                        commands.append(cmd)
                
                await transport.cycle(commands)
            
            # F. ACTUALIZAR TIEMPO Y ESPERAR
            t_ctrl += DT
            
            # Mantener ciclo estable de 50Hz
            elapsed = time.time() - start_loop
            sleep_time = max(0, DT - elapsed)
            await asyncio.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupción detectada.")
    
    finally:
        print("Deteniendo motores suavemente...")
        stop_cmds = [servos[id].make_stop(query=False) for id in ALL_IDS]
        for _ in range(5):
            await transport.cycle(stop_cmds)
            await asyncio.sleep(0.01)
        print("? Sistema detenido.")

if __name__ == '__main__':
    asyncio.run(main())
