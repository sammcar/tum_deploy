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
import struct

# Intentamos importar la cinemática
try:
    from libs.robot_class import QuadrupedRobot
except ImportError:
    print("? ERROR: No se encuentra 'libs/robot_class.py'.")
    sys.exit(1)

# ==========================================
# 1. CONFIGURACIÓN FÍSICA
# ==========================================
ROBOT_LENGTH = 0.380 
ROBOT_WIDTH  = 0.200     
L1, L2, L3 = 0.093, 0.147, 0.230

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
ang = deg_to_rad(20)

OFFSET_FL = [0.0, math.pi/2, 0.0]
OFFSET_FR = [0.0, math.pi/2, 0.0]
OFFSET_BL = [0.0, math.pi/2, 0.0] 
OFFSET_BR = [0.0, math.pi/2, 0.0]

TIBIA_GEAR_RATIO = 1.6

LEGS_CONFIG = {
    'FL': {'ids': [1, 2, 3],   'dirs': DIRS_L, 'offsets': OFFSET_FL},
    'FR': {'ids': [4, 5, 6],   'dirs': DIRS_R, 'offsets': OFFSET_FR},
    'BL': {'ids': [7, 8, 9],   'dirs': DIRS_L, 'offsets': OFFSET_BL},
    'BR': {'ids': [10, 11, 12],'dirs': DIRS_R, 'offsets': OFFSET_BR},
}

ALL_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
VEL_LIMIT = 5.0   
ACCEL_LIMIT = 8.0 
TORQUE_LIMIT = 15.0 

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
        
        # DEFINICIÓN EXACTA DE TU STRUCT C++
        # 11 floats (4 bytes c/u)
        # 2 int8 (signed char)
        # 1 bool
        # '=' asegura orden nativo sin padding extraño
        self.struct_fmt = '=11f2b?' 
        self.struct_size = struct.calcsize(self.struct_fmt)

        try:
            self.fd = os.open(self.filename, os.O_RDONLY)
            self.mem = mmap.mmap(self.fd, self.size, mmap.MAP_SHARED, mmap.PROT_READ)
            self.connected = True
            print(f"? Conectado a SHM Binaria: {filename}")
        except FileNotFoundError:
            print(f"?? Esperando al proceso C++...")
        except Exception as e:
            print(f"? Error SHM init: {e}")

    def read(self):
        if not self.connected:
            try:
                self.fd = os.open(self.filename, os.O_RDONLY)
                self.mem = mmap.mmap(self.fd, self.size, mmap.MAP_SHARED, mmap.PROT_READ)
                self.connected = True
            except:
                return None

        try:
            self.mem.seek(0)
            # Leemos SOLAMENTE el tamaño de la estructura (aprox 47 bytes)
            buf = self.mem.read(self.struct_size)
            
            # Desempaquetamos los bytes a variables
            data = struct.unpack(self.struct_fmt, buf)
            
            # Asignamos en el orden EXACTO de tu struct C++
            return {
                'vx': data[0], 'vy': data[1], 'wz': data[2],       # float vx, vy, wz
                'body_x': data[3], 'body_y': data[4], 'body_z': data[5], # float body_x, body_y, body_z
                'body_roll': data[6], 'body_pitch': data[7], 'body_yaw': data[8], # float roll, pitch, yaw
                'freq': data[9],         # float freq
                'default_z': data[10],   # float default_z
                
                # Convertimos los enteros de vuelta a strings para que tu lógica vieja no se rompa
                'gait_mode': ['STAND', 'CRAWL', 'TROT'][data[11]], # int8_t mode
                'submode': ['TRANSLATION', 'ROTATION'][data[12]],  # int8_t submode
                'use_imu': data[13]      # bool use_imu
            }
        except (struct.error, IndexError, ValueError):
            # Captura errores si C++ no ha escrito nada válido aún
            return None
        except Exception as e:
            print(f"Error lectura SHM: {e}")
            return None

# ==========================================
# 3. FUNCIONES DE UTILIDAD
# ==========================================
def rad_to_rev(rads, direction, offset, gear_ratio=1.0):
    """
    Convierte radianes de articulación a revoluciones del motor.
    Aplica dirección, offset y gear_ratio.
    """
    angle_corrected = (rads * direction) + offset
    # Multiplicamos por gear_ratio (ej: 1.6) porque el motor gira más que la joint
    return (angle_corrected / (2.0 * math.pi)) * gear_ratio

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
    robot = QuadrupedRobot(ROBOT_LENGTH, ROBOT_WIDTH, L1, L2, L3)
    feet_targets = robot.get_default_stance()
    
    # Estado inicial
    body_pos = np.array([0.0, 0.0, 0.15]) 
    body_rpy = np.array([0.0, 0.0, 0.0])
    
    print("\n? ROBOT SHM BRIDGE ACTIVO")
    print("Esperando datos del Joystick (C++)... presiona Ctrl+C para salir.")

    last_print = time.time()
    
    try:
        while True:
            # A. LEER C++
            data = shm.read()
            
            if data:
                # Actualizar variables desde SHM
                body_pos[0] = data.get('body_x', 0.0)
                body_pos[1] = data.get('body_y', 0.0)
                body_pos[2] = data.get('body_z', 0.22) 

                roll_deg  = data.get('body_roll', 0.0)
                pitch_deg = data.get('body_pitch', 0.0)
                yaw_deg   = data.get('body_yaw', 0.0)
                
                body_rpy[0] = math.radians(roll_deg)
                body_rpy[1] = math.radians(pitch_deg)
                body_rpy[2] = math.radians(yaw_deg)

            # B. CALCULAR CINEMÁTICA INVERSA
            angles = robot.compute_pose(body_pos, body_rpy, [0,0,0], feet_targets)            

            # --- C. DEBUG UNIFICADO (SHM + ÁNGULOS + REVS) ---
            if time.time() - last_print > 0.5:
                # Info General
                mode = data.get('gait_mode', 'UNKNOWN') if data else 'NO_DATA'
                print(f"\nSTATUS: [{mode}] Z:{body_pos[2]:.3f} | RPY: {math.degrees(body_rpy[0]):.1f}, {math.degrees(body_rpy[1]):.1f}, {math.degrees(body_rpy[2]):.1f}")
                
                print("--- DETALLE MOTORES [Grados -> Revoluciones] ---")
                # Info por Pata
                for leg, rads in angles.items():
                    if leg not in LEGS_CONFIG: continue
                    cfg = LEGS_CONFIG[leg]
                    
                    degs_list = []
                    revs_list = []
                    
                    for i in range(3): # Coxa, Femur, Tibia
                        # Grados visuales (Aseguramos float nativo)
                        d = float(math.degrees(rads[i]))
                        degs_list.append(round(d, 1))

                        current_ratio = TIBIA_GEAR_RATIO if i == 2 else 1.0
                    
                        rev = rad_to_rev(rads[i], cfg['dirs'][i], cfg['offsets'][i], gear_ratio=current_ratio)
                        
                        # Convertimos r (que es numpy) a float nativo
                        revs_list.append(round(float(rev), 3))
                    
                    print(f" {leg}: {degs_list} -> {revs_list}")
                
                last_print = time.time()

            # D. ENVIAR A MOTORES
            commands = []
            for leg_key, rads in angles.items():
                if leg_key not in LEGS_CONFIG: continue
                cfg = LEGS_CONFIG[leg_key]
                
                for i in range(3):
                    motor_id = cfg['ids'][i]
                    current_ratio = TIBIA_GEAR_RATIO if i == 2 else 1.0
                    rev = rad_to_rev(rads[i], cfg['dirs'][i], cfg['offsets'][i],gear_ratio=current_ratio)
                    
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
            await asyncio.sleep(0.02)

    except KeyboardInterrupt:
        print("\nInterrupción detectada.")

    finally:
        print("ESTADO: Moviendo a pose segura (-5 grados)...")
        
        # 1. DEFINIR LA POSE SEGURA
        safe_deg = -5.0
        safe_revs = safe_deg / 360.0
        
        # 2. BUCLE DE TRANSICIÓN (2 SEGUNDOS)
        # Enviamos el comando de posición durante 2 segundos para dar tiempo a que lleguen
        # 100 iteraciones * 0.02s = 2.0 segundos aprox
        for _ in range(100):
            safe_cmds = []
            for id in ALL_IDS:
                cmd = servos[id].make_position(
                    position=safe_revs,
                    velocity=0.1,
                    velocity_limit=2.0,      # Velocidad controlada
                    maximum_torque=2.0,     # Torque suficiente para sostenerse
                    kp_scale=1.0,
                    kd_scale=1.0,
                    query=False              # No necesitamos leer, solo escribir rápido
                )
                safe_cmds.append(cmd)
            
            await transport.cycle(safe_cmds)
            await asyncio.sleep(0.02) # 50Hz para el apagado suave

        print("ESTADO: Deteniendo motores (Desenergizando)...")
        
        # 3. APAGADO FINAL (STOP)
        # Enviar comandos de parada varias veces para asegurar recepción
        stop_cmds = [servos[id].make_stop(query=False) for id in ALL_IDS]
        for _ in range(5):
            await transport.cycle(stop_cmds)
            await asyncio.sleep(0.01)
            
        print("? Sistema detenido y seguro.")

if __name__ == '__main__':
    asyncio.run(main())
