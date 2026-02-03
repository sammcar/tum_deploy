import asyncio
import socket
import struct
import math
import time
import sys
import numpy as np

# Importamos el driver robusto que acabamos de crear
try:
    from motors_utils.moteus_driver import MoteusInterface # type: ignore
except ImportError:
    print("? Error: No se encuentra 'moteus_driver.py'.")
    sys.exit(1)

# ==========================================
# 1. CONFIGURACIÓN
# ==========================================
UDP_IP = "0.0.0.0" # Escuchar en todas las interfaces
UDP_PORT = 5006    # Puerto acordado

# Mapeo de IDs (Ajusta según tu robot)
BUS_MAP = {
    1: [4, 5, 6],     # FR
    2: [1, 2, 3],     # FL
    4: [7, 8, 9],     # BL
    5: [10, 11, 12],  # BR
}

# Orden estricto de recepción (mismo que el Sender)
# FL(c,f,t), FR(c,f,t), BL(...), BR(...)
LEG_ORDER = ["FL", "FR", "BL", "BR"]
IDS_BY_LEG = {
    "FL": [1, 2, 3],
    "FR": [4, 5, 6],
    "BL": [7, 8, 9],
    "BR": [10, 11, 12]
}

# Límites de seguridad para operación normal
OP_VEL_LIMIT = 5.0
OP_ACC_LIMIT = 5.0
OP_TORQUE = 10.0
OP_KP = 1.0 # KP más alto para seguimiento preciso de posición
OP_KD = 1.0

# ==========================================
# 2. CLASE RECEPTOR
# ==========================================
class RobotReceiver:
    def __init__(self):
        # Driver de Motores
        self.driver = MoteusInterface(BUS_MAP, debug=False)
        
        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False) # No bloqueante
        
        # Estado
        self.last_angles = {} # Diccionario {motor_id: revs}
        self.running = True
        self.emergency_active = False # Estado del E-Stop
        
        # Inicializar last_angles con 0.0
        for leg_ids in IDS_BY_LEG.values():
            for mid in leg_ids:
                self.last_angles[mid] = 0.0

    def parse_udp_packet(self, data):
        """
        Decodifica: 12 floats (ángulos) + 1 bool (estop)
        Retorna: (target_revs, is_emergency)
        """
        # 12 floats (48 bytes) + 1 bool (1 byte) = 49 bytes
        if len(data) != 49: 
            return None, False
            
        # Desempaquetar
        unpacked = struct.unpack('<12f?', data)
        angles_deg = unpacked[:12]
        is_emergency = unpacked[12] # El último valor es el bool
        #print(angles_deg)
        
        target_revs = {}
        idx = 0
        
        for leg_name in LEG_ORDER:
            ids = IDS_BY_LEG[leg_name]
            for motor_id in ids:
                deg = angles_deg[idx]
                revs = deg / 360.0 
                target_revs[motor_id] = revs
                idx += 1
                
        return target_revs, is_emergency

    async def loop(self):
        print(f"? Escuchando UDP en {UDP_IP}:{UDP_PORT}...")
        self.driver.start() # Homing inicial
        await asyncio.sleep(2.0)
        
        last_packet_time = time.time()
        
        try:
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    
                    # 1. PARSEAR NUEVO FORMATO
                    targets, remote_estop = self.parse_udp_packet(data)
                    
                    if targets is not None:
                        last_packet_time = time.time()
                        self.last_angles = targets

                        # 2. LÓGICA DE EMERGENCIA
                        if remote_estop:
                            if not self.emergency_active:
                                print("? E-STOP RECIBIDO DESDE PC. CORTANDO ENERGÍA.")
                                self.driver.emergency_stop() # Corta al instante
                                self.emergency_active = True
                        else:
                            # Si recibimos False, y estábamos en emergencia -> REINICIAR
                            if self.emergency_active:
                                print("? E-STOP LIBERADO. Reiniciando Driver (Homing)...")
                                # El driver quedó en estado invalido, lo reiniciamos
                                self.driver.start() 
                                self.emergency_active = False

                except BlockingIOError:
                    pass

                # 3. TIMEOUT DE SEGURIDAD (Si el PC se desconecta)
                if time.time() - last_packet_time > 0.5:
                    if not self.emergency_active:
                        # Opcional: Entrar en modo seguro si se pierde conexión
                        pass

                # 4. ENVIAR COMANDOS (Solo si no hay emergencia)
                if not self.emergency_active:
                    for mid, rev in self.last_angles.items():
                        self.driver.set_command(
                            motor_id=mid,
                            position=rev,
                            velocity=0.0,
                            kp_scale=OP_KP,
                            kd_scale=OP_KD,
                            max_torque=OP_TORQUE,
                            velocity_limit=OP_VEL_LIMIT,
                            accel_limit=OP_ACC_LIMIT
                        )
                
                await asyncio.sleep(0.002)

        except KeyboardInterrupt:
            print("\n? Cerrando receptor...")
        finally:
            self.driver.stop()
            print("\n?")

if __name__ == "__main__":
    receiver = RobotReceiver()
    try:
        asyncio.run(receiver.loop())
    except KeyboardInterrupt:
        pass
