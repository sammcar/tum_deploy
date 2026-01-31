import asyncio
import math
import sys
import os
import moteus
import moteus_pi3hat

# --- IMPORTAR LIBRERÍA LOCAL ---
sys.path.append(os.path.join(os.path.dirname(__file__), 'libs'))
try:
    import kinematics_sam as ik
except ImportError:
    print("ERROR: No se encuentra 'libs/kinematics_sam.py'.")
    sys.exit(1)

# ==========================================
# 1. CONFIGURACIÓN
# ==========================================

L1 = 0.093
L2 = 0.144
L3 = 0.210

BUS_MAP = {
    1: [4, 5, 6],     # FR
    2: [1, 2, 3],     # FL
    4: [7, 8, 9],     # BL
    5: [10, 11, 12],  # BR
}

COMMON_DIRS_L = [1.0, 1.0, 1.0]
COMMON_DIRS_R = [-1.0, 1.0, 1.0]
COMMON_OFFSETS = [0.0, math.pi/2, 0.0]

LEGS = {
    'FL': {'ids': [1, 2, 3],   'bus': 2, 'right': False, 'knee': False, 'offsets': COMMON_OFFSETS, 'dirs': COMMON_DIRS_L},
    'BL': {'ids': [7, 8, 9],   'bus': 4, 'right': False, 'knee': False, 'offsets': COMMON_OFFSETS, 'dirs': COMMON_DIRS_L},
    'FR': {'ids': [4, 5, 6],   'bus': 1, 'right': True,  'knee': False, 'offsets': COMMON_OFFSETS, 'dirs': COMMON_DIRS_R},
    'BR': {'ids': [10, 11, 12],'bus': 5, 'right': True,  'knee': False, 'offsets': COMMON_OFFSETS, 'dirs': COMMON_DIRS_R},
}

ALL_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

# Configuración de Fuerza
# Subimos un poco el torque límite para que aguante el peso
VEL_LIMIT = 2.0
ACCEL_LIMIT = 4.0
TORQUE_LIMIT = 5.0 # <--- AUMENTADO A 5.0 Nm para aguantar mejor

# Variables Globales
target_revs = {id: None for id in ALL_IDS}
telemetry_data = {
    id: {'pos': 0.0, 'fault': 0, 'temp': 0.0, 'voltage': 0.0, 'torque': 0.0} 
    for id in ALL_IDS
}
ik_targets_rad = {}
is_running = True
monitor_active = False # Flag para saber si estamos en modo monitor

# ==========================================
# 2. PARADA DE EMERGENCIA
# ==========================================
async def emergency_stop(transport, servos):
    global is_running
    is_running = False
    print("\n!!! RELAJANDO MOTORES !!!")
    stop_cmd = [servos[id].make_stop(query=False) for id in ALL_IDS]
    for _ in range(5):
        await transport.cycle(stop_cmd)
        await asyncio.sleep(0.02)
    print(">> Motores libres.")

# ==========================================
# 3. BUCLE DE CONTROL (HOLD CONSTANTE)
# ==========================================
async def control_loop(transport, servos):
    global is_running
    
    while is_running:
        commands = []
        # Si el monitor está activo, pedimos datos (Query) SIEMPRE para refrescar rápido.
        # Si no, pedimos de vez en cuando.
        do_query = True if monitor_active else True 
        
        for motor_id in ALL_IDS:
            target = target_revs[motor_id]
            
            if target is not None:
                # --- AQUÍ ESTÁ EL COMANDO CONSTANTE QUE PEDISTE ---
                cmd = servos[motor_id].make_position(
                    position=target,
                    velocity=0.0,         # <--- VELOCIDAD 0 CONSTANTE
                    velocity_limit=VEL_LIMIT,
                    accel_limit=ACCEL_LIMIT,
                    maximum_torque=TORQUE_LIMIT,
                    kp_scale=1.0, 
                    kd_scale=1.0, 
                    watchdog_timeout=5.0, # 5 Segundos de paciencia
                    query=do_query 
                )
            else:
                cmd = servos[motor_id].make_stop(query=do_query)
            
            commands.append(cmd)
        
        try:
            results = await transport.cycle(commands)
            
            # Guardamos TODOS los datos relevantes
            for r in results:
                if r.id in telemetry_data:
                    telemetry_data[r.id]['pos'] = r.values.get(moteus.Register.POSITION, 0.0)
                    telemetry_data[r.id]['fault'] = r.values.get(moteus.Register.FAULT, 0)
                    telemetry_data[r.id]['temp'] = r.values.get(moteus.Register.TEMPERATURE, 0.0)
                    telemetry_data[r.id]['voltage'] = r.values.get(moteus.Register.VOLTAGE, 0.0)
                    telemetry_data[r.id]['torque'] = r.values.get(moteus.Register.TORQUE, 0.0)
        except Exception:
            pass
            
        await asyncio.sleep(0.02) # 50Hz estricto

# ==========================================
# 4. MONITOR EN TIEMPO REAL
# ==========================================
def rad_to_rev(r): return r / (2.0 * math.pi)
def rev_to_rad(r): return r * 2.0 * math.pi

def get_deg(revs, direction, offset):
    rads = rev_to_rad(revs)
    angle = (rads - offset) / direction
    return math.degrees(angle)

async def monitor_loop():
    """Bucle que borra pantalla y actualiza datos"""
    global monitor_active
    print("Iniciando Monitor... (Ctrl+C para salir del monitor)")
    
    try:
        while monitor_active and is_running:
            os.system('clear') # Limpiar pantalla (Linux)
            print(f"{'PATA':<4}|{'JOINT':<5}|{'ID':<3}|{'OBJ':<7}|{'REAL':<7}|{'ERR':<6}|{'TRQ':<5}|{'V':<4}|{'°C':<3}|{'FAULT'}")
            print("-" * 75)
            
            legs = sorted(LEGS.keys())
            names = ["H", "F", "T"] # Hip, Femur, Tibia
            
            for name in legs:
                cfg = LEGS[name]
                for i in range(3):
                    mid = cfg['ids'][i]
                    # Target
                    key = f"{name}_{i}"
                    tgt = math.degrees(ik_targets_rad.get(key, 0.0))
                    
                    # Real Data
                    d = telemetry_data[mid]
                    real = get_deg(d['pos'], cfg['dirs'][i], cfg['offsets'][i])
                    err = abs(tgt - real)
                    trq = d['torque']
                    volt = d['voltage']
                    temp = d['temp']
                    flt = d['fault']
                    
                    # Formato condicional
                    s_err = "!" if err > 5.0 else " "
                    s_flt = f"ERR {flt}" if flt != 0 else "OK"
                    
                    # Alerta Visual si falla
                    row_str = f"{name:<4}|{names[i]:<5}|{mid:<3}|{tgt:7.1f}|{real:7.1f}|{err:6.1f}{s_err}|{trq:5.1f}|{volt:4.1f}|{temp:3.0f}|{s_flt}"
                    print(row_str)
            
            print("-" * 75)
            print(" Presiona Ctrl+C para volver al menú de comandos.")
            await asyncio.sleep(0.2) # Refresco de pantalla 5Hz
            
    except KeyboardInterrupt:
        pass # Regresar al menú principal
    finally:
        monitor_active = False

# ==========================================
# 5. INPUT Y MENU
# ==========================================
def parse_input(text):
    parts = text.upper().split()
    groups = {
        'ALL': ['FL', 'FR', 'BL', 'BR'], 'TODAS': ['FL', 'FR', 'BL', 'BR'],
        'LEFT': ['FL', 'BL'], 'RIGHT': ['FR', 'BR'],
        'FL': ['FL'], 'FR': ['FR'], 'BL': ['BL'], 'BR': ['BR']
    }
    try:
        if len(parts) == 3:
            c = [float(p) for p in parts]
            return groups['ALL'], c[0], c[1], c[2]
        elif len(parts) == 4:
            sel = parts[0]
            c = [float(p) for p in parts[1:]]
            return groups.get(sel, groups['ALL']), c[0], c[1], c[2]
    except ValueError: pass
    return None, 0, 0, 0

async def main():
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=BUS_MAP)
    servos = {id: moteus.Controller(id=id, transport=transport) for id in ALL_IDS}

    await emergency_stop(transport, servos)
    global is_running, monitor_active
    is_running = True
    
    # Arrancar Loop HOLD en fondo
    control_task = asyncio.create_task(control_loop(transport, servos))
    
    curr_x, curr_y, curr_z = 0.0, 0.1, -0.25
    print("\n--- SISTEMA LISTO (V7 DASHBOARD) ---")
    print(" 1. Mueve el robot: [SELECTOR] X Y Z")
    print(" 2. Escribe 'm' para ver el MONITOR EN VIVO.")
    print(" 3. 'q' para salir.")

    try:
        while is_running:
            if not monitor_active:
                user_text = await asyncio.to_thread(input, f"\nCMD ({curr_x}, {curr_y}, {curr_z}) >> ")
            else:
                # Si volvemos del monitor, limpiamos buffer
                user_text = "" 
            
            if user_text.lower() in ['q', 'exit']:
                break
            
            # ACTIVAR MONITOR
            if user_text.lower() == 'm':
                monitor_active = True
                await monitor_loop() # Bloquea aquí hasta Ctrl+C
                print("\n--- MENU COMANDOS ---")
                continue
            
            # PROCESAR MOVIMIENTO
            legs, tx, ty, tz = parse_input(user_text)
            
            if legs:
                curr_x, curr_y, curr_z = tx, ty, tz
                print(f" -> Moviendo {legs}...")

                for name in legs:
                    cfg = LEGS[name]
                    q1, q2, q3 = ik.solve_IK(tx, ty, tz, L1, L2, L3, cfg['right'], cfg['knee'])
                    
                    ik_targets_rad[f"{name}_0"] = q1
                    ik_targets_rad[f"{name}_1"] = q2
                    ik_targets_rad[f"{name}_2"] = q3

                    m_hip   = (q1 * cfg['dirs'][0]) + cfg['offsets'][0]
                    m_thigh = (q2 * cfg['dirs'][1]) + cfg['offsets'][1]
                    m_calf  = (q3 * cfg['dirs'][2]) + cfg['offsets'][2]
                    
                    target_revs[cfg['ids'][0]] = rad_to_rev(m_hip)
                    target_revs[cfg['ids'][1]] = rad_to_rev(m_thigh)
                    target_revs[cfg['ids'][2]] = rad_to_rev(m_calf)
                
                # Entrar a monitor automáticamente por 2 segundos para ver llegada
                # monitor_active = True
                # asyncio.create_task(stop_monitor_later()) 
            else:
                if user_text: print("Error formato.")

    except KeyboardInterrupt:
        pass
    finally:
        control_task.cancel()
        try: await control_task
        except: pass
        await emergency_stop(transport, servos)

if __name__ == '__main__':
    asyncio.run(main())
