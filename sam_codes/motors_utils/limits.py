import asyncio
import moteus
import moteus_pi3hat

# --- CONFIGURACIÓN DE BUS (Hardware) ---
BUS_MAP = {
    1: [4, 5, 6],
    2: [1, 2, 3],
    4: [7, 8, 9],
    5: [10, 11, 12],
}

# --- LÍMITES POR ARTICULACIÓN (Min, Max) en revoluciones ---
LIMITS_COXA  = (float('nan'), float('nan')) #(-0.25, 0.25)
LIMITS_FEMUR = (float('nan'), float('nan')) #(-0.45, 0.25)
LIMITS_TIBIA = (float('nan'), float('nan')) #(-0.5, 0.5)

JOINT_LIMITS = [LIMITS_COXA, LIMITS_FEMUR, LIMITS_TIBIA]
JOINT_NAMES  = ["Coxa", "Fémur", "Tibia"]

CONFIG_TUM = {
    'servopos.position_min' : [
        ('1,2,4,5,7,8,10,11', '-.70'),
        ('3,6,9,12', '-.25'),
    ],
    'servopos.position_max' : [
        ('1,2,4,5,7,8,10,11', '.70'),
        ('3,6,9,12', '.25'),
    ],
    'servo.flux_brake_min_voltage' : '41.0',
    'servo.flux_brake_resistance_ohm' : '0.05',
    'servo.pid_position.kp' : [
        ('1,2,4,5,7,8,10,11', '50'),
        ('3,6,9,12', '200'),
    ],
    'servo.velocity_threshold' : '0.01',
    'servo.pid_position.kd' : '9',
    'servo.pid_dq.kp' : '0.026',
    'servo.pid_dq.ki' : '100',
    'motor_position.rotor_to_output_ratio' : [
        ('1,3,4,6,7,9,10,12', '0.16666667'),
        ('2,5,8,11', '0.093750'),
    ],
    'motor_position.sources.0.pll_filter_hz' : '200.0',
    'servo.default_velocity_limit': 'nan',
    'servo.default_accel_limit': 'nan',
}

# --- MATRIZ DE CONFIGURACIÓN POR PATA ---
LEG_MATRIX = {
    #                      IDS           KP     KI    KD    ILIM   VOLT   AMP    VEL   D_VEL  D_ACC
    "FRONT_RIGHT (FR)":  ([4, 5, 6],    50.0,  0.0,  7.0,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0),
    "FRONT_LEFT (FL)":   ([1, 2, 3],    50.0,  0.0,  7.0,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0),
    "BACK_RIGHT (BR)":   ([10, 11, 12], 50.0,  0.0,  7.0,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0), 
    "BACK_LEFT (BL)":    ([7, 8, 9],    50.0,  0.0,  7.0,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0),
}

def get_tum_value(motor_id, key):
    """Busca y extrae el valor específico para un motor desde CONFIG_TUM."""
    if key not in CONFIG_TUM:
        return None
    val = CONFIG_TUM[key]
    
    # Si es un string directo, aplica para todos
    if isinstance(val, str):
        return val
        
    # Si es una lista, buscamos en qué tupla se encuentra el ID
    if isinstance(val, list):
        for id_str, target_val in val:
            valid_ids = [int(x) for x in id_str.split(',')]
            if motor_id in valid_ids:
                return target_val
    return None

async def send_command_safe(stream, command_str, timeout=1.8):
    """Envía comando y gestiona errores de comunicación"""
    try:
        cmd_bytes = command_str.encode('utf-8')
        response = await asyncio.wait_for(stream.command(cmd_bytes), timeout=timeout)
        if b"ERR" in response:
            return None, f"Moteus ERR: {response.decode().strip()}"
        return response, None
    except asyncio.TimeoutError:
        return None, "TIMEOUT"
    except Exception as e:
        return None, f"EXCEPTION: {e}"

async def main():
    print(f"--- INICIANDO CONFIGURACIÓN DE PARAMETROS ---")
    
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=BUS_MAP)
    
    all_ids = []
    for params in LEG_MATRIX.values():
        all_ids.extend(params[0])
    
    servos = {id: moteus.Controller(id=id, transport=transport) for id in all_ids}
    errores = []

    for leg_name, params in LEG_MATRIX.items():
        ids, mat_kp, mat_ki, mat_kd, ilimit, volt, amp, vel, def_vel, def_acc = params
        
        print(f"\n>> CONFIGURANDO {leg_name}")

        for i, motor_id in enumerate(ids):
            joint_name = JOINT_NAMES[i]
            pos_min, pos_max = JOINT_LIMITS[i]
            
            # --- SOBRESCRIBIR PID CON VALORES DE CONFIG_TUM ---
            # Si get_tum_value no encuentra nada, mantenemos el valor de la matriz como respaldo
            final_pos_kp = get_tum_value(motor_id, 'servo.pid_position.kp') or mat_kp
            final_pos_kd = get_tum_value(motor_id, 'servo.pid_position.kd') or mat_kd
            final_pos_ki = mat_ki # No está en CONFIG_TUM, usamos el de la matriz
            
            # Extraer PID de corriente (DQ)
            tum_dq_kp = get_tum_value(motor_id, 'servo.pid_dq.kp')
            tum_dq_ki = get_tum_value(motor_id, 'servo.pid_dq.ki')
            
            print(f"   -> {joint_name} (Motor {motor_id}) | PID Pos: {final_pos_kp}/{final_pos_ki}/{final_pos_kd}:", end=" ", flush=True)
            stream = moteus.Stream(servos[motor_id])
            
            # Lista base de comandos
            commands = [
                f"conf set servo.max_voltage {volt}",
                f"conf set servo.max_current_A {amp}",
                f"conf set servo.max_velocity {vel}",
                f"conf set servo.pid_position.kp {final_pos_kp}",
                f"conf set servo.pid_position.ki {final_pos_ki}",
                f"conf set servo.pid_position.kd {final_pos_kd}",
                f"conf set servo.pid_position.ilimit {ilimit}",
            ]
            
            # Agregar comandos de PID DQ solo si existen en CONFIG_TUM
            if tum_dq_kp is not None:
                commands.append(f"conf set servo.pid_dq.kp {tum_dq_kp}")
            if tum_dq_ki is not None:
                commands.append(f"conf set servo.pid_dq.ki {tum_dq_ki}")

            # Agregar el resto de límites
            commands.extend([
                f"conf set servo.default_velocity_limit {def_vel}",
                f"conf set servo.default_accel_limit {def_acc}",
                f"conf set servopos.position_min {pos_min}", 
                f"conf set servopos.position_max {pos_max}", 
                "conf write"
            ])

            motor_success = True
            for cmd in commands:
                resp, err = await send_command_safe(stream, cmd)
                if err:
                    print(f"[FALLÓ: {cmd} -> {err}]", end=" ")
                    motor_success = False
                    break 
            
            if motor_success:
                print("[OK]")
            else:
                errores.append(motor_id)
                print("") 

    print("\n" + "="*60)
    if errores:
        print(f"RESUMEN: Fallaron los motores: {errores}")
    else:
        print("¡ÉXITO TOTAL! Todos los parámetros han sido escritos y guardados.")
    print("="*60)

if __name__ == '__main__':
    asyncio.run(main())
