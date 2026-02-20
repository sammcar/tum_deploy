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
# Ajusta estos valores según los topes mecánicos de tu diseño.
LIMITS_COXA  = (-0.25, 0.25)
LIMITS_FEMUR = (-0.45, 0.25)
LIMITS_TIBIA = (-0.5, 0.5)

# Esta lista mapea los límites con el orden de los IDs en la matriz: [COXA, FEMUR, TIBIA]
JOINT_LIMITS = [LIMITS_COXA, LIMITS_FEMUR, LIMITS_TIBIA]
JOINT_NAMES  = ["Coxa", "Fémur", "Tibia"]

# --- MATRIZ DE CONFIGURACIÓN POR PATA ---
# Formato: 
# "NOMBRE_PATA": ([ID_COXA, ID_FEMUR, ID_TIBIA], KP, KI, KD, ILIMIT, MAX_VOLT, MAX_AMP, MAX_VEL, DEF_VEL_LIM, DEF_ACC_LIM)

LEG_MATRIX = {
    #                      IDS           KP     KI    KD    ILIM   VOLT   AMP    VEL   D_VEL  D_ACC
    "FRONT_RIGHT (FR)":  ([4, 5, 6],    700.0,  0.0,  7.0,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0),
    "FRONT_LEFT (FL)":   ([1, 2, 3],    600.0,  0.0,  5.0,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0),
    "BACK_RIGHT (BR)":   ([10, 11, 12], 950.0,  0.0,  9.5,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0), 
    "BACK_LEFT (BL)":    ([7, 8, 9],    650.0,  0.0,  6.5,   0.0,   50.0,  36.0, 7.0,  2.0,   4.0),
}

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
        # Desempaquetar los 10 valores
        ids, kp, ki, kd, ilimit, volt, amp, vel, def_vel, def_acc = params
        
        print(f"\n>> CONFIGURANDO {leg_name}")
        print(f"   PID: {kp}/{ki}/{kd} | Limites Def: Vel={def_vel} Acc={def_acc}")

        # enumerate(ids) nos da un índice (0, 1, 2) para saber si es Coxa, Fémur o Tibia
        for i, motor_id in enumerate(ids):
            joint_name = JOINT_NAMES[i]
            pos_min, pos_max = JOINT_LIMITS[i]
            
            print(f"   -> {joint_name} (Motor {motor_id}) [Pos: {pos_min} a {pos_max}]:", end=" ", flush=True)
            stream = moteus.Stream(servos[motor_id])
            
            commands = [
                f"conf set servo.max_voltage {volt}",
                f"conf set servo.max_current_A {amp}",
                f"conf set servo.max_velocity {vel}",
                f"conf set servo.pid_position.kp {kp}",
                f"conf set servo.pid_position.ki {ki}",
                f"conf set servo.pid_position.kd {kd}",
                f"conf set servo.pid_position.ilimit {ilimit}",
                f"conf set servo.default_velocity_limit {def_vel}",
                f"conf set servo.default_accel_limit {def_acc}",
                f"conf set servopos.position_min {pos_min}", 
                f"conf set servopos.position_max {pos_max}", 
                "conf write"
            ]

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
