import mmap
import struct
import time
import os

# ==========================================
# CONFIGURACIÓN DE ESTRUCTURA (SHM)
# ==========================================
# 61 doubles: 12 angles, 12 vels, 12 accels, 12 kp, 12 kd, 1 time
# 5 bools: 4 is_stance, 1 is_walking
SHM_FORMAT = "<61d5?" 
SHM_SIZE = 493  # (61 * 8) + 5
SHM_NAME = "/dev/shm/rex_cmd"

def main():
    # 1. Intentar abrir la memoria compartida
    try:
        fd = os.open(SHM_NAME, os.O_RDWR)
        shm = mmap.mmap(fd, SHM_SIZE)
    except FileNotFoundError:
        print(f"❌ Error: No se encontró {SHM_NAME}. ¿Está corriendo el programa C++?")
        return

    # 2. Configurar los datos para el envío
    target_angle = 15.0      # Grados deseados
    time_to_reach = 3.0      # Segundos para la transición
    
    # Construcción del bloque de 61 doubles
    angles = [target_angle] * 12         # Motores 0-11
    velocities = [20.0] * 12              # Velocidad límite (deg/s)
    accels = [0.0] * 12                  # No necesario en modo estático
    kp_scales = [1.0] * 12               # Rigidez total
    kd_scales = [1.0] * 12               # Amortiguación total
    
    # Unimos todo en una lista plana
    doubles_list = angles + velocities + accels + kp_scales + kd_scales
    doubles_list.append(float(time_to_reach)) # El double número 61
    
    # Booleans: 4 is_stance (True) + 1 is_walking (False)
    bools_list = [True, True, True, True, False]

    print(f">> Enviando {target_angle}° en {time_to_reach} segundos...")
    print(">> Presiona CTRL+C para finalizar.")

    try:
        while True:
            # Empaquetar y escribir
            packed_data = struct.pack(SHM_FORMAT, *doubles_list, *bools_list)
            shm.seek(0)
            shm.write(packed_data)
            
            # 100 Hz es suficiente para mantener el watchdog vivo
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n>> Deteniendo script de Python.")
    finally:
        shm.close()
        os.close(fd)

if __name__ == "__main__":
    main()
