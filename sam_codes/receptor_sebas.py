import socket
import struct
import mmap
import os
import time

# ================= CONFIGURACIÓN =================
UDP_IP = "192.168.120.114"
UDP_PORT = 5005
SHM_NAME = "/dev/shm/rex_shm" 

# Estructura: 12 doubles (ángulos) + 12 doubles (tiempos) + 1 bool (walking)
# (12 * 8) + (12 * 8) + 1 = 193 bytes
SHM_SIZE = 193 

# ================= INICIAR UDP =================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((UDP_IP, UDP_PORT))
    print(f"📡 Puente UDP activo en puerto {UDP_PORT}")
except Exception as e:
    print(f"❌ No se pudo bindear el puerto {UDP_PORT}: {e}")
    exit(1)

# ================= INICIAR MEMORIA COMPARTIDA =================
print(f"💾 Abriendo Memoria Compartida en {SHM_NAME}...")

try:
    # os.O_CREAT crea el archivo si no existe, os.O_RDWR permite lectura/escritura
    fd = os.open(SHM_NAME, os.O_CREAT | os.O_RDWR, 0o666)
    os.ftruncate(fd, SHM_SIZE)
    shm = mmap.mmap(fd, SHM_SIZE)

    # --- RESET INICIAL A 0.0 ---
    shm.seek(0)
    shm.write(b'\x00' * SHM_SIZE)
    shm.flush()
    print("✅ Memoria Compartida Reseteada a 0.0 y Lista!")

except OSError as e:
    print(f"❌ Error crítico con SHM: {e}")
    exit(1)

# ================= BUCLE PRINCIPAL =================
print("🚀 Esperando datos del generador...")

try:
    while True:
        # 1. Recibir datos del socket
        data, addr = sock.recvfrom(2048) 
        
        try:
            # 2. Decodificar y convertir a lista de flotantes
            data_str = data.decode('utf-8')
            values = [float(x) for x in data_str.split(',')]

            # 3. Validar estructura de 17 valores
            # (12 angs + 4 tiempos + 1 flag walking)
            if len(values) == 17:
                
                # Separar datos
                angles_flat = values[:12]
                leg_times = values[12:16] # tFL, tFR, tBL, tBR
                is_walking = bool(values[16]) 

                # 4. Expandir 4 tiempos (uno por pata) a 12 (uno por motor)
                # Esto es necesario porque la SHM espera un tiempo por cada joint
                times_flat = []
                for t in leg_times:
                    times_flat.extend([t, t, t]) # Repetir tiempo para Cadera, Muslo, Rodilla

                # 5. Empaquetar en binario (IEEE 754 Doubles)
                # '12d' = 12 doubles, '12d' = 12 doubles, '?' = 1 boolean
                packed_data = struct.pack('12d 12d ?', *angles_flat, *times_flat, is_walking)

                # 6. Escribir en Memoria Compartida
                shm.seek(0)
                shm.write(packed_data)
                
                # Debug en consola (se actualiza en la misma línea)
                status = "WALK" if is_walking else "IDLE"
                print(f"\r[OK] Recibido de {addr[0]} | Mode: {status} | T_FL: {leg_times[0]:.3f}s", end="")

            else:
                print(f"\n⚠️ Paquete inválido: {len(values)} valores recibidos (se esperan 17)")

        except ValueError:
            print("\n⚠️ Error: No se pudieron convertir los datos a float")
        except Exception as e:
            print(f"\n❌ Error procesando paquete: {e}")

except KeyboardInterrupt:
    print("\n\n🛑 Cerrando puente de forma segura...")
    
    # Resetear a cero antes de salir por seguridad
    shm.seek(0)
    shm.write(b'\x00' * SHM_SIZE)
    
    shm.close()
    os.close(fd)
    sock.close()
    print("👋 Puente desconectado.")
