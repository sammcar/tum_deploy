import socket
import struct
import mmap
import os
import time

# ================= CONFIGURACIÓN =================
UDP_IP = "0.0.0.0" 
UDP_PORT = 5005
SHM_NAME = "/dev/shm/rex_shm" 
SHM_SIZE = 193 # 12d (ang) + 12d (time) + 1? (bool) = 193 bytes

# ================= INICIAR UDP =================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"📡 Puente UDP activo en puerto {UDP_PORT}")

# ================= INICIAR MEMORIA COMPARTIDA =================
# MODIFICACIÓN: Ahora Python crea la memoria si no existe
print(f"💾 Iniciando Memoria Compartida en {SHM_NAME}...")

try:
    # O_CREAT: Crea el archivo si no existe
    # O_RDWR: Abre para lectura y escritura
    # 0o666: Permisos para que cualquier usuario pueda leer/escribir
    fd = os.open(SHM_NAME, os.O_CREAT | os.O_RDWR, 0o666)
    
    # IMPORTANTE: Definir el tamaño exacto del bloque de memoria
    # Si esto no se hace, el mapa de memoria estará vacío y fallará.
    os.ftruncate(fd, SHM_SIZE)
    
    # Mapear memoria
    shm = mmap.mmap(fd, SHM_SIZE)
    print("✅ Memoria Compartida Lista (Creada/Abierta por Python)!")

except OSError as e:
    print(f"❌ Error crítico con SHM: {e}")
    exit(1)

# ================= BUCLE PRINCIPAL =================
try:
    while True:
        data, addr = sock.recvfrom(1024) 
        
        try:
            # Decodificar string "ang1,ang2...,tFL,tFR,tBL,tBR"
            data_str = data.decode('utf-8')
            values = [float(x) for x in data_str.split(',')]

            # Verificar que tengamos 16 valores (12 Ángulos + 4 Tiempos)
            if len(values) == 16:
                # 1. Extraer Ángulos (Primeros 12)
                angles_flat = values[:12]

                # 2. Extraer Tiempos por Pata (Últimos 4)
                t_fl = values[12]
                t_fr = values[13]
                t_bl = values[14]
                t_br = values[15]

                # 3. EXPANDIR TIEMPOS A MATRIZ 4x3 (Aplanada a 12 valores)
                times_flat = []
                
                # Pata 0 (FL)
                times_flat.extend([t_fl, t_fl, t_fl])
                # Pata 1 (FR)
                times_flat.extend([t_fr, t_fr, t_fr])
                # Pata 2 (BL)
                times_flat.extend([t_bl, t_bl, t_bl])
                # Pata 3 (BR)
                times_flat.extend([t_br, t_br, t_br])

                # 4. EMPAQUETAR BINARIO
                # Formato: 12 doubles (ang) + 12 doubles (time) + 1 bool (stop)
                packed_data = struct.pack('12d 12d ?', *angles_flat, *times_flat, False)

                # 5. ESCRIBIR EN MEMORIA
                shm.seek(0)
                shm.write(packed_data)
                
                print(f"Update OK: FL={t_fl}s") # Debug opcional

            else:
                print(f"⚠️ Paquete incompleto: llegaron {len(values)} valores")

        except ValueError:
            print("⚠️ Error de formato numérico en UDP")
        except Exception as e:
            print(f"❌ Error: {e}")

except KeyboardInterrupt:
    print("\nCerrando puente...")
    shm.close()
    os.close(fd)
    sock.close()
