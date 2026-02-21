import serial
import time

# --- CONFIGURACIÓN ---
PORT = '/dev/ttyAMA0'
BAUD_ACTUAL = 9600
# ---------------------

# Comandos WitMotion (en formato hexadecimal)
UNLOCK = [0xFF, 0xAA, 0x69, 0x88, 0xB5]
SET_115200 = [0xFF, 0xAA, 0x04, 0x06, 0x00]
SAVE = [0xFF, 0xAA, 0x00, 0x00, 0x00]

def send_cmd(ser, cmd, name):
    print(f"Enviando: {name}...")
    ser.write(bytearray(cmd))
    time.sleep(0.2) # Pausa para que el sensor procese

def main():
    try:
        # 1. Abrir puerto a 9600
        ser = serial.Serial(PORT, BAUD_ACTUAL, timeout=1)
        print(f"--- Conectado a {PORT} a {BAUD_ACTUAL} baudios ---")

        # 2. Secuencia de configuración
        send_cmd(ser, UNLOCK, "Desbloqueo (Unlock)")
        send_cmd(ser, SET_115200, "Cambio a 115200 baudios")
        send_cmd(ser, SAVE, "Guardar en Flash")

        print("\n[ÉXITO] El sensor ha sido configurado a 115200.")
        print("[AVISO] El sensor dejará de responder a 9600 ahora mismo.")
        
        ser.close()

    except Exception as e:
        print(f"\n[ERROR] No se pudo acceder al puerto: {e}")
        print("Asegúrate de tener permisos o usa 'sudo'.")

if __name__ == "__main__":
    main()
