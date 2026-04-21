"""
zeros_auto.py
Versión no-interactiva de zeros.py para uso en arranque automático.
Calibra el zero-offset de los 12 motores del robot sin pedir input.
"""
import subprocess
import os
import sys

# --- CONFIGURACIÓN DE HARDWARE ---
PI3HAT_CFG    = "1=4,5,6;2=1,2,3;4=7,8,9;5=10,11,12"
MOTEUS_TOOL   = "/home/tum/tum/bin/moteus_tool"
ALL_IDS       = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

def ejecutar_calibracion(ids_lista):
    targets_str = ",".join(map(str, ids_lista))
    print(f">> Calibrando IDs: {targets_str}")

    cmd = [
        "sudo",
        MOTEUS_TOOL,
        "--pi3hat-cfg", PI3HAT_CFG,
        "-t", targets_str,
        "--zero-offset"
    ]
    subprocess.run(cmd, check=True)

if __name__ == "__main__":
    if os.geteuid() != 0:
        print("ERROR: Necesita permisos de root.")
        print("Ejecuta: sudo /home/tum/tum/bin/python3 zeros_auto.py")
        sys.exit(1)

    print("========================================")
    print("  AUTO-CALIBRACIÓN: TODOS LOS MOTORES   ")
    print("========================================")

    try:
        ejecutar_calibracion(ALL_IDS)
        print("\n[ÉXITO] Todos los motores calibrados correctamente.")
        sys.exit(0)

    except subprocess.CalledProcessError as e:
        print(f"\n[ERROR] moteus_tool falló con código {e.returncode}.")
        print("Verifica cables y batería.")
        sys.exit(1)

    except Exception as e:
        print(f"\n[ERROR FATAL] {e}")
        sys.exit(1)
