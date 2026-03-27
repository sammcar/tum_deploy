import subprocess
import os
import sys
import shutil

# --- CONFIGURACIÓN DE HARDWARE ---
# Mapeo idéntico al que funciona en tu terminal
PI3HAT_CFG = "1=4,5,6;2=1,2,3;4=7,8,9;5=10,11,12"

# Ruta exacta a la herramienta moteus_tool
MOTEUS_TOOL_PATH = "/home/tum/tum/bin/moteus_tool"

def ejecutar_calibracion(ids_lista):
    """
    Ejecuta el comando de terminal moteus_tool para los IDs dados.
    """
    # Convertimos la lista [1, 2, 3] a string "1,2,3"
    targets_str = ",".join(map(str, ids_lista))
    
    print(f"\n>> Iniciando calibración para IDs: {targets_str}...")
    
    # Construimos el comando
    cmd = [
        "sudo", 
        MOTEUS_TOOL_PATH,
        "--pi3hat-cfg", PI3HAT_CFG,
        "-t", targets_str,
        "--zero-offset"
    ]
    
    # Ejecutamos el comando. check=True lanza una excepción si el comando falla.
    subprocess.run(cmd, check=True)

def main():
    # Definición de las patas según tu mapa
    opciones = {
        "1": {"nombre": "FL (Front Left)",  "ids": [1, 2, 3]},
        "2": {"nombre": "FR (Front Right)", "ids": [4, 5, 6]},
        "3": {"nombre": "BL (Back Left)",   "ids": [7, 8, 9]},
        "4": {"nombre": "BR (Back Right)",  "ids": [10, 11, 12]},
        "5": {"nombre": "TODAS",            "ids": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]}
    }

    while True:
        # Limpiar pantalla (compatible con Linux)
        os.system('clear')
        
        print("========================================")
        print("   CALIBRADOR DE PATAS (ZERO-OFFSET)    ")
        print("========================================")
        print("Coloca la pata en posición 0 antes de elegir.")
        print("----------------------------------------")
        print("  1. FL (Front Left)  [IDs 1, 2, 3]")
        print("  2. FR (Front Right) [IDs 4, 5, 6]")
        print("  3. BL (Back Left)   [IDs 7, 8, 9]")
        print("  4. BR (Back Right)  [IDs 10, 11, 12]")
        print("  5. CALIBRAR TODO EL ROBOT")
        print("  Q. Salir")
        print("----------------------------------------")
        
        seleccion = input("Elige una opción (1-5 o Q): ").strip().upper()

        if seleccion == 'Q':
            print("Saliendo...")
            break
        
        # Bloque corregido con indentación correcta y manejo de errores
        if seleccion in opciones:
            pata = opciones[seleccion]
            
            try:
                print(f"\nHas seleccionado: {pata['nombre']}")
                
                # Llamada a la función de calibración
                ejecutar_calibracion(pata['ids'])
                
                print(f"\n[EXITO] {pata['nombre']} calibrado y guardado correctamente.")
                input("\nPresiona ENTER para volver al menú...")
            
            except subprocess.CalledProcessError as e:
                # Error específico del comando de terminal
                print(f"\n[ERROR] El comando moteus_tool falló con código {e.returncode}.")
                print("Verifica que los cables estén conectados y la batería encendida.")
                input("Presiona ENTER para continuar...")
                
            except Exception as e:
                # Cualquier otro error (ej. archivo no encontrado)
                print(f"\n[ERROR FATAL] Ocurrió un error inesperado:")
                print(f"Detalle: {e}")
                input("Presiona ENTER para continuar...")
        
        else:
            input("\nOpción no válida. Presiona ENTER e intenta de nuevo.")

if __name__ == "__main__":
    # Verificación de permisos de superusuario
    if os.geteuid() != 0:
        print("ERROR: Este script necesita permisos de root para acceder al Pi3Hat.")
        print("Ejecuta: sudo /home/tum/moteus-venv/bin/python3 zeros.py")
        sys.exit(1)
        
    main()
