import asyncio
import moteus
import moteus_pi3hat

# --- CONFIGURACIÓN ---
BUS_MAP = {
    1: [4, 5, 6],
    2: [1, 2, 3],
    4: [7, 8, 9],
    5: [10, 11, 12],
}

# Lista de motores a inspeccionar
IDS_A_REVISAR = [1, 7, 10, 11]

async def main():
    print("--- INSPECTOR DE PARÁMETROS (LÓGICA EXACTA) ---")
    
    # Inicializamos el transporte UNA sola vez para todo el script
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=BUS_MAP)

    for motor_id in IDS_A_REVISAR:
        print(f"\n" + "="*45)
        print(f"   CONSULTANDO MOTOR {motor_id}")
        print("="*45)
        
        # Creamos el controlador y el stream para este ID específico
        c = moteus.Controller(id=motor_id, transport=transport)
        stream = moteus.Stream(c)

        try:
            print(f"Leyendo configuración completa de ID {motor_id}...")
            
            # --- LÓGICA EXACTA QUE TE FUNCIONÓ ---
            # Enviamos 'conf enumerate' y esperamos sin timeout hasta que termine
            response = await stream.command(b"conf enumerate")
            
            texto_completo = response.decode('utf-8', errors='ignore')
            lineas = texto_completo.split('\n')
            
            print(f"--- RESULTADOS (Motor {motor_id}) ---")
            encontrados = 0
            
            for linea in lineas:
                # Buscamos palabras clave
                l = linea.lower()
                
                # Filtramos PID, Corriente, Torque y Velocidad
                condicion = ("current" in l or 
                             "torque" in l or 
                             "velocity" in l or 
                             "pid_position" in l) # Agregué PID para que veas el KP también
                             
                if condicion:
                    print(f" > {linea.strip()}")
                    encontrados += 1
            
            if encontrados == 0:
                print("No se encontraron parámetros con las palabras clave.")
            else:
                print("---------------------------------------------")
                print("VERIFICACIÓN RÁPIDA:")
                if "max_current_a 30" in texto_completo.lower():
                    print(" [OK] Corriente está en 30.0")
                elif "max_current_a" in texto_completo.lower():
                     print(" [!!] ALERTA: La corriente NO es 30.0")

        except Exception as e:
            print(f"Error al leer Motor {motor_id}: {e}")
        
        # Pequeña pausa de seguridad entre motores
        await asyncio.sleep(0.5)

if __name__ == '__main__':
    asyncio.run(main())
