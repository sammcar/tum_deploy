import asyncio
import moteus
import moteus_pi3hat
import os
import time

async def main():
    # 1. HARDWARE: Tu configuración exacta
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            1: [4, 5, 6],
            2: [1, 2, 3],
            4: [7, 8, 9],
            5: [10, 11, 12],
        }
    )

    # 2. RESOLUCIÓN DE DATOS
    # Configuramos para recibir datos flotantes precisos
    qr = moteus.QueryResolution()
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    qr.torque = moteus.F32
    qr.q_current = moteus.F32

    all_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

    # Vinculamos la resolución al controlador
    servos = {
        id: moteus.Controller(id=id, transport=transport, query_resolution=qr) 
        for id in all_ids
    }

    print("--- MONITOR PASIVO (MODO STOP) ---")
    print("Los motores estarán SUELTOS. Muevelos con la mano.")
    time.sleep(1)

    try:
        while True:
            # 3. EL CAMBIO CLAVE:
            # Usamos make_stop(query=True). 
            # Esto pone el motor en modo "Idle" (suelto) y pide datos.
            # Al no estar en modo posición, no generará torque ni tirones.
            commands = [
                servos[id].make_stop(query=True) 
                for id in all_ids
            ]

            # Enviamos ciclo rápido
            results = await transport.cycle(commands)

            # --- VISUALIZACIÓN ---
            os.system('clear') 
            print(f"{'ID':<3} | {'POS (rev)':<10} | {'VEL (Hz)':<10} | {'TRQ (Nm)':<10} | {'CUR (A)':<10}")
            print("-" * 60)

            results_map = {r.id: r for r in results}

            for id in all_ids:
                motor_data = results_map.get(id)
                
                # Verificamos que el dato sea válido y tenga valores
                if motor_data and motor_data.id in results_map and hasattr(motor_data, 'values'):
                    vals = motor_data.values
                    
                    # Al estar en STOP, el torque debería ser cercano a 0 
                    # a menos que lo estés forzando manualmente.
                    p = vals.get(moteus.Register.POSITION, 0.0)
                    v = vals.get(moteus.Register.VELOCITY, 0.0)
                    t = vals.get(moteus.Register.TORQUE, 0.0)
                    c = vals.get(moteus.Register.Q_CURRENT, 0.0)

                    print(f"{id:<3} | {p:10.4f} | {v:10.4f} | {t:10.4f} | {c:10.4f}")
                else:
                    print(f"{id:<3} | {'-- SIN CONEXIÓN --':^49}")

            await asyncio.sleep(0.02)

    except KeyboardInterrupt:
        print("\nSaliendo...")
    
    finally:
        # Aseguramos parada final
        stop_commands = [servos[id].make_stop() for id in all_ids]
        await transport.cycle(stop_commands)

if __name__ == '__main__':
    asyncio.run(main())
