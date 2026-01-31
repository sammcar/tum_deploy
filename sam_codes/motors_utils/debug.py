import asyncio
import moteus
import moteus_pi3hat
import time

# Tu mapa de buses
BUS_MAP = {
    1: [4, 5, 6],
    2: [1, 2, 3],
    4: [7, 8, 9],
    5: [10, 11, 12],
}

# Probaremos solo con el Motor 4 (Bus 1)
TEST_ID = 4

async def main():
    print("--- DIAGNÓSTICO CRUDO ---")
    print(f"Conectando a ID {TEST_ID}...")
    
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=BUS_MAP)
    controller = moteus.Controller(id=TEST_ID, transport=transport)

    # 1. Detener motor primero
    await transport.cycle([controller.make_stop(query=False)])
    
    # 2. Enviar comando pidiendo DATOS (Query=True)
    # Usamos un timeout generoso de 0.1s para descartar problemas de tiempo
    print("Enviando comando con Query=True...")
    
    cmd = controller.make_position(
        position=0.0, 
        maximum_torque=0.0, # Solo leer, no hacer fuerza
        query=True
    )
    
    start = time.time()
    result = await transport.cycle([cmd])
    end = time.time()
    
    print(f"Tiempo de ciclo: {(end-start)*1000:.2f} ms")
    print(f"Respuestas recibidas: {len(result)}")
    
    if len(result) > 0:
        item = result[0]
        print(f"\n--- DATOS DEL MOTOR {item.id} ---")
        print(f"Raw Values: {item.values}")
        
        # Intentamos acceder a los registros manualmente para ver si falla ahí
        if item.values:
            pos = item.values.get(moteus.Register.POSITION)
            volt = item.values.get(moteus.Register.VOLTAGE)
            mode = item.values.get(moteus.Register.MODE)
            print(f"Posición decodificada: {pos}")
            print(f"Voltaje decodificado: {volt}")
            print(f"Modo: {mode}")
        else:
            print("ERROR: 'values' está vacío (None). El motor respondió pero sin datos.")
    else:
        print("\n!!! EL MOTOR NO RESPONDIÓ NADA (Lista vacía) !!!")
        print("Posibles causas:")
        print("1. El BUS_MAP está mal (Quizás el ID 4 no está en el puerto 1 del Pi3Hat?)")
        print("2. El cable de datos (CAN L / CAN H) está flojo (TX va, RX falla).")

if __name__ == "__main__":
    asyncio.run(main())
