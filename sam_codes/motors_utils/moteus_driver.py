import asyncio
import threading
import time
import math
import moteus
import moteus_pi3hat

class MoteusInterface:
    """
    Wrapper robusto para Moteus con Pi3Hat.
    """
    def __init__(self, bus_map, debug=False):
        self.bus_map = bus_map
        self.motor_ids = [id for ids in bus_map.values() for id in ids]
        self.debug = debug
        
        self._running = False
        self._lock = threading.Lock()
        
        self._telemetry = {id: None for id in self.motor_ids}
        
        # Comandos con valores por defecto suaves
        self._commands = {}
        for id in self.motor_ids:
            self._commands[id] = {
                'pos': 0.0, 
                'vel': 0.0, 
                'tor': 0.0, 
                'kp': 1.0, 
                'kd': 1.0,
                'max_tor': 2.0,
                'vel_lim': 2.0, # <--- NUEVO: Límite de velocidad (rev/s)
                'acc_lim': 2.0  # <--- NUEVO: Límite de aceleración (rev/s^2)
            }
            
        self.loop_hz = 0.0
        self.errors = 0
        self._thread = None

    def start(self):
        if self._thread is not None: return
        self._running = True
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        print(f"[MoteusDriver] Driver iniciado (Debug={'ON' if self.debug else 'OFF'}).")

    def stop(self):
        self._running = False
        if self._thread: self._thread.join()
        print("[MoteusDriver] Driver detenido.")

    def set_command(self, motor_id, position, velocity=0.0, feedforward_torque=0.0, 
                    kp_scale=1.0, kd_scale=1.0, max_torque=2.0, 
                    velocity_limit=2.0, accel_limit=2.0): # <--- NUEVOS ARGUMENTOS
        """
        Actualiza el comando.
        velocity_limit: Velocidad máxima de la trayectoria (rev/s).
        accel_limit: Aceleración máxima de la trayectoria (rev/s^2).
        """
        with self._lock:
            if motor_id in self._commands:
                self._commands[motor_id]['pos'] = position
                self._commands[motor_id]['vel'] = velocity
                self._commands[motor_id]['tor'] = feedforward_torque
                self._commands[motor_id]['kp'] = kp_scale
                self._commands[motor_id]['kd'] = kd_scale
                self._commands[motor_id]['max_tor'] = max_torque
                self._commands[motor_id]['vel_lim'] = velocity_limit # Guardar
                self._commands[motor_id]['acc_lim'] = accel_limit    # Guardar

    def get_data(self):
        with self._lock: return self._telemetry.copy()

    def get_hz(self): return self.loop_hz

    # --- MOTOR INTERNO ---
    def _run_async_loop(self): asyncio.run(self._loop_logic())

    async def _loop_logic(self):
        transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=self.bus_map)
        servos = {id: moteus.Controller(id=id, transport=transport) for id in self.motor_ids}

        await transport.cycle([servos[id].make_stop(query=False) for id in self.motor_ids])
        
        last_hz_time = time.time()
        cycles = 0

        while self._running:
            moteus_cmds = []
            with self._lock:
                current_cmds = {k: v.copy() for k, v in self._commands.items()}

            for id in self.motor_ids:
                p = current_cmds.get(id)
                if p:
                    cmd = servos[id].make_position(
                        position=p['pos'],
                        velocity=p['vel'],
                        feedforward_torque=p['tor'],
                        kp_scale=p['kp'],
                        kd_scale=p['kd'],
                        maximum_torque=p['max_tor'],
                        velocity_limit=p['vel_lim'], # <--- USAR LÍMITE
                        accel_limit=p['acc_lim'],    # <--- USAR LÍMITE
                        query=True
                    )
                    moteus_cmds.append(cmd)

            try:
                results = await asyncio.wait_for(transport.cycle(moteus_cmds), timeout=0.09)
                with self._lock:
                    for r in results:
                        if r.id in self.motor_ids and r.values:
                            self._telemetry[r.id] = {
                                'pos': r.values.get(moteus.Register.POSITION),
                                'vel': r.values.get(moteus.Register.VELOCITY),
                                'tor': r.values.get(moteus.Register.TORQUE),
                                'temp': r.values.get(moteus.Register.TEMPERATURE),
                                'volt': r.values.get(moteus.Register.VOLTAGE),
                                'fault': r.values.get(moteus.Register.FAULT),
                                'mode': r.values.get(moteus.Register.MODE)
                            }
            except asyncio.TimeoutError:
                self.errors += 1
            except Exception as e:
                print(f"[Driver Error] {e}")

            if self.debug and cycles % 100 == 0: self._print_debug_info()

            cycles += 1
            if cycles % 200 == 0:
                now = time.time()
                dt = now - last_hz_time
                if dt > 0: self.loop_hz = 200.0 / dt
                last_hz_time = now
            
            await asyncio.sleep(0)

        stop_cmds = [servos[id].make_stop(query=False) for id in self.motor_ids]
        await transport.cycle(stop_cmds)

    def _print_debug_info(self):
        print(f"\n--- DEBUG ({self.loop_hz:.0f} Hz) ---")
        with self._lock:
            for id, d in self._telemetry.items():
                if d and d['pos'] is not None:
                    print(f"ID {id}: P={d['pos']:.3f} | T={d['tor']:.2f} | Err={d['fault']}")
        print("-" * 20)
