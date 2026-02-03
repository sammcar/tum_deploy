import asyncio
import threading
import time
import math
import moteus # type: ignore
import moteus_pi3hat # type: ignore

class MoteusInterface:
    """
    Wrapper robusto para Moteus con Pi3Hat.
    Incluye:
    1. Arranque Suave (Homing a 0.0)
    2. Apagado Seguro (Landing a -5.0 grados)
    3. Parada de Emergencia (Corte inmediato)
    """
    def __init__(self, bus_map, debug=False):
        self.bus_map = bus_map
        self.motor_ids = [id for ids in bus_map.values() for id in ids]
        self.debug = debug
        
        self._running = False
        self._estop_triggered = False
        self._lock = threading.Lock()
        
        self._telemetry = {id: None for id in self.motor_ids}
        
        # Comandos iniciales
        self._commands = {}
        for id in self.motor_ids:
            self._commands[id] = {
                'pos': 0.0, 'vel': 0.0, 'tor': 0.0, 
                'kp': 1.0, 'kd': 1.0,
                'max_tor': 2.0, 'vel_lim': 2.0, 'acc_lim': 2.0 
            }
            
        self.loop_hz = 0.0
        self.errors = 0
        self._thread = None

        # Mapeo de Registros a Enteros (Para seguridad en lectura)
        self.REG_POS = int(moteus.Register.POSITION)
        self.REG_VEL = int(moteus.Register.VELOCITY)
        self.REG_TOR = int(moteus.Register.TORQUE)
        self.REG_TEMP = int(moteus.Register.TEMPERATURE)
        self.REG_VOLT = int(moteus.Register.VOLTAGE)
        self.REG_FAULT = int(moteus.Register.FAULT)
        self.REG_MODE = int(moteus.Register.MODE)

    def start(self):
        if self._thread is not None: return
        self._running = True
        self._estop_triggered = False 
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        print(f"[MoteusDriver] Iniciando... (Debug={'ON' if self.debug else 'OFF'})")

    def stop(self):
        print("[MoteusDriver] Solicitud de parada normal...")
        self._running = False
        if self._thread: self._thread.join()

    def emergency_stop(self):
        print("\n[? E-STOP ?] ¡PARADA DE EMERGENCIA SOLICITADA!")
        self._estop_triggered = True
        self._running = False

    def set_command(self, motor_id, position, velocity=0.0, feedforward_torque=0.0, 
                    kp_scale=1.0, kd_scale=1.0, max_torque=2.0, 
                    velocity_limit=2.0, accel_limit=2.0):
        """
        Actualiza el comando.
        NOTA: Si no especificas max_torque, se reseteará a 2.0 Nm por seguridad.
        """
        with self._lock:
            if motor_id in self._commands:
                self._commands[motor_id].update({
                    'pos': position, 'vel': velocity, 'tor': feedforward_torque,
                    'kp': kp_scale, 'kd': kd_scale, 'max_tor': max_torque,
                    'vel_lim': velocity_limit, 'acc_lim': accel_limit
                })

    def get_data(self):
        with self._lock: return self._telemetry.copy()

    def get_hz(self): return self.loop_hz

    # --- MOTOR INTERNO ---
    def _run_async_loop(self): 
        try:
            asyncio.run(self._loop_logic())
        except Exception as e:
            print(f"[MoteusDriver CRASH] {e}")

    async def _loop_logic(self):
        transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=self.bus_map)
        servos = {id: moteus.Controller(id=id, transport=transport) for id in self.motor_ids}

        # 1. Parada inicial (Reset de estado)
        await transport.cycle([servos[id].make_stop(query=False) for id in self.motor_ids])
        
        # ===============================================================
        # A. RUTINA DE ARRANQUE (HOMING SUAVE)
        # ===============================================================
        print("[STARTUP] Homing suave a 0.0...")
        for _ in range(50): # 1 segundo aprox
            if not self._running or self._estop_triggered: break
            
            startup_cmds = []
            for id in self.motor_ids:
                cmd = servos[id].make_position(
                    position=0.0, 
                    velocity=0.0, 
                    velocity_limit=0.5,   # <--- BAJAMOS ESTO: Movimiento lento y seguro
                    maximum_torque=2.0, 
                    kp_scale=1.0, kd_scale=1.0,
                    query=True
                )
                startup_cmds.append(cmd)
            
            try:
                # Timeout relajado para arranque
                await asyncio.wait_for(transport.cycle(startup_cmds), timeout=0.09)
            except:
                pass 
            await asyncio.sleep(0.02)
        
        print("[STARTUP] Listo. Control Activo.")

        # ===============================================================
        # B. BUCLE DE CONTROL PRINCIPAL
        # ===============================================================
        last_hz_time = time.time()
        cycles = 0

        try: 
            while self._running:
                moteus_cmds = []
                
                # Copia Thread-Safe de comandos
                with self._lock:
                    current_cmds = {k: v.copy() for k, v in self._commands.items()}

                for id in self.motor_ids:
                    p = current_cmds.get(id)
                    if p:
                        cmd = servos[id].make_position(
                            position=p.get('pos', 0.0),
                            velocity=p.get('vel', 0.0),
                            feedforward_torque=p.get('tor', 0.0),
                            kp_scale=p.get('kp', 1.0),
                            kd_scale=p.get('kd', 1.0),
                            maximum_torque=p.get('max_tor', 2.0),
                            velocity_limit=p.get('vel_lim', 2.0),
                            accel_limit=p.get('acc_lim', 2.0),
                            query=True
                        )
                        moteus_cmds.append(cmd)

                try:
                    # Enviar y Recibir con Timeout protegido
                    results = await asyncio.wait_for(transport.cycle(moteus_cmds), timeout=0.09)
                    
                    with self._lock:
                        for r in results:
                            # Usamos claves ENTERAS para garantizar lectura
                            vals = r.values
                            if r.id in self.motor_ids and vals:
                                self._telemetry[r.id] = {
                                    'pos': vals.get(self.REG_POS),
                                    'vel': vals.get(self.REG_VEL),
                                    'tor': vals.get(self.REG_TOR),
                                    'temp': vals.get(self.REG_TEMP),
                                    'volt': vals.get(self.REG_VOLT),
                                    'fault': vals.get(self.REG_FAULT),
                                    'mode': vals.get(self.REG_MODE)
                                }

                except asyncio.TimeoutError:
                    self.errors += 1
                except Exception as e:
                    # Si falla algo raro, lo imprimimos pero no morimos
                    if cycles % 100 == 0: print(f"[Driver Warning] {e}")

                # Diagnóstico
                if self.debug and cycles % 100 == 0: self._print_debug_info()
                
                cycles += 1
                if cycles % 200 == 0:
                    now = time.time()
                    dt = now - last_hz_time
                    if dt > 0: self.loop_hz = 200.0 / dt
                    last_hz_time = now
                
                await asyncio.sleep(0)

        except Exception as e:
            print(f"\n[CRITICAL ERROR] El driver murió: {e}")

        finally:
            # ===============================================================
            # C. SECUENCIA DE APAGADO (LANDING)
            # ===============================================================
            if self._estop_triggered:
                print("\n[? E-STOP ?] CORTE INMEDIATO.")
                # En E-Stop cortamos torque inmediatamente, no hacemos landing suave
            else:
                print("\n[SHUTDOWN] Moviendo a pose segura (-5 grados)...")
                safe_revs = -5.0 / 360.0
                
                for _ in range(50): # 1 segundo de transición
                    safe_cmds = []
                    for id in self.motor_ids:
                        cmd = servos[id].make_position(
                            position=safe_revs, 
                            velocity=0.0, 
                            velocity_limit=2.0,
                            maximum_torque=5.0, # Torque firme para bajar
                            kp_scale=1.0, 
                            kd_scale=1.0, 
                            query=False # Solo escritura rápida
                        )
                        safe_cmds.append(cmd)
                    try: await transport.cycle(safe_cmds)
                    except: pass 
                    await asyncio.sleep(0.02)
                
            print("ESTADO: Desenergizando motores...")
            stop_cmds = [servos[id].make_stop(query=False) for id in self.motor_ids]
            for _ in range(5):
                try: await transport.cycle(stop_cmds)
                except: pass
                await asyncio.sleep(0.01)

    def _print_debug_info(self):
        print(f"\n--- DEBUG ({self.loop_hz:.0f} Hz) ---")
        with self._lock:
            for id, d in self._telemetry.items():
                if d and d['pos'] is not None:
                    print(f"ID {id}: P={d['pos']:.3f} | T={d['tor']:.2f} | Err={d['fault']}")
        print("-" * 20)
