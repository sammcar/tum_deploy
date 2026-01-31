import time
import curses
# Importación del driver encapsulado
from moteus_driver import MoteusInterface

# --- CONFIGURACIÓN DE HARDWARE ---
BUS_MAP = {1: [4, 5, 6]}
ALL_IDS = [4, 5, 6]

# --- CONFIGURACIÓN DE CONTROL Y SEGURIDAD ---
# Ajusta estos valores según la prueba que vayas a hacer
MAX_TORQUE_LIMIT = 3.0  # Nm - Límite de seguridad para pruebas
KP_SCALE = 1.0          # Rigidez
KD_SCALE = 1.0          # Amortiguación (Evita oscilaciones)

def main(stdscr):
    # 1. INICIALIZAR DRIVER
    # Instanciamos el driver (debug=False porque usamos nuestra propia UI)
    robot = MoteusInterface(BUS_MAP, debug=False)
    
    # Arrancamos el hilo de alta velocidad (200Hz+) en background
    robot.start()
    
    # --- CONFIGURACIÓN UI (Curses) ---
    stdscr.nodelay(True) # No bloquear esperando teclas
    stdscr.timeout(0)
    curses.curs_set(0)   # Ocultar cursor físico
    
    # Variables de la Interfaz
    target_deg = 0.0
    input_buffer = ""
    running = True
    message = f"Sistema listo. Torque Max: {MAX_TORQUE_LIMIT} Nm"

    try:
        while running:
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            
            # 2. OBTENER DATOS (Thread-Safe)
            # Leemos una "foto" instantánea del estado de los motores
            telemetry = robot.get_data()
            hz = robot.get_hz()
            
            # 3. DIBUJAR PANTALLA
            # Header
            status = f" CONTROL MOTEUS | Loop: {hz:.0f} Hz | Errores: {robot.errors}"
            stdscr.addstr(0, 0, status, curses.A_BOLD)
            stdscr.addstr(1, 0, "=" * (w - 1))
            
            # Tabla de datos
            header = f"{'ID':<4} {'POS (Deg)':<12} {'VEL (Rev/s)':<12} {'TORQUE':<10} {'TEMP':<8} {'VOLT'}"
            stdscr.addstr(3, 2, header, curses.A_UNDERLINE)
            
            row = 5
            for id in ALL_IDS:
                motor = telemetry.get(id)
                if motor and motor['pos'] is not None:
                    # Conversiones para visualización
                    p_deg = motor['pos'] * 360.0
                    v_rev = motor['vel']
                    t_nm  = motor['tor']
                    temp  = motor['temp']
                    volt  = motor['volt']
                    fault = motor['fault']
                    
                    # Formato condicional (Resaltar si hay error o alto torque)
                    style = curses.A_NORMAL
                    if fault != 0: 
                        style = curses.A_BOLD | curses.A_STANDOUT
                    elif abs(t_nm) > (MAX_TORQUE_LIMIT * 0.8): # Advertencia visual de torque
                        style = curses.A_BOLD
                    
                    line = f" #{id:<3} {p_deg:8.2f}     {v_rev:8.2f}     {t_nm:6.2f}     {temp:5.1f}C   {volt:4.1f}V"
                    stdscr.addstr(row, 2, line, style)
                    
                    if fault != 0:
                        stdscr.addstr(row, 65, f"FAULT: {fault}", curses.A_BOLD)
                else:
                    stdscr.addstr(row, 2, f" #{id:<3} -- ESPERANDO DATOS --", curses.A_DIM)
                row += 1

            # Zona de Input
            stdscr.addstr(row + 2, 0, "-" * (w - 1))
            stdscr.addstr(row + 3, 2, f"Info: {message}")
            stdscr.addstr(row + 4, 2, f"Objetivo Actual: {target_deg:.1f} grados")
            stdscr.addstr(row + 6, 2, f"Comando >> {input_buffer}_")

            # 4. ENVIAR COMANDOS AL ROBOT
            # Aquí traducimos la intención (grados) a órdenes para el driver
            target_revs = target_deg / 360.0
            
            for id in ALL_IDS:
                robot.set_command(
                    motor_id=id,
                    position=target_revs,
                    velocity=0.0,
                    kp_scale=KP_SCALE,
                    kd_scale=KD_SCALE,
                    feedforward_torque=0.0,
                    max_torque=MAX_TORQUE_LIMIT # <--- AQUI APLICAMOS LA SEGURIDAD
                )

            # 5. GESTIÓN DEL TECLADO
            try:
                key = stdscr.getch()
                if key != -1:
                    # Enter (Confirmar comando)
                    if key in [10, 13]:
                        cmd = input_buffer.strip().lower()
                        if cmd in ['q', 'exit']:
                            running = False
                        elif cmd:
                            try:
                                target_deg = float(cmd)
                                message = f"Nuevo objetivo: {target_deg}°"
                            except ValueError:
                                message = "ERROR: Introduce un número válido."
                        input_buffer = ""
                    
                    # Backspace (Borrar)
                    elif key in [8, 127, curses.KEY_BACKSPACE]:
                        input_buffer = input_buffer[:-1]
                    
                    # Escribir caracteres
                    elif 32 <= key <= 126:
                        input_buffer += chr(key)
            except:
                pass

            # Refresco visual (20Hz es suficiente para el ojo, el motor va a 200Hz por su lado)
            stdscr.refresh()
            time.sleep(0.05)

    except Exception as e:
        # Si algo explota en la UI, lo mostramos al salir
        # Importante: El driver se detendrá en el bloque finally
        pass 
        
    finally:
        # 6. APAGADO LIMPIO
        # Fundamental: Detener el driver para que los motores no queden 'locos'
        robot.stop()
        print("Driver detenido y motores parados.")

if __name__ == '__main__':
    # Wrapper maneja la inicialización/restauración de la terminal
    curses.wrapper(main)
