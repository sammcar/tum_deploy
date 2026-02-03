import time
import curses
import sys
# Importamos el driver encapsulado
from moteus_driver import MoteusInterface

# --- CONFIGURACIÓN DE MODOS ---
DEBUG_MODE = False  # <--- CAMBIA A FALSE PARA MODO TEXTO SIMPLE

# --- CONFIGURACIÓN HARDWARE ---
BUS_MAP = {
    1: [4, 5, 6],
    2: [1, 2, 3],
    4: [7, 8, 9],
    5: [10, 11, 12],
}
# Aplanamos la lista de IDs
ALL_IDS = [id for sublist in BUS_MAP.values() for id in sublist]
ALL_IDS.sort()

# --- PARÁMETROS DE CONTROL ---
# Estos valores se pasarán al driver
KP_SCALE = 1.0
KD_SCALE = 1.0
MAX_TORQUE = 2.0 
# Nota: Velocidad y Aceleración límite son gestionadas internamente 
# por el driver para garantizar seguridad en este wrapper simplificado.

# ==========================================
# MODO 1: INTERFAZ VISUAL (Curses)
# ==========================================
def run_curses_mode(robot):
    def ui_loop(stdscr):
        stdscr.nodelay(True)
        curses.curs_set(0)
        
        target_deg = 0.0
        input_buffer = ""
        running = True
        msg = "Modo Debug Activo."

        while running:
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            
            # 1. Leer Telemetría (Thread Safe)
            data = robot.get_data()
            hz = robot.get_hz()
            
            # 2. Header
            stdscr.addstr(0, 0, f" MOTEUS DASHBOARD (12 MOTORES) | Hz: {hz:.0f}", curses.A_BOLD)
            stdscr.addstr(1, 0, "=" * (w-1))
            
            # 3. Tabla de Motores (2 columnas para que quepan los 12)
            col1_x = 2
            col2_x = w // 2 + 2
            
            header = f"{'ID':<3} {'POS°':<8} {'TOR':<6} {'TEMP'}"
            stdscr.addstr(3, col1_x, header, curses.A_UNDERLINE)
            stdscr.addstr(3, col2_x, header, curses.A_UNDERLINE)
            
            for i, motor_id in enumerate(ALL_IDS):
                # Calcular posición en pantalla (2 columnas)
                x = col1_x if i < 6 else col2_x
                y = 4 + (i if i < 6 else i - 6)
                
                info = data.get(motor_id)
                if info and info['pos'] is not None:
                    pos = info['pos'] * 360.0
                    tor = info['tor']
                    temp = info['temp']
                    fault = info['fault']
                    
                    style = curses.A_NORMAL
                    if fault != 0: style = curses.A_BOLD | curses.A_STANDOUT
                    
                    line = f"#{motor_id:<2} {pos:8.1f} {tor:6.1f} {temp:4.1f}C"
                    stdscr.addstr(y, x, line, style)
                else:
                    stdscr.addstr(y, x, f"#{motor_id:<2} -- WAIT --", curses.A_DIM)

            # 4. Input Footer
            footer_y = 11
            stdscr.addstr(footer_y, 0, "-" * (w-1))
            stdscr.addstr(footer_y + 1, 2, f"Info: {msg}")
            stdscr.addstr(footer_y + 2, 2, f"Target Actual: {target_deg:.1f}°")
            stdscr.addstr(footer_y + 4, 2, f"Cmd >> {input_buffer}_")

            # 5. Enviar Comandos
            target_revs = target_deg / 360.0
            for id in ALL_IDS:
                robot.set_command(
                    motor_id=id,
                    position=target_revs,
                    kp_scale=KP_SCALE,
                    kd_scale=KD_SCALE
                )

            # 6. Teclado
            try:
                k = stdscr.getch()
                if k != -1:
                    if k == 10: # Enter
                        cmd = input_buffer.strip().lower()
                        if cmd in ['q', 'exit']: running = False
                        else:
                            try:
                                target_deg = float(cmd)
                                msg = f"Target set: {target_deg}°"
                            except: msg = "Numero invalido"
                        input_buffer = ""
                    elif k == 127: input_buffer = input_buffer[:-1]
                    elif 32 <= k <= 126: input_buffer += chr(k)
            except: pass
            
            stdscr.refresh()
            time.sleep(0.05)

    curses.wrapper(ui_loop)

# ==========================================
# MODO 2: TEXTO SIMPLE (Input Bloqueante)
# ==========================================
def run_simple_mode(robot):
    print("\n" + "="*40)
    print("   CONTROL SIMPLE (DEBUG OFF)")
    print("   El driver mantiene el torque en background.")
    print("="*40)
    
    target_deg = 0.0
    
    try:
        while True:
            # Mostramos estado básico antes de pedir input
            data = robot.get_data()
            # Ejemplo: Mostramos solo el primer motor disponible
            m_demo = data.get(ALL_IDS[0])
            if m_demo and m_demo['pos']:
                print(f"Estado Motor {ALL_IDS[0]}: {m_demo['pos']*360:.1f}° | Temp: {m_demo['temp']:.1f}C")
            
            # INPUT BLOQUEANTE 
            # (Ya no es peligroso porque el driver es un hilo independiente)
            try:
                user_input = input(f"Target actual {target_deg}° >> ").strip().lower()
            except EOFError:
                break
                
            if user_input in ['q', 'exit']:
                break
            
            try:
                target_deg = float(user_input)
                target_revs = target_deg / 360.0
                
                # Actualizamos comandos para todos
                for id in ALL_IDS:
                    robot.set_command(
                        motor_id=id,
                        position=target_revs,
                        kp_scale=KP_SCALE,
                        kd_scale=KD_SCALE
                    )
                print(f" -> Moviendo a {target_deg}°")
                
            except ValueError:
                print("Error: Solo números.")
                
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupción.")

# ==========================================
# MAIN
# ==========================================
def main():
    # 1. Instanciar Driver
    # Pasamos debug=False al driver interno porque la UI la manejamos nosotros
    robot = MoteusInterface(BUS_MAP, debug=False)
    
    # 2. Arrancar Hilo de Seguridad
    robot.start()
    
    # Esperar un momento a que conecte
    time.sleep(1)
    
    try:
        if DEBUG_MODE:
            run_curses_mode(robot)
        else:
            run_simple_mode(robot)
            
    finally:
        # 3. Apagado Limpio
        print("Deteniendo driver...")
        robot.stop()
        print("Listo.")

if __name__ == '__main__':
    main()
