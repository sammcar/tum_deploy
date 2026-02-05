import time
import curses
import sys
from moteus_driver import MoteusInterface

# --- CONFIGURACIÓN DE MODOS ---
DEBUG_MODE = False   # True = Interfaz Visual, False = Texto Simple

# --- CONFIGURACIÓN HARDWARE ---
BUS_MAP = {
    1: [4, 5, 6],
    2: [1, 2, 3],
    4: [7, 8, 9],
    5: [10, 11, 12],
}

# =====================================================
#  CONFIGURACIÓN DE LA PATA A CONTROLAR (EDITAR AQUÍ)
# =====================================================
# Define aquí los IDs de los 3 motores de la pata que quieres probar.
# Ejemplo Pata Frontal Izq (Bus 2): [1, 2, 3]
# Ejemplo Pata Frontal Der (Bus 1): [4, 5, 6]
IDS_PATA = [4, 5, 6] 

# Validamos que todos los IDs existan en el mapa general
ALL_IDS_HARDWARE = [id for sublist in BUS_MAP.values() for id in sublist]
for id in IDS_PATA:
    if id not in ALL_IDS_HARDWARE:
        print(f"Error: El ID {id} no está en el BUS_MAP")
        sys.exit(1)

# --- PARÁMETROS DE CONTROL ---
KP_SCALE = 1.0
KD_SCALE = 1.0
MAX_TORQUE = 2.0 

# ==========================================
# MODO 1: INTERFAZ VISUAL (Curses) - SOLO 1 PATA
# ==========================================
def run_curses_mode(robot):
    def ui_loop(stdscr):
        stdscr.nodelay(True)
        curses.curs_set(0)
        
        target_deg = 0.0
        input_buffer = ""
        running = True
        msg = f"Controlando IDs: {IDS_PATA}"

        while running:
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            
            # 1. Leer Telemetría
            data = robot.get_data()
            hz = robot.get_hz()
            
            # 2. Header
            stdscr.addstr(0, 0, f" TEST DE PATA INDIVIDUAL | Hz: {hz:.0f}", curses.A_BOLD)
            stdscr.addstr(1, 0, "=" * (w-1))
            
            # 3. Tabla de Motores (Simplificada para 3 motores)
            header = f"{'ID':<3} {'POS°':<8} {'TOR':<6} {'TEMP'}"
            stdscr.addstr(3, 2, header, curses.A_UNDERLINE)
            
            for i, motor_id in enumerate(IDS_PATA):
                y = 4 + i
                x = 2
                
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
            footer_y = 9
            stdscr.addstr(footer_y, 0, "-" * (w-1))
            stdscr.addstr(footer_y + 1, 2, f"Estado: {msg}")
            stdscr.addstr(footer_y + 2, 2, f"Target Pata: {target_deg:.1f}°")
            stdscr.addstr(footer_y + 4, 2, f"Cmd >> {input_buffer}_")

            # 5. Enviar Comandos (SOLO A LA PATA SELECCIONADA)
            target_revs = target_deg / 360.0
            for id in IDS_PATA:
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
                                msg = f"Moviendo a: {target_deg}°"
                            except: msg = "Numero invalido"
                        input_buffer = ""
                    elif k == 127: input_buffer = input_buffer[:-1]
                    elif 32 <= k <= 126: input_buffer += chr(k)
            except: pass
            
            stdscr.refresh()
            time.sleep(0.05)

    curses.wrapper(ui_loop)

# ==========================================
# MODO 2: TEXTO SIMPLE
# ==========================================
def run_simple_mode(robot):
    print("\n" + "="*40)
    print(f"   CONTROL SIMPLE - PATA {IDS_PATA}")
    print("="*40)
    
    target_deg = 0.0
    
    try:
        while True:
            # Mostramos estado del primer motor de la pata como referencia
            data = robot.get_data()
            ref_id = IDS_PATA[0]
            m_demo = data.get(ref_id)
            
            if m_demo and m_demo['pos']:
                print(f"Motor Ref #{ref_id}: {m_demo['pos']*360:.1f}° | Temp: {m_demo['temp']:.1f}C")
            
            try:
                user_input = input(f"Target pata {target_deg}° >> ").strip().lower()
            except EOFError:
                break
                
            if user_input in ['q', 'exit']:
                break
            
            try:
                target_deg = float(user_input)
                target_revs = target_deg / 360.0
                
                # Actualizamos comandos SOLO para la pata seleccionada
                for id in IDS_PATA:
                    robot.set_command(
                        motor_id=id,
                        position=target_revs,
                        kp_scale=KP_SCALE,
                        kd_scale=KD_SCALE
                    )
                print(f" -> Motores {IDS_PATA} moviendo a {target_deg}°")
                
            except ValueError:
                print("Error: Solo números.")
                
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupción.")

# ==========================================
# MAIN
# ==========================================
def main():
    # Inicializamos con TODO el mapa de hardware para que el driver sepa que existen,
    # aunque solo mandemos comandos a unos pocos.
    robot = MoteusInterface(BUS_MAP, debug=False)
    
    robot.start()
    time.sleep(1)
    
    try:
        if DEBUG_MODE:
            run_curses_mode(robot)
        else:
            run_simple_mode(robot)
            
    finally:
        print("Deteniendo driver...")
        robot.stop()
        print("Listo.")

if __name__ == '__main__':
    main()
