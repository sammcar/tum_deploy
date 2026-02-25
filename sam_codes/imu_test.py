import curses
import serial
import struct
import time
import math

# --- CONFIGURACIÓN INICIAL ---
PORT = '/dev/ttyAMA0'  
BAUD = 115200            
# -----------------------------

# DICCIONARIO DE COMANDOS HEXADECIMALES
CMDS = {
    'UNLOCK':       [0xFF, 0xAA, 0x69, 0x88, 0xB5],
    'SAVE':         [0xFF, 0xAA, 0x00, 0x00, 0x00],
    'CALIB_ACC':    [0xFF, 0xAA, 0x01, 0x01, 0x00],
    'CALIB_MAG_START': [0xFF, 0xAA, 0x01, 0x07, 0x00],
    'CALIB_MAG_END':   [0xFF, 0xAA, 0x01, 0x00, 0x00],
    'RESET_H_YAW':  [0xFF, 0xAA, 0x01, 0x03, 0x00], # Altura y Yaw a 0
    'RATE_10HZ':    [0xFF, 0xAA, 0x03, 0x06, 0x00],
    'RATE_50HZ':    [0xFF, 0xAA, 0x03, 0x08, 0x00],
    'RATE_100HZ':   [0xFF, 0xAA, 0x03, 0x09, 0x00],
    'RATE_200HZ':   [0xFF, 0xAA, 0x03, 0x0B, 0x00],
    'INSTALL_HORIZ':[0xFF, 0xAA, 0x23, 0x00, 0x00],
    'INSTALL_VERT': [0xFF, 0xAA, 0x23, 0x01, 0x00],
    'ALG_6_AXIS':   [0xFF, 0xAA, 0x24, 0x01, 0x00], # Ignora magnetómetro
    'ALG_9_AXIS':   [0xFF, 0xAA, 0x24, 0x00, 0x00], # Usa magnetómetro
}

def send_cmd(ser, cmd_key):
    """Envía desbloqueo + comando"""
    try:
        ser.write(bytearray(CMDS['UNLOCK']))
        time.sleep(0.02)
        ser.write(bytearray(CMDS[cmd_key]))
        time.sleep(0.02)
        return True
    except:
        return False

def euler_to_quat(r, p, y):
    # Deg to Rad
    r, p, y = math.radians(r), math.radians(p), math.radians(y)
    cy, sy = math.cos(y*0.5), math.sin(y*0.5)
    cp, sp = math.cos(p*0.5), math.sin(p*0.5)
    cr, sr = math.cos(r*0.5), math.sin(r*0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, yy, z

def get_bar(val, min_v, max_v, width=12):
    try:
        norm = (val - min_v) / (max_v - min_v)
        fill = int(max(0.0, min(1.0, norm)) * width)
        return '[' + '|' * fill + ' ' * (width - fill) + ']'
    except: return "[]"

def main(stdscr):
    # Configuración Curses
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    # Paleta de colores
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK) # Títulos
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)# Datos clave
    curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)  # Quats
    curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK)# Menú
    curses.init_pair(6, curses.COLOR_RED, curses.COLOR_BLACK)   # Alertas

    # Variables de Estado
    d = {'acc': [0]*3, 'gyro': [0]*3, 'ang': [0]*3, 'quat': [1,0,0,0], 'temp': 0}
    status_msg = "Listo. Presiona teclas para configurar."
    mag_calib_mode = False
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        stdscr.addstr(0,0, f"ERROR: {e}", curses.color_pair(6))
        stdscr.nodelay(False); stdscr.getch(); return

    while True:
        # 1. LECTURA DE DATOS
        while ser.in_waiting:
            if ser.read(1) == b'\x55':
                t = ser.read(1)
                if not t: continue
                pay = ser.read(9)
                if len(pay) != 9: continue
                
                v = struct.unpack('<hhhh', pay[:8])
                dt = ord(t)
                
                if dt == 0x51: # Acc
                    d['acc'] = [x/32768.0*16.0 for x in v[:3]]
                    d['temp'] = v[3]/100.0
                elif dt == 0x52: # Gyro
                    d['gyro'] = [x/32768.0*2000.0 for x in v[:3]]
                elif dt == 0x53: # Angle
                    d['ang'] = [x/32768.0*180.0 for x in v[:3]]
                    d['quat'] = euler_to_quat(*d['ang'])

        # 2. INTERFAZ GRÁFICA
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        
        # Título
        stdscr.addstr(0, 2, f"WIT MOTION CONTROL - {PORT}", curses.color_pair(2)|curses.A_BOLD)
        
        # --- SECCIÓN DATOS (Izquierda) ---
        # Euler
        stdscr.addstr(2, 2, "EULER (Grados)", curses.color_pair(3))
        stdscr.addstr(3, 2, f"R: {d['ang'][0]:>7.2f} {get_bar(d['ang'][0],-180,180)}")
        stdscr.addstr(4, 2, f"P: {d['ang'][1]:>7.2f} {get_bar(d['ang'][1],-90,90)}")
        stdscr.addstr(5, 2, f"Y: {d['ang'][2]:>7.2f} {get_bar(d['ang'][2],-180,180)}")
        
        # Quaterniones
        stdscr.addstr(7, 2, "QUATERNIONES", curses.color_pair(4))
        stdscr.addstr(8, 2, f"w: {d['quat'][0]:.4f}  x: {d['quat'][1]:.4f}")
        stdscr.addstr(9, 2, f"y: {d['quat'][2]:.4f}  z: {d['quat'][3]:.4f}")
        norm = math.sqrt(sum(x*x for x in d['quat']))
        stdscr.addstr(10, 2, f"Norm: {norm:.5f}")

        # Raw Data
        stdscr.addstr(12, 2, "SENSORES RAW", curses.color_pair(1))
        stdscr.addstr(13, 2, f"Acc: {d['acc'][0]:.2f}, {d['acc'][1]:.2f}, {d['acc'][2]:.2f} g")
        stdscr.addstr(14, 2, f"Gyr: {d['gyro'][0]:.0f}, {d['gyro'][1]:.0f}, {d['gyro'][2]:.0f} d/s")
        stdscr.addstr(15, 2, f"Temp: {d['temp']:.1f} C")

        # --- SECCIÓN MENÚ / COMANDOS (Derecha) ---
        x_menu = 45
        stdscr.addstr(2, x_menu, "--- CONFIGURACION ---", curses.color_pair(5))
        
        menu_items = [
            ("[c]", "Calibrar Acelerometro (Dejar plano)"),
            ("[m]", "Calibrar Magnetometro (INICIAR/FIN)"),
            ("[h]", "Instalacion Horizontal"),
            ("[v]", "Instalacion Vertical"),
            ("[z]", "Resetear Altura/Yaw (Z=0)"),
            ("[6]", "Modo 6-Ejes (Sin Mag)"),
            ("[9]", "Modo 9-Ejes (Con Mag)"),
            ("---", "----------------"),
            ("[1]", "Tasa 10 Hz"),
            ("[2]", "Tasa 50 Hz"),
            ("[3]", "Tasa 100 Hz (Default)"),
            ("[4]", "Tasa 200 Hz"),
            ("---", "----------------"),
            ("[s]", "GUARDAR CONFIGURACION (Flash)"),
            ("[q]", "Salir")
        ]
        
        for i, (key, desc) in enumerate(menu_items):
            color = curses.color_pair(1)
            if "GUARDAR" in desc: color = curses.color_pair(3) | curses.A_BOLD
            if "Magnetometro" in desc and mag_calib_mode: 
                desc = "!!! GIRAR SENSOR Y PULSAR 'm' !!!"
                color = curses.color_pair(6) | curses.A_BLINK
            
            stdscr.addstr(4+i, x_menu, f"{key:4} {desc}", color)

        # Barra de Estado Inferior
        stdscr.addstr(h-2, 2, f"ESTADO: {status_msg}", curses.color_pair(3))

        # 3. INPUT TECLADO
        k = stdscr.getch()
        
        if k == -1: pass
        elif k == ord('q'): break
        elif k == ord('s'):
            send_cmd(ser, 'SAVE')
            status_msg = "Configuracion guardada en FLASH."
        elif k == ord('c'):
            status_msg = "Calibrando Acelerometro... (ESPERA)"
            stdscr.refresh(); 
            send_cmd(ser, 'CALIB_ACC')
            time.sleep(2) # Pausa critica
            status_msg = "Acelerometro Calibrado."
        elif k == ord('m'):
            if not mag_calib_mode:
                send_cmd(ser, 'CALIB_MAG_START')
                mag_calib_mode = True
                status_msg = "MODO CALIB MAG: Rota el sensor en 360 grados..."
            else:
                send_cmd(ser, 'CALIB_MAG_END')
                mag_calib_mode = False
                status_msg = "Calibracion Mag Finalizada."
        elif k == ord('z'):
            send_cmd(ser, 'RESET_H_YAW')
            status_msg = "Yaw y Altura reseteados."
        elif k == ord('h'):
            send_cmd(ser, 'INSTALL_HORIZ')
            status_msg = "Modo: Horizontal"
        elif k == ord('v'):
            send_cmd(ser, 'INSTALL_VERT')
            status_msg = "Modo: Vertical"
        elif k == ord('6'):
            send_cmd(ser, 'ALG_6_AXIS')
            status_msg = "Algoritmo: 6 Ejes (Mag OFF)"
        elif k == ord('9'):
            send_cmd(ser, 'ALG_9_AXIS')
            status_msg = "Algoritmo: 9 Ejes (Mag ON)"
        # Tasas
        elif k == ord('1'): send_cmd(ser, 'RATE_10HZ'); status_msg = "Rate: 10 Hz"
        elif k == ord('2'): send_cmd(ser, 'RATE_50HZ'); status_msg = "Rate: 50 Hz"
        elif k == ord('3'): send_cmd(ser, 'RATE_100HZ'); status_msg = "Rate: 100 Hz"
        elif k == ord('4'): send_cmd(ser, 'RATE_200HZ'); status_msg = "Rate: 200 Hz"

        time.sleep(0.01)

if __name__ == "__main__":
    curses.wrapper(main)
