import tkinter as tk
from tkinter import ttk
import numpy as np
import pygame
import socket
import struct
import mmap
import os
import sys
import signal

SHM_FILE = "/dev/shm/ui_command"
SIZE = 64

# --- CLASE JOYSTICK (Tuya original) ---
class VirtualJoystick(tk.Canvas):
    def __init__(self, parent, size=180, color="#34495e", handle_color="#e74c3c"):
        super().__init__(parent, width=size, height=size, bg="#ecf0f1", highlightthickness=0)
        self.size = size
        self.cx, self.cy = size // 2, size // 2
        self.val_x, self.val_y = 0.0, 0.0
        
        self.create_oval(10, 10, size-10, size-10, outline="#bdc3c7", width=2)
        self.create_line(self.cx, 10, self.cx, size-10, fill="#bdc3c7")
        self.create_line(10, self.cy, size-10, self.cy, fill="#bdc3c7")
        
        self.r = 20
        self.stick = self.create_oval(self.cx-self.r, self.cy-self.r, self.cx+self.r, self.cy+self.r, 
                                      fill=handle_color, outline=color)
        
        self.bind("<B1-Motion>", self._on_move)
        self.bind("<Button-1>", self._on_move)
        self.bind("<ButtonRelease-1>", lambda e: self.reset() if False else None) 

    def _on_move(self, event):
        dx = (event.x - self.cx) / (self.size / 2 - 20)
        dy = (event.y - self.cy) / (self.size / 2 - 20)
        dist = np.sqrt(dx**2 + dy**2)
        if dist > 1.0: dx /= dist; dy /= dist
        self.update_pos(dx, dy)

    def update_pos(self, x, y):
        self.val_x = x
        self.val_y = y
        px = self.cx + (x * (self.size/2 - 20))
        py = self.cy + (y * (self.size/2 - 20))
        self.coords(self.stick, px-self.r, py-self.r, px+self.r, py+self.r)

    def reset(self):
        self.update_pos(0.0, 0.0)

    def get_values(self):
        return self.val_x, self.val_y

# --- CONTROLADOR PRINCIPAL ---
class RobotControllerUI:
    def __init__(self):
        self.is_running = True
        
        # Escuchar Ctrl+C limpiamente
        signal.signal(signal.SIGINT, self._signal_handler)

        self.shm_f = None
        self.shm_mm = None
        self._init_shared_memory()
        
        self.state = {}
        
        # --- RED (UDP SERVER) ---
        self.udp_ip = "0.0.0.0"
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False) 
        self.remote_active = False
        self.prev_buttons = {} 
        
        self.joystick = None
        self.has_controller = False
        self._init_controller()

        self.mode = "STAND"
        self.stand_submode = "TRANSLATION"

        self.smooth_state = {
            'body_x': 0.0, 'body_y': 0.0, 'body_z': 0.18,
            'body_roll': 0.0, 'body_pitch': 0.0, 'body_yaw': 0.0
        }
        
        self.lerp_factor = 0.1
        
        self.root = tk.Tk()
        self.root.title("REX: Ground Station V3 (Network + Local)")
        self.root.geometry("600x820") 
        self.root.configure(bg="#222f3e")
        
        # Capturar el cierre por la "X" de la ventana
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TCombobox", fieldbackground="#34495e", background="#34495e", foreground="white", arrowcolor="white")

        self._setup_ui()
        
        # Bucle para escuchar señales del sistema
        self._check_signals()
        # Bucle de lógica (reducido a 20Hz para no congelar la UI de Linux)
        self.root.after(50, self._start_update_loop)

    def _signal_handler(self, sig, frame):
        """Maneja el Ctrl+C de la terminal"""
        self._on_closing()

    def _check_signals(self):
        """Permite que Tkinter procese las señales de la terminal"""
        if self.is_running:
            self.root.after(200, self._check_signals)

    def _on_closing(self):
        """Cierra el programa limpiamente sin errores rojos"""
        self.is_running = False
        if self.shm_mm:
            self.shm_mm.close()
        if self.shm_f:
            self.shm_f.close()
        self.root.quit()
        self.root.destroy()
        print("\n[OK] Interfaz cerrada correctamente.")
        sys.exit(0)

    def _init_shared_memory(self):
        try:
            if not os.path.exists(SHM_FILE):
                with open(SHM_FILE, "wb") as f:
                    f.write(b'\x00' * SIZE)
            self.shm_f = open(SHM_FILE, "r+b")
            self.shm_mm = mmap.mmap(self.shm_f.fileno(), SIZE)
        except PermissionError:
            print(f"\n[ERROR FATAL] Python no tiene permisos para escribir en {SHM_FILE}.")
            print("👉 EJECUTA EN TERMINAL: sudo rm /dev/shm/ui_command")
            sys.exit(1)

    def _write_shm(self, data):
        if not self.shm_mm or not self.is_running: return
        modes = {"STAND": 0, "CRAWL": 1, "TROT": 2, "JUMP": 3}
        mode_int = modes.get(data.get('gait_mode', 'STAND'), 0)
        try:
            packed_data = struct.pack('<10f i ?',
                float(data.get('vx', 0.0)),
                float(data.get('vy', 0.0)),
                float(data.get('wz', 0.0)),
                float(data.get('body_x', 0.0)),
                float(data.get('body_y', 0.0)),
                float(data.get('body_z', 0.16)),
                float(data.get('body_roll', 0.0)),
                float(data.get('body_pitch', 0.0)),
                float(data.get('body_yaw', 0.0)),
                float(data.get('trot_freq', 2.5)),
                int(mode_int),
                bool(data.get('use_imu', True))
            )
            self.shm_mm.seek(0)
            self.shm_mm.write(packed_data.ljust(SIZE, b'\x00'))
        except Exception:
            pass

    def _init_controller(self):
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.has_controller = True
                print(f"[INFO] Control detectado: {self.joystick.get_name()}")
            else:
                print("[WARN] No se detectó control físico. Usando modo Mouse.")
        except Exception as e:
            self.has_controller = False

    def _read_udp_packet(self):
        try:
            data, addr = self.sock.recvfrom(128) 
            if len(data) < 47: return None 
            unpacked = struct.unpack('<11f2b?', data[:47])
            state = {
                'vx': unpacked[0], 'vy': unpacked[1], 'wz': unpacked[2],
                'body_x': unpacked[3], 'body_y': unpacked[4], 'body_z': unpacked[5],
                'body_roll': unpacked[6], 'body_pitch': unpacked[7], 'body_yaw': unpacked[8],
                'trot_freq': unpacked[9],
                'default_z': unpacked[10], 
                'mode_int': unpacked[11],
                'submode_int': unpacked[12],
                'use_imu': unpacked[13]
            }
            modes = {0: "STAND", 1: "CRAWL", 2: "TROT"}
            submodes = {0: "TRANSLATION", 1: "ROTATION"}
            state['gait_mode'] = modes.get(state['mode_int'], "STAND")
            state['submode'] = submodes.get(state['submode_int'], "TRANSLATION")
            return state
        except: return None

    def _sync_gui_from_state(self, remote_state):
        r_mode = remote_state.get('gait_mode', 'STAND')
        if r_mode != self.mode_var.get():
            self.mode_var.set(r_mode)
            self.mode = r_mode
        self.scale_freq.set(remote_state.get('trot_freq', 2.6))
        self.imu_var.set(remote_state.get('use_imu', False))
        def clamp(n): return max(-1.0, min(1.0, n))
        if self.mode in ["TROT", "CRAWL"]:
            vx = remote_state.get('vx', 0.0)
            vy = remote_state.get('vy', 0.0)
            wz = remote_state.get('wz', 0.0)
            self.joy_L.update_pos(clamp(vy / 0.3), clamp(-vx / 0.5))
            self.joy_R.update_pos(clamp(-wz / 1.5), 0.0)
            self.lbl_sub.config(text="CONTROL: VELOCIDAD (Remoto)")
            self.btn_sub.config(text="[MODO MARCHA ACTIVO]")
        elif self.mode == "STAND":
            r_submode = remote_state.get('submode', 'TRANSLATION')
            self.stand_submode = r_submode 
            bx = remote_state.get('body_x', 0.0)
            by = remote_state.get('body_y', 0.0)
            bz = remote_state.get('body_z', 0.22) 
            roll = remote_state.get('body_roll', 0.0)
            pitch = remote_state.get('body_pitch', 0.0)
            yaw = remote_state.get('body_yaw', 0.0)
            center_z = remote_state.get('default_z', 0.16) 
            if r_submode == "ROTATION":
                self.joy_L.update_pos(clamp(roll / 25.0), clamp(-pitch / 20.0))
                self.joy_R.update_pos(clamp(yaw / 15.0), 0.0)
                self.lbl_sub.config(text="CONTROL: ROTACIÓN (Remoto)")
                self.btn_sub.config(text="[MODO ROTACIÓN ACTIVO]")
            else: 
                self.joy_L.update_pos(clamp(-by / 0.06), clamp(-bx / 0.08))
                self.joy_R.update_pos(0.0, clamp(-(bz - center_z) / 0.09)) 
                self.lbl_sub.config(text="CONTROL: TRASLACIÓN (Remoto)")
                self.btn_sub.config(text="[MODO TRASLACIÓN ACTIVO]")

    def _read_physical_input(self):
        if not self.has_controller: return
        pygame.event.pump()
        axis_lx = self.joystick.get_axis(0)
        axis_ly = self.joystick.get_axis(1)
        axis_rx = self.joystick.get_axis(2) 
        axis_ry = self.joystick.get_axis(3)
        def deadzone(val, threshold=0.08): return val if abs(val) > threshold else 0.0
        lx = deadzone(axis_lx); ly = deadzone(axis_ly)
        rx = deadzone(axis_rx); ry = deadzone(axis_ry)

        if not self.remote_active:
            if abs(lx) > 0 or abs(ly) > 0: self.joy_L.update_pos(lx, ly)
            elif abs(lx) == 0 and abs(ly) == 0: self.joy_L.update_pos(0, 0)
            if abs(rx) > 0 or abs(ry) > 0: self.joy_R.update_pos(rx, ry)
            elif abs(rx) == 0 and abs(ry) == 0: self.joy_R.update_pos(0, 0)

        def is_pressed(btn_idx):
            current = self.joystick.get_button(btn_idx)
            prev = self.prev_buttons.get(btn_idx, 0)
            self.prev_buttons[btn_idx] = current 
            return current and not prev

        if not self.remote_active: 
            if is_pressed(0) and self.mode == "STAND": self.toggle_submode()
            if is_pressed(1): self.mode_var.set("STAND"); self.on_mode_change(None)
            if is_pressed(3): self.mode_var.set("CRAWL"); self.on_mode_change(None)
            if is_pressed(4): self.mode_var.set("TROT"); self.on_mode_change(None)
            if is_pressed(6): self.scale_freq.set(self.scale_freq.get() - 0.1)
            if is_pressed(7): self.scale_freq.set(self.scale_freq.get() + 0.1)
            if is_pressed(10): self.imu_var.set(not self.imu_var.get())
            if is_pressed(13): self.joy_L.reset()
            if is_pressed(14): self.joy_R.reset()
        else:
            for i in range(15): self.prev_buttons[i] = self.joystick.get_button(i)

    def _setup_ui(self):
        f_top = tk.Frame(self.root, bg="#222f3e")
        f_top.pack(pady=15, fill="x")
        self.lbl_mode = tk.Label(f_top, text="ESTADO: INICIANDO...", font=("Segoe UI", 12, "bold"), fg="white", bg="#222f3e")
        self.lbl_mode.pack()
        f_selector = tk.Frame(self.root, bg="#222f3e")
        f_selector.pack(pady=10)
        tk.Label(f_selector, text="SELECCIONAR MARCHA:", fg="#bdc3c7", bg="#222f3e", font=("Arial", 10, "bold")).pack(anchor="w")
        
        self.mode_var = tk.StringVar(value="STAND")
        self.combo_modes = ttk.Combobox(f_selector, textvariable=self.mode_var, 
                                        values=["STAND", "CRAWL", "TROT","JUMP"], 
                                        state="readonly", font=("Arial", 14, "bold"), width=15)
        self.combo_modes.pack(pady=5)
        self.combo_modes.bind("<<ComboboxSelected>>", self.on_mode_change)

        self.lbl_sub = tk.Label(self.root, text="CONTROL: TRASLACIÓN", font=("Segoe UI", 11), fg="#feca57", bg="#222f3e")
        self.lbl_sub.pack()
        self.btn_sub = tk.Button(self.root, text="CAMBIAR A ROTACIÓN", bg="#9b59b6", fg="white", command=self.toggle_submode)
        self.btn_sub.pack(pady=5)

        f_joy = tk.Frame(self.root, bg="#222f3e"); f_joy.pack(fill="x", pady=15)
        f_L = tk.Frame(f_joy, bg="#222f3e"); f_L.pack(side="left", expand=True)
        tk.Label(f_L, text="IZQUIERDO", fg="#bdc3c7", bg="#222f3e").pack()
        self.joy_L = VirtualJoystick(f_L, handle_color="#0abde3")
        self.joy_L.pack()
        tk.Button(f_L, text="RESETEAR L", bg="#ee5253", fg="white", command=self.joy_L.reset).pack(pady=5)

        f_R = tk.Frame(f_joy, bg="#222f3e"); f_R.pack(side="right", expand=True)
        tk.Label(f_R, text="DERECHO", fg="#bdc3c7", bg="#222f3e").pack()
        self.joy_R = VirtualJoystick(f_R, handle_color="#feca57")
        self.joy_R.pack()
        tk.Button(f_R, text="RESETEAR R", bg="#ee5253", fg="white", command=self.joy_R.reset).pack(pady=5)

        f_settings = tk.LabelFrame(self.root, text="AJUSTES DE MARCHA", bg="#222f3e", fg="#ecf0f1", font=("Arial", 10, "bold"))
        f_settings.pack(fill="x", padx=20, pady=10)
        tk.Label(f_settings, text="Frecuencia (Hz)", bg="#222f3e", fg="#bdc3c7").pack(anchor="w", padx=10)
        self.scale_freq = tk.Scale(f_settings, from_=1.0, to=4.0, resolution=0.1, orient="horizontal", 
                                   bg="#222f3e", fg="white", highlightthickness=0, troughcolor="#34495e", activebackground="#0abde3")
        self.scale_freq.set(2.6) 
        self.scale_freq.pack(fill="x", padx=10, pady=5)

        f_hw = tk.Frame(self.root, bg="#222f3e"); f_hw.pack(fill="x", pady=10)
        self.imu_var = tk.BooleanVar(value=False)
        tk.Checkbutton(f_hw, text="ESTABILIZACIÓN IMU", variable=self.imu_var, bg="#222f3e", fg="white", 
                       selectcolor="#222f3e", font=("Arial", 11, "bold")).pack()
        
        f_l = tk.Frame(f_hw, bg="#222f3e"); f_l.pack(pady=5)
        self.leg_vars = {}
        for leg in ['FL', 'FR', 'BL', 'BR']:
            v = tk.BooleanVar(value=False)
            self.leg_vars[leg] = v
            tk.Checkbutton(f_l, text=leg, variable=v, bg="#34495e", fg="white", 
                           selectcolor="#222f3e", indicatoron=0, padx=10).pack(side="left", padx=2)

    def on_mode_change(self, event):
        new_mode = self.mode_var.get()
        self.mode = new_mode
        self.joy_L.reset()
        self.joy_R.reset()
        if new_mode == "STAND":
            self.btn_sub.config(state="normal", bg="#9b59b6")
            self.lbl_sub.config(text=f"CONTROL: {self.stand_submode}")
        else:
            self.btn_sub.config(state="disabled", bg="gray")
            self.lbl_sub.config(text="CONTROL: VELOCIDAD")

    def toggle_submode(self):
        self.joy_L.reset()
        self.joy_R.reset()
        if self.stand_submode == "TRANSLATION":
            self.stand_submode = "ROTATION"
            self.btn_sub.config(text="IR A TRASLACIÓN")
        else:
            self.stand_submode = "TRANSLATION"
            self.btn_sub.config(text="IR A ROTACIÓN")
        self.lbl_sub.config(text=f"CONTROL: {self.stand_submode}")

    def _start_update_loop(self):
        if not self.is_running: return
        try:
            remote_data = self._read_udp_packet()
            out = self.state.copy()
            
            if remote_data:
                self.remote_active = True
                self.lbl_mode.config(text="ESTADO: RECIBIENDO DATOS RASPBERRY", fg="#e056fd")
                self._sync_gui_from_state(remote_data)
                out = remote_data 
                for k in ['body_x','body_y','body_z','body_roll','body_pitch','body_yaw']:
                    if k in remote_data: self.smooth_state[k] = remote_data[k]
            else:
                self.remote_active = False
                status_txt = "ONLINE (CONTROL LOCAL)" if self.has_controller else "ONLINE (MOUSE)"
                color = "#2ecc71" if self.has_controller else "#48dbfb"
                self.lbl_mode.config(text=f"ESTADO: {status_txt}", fg=color)

                self._read_physical_input()
                lx, ly = self.joy_L.get_values()
                rx, ry = self.joy_R.get_values()
                
                target = {
                    'vx': 0.0, 'vy': 0.0, 'wz': 0.0,
                    'body_x': self.smooth_state['body_x'], 
                    'body_y': self.smooth_state['body_y'],
                    'body_z': self.smooth_state['body_z'],
                    'body_roll': self.smooth_state['body_roll'],
                    'body_pitch': self.smooth_state['body_pitch'],
                    'body_yaw': self.smooth_state['body_yaw']
                }

                if self.mode in ["TROT", "CRAWL"]:
                    target['vx'] = -ly * 0.5
                    target['vy'] = lx * 0.3
                    target['wz'] = -rx * 1.5
                elif self.mode == "STAND":
                    if self.stand_submode == "TRANSLATION":
                        target['body_x'] = -ly * 0.08
                        target['body_y'] = -lx * 0.06
                        target['body_z'] = 0.16 + (-ry * 0.09)
                    elif self.stand_submode == "ROTATION":
                        target['body_roll'] = lx * 25
                        target['body_pitch'] = -ly * 20 
                        target['body_yaw'] = rx * 15

                for key in self.smooth_state.keys():
                    diff = target[key] - self.smooth_state[key]
                    self.smooth_state[key] += diff * self.lerp_factor
                    out[key] = self.smooth_state[key]

                out['vx'] = target['vx']; out['vy'] = target['vy']; out['wz'] = target['wz']
                out['gait_mode'] = self.mode 
                out['use_imu'] = self.imu_var.get()
                out['leg_modes'] = {k: v.get() for k, v in self.leg_vars.items()}
                out['trot_freq'] = self.scale_freq.get()

            self.state = out
            self._write_shm(out)
            
            # Repetir el bucle (50ms = 20Hz, ideal para no congelar la GUI en Linux)
            self.root.after(50, self._start_update_loop)
            
        except tk.TclError:
            pass # Si la ventana se cierra durante la actualización, no hacemos nada

if __name__ == "__main__": 
    RobotControllerUI()
    tk.mainloop()