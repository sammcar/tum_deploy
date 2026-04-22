# tum_quad_gui.py - TUM - Interfaz de Control de Cuadrúpedo
import sys
import json
import asyncio
import signal
import math
import threading
from dataclasses import dataclass
from typing import Optional
import websockets
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import *
import time
try:
    from inputs import get_gamepad
    GAMEPAD_AVAILABLE = True
except ImportError:
    GAMEPAD_AVAILABLE = False
    print("⚠️  'inputs' library not found. Gamepad support disabled.")
    print("   Install with: pip install inputs")

# === CONSTANTS ===
CMD_MAX_RATE_Y = 0.2
CMD_MAX_RATE_Z = math.radians(60)

CMD_MAX_POSE_YAW = math.radians(10)
CMD_MAX_POSE_PITCH = math.radians(10)
CMD_MAX_POSE_ROLL = math.radians(10)  # NUEVO: Límite de Roll
CMD_MAX_POSE_X = 0.05
CMD_MAX_POSE_Y = 0.02
CMD_MAX_POSE_Z = 0.06  # NUEVO: Límite de altura (6 cm arriba/abajo)

@dataclass
class RobotState:
    mode: str = "unknown"
    voltage: float = 0.0
    max_temp: float = 0.0
    joints_count: int = 0
    fault: str = ""
    connected: bool = False
    routine_done: bool = False

# ------------------------------------------------------------------
class GamepadThread(QThread):
    """Thread for reading gamepad input"""
    state_changed = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.running = True
        self.state = {
            "lx": 0, "ly": 0, "rx": 0, "ry": 0,
            "buttons": 0, "hat_x": 0, "hat_y": 0, "hat_pressed": False,
            "hat_up": False, "hat_down": False, "hat_left": False, "hat_right": False,
            "lt": 0, "rt": 0, "lt_pressed": False, "rt_pressed": False
        }

    def run(self):
        if not GAMEPAD_AVAILABLE:
            return

        while self.running:
            try:
                events = get_gamepad()
                for event in events:
                    if not self.running:
                        return

                    if event.ev_type == "Absolute":
                        if event.code == "ABS_X":
                            self.state["lx"] = event.state / 32768.0
                        elif event.code == "ABS_Y":
                            self.state["ly"] = -event.state / 32768.0
                        elif event.code == "ABS_RX":
                            self.state["rx"] = event.state / 32768.0
                        elif event.code == "ABS_RY":
                            self.state["ry"] = -event.state / 32768.0
                        elif event.code == "ABS_HAT0X":
                            self.state["hat_x"] = event.state
                            self.state["hat_left"]  = (event.state == -1)
                            self.state["hat_right"] = (event.state ==  1)
                        elif event.code == "ABS_HAT0Y":
                            self.state["hat_y"] = -event.state
                            self.state["hat_up"]   = (event.state == -1)
                            self.state["hat_down"] = (event.state ==  1)
                        
                        # Gatillos como Eje Analógico (XInput / Genérico)
                        elif event.code in ["ABS_Z", "ABS_BRAKE"]:  
                            old_lt = self.state["lt"]
                            self.state["lt"] = event.state / 255.0
                            self.state["lt_pressed"] = (old_lt < 0.5 and self.state["lt"] >= 0.5)
                        elif event.code in ["ABS_RZ", "ABS_GAS"]:  
                            old_rt = self.state["rt"]
                            self.state["rt"] = event.state / 255.0
                            self.state["rt_pressed"] = (old_rt < 0.5 and self.state["rt"] >= 0.5)

                    elif event.ev_type == "Key":
                        # Mapeo corregido (Intercambiamos WEST/NORTH y agregamos soporte a botones TR2/TL2)
                        bit_map = {
                            "BTN_SOUTH": 0, "BTN_A": 0,    # A
                            "BTN_EAST": 1,  "BTN_B": 1,    # B
                            "BTN_NORTH": 2, "BTN_X": 2,    # Físicamente X mapeado lógico al bit 2
                            "BTN_WEST": 3,  "BTN_Y": 3,    # Físicamente Y mapeado lógico al bit 3
                            "BTN_TL": 4,                   # LB
                            "BTN_TR": 5,                   # RB
                            "BTN_TR2": 6,                  # RT (si es botón digital)
                            "BTN_TL2": 7                   # LT (si es botón digital)
                        }
                        
                        if event.code in bit_map:
                            bit = bit_map[event.code]
                            if event.state:
                                self.state["buttons"] |= (1 << bit)
                            else:
                                self.state["buttons"] &= ~(1 << bit)

                self.state_changed.emit(self.state.copy())
                QThread.msleep(8)

            except Exception as e:
                if not self.running:
                    break
                QThread.msleep(100)

    def stop(self):
        self.running = False
        self.wait(2000)

# ------------------------------------------------------------------

class VirtualJoystick(QWidget):
    """Joystick virtual — visualización + control por mouse"""
    # Señal emitida cuando el usuario mueve o suelta el joystick con el mouse
    mouse_moved = pyqtSignal(float, float)   # (x, y) en [-1, 1]
    mouse_released = pyqtSignal()

    def __init__(self, color="#34495e", handle_color="#0abde3"):
        super().__init__()
        self.setMinimumSize(150, 150)
        self.val_x = 0.0
        self.val_y = 0.0
        self.color = QColor(color)
        self.handle_color = QColor(handle_color)
        self._mouse_active = False          # True mientras el botón está presionado
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def sizeHint(self):
        return QSize(200, 200)

    def set_values(self, x: float, y: float):
        """Actualiza la posición del indicador (llamado por gamepad o externamente)"""
        self.val_x = max(-1.0, min(1.0, x))
        self.val_y = max(-1.0, min(1.0, y))
        self.update()

    # --- Eventos de mouse ---
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._mouse_active = True
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            self._update_from_mouse(event.position())

    def mouseMoveEvent(self, event):
        if self._mouse_active:
            self._update_from_mouse(event.position())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._mouse_active = False
            self.setCursor(Qt.CursorShape.OpenHandCursor)
            self.set_values(0.0, 0.0)
            self.mouse_released.emit()

    def _update_from_mouse(self, pos):
        """Convierte posición del mouse a valores normalizados [-1, 1] y emite señal"""
        size = min(self.width(), self.height())
        cx, cy = self.width() / 2, self.height() / 2
        radius = size / 2 - 15

        dx = (pos.x() - cx) / radius
        dy = (pos.y() - cy) / radius

        # Clampear al círculo unitario
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if dist > 1.0:
            dx /= dist
            dy /= dist

        self.set_values(dx, dy)
        self.mouse_moved.emit(self.val_x, self.val_y)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width, height = self.width(), self.height()
        size = min(width, height)
        cx, cy = width // 2, height // 2
        radius = size // 2 - 15

        # Borde más brillante si el mouse está presionado
        border_color = QColor("#0abde3") if self._mouse_active else QColor("#bdc3c7")
        painter.setPen(QPen(border_color, 2 if not self._mouse_active else 3))
        painter.setBrush(self.color)
        painter.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)

        # Líneas cruzadas centrales
        painter.setPen(QPen(QColor("#bdc3c7"), 1))
        painter.drawLine(cx, cy - radius, cx, cy + radius)
        painter.drawLine(cx - radius, cy, cx + radius, cy)

        # Posición del stick
        stick_radius = 18
        px = cx + int(self.val_x * (radius - stick_radius))
        py = cy + int(self.val_y * (radius - stick_radius))

        # Handle con brillo extra si está activo por mouse
        handle = self.handle_color.lighter(130) if self._mouse_active else self.handle_color
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(handle)
        painter.drawEllipse(px - stick_radius, py - stick_radius, stick_radius * 2, stick_radius * 2)

# ------------------------------------------------------------------
class StatusIndicator(QWidget):
    """LED-style status indicator"""
    
    def __init__(self, label: str):
        super().__init__()
        self.label = label
        self.active = False
        self.setFixedSize(120, 40)

    def set_active(self, active: bool):
        self.active = active
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # LED circle (Verde brillante o gris inactivo)
        color = QColor("#2ecc71") if self.active else QColor("#7f8c8d")
        painter.setBrush(color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(5, 10, 20, 20)

        # Label
        painter.setPen(Qt.GlobalColor.white)
        font = painter.font()
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(30, 25, self.label)

# ------------------------------------------------------------------
class BatteryWidget(QWidget):
    """Battery level indicator with improved visual"""
    
    def __init__(self):
        super().__init__()
        self.voltage = 0.0
        self.percentage = 0
        self.setFixedSize(150, 70)

    def set_voltage(self, voltage: float):
        self.voltage = voltage
        # 10S LiPo: 36V = 0% (safe minimum, 3.6V/cell), 42V = 100% (4.2V/cell max)
        self.percentage = max(0, min(100, int(100 * (voltage - 36.0) / (42.0 - 36.0))))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Battery outline
        painter.setPen(QPen(QColor("#bdc3c7"), 2))
        painter.setBrush(QColor("#34495e"))
        painter.drawRoundedRect(10, 15, 100, 35, 5, 5)
        painter.drawRoundedRect(110, 25, 8, 15, 3, 3)

        # Battery fill with gradient
        fill_width = int(96 * self.percentage / 100)
        
        if self.percentage > 60:
            color = QColor("#2ecc71")  # Green
        elif self.percentage > 30:
            color = QColor("#feca57")  # Yellow
        else:
            color = QColor("#e74c3c")  # Red

        gradient = QLinearGradient(12, 17, 12, 48)
        gradient.setColorAt(0, color.lighter(120))
        gradient.setColorAt(1, color)
        
        painter.setBrush(gradient)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(12, 17, fill_width, 31, 3, 3)

        painter.setPen(Qt.GlobalColor.white)
        font = painter.font()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        text = f"{self.voltage:.1f}V ({self.percentage}%)"
        painter.drawText(15, 62, text)

# ------------------------------------------------------------------
class QuadControlGUI(QMainWindow):
    """Main application window"""
    
    # Signals for thread-safe updates
    update_display_signal = pyqtSignal(float, float, float, float, str)
    update_status_signal = pyqtSignal(dict)

    def __init__(self, robot_ip: str = "192.168.22.14"):
        super().__init__()
        
        self.robot_ip = robot_ip
        self.robot_state = RobotState()
        self.mode = "stop"
        self.max_speed = 0.5  # Start at 0.5 m/s (slower default)

        # Joystick state
        self.joy_state = {
            "lx": 0, "ly": 0, "rx": 0, "ry": 0,
            "buttons": 0, "hat_x": 0, "hat_y": 0, "lt": 0, "rt": 0
        }
        # ¡CAMBIO AQUÍ! En lugar de False, iniciamos en modo velocidad
        self.body_pose_mode = "velocity"   # puede ser: "velocity", "translation", "rotation"
        self.prev_buttons = 0
        self.prev_hat = {"up": False, "down": False, "left": False, "right": False}

        self.ROUTINES = [
            ("kFlexion",    "Flexiones"),
            ("kBaile",      "Baile"),
            ("kSentarse",   "Sentarse"),
            ("kLevantarse", "Levantarse"),
            ("kSaludo",     "Saludo 🐾"),
        ]
        self.routine_index = 0   # rutina seleccionada en el selector (UI)
        self.routine_active_id = self.ROUTINES[0][0]  # routine_id corriendo en el controlador
        self.routine_active = False  # True mientras se ejecuta kRoutine
        self.routine_was_active = False
        # True cuando la rutina terminó sus pasos pero mantiene pose (hold_forever).
        # En este estado el selector se desbloquea para poder lanzar kLevantarse,
        # pero el comando sigue enviando mode:routine para mantener la pose.
        self.routine_holding = False
        
        # Websocket
        self.websocket = None
        self.ws_connected = False
        
        # Settings
        self.enable_strafe = False  
        self.always_step = False
        self.record_data = False
        self.last_trigger_time = 0.0
        # Control por mouse (activo cuando no hay gamepad o como fallback)
        self._mouse_controlling = False   # True si algún joystick está siendo arrastrado por mouse
        
        self.setup_ui()
        self.setup_connections()
        
        # Start threads
        if GAMEPAD_AVAILABLE:
            self.gamepad_thread = GamepadThread()
            self.gamepad_thread.state_changed.connect(self.on_gamepad_update)
            self.gamepad_thread.start()
        
        # Command timer (20Hz)
        self.command_timer = QTimer()
        self.command_timer.timeout.connect(self.send_command)
        self.command_timer.start(50)
        
        # Start websocket in separate thread
        self.loop = asyncio.new_event_loop()
        self.ws_thread = threading.Thread(target=self._run_websocket_loop, daemon=True)
        self.ws_thread.start()

    def setup_ui(self):
        """Build the user interface"""
        self.setWindowTitle(f"TUM  - Control Cuadrúpedo [{self.robot_ip}]")
        self.setMinimumSize(1000, 700)
        
        # Styling - Purple/Blue theme with high contrast
        # Styling - Tema oscuro con acentos Cyan (#0abde3) y Gris (#bdc3c7)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #222f3e;
            }
            QGroupBox {
                color: #ecf0f1;
                border: 2px solid #bdc3c7;
                border-radius: 8px;
                margin-top: 6px;
                font-weight: bold;
                padding-top: 5px;
                background-color: #34495e;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #ffffff;
            }
            QRadioButton {
                color: #ffffff;
                font-size: 13px;
                padding: 4px;
            }
            QRadioButton::indicator {
                width: 16px;
                height: 16px;
            }
            QRadioButton::indicator:unchecked {
                background-color: #222f3e;
                border: 2px solid #bdc3c7;
                border-radius: 8px;
            }
            QRadioButton::indicator:checked {
                background-color: #0abde3;
                border: 2px solid #0abde3;
                border-radius: 8px;
            }
            QLabel {
                color: #ecf0f1;
            }
            QCheckBox {
                color: #ffffff;
                font-size: 13px;
                padding: 5px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
            QCheckBox::indicator:unchecked {
                background-color: #222f3e;
                border: 2px solid #bdc3c7;
                border-radius: 3px;
            }
            QCheckBox::indicator:checked {
                background-color: #0abde3;
                border: 2px solid #0abde3;
                border-radius: 3px;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #222f3e;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #0abde3;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QTextEdit {
                background-color: #34495e;
                color: #ecf0f1;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                font-family: 'Courier New';
                font-size: 11px;
            }
        """)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        header_container = QHBoxLayout()
        
        # Main title
        header = QLabel("PLATAFORMA CUADRUPEDA TUM")
        header.setStyleSheet("""
            color: #ffffff;
            font-size: 28px;
            font-weight: bold;
            padding: 8px;
            background: #34495e;
            border: 2px solid #bdc3c7;
            border-radius: 10px;
        """)
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Control mode indicator
        self.control_mode_label = QLabel("CONTROL DE VELOCIDAD")
        self.control_mode_label.setStyleSheet("""
            color: #feca57;
            font-size: 16px;
            font-weight: bold;
            padding: 8px 15px;
            background-color: #34495e;
            border: 1px solid #bdc3c7;
            border-radius: 8px;
        """)
        self.control_mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.control_mode_label.setCursor(Qt.CursorShape.PointingHandCursor)
        self.control_mode_label.mousePressEvent = lambda e: self._cycle_body_pose_mode()

        self.control_mode_label.setMinimumWidth(200)
        
        header_container.addWidget(header, stretch=1)# === HEADER & STATUS BAR UNIFICADO ===
        header_container = QHBoxLayout()
        header_container.setSpacing(10)
        
        # Main title
        header = QLabel("PLATAFORMA CUADRUPEDA TUM")
        header.setStyleSheet("""
            color: #ffffff;
            font-size: 24px;
            font-weight: bold;
            padding: 8px;
            background: #34495e;
            border: 2px solid #bdc3c7;
            border-radius: 10px;
        """)
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Control mode indicator
        self.control_mode_label = QLabel("CONTROL DE VELOCIDAD")
        self.control_mode_label.setStyleSheet("""
            color: #feca57;
            font-size: 14px;
            font-weight: bold;
            padding: 8px 15px;
            background-color: #34495e;
            border: 1px solid #bdc3c7;
            border-radius: 8px;
        """)
        self.control_mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Instanciar widgets de estado
        self.connection_indicator = StatusIndicator("CONECTADO")
        self.gamepad_indicator = StatusIndicator("MANDO")
        self.battery_widget = BatteryWidget()
        
        # Añadir todo a la misma barra superior para evitar que se oculte
        header_container.addWidget(header, stretch=1)
        header_container.addWidget(self.control_mode_label)
        header_container.addWidget(self.connection_indicator)
        header_container.addWidget(self.gamepad_indicator)
        header_container.addWidget(self.battery_widget)
        
        main_layout.addLayout(header_container)


        header_container.addWidget(self.control_mode_label)
        
        main_layout.addLayout(header_container)

        # === JOYSTICKS (Sección Superior) ===
        joysticks_panel = QVBoxLayout()
        
        vel_label = QLabel("<h2 style='color: #0abde3;'>Estado de Joysticks</h2>")
        vel_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        joysticks_panel.addWidget(vel_label)
        
        joysticks_layout = QHBoxLayout()
        
        # Joystick Izquierdo
        joy_l_layout = QVBoxLayout()
        lbl_l = QLabel("IZQUIERDO")
        lbl_l.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_l.setStyleSheet("color: #bdc3c7; font-weight: bold;")
        self.joy_L = VirtualJoystick(handle_color="#0abde3")  # Cyan
        joy_l_layout.addWidget(lbl_l)
        joy_l_layout.addWidget(self.joy_L, alignment=Qt.AlignmentFlag.AlignCenter)
        
        # Joystick Derecho
        joy_r_layout = QVBoxLayout()
        lbl_r = QLabel("DERECHO")
        lbl_r.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_r.setStyleSheet("color: #bdc3c7; font-weight: bold;")
        self.joy_R = VirtualJoystick(handle_color="#feca57")  # Amarillo
        joy_r_layout.addWidget(lbl_r)
        joy_r_layout.addWidget(self.joy_R, alignment=Qt.AlignmentFlag.AlignCenter)

        # Conectar eventos de mouse de ambos joysticks
        self.joy_L.mouse_moved.connect(self._on_mouse_joy_L)
        self.joy_L.mouse_released.connect(self._on_mouse_joy_L_released)
        self.joy_R.mouse_moved.connect(self._on_mouse_joy_R)
        self.joy_R.mouse_released.connect(self._on_mouse_joy_R_released)
        
        joysticks_layout.addLayout(joy_l_layout)
        joysticks_layout.addLayout(joy_r_layout)
        
        joysticks_panel.addLayout(joysticks_layout)
        
        # Velocity magnitude display
        self.velocity_magnitude_label = QLabel("Velocidad: 0.00 m/s")
        self.velocity_magnitude_label.setStyleSheet("""
            color: #0abde3;
            font-size: 16px;
            font-weight: bold;
            padding: 8px;
            background-color: #34495e;
            border: 1px solid #bdc3c7;
            border-radius: 6px;
        """)
        self.velocity_magnitude_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        joysticks_panel.addWidget(self.velocity_magnitude_label)
        
        main_layout.addLayout(joysticks_panel, stretch=2)

        # === PANELES DE CONTROL (Sección Inferior) ===
        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(10)

        # 1. Mode Selection
        mode_group = QGroupBox("Modo del Robot")
        mode_layout = QGridLayout()
        mode_layout.setSpacing(6)
        self.mode_button_group = QButtonGroup(self)
        
        modes = [
            ("Apagar", "off"), ("Detener", "stop"), ("Reposo", "idle"),
            ("Caminar", "walk"), ("Saltar", "pronk"), ("Sentadilla", "situp")
        ]
        self.mode_radios = {}
        for i, (label, mode_val) in enumerate(modes):
            radio = QRadioButton(label)
            self.mode_button_group.addButton(radio)
            if mode_val == "stop": radio.setChecked(True)
            self.mode_radios[mode_val] = radio
            mode_layout.addWidget(radio, i // 2, i % 2)
        
        # Radio extra para modo rutinas
        self.routine_radio = QRadioButton("▶ Rutinas")
        self.routine_radio.setStyleSheet("color: #feca57; font-weight: bold;")
        self.mode_button_group.addButton(self.routine_radio)
        self.mode_radios["routine"] = self.routine_radio
        mode_layout.addWidget(self.routine_radio, len(modes) // 2, len(modes) % 2)

        self.mode_button_group.buttonClicked.connect(self.on_mode_changed)
        mode_group.setLayout(mode_layout)
        controls_layout.addWidget(mode_group)

        # Panel de rutinas estáticas
        routine_group = QGroupBox("Rutinas Estáticas")
        routine_group.setStyleSheet("""
            QGroupBox {
                color: #feca57;
                border: 2px solid #feca57;
                border-radius: 8px;
                margin-top: 6px;
                font-weight: bold;
                padding-top: 5px;
                background-color: #2c3e50;
            }
            QGroupBox::title { color: #feca57; left: 10px; padding: 0 5px; }
        """)
        routine_layout = QVBoxLayout()
        routine_btn_layout = QGridLayout()
        routine_btn_layout.setSpacing(4)
        self.routine_buttons = []
        for i, (rid, label) in enumerate(self.ROUTINES):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setFixedHeight(32)
            btn.setStyleSheet("""
                QPushButton { background-color:#34495e; color:#ecf0f1;
                    border:1px solid #bdc3c7; border-radius:5px; font-size:11px; font-weight:bold; }
                QPushButton:checked { background-color:#feca57; color:#222f3e; border:2px solid #f9ca24; }
                QPushButton:hover { background-color:#4a6278; }
            """)
            btn.clicked.connect(lambda checked, idx=i: self._select_routine(idx))
            routine_btn_layout.addWidget(btn, i // 2, i % 2)
            self.routine_buttons.append(btn)
        self.routine_buttons[0].setChecked(True)
        # Saludo empieza deshabilitado — solo se activa desde hold de kSentarse
        saludo_idx = next(
            (i for i, (rid, _) in enumerate(self.ROUTINES) if rid == "kSaludo"), None)
        if saludo_idx is not None:
            self.routine_buttons[saludo_idx].setEnabled(False)
            self.routine_buttons[saludo_idx].setStyleSheet("""
                QPushButton { background-color:#2c3e50; color:#7f8c8d;
                    border:1px solid #4a5568; border-radius:5px;
                    font-size:11px; font-weight:bold; }
                QPushButton:disabled { background-color:#2c3e50; color:#7f8c8d; }
            """)
        routine_layout.addLayout(routine_btn_layout)

        self.launch_btn = QPushButton("▶ LANZAR")
        self.launch_btn.setFixedHeight(34)
        self.launch_btn.setStyleSheet("""
            QPushButton { background-color:#e17055; color:#fff; border:none;
                border-radius:6px; font-size:12px; font-weight:bold; }
            QPushButton:hover { background-color:#d63031; }
        """)
        self.launch_btn.clicked.connect(self._launch_routine)
        routine_layout.addWidget(self.launch_btn)

        self.routine_status_label = QLabel("Sin rutina activa")
        self.routine_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.routine_status_label.setStyleSheet("color:#7f8c8d; font-size:10px;")
        routine_layout.addWidget(self.routine_status_label)

        hint = QLabel("Gamepad: cruceta ◀▶ elegir | Y lanzar")
        hint.setStyleSheet("color:#7f8c8d; font-size:9px; font-style:italic;")
        routine_layout.addWidget(hint)
        routine_group.setLayout(routine_layout)
        controls_layout.addWidget(routine_group)

        # 2. Speed Settings
        speed_group = QGroupBox("Límite de Velocidad")
        speed_layout = QVBoxLayout()
        speed_control = QHBoxLayout()
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 20)
        self.speed_slider.setValue(5)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setTickInterval(5)
        
        self.speed_label = QLabel("0.5 m/s")
        self.speed_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #0abde3;")
        self.speed_label.setMinimumWidth(80)
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        
        speed_control.addWidget(QLabel("Lento"))
        speed_control.addWidget(self.speed_slider)
        speed_control.addWidget(QLabel("Rápido"))
        
        speed_layout.addLayout(speed_control)
        speed_layout.addWidget(self.speed_label, alignment=Qt.AlignmentFlag.AlignCenter)
        hint_label = QLabel("Usa gatillos (LT/RT) para ajustar velocidad")
        hint_label.setStyleSheet("color: #bdc3c7; font-size: 10px; font-style: italic;")
        speed_layout.addWidget(hint_label, alignment=Qt.AlignmentFlag.AlignCenter)
        speed_group.setLayout(speed_layout)
        controls_layout.addWidget(speed_group)

       # 3. Movement Options (Mutuamente Exclusivas)
        options_group = QGroupBox("Opciones de Marcha")
        options_layout = QVBoxLayout()
        
        self.strafe_check = QCheckBox("Habilitar Desplazamiento Lateral (Y)")
        self.always_step_check = QCheckBox("Modo Caminata (A)")
        self.record_check = QCheckBox("Grabar datos de telemetría (B)")
        
        # Grupo exclusivo para que solo uno esté activo
        self.options_bg = QButtonGroup(self)
        self.options_bg.setExclusive(False)
        self.options_bg.addButton(self.strafe_check)
        self.options_bg.addButton(self.always_step_check)
        self.options_bg.addButton(self.record_check)
        
        #self.strafe_check.setChecked(True) # Seleccionado por defecto
        
        self.strafe_check.toggled.connect(lambda c: setattr(self, "enable_strafe", c))
        self.always_step_check.toggled.connect(lambda c: setattr(self, "always_step", c))
        self.record_check.toggled.connect(lambda c: setattr(self, "record_data", c))
        
        options_layout.addWidget(self.strafe_check)
        options_layout.addWidget(self.always_step_check)
        options_layout.addWidget(self.record_check)
        options_group.setLayout(options_layout)
        controls_layout.addWidget(options_group)

        # 4. Telemetry & Help (Juntos en una columna)
        info_layout = QVBoxLayout()
        telem_group = QGroupBox("Telemetría")
        telem_inner = QVBoxLayout()
        self.telemetry_text = QTextEdit()
        self.telemetry_text.setReadOnly(True)
        self.telemetry_text.setMaximumHeight(80)
        telem_inner.addWidget(self.telemetry_text)
        telem_group.setLayout(telem_inner)
        
        help_group = QGroupBox("Controles")
        help_inner = QVBoxLayout()
        help_text = QLabel("Sticks: Mover/Rotar | <b>LB: Postura</b><br>"
                           "<b>Cruceta:</b> Modos | <b>X:</b> Detener | <b>RB:</b> Apagar<br>"
                           "<b>LT/RT:</b> Velocidad | <b>Y/A/B:</b> Opciones (Exclusivas)")
        
        help_text.setWordWrap(True)
        help_text.setStyleSheet("font-size: 10px;")
        help_inner.addWidget(help_text)
        help_group.setLayout(help_inner)
        
        info_layout.addWidget(telem_group)
        info_layout.addWidget(help_group)
        controls_layout.addLayout(info_layout)

        main_layout.addLayout(controls_layout, stretch=1)
        
        # === FAULT DISPLAY (Bottom) ===

        # === FAULT DISPLAY (Bottom) ===
        self.fault_label = QLabel()
        self.fault_label.setVisible(False)
        self.fault_label.setStyleSheet("""
            background-color: #ef4444;
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            padding: 12px;
            border-radius: 8px;
        """)
        self.fault_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(self.fault_label)

    def setup_connections(self):
        """Connect signals to slots"""
        self.update_display_signal.connect(self.update_velocity_display)
        self.update_status_signal.connect(self.update_ui_from_status)
    
    def update_velocity_display(self, vx: float, vy: float, wz: float, max_speed: float, body_pose: str):
        """Update velocity magnitude label"""
        # Ya no usamos self.velocity_display
        
        # Update magnitude label
        vel_magnitude = math.sqrt(vx**2 + vy**2)
        yaw_deg = math.degrees(wz)
        
        if abs(yaw_deg) > 1:
            self.velocity_magnitude_label.setText(f"Velocidad: {vel_magnitude:.2f} m/s | Giro: {yaw_deg:+.1f}°/s")
        else:
            self.velocity_magnitude_label.setText(f"Velocidad: {vel_magnitude:.2f} m/s")

    def on_mode_changed(self, button):
        """Handle mode change (from button group)"""
        for mode_val, radio in self.mode_radios.items():
            if radio == button:
                # Validación: solo permitir "stop" si está en "idle"
                if mode_val == "stop" and self.mode not in ["idle", "off"]:
                    # Revertir el cambio
                    self.mode_radios[self.mode].setChecked(True)
                    return
                self.mode = mode_val
                break
    
    def update_control_mode_indicator(self):
        """Update the control mode indicator (velocity / translation / rotation / routine)"""
        if self.body_pose_mode == "translation":
            self.control_mode_label.setText("POSTURA: TRASLACIÓN  ▶")
            self.control_mode_label.setStyleSheet("""
                color: #222f3e;
                font-size: 16px;
                font-weight: bold;
                padding: 8px 15px;
                background-color: #feca57;
                border-radius: 8px;
            """)
        elif self.body_pose_mode == "rotation":
            self.control_mode_label.setText("POSTURA: ROTACIÓN  ▶")
            self.control_mode_label.setStyleSheet("""
                color: #ffffff;
                font-size: 16px;
                font-weight: bold;
                padding: 8px 15px;
                background-color: #9b59b6;
                border-radius: 8px;
            """)
        elif self.body_pose_mode == "routine":
            rid, label = self.ROUTINES[self.routine_index]
            self.control_mode_label.setText(f"RUTINA: {label.upper()}  ▶")
            self.control_mode_label.setStyleSheet("""
                color: #222f3e;
                font-size: 16px;
                font-weight: bold;
                padding: 8px 15px;
                background-color: #e17055;
                border-radius: 8px;
            """)
        else:
            self.control_mode_label.setText("CONTROL DE VELOCIDAD  ▶")
            self.control_mode_label.setStyleSheet("""
                color: #feca57;
                font-size: 16px;
                font-weight: bold;
                padding: 8px 15px;
                background-color: #34495e;
                border: 1px solid #bdc3c7;
                border-radius: 8px;
            """)

    def on_speed_changed(self):
        """Update speed label"""
        self.max_speed = self.speed_slider.value() / 10.0
        self.speed_label.setText(f"{self.max_speed:.1f} m/s")

    def on_gamepad_update(self, state: dict):
        """Handle gamepad state updates"""
        self.joy_state = state
        current_buttons = state["buttons"]
        
        # --- DETECCIÓN DE ESTADO DE BOTONES ---
        lb_pressed_now = bool(current_buttons & (1 << 4))
        lb_pressed_prev = bool(self.prev_buttons & (1 << 4))
        
        # Necesitamos el estado previo de RB para usarlo como botón de apagado
        rb_pressed_now = bool(current_buttons & (1 << 5))
        rb_pressed_prev = bool(self.prev_buttons & (1 << 5))
        
        a_pressed_now = bool(current_buttons & (1 << 0))
        a_pressed_prev = bool(self.prev_buttons & (1 << 0))
        
        b_pressed_now = bool(current_buttons & (1 << 1))
        b_pressed_prev = bool(self.prev_buttons & (1 << 1))
        
        x_pressed_now = bool(current_buttons & (1 << 2))
        x_pressed_prev = bool(self.prev_buttons & (1 << 2))
        
        y_pressed_now = bool(current_buttons & (1 << 3))
        y_pressed_prev = bool(self.prev_buttons & (1 << 3))

        # ¡CORRECCIÓN! Lectura robusta de gatillos (si el análogo pasa el 50% o se presiona el botón digital)
        lt_pressed_now = state.get("lt", 0.0) > 0.5 or bool(current_buttons & (1 << 7))
        rt_pressed_now = state.get("rt", 0.0) > 0.5 or bool(current_buttons & (1 << 6))

        # --- CONTROL DE POSTURA (LB) ---
        # Cicla entre: velocity → translation → rotation → routine → velocity
        if lb_pressed_now and not lb_pressed_prev:
            self._cycle_body_pose_mode()

        # --- OPCIONES INDEPENDIENTES (Y, A, B) ---

        if y_pressed_now and not y_pressed_prev:
            if self.body_pose_mode == "routine":
                self._launch_routine()
            else:
                self.strafe_check.setChecked(not self.strafe_check.isChecked())

        if a_pressed_now and not a_pressed_prev:
            self.always_step_check.setChecked(not self.always_step_check.isChecked())

        if b_pressed_now and not b_pressed_prev:
            self.record_check.setChecked(not self.record_check.isChecked())

        # --- MODOS RÁPIDOS (X = Detener, RB = Apagar) ---
        if x_pressed_now and not x_pressed_prev:
            self.set_mode("stop")

        if rb_pressed_now and not rb_pressed_prev:
            self.set_mode("off")

        self.prev_buttons = current_buttons

        # --- Actualizar la posición visual de los joysticks ---
        if not self.joy_L._mouse_active:
            self.joy_L.set_values(state["lx"], -state["ly"])
            if not self._mouse_controlling:
                self.joy_state["lx"] = state["lx"]
                self.joy_state["ly"] = state["ly"]
        if not self.joy_R._mouse_active:
            self.joy_R.set_values(state["rx"], -state["ry"])
            if not self._mouse_controlling:
                self.joy_state["rx"] = state["rx"]
                self.joy_state["ry"] = state["ry"]
        self.gamepad_indicator.set_active(True)

        # --- SELECCIÓN DE MODO (Cruceta) — flancos independientes ---
        hat_up    = state.get("hat_up",    False)
        hat_down  = state.get("hat_down",  False)
        hat_left  = state.get("hat_left",  False)
        hat_right = state.get("hat_right", False)
        prev_hat_up    = self.prev_hat.get("up",    False)
        prev_hat_down  = self.prev_hat.get("down",  False)
        prev_hat_left  = self.prev_hat.get("left",  False)
        prev_hat_right = self.prev_hat.get("right", False)

        if self.body_pose_mode == "routine":
            # ◀ / ▶ navegan entre rutinas, flanco de subida
            if hat_left  and not prev_hat_left:
                self._select_routine((self.routine_index - 1) % len(self.ROUTINES))
            if hat_right and not prev_hat_right:
                self._select_routine((self.routine_index + 1) % len(self.ROUTINES))
            # ▲ / ▼ no tienen función aquí — usa LB para salir de routine
        else:
            # Comportamiento original con flancos
            if hat_up    and not prev_hat_up:    self.set_mode("idle")
            if hat_down  and not prev_hat_down:  self.set_mode("situp")
            if hat_right and not prev_hat_right: self.set_mode("walk")
            if hat_left  and not prev_hat_left:  self.set_mode("pronk")

        self.prev_hat = {
            "up": hat_up, "down": hat_down,
            "left": hat_left, "right": hat_right
        }

        # --- ¡CORRECCIÓN! Ajuste de velocidad con LT (-) y RT (+) ---
        current_time = time.time()
        if lt_pressed_now and (current_time - self.last_trigger_time > 0.2):
            new_val = max(1, self.speed_slider.value() - 1)
            self.speed_slider.setValue(new_val)
            self.last_trigger_time = current_time
            
        if rt_pressed_now and (current_time - self.last_trigger_time > 0.2):
            new_val = min(20, self.speed_slider.value() + 1)
            self.speed_slider.setValue(new_val)
            self.last_trigger_time = current_time

            
    def set_mode(self, mode: str):
        """Set specific mode and update radio button"""
        if mode in self.mode_radios and mode != self.mode:
            safe_modes = ["idle", "walk", "pronk", "situp", "stop", "off", "routine"]
            if mode in safe_modes:
                self.mode_radios[mode].setChecked(True)
                self.mode = mode
                if mode != "routine":
                    self.routine_active = False
                    self.routine_holding = False
                    self.routine_status_label.setText("Sin rutina activa")
                    self.routine_status_label.setStyleSheet("color:#7f8c8d; font-size:10px;")

    def _cycle_body_pose_mode(self):
        """Cicla entre velocity → translation → rotation → routine → velocity.
        Llamado por clic en el label O por LB en el gamepad."""
        if self.body_pose_mode == "velocity":
            self.body_pose_mode = "translation"
        elif self.body_pose_mode == "translation":
            self.body_pose_mode = "rotation"
        elif self.body_pose_mode == "rotation":
            self.body_pose_mode = "routine"
            # Al entrar en routine, sincronizar el modo del robot
        else:
            self.body_pose_mode = "velocity"
            # Al salir de routine: volver a idle SOLO si no hay una pose mantenida.
            # Si routine_holding=True, el robot sigue en kRoutine con hold_forever —
            # no interrumpir la pose; el usuario puede seguir usando el joystick
            # en modo velocidad mientras el robot mantiene la postura.
            if self.mode == "routine" and not self.routine_holding:
                self.set_mode("idle")
        self.update_control_mode_indicator()

    def _select_routine(self, idx: int):
        """Selecciona una rutina por índice y actualiza el label si estamos en modo routine."""
        if self.routine_active and not self.routine_holding:
            # Bloquear solo mientras la rutina está en progreso, no en hold.
            for i, btn in enumerate(self.routine_buttons):
                btn.setChecked(i == self.routine_index)
            return
        # No permitir seleccionar Saludo si el botón está deshabilitado.
        if not self.routine_buttons[idx].isEnabled():
            return
        self.routine_index = idx
        for i, btn in enumerate(self.routine_buttons):
            btn.setChecked(i == idx)
        if self.body_pose_mode == "routine":
            self.update_control_mode_indicator()

    def _launch_routine(self):
        """Entra en modo rutina y activa la rutina seleccionada"""
        if self.mode != "routine":
            self.set_mode("routine")
        self.routine_active = True
        self.routine_holding = False    # nueva ejecución, ya no estamos en hold
        self.routine_was_active = False  # limpiar para evitar falso positivo inmediato
        self.routine_active_id = self.ROUTINES[self.routine_index][0]  # confirmar selección
        label = self.ROUTINES[self.routine_index][1]
        self.routine_status_label.setText(f"Ejecutando: {label}...")
        self.routine_status_label.setStyleSheet(
            "color:#feca57; font-size:10px; font-weight:bold;")

    # --- Control por mouse ---
    def _on_mouse_joy_L(self, x: float, y: float):
        """Mouse presionado/arrastrado en joystick izquierdo"""
        # Solo actúa si no hay gamepad controlando en este momento
        self._mouse_controlling = True
        self.joy_state["lx"] = x
        self.joy_state["ly"] = -y    # Invertir Y: arriba en pantalla = positivo en el robot

    def _on_mouse_joy_L_released(self):
        """Mouse soltado en joystick izquierdo — vuelve al centro"""
        self.joy_state["lx"] = 0.0
        self.joy_state["ly"] = 0.0
        # Solo apaga el flag si el joystick derecho tampoco está activo
        if not self.joy_R._mouse_active:
            self._mouse_controlling = False

    def _on_mouse_joy_R(self, x: float, y: float):
        """Mouse presionado/arrastrado en joystick derecho"""
        self._mouse_controlling = True
        self.joy_state["rx"] = x
        self.joy_state["ry"] = -y    # Invertir Y igual que el izquierdo

    def _on_mouse_joy_R_released(self):
        """Mouse soltado en joystick derecho — vuelve al centro"""
        self.joy_state["rx"] = 0.0
        self.joy_state["ry"] = 0.0
        if not self.joy_L._mouse_active:
            self._mouse_controlling = False

    def build_command(self) -> dict:
        """Build command dictionary from current state"""
        j = self.joy_state
        
        # Determine if we're in body pose mode
        if self.body_pose_mode == "translation":
            v_R = [0.0, 0.0, 0.0]
            w_R = [0.0, 0.0, 0.0]

            pose = {
                "translation": [
                    j["ly"] * CMD_MAX_POSE_X,      # Stick L Y  → adelante/atrás
                    j["lx"] * CMD_MAX_POSE_Y,      # Stick L X  → izquierda/derecha
                    -j["ry"] * CMD_MAX_POSE_Z        # Stick R Y  → altura
                ],
                "so3": {
                    "w": 1.0,
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }

        elif self.body_pose_mode == "rotation":
            v_R = [0.0, 0.0, 0.0]
            w_R = [0.0, 0.0, 0.0]

            pose = {
                "translation": [0.0, 0.0, 0.0],
                "so3": {
                    "w": 1.0,
                    "x": j["lx"] * CMD_MAX_POSE_ROLL,   # Stick L X → Roll
                    "y": j["ly"] * CMD_MAX_POSE_PITCH,   # Stick L Y → Pitch
                    "z": j["rx"] * CMD_MAX_POSE_YAW      # Stick R X → Yaw
                }
            }

        else:

            # Normal movement
            forward_input = j["ly"]
            vx = forward_input * self.max_speed
            vy = j["lx"] * CMD_MAX_RATE_Y if self.enable_strafe else 0.0
            
            v_R = [vx, vy, 0.0]
            w_R = [0.0, 0.0, j["rx"] * CMD_MAX_RATE_Z]
            pose = None

        # Check if moving
        moving = any(abs(x) > 0.025 for x in v_R + w_R)

        mode_map = {
            "off":     "stopped",
            "stop":    "zero_velocity",
            "idle":    "rest",
            "walk":    "walk" if (moving or self.always_step) else "rest",
            "pronk":   "jump",
            "situp":   "situp",
            "routine": "routine",
        }

        command = {
            "command": {
                "mode": mode_map[self.mode],
                "v_R": v_R,
                "w_R": w_R,
                "log": "enable" if self.record_data else "disable"
            }
        }

        if pose:
            command["command"]["rest"] = {"offset_RB": pose}

        if self.mode == "pronk":
            command["command"]["jump"] = {
                "acceleration": 8.0,
                "repeat": True
            }

        if self.mode == "situp":
            command["command"]["situp"] = {}

        if self.mode == "routine" and self.routine_active:
            # Enviar el id de la rutina que está corriendo, NO el del selector.
            # routine_active_id solo se actualiza en _launch_routine() con Y.
            command["command"]["routine"] = {"routine_id": self.routine_active_id}

        # Update display
        self.update_display_signal.emit(
            v_R[0], v_R[1], w_R[2], 
            self.max_speed, self.body_pose_mode
        )

        return command

    def send_command(self):
        """Send command to robot (called by timer)"""
        command = self.build_command()
        if self.websocket and self.ws_connected:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.websocket.send(json.dumps(command)),
                    self.loop
                )
                future.result(timeout=0.01)
            except Exception:
                pass

    def _run_websocket_loop(self):
        """Run websocket in dedicated thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._websocket_handler())

    async def _websocket_handler(self):
        """Handle websocket connection with auto-reconnect"""
        uri = f"ws://{self.robot_ip}:4778/control"
        
        while True:
            try:
                async with websockets.connect(uri) as ws:
                    self.websocket = ws
                    self.ws_connected = True
                    print(f"✓ Conectado a TUM en {uri}")
                    
                    self.connection_indicator.set_active(True)
                    
                    async for message in ws:
                        try:
                            status = json.loads(message)
                            self.update_status_signal.emit(status)
                        except json.JSONDecodeError:
                            continue
                            
            except Exception as e:
                self.ws_connected = False
                self.connection_indicator.set_active(False)
                print(f"⚠️  Conexión perdida: {e}. Reintentando en 2s...")
                await asyncio.sleep(2)

    @pyqtSlot(dict)
    def update_ui_from_status(self, status: dict):
        """Update UI from robot status (runs in main thread)"""
        try:
            state = status.get("state", {})
            joints = state.get("joints", [])
            robot_info = state.get("robot", {})
            
            self.robot_state.mode = status.get("mode", "unknown")
            # Leer done desde state.routine — más fiable que esperar mode==rest
            routine_state = state.get("routine", {})
            self.robot_state.routine_done = routine_state.get("done", False)
            self.robot_state.voltage = robot_info.get("voltage", 0.0)
            self.robot_state.max_temp = max(
                (j.get("temperature_C", 0) for j in joints), 
                default=0.0
            )
            self.robot_state.joints_count = len(joints)
            self.robot_state.fault = status.get("fault", "")
            self.robot_state.connected = True

            # Rutinas con hold_forever (kSentarse, kLevantarse, kSaludo):
            # done nunca llega a true — se detectan por routine_active_id.
            HOLD_ROUTINES = ("kSentarse", "kLevantarse", "kSaludo")

            if self.robot_state.mode == "routine" and self.routine_active:
                self.routine_was_active = True
                if self.routine_active_id in HOLD_ROUTINES and not self.routine_holding:
                    self.routine_holding = True
                    label = next(
                        (lbl for rid, lbl in self.ROUTINES
                         if rid == self.routine_active_id),
                        self.routine_active_id
                    )
                    self.routine_status_label.setText(f"⏸ Pose: {label}")
                    self.routine_status_label.setStyleSheet(
                        "color:#74b9ff; font-size:10px; font-weight:bold;")

            # Habilitar/deshabilitar botón Saludo según estado del robot.
            # Solo se puede lanzar desde el hold de kSentarse.
            saludo_idx = next(
                (i for i, (rid, _) in enumerate(self.ROUTINES) if rid == "kSaludo"),
                None)
            if saludo_idx is not None:
                saludo_disponible = (
                    self.routine_holding and
                    self.routine_active_id == "kSentarse"
                )
                btn = self.routine_buttons[saludo_idx]
                btn.setEnabled(saludo_disponible)
                btn.setStyleSheet("""
                    QPushButton { background-color:#2ecc71; color:#222f3e;
                        border:2px solid #27ae60; border-radius:5px;
                        font-size:11px; font-weight:bold; }
                    QPushButton:checked { background-color:#feca57; color:#222f3e;
                        border:2px solid #f9ca24; }
                    QPushButton:hover { background-color:#27ae60; }
                    QPushButton:disabled { background-color:#2c3e50; color:#7f8c8d;
                        border:1px solid #4a5568; }
                """ if saludo_disponible else """
                    QPushButton { background-color:#2c3e50; color:#7f8c8d;
                        border:1px solid #4a5568; border-radius:5px;
                        font-size:11px; font-weight:bold; }
                    QPushButton:disabled { background-color:#2c3e50; color:#7f8c8d; }
                """)

            # Rutinas normales (kFlexion, kBaile): detectar fin via routine.done
            # en la telemetría. El controlador se queda en kRoutine con done=true
            # manteniendo la última posición — no transiciona a kRest solo.
            # La GUI detecta done=true, limpia los flags, y manda mode:rest.
            if self.routine_active and self.routine_was_active and \
                    not self.routine_holding and \
                    self.robot_state.routine_done:
                self.routine_active = False
                self.routine_was_active = False
                self.routine_holding = False
                # Mandar mode:rest explícitamente para que el robot vuelva a stand.
                # Se hace forzando self.mode = "idle" ANTES del siguiente build_command.
                self.mode = "idle"
                if "idle" in self.mode_radios:
                    self.mode_radios["idle"].setChecked(True)
                # NO cambiar body_pose_mode — el usuario se queda en modo rutinas
                # para poder lanzar otra rutina sin salir.
                label = next(
                    (lbl for rid, lbl in self.ROUTINES
                     if rid == self.routine_active_id),
                    self.routine_active_id
                )
                self.routine_status_label.setText(f"✓ {label} completada")
                self.routine_status_label.setStyleSheet(
                    "color:#2ecc71; font-size:10px; font-weight:bold;")
            
            self.battery_widget.set_voltage(self.robot_state.voltage)
            
            if self.robot_state.fault:
                self.fault_label.setText(f"⚠️ FALLO: {self.robot_state.fault}")
                self.fault_label.setVisible(True)
            else:
                self.fault_label.setVisible(False)
            
            timing = status.get("timing", {})
            telemetry_lines = [
                f"MODO:       {self.robot_state.mode.upper()}",
                f"TEMP:       {self.robot_state.max_temp:.1f}°C",
                f"BATERÍA:    {self.robot_state.voltage:.2f}V",
                f"MOTORES:    {self.robot_state.joints_count}/12",
                f"CICLO:      {timing.get('cycle_s', 0)*1000:.1f}ms",
                f"PERDIDOS:   {status.get('missing_replies', 0)}",
            ]
            self.telemetry_text.setPlainText("\n".join(telemetry_lines))
            
        except Exception as e:
            print(f"Error updating UI: {e}")

    def closeEvent(self, event):
        """Clean shutdown"""
        print("\n🛑 Cerrando control TUM ...")
        
        self.command_timer.stop()
        
        if GAMEPAD_AVAILABLE and hasattr(self, 'gamepad_thread'):
            self.gamepad_thread.stop()
        
        if self.websocket and self.ws_connected:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.websocket.close(), 
                    self.loop
                )
                future.result(timeout=1.0)
            except:
                pass
        
        self.loop.call_soon_threadsafe(self.loop.stop)
        
        event.accept()
        print("✓ Limpieza completa")

# ------------------------------------------------------------------
def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="TUM - Control de Cuadrúpedo GUI")
    parser.add_argument("--ip", default="192.168.16.47", 
                       help="Dirección IP del robot (default: 192.168.16.47)")
    parser.add_argument("--local", action="store_true",
                       help="Usar localhost (para simulación)")
    args = parser.parse_args()
    
    robot_ip = "localhost" if args.local else args.ip
    
    def signal_handler(sig, frame):
        print("\n⚠️  Ctrl+C detectado - cerrando correctamente...")
        QApplication.instance().quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # En def main():
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#222f3e"))
    palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Base, QColor("#34495e"))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#222f3e"))
    palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Button, QColor("#34495e"))
    palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    app.setPalette(palette)
    
    window = QuadControlGUI(robot_ip)
    window.show()
    
    print(f"""
    ===============================
       TUM - Control Cuadrúpedo
    ===============================
    
    📡 Conectando a {robot_ip}:4778
    🎮 Mando: {'Habilitado' if GAMEPAD_AVAILABLE else 'Deshabilitado'}
    
    Presiona Ctrl+C para salir
    """)
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()