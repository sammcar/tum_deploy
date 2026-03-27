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
CMD_MAX_POSE_YAW = math.radians(20)
CMD_MAX_POSE_PITCH = math.radians(11)
CMD_MAX_POSE_X = 0.05
CMD_MAX_POSE_Y = 0.02

@dataclass
class RobotState:
    mode: str = "unknown"
    voltage: float = 0.0
    max_temp: float = 0.0
    joints_count: int = 0
    fault: str = ""
    connected: bool = False

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
                            old = self.state["hat_x"]
                            self.state["hat_x"] = event.state
                            self.state["hat_pressed"] = (old == 0 and self.state["hat_x"] != 0)
                        elif event.code == "ABS_HAT0Y":
                            old = self.state["hat_y"]
                            self.state["hat_y"] = -event.state
                            self.state["hat_pressed"] = (old == 0 and self.state["hat_y"] != 0)
                        
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
    """Visualizador de Joystick Virtual (Solo visualización)"""
    def __init__(self, color="#34495e", handle_color="#0abde3"):
        super().__init__()
        self.setMinimumSize(150, 150)
        self.val_x = 0.0
        self.val_y = 0.0
        self.color = QColor(color)
        self.handle_color = QColor(handle_color)
        

    def sizeHint(self):
        return QSize(200, 200)

    def set_values(self, x: float, y: float):
        """Actualiza la posición del indicador"""
        self.val_x = max(-1.0, min(1.0, x))
        self.val_y = max(-1.0, min(1.0, y))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        width, height = self.width(), self.height()
        size = min(width, height)
        cx, cy = width // 2, height // 2
        radius = size // 2 - 15

        # Fondo del joystick
        painter.setPen(QPen(QColor("#bdc3c7"), 2))
        painter.setBrush(self.color)
        painter.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)

        # Líneas cruzadas centrales
        painter.drawLine(cx, cy - radius, cx, cy + radius)
        painter.drawLine(cx - radius, cy, cx + radius, cy)

        # Posición del stick
        stick_radius = 18
        px = cx + int(self.val_x * (radius - stick_radius))
        py = cy + int(self.val_y * (radius - stick_radius))

        # Dibujar el "handle"
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(self.handle_color)
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
    update_display_signal = pyqtSignal(float, float, float, float, bool)
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
        self.body_pose_mode = False
        self.prev_buttons = 0  # <--- AÑADIR ESTA LÍNEA AQUÍ
        
        # Websocket
        self.websocket = None
        self.ws_connected = False
        
        # Settings
        self.enable_strafe = True  
        self.always_step = False
        self.record_data = False
        self.last_trigger_time = 0.0
        
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
        
        self.mode_button_group.buttonClicked.connect(self.on_mode_changed)
        mode_group.setLayout(mode_layout)
        controls_layout.addWidget(mode_group)

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
        
        self.strafe_check = QCheckBox("Lateral (Y)")
        self.always_step_check = QCheckBox("Siempre Caminar (A)")
        self.record_check = QCheckBox("Grabar (B)")
        
        # Grupo exclusivo para que solo uno esté activo
        self.options_bg = QButtonGroup(self)
        self.options_bg.setExclusive(True)
        self.options_bg.addButton(self.strafe_check)
        self.options_bg.addButton(self.always_step_check)
        self.options_bg.addButton(self.record_check)
        
        self.strafe_check.setChecked(True) # Seleccionado por defecto
        
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
                           "<b>Cruceta:</b> Modos | <b>X:</b> Detener | <b>RT:</b> Apagar<br>"
                           "<b>LT/RB:</b> Velocidad | <b>Y/A/B:</b> Opciones (Exclusivas)")
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
    
    def update_velocity_display(self, vx: float, vy: float, wz: float, max_speed: float, body_pose: bool):
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
        """Update the control mode indicator (velocity vs pose)"""
        if self.body_pose_mode:
            self.control_mode_label.setText("CONTROL DE POSTURA")
            self.control_mode_label.setStyleSheet("""
                color: #222f3e;
                font-size: 16px;
                font-weight: bold;
                padding: 8px 15px;
                background-color: #feca57;
                border-radius: 8px;
            """)
        else:
            self.control_mode_label.setText("CONTROL DE VELOCIDAD")
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
        
        rb_pressed_now = bool(current_buttons & (1 << 5))
        
        a_pressed_now = bool(current_buttons & (1 << 0))
        a_pressed_prev = bool(self.prev_buttons & (1 << 0))
        
        b_pressed_now = bool(current_buttons & (1 << 1))
        b_pressed_prev = bool(self.prev_buttons & (1 << 1))
        
        x_pressed_now = bool(current_buttons & (1 << 2))
        x_pressed_prev = bool(self.prev_buttons & (1 << 2))
        
        y_pressed_now = bool(current_buttons & (1 << 3))
        y_pressed_prev = bool(self.prev_buttons & (1 << 3))

        # RT/LT con doble redundancia (pueden venir como eje o como botón 6 y 7)
        rt_btn_now = bool(current_buttons & (1 << 6))
        rt_btn_prev = bool(self.prev_buttons & (1 << 6))
        rt_triggered = state.get("rt_pressed", False) or (rt_btn_now and not rt_btn_prev)

        lt_triggered = state.get("lt_pressed", False) or bool(current_buttons & (1 << 7))

        # --- CONTROL DE POSTURA (LB) ---
        old_body_pose = self.body_pose_mode
        if lb_pressed_now and not lb_pressed_prev:
            self.body_pose_mode = not self.body_pose_mode
            
        if old_body_pose != self.body_pose_mode:
            self.update_control_mode_indicator()

        # --- OPCIONES EXCLUSIVAS (Y, A, B) ---
        if y_pressed_now and not y_pressed_prev:
            self.strafe_check.setChecked(True)
            
        if a_pressed_now and not a_pressed_prev:
            self.always_step_check.setChecked(True)
            
        if b_pressed_now and not b_pressed_prev:
            self.record_check.setChecked(True)

        # --- MODOS RÁPIDOS (X = Detener, RT = Apagar) ---
        if x_pressed_now and not x_pressed_prev:
            self.set_mode("stop")
            
        if rt_triggered:
            self.set_mode("off")

        self.prev_buttons = current_buttons

        # --- Actualizar la posición visual de los joysticks ---
        self.joy_L.set_values(state["lx"], -state["ly"])
        self.joy_R.set_values(state["rx"], -state["ry"])
        self.gamepad_indicator.set_active(True)
        
        # --- SELECCIÓN DE MODO (Cruceta original) ---
        if state["hat_pressed"]:
            hat_x = state["hat_x"]
            hat_y = state["hat_y"]
            if abs(hat_y) > abs(hat_x):
                if hat_y > 0: self.set_mode("idle")
                elif hat_y < 0: self.set_mode("situp")
            else:
                if hat_x > 0: self.set_mode("walk")
                elif hat_x < 0: self.set_mode("pronk")

        # --- Ajuste de velocidad con LT (-) y RB (+) ---
        current_time = time.time()
        if lt_triggered and (current_time - self.last_trigger_time > 0.2):
            new_val = max(1, self.speed_slider.value() - 1)
            self.speed_slider.setValue(new_val)
            self.last_trigger_time = current_time
            
        if rb_pressed_now and (current_time - self.last_trigger_time > 0.2):
            new_val = min(20, self.speed_slider.value() + 1)
            self.speed_slider.setValue(new_val)
            self.last_trigger_time = current_time

            
    def set_mode(self, mode: str):
        """Set specific mode and update radio button"""
        if mode in self.mode_radios and mode != self.mode:
            # Validación: solo permitir si es un modo seguro
            safe_modes = ["idle", "walk", "pronk", "situp"]
            if mode in safe_modes:
                self.mode_radios[mode].setChecked(True)
                self.mode = mode

    def build_command(self) -> dict:
        """Build command dictionary from current state"""
        j = self.joy_state
        
        # Determine if we're in body pose mode
        if self.body_pose_mode:
            v_R = [0.0, 0.0, 0.0]
            w_R = [0.0, 0.0, 0.0]
            
            pose = {
                "translation": [
                    j["ly"] * CMD_MAX_POSE_X,
                    j["lx"] * CMD_MAX_POSE_Y,
                    0.0
                ],
                "so3": {
                    "w": 1.0,
                    "x": 0.0,
                    "y": j["ry"] * CMD_MAX_POSE_PITCH,
                    "z": j["rx"] * CMD_MAX_POSE_YAW
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
            "off": "stopped",
            "stop": "zero_velocity",
            "idle": "rest",
            "walk": "walk" if (moving or self.always_step) else "rest",
            "pronk": "jump",
            "situp": "situp"
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
            self.robot_state.voltage = robot_info.get("voltage", 0.0)
            self.robot_state.max_temp = max(
                (j.get("temperature_C", 0) for j in joints), 
                default=0.0
            )
            self.robot_state.joints_count = len(joints)
            self.robot_state.fault = status.get("fault", "")
            self.robot_state.connected = True
            
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