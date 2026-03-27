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
                        elif event.code == "ABS_Z":  # Left trigger
                            old_lt = self.state["lt"]
                            self.state["lt"] = event.state / 255.0
                            # Detect LT press
                            self.state["lt_pressed"] = (old_lt < 0.5 and self.state["lt"] >= 0.5)
                        elif event.code == "ABS_RZ":  # Right trigger
                            old_rt = self.state["rt"]
                            self.state["rt"] = event.state / 255.0
                            # Detect RT press
                            self.state["rt_pressed"] = (old_rt < 0.5 and self.state["rt"] >= 0.5)

                    elif event.ev_type == "Key":
                        # BTN_TL is left bumper (bit 4)
                        if event.code == "BTN_TL":
                            if event.state:
                                self.state["buttons"] |= (1 << 4)
                            else:
                                self.state["buttons"] &= ~(1 << 4)

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
class VelocityDisplay(QWidget):
    """Beautiful velocity visualization widget with responsive sizing"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(300, 300)
        self.setMaximumSize(600, 600)  # Increased from 500 to 600
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.max_speed = 1.0
        self.body_pose_mode = False

    def sizeHint(self):
        return QSize(400, 400)  # Increased from 350 to 400

    def set_values(self, vx: float, vy: float, wz: float, max_speed: float, body_pose: bool = False):
        """Update display values (thread-safe via signal/slot)"""
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.max_speed = max_speed
        self.body_pose_mode = body_pose
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        width = self.width()
        height = self.height()
        center_x = width // 2
        center_y = height // 2
        radius = min(width, height) // 2 - 40

        # Background gradient
        gradient = QRadialGradient(center_x, center_y, radius + 20)
        gradient.setColorAt(0, QColor(45, 45, 95))
        gradient.setColorAt(1, QColor(34, 32, 71))
        painter.setBrush(gradient)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(center_x - radius - 20, center_y - radius - 20,
                           2 * (radius + 20), 2 * (radius + 20))

        # Grid circles
        painter.setPen(QPen(QColor(91, 95, 199), 1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        for i in [0.33, 0.66, 1.0]:
            r = int(radius * i)
            painter.drawEllipse(center_x - r, center_y - r, 2 * r, 2 * r)

        # Axes
        painter.setPen(QPen(QColor(124, 58, 237), 2))
        painter.drawLine(center_x - radius, center_y, center_x + radius, center_y)
        painter.drawLine(center_x, center_y - radius, center_x, center_y + radius)

        # Translation vector (main indicator)
        if not self.body_pose_mode:
            # Calculate position based on velocity
            max_vel = max(abs(self.max_speed), 0.1)
            x_offset = (self.vy / CMD_MAX_RATE_Y) * radius
            y_offset = -(self.vx / max_vel) * radius

            target_x = center_x + x_offset
            target_y = center_y + y_offset

            # Draw velocity vector line
            if abs(x_offset) > 2 or abs(y_offset) > 2:
                painter.setPen(QPen(QColor(167, 139, 250, 200), 3))
                painter.drawLine(center_x, center_y, int(target_x), int(target_y))

            # Draw velocity indicator circle
            indicator_size = 40
            gradient = QRadialGradient(target_x, target_y, indicator_size // 2)
            gradient.setColorAt(0, QColor(167, 139, 250, 220))
            gradient.setColorAt(1, QColor(167, 139, 250, 80))
            painter.setBrush(gradient)
            painter.setPen(QPen(QColor(167, 139, 250), 2))
            painter.drawEllipse(int(target_x - indicator_size // 2),
                               int(target_y - indicator_size // 2),
                               indicator_size, indicator_size)

        # Direction labels with better visibility
        painter.setPen(QColor(196, 181, 253))
        font = painter.font()
        font.setPointSize(11)
        font.setBold(True)
        painter.setFont(font)
        
        # Position labels - FWD and BACK are good, LEFT and RIGHT need to be more inside
        label_distance_vertical = radius + 15
        label_distance_horizontal = radius - 10  # Bring LEFT and RIGHT closer to center
        
        painter.drawText(center_x - 30, center_y - label_distance_vertical, "ADELANTE")
        painter.drawText(center_x - label_distance_horizontal - 45, center_y + 5, "IZQUIERDA")
        painter.drawText(center_x + label_distance_horizontal - 35, center_y + 5, "DERECHA")
        painter.drawText(center_x - 20, center_y + label_distance_vertical + 5, "ATRÁS")

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

        # LED circle
        color = QColor(16, 185, 129) if self.active else QColor(91, 95, 199)
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
        painter.setPen(QPen(QColor(167, 139, 250), 2))
        painter.setBrush(QColor(45, 45, 95))
        painter.drawRoundedRect(10, 15, 100, 35, 5, 5)
        painter.drawRoundedRect(110, 25, 8, 15, 3, 3)  # Terminal

        # Battery fill with gradient
        fill_width = int(96 * self.percentage / 100)
        
        if self.percentage > 60:
            color = QColor(16, 185, 129)  # Green
        elif self.percentage > 30:
            color = QColor(245, 158, 11)  # Yellow
        else:
            color = QColor(239, 68, 68)  # Red

        # Create gradient for fill
        gradient = QLinearGradient(12, 17, 12, 48)
        gradient.setColorAt(0, color.lighter(120))
        gradient.setColorAt(1, color)
        
        painter.setBrush(gradient)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(12, 17, fill_width, 31, 3, 3)

        # Percentage text
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

    def __init__(self, robot_ip: str = "192.168.16.47"):
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
        self.setStyleSheet("""
            QMainWindow {
                background-color: #222047;
            }
            QGroupBox {
                color: #e0e0ff;
                border: 2px solid #5b5fc7;
                border-radius: 8px;
                margin-top: 6px;
                font-weight: bold;
                padding-top: 5px;
                background-color: #2d2d5f;
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
                background-color: #3d3d7a;
                border: 2px solid #5b5fc7;
                border-radius: 8px;
            }
            QRadioButton::indicator:checked {
                background-color: #7c3aed;
                border: 2px solid #a78bfa;
                border-radius: 8px;
            }
            QLabel {
                color: #e0e0ff;
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
                background-color: #3d3d7a;
                border: 2px solid #5b5fc7;
                border-radius: 3px;
            }
            QCheckBox::indicator:checked {
                background-color: #7c3aed;
                border: 2px solid #a78bfa;
                border-radius: 3px;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #3d3d7a;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #7c3aed;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QTextEdit {
                background-color: #2d2d5f;
                color: #e0e0ff;
                border: 1px solid #5b5fc7;
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
        
        # Talentum logo (on the left)
        talentum_logo_label = QLabel()
        try:
            from pathlib import Path
            script_dir = Path(__file__).parent
            talentum_path = script_dir / "gui_assets" / "talentum_logo.jpeg"
            if talentum_path.exists():
                talentum_pixmap = QPixmap(str(talentum_path))
                
                # Calcular el recorte desde el centro
                original_width = talentum_pixmap.width()
                original_height = talentum_pixmap.height()
                
                # Factor de zoom (1.5 = 150%, 2.0 = 200%, etc.)
                zoom_factor = 1.5
                
                crop_width = int(original_width / zoom_factor)
                crop_height = int(original_height / zoom_factor)
                
                # Calcular posición del recorte (centrado)
                x = (original_width - crop_width) // 2
                y = (original_height - crop_height) // 2
                
                # Recortar desde el centro
                cropped = talentum_pixmap.copy(x, y, crop_width, crop_height)
                
                # Escalar al tamaño del contenedor
                talentum_logo_label.setPixmap(cropped.scaled(200, 80, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
            else:
                talentum_logo_label.setText("TALENTUM")
                talentum_logo_label.setStyleSheet("color: #ffffff; font-size: 14px; font-weight: bold;")
        except:
            talentum_logo_label.setText("TALENTUM")
            talentum_logo_label.setStyleSheet("color: #ffffff; font-size: 14px; font-weight: bold;")
        
        talentum_logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        talentum_logo_label.setFixedWidth(200)
        talentum_logo_label.setContentsMargins(0, 0, 0, 0)
        # Main title
        header = QLabel("PLATAFORMA CUADRUPEDA TUM")
        header.setStyleSheet("""
            color: #ffffff;
            font-size: 28px;
            font-weight: bold;
            padding: 8px;
            background: #5b5fc7;
            border-radius: 10px;
        """)
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Control mode indicator (velocity vs pose)
        self.control_mode_label = QLabel("CONTROL DE VELOCIDAD")
        self.control_mode_label.setStyleSheet("""
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            padding: 8px 15px;
            background-color: #5b5fc7;
            border-radius: 8px;
        """)
        self.control_mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.control_mode_label.setMinimumWidth(200)
        
        header_container.addWidget(talentum_logo_label)
        header_container.addWidget(header, stretch=1)
        header_container.addWidget(self.control_mode_label)
        
        main_layout.addLayout(header_container)

        # === TOP BAR - Status Indicators ===
        top_bar = QHBoxLayout()
        
        self.connection_indicator = StatusIndicator("CONECTADO")
        self.gamepad_indicator = StatusIndicator("MANDO")
        self.battery_widget = BatteryWidget()
        
        top_bar.addWidget(self.connection_indicator)
        top_bar.addWidget(self.gamepad_indicator)
        top_bar.addStretch()
        top_bar.addWidget(self.battery_widget)
        
        main_layout.addLayout(top_bar)

        # === MAIN CONTENT ===
        content_layout = QHBoxLayout()
        content_layout.setSpacing(12)
        
        # === LEFT PANEL - Velocity Display ===
        left_panel = QVBoxLayout()
        
        vel_label = QLabel("<h2 style='color: #a78bfa;'>Comando de Velocidad</h2>")
        vel_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_panel.addWidget(vel_label)
        
        self.velocity_display = VelocityDisplay()
        self.velocity_display.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )
        left_panel.addWidget(self.velocity_display, stretch=1)
        
        # Velocity magnitude display (below joystick)
        self.velocity_magnitude_label = QLabel("Velocidad: 0.00 m/s")
        self.velocity_magnitude_label.setStyleSheet("""
            color: #a78bfa;
            font-size: 16px;
            font-weight: bold;
            padding: 8px;
            background-color: #2d2d5f;
            border-radius: 6px;
        """)
        self.velocity_magnitude_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_panel.addWidget(self.velocity_magnitude_label)
        
        left_panel.addStretch()
        
        content_layout.addLayout(left_panel, stretch=1)

        # === RIGHT PANEL - Controls ===
        right_panel = QVBoxLayout()
        right_panel.setSpacing(10)
        
        # Create a container widget for right panel with max width
        right_panel_widget = QWidget()
        right_panel_widget.setMaximumWidth(500)  # Limit width when maximized
        right_panel_widget.setLayout(right_panel)

        # Mode Selection - GRID LAYOUT (2 columns)
        mode_group = QGroupBox("Modo del Robot")
        mode_layout = QGridLayout()
        mode_layout.setSpacing(6)
        mode_layout.setContentsMargins(8, 12, 8, 8)
        
        # Create button group for mutual exclusivity
        self.mode_button_group = QButtonGroup(self)
        
        modes = [
            ("Apagar", "off"),
            ("Detener", "stop"),
            ("Reposo", "idle"),
            ("Caminar", "walk"),
            ("Saltar", "pronk"),
            ("Sentadilla", "situp")
        ]
        
        self.mode_radios = {}
        for i, (label, mode_val) in enumerate(modes):
            radio = QRadioButton(label)
            self.mode_button_group.addButton(radio)
            if mode_val == "stop":
                radio.setChecked(True)
            
            self.mode_radios[mode_val] = radio
            mode_layout.addWidget(radio, i // 2, i % 2)
        
        self.mode_button_group.buttonClicked.connect(self.on_mode_changed)
        
        mode_group.setLayout(mode_layout)
        right_panel.addWidget(mode_group)

        # Speed Settings - SIMPLIFIED (no presets)
        speed_group = QGroupBox("Límite de Velocidad")
        speed_layout = QVBoxLayout()
        speed_layout.setSpacing(8)
        
        # Speed slider
        speed_control = QHBoxLayout()
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 20)  # 0.1 to 2.0 m/s
        self.speed_slider.setValue(5)  # 0.5 m/s default
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setTickInterval(5)
        
        self.speed_label = QLabel("0.5 m/s")
        self.speed_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #a78bfa;")
        self.speed_label.setMinimumWidth(80)
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        
        speed_control.addWidget(QLabel("Lento"))
        speed_control.addWidget(self.speed_slider)
        speed_control.addWidget(QLabel("Rápido"))
        speed_control.addWidget(self.speed_label)
        
        speed_layout.addLayout(speed_control)
        
        # Controller hint
        hint_label = QLabel("Usa gatillos (LT/RT) para ajustar velocidad")
        hint_label.setStyleSheet("color: #c4b5fd; font-size: 10px; font-style: italic;")
        speed_layout.addWidget(hint_label)
        
        speed_group.setLayout(speed_layout)
        right_panel.addWidget(speed_group)

        # Movement Options
        options_group = QGroupBox("Opciones de Movimiento")
        options_layout = QVBoxLayout()
        options_layout.setSpacing(6)
        
        self.strafe_check = QCheckBox("Habilitar Desplazamiento Lateral")
        self.strafe_check.setChecked(True)  # ✅ Default to enabled
        self.strafe_check.toggled.connect(lambda c: setattr(self, "enable_strafe", c))
        
        self.always_step_check = QCheckBox("Siempre Caminar (Modo Caminar)")
        self.always_step_check.toggled.connect(lambda c: setattr(self, "always_step", c))
        
        self.record_check = QCheckBox("Grabar Datos de Telemetría")
        self.record_check.toggled.connect(lambda c: setattr(self, "record_data", c))
        
        options_layout.addWidget(self.strafe_check)
        options_layout.addWidget(self.always_step_check)
        options_layout.addWidget(self.record_check)
        
        options_group.setLayout(options_layout)
        right_panel.addWidget(options_group)

        # Telemetry - COMPACT
        telem_group = QGroupBox("Telemetría")
        telem_layout = QVBoxLayout()
        
        self.telemetry_text = QTextEdit()
        self.telemetry_text.setReadOnly(True)
        self.telemetry_text.setMaximumHeight(150)
        
        telem_layout.addWidget(self.telemetry_text)
        telem_group.setLayout(telem_layout)
        right_panel.addWidget(telem_group)

        # Gamepad Help - COMPACT
        help_group = QGroupBox("Controles del Mando")
        help_layout = QVBoxLayout()
        help_text = QLabel(
            "• <b>Stick Izq:</b> Mover y Desplazamiento<br>"
            "• <b>Stick Der:</b> Rotar<br>"
            "• <b>LB:</b> Modo Control de Postura<br>"
            "• <b>LT/RT:</b> Disminuir/Aumentar Velocidad<br>"
            "• <b>Cruceta:</b> Cambiar Modos"
        )
        help_text.setWordWrap(True)
        help_layout.addWidget(help_text)
        help_group.setLayout(help_layout)
        right_panel.addWidget(help_group)

        right_panel.addStretch()
        
        content_layout.addWidget(right_panel_widget, stretch=1)

        main_layout.addLayout(content_layout, stretch=1)

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
        """Update velocity display and magnitude label"""
        self.velocity_display.set_values(vx, vy, wz, max_speed, body_pose)
        
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
                color: #ffffff;
                font-size: 16px;
                font-weight: bold;
                padding: 8px 15px;
                background-color: #10b981;
                border-radius: 8px;
            """)
        else:
            self.control_mode_label.setText("CONTROL DE VELOCIDAD")
            self.control_mode_label.setStyleSheet("""
                color: #ffffff;
                font-size: 16px;
                font-weight: bold;
                padding: 8px 15px;
                background-color: #5b5fc7;
                border-radius: 8px;
            """)

    def on_speed_changed(self):
        """Update speed label"""
        self.max_speed = self.speed_slider.value() / 10.0
        self.speed_label.setText(f"{self.max_speed:.1f} m/s")

    def on_gamepad_update(self, state: dict):
        """Handle gamepad state updates"""
        self.joy_state = state
        old_body_pose = self.body_pose_mode
        self.body_pose_mode = bool(state["buttons"] & (1 << 4))
        
        # Update control mode indicator if body pose mode changed
        if old_body_pose != self.body_pose_mode:
            self.update_control_mode_indicator()
        
        # Update gamepad indicator
        self.gamepad_indicator.set_active(True)
        
        # Handle mode selection with D-pad (cruceta)
        if state["hat_pressed"]:
            hat_x = state["hat_x"]
            hat_y = state["hat_y"]
            if abs(hat_y) > abs(hat_x):  # Prioridad vertical (arriba/abajo)
                if hat_y > 0:  # Arriba -> Reposo (idle)
                    self.set_mode("idle")
                elif hat_y < 0:  # Abajo -> Sentadilla (situp)
                    self.set_mode("situp")
            else:  # Horizontal (izquierda/derecha)
                if hat_x > 0:  # Izquierda -> Caminar (walk)
                    self.set_mode("walk")
                elif hat_x < 0:  # Derecha -> Saltar (pronk)
                    self.set_mode("pronk")

        # Ajuste de velocidad con triggers (ya es de 0.1 en 0.1, no cambia)
        if state.get("lt_pressed", False):
            if time.time() - self.last_trigger_time > 0.2:
                new_val = max(1, self.speed_slider.value() - 1)
                self.speed_slider.setValue(new_val)
                self.last_trigger_time = time.time()
        if state.get("rt_pressed", False):
            if time.time() - self.last_trigger_time > 0.2:
                new_val = min(20, self.speed_slider.value() + 1)
                self.speed_slider.setValue(new_val)
                self.last_trigger_time = time.time()

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
        if self.websocket and self.ws_connected:
            command = self.build_command()
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
    
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(34, 32, 71))
    palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Base, QColor(45, 45, 95))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(34, 32, 71))
    palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Button, QColor(45, 45, 95))
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