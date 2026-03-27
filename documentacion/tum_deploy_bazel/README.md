# Robot Cuadrúpedo TUM - Guía Completa

## Tabla de Contenidos
- [Descripción General](#descripción-general)
- [Requisitos Previos](#requisitos-previos)
- [Configuración Inicial del Sistema](#configuración-inicial-del-sistema)
- [Preparación de la Raspberry Pi](#preparación-de-la-raspberry-pi)
- [Configuración de Motores y Batería](#configuración-de-motores-y-batería)
- [Calibración del Robot](#calibración-del-robot)
- [Operación del Robot](#operación-del-robot)
- [Desarrollo y Actualización de Software](#desarrollo-y-actualización-de-software)
- [Interfaz de Control](#interfaz-de-control)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Marcos de Referencia](#marcos-de-referencia)
- [Solución de Problemas](#solución-de-problemas)

---

## Descripción General

Este proyecto es un sistema de control para robot cuadrúpedo utilizando motores Moteus y comunicación a través de Raspberry Pi con Pi3Hat. El sistema incluye múltiples modos de movimiento, interfaz Python para control remoto, y capacidades de telemetría.

### Características Principales
- Control en tiempo real con comunicación de baja latencia
- Múltiples modos de movimiento (caminar, saltar, sit-up, reposo)
- Interfaz Python GUI con soporte para gamepad
- Sistema de calibración manual de posiciones
- Monitoreo de temperatura y voltaje de batería
- Detección automática de fallas y protección de motores

---

## Requisitos Previos

### Hardware Necesario
- Robot cuadrúpedo TUM con 12 motores Moteus
- Raspberry Pi 4 con Pi3Hat
- Batería LiPo 10S (nominal 37V, cargado ~42V)
- Computador de desarrollo con Ubuntu (20.04, 22.04 o 24.04)
- Tarjeta SD de **mínimo 32GB** (obligatorio)
- Plataforma suspendida para calibración
- Conexión WiFi

### Software en el Computador de Desarrollo
- Ubuntu 20.04, 22.04 o 24.04
- Bazel (se descarga automáticamente por el script)
- Python 3
- Git

---

## Configuración Inicial del Sistema

### 1. Instalación de Paquetes en el Computador de Desarrollo

Ejecuta el siguiente comando en la raíz del proyecto:

```bash
./install-packages
```

Este script instala todas las dependencias necesarias incluyendo:
- Herramientas de compilación (gcc, g++, make)
- Python y paquetes relacionados
- Bibliotecas de desarrollo necesarias
- Intenta instalar automáticamente `libtinfo5`

**Nota para Ubuntu 24.04:** El script intentará instalar automáticamente `libtinfo5`. Si la instalación automática falla, deberás instalarlo manualmente:
```bash
# Descarga desde: https://packages.ubuntu.com/jammy/amd64/libtinfo5/download
# Luego instala:
sudo dpkg -i libtinfo5_*.deb
```

---

## Preparación de la Raspberry Pi

### 1. Instalación del Sistema Operativo

**IMPORTANTE:** 
- El robot requiere **Raspbian Bookworm Legacy de 32 bits** (obligatorio)
- Bookworm Legacy es actualmente la versión disponible en Raspberry Pi Imager
- Tarjeta SD de **mínimo 32GB** requerida

**Pasos de instalación:**
1. Descarga e instala Raspberry Pi Imager
2. Selecciona: **Raspberry Pi OS (Legacy, 32-bit)**
3. Configura el sistema básico:
   - Usuario: `pi`
   - Contraseña: (a tu elección)
   - Habilita SSH
   - Configura hostname si lo deseas
4. Graba en tarjeta SD de mínimo 32GB

### 2. Instalación de Dependencias en la Raspberry Pi

Conecta via SSH a la Raspberry Pi y ejecuta:

```bash
pip3 install moteus moteus_pi3hat
```

### 3. Configuración del Sistema

El paquete `tum_deploy` incluye un archivo `setup_system` que configura:
- Red WiFi del robot
- Servicios del sistema
- Permisos necesarios

**Configuración WiFi del Robot (por defecto):**
- **SSID:** mjbots[número-de-placa]
- **Contraseña:** WalkingRobots

Si necesitas modificar la configuración WiFi, edita el archivo `setup_system` antes de ejecutarlo.

---

## Configuración de Motores y Batería

### CONFIGURACIÓN CRÍTICA DE BATERÍA

El robot viene configurado por defecto para una batería **10S** (10 celdas en serie, ~37V nominal, ~42V cargada). Esta configuración se encuentra en el archivo `config_servos.py`.

#### Parámetros Críticos de Batería

```python
# En config_servos.py
CONFIG_TUM = {
    'servo.flux_brake_min_voltage' : '41.0',      # Voltaje mínimo para freno de flujo
    'servo.flux_brake_resistance_ohm' : '0.05',   # Resistencia virtual del freno
    # ... otros parámetros
}
```

### ADVERTENCIA - Cambio de Voltaje de Batería

**Si usas una batería de voltaje diferente a 10S, DEBES modificar estos dos parámetros:**

#### 1. `servo.flux_brake_min_voltage`
Este valor debe ajustarse proporcionalmente al voltaje máximo de tu batería:
- **10S** (42V max): `41.0`
- **8S** (33.6V max): `32.0` aprox
- **6S** (25.2V max): `24.0` aprox

#### 2. `servo.flux_brake_resistance_ohm`
La resistencia virtual debe aumentarse para voltajes menores:
- **10S**: `0.05` ohm
- **8S**: `0.065` ohm aprox (aumentar ~30%)
- **6S**: `0.085` ohm aprox (aumentar ~70%)

### ⚡ Protección por Resistencia Virtual

**Importante:** Si se usa una batería con voltaje mucho más alto del configurado en `flux_brake_min_voltage`, toda la diferencia de voltaje será disipada como calor a través de la resistencia virtual (`flux_brake_resistance_ohm`).

**Esto es intencional y protege los motores:**
- Los motores generan voltaje de vuelta durante desaceleración
- Si este voltaje excede el `flux_brake_min_voltage`, se activa el freno de flujo
- La resistencia virtual disipa el exceso de energía como calor
- Esto evita daños por sobrevoltaje en los controladores Moteus
- Previene fallas por regeneración excesiva

**Por eso es crítico configurar correctamente estos valores según tu batería.**

### Aplicar Configuración de Motores

Cada vez que cambies parámetros de batería o configures motores nuevos:

```bash
# En la Raspberry Pi
cd /home/pi/tum_deploy
sudo python3 config_servos.py
```

---

## Calibración del Robot

### Identificación de las Patas

El robot utiliza la siguiente numeración y convenciones:

```
   1(FL)    2(FR)
      
     FRONTAL
   (interruptor)
     
     TRASERO
      
   3(BL)    4(BR)
```

**Convenciones:**
- **FL** = Front Left (Frontal Izquierda)
- **FR** = Front Right (Frontal Derecha)
- **BL** = Back Left (Trasera Izquierda)
- **BR** = Back Right (Trasera Derecha)

**Identificación:** El lado frontal se identifica por la ubicación del interruptor de energía.

### IMPORTANCIA DE LA CALIBRACIÓN

La calibración (zero) es **CRÍTICA** y debe realizarse **cada vez que se va a utilizar el robot** por motivos de seguridad. Una calibración incorrecta puede causar:
- Fallas durante el proceso de inicialización
- Movimientos erráticos o inestables
- Daño a las articulaciones mecánicas
- Fallas de motor por posiciones imposibles

### Procedimiento de Calibración Manual

**La calibración NO es automática - Es un proceso manual paso a paso:**

#### Preparación:
1. Coloca el robot en la plataforma suspendida
2. Conecta via SSH al robot
3. Navega al directorio: `cd /home/pi/tum_deploy`

#### Proceso de Calibración con `zero_leg.py`:

Este script es flexible y permite calibrar motores individuales o patas completas.

```bash
# Para calibrar una pata completa (ejemplo: pata 1)
./zero_leg.py -l 1
```

**Pasos que realizarás:**
1. El script muestra las posiciones actuales de los 3 motores de la pata
2. **TÚ debes mover manualmente la pata** a la posición extendida deseada
3. Asegúrate de que la pata esté completamente recta hacia abajo
4. **Presiona Enter** para confirmar que la posición es correcta
5. El script realiza el zero de los motores en esa posición (toma ~5 segundos)

**Repite para cada pata:**
```bash
./zero_leg.py -l 1  # Pata frontal izquierda
./zero_leg.py -l 2  # Pata frontal derecha
./zero_leg.py -l 3  # Pata trasera izquierda
./zero_leg.py -l 4  # Pata trasera derecha
```

#### Calibración con `zero_motors.py`:

Esta herramienta ofrece mayor flexibilidad para calibrar motores individuales:

```bash
python3 zero_motors.py
```

Permite calibrar cualquier motor específico o combinación según necesites.

### Verificación de Calibración

Después de calibrar, el robot verificará automáticamente la calibración en el siguiente arranque. Si la calibración es incorrecta, verás un mensaje de falla indicando qué motor está fuera de posición.

**Nota:** Puedes mover las patas manualmente después de calibrar sin problema.

---

## Operación del Robot

### Flujo Completo de Operación

#### 1. Preparación Física

1. **Verifica la batería:** Asegúrate de que esté cargada (>36V para 10S)
2. **Coloca el robot** en una plataforma suspendida con patas extendidas hacia abajo
3. **Conecta la batería** al power distribution board
4. **Enciende el robot** usando el interruptor de energía
5. **Espera ~30 segundos** para que la Raspberry Pi arranque completamente

#### 2. Conexión al Robot

1. **Busca la red WiFi del robot:**
   - **SSID:** mjbots[número-identificación]
   - **Contraseña:** WalkingRobots

2. **Conecta tu computador** a esta red WiFi

3. **Accede via SSH:**
   ```bash
   ssh pi@192.168.16.47
   ```

#### 3. Calibración (Obligatoria cada sesión)

**Por motivos de seguridad, calibra el robot antes de cada uso:**

```bash
cd /home/pi/tum_deploy

# Calibra cada pata manualmente:
# 1. Ejecuta el comando
# 2. Mueve la pata a posición extendida
# 3. Presiona Enter para confirmar

./zero_leg.py -l 1
./zero_leg.py -l 2
./zero_leg.py -l 3
./zero_leg.py -l 4
```

#### 4. Iniciar el Software del Robot

```bash
cd /home/pi/tum_deploy
./start-robot.sh
```

**Este script realiza automáticamente:**
- Configuración de CPU governor para rendimiento máximo
- Inicialización del sistema de control
- Apertura del servidor WebSocket en puerto 4778

#### 5. Posicionamiento para Operación

1. **Observa los logs** para verificar que no hay errores
2. Una vez que veas el sistema listo, **baja cuidadosamente el robot** de la plataforma
3. **Colócalo sobre una superficie plana y estable** con las patas completamente extendidas
4. **En esta posición inicial (patas planas)** el robot está listo para recibir comandos

**IMPORTANTE:** El robot DEBE iniciarse con las patas planas en el suelo. Si intentas que se levante desde otra posición, puede fallar o comportarse de manera impredecible.

#### 6. Monitoreo Durante Operación

El sistema monitorea continuamente:
- **Voltaje de batería:** Alerta si baja de 33V (10S)
- **Temperatura de motores:** Alerta si supera 60°C
- **Errores de comunicación:** Reintenta automáticamente
- **Inclinación:** Entra en modo falla si supera 45° de roll o pitch

---

## Desarrollo y Actualización de Software

### Estructura de Desarrollo

El proyecto usa Bazel como sistema de compilación. La estructura principal:

```
tum_deploy/
├── mech/                # Código principal del cuadrúpedo
│   ├── quadruped_control.cc/.h    # Lógica de control principal
│   ├── quadruped_command.h        # Estructuras de comandos
│   ├── quadruped_state.h          # Estado del robot
│   ├── quadruped_config.h         # Configuración
│   └── servo_interface.h          # Interfaz con motores
├── base/                # Utilidades base
├── utils/               # Scripts de utilidad
│   ├── zero_leg.py              # Calibración de patas
│   ├── zero_motors.py           # Calibración flexible de motores
│   ├── config_servos.py         # Configuración de motores
│   ├── start-robot.sh           # Script de inicio
│   └── tum_gui.py               # Interfaz de control Python
├── configs/             # Archivos de configuración
│   ├── tum.ini                  # Configuración principal
│   └── tum.cfg                  # Parámetros adicionales
└── tools/               # Herramientas de compilación
```

### 1. Compilación del Código

Después de realizar modificaciones en el código fuente:

```bash
# Desde el directorio raíz del proyecto

# Limpia compilaciones anteriores (recomendado después de cambios importantes)
tools/bazel clean

# Compila el paquete para Raspberry Pi
tools/bazel build --config pi -c opt //mech:tum_deploy.tar
```

**Notas sobre la compilación:**
- La primera compilación puede ser demorada
- Compilaciones subsecuentes son incrementales y más rápidas
- El flag `--config pi` configura la compilación cruzada para ARM
- El flag `-c opt` habilita optimizaciones

### 2. Transferir al Robot

1. **Asegúrate de estar conectado** a la red WiFi del robot

2. **Copia el archivo compilado:**
   ```bash
   rsync -avP bazel-bin/mech/tum_deploy.tar pi@192.168.16.47:/home/pi/
   ```

   El flag `-avP` proporciona:
   - `-a`: Modo archivo (preserva permisos)
   - `-v`: Modo verboso (muestra progreso)
   - `-P`: Permite reanudar transferencias interrumpidas

### 3. Actualizar e Iniciar en el Robot

```bash
# Conecta via SSH
ssh pi@192.168.16.47

# Si el robot está ejecutándose, detén el proceso actual
# (Ctrl+C si está en primer plano, o usa pkill si está en background)

# Extrae el nuevo software
cd /home/pi
tar xvf tum_deploy.tar

# Inicia el robot con el software actualizado
cd tum_deploy
./start-robot.sh
```

---

## Interfaz de Control

### Interfaz Python GUI

El robot incluye una interfaz gráfica en Python que se conecta vía WebSocket para control remoto:

```bash
# En tu computador de desarrollo (conectado a la red del robot)
cd utils
python3 tum_gui.py --ip 192.168.16.47
```

**Características de la interfaz:**
- **Control con gamepad/joystick** (si está disponible)
- **Visualización de telemetría** en tiempo real
- **Control de velocidad ajustable** (0.1 - 2.0 m/s)
- **Múltiples modos de movimiento**
- **Indicadores visuales** de conexión, batería y estado

### Modos de Operación Disponibles

La interfaz Python soporta los siguientes modos:

| Modo | Comando | Descripción |
|------|---------|-------------|
| **Apagar** | `stopped` | Motores sin energía, robot puede moverse libremente |
| **Detener** | `zero_velocity` | Motores con baja impedancia, posición controlada |
| **Reposo** | `rest` | Posición bajada controlada |
| **Caminar** | `walk` | Movimiento con marcha estándar |
| **Saltar** | `jump` | Todos las patas saltan simultáneamente (pronk) |
| **Sentadilla** | `situp` | Movimiento repetitivo arriba/abajo del cuerpo |

### Controles con Gamepad

- **Stick Izquierdo:** Control de movimiento (adelante/atrás/lateral)
- **Stick Derecho:** Control de rotación (yaw)
- **Botón LB (Shoulder Left):** Activa modo de control de postura corporal
- **LT/RT (Triggers):** Disminuir/Aumentar velocidad máxima
- **Cruceta (D-Pad):**
  - Arriba: Modo Reposo (idle)
  - Abajo: Modo Sentadilla (situp)
  - Derecha: Modo Caminar (walk)
  - Izquierda: Modo Saltar (pronk)

### Opciones de Configuración

- **Habilitar Desplazamiento Lateral:** Permite movimiento perpendicular (strafe)
- **Siempre Caminar:** Mantiene movimiento de patas en modo walk
- **Grabar Datos de Telemetría:** Habilita logging de datos

### Comunicación WebSocket

La interfaz se conecta al robot mediante WebSocket:

**Endpoint:** `ws://192.168.16.47:4778/control`

**Formato de comando:**
```json
{
  "command": {
    "mode": "walk",
    "v_R": [vx, vy, vz],
    "w_R": [wx, wy, wz],
    "rest": { "offset_RB": {...} }
  }
}
```

**Respuesta (status):**
```json
{
  "mode": "walk",
  "state": {
    "joints": [...],
    "robot": {
      "voltage": 38.5,
      ...
    }
  },
  "timing": {...},
  "fault": ""
}
```

---

## Estructura del Proyecto

### Directorios Principales

```
tum_deploy/
├── mech/                          # Sistema principal del cuadrúpedo
│   ├── quadruped_control.cc/h     # Control loop principal
│   ├── quadruped_command.h        # Definiciones de comandos
│   ├── quadruped_state.h          # Estado del sistema
│   ├── quadruped_config.h         # Parámetros de configuración
│   ├── servo_interface.h          # Interfaz con motores Moteus
│   └── pi3hat_wrapper.cc/h        # Comunicación con Pi3Hat
├── base/                          # Bibliotecas base
│   ├── fit_plane.cc/h             # Ajuste de plano para terreno
│   ├── logging.cc/h               # Sistema de logging
│   ├── quaternion.cc/h            # Matemáticas de orientación
│   └── ...
├── utils/                         # Scripts de utilidad
│   ├── zero_leg.py                # Calibración de patas completas
│   ├── zero_motors.py             # Calibración flexible de motores
│   ├── config_servos.py           # Configuración de parámetros de motores
│   ├── start-robot.sh             # Script de inicio del robot
│   ├── tum_gui.py                 # Interfaz gráfica de control
│   ├── performance_governor.sh    # Configuración de CPU
│   └── setup_system               # Setup inicial del sistema
├── configs/                       # Archivos de configuración
│   ├── tum.ini                    # Configuración principal del robot
│   └── tum.cfg                    # Configuración de parámetros
├── tools/                         # Herramientas de compilación
│   ├── bazel                      # Script wrapper de Bazel
│   └── workspace/                 # Configuración de dependencias
├── WORKSPACE                      # Configuración del workspace de Bazel
└── BUILD                          # Archivo de build principal
```

### Archivos de Configuración Importantes

#### `configs/tum.ini`
Configuración principal del robot incluyendo:
- Parámetros de control PID
- Límites de movimiento
- Configuración de hardware

#### `configs/tum.cfg`
Parámetros detallados de cada modo:
- Alturas y velocidades para stand_up, rest, situp
- Parámetros de marcha (walk)
- Configuración de saltos (jump)

#### `utils/config_servos.py`
Configuración de bajo nivel de motores Moteus:
- Límites de posición por motor
- Parámetros de freno de flujo (voltaje y resistencia)
- Ganancias PID de motor
- Ratios de reducción

---

## Marcos de Referencia

El sistema utiliza varios marcos de referencia para el control cinemático del robot. Los marcos están relacionados con orientación y posición del robot en el espacio.

### Marco de la Pata (G)
**Ubicación:** Centro de la articulación del hombro de cada pata
- **+X**: Hacia adelante (alejándose del centro del robot longitudinalmente)
- **+Y**: Hacia la derecha (relativo a la orientación de la pata)
- **+Z**: Hacia abajo

### Marco del Cuerpo (B)
**Ubicación:** Centro del cuerpo del cuadrúpedo
- Fijo al chasis
- Orientación estándar del robot

### Marco del Centro de Masa (M)
**Ubicación:** Centro de masa del robot medido con patas en posición de reposo
- Orientado nivel con el suelo
- **+Z**: Siempre apunta hacia abajo con respecto a la gravedad
- **+X**: Apunta hacia el mismo rumbo que el +X del cuerpo
- Se usa para cálculos de estabilidad

### Marco de Attitude (A)
**Ubicación:** Centro de masa del chasis
- **+Z**: Hacia abajo con respecto a la gravedad
- Rotación de **+X/+Y**: Arbitraria al encender
- Sigue la rotación de la transformación B/L
- **Attitude** se refiere a la altitud/orientación obtenida del IMU (sensor inercial)
- Usado para control de orientación basado en IMU

### Marco del Robot (R)
**Ubicación:** Desplazamiento del Marco del Cuerpo (B)
- Puede estar desplazado o rotado para aplicar cambios cinemáticos cosméticos
- Permite ajustes de pose sin modificar cinemática base
- Se usa para comandos de usuario

### Marco del Terreno (T)
**Ubicación:** Centro de masa proyectado en el suelo
- Proyección a lo largo de la dirección Z del cuerpo
- Orientado paralelo al terreno local
- Rotado para ser consistente con el marco A
- Se actualiza dinámicamente basado en contacto de patas

### Marco Local (L)
**Ubicación:** Relativo al punto de inicio
- Hacia abajo es paralelo a la gravedad
- Inicializado con el centro del robot al encender
- Sin correlación absoluta con el mundo exterior
- Usado para navegación relativa

### Transformaciones Importantes

El sistema mantiene las siguientes transformaciones:
- **tf_AB**: Attitude del cuerpo (B) a Attitude (A)
- **tf_TA**: Terreno (T) a Attitude (A)
- **tf_RB**: Robot (R) a Cuerpo (B)

Estas transformaciones se actualizan en cada ciclo de control (~200Hz) basado en:
- Lecturas del IMU (orientación)
- Posición de patas (contacto con terreno)
- Comandos de usuario (pose deseada)

---

## Solución de Problemas

### El robot se bloquea durante inicialización

**Causa más común:** Calibración incorrecta del zero de las patas.

**Síntomas:**
- Robot intenta inicializarse pero no progresa
- Logs muestran: `"Legs not in turn-on position id X=Y.YY"`
- Motores intentan llegar a posiciones pero no pueden
- El proceso se detiene en alguna etapa de stand_up o rest

**Solución:**
1. Apaga el robot completamente
2. Colócalo en plataforma suspendida con patas extendidas
3. Enciende y conecta via SSH
4. Re-calibra TODAS las patas cuidadosamente:
   ```bash
   cd /home/pi/tum_deploy
   # Para cada pata:
   # 1. Ejecuta el comando
   # 2. Mueve manualmente la pata a posición extendida
   # 3. Presiona Enter para confirmar
   ./zero_leg.py -l 1
   ./zero_leg.py -l 2
   ./zero_leg.py -l 3
   ./zero_leg.py -l 4
   ```
5. Baja el robot y colócalo con patas planas en el suelo
6. Reinicia el software: `./start-robot.sh`

**Prevención:**
- Siempre calibra antes de cada uso
- Asegúrate de que la pata esté en posición correcta antes de confirmar
- Verifica visualmente cada calibración

### Motores se sobrecalientan (>60°C)

**Causa posible:** Batería de voltaje incorrecto sin ajustar parámetros en `config_servos.py`.

**Soluciones:**
1. **Verifica configuración de batería:**
   - Si no usas 10S, ajusta `servo.flux_brake_min_voltage` y `servo.flux_brake_resistance_ohm` en `config_servos.py`
   - Aplica la configuración: `sudo python3 config_servos.py`

### No puedo conectarme via SSH

**Verificaciones:**
1. Espera 30-45 segundos después de encender para que arranque completamente
2. Verifica que estés conectado a la red WiFi correcta (mjbots...)
3. Prueba hacer ping: `ping 192.168.16.47`
4. Revisa que la Raspberry Pi tenga power (LED verde parpadeando)

**Si sigue sin funcionar:**
- Conecta un monitor y teclado USB directamente a la Raspberry Pi
- Verifica configuración de red en `/etc/wpa_supplicant/wpa_supplicant.conf`
- Revisa logs del sistema: `sudo journalctl -xe`

### Robot tiene movimientos erráticos

**Checklist:**
1. ✓ Calibración correcta de todas las patas (realizada antes de esta sesión)
2. ✓ Superficie plana y estable
3. ✓ Batería con carga suficiente (>36V para 10S)
4. ✓ Sin errores en logs (`fault` vacío en telemetría)
5. ✓ Temperatura de motores normal (<60°C)

**Si continúa:**
- Revisa logs en detalle para errores específicos
- Verifica comunicación con motores (no debe haber `missing_replies` en telemetría)
- Reduce velocidad gradualmente para aislar el problema

### Errores de compilación

**Error: "libtinfo5 not found" en Ubuntu 24.04**
```bash
# Si la instalación automática falló, instala manualmente:
wget http://archive.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2_amd64.deb
sudo dpkg -i libtinfo5_6.3-2_amd64.deb
```

**Error: "Bazel version mismatch"**
```bash
# Limpia y re-descarga Bazel
rm -rf ~/.cache/bazel
tools/bazel clean --expunge
# Intenta compilar nuevamente
```

**Error: "Permission denied" durante compilación**
```bash
# Asegúrate de que los scripts tengan permisos de ejecución
chmod +x tools/bazel
chmod +x utils/*.sh
```

---

## Mantenimiento y Mejores Prácticas

### Mantenimiento Regular

**Cada sesión de uso:**
- Verifica voltaje de batería antes y después
- Revisa temperatura máxima alcanzada
- Inspecciona visualmente conexiones y cables
- **Realiza calibración antes de usar**

**Semanalmente:**
- Limpia polvo y suciedad de motores y sensores
- Verifica ajuste de tornillos de montaje
- Revisa desgaste de patas/contacto con suelo

**Mensualmente:**
- Calibra celdas de batería
- Verifica firmware de motores Moteus
- Backup de configuración y código personalizado

### Mejores Prácticas de Operación

1. **Siempre calibra antes de cada uso** por motivos de seguridad
2. **Inicia en superficie plana** con patas extendidas
3. **Monitorea temperatura** durante sesiones largas
4. **Evita comandos bruscos** de velocidad o dirección
5. **Mantén batería** entre 20-80% de carga para mayor vida útil

---

**Última actualización:** Diciembre 2025