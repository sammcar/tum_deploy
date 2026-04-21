#!/bin/bash
# ============================================================
#  startup_tum.sh
#  Ejecuta automáticamente el flujo de arranque de TUM:
#    1. Configura el baudrate del sensor LiDAR
#    2. Calibra los zero-offsets de los 12 motores
# ============================================================

set -e  # Detener si cualquier comando falla

VENV_PYTHON="/home/tum/tum/bin/python3"
SAM_CODES="/home/tum/tum/sam_codes"
LOG_FILE="/home/tum/tum_startup.log"

echo "========================================"  | tee -a "$LOG_FILE"
echo "  TUM STARTUP - $(date)"                   | tee -a "$LOG_FILE"
echo "========================================"  | tee -a "$LOG_FILE"

# ── PASO 1: Configurar baudrate del sensor ─────────────────
echo "[1/2] Configurando baudrate del sensor..."  | tee -a "$LOG_FILE"

"$VENV_PYTHON" "$SAM_CODES/set_baudrate.py" 2>&1 | tee -a "$LOG_FILE"

echo "[1/2] Baudrate configurado correctamente."  | tee -a "$LOG_FILE"

# ── PASO 2: Calibrar zero-offsets de todos los motores ─────
echo "[2/2] Calibrando zero-offsets (todos los motores)..."  | tee -a "$LOG_FILE"

sudo "$VENV_PYTHON" "$SAM_CODES/motors_utils/zeros_auto.py" 2>&1 | tee -a "$LOG_FILE"

echo "[2/2] Calibración completada."              | tee -a "$LOG_FILE"
echo "✓ Startup TUM finalizado exitosamente."     | tee -a "$LOG_FILE"
