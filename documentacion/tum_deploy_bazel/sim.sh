#!/bin/bash
# sim.sh - Compilación y ejecución del simulador TUM en PC local (x86_64)

set -e
cd "$(dirname "$(readlink -f "$0")")"

# --- Colores ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

BUILD_CMD="tools/bazel build -c opt //mech:quadruped_sim \
  --crosstool_top=@rpi_bazel//tools/cc_toolchain:toolchain \
  --cpu=k8 \
  --cxxopt='-std=c++2a'"

RUN_CMD="./bazel-bin/mech/quadruped_sim -c configs/tum.ini"

usage() {
    echo -e "Uso: $0 [opción]"
    echo -e "  ${GREEN}build${NC}   Compila quadruped_sim"
    echo -e "  ${GREEN}run${NC}     Ejecuta el simulador"
    echo -e "  ${GREEN}both${NC}    Compila y luego ejecuta  (default)"
    echo -e "  ${GREEN}debug${NC}   Ejecuta con --debug (sin señales RT)"
    echo ""
}

do_build() {
    echo -e "${YELLOW}► Compilando quadruped_sim...${NC}"
    eval $BUILD_CMD
    echo -e "${GREEN}✓ Compilación exitosa → bazel-bin/mech/quadruped_sim${NC}"
}

do_run() {
    echo -e "${YELLOW}► Iniciando simulador (WebSocket en puerto 4778)...${NC}"
    echo -e "  GUI: ${GREEN}python3 utils/tum_gui_sam_mouse.py --ip 127.0.0.1${NC}"
    echo ""
    eval $RUN_CMD
}

do_run_debug() {
    echo -e "${YELLOW}► Iniciando simulador en modo debug...${NC}"
    echo -e "  GUI: ${GREEN}python3 utils/tum_gui_sam_mouse.py --ip 127.0.0.1${NC}"
    echo ""
    eval "$RUN_CMD --debug"
}

case "${1:-both}" in
    build) do_build ;;
    run)   do_run ;;
    debug) do_run_debug ;;
    both)  do_build && do_run ;;
    *)     usage; exit 1 ;;
esac
