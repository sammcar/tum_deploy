#!/bin/bash
# sim.sh - Compilación y ejecución del simulador TUM en PC local (x86_64)

set -e
cd "$(dirname "$(readlink -f "$0")")"

# --- Colores ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

BAZEL_FLAGS="--crosstool_top=@rpi_bazel//tools/cc_toolchain:toolchain \
  --cpu=k8 \
  --cxxopt='-std=c++2a'"

BUILD_SIM_CMD="tools/bazel build -c opt //mech:quadruped_sim $BAZEL_FLAGS"
BUILD_GUI_CMD="tools/bazel build -c opt //mech:tum_quad_terminal $BAZEL_FLAGS"

RUN_CMD="./bazel-bin/mech/quadruped_sim -c configs/tum.ini"

usage() {
    echo -e "Uso: $0 [opción] [--ip <dirección>]"
    echo -e "  ${GREEN}build${NC}                Compila quadruped_sim y tum_quad_terminal"
    echo -e "  ${GREEN}run${NC}                  Ejecuta el simulador"
    echo -e "  ${GREEN}both${NC}                 Compila y luego ejecuta  (default)"
    echo -e "  ${GREEN}debug${NC}                Ejecuta con --debug (sin señales RT)"
    echo -e "  ${GREEN}control --local${NC}      Lanza la GUI terminal conectada a localhost"
    echo -e "  ${GREEN}control --ip <addr>${NC}  Lanza la GUI terminal conectada a una IP"
    echo ""
}

do_build() {
    echo -e "${YELLOW}► Compilando quadruped_sim...${NC}"
    eval $BUILD_SIM_CMD
    echo -e "${GREEN}✓ quadruped_sim → bazel-bin/mech/quadruped_sim${NC}"

    echo -e "${YELLOW}► Compilando tum_quad_terminal...${NC}"
    eval $BUILD_GUI_CMD
    echo -e "${GREEN}✓ tum_quad_terminal → bazel-bin/mech/tum_quad_terminal${NC}"
}

do_run() {
    echo -e "${YELLOW}► Iniciando simulador (WebSocket en puerto 4778)...${NC}"
    echo -e "  GUI Python:   ${GREEN}python3 utils/tum_gui_sam_mouse.py --ip 127.0.0.1${NC}"
    echo -e "  GUI Terminal: ${GREEN}./bazel-bin/mech/tum_quad_terminal --local${NC}"
    echo ""
    eval $RUN_CMD
}

do_run_debug() {
    echo -e "${YELLOW}► Iniciando simulador en modo debug...${NC}"
    echo -e "  GUI Python:   ${GREEN}python3 utils/tum_gui_sam_mouse.py --ip 127.0.0.1${NC}"
    echo -e "  GUI Terminal: ${GREEN}./bazel-bin/mech/tum_quad_terminal --local${NC}"
    echo ""
    eval "$RUN_CMD --debug"
}

# --- Parseo de --ip para control ---
IP_ARG=""
if [[ "$1" == "control" ]]; then
    case "$2" in
        --local) IP_ARG="--local" ;;
        --ip)    IP_ARG="--ip $3" ;;
        *)
            echo -e "${RED}Uso: $0 control --local  o  $0 control --ip <dirección>${NC}"
            exit 1
            ;;
    esac
fi

case "${1:-both}" in
    build)   do_build ;;
    run)     do_run ;;
    debug)   do_run_debug ;;
    both)    do_build && do_run ;;
    control) ./bazel-bin/mech/tum_quad_terminal $IP_ARG ;;
    *)       usage; exit 1 ;;
esac