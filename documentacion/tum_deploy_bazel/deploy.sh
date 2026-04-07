#!/bin/bash
# deploy.sh - Compilación y despliegue del robot TUM en Raspberry Pi
# Uso: ./deploy.sh [opciones]

# ============================================================
# CONFIGURACIÓN — edita estos valores según tu entorno
# ============================================================
PI_USER="tum"
PI_IP="192.168.133.114"
PI_DEST="/home/tum/tum_deploy_bazel/"
# ============================================================

set -e
cd "$(dirname "$(readlink -f "$0")")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

TAR_PATH="bazel-bin/mech/tum_deploy.tar"

usage() {
    echo -e "Uso: $0 [opción] [--ip <dirección>]"
    echo ""
    echo -e "  ${GREEN}build${NC}       Compila tum_deploy.tar para la Pi"
    echo -e "  ${GREEN}clean${NC}       Limpia la caché de Bazel"
    echo -e "  ${GREEN}send${NC}        Envía el .tar ya compilado a la Pi"
    echo -e "  ${GREEN}all${NC}         Limpia, compila y envía  (default)"
    echo -e "  ${GREEN}build-send${NC}  Compila y envía sin limpiar"
    echo ""
    echo -e "  ${CYAN}--ip <addr>${NC}  Sobreescribe PI_IP definida en el script"
    echo ""
    echo -e "Ejemplo: $0 all --ip 192.168.1.50"
    echo ""
}

# --- Parseo de argumentos ---
ACTION="${1:-all}"
shift || true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip)
            PI_IP="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo -e "${RED}Argumento desconocido: $1${NC}"
            usage
            exit 1
            ;;
    esac
done

PI_TARGET="${PI_USER}@${PI_IP}:${PI_DEST}"

do_clean() {
    echo -e "${YELLOW}► Limpiando caché de Bazel...${NC}"
    tools/bazel clean
    echo -e "${GREEN}✓ Limpieza completa${NC}"
}

do_build() {
    echo -e "${YELLOW}► Compilando tum_deploy.tar para Raspberry Pi (ARM)...${NC}"
    tools/bazel build --config pi -c opt //mech:tum_deploy.tar
    echo -e "${GREEN}✓ Compilación exitosa → ${TAR_PATH}${NC}"
}

do_send() {
    if [[ ! -f "$TAR_PATH" ]]; then
        echo -e "${RED}✗ No se encontró ${TAR_PATH}. Compila primero con: $0 build${NC}"
        exit 1
    fi

    echo -e "${YELLOW}► Enviando a ${PI_TARGET}...${NC}"

    # Verificar conectividad antes de intentar
    if ! ping -c 1 -W 2 "$PI_IP" &>/dev/null; then
        echo -e "${RED}✗ No se puede alcanzar ${PI_IP}. Verifica la red o usa --ip para cambiar la IP.${NC}"
        exit 1
    fi

    rsync -avP "$TAR_PATH" "$PI_TARGET"
    echo -e "${GREEN}✓ Transferencia completa → ${PI_TARGET}${NC}"
    echo ""
    echo -e "Para actualizar en la Pi:"
    echo -e "  ${CYAN}ssh ${PI_USER}@${PI_IP}${NC}"
    echo -e "  ${CYAN}cd ${PI_DEST} && tar xvf tum_deploy.tar && cd tum_deploy && ./start-robot.sh${NC}"
}

case "$ACTION" in
    build)      do_build ;;
    clean)      do_clean ;;
    send)       do_send ;;
    build-send) do_build && do_send ;;
    all)        do_clean && do_build && do_send ;;
    -h|--help)  usage ;;
    *)          echo -e "${RED}Opción desconocida: $ACTION${NC}"; usage; exit 1 ;;
esac
