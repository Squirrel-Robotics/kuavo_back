#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-auto}"
VR_ARG="${2:-${KUAVO_VR_IP:-192.168.50.79}}"
LOWER_VR_IF="${3:-${KUAVO_WIRED_VR_IF:-eno1}}"
LOWER_VR_ADDR="${4:-${KUAVO_WIRED_VR_LOCAL_IP:-192.168.50.80/24}}"
UPPER_WIRED_IP="${5:-${KUAVO_UPPER_WIRED_IP:-192.168.26.12}}"
LOWER_WIRED_IF="${KUAVO_LOWER_WIRED_IF:-enp3s0}"

VR_IP="${VR_ARG%%:*}"
LOWER_VR_IP="${LOWER_VR_ADDR%%/*}"

log() { echo "[setup_wired_vr_route] $*"; }
run_net() {
  if [ "${EUID:-$(id -u)}" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}
carrier_is_up() {
  [ -r "/sys/class/net/$1/carrier" ] && [ "$(cat "/sys/class/net/$1/carrier" 2>/dev/null || echo 0)" = "1" ]
}

if [ -z "$VR_IP" ] || [ "$VR_IP" = "0.0.0.0" ]; then
  log "VR IP is empty, skip wired route setup."
  exit 0
fi

case "$MODE" in
  off|none|false)
    log "wired route setup disabled. Current route:"
    ip route get "$VR_IP" || true
    exit 0
    ;;
  status)
    log "status for VR $VR_IP"
    ip -br addr || true
    ip route get "$VR_IP" || true
    ip neigh show "$VR_IP" || true
    exit 0
    ;;
  auto)
    if carrier_is_up "$LOWER_VR_IF"; then
      MODE="lower-direct"
      log "$LOWER_VR_IF has carrier, using lower-direct mode."
    else
      log "$LOWER_VR_IF has no carrier, not changing route automatically."
      log "If VR is plugged into upper machine, run upper relay setup and launch with wired_vr_route_mode:=upper-relay."
      ip route get "$VR_IP" || true
      exit 0
    fi
    ;;
  lower-direct|upper-relay)
    ;;
  *)
    log "unknown mode '$MODE'. Use auto, lower-direct, upper-relay, status, or off."
    exit 2
    ;;
esac

if [ "$MODE" = "lower-direct" ]; then
  log "configuring lower direct wired route: $VR_IP via $LOWER_VR_IF, local $LOWER_VR_ADDR"
  run_net ip link set "$LOWER_VR_IF" up
  run_net ip addr replace "$LOWER_VR_ADDR" dev "$LOWER_VR_IF"
  run_net ip route replace "$VR_IP/32" dev "$LOWER_VR_IF" src "$LOWER_VR_IP" metric 10
elif [ "$MODE" = "upper-relay" ]; then
  log "configuring lower route to upper relay: $VR_IP via $UPPER_WIRED_IP dev $LOWER_WIRED_IF"
  run_net ip route replace "$VR_IP/32" via "$UPPER_WIRED_IP" dev "$LOWER_WIRED_IF" metric 10
fi

log "result route:"
ip route get "$VR_IP" || true
