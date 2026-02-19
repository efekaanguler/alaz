#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-headless}"                 # gui | headless
PORT="${CARLA_PORT:-2000}"
QUALITY="${QUALITY:-Epic}"       # Low | Epic, etc.

# CarlaUE4.sh'yi bul
CARLA_SH="$(command -v CarlaUE4.sh || true)"
if [[ -z "${CARLA_SH}" ]]; then
  if [[ -f /opt/carla/CarlaUE4.sh ]]; then
    CARLA_SH=/opt/carla/CarlaUE4.sh
  elif [[ -f /home/carla/CarlaUE4.sh ]]; then
    CARLA_SH=/home/carla/CarlaUE4.sh
  else
    echo "ERROR: CarlaUE4.sh not found."
    echo "Try inside container: find / -name CarlaUE4.sh 2>/dev/null"
    exit 1
  fi
fi

ARGS=(
  -quality-level="${QUALITY}"
  -world-port="${PORT}"
)

if [[ "${MODE}" == "headless" ]]; then
  ARGS+=(-RenderOffScreen)
fi

exec "${CARLA_SH}" "${ARGS[@]}"
