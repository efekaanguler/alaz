#!/usr/bin/env bash
set -euo pipefail
NAME="carla_sim_0_9_15"

if docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  docker stop "${NAME}" >/dev/null
fi

if docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
  docker rm "${NAME}" >/dev/null
fi

echo "Removed container: ${NAME}"
