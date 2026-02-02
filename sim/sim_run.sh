#!/usr/bin/env bash
set -euo pipefail

# --- config ---
IMAGE="carlasim/carla:0.9.15"
NAME="carla_sim_0_9_15"

# repo root = sim_start.sh'ın bulunduğu dizinin bir üstü
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# Container içindeki mount noktası
MOUNT_POINT="/repo"

# --- helpers ---
has_cmd() { command -v "$1" >/dev/null 2>&1; }

echo "[sim_start] repo root: ${REPO_ROOT}"
echo "[sim_start] image:     ${IMAGE}"
echo "[sim_start] name:      ${NAME}"

# --- X11 GUI setup (optional but enabled) ---
# Eğer GUI kullanmak istemezsen DISPLAY kısmını disable edebilirsin.
DISPLAY_ENV=()
X11_MOUNTS=()

if [[ -n "${DISPLAY:-}" ]] && has_cmd xhost; then
  # Docker konteynerlerinin X server'a bağlanmasına izin ver
  xhost +local:docker >/dev/null 2>&1 || true
  DISPLAY_ENV=(-e "DISPLAY=${DISPLAY}")
  X11_MOUNTS=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
else
  echo "[sim_start] DISPLAY/xhost not available -> GUI may not work (headless is still fine)."
fi

# --- NVIDIA GPU check (best effort) ---
if ! has_cmd nvidia-smi; then
  echo "[sim_start] WARNING: nvidia-smi not found on host. GPU may not be available."
fi

# --- create container if not exists ---
if ! docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[sim_start] Creating container ${NAME}..."

  docker create -it \
    --name "${NAME}" \
    --gpus all \
    --net=host \
    --shm-size=8g \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    "${DISPLAY_ENV[@]}" \
    "${X11_MOUNTS[@]}" \
    -v "${REPO_ROOT}:${MOUNT_POINT}:rw" \
    -w "${MOUNT_POINT}" \
    "${IMAGE}" \
    bash
else
  echo "[sim_start] Container ${NAME} already exists."
fi

# --- start container (if stopped) ---
if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[sim_start] Starting container ${NAME}..."
  docker start "${NAME}" >/dev/null
else
  echo "[sim_start] Container ${NAME} is already running."
fi

echo
echo "[sim_start] Mounted repo inside container at: ${MOUNT_POINT}"
echo "[sim_start] You are now entering the container shell."
echo

# --- enter container ---
docker exec -it "${NAME}" bash

