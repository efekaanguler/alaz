#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
AUTOWARE_RUN="${SCRIPT_DIR}/../autoware/docker/run.sh"


WORKSPACE_PATH="$SCRIPT_DIR"

# GPU kontrolü - nvidia-smi varsa ve çalışıyorsa GPU kullan
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
  echo "GPU tespit edildi, GPU ile başlatılıyor..."
  exec "${AUTOWARE_RUN}" --devel --workspace "${WORKSPACE_PATH}" /bin/bash
else
  echo "GPU bulunamadı, GPU'suz başlatılıyor..."
  exec "${AUTOWARE_RUN}" --devel --no-nvidia --workspace "${WORKSPACE_PATH}" /bin/bash
fi
