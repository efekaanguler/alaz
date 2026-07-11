#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CUSTOM_RUN="${SCRIPT_DIR}/custom_autoware_run.sh"

WORKSPACE_PATH="$SCRIPT_DIR"

if [ ! -x "$CUSTOM_RUN" ]; then
    echo "HATA: custom_autoware_run.sh bulunamadı! Lütfen repodan çektiğinize emin olun."
    exit 1
fi

# GPU kontrolü ve başlatma
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
  echo "GPU tespit edildi, GPU ile başlatılıyor..."
  exec "${CUSTOM_RUN}" --devel --workspace "${WORKSPACE_PATH}" /bin/bash
else
  echo "GPU bulunamadı, GPU'suz başlatılıyor..."
  exec "${CUSTOM_RUN}" --devel --no-nvidia --workspace "${WORKSPACE_PATH}" /bin/bash
fi
