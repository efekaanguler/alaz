#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
AUTOWARE_RUN="${SCRIPT_DIR}/../autoware/docker/run.sh"
CUSTOM_RUN="${SCRIPT_DIR}/custom_autoware_run.sh"

WORKSPACE_PATH="$SCRIPT_DIR"

# Alaz'ın özel Docker betiğini Autoware'in içine kopyala
if [ -f "$CUSTOM_RUN" ]; then
    echo "Alaz özel sensör ayarları Autoware'e entegre ediliyor..."
    cp "$CUSTOM_RUN" "$AUTOWARE_RUN"
    chmod +x "$AUTOWARE_RUN"
else
    echo "HATA: custom_autoware_run.sh bulunamadı! Lütfen repodan çektiğinize emin olun."
    exit 1
fi

# GPU kontrolü ve başlatma
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
  echo "GPU tespit edildi, GPU ile başlatılıyor..."
  exec "${AUTOWARE_RUN}" --devel --workspace "${WORKSPACE_PATH}" /bin/bash
else
  echo "GPU bulunamadı, GPU'suz başlatılıyor..."
  exec "${AUTOWARE_RUN}" --devel --no-nvidia --workspace "${WORKSPACE_PATH}" /bin/bash
fi
