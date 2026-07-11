#!/bin/bash
set -euo pipefail

# modules altındaki tüm paketleri derle

# ROS 2 and Autoware setup scripts may reference unset variables.
set +u
source /opt/ros/humble/setup.bash
source /opt/autoware/setup.bash
set -u

# Dizin ayarları
BUILD_DIR="./build"
INSTALL_DIR="./install"

# Temizleme seçeneği
if [ "${1:-}" == "--clean" ]; then
    rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
fi

# Dizinleri oluştur
mkdir -p "${BUILD_DIR}"
mkdir -p "${INSTALL_DIR}"

# modules altındaki tüm paketleri derle
colcon build \
    --base-paths modules \
    --build-base "${BUILD_DIR}" \
    --install-base "${INSTALL_DIR}" \
    --symlink-install

echo "Derleme tamamlandı: ${INSTALL_DIR}"
