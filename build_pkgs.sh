#!/bin/bash

# modules altındaki tüm paketleri derle

# ROS2 ortamını yükle
source /opt/ros/humble/setup.bash

# Dizin ayarları
BUILD_DIR="./build"
INSTALL_DIR="./install"

# Temizleme seçeneği
if [ "$1" == "--clean" ]; then
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
