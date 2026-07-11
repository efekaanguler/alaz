#!/bin/bash
set -euo pipefail

# sim_pkg paketini ./build altına derle

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

# Paketi derle
colcon build \
    --base-paths sim \
    --build-base "${BUILD_DIR}" \
    --install-base "${INSTALL_DIR}" \
    --symlink-install

# ROS 2 Python paketleri için lib/sim_pkg dizinini oluştur ve executable'ları symlink et
SIM_PKG_LIB="${INSTALL_DIR}/sim_pkg/lib/sim_pkg"
if [ -d "${INSTALL_DIR}/sim_pkg/bin" ]; then
    mkdir -p "${SIM_PKG_LIB}"
    for exec in "${INSTALL_DIR}/sim_pkg/bin"/*; do
        if [ -f "$exec" ]; then
            exec_name=$(basename "$exec")
            ln -sf "../../bin/${exec_name}" "${SIM_PKG_LIB}/${exec_name}"
        fi
    done
    echo "Executable'lar lib/sim_pkg/ dizinine symlink edildi"
fi

echo "Derleme tamamlandı: ${INSTALL_DIR}"
