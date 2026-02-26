#!/usr/bin/env bash
set -e

# ==== AYAR: ROS distro (gerekirse humble -> iron/jazzy yap) ====
ROS_DISTRO=humble

# ==== 1) ROS'u source et ====
source /opt/ros/${ROS_DISTRO}/setup.bash

# ==== 2) Gerekli minimum tool'lar ====
apt-get update
apt-get install -y \
  git \
  build-essential \
  python3-rosdep \
  python3-colcon-common-extensions

# ==== 3) Kamera driver (apt) ====
apt-get install -y ros-${ROS_DISTRO}-v4l2-camera

# ==== 4) SLLidar (source) - /sllidar altına ====
mkdir -p /sllidar/src
cd /sllidar/src
rm -rf sllidar_ros2
git clone https://github.com/Slamtec/sllidar_ros2.git

# ==== 5) Bağımlılıklar ====
rosdep init 2>/dev/null || true
rosdep update
cd /sllidar
rosdep install --from-paths src --ignore-src -r -y

# ==== 6) Build ====
colcon build --symlink-install

# ==== 7) Kullanıma hazır ====
source /sllidar/install/setup.bash
ros2 pkg list | grep -E "v4l2_camera|sllidar"
echo "OK: setup bitti. Her shell'de:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source /sllidar/install/setup.bash"