#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=humble"
  exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get not found. Install dependencies manually for your OS."
  exit 1
fi

if command -v sudo >/dev/null 2>&1; then
  APT_PREFIX=(sudo)
else
  APT_PREFIX=()
fi

"${APT_PREFIX[@]}" apt-get update
"${APT_PREFIX[@]}" apt-get install -y \
  python3-pip \
  python3-opencv \
  python3-numpy \
  python3-colcon-common-extensions \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-launch-xml

python3 -m pip install --user --upgrade pip
# ROS Humble cv_bridge is compiled against NumPy 1.x ABI.
python3 -m pip install --user --upgrade "numpy<2"
if ! python3 -c "import onnxruntime" >/dev/null 2>&1; then
  python3 -m pip install --user onnxruntime
fi
# Keep NumPy pinned after potential pip dependency resolution by onnxruntime.
python3 -m pip install --user --upgrade "numpy<2"

echo "Dependency installation complete."
