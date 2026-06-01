#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="$SCRIPT_DIR"
ROS_DISTRO="${ROS_DISTRO:-humble}"

usage() {
  cat <<'EOF'
ultimate_starter.sh - build workspace, setup sensors, and launch bringup

Usage: ./ultimate_starter.sh [--skip-build] [--skip-sensor-setup] [--skip-launch]

Options:
  --skip-build          Skip running build_pkgs.sh
  --skip-sensor-setup   Skip running modules/sensor/sensor_setup.sh
  --skip-launch         Skip launching global_bringup (useful for manual start)
EOF
}

SKIP_BUILD=false
SKIP_SENSOR=false
SKIP_LAUNCH=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build) SKIP_BUILD=true; shift ;;
    --skip-sensor-setup) SKIP_SENSOR=true; shift ;;
    --skip-launch) SKIP_LAUNCH=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

echo "Workspace: ${WORKSPACE}"
echo "ROS distro: ${ROS_DISTRO}"

if ! $SKIP_BUILD; then
  if [[ -x "${WORKSPACE}/build_pkgs.sh" ]]; then
    echo "Running build_pkgs.sh..."
    pushd "${WORKSPACE}" >/dev/null
    ./build_pkgs.sh
    popd >/dev/null
  else
    echo "Warning: build_pkgs.sh not found or not executable in ${WORKSPACE}, skipping build step." >&2
  fi
fi

echo "Sourcing ROS and workspace setups..."
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found" >&2
fi
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE}/install/setup.bash"
fi

if ! $SKIP_SENSOR; then
  SENSOR_SCRIPT="${WORKSPACE}/modules/sensor/sensor_setup.sh"
  if [[ -x "$SENSOR_SCRIPT" || -f "$SENSOR_SCRIPT" ]]; then
    echo "Running sensor setup script: $SENSOR_SCRIPT"
    if [[ $(id -u) -ne 0 ]]; then
      if command -v sudo >/dev/null 2>&1; then
        echo "Using sudo to run sensor setup (may prompt for password)..."
        sudo bash "$SENSOR_SCRIPT"
      else
        echo "sudo not available, running sensor setup as current user (may fail)..."
        bash "$SENSOR_SCRIPT"
      fi
    else
      bash "$SENSOR_SCRIPT"
    fi
  else
    echo "Sensor setup script not found at ${SENSOR_SCRIPT}, skipping." >&2
  fi

  # source sllidar install if present
  if [[ -f "/sllidar/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source /sllidar/install/setup.bash
  fi
  # re-source workspace setup in case sensor build installed packages there
  if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${WORKSPACE}/install/setup.bash"
  fi
fi

if ! $SKIP_LAUNCH; then
  echo "Launching global_bringup (Ctrl-C to stop)..."
  exec ros2 launch global_bringup global.launch.py
else
  echo "Skipped launch as requested." 
fi
