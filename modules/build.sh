#!/usr/bin/env bash
# filepath: /home/efekaan/Desktop/alaz/2026/alaz/modules/build.sh
set -eo pipefail

# -----------------------------------------------------------------------------
# Build all ROS2 packages under ./modules using colcon.
#
# Outputs go to /tmp by default (ephemeral). Container exits => outputs gone.
#
# Usage (inside container):
#   /workspace/modules/build.sh
#
# Optional:
#   BUILD_ROOT=/tmp/mybuild /workspace/modules/build.sh
#   /workspace/modules/build.sh --packages-select my_pkg
# -----------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODULES_DIR="${REPO_ROOT}/modules"

BUILD_ROOT="${BUILD_ROOT:-/tmp/colcon_ws}"
BUILD_BASE="${BUILD_ROOT}/build"
INSTALL_BASE="${BUILD_ROOT}/install"

if [[ ! -d "${MODULES_DIR}" ]]; then
  echo "ERROR: modules directory not found: ${MODULES_DIR}"
  exit 1
fi

# Source ROS 2 (prefer ROS_DISTRO if set, otherwise try common distros)
source_ros() {
  local ros_distro="${ROS_DISTRO:-}"
  
  if [[ -n "${ros_distro}" ]] && [[ -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/${ros_distro}/setup.bash"
    return 0
  fi

  for d in jazzy humble iron; do
    if [[ -f "/opt/ros/${d}/setup.bash" ]]; then
      echo "WARN: ROS_DISTRO not set; using detected distro: ${d}"
      # shellcheck disable=SC1091
      source "/opt/ros/${d}/setup.bash"
      export ROS_DISTRO="${d}"
      return 0
    fi
  done

  echo "ERROR: Could not find any ROS setup.bash under /opt/ros (jazzy/humble/iron)."
  exit 1
}

source_ros

if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon not found in PATH after sourcing ROS. Something is wrong with the image/environment."
  exit 1
fi

echo "==> ROS_DISTRO=${ROS_DISTRO}"
echo "==> Repo root:     ${REPO_ROOT}"
echo "==> Modules path:  ${MODULES_DIR}"
echo "==> Build outputs: ${BUILD_ROOT} (ephemeral)"
echo ""

# Check if there are any packages to build
PACKAGE_COUNT=$(find "${MODULES_DIR}" -maxdepth 2 -name "package.xml" | wc -l)
if [[ "${PACKAGE_COUNT}" -eq 0 ]]; then
  echo "WARN: No ROS2 packages found in ${MODULES_DIR}"
  echo "      (no package.xml files detected)"
  echo ""
  echo "Create a new package with:"
  echo "  cd /workspace/modules"
  echo "  ros2 pkg create --build-type ament_cmake alaz_perception"
  echo ""
  exit 0
fi

echo "Found ${PACKAGE_COUNT} package(s) to build..."
echo ""

mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}"

echo "==> Running colcon build..."
cd "${REPO_ROOT}"
colcon build \
  --symlink-install \
  --base-paths "${MODULES_DIR}" \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "$@"

BUILD_STATUS=$?

if [[ ${BUILD_STATUS} -eq 0 ]]; then
  echo ""
  echo "✓ Build finished successfully!"
  echo ""
  echo "To use your packages in THIS shell:"
  echo "  source ${INSTALL_BASE}/setup.bash"
  echo ""
  echo "Or add to ~/.bashrc (inside container):"
  echo "  echo 'source ${INSTALL_BASE}/setup.bash' >> ~/.bashrc"
  echo ""
else
  echo ""
  echo "✗ Build failed with exit code ${BUILD_STATUS}"
  echo ""
  exit ${BUILD_STATUS}
fi