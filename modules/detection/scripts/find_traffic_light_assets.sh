#!/usr/bin/env bash
set -euo pipefail

TARGETS=(
  "ped_traffic_light_classifier_mobilenetv2_batch_6.onnx"
  "lamp_labels_ped.txt"
)

SEARCH_DIRS=(
  "${HOME}/autoware_data"
  "/opt/autoware_data"
  "/opt/autoware"
  "${HOME}/Desktop/detection/models"
)

echo "Searching common directories for traffic light assets..."
for dir in "${SEARCH_DIRS[@]}"; do
  if [[ -d "${dir}" ]]; then
    echo "\n[DIR] ${dir}"
    for target in "${TARGETS[@]}"; do
      find "${dir}" -type f -name "${target}" 2>/dev/null || true
    done
  fi
done

if command -v ros2 >/dev/null 2>&1; then
  echo "\nChecking installed ROS package path..."
  if ros2 pkg prefix autoware_traffic_light_classifier >/dev/null 2>&1; then
    PREFIX="$(ros2 pkg prefix autoware_traffic_light_classifier)"
    SHARE_DIR="${PREFIX}/share/autoware_traffic_light_classifier"
    echo "Package prefix: ${PREFIX}"
    echo "Launch file:    ${SHARE_DIR}/launch/pedestrian_traffic_light_classifier.launch.xml"
    echo "Param file:     ${SHARE_DIR}/config/pedestrian_traffic_light_classifier.param.yaml"
  else
    echo "autoware_traffic_light_classifier package is not in current ROS environment."
  fi
fi
