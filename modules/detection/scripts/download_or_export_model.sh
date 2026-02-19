#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_DIR="${ROOT_DIR}/models"
DETECTOR_MODEL_PATH="${MODEL_DIR}/yolov8n.onnx"
DETECTOR_MODEL_URL="https://github.com/ultralytics/assets/releases/latest/download/yolov8n.onnx"

TL_DIR="${MODEL_DIR}/traffic_light_classifier"
TL_MODEL_PATH="${TL_DIR}/ped_traffic_light_classifier_mobilenetv2_batch_6.onnx"
TL_LABEL_PATH="${TL_DIR}/lamp_labels_ped.txt"

TL_MODEL_URLS=(
  "https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/ped_traffic_light_classifier_mobilenetv2_batch_6.onnx"
  "https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/ped_traffic_light_classifier_mobilenetv2_batch_6.onnx"
)
TL_LABEL_URLS=(
  "https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/lamp_labels_ped.txt"
  "https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/lamp_labels_ped.txt"
)

mkdir -p "${MODEL_DIR}" "${TL_DIR}"

# ROS Humble cv_bridge is not compatible with NumPy 2.x ABI.
python3 -m pip install --user --upgrade "numpy<2"

download_first() {
  local output_path="$1"
  shift

  if [[ -f "${output_path}" ]]; then
    echo "Already exists: ${output_path}"
    return 0
  fi

  if ! command -v curl >/dev/null 2>&1; then
    echo "curl not found, cannot download ${output_path}"
    return 1
  fi

  local url
  for url in "$@"; do
    if curl -L --fail "${url}" -o "${output_path}"; then
      echo "Downloaded: ${output_path}"
      echo "  source: ${url}"
      return 0
    fi
  done

  rm -f "${output_path}"
  return 1
}

if [[ -f "${DETECTOR_MODEL_PATH}" ]]; then
  echo "Model already exists: ${DETECTOR_MODEL_PATH}"
else
  DOWNLOADED=0
  if command -v curl >/dev/null 2>&1; then
    if curl -L --fail "${DETECTOR_MODEL_URL}" -o "${DETECTOR_MODEL_PATH}"; then
      DOWNLOADED=1
      echo "Model downloaded from ${DETECTOR_MODEL_URL}"
    fi
  fi

  if [[ "${DOWNLOADED}" -eq 0 ]]; then
    echo "Direct download failed, falling back to local export with ultralytics..."
    python3 -m pip install --user ultralytics onnx onnxsim onnxruntime onnxslim

    WORK_DIR="$(mktemp -d)"
    pushd "${WORK_DIR}" >/dev/null
    python3 - << 'PY'
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
model.export(format="onnx", imgsz=640, simplify=True, opset=12)
PY
    mv yolov8n.onnx "${DETECTOR_MODEL_PATH}"
    popd >/dev/null
    rm -rf "${WORK_DIR}"
    echo "Model exported to ${DETECTOR_MODEL_PATH}"
  fi
fi

# Keep NumPy pinned after optional ultralytics/onnxruntime installs.
python3 -m pip install --user --upgrade "numpy<2"

cp "${ROOT_DIR}/config/coco_labels.txt" "${MODEL_DIR}/labels.txt"
cp "${ROOT_DIR}/config/coco_color_map.json" "${MODEL_DIR}/color_map.json"

if ! download_first "${TL_MODEL_PATH}" "${TL_MODEL_URLS[@]}"; then
  echo "Traffic light model could not be downloaded automatically."
  echo "Put model manually at: ${TL_MODEL_PATH}"
fi

if ! download_first "${TL_LABEL_PATH}" "${TL_LABEL_URLS[@]}"; then
  echo "Traffic light labels could not be downloaded automatically."
  echo "Put labels manually at: ${TL_LABEL_PATH}"
fi

echo "Artifacts ready:"
echo "  detector model: ${DETECTOR_MODEL_PATH}"
echo "  detector label: ${MODEL_DIR}/labels.txt"
echo "  detector color: ${MODEL_DIR}/color_map.json"
echo "  tl model:       ${TL_MODEL_PATH}"
echo "  tl labels:      ${TL_LABEL_PATH}"
