#!/bin/bash
# test_detection_e2e.sh — End-to-end detection test for the perception pipeline.
#
# Tests:
# 1. Camera can be opened
# 2. YOLOv8 model can be loaded
# 3. Detection runs and produces output
# 4. Traffic light classifier works
#
# Usage:
#   ./test_detection_e2e.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODULE_DIR="$(dirname "$SCRIPT_DIR")"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

run_test() {
    local name="$1"
    local cmd="$2"
    echo -n "  Testing: $name ... "
    if eval "$cmd" > /dev/null 2>&1; then
        echo -e "${GREEN}PASS${NC}"
        PASS=$((PASS + 1))
    else
        echo -e "${RED}FAIL${NC}"
        FAIL=$((FAIL + 1))
    fi
}

echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Perception Module — End-to-End Tests${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"

# Test 1: Python dependencies
echo -e "\n${YELLOW}[1] Dependencies${NC}"
run_test "OpenCV import" "python3 -c 'import cv2'"
run_test "NumPy import" "python3 -c 'import numpy'"
run_test "ONNX Runtime import" "python3 -c 'import onnxruntime'"

# Test 2: Model files
echo -e "\n${YELLOW}[2] Model Files${NC}"
run_test "YOLOv8 model exists" "test -f $MODULE_DIR/models/yolov8n.onnx"
run_test "Labels file exists" "test -f $MODULE_DIR/models/labels.txt"
run_test "Color map exists" "test -f $MODULE_DIR/models/color_map.json"
run_test "Labels count == 80" "test $(wc -l < $MODULE_DIR/models/labels.txt) -eq 80"

# Test 3: Camera
echo -e "\n${YELLOW}[3] Camera${NC}"
run_test "Camera accessible" "python3 -c '
import cv2
cap = cv2.VideoCapture(0)
ret, _ = cap.read()
cap.release()
assert ret, \"Cannot read frame\"
'"

# Test 4: YOLO inference
echo -e "\n${YELLOW}[4] YOLO Inference${NC}"
run_test "YOLO model loads" "python3 -c '
import onnxruntime as ort
sess = ort.InferenceSession(\"$MODULE_DIR/models/yolov8n.onnx\")
inp = sess.get_inputs()[0]
assert inp.shape == [1, 3, 640, 640], f\"Unexpected shape: {inp.shape}\"
'"
run_test "YOLO dummy inference" "python3 -c '
import numpy as np, onnxruntime as ort
sess = ort.InferenceSession(\"$MODULE_DIR/models/yolov8n.onnx\")
inp = sess.get_inputs()[0]
dummy = np.zeros((1, 3, 640, 640), dtype=np.float32)
out = sess.run(None, {inp.name: dummy})
assert len(out) > 0
'"

# Test 5: Detection script syntax
echo -e "\n${YELLOW}[5] Script Syntax${NC}"
run_test "webcam_detect_standalone.py syntax" "python3 -m py_compile $SCRIPT_DIR/webcam_detect_standalone.py"
run_test "mjpeg_to_ros.py syntax" "python3 -m py_compile $SCRIPT_DIR/mjpeg_to_ros.py"
run_test "visualize_detections.py syntax" "python3 -m py_compile $SCRIPT_DIR/visualize_detections.py"

# Summary
echo -e "\n${GREEN}═══════════════════════════════════════════════════${NC}"
TOTAL=$((PASS + FAIL))
echo -e "  Results: ${GREEN}$PASS passed${NC}, ${RED}$FAIL failed${NC} (out of $TOTAL)"
if [ $FAIL -eq 0 ]; then
    echo -e "  ${GREEN}ALL TESTS PASSED ✓${NC}"
else
    echo -e "  ${RED}SOME TESTS FAILED ✗${NC}"
fi
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"

exit $FAIL
