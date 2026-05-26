#!/bin/bash
# run.sh — Quick-start script for the perception module.
#
# Sets up the environment and launches the standalone detection script
# or the full ROS 2 pipeline.
#
# Usage:
#   ./run.sh                    # standalone mode (webcam, no ROS/Docker)
#   ./run.sh --pipeline         # full ROS 2 pipeline with Docker
#   ./run.sh --test             # run all tests
#   ./run.sh --setup            # install dependencies only

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

MODE="standalone"
for arg in "$@"; do
    case $arg in
        --pipeline) MODE="pipeline" ;;
        --test)     MODE="test" ;;
        --setup)    MODE="setup" ;;
        --help|-h)
            echo "Usage: ./run.sh [--pipeline|--test|--setup]"
            echo ""
            echo "Modes:"
            echo "  (default)    Standalone webcam detection (no ROS/Docker)"
            echo "  --pipeline   Full ROS 2 detection pipeline with Docker"
            echo "  --test       Run end-to-end tests"
            echo "  --setup      Install Python dependencies only"
            exit 0
            ;;
    esac
done

echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  SDC 2026 — Perception Module${NC}"
echo -e "${GREEN}  Mode: $MODE${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"

# ── Setup virtual environment ──
setup_venv() {
    if [ ! -d "$SCRIPT_DIR/venv" ]; then
        echo -e "${YELLOW}Creating virtual environment...${NC}"
        python3 -m venv "$SCRIPT_DIR/venv"
    fi
    source "$SCRIPT_DIR/venv/bin/activate"

    echo -e "${YELLOW}Installing dependencies...${NC}"
    pip install --quiet --upgrade pip
    pip install --quiet opencv-python numpy onnxruntime
    echo -e "${GREEN}  ✓ Dependencies installed${NC}"
}

# ── Download model if needed ──
check_model() {
    MODEL_PATH="$SCRIPT_DIR/models/yolov8n.onnx"
    if [ ! -f "$MODEL_PATH" ]; then
        echo -e "${YELLOW}Downloading YOLOv8n model...${NC}"
        mkdir -p "$SCRIPT_DIR/models"
        curl -L "https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.onnx" \
            -o "$MODEL_PATH"
        echo -e "${GREEN}  ✓ Model downloaded ($(du -h $MODEL_PATH | cut -f1))${NC}"
    else
        echo -e "${GREEN}  ✓ Model exists ($(du -h $MODEL_PATH | cut -f1))${NC}"
    fi
}

case $MODE in
    setup)
        setup_venv
        check_model
        echo -e "\n${GREEN}Setup complete!${NC}"
        ;;
    standalone)
        setup_venv
        check_model
        echo -e "\n${GREEN}Starting standalone detection...${NC}"
        echo -e "${GREEN}Press 'q' to quit.${NC}\n"
        python3 "$SCRIPT_DIR/scripts/webcam_detect_standalone.py"
        ;;
    pipeline)
        setup_venv
        check_model
        echo -e "\n${GREEN}Starting Docker pipeline...${NC}\n"
        bash "$SCRIPT_DIR/scripts/mac_docker_start.sh"
        ;;
    test)
        setup_venv
        check_model
        echo -e "\n${GREEN}Running tests...${NC}\n"
        bash "$SCRIPT_DIR/test_scripts/test_detection_e2e.sh"
        ;;
esac
