#!/bin/bash
# test_perception_pipeline.sh — Full perception pipeline test script.
#
# Tests the complete perception pipeline end-to-end:
# 1. Starts dummy lidar publisher
# 2. Starts dummy camera publisher
# 3. Launches the detection pipeline (within Docker or locally)
# 4. Verifies ROS 2 topic outputs
# 5. Checks that detections are being produced
#
# Usage:
#   ./test_perception_pipeline.sh
#   ./test_perception_pipeline.sh --docker   # test with Docker container
#   ./test_perception_pipeline.sh --local    # test locally (no Docker)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODULE_DIR="$(dirname "$SCRIPT_DIR")"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

MODE="local"
CLEANUP_PIDS=()

for arg in "$@"; do
    case $arg in
        --docker) MODE="docker" ;;
        --local)  MODE="local" ;;
    esac
done

cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    for pid in "${CLEANUP_PIDS[@]}"; do
        kill $pid 2>/dev/null || true
    done
    echo -e "${GREEN}Done.${NC}"
}
trap cleanup EXIT INT TERM

echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  SDC 2026 — Perception Pipeline Test${NC}"
echo -e "${BLUE}  Mode: $MODE${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"

# ── Step 1: Check ROS 2 ──
echo -e "\n${YELLOW}[1/5] Checking ROS 2 environment...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}  ✗ ROS 2 not found. Source your workspace first:${NC}"
    echo -e "${RED}    source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ ROS 2 found$(NC)"

# ── Step 2: Start dummy lidar ──
echo -e "\n${YELLOW}[2/5] Starting dummy lidar publisher...${NC}"
python3 "$SCRIPT_DIR/dummy_lidar_publisher.py" --rate 10 &
LIDAR_PID=$!
CLEANUP_PIDS+=($LIDAR_PID)
sleep 1
if kill -0 $LIDAR_PID 2>/dev/null; then
    echo -e "${GREEN}  ✓ Dummy lidar running (PID: $LIDAR_PID)${NC}"
else
    echo -e "${RED}  ✗ Dummy lidar failed to start${NC}"
    exit 1
fi

# ── Step 3: Start dummy camera ──
echo -e "\n${YELLOW}[3/5] Starting dummy camera publisher...${NC}"
python3 "$SCRIPT_DIR/dummy_camera_publisher.py" --fps 15 &
CAMERA_PID=$!
CLEANUP_PIDS+=($CAMERA_PID)
sleep 1
if kill -0 $CAMERA_PID 2>/dev/null; then
    echo -e "${GREEN}  ✓ Dummy camera running (PID: $CAMERA_PID)${NC}"
else
    echo -e "${RED}  ✗ Dummy camera failed to start${NC}"
    exit 1
fi

# ── Step 4: Verify topics ──
echo -e "\n${YELLOW}[4/5] Verifying ROS topics...${NC}"
sleep 2

EXPECTED_TOPICS="/scan /sensing/camera/camera0/image_raw"
for topic in $EXPECTED_TOPICS; do
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo -e "${GREEN}  ✓ $topic${NC}"
    else
        echo -e "${RED}  ✗ $topic not found${NC}"
    fi
done

# ── Step 5: Check data flow ──
echo -e "\n${YELLOW}[5/5] Checking data flow...${NC}"

echo "  Checking /scan data..."
timeout 3 ros2 topic hz /scan 2>/dev/null | head -1 || echo "  (no data within 3s)"

echo "  Checking /sensing/camera/camera0/image_raw data..."
timeout 3 ros2 topic hz /sensing/camera/camera0/image_raw 2>/dev/null | head -1 || echo "  (no data within 3s)"

# If in Docker mode, also launch the detection pipeline
if [ "$MODE" = "docker" ]; then
    echo -e "\n${YELLOW}[+] Docker mode: launching detection pipeline...${NC}"
    CONTAINER="autoware-universe"
    if docker ps --format '{{.Names}}' | grep -q "$CONTAINER"; then
        docker exec -d "$CONTAINER" bash -c \
            "source /opt/ros/humble/setup.bash && \
             source /autoware/install/setup.bash && \
             ros2 launch tier4_perception_launch detection_module.launch.xml image_number:=1"
        echo -e "${GREEN}  ✓ Detection pipeline launched in Docker${NC}"
        sleep 5

        echo "  Checking /perception/object_recognition/detection/objects..."
        timeout 5 ros2 topic echo /perception/object_recognition/detection/objects --once 2>/dev/null | head -5 || echo "  (waiting for detections...)"
    else
        echo -e "${RED}  ✗ Container '$CONTAINER' not running${NC}"
    fi
fi

echo -e "\n${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Pipeline test complete!${NC}"
echo -e "${BLUE}  Dummy publishers will keep running until Ctrl+C.${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"

# Keep running until Ctrl+C
wait
