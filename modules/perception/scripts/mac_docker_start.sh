#!/bin/bash
# docker_pipeline_start.sh — Start the full perception + sensor fusion pipeline in Docker.
#
# Pipeline:
#   Mac Webcam → MJPEG → ROS Image → YOLOv8 → ByteTrack → Autoware Bridge
#   LiDAR (real /scan) or Dummy LaserScan → PointCloud2 → OccupancyGrid
#
# Auto-detects if a real /scan topic is available (NUC with LiDAR).
# If not, starts the dummy lidar publisher as fallback.
#
# Usage:
#   ./docker_pipeline_start.sh                  # full pipeline (auto-detect lidar)
#   ./docker_pipeline_start.sh --dummy-lidar    # force dummy lidar
#   ./docker_pipeline_start.sh --no-lidar       # skip lidar entirely
#   ./docker_pipeline_start.sh --no-viz         # without visualization
#   ./docker_pipeline_start.sh --nuc            # NUC mode (real sensors, no webcam bridge)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODULE_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$(dirname "$MODULE_DIR")"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ── Parse arguments ──
NO_VIZ=false
LIDAR_MODE="auto"   # auto | dummy | none
NUC_MODE=false
CAMERA_COUNT=1

for arg in "$@"; do
    case $arg in
        --no-viz)       NO_VIZ=true ;;
        --dummy-lidar)  LIDAR_MODE="dummy" ;;
        --no-lidar)     LIDAR_MODE="none" ;;
        --nuc)          NUC_MODE=true ;;
        --cameras=*)    CAMERA_COUNT="${arg#--cameras=}" ;;
    esac
done

CLEANUP_PIDS=()
cleanup() {
    echo -e "\n${YELLOW}Cleaning up pipeline...${NC}"
    for pid in "${CLEANUP_PIDS[@]}"; do
        kill $pid 2>/dev/null || true
    done
    docker exec "$CONTAINER_NAME" bash -c "pkill -f 'ros2 launch\|yolov8_node\|bytetrack\|detection_autoware_bridge'" 2>/dev/null || true
    echo -e "${GREEN}Pipeline stopped.${NC}"
}
trap cleanup EXIT INT TERM

echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  SDC 2026 — Perception + Sensor Fusion Pipeline${NC}"
echo -e "${GREEN}  Mode: $([ "$NUC_MODE" = true ] && echo 'NUC (real sensors)' || echo 'Mac (webcam + bridge)')${NC}"
echo -e "${GREEN}  Lidar: $LIDAR_MODE  |  Cameras: $CAMERA_COUNT${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"

# ── Step 0: Docker check ──
CONTAINER_NAME="autoware-universe"
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker is not running. Please start Docker Desktop.${NC}"
    exit 1
fi

if ! docker ps -a --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
    echo -e "${RED}ERROR: Container '$CONTAINER_NAME' not found.${NC}"
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
    echo -e "${YELLOW}Starting container '$CONTAINER_NAME'...${NC}"
    docker start "$CONTAINER_NAME"
    sleep 2
fi
echo -e "${GREEN}[✓] Docker container running${NC}"

# ── Step 1: Camera input ──
echo -e "\n${BLUE}[1/4] Camera input...${NC}"
if [ "$NUC_MODE" = true ]; then
    echo -e "${GREEN}  NUC mode: using real camera topics from sensor_kit${NC}"
else
    # Mac mode: start MJPEG stream + ROS bridge
    if python3 -c "import cv2; cap=cv2.VideoCapture(0); ret,_=cap.read(); cap.release(); assert ret" 2>/dev/null; then
        echo -e "${GREEN}  ✓ Webcam accessible${NC}"
    else
        echo -e "${RED}  ✗ Cannot open webcam (device 0)${NC}"
        exit 1
    fi

    # Start webcam MJPEG stream server
    python3 "$WORKSPACE_DIR/modules/sensor_start/mac_webcam_stream.py" &
    CLEANUP_PIDS+=($!)
    sleep 1
    echo -e "${GREEN}  ✓ MJPEG stream started on :8090${NC}"

    # Start MJPEG → ROS Image bridge (inside Docker)
    docker exec -d "$CONTAINER_NAME" bash -c \
        "source /opt/ros/humble/setup.bash && \
         source /autoware/install/setup.bash && \
         python3 $SCRIPT_DIR/mjpeg_to_ros.py \
           --url http://host.docker.internal:8090/video"
    sleep 1
    echo -e "${GREEN}  ✓ MJPEG → ROS bridge started in Docker${NC}"
fi

# ── Step 2: LiDAR input (auto-detect / dummy / none) ──
echo -e "\n${BLUE}[2/4] LiDAR input (mode: $LIDAR_MODE)...${NC}"
if [ "$LIDAR_MODE" = "none" ]; then
    echo -e "${YELLOW}  Skipping LiDAR (--no-lidar)${NC}"
elif [ "$LIDAR_MODE" = "dummy" ]; then
    echo -e "${YELLOW}  Starting dummy lidar publisher...${NC}"
    docker exec -d "$CONTAINER_NAME" bash -c \
        "source /opt/ros/humble/setup.bash && \
         python3 /workspace/modules/perception/test_scripts/dummy_lidar_publisher.py --rate 10 --obstacle"
    sleep 1
    echo -e "${GREEN}  ✓ Dummy lidar running (10 Hz, with obstacle)${NC}"
else
    # Auto-detect: check if /scan topic exists
    echo -e "${YELLOW}  Auto-detecting /scan topic...${NC}"
    SCAN_EXISTS=$(docker exec "$CONTAINER_NAME" bash -c \
        "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic list 2>/dev/null | grep -c '/scan'" 2>/dev/null || echo "0")

    if [ "$SCAN_EXISTS" -gt 0 ]; then
        echo -e "${GREEN}  ✓ Real /scan topic detected — using real LiDAR${NC}"
    else
        echo -e "${YELLOW}  ⚠ No /scan topic found — starting dummy lidar fallback${NC}"
        docker exec -d "$CONTAINER_NAME" bash -c \
            "source /opt/ros/humble/setup.bash && \
             python3 /workspace/modules/perception/test_scripts/dummy_lidar_publisher.py --rate 10 --obstacle"
        sleep 1
        echo -e "${GREEN}  ✓ Dummy lidar running as fallback${NC}"
    fi

    # Start LaserScan → PointCloud2 → OccupancyGrid pipeline
    docker exec -d "$CONTAINER_NAME" bash -c \
        "source /opt/ros/humble/setup.bash && \
         source /autoware/install/setup.bash && \
         ros2 launch perception laserscan_to_pcl_and_occ.launch.xml" 2>/dev/null || true
    echo -e "${GREEN}  ✓ LaserScan → PointCloud2 pipeline launched${NC}"
fi

# ── Step 3: Detection + Tracking + Bridge ──
echo -e "\n${BLUE}[3/4] Detection pipeline (YOLOv8 → ByteTrack → Autoware Bridge)...${NC}"
docker exec -d "$CONTAINER_NAME" bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /autoware/install/setup.bash && \
     ros2 launch tier4_perception_launch detection_module.launch.xml \
       image_number:=$CAMERA_COUNT"
sleep 2
echo -e "${GREEN}  ✓ Detection pipeline launched:${NC}"
echo -e "${GREEN}    YOLOv8 detector → ROIs${NC}"
echo -e "${GREEN}    ByteTrack tracker → tracked ROIs${NC}"
echo -e "${GREEN}    Autoware bridge → DetectedObjects / TrackedObjects / PredictedObjects${NC}"
echo -e "${GREEN}    Traffic light ROI → TL classifier → TrafficLightGroupArray${NC}"

# ── Step 4: Visualization ──
if [ "$NO_VIZ" = false ]; then
    echo -e "\n${BLUE}[4/4] Starting detection visualizer...${NC}"
    if [ "$NUC_MODE" = true ]; then
        docker exec -d "$CONTAINER_NAME" bash -c \
            "source /opt/ros/humble/setup.bash && \
             source /autoware/install/setup.bash && \
             python3 /workspace/modules/perception/scripts/visualize_detections.py"
    else
        python3 "$SCRIPT_DIR/visualize_detections.py" &
        CLEANUP_PIDS+=($!)
    fi
    echo -e "${GREEN}  ✓ Visualizer started${NC}"
else
    echo -e "\n${YELLOW}[4/4] Visualization skipped (--no-viz)${NC}"
fi

# ── Pipeline info ──
echo -e "\n${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Pipeline running!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e ""
echo -e "  ${BLUE}Autoware output topics:${NC}"
echo -e "    /perception/object_recognition/detection/objects"
echo -e "    /perception/object_recognition/tracking/objects"
echo -e "    /perception/object_recognition/objects"
echo -e "    /perception/traffic_light_recognition/traffic_signals"
echo -e ""
echo -e "  ${BLUE}Monitor:${NC}"
echo -e "    docker exec $CONTAINER_NAME ros2 topic list"
echo -e "    docker exec $CONTAINER_NAME ros2 topic hz /perception/object_recognition/detection/objects"
echo -e ""
echo -e "  ${YELLOW}Press Ctrl+C to stop.${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"

# Keep running
wait
