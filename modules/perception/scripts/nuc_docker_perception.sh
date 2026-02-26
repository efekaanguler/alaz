#!/bin/bash
# nuc_docker_perception.sh — NUC deployment script for perception + sensor fusion pipeline.
#
# Runs INSIDE Docker on the NUC with real LiDAR + real camera.
# Handles:
#   1. Auto-detection of /scan and /points_raw topics (real LiDAR)
#   2. Auto-detection of camera image topics
#   3. ROI Cluster Fusion (LiDAR clusters + Camera 2D detection ROIs)
#   4. Full detection pipeline: YOLOv8 → ByteTrack → Autoware Bridge
#   5. Traffic light detection + classification
#   6. If no LiDAR → falls back to camera-only mode with dummy LaserScan
#
# Expected Autoware packages (pre-installed in Docker):
#   - autoware_image_projection_based_fusion (roi_cluster_fusion)
#   - autoware_euclidean_cluster (LiDAR clustering)
#   - pointcloud_preprocessor (voxel grid, ground removal)
#
# Usage (inside Docker):
#   bash /workspace/modules/perception/scripts/nuc_docker_perception.sh
#   bash /workspace/modules/perception/scripts/nuc_docker_perception.sh --cameras 2
#   bash /workspace/modules/perception/scripts/nuc_docker_perception.sh --no-fusion  # camera-only

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODULE_DIR="$(dirname "$SCRIPT_DIR")"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

CAMERA_COUNT=1
SKIP_FUSION=false
ENABLE_VIZ=false
LIDAR_TOPIC="/points_raw"
SCAN_TOPIC="/scan"

for arg in "$@"; do
    case $arg in
        --cameras=*) CAMERA_COUNT="${arg#--cameras=}" ;;
        --no-fusion) SKIP_FUSION=true ;;
        --viz)       ENABLE_VIZ=true ;;
        --lidar=*)   LIDAR_TOPIC="${arg#--lidar=}" ;;
        --scan=*)    SCAN_TOPIC="${arg#--scan=}" ;;
        --help|-h)
            echo "Usage: nuc_docker_perception.sh [options]"
            echo ""
            echo "Options:"
            echo "  --cameras=N     Number of cameras (default: 1)"
            echo "  --no-fusion     Camera-only mode (no LiDAR fusion)"
            echo "  --viz           Enable debug visualizer"
            echo "  --lidar=TOPIC   LiDAR PointCloud2 topic (default: /points_raw)"
            echo "  --scan=TOPIC    LaserScan topic (default: /scan)"
            exit 0
            ;;
    esac
done

echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  SDC 2026 — NUC Perception + Sensor Fusion Pipeline${NC}"
echo -e "${CYAN}  Cameras: $CAMERA_COUNT  |  Fusion: $([ "$SKIP_FUSION" = true ] && echo 'OFF' || echo 'ON')${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"

# ── Source ROS 2 ──
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /autoware/install/setup.bash 2>/dev/null || true

# Check if detection_ws is built
if [ -d "$MODULE_DIR/detection_ws/install" ]; then
    source "$MODULE_DIR/detection_ws/install/setup.bash"
    echo -e "${GREEN}[✓] detection_ws sourced${NC}"
else
    echo -e "${YELLOW}[!] detection_ws not built yet — building...${NC}"
    cd "$MODULE_DIR/detection_ws"
    colcon build --symlink-install 2>&1 | tail -5
    source install/setup.bash
    echo -e "${GREEN}[✓] detection_ws built and sourced${NC}"
fi

# ══════════════════════════════════════
# STEP 1: Detect sensors
# ══════════════════════════════════════
echo -e "\n${BLUE}[1/5] Detecting sensors...${NC}"

HAS_LIDAR=false
HAS_CAMERA=false

# Check for LiDAR
TOPICS=$(timeout 3 ros2 topic list 2>/dev/null || echo "")
if echo "$TOPICS" | grep -q "$LIDAR_TOPIC"; then
    HAS_LIDAR=true
    echo -e "${GREEN}  ✓ LiDAR PointCloud2: $LIDAR_TOPIC${NC}"
elif echo "$TOPICS" | grep -q "$SCAN_TOPIC"; then
    HAS_LIDAR=true
    echo -e "${GREEN}  ✓ LiDAR LaserScan: $SCAN_TOPIC${NC}"
elif echo "$TOPICS" | grep -q "/sensing/lidar"; then
    LIDAR_TOPIC=$(echo "$TOPICS" | grep "/sensing/lidar" | head -1)
    HAS_LIDAR=true
    echo -e "${GREEN}  ✓ LiDAR detected: $LIDAR_TOPIC${NC}"
else
    echo -e "${YELLOW}  ⚠ No LiDAR topic found${NC}"
fi

# Check for camera
for i in $(seq 0 $((CAMERA_COUNT - 1))); do
    CAM_TOPIC="/sensing/camera/camera${i}/image_raw"
    if echo "$TOPICS" | grep -q "$CAM_TOPIC"; then
        HAS_CAMERA=true
        echo -e "${GREEN}  ✓ Camera $i: $CAM_TOPIC${NC}"
    else
        echo -e "${YELLOW}  ⚠ Camera $i not found ($CAM_TOPIC)${NC}"
    fi
done

# ══════════════════════════════════════
# STEP 2: LiDAR → PointCloud2 (ara adım — her zaman gerekli)
# ══════════════════════════════════════
echo -e "\n${BLUE}[2/5] LiDAR → PointCloud2 dönüşümü...${NC}"

if [ "$HAS_LIDAR" = false ]; then
    echo -e "${YELLOW}  No LiDAR — starting dummy LaserScan for costmap...${NC}"
    python3 "$SCRIPT_DIR/dummy_lidar_publisher.py" --rate 10 --obstacle &
    sleep 1
    echo -e "${GREEN}  ✓ Dummy LiDAR running (10 Hz)${NC}"
fi

# LaserScan → PointCloud2 (2D LiDAR → PointCloud2 dönüşümü)
# Bu adım hem ROI cluster fusion'ı hem costmap'i besler
echo -e "${YELLOW}  Starting LaserScan → PointCloud2 converter...${NC}"
ros2 run pointcloud_to_laserscan laserscan_to_pointcloud_node \
    --ros-args \
    -r scan_in:="$SCAN_TOPIC" \
    -r cloud:="/perception/lidar/pointcloud" \
    -p target_frame:="base_link" &
sleep 1
echo -e "${GREEN}  ✓ LaserScan → PointCloud2 → /perception/lidar/pointcloud${NC}"

# Paralel yol A: PointCloud2 → OccupancyGrid → Costmap
echo -e "${YELLOW}  Starting PointCloud2 → OccupancyGrid (costmap)...${NC}"
ros2 launch perception laserscan_to_pcl_and_occ.launch.xml 2>/dev/null &
sleep 1
echo -e "${GREEN}  ✓ OccupancyGrid → Costmap pipeline running${NC}"

# Paralel yol B: PointCloud2 → Euclidean Clustering (ROI fusion için)
if [ "$SKIP_FUSION" = false ]; then
    echo -e "${YELLOW}  Starting Euclidean clustering...${NC}"
    if ros2 pkg list 2>/dev/null | grep -q "autoware_euclidean_cluster"; then
        ros2 launch autoware_euclidean_cluster voxel_grid_based_euclidean_cluster.launch.xml \
            input_pointcloud:="/perception/lidar/pointcloud" \
            output_clusters:="/perception/lidar/clusters" &
        sleep 2
        echo -e "${GREEN}  ✓ Euclidean clustering → /perception/lidar/clusters${NC}"
    else
        echo -e "${YELLOW}  ⚠ autoware_euclidean_cluster not installed — ROI fusion will not have clusters${NC}"
    fi
fi

# ══════════════════════════════════════
# STEP 3: Detection pipeline (YOLOv8 → ByteTrack → Bridge)
# ══════════════════════════════════════
echo -e "\n${BLUE}[3/5] Detection pipeline (YOLOv8 → ByteTrack → Bridge)...${NC}"

# Determine model path
MODEL_DIR="/workspace/modules/detection/models"
if [ ! -f "$MODEL_DIR/yolov8n.onnx" ]; then
    MODEL_DIR="$MODULE_DIR/models"
fi

ros2 launch tier4_perception_launch detection_module.launch.xml \
    image_number:="$CAMERA_COUNT" \
    camera_2d_detector/model_path:="$MODEL_DIR/yolov8n.onnx" \
    camera_2d_detector/label_path:="$MODEL_DIR/labels.txt" \
    camera_2d_detector/color_map_path:="$MODEL_DIR/color_map.json" \
    enable_visualizer:="$ENABLE_VIZ" &
sleep 3
echo -e "${GREEN}  ✓ Detection pipeline launched:${NC}"
echo -e "${GREEN}    Camera(s) → YOLOv8 → ByteTrack → tracked rois${NC}"
echo -e "${GREEN}    Camera → TL ROI detector → TL classifier${NC}"
echo -e "${GREEN}    Tracked rois + TL signals → Autoware Bridge${NC}"

# ══════════════════════════════════════
# STEP 4: ROI Cluster Fusion (LiDAR + Camera)
# ══════════════════════════════════════
echo -e "\n${BLUE}[4/5] ROI Cluster Fusion...${NC}"

if [ "$HAS_LIDAR" = true ] && [ "$SKIP_FUSION" = false ]; then
    if ros2 pkg list 2>/dev/null | grep -q "autoware_image_projection_based_fusion"; then
        echo -e "${GREEN}  Launching roi_cluster_fusion...${NC}"

        # ROI Cluster Fusion node
        # Input: 3D clusters (from euclidean_cluster) + 2D ROIs (from YOLOv8)
        # Output: fused DetectedObjectsWithFeature (overwritten labels)
        ros2 run autoware_image_projection_based_fusion roi_cluster_fusion_node \
            --ros-args \
            -r input:="/perception/lidar/clusters" \
            -r input/rois0:="/rois0" \
            -r input/camera_info0:="/sensing/camera/camera0/camera_info" \
            -r input/image_raw0:="/sensing/camera/camera0/image_raw" \
            -r output:="/perception/object_recognition/detection/fused_objects" \
            -p trust_distance:=40.0 \
            -p fusion_distance:=100.0 \
            -p iou_threshold.UNKNOWN:=0.1 \
            -p iou_threshold.CAR:=0.65 \
            -p iou_threshold.TRUCK:=0.65 \
            -p iou_threshold.BUS:=0.65 \
            -p iou_threshold.MOTORCYCLE:=0.65 \
            -p iou_threshold.BICYCLE:=0.65 \
            -p iou_threshold.PEDESTRIAN:=0.65 &
        sleep 2
        echo -e "${GREEN}  ✓ ROI Cluster Fusion running${NC}"
        echo -e "${GREEN}    Input: /perception/lidar/clusters + /rois0 + camera_info${NC}"
        echo -e "${GREEN}    Output: /perception/object_recognition/detection/fused_objects${NC}"
    else
        echo -e "${YELLOW}  ⚠ autoware_image_projection_based_fusion not installed${NC}"
        echo -e "${YELLOW}    Using camera-only detection (no LiDAR fusion)${NC}"
    fi
else
    echo -e "${YELLOW}  Skipping ROI fusion (no LiDAR or --no-fusion)${NC}"
    echo -e "${YELLOW}  Using camera-only detection via bridge output${NC}"
fi

# ══════════════════════════════════════
# STEP 5: Verify output topics
# ══════════════════════════════════════
echo -e "\n${BLUE}[5/5] Verifying output topics...${NC}"
sleep 3

EXPECTED_TOPICS=(
    "/perception/object_recognition/detection/objects"
    "/perception/object_recognition/tracking/objects"
    "/perception/object_recognition/objects"
    "/perception/traffic_light_recognition/traffic_signals"
)

FOUND=0
for topic in "${EXPECTED_TOPICS[@]}"; do
    if timeout 2 ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo -e "${GREEN}  ✓ $topic${NC}"
        FOUND=$((FOUND + 1))
    else
        echo -e "${YELLOW}  ⚠ $topic (not yet active — may need data)${NC}"
    fi
done

# ══════════════════════════════════════
# Pipeline info
# ══════════════════════════════════════
echo -e "\n${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  NUC Perception Pipeline Running!${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "  ${BLUE}Pipeline Architecture:${NC}"
echo "    2D LiDAR → LaserScan → PointCloud2 ─┬─ Clustering → ROI Cluster Fusion"
echo "                                         └─ OccupancyGrid → Costmap"
echo "    Camera → YOLOv8 → ByteTrack → Bridge → Autoware"
echo "    Camera → YOLOv8 TL → TL Classifier → Bridge → TrafficLightGroupArray"
echo ""
echo -e "  ${BLUE}Monitor:${NC}"
echo "    ros2 topic hz /perception/object_recognition/detection/objects"
echo "    ros2 topic hz /perception/lidar/pointcloud"
echo "    ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once"
echo ""

echo -e "  ${YELLOW}Press Ctrl+C to stop all nodes.${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"

wait
