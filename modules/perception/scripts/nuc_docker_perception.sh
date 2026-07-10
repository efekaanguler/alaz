#!/bin/bash
# nuc_docker_perception.sh — NUC deployment script for perception + sensor fusion pipeline.
#
# Runs INSIDE Docker on the NUC with real LiDAR + real camera.
# Handles:
#   1. Auto-detection of /sensing/scan and /points_raw topics (real LiDAR)
#   2. Auto-detection of camera image topics
#   3. Fallback camera_info + static TF (only when sensor kit calibration is missing)
#   4. ROI Cluster Fusion (LiDAR clusters + Camera 2D detection ROIs)
#   5. Full detection pipeline: YOLOv8 -> ByteTrack -> Autoware Bridge
#   6. Traffic light detection + classification
#   7. Parallel pointcloud -> occupancy grid fallback for planner costmap
#
# Usage (inside Docker):
#   bash /workspace/modules/perception/scripts/nuc_docker_perception.sh
#   bash /workspace/modules/perception/scripts/nuc_docker_perception.sh --cameras 2
#   bash /workspace/modules/perception/scripts/nuc_docker_perception.sh --no-fusion

set -e

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [[ "$SCRIPT_DIR" == *"/install/"* ]]; then
    # Running from ROS 2 install space
    MODULE_DIR="/workspace/modules/perception"
else
    # Running from source space
    MODULE_DIR="$(dirname "$SCRIPT_DIR")"
fi

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
SCAN_TOPIC="/sensing/scan"
PRIMARY_CAMERA_TOPIC="/sensing/image_raw"
BASE_FRAME="base_link"

ENABLE_FALLBACK_CAMERA_INFO=true
ENABLE_FALLBACK_TF=true
ENABLE_FALLBACK_OCCUPANCY=true
CAMERA_INFO_YAML="$MODULE_DIR/config/nuc/camera_info_logitech_720p.yaml"
FORCE_DUMMY_CAMERA=false
FORCE_DUMMY_LIDAR=false
DUMMY_CAMERA_WIDTH=1280
DUMMY_CAMERA_HEIGHT=720
DUMMY_CAMERA_FPS=15.0
EXIT_AFTER_VERIFY=false

LIDAR_POINTCLOUD_TOPIC="/perception/lidar/pointcloud"
LIDAR_CLUSTER_TOPIC="/perception/lidar/clusters"
OBSTACLE_POINTCLOUD_TOPIC="/perception/obstacle/pointcloud"
OCCUPANCY_TOPIC="/perception/occupancy_grid"
FUSED_OBJECTS_TOPIC="/perception/object_recognition/detection/fused_objects"
FUSION_ROIS_TOPIC="/perception/fusion/rois0"
ROI_FUSION_MSG3D_TIMEOUT_SEC="${ROI_FUSION_MSG3D_TIMEOUT_SEC:-0.5}"
ROI_FUSION_ROIS_TIMEOUT_SEC="${ROI_FUSION_ROIS_TIMEOUT_SEC:-0.5}"
ROI_FUSION_ROIS_TIMESTAMP_OFFSETS="${ROI_FUSION_ROIS_TIMESTAMP_OFFSETS:-[0.0]}"
ROI_FUSION_POINT_PROJECT_TO_UNRECTIFIED_IMAGE="${ROI_FUSION_POINT_PROJECT_TO_UNRECTIFIED_IMAGE:-[true]}"
ROI_FUSION_APPROXIMATE_CAMERA_PROJECTION="${ROI_FUSION_APPROXIMATE_CAMERA_PROJECTION:-[false]}"
ROI_FUSION_APPROX_GRID_CELL_WIDTH="${ROI_FUSION_APPROX_GRID_CELL_WIDTH:-1.0}"
ROI_FUSION_APPROX_GRID_CELL_HEIGHT="${ROI_FUSION_APPROX_GRID_CELL_HEIGHT:-1.0}"
ROI_FUSION_IMAGE_BUFFER_SIZE="${ROI_FUSION_IMAGE_BUFFER_SIZE:-15}"
ROI_FUSION_FILTER_SCOPE_MIN_X="${ROI_FUSION_FILTER_SCOPE_MIN_X:--100.0}"
ROI_FUSION_FILTER_SCOPE_MIN_Y="${ROI_FUSION_FILTER_SCOPE_MIN_Y:--100.0}"
ROI_FUSION_FILTER_SCOPE_MIN_Z="${ROI_FUSION_FILTER_SCOPE_MIN_Z:--100.0}"
ROI_FUSION_FILTER_SCOPE_MAX_X="${ROI_FUSION_FILTER_SCOPE_MAX_X:-100.0}"
ROI_FUSION_FILTER_SCOPE_MAX_Y="${ROI_FUSION_FILTER_SCOPE_MAX_Y:-100.0}"
ROI_FUSION_FILTER_SCOPE_MAX_Z="${ROI_FUSION_FILTER_SCOPE_MAX_Z:-100.0}"
ROI_FUSION_DEBUG_MODE="${ROI_FUSION_DEBUG_MODE:-false}"
ROI_FUSION_COLLECTOR_DEBUG_MODE="${ROI_FUSION_COLLECTOR_DEBUG_MODE:-false}"
ROI_FUSION_PUBLISH_PROCESSING_TIME_DETAIL="${ROI_FUSION_PUBLISH_PROCESSING_TIME_DETAIL:-false}"
ROI_FUSION_PUBLISH_PREVIOUS_BUT_LATE_OUTPUT_MSG="${ROI_FUSION_PUBLISH_PREVIOUS_BUT_LATE_OUTPUT_MSG:-false}"
ROI_FUSION_ROSBAG_LENGTH="${ROI_FUSION_ROSBAG_LENGTH:-10.0}"
ROI_FUSION_MATCHING_STRATEGY_TYPE="${ROI_FUSION_MATCHING_STRATEGY_TYPE:-naive}"
ROI_FUSION_MATCHING_STRATEGY_THRESHOLD="${ROI_FUSION_MATCHING_STRATEGY_THRESHOLD:-0.05}"
ROI_FUSION_MATCHING_STRATEGY_MSG3D_NOISE_WINDOW="${ROI_FUSION_MATCHING_STRATEGY_MSG3D_NOISE_WINDOW:-0.001}"
ROI_FUSION_MATCHING_STRATEGY_ROIS_TIMESTAMP_NOISE_WINDOW="${ROI_FUSION_MATCHING_STRATEGY_ROIS_TIMESTAMP_NOISE_WINDOW:-[0.005]}"
ROI_FUSION_PARAMS_FILE="${ROI_FUSION_PARAMS_FILE:-}"

# Default fallback extrinsics (from sensor_kit_base_link ~= base_link)
CAM_X=(0.98 0.98 0.98 0.98 0.98 0.98 0.98 0.98 0.98 0.98)
CAM_Y=(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
CAM_Z=(0.39 0.39 0.39 0.39 0.39 0.39 0.39 0.39 0.39 0.39)
CAM_ROLL=(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
CAM_PITCH=(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
CAM_YAW=(0.0 0.52 -0.52 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
LIDAR_X=1.36
LIDAR_Y=0.0
LIDAR_Z=0.17
LIDAR_ROLL=0.0
LIDAR_PITCH=0.0
LIDAR_YAW=0.0

CLEANUP_PIDS=()
ROI_FUSION_STARTED=false
ROI_FUSION_RUNNING=false
ROI_FUSION_PID=""
CLEANUP_DONE=false
cleanup() {
    if [ "$CLEANUP_DONE" = true ]; then
        return
    fi
    CLEANUP_DONE=true
    if [ "${#CLEANUP_PIDS[@]}" -eq 0 ]; then
        return
    fi
    echo -e "\n${YELLOW}Stopping NUC perception pipeline...${NC}"
    for pid in "${CLEANUP_PIDS[@]:-}"; do
        if command -v pkill >/dev/null 2>&1; then
            pkill -TERM -P "$pid" 2>/dev/null || true
        fi
        kill "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${CLEANUP_PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            if command -v pkill >/dev/null 2>&1; then
                pkill -KILL -P "$pid" 2>/dev/null || true
            fi
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    for pid in "${CLEANUP_PIDS[@]:-}"; do
        wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

topic_exists() {
    local topic="$1"
    timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "$topic"
}

topic_type() {
    local topic="$1"
    ros2 topic type "$topic" 2>/dev/null | head -1
}

pkg_available() {
    local pkg="$1"
    ros2 pkg prefix "$pkg" >/dev/null 2>&1
}

pkg_share_dir() {
    local pkg="$1"
    local prefix
    prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null)" || return 1
    echo "$prefix/share/$pkg"
}

wait_for_topic() {
    local topic="$1"
    local timeout_s="${2:-10}"
    local t=0
    while [ "$t" -lt "$timeout_s" ]; do
        if topic_exists "$topic"; then
            return 0
        fi
        sleep 1
        t=$((t + 1))
    done
    return 1
}

wait_for_topic_type_match() {
    local topic="$1"
    local expected_substr="$2"
    local timeout_s="${3:-10}"
    local t=0
    local actual=""
    while [ "$t" -lt "$timeout_s" ]; do
        actual="$(topic_type "$topic" || true)"
        if [ -n "$actual" ] && echo "$actual" | grep -q "$expected_substr"; then
            return 0
        fi
        sleep 1
        t=$((t + 1))
    done
    echo "$actual"
    return 1
}

verify_topic_active() {
    local topic="$1"
    local timeout_s="${2:-3}"

    if wait_for_topic "$topic" "$timeout_s"; then
        return 0
    fi

    # Fallback check: sometimes `ros2 topic list` is flaky under load, but type query succeeds.
    local ttype
    ttype="$(topic_type "$topic" || true)"
    [ -n "$ttype" ]
}

find_roi_cluster_fusion_params_file() {
    if [ -n "$ROI_FUSION_PARAMS_FILE" ]; then
        if [ -f "$ROI_FUSION_PARAMS_FILE" ]; then
            echo "$ROI_FUSION_PARAMS_FILE"
            return 0
        fi
        return 1
    fi

    local share_dir
    share_dir="$(pkg_share_dir "autoware_image_projection_based_fusion" 2>/dev/null)" || return 1
    [ -d "$share_dir" ] || return 1

    local patterns=(
        '*roi_cluster_fusion*.param.yaml'
        '*roi_cluster_fusion*.yaml'
        '*roi*cluster*fusion*.yaml'
    )
    local pat
    local candidate
    local first_generic=""
    for pat in "${patterns[@]}"; do
        while IFS= read -r candidate; do
            [ -n "$candidate" ] || continue
            if grep -Eq 'approximation_grid_cell_width' "$candidate" 2>/dev/null; then
                echo "$candidate"
                return 0
            fi
            if [ -z "$first_generic" ] && grep -Eq 'ros__parameters|rois_number' "$candidate" 2>/dev/null; then
                first_generic="$candidate"
            fi
        done < <(find "$share_dir" -type f -name "$pat" 2>/dev/null | sort)
    done

    if [ -n "$first_generic" ]; then
        echo "$first_generic"
        return 0
    fi

    return 1
}

is_pid_alive() {
    local pid="$1"
    kill -0 "$pid" 2>/dev/null
}

show_help() {
    cat <<EOF
Usage: nuc_docker_perception.sh [options]

Options:
  --cameras N | --cameras=N          Number of cameras (default: 1)
  --no-fusion                        Camera-only mode (skip ROI fusion)
  --viz                              Enable detection visualizer
  --lidar TOPIC | --lidar=TOPIC      LiDAR PointCloud2 topic (default: /points_raw)
  --scan TOPIC | --scan=TOPIC        LiDAR LaserScan topic (default: /sensing/scan)
  --camera-topic TOPIC               Primary camera image topic (default: /sensing/image_raw)
  --base-frame FRAME                 Target/base frame for fallback TF (default: base_link)
  --camera-info-yaml PATH            ROS camera_info YAML for fallback publisher
  --dummy-camera                     Start dummy camera publisher(s) for local/Docker tests
  --dummy-lidar                      Force dummy LaserScan even if a real LiDAR topic exists
  --dummy-camera-width PX            Dummy camera width (default: 1280)
  --dummy-camera-height PX           Dummy camera height (default: 720)
  --dummy-camera-fps FPS             Dummy camera FPS (default: 15.0)
  --exit-after-verify | --smoke-test Exit after output-topic verification
  --no-fallback-camera-info          Do not publish fallback CameraInfo
  --no-fallback-tf                   Do not publish fallback static TF
  --no-fallback-occupancy            Do not run fallback PointCloud2->OccupancyGrid node
  --help, -h                         Show this help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --cameras)
            CAMERA_COUNT="${2:-}"
            shift 2
            ;;
        --cameras=*)
            CAMERA_COUNT="${1#--cameras=}"
            shift
            ;;
        --no-fusion)
            SKIP_FUSION=true
            shift
            ;;
        --viz)
            ENABLE_VIZ=true
            shift
            ;;
        --lidar)
            LIDAR_TOPIC="${2:-}"
            shift 2
            ;;
        --lidar=*)
            LIDAR_TOPIC="${1#--lidar=}"
            shift
            ;;
        --scan)
            SCAN_TOPIC="${2:-}"
            shift 2
            ;;
        --scan=*)
            SCAN_TOPIC="${1#--scan=}"
            shift
            ;;
        --camera-topic)
            PRIMARY_CAMERA_TOPIC="${2:-}"
            shift 2
            ;;
        --camera-topic=*)
            PRIMARY_CAMERA_TOPIC="${1#--camera-topic=}"
            shift
            ;;
        --base-frame)
            BASE_FRAME="${2:-}"
            shift 2
            ;;
        --base-frame=*)
            BASE_FRAME="${1#--base-frame=}"
            shift
            ;;
        --camera-info-yaml)
            CAMERA_INFO_YAML="${2:-}"
            shift 2
            ;;
        --camera-info-yaml=*)
            CAMERA_INFO_YAML="${1#--camera-info-yaml=}"
            shift
            ;;
        --dummy-camera)
            FORCE_DUMMY_CAMERA=true
            shift
            ;;
        --dummy-lidar)
            FORCE_DUMMY_LIDAR=true
            shift
            ;;
        --dummy-camera-width)
            DUMMY_CAMERA_WIDTH="${2:-}"
            shift 2
            ;;
        --dummy-camera-width=*)
            DUMMY_CAMERA_WIDTH="${1#--dummy-camera-width=}"
            shift
            ;;
        --dummy-camera-height)
            DUMMY_CAMERA_HEIGHT="${2:-}"
            shift 2
            ;;
        --dummy-camera-height=*)
            DUMMY_CAMERA_HEIGHT="${1#--dummy-camera-height=}"
            shift
            ;;
        --dummy-camera-fps)
            DUMMY_CAMERA_FPS="${2:-}"
            shift 2
            ;;
        --dummy-camera-fps=*)
            DUMMY_CAMERA_FPS="${1#--dummy-camera-fps=}"
            shift
            ;;
        --exit-after-verify|--smoke-test)
            EXIT_AFTER_VERIFY=true
            shift
            ;;
        --no-fallback-camera-info)
            ENABLE_FALLBACK_CAMERA_INFO=false
            shift
            ;;
        --no-fallback-tf)
            ENABLE_FALLBACK_TF=false
            shift
            ;;
        --no-fallback-occupancy)
            ENABLE_FALLBACK_OCCUPANCY=false
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

if [[ -z "$CAMERA_COUNT" || ! "$CAMERA_COUNT" =~ ^[0-9]+$ ]]; then
    echo -e "${RED}Invalid --cameras value: '$CAMERA_COUNT'${NC}"
    exit 1
fi
if [[ ! "$DUMMY_CAMERA_WIDTH" =~ ^[0-9]+$ ]]; then
    echo -e "${RED}Invalid --dummy-camera-width value: '$DUMMY_CAMERA_WIDTH'${NC}"
    exit 1
fi
if [[ ! "$DUMMY_CAMERA_HEIGHT" =~ ^[0-9]+$ ]]; then
    echo -e "${RED}Invalid --dummy-camera-height value: '$DUMMY_CAMERA_HEIGHT'${NC}"
    exit 1
fi
if [[ ! "$DUMMY_CAMERA_FPS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo -e "${RED}Invalid --dummy-camera-fps value: '$DUMMY_CAMERA_FPS'${NC}"
    exit 1
fi

echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  SDC 2026 — NUC Perception + Sensor Fusion Pipeline${NC}"
echo -e "${CYAN}  Cameras: $CAMERA_COUNT  |  Fusion: $([ "$SKIP_FUSION" = true ] && echo 'OFF' || echo 'ON')${NC}"
echo -e "${CYAN}  Base frame: $BASE_FRAME${NC}"
echo -e "${CYAN}  Fallbacks: camera_info=$ENABLE_FALLBACK_CAMERA_INFO tf=$ENABLE_FALLBACK_TF occupancy=$ENABLE_FALLBACK_OCCUPANCY${NC}"
echo -e "${CYAN}  Dummy modes: camera=$FORCE_DUMMY_CAMERA lidar=$FORCE_DUMMY_LIDAR${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"

source_env_safe() {
    # ROS/Autoware setup scripts may assume nounset is disabled.
    local flags="$-"
    set +e
    set +u
    source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
    source /autoware/install/setup.bash >/dev/null 2>&1 || true
    case "$flags" in
        *u*) set -u ;;
    esac
    case "$flags" in
        *e*) set -e ;;
    esac
}

# ── Source ROS 2 + Autoware (must be before detection_ws so vision_msgs etc. are available) ──
source_env_safe

# ── Install vision_msgs if not available (required by YOLO, ByteTrack, Bridge nodes) ──
if ! python3 -c "import vision_msgs" 2>/dev/null; then
    echo -e "${YELLOW}[!] vision_msgs not found — installing ros-humble-vision-msgs...${NC}"
    apt-get update -qq && apt-get install -y -qq ros-humble-vision-msgs > /dev/null 2>&1
    # Re-source so the newly installed package is on PYTHONPATH
    source_env_safe
    echo -e "${GREEN}[✓] vision_msgs installed${NC}"
else
    echo -e "${GREEN}[✓] vision_msgs already available${NC}"
fi

# ── Install onnxruntime if not available (required by YOLO inference) ──
if ! python3 -c "import onnxruntime" 2>/dev/null; then
    echo -e "${YELLOW}[!] onnxruntime not found — installing via pip3...${NC}"
    # Ensure pip is installed first
    apt-get update -qq && apt-get install -y -qq python3-pip > /dev/null 2>&1
    # Pin numpy to <2.0.0 to prevent breaking ROS2/OpenCV numpy 1.x compatibility
    pip3 install -q "numpy<2.0.0" onnxruntime
    echo -e "${GREEN}[✓] onnxruntime installed (OpenCV fallback will be avoided)${NC}"
else
    echo -e "${GREEN}[✓] onnxruntime already available${NC}"
fi

# ── Force downgrade numpy if it accidentally got upgraded to 2.x ──
if python3 -c "import numpy; import sys; sys.exit(0 if numpy.__version__.startswith('2.') else 1)" 2>/dev/null; then
    echo -e "${YELLOW}[!] NumPy 2.x detected! Downgrading to NumPy 1.x to fix OpenCV compatibility...${NC}"
    pip3 install -q "numpy<2.0.0"
    echo -e "${GREEN}[✓] NumPy downgraded to 1.x${NC}"
fi

# Explicitly build and export PYTHONPATH so ALL child processes (ros2 launch nodes) inherit it
_build_pythonpath() {
    local py_ver
    py_ver="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    local paths=(
        "/opt/ros/humble/lib/python${py_ver}/site-packages"
        "/opt/ros/humble/local/lib/python${py_ver}/dist-packages"
        "/autoware/install/vision_msgs/lib/python${py_ver}/site-packages"
        "/autoware/install/vision_msgs/local/lib/python${py_ver}/dist-packages"
        "/opt/autoware/lib/python${py_ver}/site-packages"
        "/opt/autoware/local/lib/python${py_ver}/dist-packages"
    )
    local extra=""
    for p in "${paths[@]}"; do
        [ -d "$p" ] && extra="${extra}:${p}"
    done
    export PYTHONPATH="${PYTHONPATH:-}${extra}"
}
_build_pythonpath
echo -e "${GREEN}[✓] PYTHONPATH exported (vision_msgs accessible to child processes)${NC}"

DETECTION_WS_DIR="$MODULE_DIR/detection_ws"
DETECTION_WS_INSTALL="$DETECTION_WS_DIR/install"
DETECTION_WS_RUNTIME_BUILD="${DETECTION_WS_RUNTIME_BUILD:-/tmp/alaz_detection_ws_build}"
DETECTION_WS_RUNTIME_INSTALL="${DETECTION_WS_RUNTIME_INSTALL:-/tmp/alaz_detection_ws_install}"
DETECTION_WS_RUNTIME_LOG="${DETECTION_WS_RUNTIME_LOG:-/tmp/alaz_detection_ws_log}"
DETECTION_REQUIRED_PACKAGES=(
    "tier4_perception_launch"
    "autoware_tensorrt_yolox"
    "autoware_bytetrack"
    "autoware_detection_autoware_bridge"
    "autoware_traffic_light_classifier"
)

source_detection_ws_safe() {
    local setup_file="$DETECTION_WS_INSTALL/setup.bash"
    [ -f "$setup_file" ] || return 1

    local flags="$-"
    set +e
    set +u
    source "$setup_file" >/dev/null 2>&1
    local rc=$?
    case "$flags" in
        *u*) set -u ;;
    esac
    case "$flags" in
        *e*) set -e ;;
    esac
    return "$rc"
}

detection_ws_ready() {
    local missing=()
    local pkg
    for pkg in "${DETECTION_REQUIRED_PACKAGES[@]}"; do
        if ! pkg_available "$pkg"; then
            missing+=("$pkg")
        fi
    done

    if [ "${#missing[@]}" -eq 0 ]; then
        return 0
    fi

    echo -e "${YELLOW}[!] detection_ws missing package(s): ${missing[*]}${NC}"
    return 1
}

# Check whether detection_ws is usable, not just whether install/ exists.
DETECTION_WS_NEEDS_BUILD=true
if source_detection_ws_safe; then
    if detection_ws_ready; then
        DETECTION_WS_NEEDS_BUILD=false
        echo -e "${GREEN}[✓] detection_ws sourced and package index verified${NC}"
    else
        echo -e "${YELLOW}[!] detection_ws install exists but is incomplete/stale — rebuilding...${NC}"
    fi
else
    echo -e "${YELLOW}[!] detection_ws not built yet — building...${NC}"
fi

if [ "$DETECTION_WS_NEEDS_BUILD" = true ]; then
    source_env_safe

    if ! colcon \
        --log-base "$DETECTION_WS_RUNTIME_LOG" \
        build \
        --base-paths "$DETECTION_WS_DIR/src" \
        --build-base "$DETECTION_WS_RUNTIME_BUILD" \
        --install-base "$DETECTION_WS_RUNTIME_INSTALL"; then
        echo -e "${RED}[✗] detection_ws build failed${NC}"
        exit 1
    fi

    DETECTION_WS_INSTALL="$DETECTION_WS_RUNTIME_INSTALL"
    source_env_safe
    if ! source_detection_ws_safe || ! detection_ws_ready; then
        echo -e "${RED}[✗] detection_ws build completed but required ROS packages are still unavailable${NC}"
        exit 1
    fi

    echo -e "${GREEN}[✓] detection_ws built, sourced, and package index verified${NC}"
fi

# Optional: start dummy camera(s) before sensor detection so topics are visible.
if [ "$FORCE_DUMMY_CAMERA" = true ]; then
    echo -e "${YELLOW}[dummy] Starting $CAMERA_COUNT dummy camera publisher(s)...${NC}"
    for i in $(seq 0 $((CAMERA_COUNT - 1))); do
        if [ "$i" -eq 0 ]; then
            cam_topic="$PRIMARY_CAMERA_TOPIC"
            cam_frame="camera_center_link"
        else
            cam_topic="/sensing/camera/camera${i}/image_raw"
            cam_frame="camera${i}_link"
        fi
        phase="$(awk -v idx="$i" 'BEGIN { printf "%.3f", idx * 0.7 }')"
        python3 "$SCRIPT_DIR/../test_scripts/dummy_camera_publisher.py" \
            --node-name "dummy_camera_publisher_${i}" \
            --topic "$cam_topic" \
            --frame-id "$cam_frame" \
            --width "$DUMMY_CAMERA_WIDTH" \
            --height "$DUMMY_CAMERA_HEIGHT" \
            --fps "$DUMMY_CAMERA_FPS" \
            --phase "$phase" &
        CLEANUP_PIDS+=($!)
        sleep 0.4
        echo -e "${GREEN}  ✓ Dummy camera $i -> $cam_topic (${DUMMY_CAMERA_WIDTH}x${DUMMY_CAMERA_HEIGHT}@${DUMMY_CAMERA_FPS})${NC}"
    done
    echo -e "${YELLOW}  Waiting for dummy camera topics to appear in ROS graph...${NC}"
    for i in $(seq 0 $((CAMERA_COUNT - 1))); do
        if [ "$i" -eq 0 ]; then
            cam_topic="$PRIMARY_CAMERA_TOPIC"
        else
            cam_topic="/sensing/camera/camera${i}/image_raw"
        fi
        if wait_for_topic "$cam_topic" 8; then
            echo -e "${GREEN}  ✓ Dummy camera topic visible: $cam_topic${NC}"
        else
            echo -e "${YELLOW}  ⚠ Dummy camera topic not visible yet: $cam_topic${NC}"
        fi
    done
fi

# ══════════════════════════════════════
# STEP 1: Detect sensors
# ══════════════════════════════════════
echo -e "\n${BLUE}[1/6] Detecting sensors...${NC}"

HAS_CAMERA=false
HAS_LIDAR=false
HAS_REAL_LIDAR=false
LIDAR_MODE="none"   # none | scan | pointcloud
LIDAR_INPUT_TOPIC=""

TOPICS="$(timeout 3 ros2 topic list 2>/dev/null || echo "")"

# Check for LiDAR (prefer explicit pointcloud, then scan)
if [ "$FORCE_DUMMY_LIDAR" = true ]; then
    echo -e "${YELLOW}  ⚠ --dummy-lidar set: ignoring real LiDAR topics and forcing dummy $SCAN_TOPIC${NC}"
elif echo "$TOPICS" | grep -Fxq "$LIDAR_TOPIC"; then
    HAS_LIDAR=true
    HAS_REAL_LIDAR=true
    LIDAR_MODE="pointcloud"
    LIDAR_INPUT_TOPIC="$LIDAR_TOPIC"
    echo -e "${GREEN}  ✓ LiDAR PointCloud2: $LIDAR_TOPIC${NC}"
elif echo "$TOPICS" | grep -Fxq "$SCAN_TOPIC"; then
    HAS_LIDAR=true
    HAS_REAL_LIDAR=true
    LIDAR_MODE="scan"
    LIDAR_INPUT_TOPIC="$SCAN_TOPIC"
    echo -e "${GREEN}  ✓ LiDAR LaserScan: $SCAN_TOPIC${NC}"
else
    AUTO_LIDAR_TOPIC="$(echo "$TOPICS" | grep "/sensing/lidar" | head -1 || true)"
    if [ -n "$AUTO_LIDAR_TOPIC" ]; then
        HAS_LIDAR=true
        HAS_REAL_LIDAR=true
        if echo "$AUTO_LIDAR_TOPIC" | grep -q "scan"; then
            LIDAR_MODE="scan"
            SCAN_TOPIC="$AUTO_LIDAR_TOPIC"
            LIDAR_INPUT_TOPIC="$AUTO_LIDAR_TOPIC"
            echo -e "${GREEN}  ✓ LiDAR scan detected: $AUTO_LIDAR_TOPIC${NC}"
        else
            LIDAR_MODE="pointcloud"
            LIDAR_TOPIC="$AUTO_LIDAR_TOPIC"
            LIDAR_INPUT_TOPIC="$AUTO_LIDAR_TOPIC"
            echo -e "${GREEN}  ✓ LiDAR pointcloud detected: $AUTO_LIDAR_TOPIC${NC}"
        fi
    else
        echo -e "${YELLOW}  ⚠ No LiDAR topic found (will use dummy $SCAN_TOPIC for costmap)${NC}"
    fi
fi

declare -a CAMERA_TOPICS=()
declare -a CAMERA_INFO_TOPICS=()
declare -a CAMERA_INFO_FOUND=()

# Check cameras
for i in $(seq 0 $((CAMERA_COUNT - 1))); do
    if [ "$i" -eq 0 ]; then
        cam_topic="$PRIMARY_CAMERA_TOPIC"
    else
        cam_topic="/sensing/camera/camera${i}/image_raw"
    fi
    if ! echo "$TOPICS" | grep -Fxq "$cam_topic"; then
        alt="$(echo "$TOPICS" | grep -E "^/sensing/image_raw$|/sensing/camera/camera${i}/image_raw$|/camera${i}/image_raw$|^/camera/image_raw$" | head -1 || true)"
        if [ -n "$alt" ]; then
            cam_topic="$alt"
        fi
    fi

    if echo "$TOPICS" | grep -Fxq "$cam_topic"; then
        HAS_CAMERA=true
        CAMERA_TOPICS[$i]="$cam_topic"
        cam_info_topic="${cam_topic%/image_raw}/camera_info"
        CAMERA_INFO_TOPICS[$i]="$cam_info_topic"
        if echo "$TOPICS" | grep -Fxq "$cam_info_topic"; then
            CAMERA_INFO_FOUND[$i]="true"
            echo -e "${GREEN}  ✓ Camera $i: $cam_topic${NC}"
            echo -e "${GREEN}    └─ camera_info: $cam_info_topic${NC}"
        else
            CAMERA_INFO_FOUND[$i]="false"
            echo -e "${GREEN}  ✓ Camera $i: $cam_topic${NC}"
            echo -e "${YELLOW}    └─ camera_info missing: $cam_info_topic${NC}"
        fi
    else
        CAMERA_TOPICS[$i]=""
        CAMERA_INFO_TOPICS[$i]=""
        CAMERA_INFO_FOUND[$i]="false"
        echo -e "${YELLOW}  ⚠ Camera $i not found ($cam_topic)${NC}"
    fi
done

if [ "$HAS_CAMERA" = false ]; then
    echo -e "${YELLOW}  ⚠ No camera topics detected — detection launch may start but produce no output${NC}"
fi

# ══════════════════════════════════════
# STEP 2: Calibration/TF fallbacks (only if missing)
# ══════════════════════════════════════
echo -e "\n${BLUE}[2/6] Starting fallback calibration support (if needed)...${NC}"

if [ "$ENABLE_FALLBACK_TF" = true ] && [ "$LIDAR_MODE" = "scan" ]; then
    python3 "$SCRIPT_DIR/scan_static_tf_fallback.py" \
        --scan-topic "$SCAN_TOPIC" \
        --base-frame "$BASE_FRAME" \
        --x "$LIDAR_X" --y "$LIDAR_Y" --z "$LIDAR_Z" \
        --roll "$LIDAR_ROLL" --pitch "$LIDAR_PITCH" --yaw "$LIDAR_YAW" &
    CLEANUP_PIDS+=($!)
    sleep 0.5
    echo -e "${GREEN}  ✓ Scan TF fallback watcher started (publishes only if needed)${NC}"
fi

if [ "$ENABLE_FALLBACK_CAMERA_INFO" = true ]; then
    for i in $(seq 0 $((CAMERA_COUNT - 1))); do
        cam_topic="${CAMERA_TOPICS[$i]:-}"
        cam_info_topic="${CAMERA_INFO_TOPICS[$i]:-}"
        cam_info_exists="${CAMERA_INFO_FOUND[$i]:-false}"
        if [ -z "$cam_topic" ]; then
            continue
        fi
        if [ "$cam_info_exists" = "true" ]; then
            continue
        fi

        cmd=(
            python3 "$SCRIPT_DIR/camera_info_fallback_publisher.py"
            --image-topic "$cam_topic"
            --camera-info-topic "$cam_info_topic"
            --camera-name "camera${i}"
            --frame-id "camera${i}_link"
            --base-frame "$BASE_FRAME"
            --x "${CAM_X[$i]:-0.98}"
            --y "${CAM_Y[$i]:-0.0}"
            --z "${CAM_Z[$i]:-0.39}"
            --roll "${CAM_ROLL[$i]:-0.0}"
            --pitch "${CAM_PITCH[$i]:-0.0}"
            --yaw "${CAM_YAW[$i]:-0.0}"
        )

        if [ -f "$CAMERA_INFO_YAML" ]; then
            cmd+=(--camera-info-yaml "$CAMERA_INFO_YAML")
        else
            echo -e "${YELLOW}  ⚠ Fallback camera YAML not found: $CAMERA_INFO_YAML (using built-in defaults)${NC}"
        fi

        if [ "$ENABLE_FALLBACK_TF" = true ]; then
            cmd+=(--publish-static-tf)
        fi

        "${cmd[@]}" &
        CLEANUP_PIDS+=($!)
        sleep 0.5
        echo -e "${GREEN}  ✓ Camera $i fallback camera_info started: $cam_info_topic${NC}"
    done
else
    echo -e "${YELLOW}  Fallback CameraInfo disabled${NC}"
fi

if [ "$ENABLE_FALLBACK_TF" = true ] && [ "$ENABLE_FALLBACK_CAMERA_INFO" = false ]; then
    echo -e "${YELLOW}  ⚠ Camera TF fallback piggybacks on camera_info fallback; camera TFs assume sensor_kit is already publishing TF${NC}"
fi

# ══════════════════════════════════════
# STEP 3: LiDAR -> PointCloud2 + OccupancyGrid (parallel)
# ══════════════════════════════════════
echo -e "\n${BLUE}[3/6] LiDAR pipeline (PointCloud2 + OccupancyGrid)...${NC}"

POINTCLOUD_FOR_FUSION_TOPIC="$LIDAR_POINTCLOUD_TOPIC"
POINTCLOUD_FOR_COSTMAP_TOPIC="$LIDAR_POINTCLOUD_TOPIC"

if [ "$HAS_REAL_LIDAR" = false ]; then
    echo -e "${YELLOW}  No real LiDAR — starting dummy LaserScan for costmap/fallback testing...${NC}"
    python3 "$SCRIPT_DIR/../test_scripts/dummy_lidar_publisher.py" --rate 10 --obstacle --topic "$SCAN_TOPIC" --frame-id "lidar_link" &
    CLEANUP_PIDS+=($!)
    sleep 1
    LIDAR_MODE="scan"
    LIDAR_INPUT_TOPIC="$SCAN_TOPIC"
    HAS_LIDAR=true
    echo -e "${GREEN}  ✓ Dummy LiDAR running on $SCAN_TOPIC (10 Hz)${NC}"
    if [ "$ENABLE_FALLBACK_TF" = true ]; then
        python3 "$SCRIPT_DIR/scan_static_tf_fallback.py" \
            --scan-topic "$SCAN_TOPIC" \
            --base-frame "$BASE_FRAME" \
            --x "$LIDAR_X" --y "$LIDAR_Y" --z "$LIDAR_Z" \
            --roll "$LIDAR_ROLL" --pitch "$LIDAR_PITCH" --yaw "$LIDAR_YAW" &
        CLEANUP_PIDS+=($!)
        sleep 0.5
        echo -e "${GREEN}  ✓ Dummy LiDAR scan TF fallback watcher started${NC}"
    fi
fi

if [ "$LIDAR_MODE" = "scan" ]; then
    echo -e "${YELLOW}  Starting LaserScan -> PointCloud2 converter via launch file...${NC}"
    ros2 launch "$MODULE_DIR/launch/laserscan_to_pcl_and_occ.launch.xml" \
        scan_topic:="$SCAN_TOPIC" \
        points_topic:="$LIDAR_POINTCLOUD_TOPIC" \
        frame_id:="$BASE_FRAME" &
    CLEANUP_PIDS+=($!)
    sleep 1
    echo -e "${GREEN}  ✓ LaserScan -> PointCloud2 -> $LIDAR_POINTCLOUD_TOPIC${NC}"
    OCC_FRAME_ID="$BASE_FRAME"
elif [ "$LIDAR_MODE" = "pointcloud" ]; then
    POINTCLOUD_FOR_FUSION_TOPIC="$LIDAR_INPUT_TOPIC"
    POINTCLOUD_FOR_COSTMAP_TOPIC="$LIDAR_INPUT_TOPIC"
    OCC_FRAME_ID=""
    echo -e "${GREEN}  ✓ Using existing LiDAR pointcloud directly: $LIDAR_INPUT_TOPIC${NC}"
else
    OCC_FRAME_ID="$BASE_FRAME"
    echo -e "${YELLOW}  ⚠ No LiDAR mode selected (unexpected). Continuing without LiDAR paths.${NC}"
fi

if [ "$ENABLE_FALLBACK_OCCUPANCY" = true ]; then
    echo -e "${YELLOW}  Starting PointCloud2 -> OccupancyGrid fallback...${NC}"
    occ_cmd=(
        python3 "$SCRIPT_DIR/pointcloud_to_occupancy_grid.py"
        --input-topic "$POINTCLOUD_FOR_COSTMAP_TOPIC"
        --occupancy-topic "$OCCUPANCY_TOPIC"
        --pointcloud-output-topic "$OBSTACLE_POINTCLOUD_TOPIC"
    )
    if [ -n "$OCC_FRAME_ID" ]; then
        occ_cmd+=(--frame-id "$OCC_FRAME_ID")
    else
        occ_cmd+=(--frame-id "")
    fi
    "${occ_cmd[@]}" &
    CLEANUP_PIDS+=($!)
    sleep 1
    echo -e "${GREEN}  ✓ OccupancyGrid fallback -> $OCCUPANCY_TOPIC${NC}"
    echo -e "${GREEN}    PointCloud relay -> $OBSTACLE_POINTCLOUD_TOPIC${NC}"
else
    echo -e "${YELLOW}  OccupancyGrid fallback disabled${NC}"
fi

if [ "$SKIP_FUSION" = false ]; then
    echo -e "${YELLOW}  Starting Euclidean clustering for ROI fusion...${NC}"
    if pkg_available "autoware_euclidean_cluster"; then
        ros2 launch autoware_euclidean_cluster voxel_grid_based_euclidean_cluster.launch.xml \
            input_pointcloud:="$POINTCLOUD_FOR_FUSION_TOPIC" \
            output_clusters:="$LIDAR_CLUSTER_TOPIC" &
        CLEANUP_PIDS+=($!)
        sleep 2
        echo -e "${GREEN}  ✓ Euclidean clustering -> $LIDAR_CLUSTER_TOPIC${NC}"
    else
        echo -e "${YELLOW}  ⚠ autoware_euclidean_cluster not installed — ROI fusion will be skipped${NC}"
    fi
fi

# ══════════════════════════════════════
# STEP 4: Detection pipeline (YOLOv8 -> ByteTrack -> Bridge + Traffic Light)
# ══════════════════════════════════════
echo -e "\n${BLUE}[4/6] Detection pipeline (YOLOv8 -> ByteTrack -> Bridge)...${NC}"

MODEL_DIR="$MODULE_DIR/models"
if ! python3 -c "import onnxruntime" >/dev/null 2>&1; then
    echo -e "${YELLOW}  ⚠ onnxruntime Python package not found. YOLOv8 will fall back to OpenCV DNN and may fail for this model.${NC}"
fi

DETECTION_LAUNCH_SOURCE="$MODULE_DIR/detection_ws/src/tier4_perception_launch/launch/detection_module.launch.xml"
DETECTION_ARGS=(
    image_number:="$CAMERA_COUNT"
    camera_2d_detector/model_path:="$MODEL_DIR/yolov8n.onnx"
    camera_2d_detector/label_path:="$MODEL_DIR/labels.txt"
    camera_2d_detector/color_map_path:="$MODEL_DIR/color_map.json"
    enable_visualizer:="$ENABLE_VIZ"
)
for i in $(seq 0 $((CAMERA_COUNT - 1))); do
    cam_topic="${CAMERA_TOPICS[$i]:-}"
    if [ -n "$cam_topic" ]; then
        DETECTION_ARGS+=("image_raw${i}:=$cam_topic")
    fi
done
if [ -f "$DETECTION_LAUNCH_SOURCE" ]; then
    echo -e "${GREEN}  ✓ Using local detection launch file: $DETECTION_LAUNCH_SOURCE${NC}"
    ros2 launch "$DETECTION_LAUNCH_SOURCE" "${DETECTION_ARGS[@]}" &
else
    echo -e "${YELLOW}  ⚠ Local detection launch file not found; falling back to package share${NC}"
    ros2 launch tier4_perception_launch detection_module.launch.xml "${DETECTION_ARGS[@]}" &
fi
CLEANUP_PIDS+=($!)
sleep 3

echo -e "${GREEN}  ✓ Detection pipeline launched:${NC}"
echo -e "${GREEN}    Camera(s) -> YOLOv8 -> ByteTrack -> tracked rois${NC}"
echo -e "${GREEN}    Camera -> TL ROI detector -> TL classifier${NC}"
echo -e "${GREEN}    Tracked rois -> Autoware bridge -> planner topics${NC}"

if wait_for_topic "/rois0" 8; then
    ROIS0_TYPE="$(topic_type "/rois0" || true)"
    echo -e "${GREEN}  ✓ /rois0 active${NC} (${ROIS0_TYPE:-unknown type})"
else
    echo -e "${YELLOW}  ⚠ /rois0 not active yet — fusion/bridge may need camera frames${NC}"
fi

if [ "$SKIP_FUSION" = false ]; then
    echo -e "${YELLOW}  Starting ROI type adapter (Detection2DArray -> tier4_perception_msgs)...${NC}"
    if python3 -c "import vision_msgs.msg" >/dev/null 2>&1; then
        python3 "$SCRIPT_DIR/detection2d_to_tier4_rois.py" \
            --ros-args \
            -p input_topic:=/rois0 \
            -p output_topic:="$FUSION_ROIS_TOPIC" &
        CLEANUP_PIDS+=($!)
        ROI_ADAPTER_PID=$!
        sleep 1
        if is_pid_alive "$ROI_ADAPTER_PID"; then
            echo -e "${GREEN}  ✓ ROI adapter process started (pid=$ROI_ADAPTER_PID)${NC}"
        else
            echo -e "${YELLOW}  ⚠ ROI adapter exited early — fusion likely unavailable${NC}"
        fi
    else
        echo -e "${YELLOW}  ⚠ Python module 'vision_msgs' not available; skipping ROI adapter${NC}"
    fi

    if wait_for_topic "$FUSION_ROIS_TOPIC" 8; then
        FUSION_ROIS_TYPE="$(topic_type "$FUSION_ROIS_TOPIC" || true)"
        echo -e "${GREEN}  ✓ ROI adapter output active: $FUSION_ROIS_TOPIC${NC}"
        echo -e "${GREEN}    Type: ${FUSION_ROIS_TYPE:-unknown}${NC}"
    else
        echo -e "${YELLOW}  ⚠ ROI adapter output topic not active yet: $FUSION_ROIS_TOPIC${NC}"
    fi
fi

# ══════════════════════════════════════
# STEP 5: ROI Cluster Fusion (LiDAR + Camera)
# ══════════════════════════════════════
echo -e "\n${BLUE}[5/6] ROI Cluster Fusion...${NC}"

CAM0_IMAGE_TOPIC="${CAMERA_TOPICS[0]:-$PRIMARY_CAMERA_TOPIC}"
CAM0_INFO_TOPIC="${CAMERA_INFO_TOPICS[0]:-/sensing/camera_info}"

if [ "$HAS_LIDAR" = true ] && [ "$SKIP_FUSION" = false ]; then
    if pkg_available "autoware_image_projection_based_fusion"; then
        FUSION_READY=true

        if ! wait_for_topic "$LIDAR_CLUSTER_TOPIC" 10; then
            echo -e "${YELLOW}  ⚠ LiDAR cluster topic not active: $LIDAR_CLUSTER_TOPIC${NC}"
            FUSION_READY=false
        fi

        if ! wait_for_topic "$FUSION_ROIS_TOPIC" 10; then
            echo -e "${YELLOW}  ⚠ ROI adapter topic not active: $FUSION_ROIS_TOPIC${NC}"
            FUSION_READY=false
        fi

        if ! wait_for_topic "$CAM0_INFO_TOPIC" 10; then
            echo -e "${YELLOW}  ⚠ camera_info topic not active: $CAM0_INFO_TOPIC${NC}"
            FUSION_READY=false
        fi

        if ! wait_for_topic "$CAM0_IMAGE_TOPIC" 10; then
            echo -e "${YELLOW}  ⚠ image topic not active: $CAM0_IMAGE_TOPIC${NC}"
            FUSION_READY=false
        fi

        CLUSTER_TYPE="$(topic_type "$LIDAR_CLUSTER_TOPIC" || true)"
        ROIS_TYPE="$(topic_type "$FUSION_ROIS_TOPIC" || true)"
        CAMINFO_TYPE="$(topic_type "$CAM0_INFO_TOPIC" || true)"
        echo -e "${GREEN}  Fusion input types:${NC}"
        echo -e "${GREEN}    $LIDAR_CLUSTER_TOPIC -> ${CLUSTER_TYPE:-unknown}${NC}"
        echo -e "${GREEN}    $FUSION_ROIS_TOPIC -> ${ROIS_TYPE:-unknown}${NC}"
        echo -e "${GREEN}    $CAM0_INFO_TOPIC -> ${CAMINFO_TYPE:-unknown}${NC}"

        if [ -n "$CLUSTER_TYPE" ] && ! echo "$CLUSTER_TYPE" | grep -q "DetectedObjectsWithFeature"; then
            echo -e "${YELLOW}  ⚠ Cluster topic type does not look like DetectedObjectsWithFeature${NC}"
            FUSION_READY=false
        fi
        if [ -n "$ROIS_TYPE" ] && ! echo "$ROIS_TYPE" | grep -q "DetectedObjectsWithFeature"; then
            echo -e "${YELLOW}  ⚠ ROI adapter topic type mismatch (expected *DetectedObjectsWithFeature)${NC}"
            FUSION_READY=false
        fi
        if [ -n "$CAMINFO_TYPE" ] && [ "$CAMINFO_TYPE" != "sensor_msgs/msg/CameraInfo" ]; then
            echo -e "${YELLOW}  ⚠ camera_info topic type mismatch: $CAMINFO_TYPE${NC}"
            FUSION_READY=false
        fi

        if [ "$FUSION_READY" = false ]; then
            echo -e "${YELLOW}  ⚠ Fusion preflight failed — skipping roi_cluster_fusion launch${NC}"
            echo -e "${YELLOW}    Planner will continue with bridge outputs + occupancy fallback${NC}"
        else
        echo -e "${GREEN}  Launching roi_cluster_fusion...${NC}"

        ROI_FUSION_PARAMS_ARGS=()
        if ROI_FUSION_PARAMS_PATH="$(find_roi_cluster_fusion_params_file)"; then
            echo -e "${GREEN}  ✓ Using roi_cluster_fusion params file: $ROI_FUSION_PARAMS_PATH${NC}"
            ROI_FUSION_PARAMS_ARGS+=(--params-file "$ROI_FUSION_PARAMS_PATH")
        elif [ -n "$ROI_FUSION_PARAMS_FILE" ]; then
            echo -e "${YELLOW}  ⚠ ROI fusion params file not found: $ROI_FUSION_PARAMS_FILE${NC}"
            echo -e "${YELLOW}    Falling back to minimal inline params (may miss version-specific required params).${NC}"
        else
            echo -e "${YELLOW}  ⚠ ROI fusion params YAML auto-discovery failed${NC}"
            echo -e "${YELLOW}    Falling back to minimal inline params (may miss version-specific required params).${NC}"
        fi

        ROI_FUSION_CMD=(
            ros2 run autoware_image_projection_based_fusion roi_cluster_fusion_node
            --ros-args
        )
        if [ "${#ROI_FUSION_PARAMS_ARGS[@]}" -gt 0 ]; then
            ROI_FUSION_CMD+=("${ROI_FUSION_PARAMS_ARGS[@]}")
        fi
        ROI_FUSION_CMD+=(
            -r "input:=$LIDAR_CLUSTER_TOPIC"
            -r "input/rois0:=$FUSION_ROIS_TOPIC"
            -r "input/camera_info0:=$CAM0_INFO_TOPIC"
            -r "input/image_raw0:=$CAM0_IMAGE_TOPIC"
            -r "output:=$FUSED_OBJECTS_TOPIC"
            -p rois_number:=1
            -p "msg3d_timeout_sec:=$ROI_FUSION_MSG3D_TIMEOUT_SEC"
            -p "rois_timeout_sec:=$ROI_FUSION_ROIS_TIMEOUT_SEC"
            -p "rois_timestamp_offsets:=$ROI_FUSION_ROIS_TIMESTAMP_OFFSETS"
            -p "point_project_to_unrectified_image:=$ROI_FUSION_POINT_PROJECT_TO_UNRECTIFIED_IMAGE"
            -p "approximate_camera_projection:=$ROI_FUSION_APPROXIMATE_CAMERA_PROJECTION"
            -p "approximation_grid_cell_width:=$ROI_FUSION_APPROX_GRID_CELL_WIDTH"
            -p "approximation_grid_cell_height:=$ROI_FUSION_APPROX_GRID_CELL_HEIGHT"
            -p "image_buffer_size:=$ROI_FUSION_IMAGE_BUFFER_SIZE"
            -p "filter_scope_min_x:=$ROI_FUSION_FILTER_SCOPE_MIN_X"
            -p "filter_scope_min_y:=$ROI_FUSION_FILTER_SCOPE_MIN_Y"
            -p "filter_scope_min_z:=$ROI_FUSION_FILTER_SCOPE_MIN_Z"
            -p "filter_scope_max_x:=$ROI_FUSION_FILTER_SCOPE_MAX_X"
            -p "filter_scope_max_y:=$ROI_FUSION_FILTER_SCOPE_MAX_Y"
            -p "filter_scope_max_z:=$ROI_FUSION_FILTER_SCOPE_MAX_Z"
            -p "debug_mode:=$ROI_FUSION_DEBUG_MODE"
            -p "collector_debug_mode:=$ROI_FUSION_COLLECTOR_DEBUG_MODE"
            -p "publish_processing_time_detail:=$ROI_FUSION_PUBLISH_PROCESSING_TIME_DETAIL"
            -p "publish_previous_but_late_output_msg:=$ROI_FUSION_PUBLISH_PREVIOUS_BUT_LATE_OUTPUT_MSG"
            -p "rosbag_length:=$ROI_FUSION_ROSBAG_LENGTH"
            -p "matching_strategy.type:=$ROI_FUSION_MATCHING_STRATEGY_TYPE"
            -p "matching_strategy.threshold:=$ROI_FUSION_MATCHING_STRATEGY_THRESHOLD"
            -p "matching_strategy.msg3d_noise_window:=$ROI_FUSION_MATCHING_STRATEGY_MSG3D_NOISE_WINDOW"
            -p "matching_strategy.rois_timestamp_noise_window:=$ROI_FUSION_MATCHING_STRATEGY_ROIS_TIMESTAMP_NOISE_WINDOW"
            -p trust_distance:=40.0
            -p fusion_distance:=100.0
            -p iou_threshold.UNKNOWN:=0.1
            -p iou_threshold.CAR:=0.65
            -p iou_threshold.TRUCK:=0.65
            -p iou_threshold.BUS:=0.65
            -p iou_threshold.MOTORCYCLE:=0.65
            -p iou_threshold.BICYCLE:=0.65
            -p iou_threshold.PEDESTRIAN:=0.65
        )
        "${ROI_FUSION_CMD[@]}" &
        ROI_FUSION_PID=$!
        ROI_FUSION_STARTED=true
        CLEANUP_PIDS+=($ROI_FUSION_PID)
        sleep 2
        if kill -0 "$ROI_FUSION_PID" 2>/dev/null; then
            ROI_FUSION_RUNNING=true
            echo -e "${GREEN}  ✓ ROI Cluster Fusion running${NC}"
            echo -e "${GREEN}    Input: $LIDAR_CLUSTER_TOPIC + $FUSION_ROIS_TOPIC + $CAM0_INFO_TOPIC${NC}"
            echo -e "${GREEN}    Output: $FUSED_OBJECTS_TOPIC${NC}"
            echo -e "${YELLOW}    Note: planner is still fed by Autoware bridge outputs (2D bridge).${NC}"
        else
            wait "$ROI_FUSION_PID" 2>/dev/null || true
            echo -e "${YELLOW}  ⚠ roi_cluster_fusion exited early${NC}"
            echo -e "${YELLOW}    Check parameter compatibility for this Autoware image (e.g. rois_number, input indices).${NC}"
            echo -e "${YELLOW}    Planner will continue with bridge outputs + occupancy fallback${NC}"
        fi
        fi
    else
        echo -e "${YELLOW}  ⚠ autoware_image_projection_based_fusion not installed${NC}"
        echo -e "${YELLOW}    Using camera-only detection for planner topics${NC}"
    fi
else
    echo -e "${YELLOW}  Skipping ROI fusion (no LiDAR input or --no-fusion)${NC}"
    echo -e "${YELLOW}  Planner still receives bridge outputs + occupancy fallback${NC}"
fi

# ══════════════════════════════════════
# STEP 6: Verify output topics
# ══════════════════════════════════════
echo -e "\n${BLUE}[6/6] Verifying output topics...${NC}"
sleep 3

if [ "$ROI_FUSION_RUNNING" = true ] && [ -n "$ROI_FUSION_PID" ] && ! is_pid_alive "$ROI_FUSION_PID"; then
    ROI_FUSION_RUNNING=false
    echo -e "${YELLOW}  ⚠ ROI Cluster Fusion died after startup; fused_objects topic will not be required${NC}"
fi

EXPECTED_TOPICS=(
    "/perception/object_recognition/detection/objects"
    "/perception/object_recognition/tracking/objects"
    "/perception/object_recognition/objects"
    "/perception/traffic_light_recognition/traffic_signals"
)

if [ "$ENABLE_FALLBACK_OCCUPANCY" = true ]; then
    EXPECTED_TOPICS+=("$OCCUPANCY_TOPIC" "$OBSTACLE_POINTCLOUD_TOPIC")
fi

if [ "$SKIP_FUSION" = false ] && [ "$HAS_LIDAR" = true ]; then
    EXPECTED_TOPICS+=("$FUSION_ROIS_TOPIC" "$LIDAR_CLUSTER_TOPIC")
    if [ "$ROI_FUSION_RUNNING" = true ]; then
        EXPECTED_TOPICS+=("$FUSED_OBJECTS_TOPIC")
    fi
fi

FOUND=0
for topic in "${EXPECTED_TOPICS[@]}"; do
    verify_timeout=3
    if [ "$topic" = "$OCCUPANCY_TOPIC" ] || [ "$topic" = "$OBSTACLE_POINTCLOUD_TOPIC" ]; then
        verify_timeout=6
    fi
    if verify_topic_active "$topic" "$verify_timeout"; then
        echo -e "${GREEN}  ✓ $topic${NC}"
        FOUND=$((FOUND + 1))
    else
        echo -e "${YELLOW}  ⚠ $topic (not yet active — may need data or package)${NC}"
    fi
done

echo -e "\n${CYAN}══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  NUC Perception Pipeline Running${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "  ${BLUE}Pipeline Architecture:${NC}"
echo "    LiDAR(scan) -> PointCloud2 -> Clustering -> ROI Cluster Fusion (optional)"
echo "                   |"
echo "                   +-> OccupancyGrid fallback -> Planner costmap"
echo "    Camera -> YOLOv8 -> ByteTrack -> Bridge -> Autoware planner topics"
echo "    Camera -> YOLOv8 TL -> TL Classifier -> TrafficLightGroupArray"
echo ""
echo -e "  ${BLUE}Monitor:${NC}"
echo "    ros2 topic hz /perception/object_recognition/detection/objects"
echo "    ros2 topic hz $OBSTACLE_POINTCLOUD_TOPIC"
echo "    ros2 topic echo $OCCUPANCY_TOPIC --once"
echo "    ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once"
echo ""
echo -e "  ${YELLOW}Press Ctrl+C to stop all nodes.${NC}"
echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"

if [ "$EXIT_AFTER_VERIFY" = true ]; then
    echo -e "${GREEN}Smoke verification complete; exiting after cleanup.${NC}"
    exit 0
fi

while true; do
    wait -n 2>/dev/null || true
    sleep 0.5 &
    wait $!
done
