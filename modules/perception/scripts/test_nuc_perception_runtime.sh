#!/bin/bash
# test_nuc_perception_runtime.sh
#
# Runtime smoke/integration checks for `scripts/nuc_docker_perception.sh`.
# Intended to run INSIDE the Docker container (or any ROS 2 shell with Autoware sourced).
#
# What it checks:
# - required topics/types exist
# - messages flow (`ros2 topic echo --once`)
# - rough topic rates (`ros2 topic hz`)
# - camera_info sanity (width/height/distortion_model)
# - optional TF checks (base_link -> scan/camera frame)
# - optional ROI fusion topics (`clusters`, `fused_objects`)
#
# It can optionally start the pipeline itself and then validate it.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODULE_DIR="$(dirname "$SCRIPT_DIR")"
DETECTION_WS_SRC_DIR="$MODULE_DIR/detection_ws/src"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

START_PIPELINE=false
EXPECT_FUSION=true
EXPECT_TRAFFIC=true
CAMERA_COUNT=1
BASE_FRAME="base_link"
SCAN_TOPIC="/scan"
LIDAR_TOPIC="/points_raw"
STARTUP_WAIT=12
PIPELINE_READY_WAIT=45
RATE_SAMPLE_SEC=5
PIPELINE_ARGS=()
LOG_DIR="${TMPDIR:-/tmp}"
PIPELINE_LOG=""
PIPELINE_PID=""
USE_DUMMY_CAMERA=false
USE_DUMMY_LIDAR=false
STRICT=false
REQUIRE_FUSION_MESSAGES=true
DEEP_MODE=false
SOAK_SEC=0
SOAK_POLL_SEC=2
ECHO_REPEAT=1

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

TEMP_DIR=""

usage() {
  cat <<EOF
Usage: test_nuc_perception_runtime.sh [options] [-- <extra args for nuc_docker_perception.sh>]

Options:
  --start                 Start scripts/nuc_docker_perception.sh in background before tests
  --no-fusion             Do not require ROI fusion topics
  --no-traffic            Do not require traffic light outputs
  --cameras N             Expected camera count (default: 1)
  --dummy-camera          When used with --start, run nuc pipeline with --dummy-camera
  --dummy-lidar           When used with --start, run nuc pipeline with --dummy-lidar
  --dummy-sensors         Shortcut for --dummy-camera + --dummy-lidar
  --strict                Treat warnings as failure (recommended for pre-race checks)
  --deep                  Enable deeper stress checks (strict + repeated echo + soak)
  --base-frame FRAME      TF base frame to validate (default: base_link)
  --scan TOPIC            LaserScan topic to probe (default: /scan)
  --lidar TOPIC           Raw PointCloud2 topic to probe (default: /points_raw)
  --startup-wait SEC      Wait after starting pipeline (default: 12)
  --rate-sample SEC       Duration for ros2 topic hz (default: 5)
  --echo-repeat N         Repeat echo checks for key topics (default: 1)
  --soak-sec SEC          Poll stability checks for SEC seconds after startup (default: 0)
  --log-dir DIR           Where to store runtime logs (default: /tmp)
  --help, -h              Show help

Examples:
  # Check an already-running pipeline
  bash scripts/test_nuc_perception_runtime.sh

  # Start pipeline and test it (inside Docker)
  bash scripts/test_nuc_perception_runtime.sh --start --cameras 1

  # Start pipeline with dummy camera + dummy lidar (no real sensors)
  bash scripts/test_nuc_perception_runtime.sh --start --dummy-sensors --cameras 1

  # Pre-race hard gate (warnings fail)
  bash scripts/test_nuc_perception_runtime.sh --start --strict --cameras 1

  # Deep stress gate (recommended before race)
  bash scripts/test_nuc_perception_runtime.sh --start --deep --cameras 1

  # Pass extra args to nuc_docker_perception.sh
  bash scripts/test_nuc_perception_runtime.sh --start -- --cameras 1 --scan /scan
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --start)
      START_PIPELINE=true
      shift
      ;;
    --no-fusion)
      EXPECT_FUSION=false
      shift
      ;;
    --no-traffic)
      EXPECT_TRAFFIC=false
      shift
      ;;
    --cameras)
      CAMERA_COUNT="${2:-}"
      shift 2
      ;;
    --cameras=*)
      CAMERA_COUNT="${1#--cameras=}"
      shift
      ;;
    --dummy-camera)
      USE_DUMMY_CAMERA=true
      shift
      ;;
    --dummy-lidar)
      USE_DUMMY_LIDAR=true
      shift
      ;;
    --dummy-sensors)
      USE_DUMMY_CAMERA=true
      USE_DUMMY_LIDAR=true
      shift
      ;;
    --strict)
      STRICT=true
      shift
      ;;
    --deep)
      DEEP_MODE=true
      STRICT=true
      ECHO_REPEAT=3
      SOAK_SEC=45
      if [ "$STARTUP_WAIT" -lt 20 ]; then
        STARTUP_WAIT=20
      fi
      if [ "$PIPELINE_READY_WAIT" -lt 60 ]; then
        PIPELINE_READY_WAIT=60
      fi
      if [ "$RATE_SAMPLE_SEC" -lt 8 ]; then
        RATE_SAMPLE_SEC=8
      fi
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
    --scan)
      SCAN_TOPIC="${2:-}"
      shift 2
      ;;
    --scan=*)
      SCAN_TOPIC="${1#--scan=}"
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
    --startup-wait)
      STARTUP_WAIT="${2:-}"
      shift 2
      ;;
    --startup-wait=*)
      STARTUP_WAIT="${1#--startup-wait=}"
      shift
      ;;
    --rate-sample)
      RATE_SAMPLE_SEC="${2:-}"
      shift 2
      ;;
    --rate-sample=*)
      RATE_SAMPLE_SEC="${1#--rate-sample=}"
      shift
      ;;
    --echo-repeat)
      ECHO_REPEAT="${2:-}"
      shift 2
      ;;
    --echo-repeat=*)
      ECHO_REPEAT="${1#--echo-repeat=}"
      shift
      ;;
    --soak-sec)
      SOAK_SEC="${2:-}"
      shift 2
      ;;
    --soak-sec=*)
      SOAK_SEC="${1#--soak-sec=}"
      shift
      ;;
    --log-dir)
      LOG_DIR="${2:-}"
      shift 2
      ;;
    --log-dir=*)
      LOG_DIR="${1#--log-dir=}"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      PIPELINE_ARGS+=("$@")
      break
      ;;
    *)
      echo -e "${RED}Unknown option: $1${NC}"
      usage
      exit 1
      ;;
  esac
done

if [[ ! "$CAMERA_COUNT" =~ ^[0-9]+$ ]]; then
  echo -e "${RED}Invalid camera count: $CAMERA_COUNT${NC}"
  exit 1
fi
if [[ ! "$RATE_SAMPLE_SEC" =~ ^[0-9]+$ ]]; then
  echo -e "${RED}Invalid rate sample seconds: $RATE_SAMPLE_SEC${NC}"
  exit 1
fi
if [[ ! "$ECHO_REPEAT" =~ ^[0-9]+$ ]] || [ "$ECHO_REPEAT" -lt 1 ]; then
  echo -e "${RED}Invalid echo repeat: $ECHO_REPEAT${NC}"
  exit 1
fi
if [[ ! "$SOAK_SEC" =~ ^[0-9]+$ ]]; then
  echo -e "${RED}Invalid soak seconds: $SOAK_SEC${NC}"
  exit 1
fi

if [ "$USE_DUMMY_CAMERA" = true ] || [ "$USE_DUMMY_LIDAR" = true ]; then
  # With synthetic sensors, fusion may be structurally correct but produce no matched output.
  REQUIRE_FUSION_MESSAGES=false
fi

mkdir -p "$LOG_DIR"
TEMP_DIR="$(mktemp -d "${LOG_DIR%/}/perception_test.XXXXXX")"

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [ -n "${PIPELINE_PID:-}" ]; then
    echo -e "\n${YELLOW}Stopping started pipeline (pid=${PIPELINE_PID})...${NC}"
    kill "${PIPELINE_PID}" 2>/dev/null || true
    wait "${PIPELINE_PID}" 2>/dev/null || true
  fi
  if [ -n "${TEMP_DIR:-}" ] && [ -d "${TEMP_DIR}" ]; then
    echo -e "${BLUE}Artifacts: ${TEMP_DIR}${NC}"
    [ -n "${PIPELINE_LOG:-}" ] && [ -f "$PIPELINE_LOG" ] && echo -e "${BLUE}Pipeline log: ${PIPELINE_LOG}${NC}"
  fi
  return $rc
}
trap cleanup EXIT INT TERM

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  echo -e "${GREEN}[PASS]${NC} $*"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  echo -e "${YELLOW}[WARN]${NC} $*"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  echo -e "${RED}[FAIL]${NC} $*"
}

section() {
  echo -e "\n${CYAN}=== $* ===${NC}"
}

run_quiet() {
  "$@" >/dev/null 2>&1
}

topic_exists() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -Fxq "$topic"
}

topic_exists_retry() {
  local topic="$1"
  local tries="${2:-3}"
  local sleep_s="${3:-0.5}"
  local i
  for i in $(seq 1 "$tries"); do
    if topic_exists "$topic"; then
      return 0
    fi
    [ "$i" -lt "$tries" ] && sleep "$sleep_s"
  done
  return 1
}

topic_type() {
  local topic="$1"
  ros2 topic type "$topic" 2>/dev/null | head -1
}

count_regex_in_file() {
  local file="$1"
  local regex="$2"
  grep -E "$regex" "$file" 2>/dev/null | wc -l | tr -d ' '
}

check_file_regex() {
  local file="$1"
  local regex="$2"
  local desc="$3"
  if [ ! -f "$file" ]; then
    fail "$desc file missing: $file"
    return 1
  fi
  if grep -Eq "$regex" "$file"; then
    pass "$desc"
  else
    fail "$desc (pattern not found)"
  fi
}

check_topic_exists() {
  local topic="$1"
  local desc="$2"
  if topic_exists_retry "$topic" 3 0.4; then
    pass "$desc topic exists: $topic"
    return 0
  fi
  fail "$desc topic missing: $topic"
  return 1
}

check_topic_exists_soft() {
  local topic="$1"
  local desc="$2"
  if topic_exists_retry "$topic" 3 0.4; then
    pass "$desc topic exists: $topic"
    return 0
  fi
  warn "$desc topic missing (soft): $topic"
  return 1
}

check_topic_type() {
  local topic="$1"
  local expected="$2"
  local desc="$3"
  local actual
  actual="$(topic_type "$topic" || true)"
  if [ -z "$actual" ]; then
    fail "$desc type query failed: $topic"
    return 1
  fi
  if [ "$actual" = "$expected" ]; then
    pass "$desc type OK ($actual)"
  else
    fail "$desc type mismatch for $topic (expected $expected, got $actual)"
  fi
}

check_topic_type_soft() {
  local topic="$1"
  local expected="$2"
  local desc="$3"
  local actual
  actual="$(topic_type "$topic" || true)"
  if [ -z "$actual" ]; then
    warn "$desc type query failed (soft): $topic"
    return 1
  fi
  if [ "$actual" = "$expected" ]; then
    pass "$desc type OK ($actual)"
  else
    warn "$desc type mismatch (soft) for $topic (expected $expected, got $actual)"
  fi
}

topic_echo_once() {
  local topic="$1"
  local outfile="$2"
  timeout 6 ros2 topic echo "$topic" --once >"$outfile" 2>"$outfile.err"
}

topic_echo_once_sensor() {
  local topic="$1"
  local outfile="$2"
  local err="$outfile.err"
  if timeout 6 ros2 topic echo "$topic" --once \
    --qos-reliability best_effort \
    --qos-durability volatile \
    >"$outfile" 2>"$err"; then
    return 0
  fi
  if grep -qi "unrecognized arguments" "$err" 2>/dev/null; then
    timeout 6 ros2 topic echo "$topic" --once >"$outfile" 2>"$err"
    return $?
  fi
  return 1
}

check_topic_echo_once() {
  local topic="$1"
  local desc="$2"
  local outfile="$TEMP_DIR/$(echo "$topic" | tr '/' '_').echo.txt"
  if topic_echo_once "$topic" "$outfile"; then
    pass "$desc message received once: $topic"
    return 0
  fi
  fail "$desc no message within timeout: $topic"
  if [ -s "$outfile.err" ]; then
    warn "$desc echo stderr: $(tr '\n' ' ' < "$outfile.err" | sed 's/[[:space:]]\+/ /g')"
  fi
  return 1
}

check_topic_echo_once_soft() {
  local topic="$1"
  local desc="$2"
  local outfile="$TEMP_DIR/$(echo "$topic" | tr '/' '_').echo.txt"
  if topic_echo_once "$topic" "$outfile"; then
    pass "$desc message received once: $topic"
    return 0
  fi
  warn "$desc no message within timeout (soft): $topic"
  if [ -s "$outfile.err" ]; then
    warn "$desc echo stderr: $(tr '\n' ' ' < "$outfile.err" | sed 's/[[:space:]]\+/ /g')"
  fi
  return 1
}

check_topic_echo_repeat() {
  local topic="$1"
  local desc="$2"
  local repeats="${3:-$ECHO_REPEAT}"
  local mode="${4:-hard}"  # hard|soft
  local ok=0
  local i
  local outfile

  if [ "$repeats" -le 1 ]; then
    if [ "$mode" = "soft" ]; then
      check_topic_echo_once_soft "$topic" "$desc"
    else
      check_topic_echo_once "$topic" "$desc"
    fi
    return 0
  fi

  section "Repeated Message Flow: $desc ($repeats x)"
  for i in $(seq 1 "$repeats"); do
    outfile="$TEMP_DIR/$(echo "${topic}_repeat_${i}" | tr '/' '_').echo.txt"
    if topic_echo_once "$topic" "$outfile"; then
      ok=$((ok + 1))
      pass "$desc repeat $i/$repeats message received: $topic"
    else
      if [ "$mode" = "soft" ]; then
        warn "$desc repeat $i/$repeats timeout (soft): $topic"
      else
        fail "$desc repeat $i/$repeats timeout: $topic"
      fi
      if [ -s "$outfile.err" ]; then
        warn "$desc repeat $i stderr: $(tr '\n' ' ' < "$outfile.err" | sed 's/[[:space:]]\+/ /g')"
      fi
    fi
  done

  if [ "$ok" -eq "$repeats" ]; then
    pass "$desc repeat-flow stable ($ok/$repeats)"
  elif [ "$mode" = "soft" ]; then
    warn "$desc repeat-flow intermittent (soft): $ok/$repeats"
  else
    fail "$desc repeat-flow intermittent: $ok/$repeats"
  fi
}

check_topic_rate() {
  local topic="$1"
  local min_rate="$2"
  local desc="$3"
  local outfile="$TEMP_DIR/$(echo "$topic" | tr '/' '_').hz.txt"

  if timeout "${RATE_SAMPLE_SEC}" ros2 topic hz "$topic" >"$outfile" 2>&1; then
    :
  else
    # ros2 topic hz often exits via timeout; that's fine if it printed rate.
    :
  fi

  local line rate
  line="$(grep -E 'average rate:' "$outfile" | tail -1 || true)"
  if [ -z "$line" ]; then
    warn "$desc rate unavailable (no samples yet): $topic"
    return 0
  fi

  rate="$(echo "$line" | sed -E 's/.*average rate:[[:space:]]*([0-9.]+).*/\1/')"
  if [ -z "$rate" ]; then
    warn "$desc rate parse failed: $topic"
    return 0
  fi

  if awk -v r="$rate" -v m="$min_rate" 'BEGIN { exit !(r >= m) }'; then
    pass "$desc rate OK: ${rate} Hz >= ${min_rate} Hz"
  else
    warn "$desc rate low: ${rate} Hz < ${min_rate} Hz"
  fi
}

extract_field_value() {
  local file="$1"
  local key="$2"
  grep -m1 -E "^[[:space:]]*${key}:" "$file" | sed -E "s/^[[:space:]]*${key}:[[:space:]]*//"
}

check_tf_echo() {
  local from_frame="$1"
  local to_frame="$2"
  local desc="$3"
  local outfile="$TEMP_DIR/tf_${from_frame}_to_${to_frame}.txt"
  if ! command -v ros2 >/dev/null 2>&1; then
    warn "$desc skipped (ros2 missing)"
    return 0
  fi
  if ! ros2 pkg list 2>/dev/null | grep -qx "tf2_ros"; then
    warn "$desc skipped (tf2_ros package missing)"
    return 0
  fi
  if timeout 4 ros2 run tf2_ros tf2_echo "$from_frame" "$to_frame" >"$outfile" 2>&1; then
    pass "$desc TF available: $from_frame -> $to_frame"
    return 0
  fi
  if grep -qiE 'At time|Translation|Rotation' "$outfile"; then
    pass "$desc TF available (timed out after output): $from_frame -> $to_frame"
    return 0
  fi
  warn "$desc TF unavailable: $from_frame -> $to_frame"
  return 0
}

snapshot_ros_state() {
  section "ROS Snapshot"
  ros2 node list >"$TEMP_DIR/node_list.txt" 2>"$TEMP_DIR/node_list.err" || true
  ros2 topic list >"$TEMP_DIR/topic_list.txt" 2>"$TEMP_DIR/topic_list.err" || true
  ros2 topic list -t >"$TEMP_DIR/topic_list_types.txt" 2>"$TEMP_DIR/topic_list_types.err" || true
  pass "Saved node/topic snapshots to $TEMP_DIR"
}

check_script_syntax_smoke() {
  section "Local Script Syntax"
  local sh py

  for sh in \
    "$SCRIPT_DIR/nuc_docker_perception.sh" \
    "$SCRIPT_DIR/test_nuc_perception_runtime.sh"; do
    if bash -n "$sh" >/dev/null 2>&1; then
      pass "bash -n OK: $(basename "$sh")"
    else
      fail "bash -n failed: $(basename "$sh")"
    fi
  done

  for py in \
    "$SCRIPT_DIR/dummy_camera_publisher.py" \
    "$SCRIPT_DIR/dummy_lidar_publisher.py" \
    "$SCRIPT_DIR/camera_info_fallback_publisher.py" \
    "$SCRIPT_DIR/scan_static_tf_fallback.py" \
    "$SCRIPT_DIR/pointcloud_to_occupancy_grid.py" \
    "$SCRIPT_DIR/detection2d_to_tier4_rois.py"; do
    if python3 -c "compile(open('$py','rb').read(), '$py', 'exec')" >/dev/null 2>&1; then
      pass "Python parse OK: $(basename "$py")"
    else
      fail "Python parse failed: $(basename "$py")"
    fi
  done
}

check_detection_stack_static_wiring() {
  section "Detection Stack Static Wiring (source checks)"

  local detection_launch="$DETECTION_WS_SRC_DIR/tier4_perception_launch/launch/detection_module.launch.xml"
  local bytetrack_launch="$DETECTION_WS_SRC_DIR/autoware_bytetrack/launch/bytetrack.launch.xml"
  local yolov8_launch="$DETECTION_WS_SRC_DIR/autoware_tensorrt_yolox/launch/yolov8.launch.xml"
  local bytetrack_node="$DETECTION_WS_SRC_DIR/autoware_bytetrack/autoware_bytetrack/bytetrack_node.py"
  local bridge_node="$DETECTION_WS_SRC_DIR/autoware_detection_autoware_bridge/autoware_detection_autoware_bridge/detection_autoware_bridge_node.py"

  check_file_regex "$yolov8_launch" 'class_allowlist" default="0,2"' \
    "YOLOv8 launch default allowlist is person+car (0,2)"
  check_file_regex "$detection_launch" 'let name="detector_ns" value="yolov8/" if="\$\(var use_bytetrack\)"' \
    "Detection launch namespaces raw YOLO ROIs under /yolov8 when ByteTrack enabled"
  check_file_regex "$detection_launch" 'autoware_bridge/input_objects" default="rois0"' \
    "Bridge consumes ByteTrack output topic rois0"
  check_file_regex "$detection_launch" 'autoware_bridge/output_tracked_objects" default="/perception/object_recognition/tracking/objects"' \
    "Bridge publishes tracked objects to planner topic"

  check_file_regex "$bytetrack_launch" 'detection_rect" default="yolov8/rois0"' \
    "ByteTrack launch default input is raw YOLO topic yolov8/rois0"
  check_file_regex "$bytetrack_launch" 'tracked_rect" default="rois0"' \
    "ByteTrack launch default output is tracked topic rois0"

  check_file_regex "$bytetrack_node" 'Second-stage association with low-score detections' \
    "ByteTrack node includes low-score second-stage association (ByteTrack core behavior)"
  check_file_regex "$bytetrack_node" "det\\.id = str\\(track\\.track_id\\)" \
    "ByteTrack node writes stable track ID into Detection2D.id"
  check_file_regex "$bridge_node" "if hasattr\\(det, 'id'\\):" \
    "Bridge reads Detection2D.id when creating UUIDs/lanelet IDs"
  check_file_regex "$bridge_node" 'uuid5' \
    "Bridge derives deterministic UUIDs from track IDs / bbox fallback"
}

node_exists_substr() {
  local needle="$1"
  ros2 node list 2>/dev/null | grep -Fqi "$needle"
}

node_exists_substr_retry() {
  local needle="$1"
  local tries="${2:-3}"
  local sleep_s="${3:-0.5}"
  local i
  for i in $(seq 1 "$tries"); do
    if node_exists_substr "$needle"; then
      return 0
    fi
    [ "$i" -lt "$tries" ] && sleep "$sleep_s"
  done
  return 1
}

check_node_exists_substr() {
  local needle="$1"
  local desc="$2"
  if node_exists_substr_retry "$needle" 4 0.5; then
    pass "$desc node present (*$needle*)"
  else
    fail "$desc node missing (*$needle*)"
  fi
}

check_optional_node_exists_substr() {
  local needle="$1"
  local desc="$2"
  if node_exists_substr_retry "$needle" 4 0.5; then
    pass "$desc node present (*$needle*)"
  else
    warn "$desc node missing (*$needle*)"
  fi
}

topic_info_counts() {
  local topic="$1"
  local out="$TEMP_DIR/$(echo "$topic" | tr '/' '_').topic_info.txt"
  ros2 topic info "$topic" >"$out" 2>"$out.err" || {
    sleep 0.4
    ros2 topic info "$topic" >"$out" 2>"$out.err" || return 1
  }
  local pubs subs
  pubs="$(grep -E 'Publisher count:' "$out" | sed -E 's/.*Publisher count:[[:space:]]*([0-9]+).*/\1/' | tail -1)"
  subs="$(grep -E 'Subscription count:' "$out" | sed -E 's/.*Subscription count:[[:space:]]*([0-9]+).*/\1/' | tail -1)"
  echo "${pubs:-0} ${subs:-0}"
}

check_topic_topology() {
  local topic="$1"
  local min_pubs="$2"
  local min_subs="$3"
  local desc="$4"

  if ! topic_exists_retry "$topic" 3 0.4; then
    fail "$desc topology skipped; topic missing: $topic"
    return 1
  fi

  local counts pubs subs
  counts="$(topic_info_counts "$topic" || true)"
  pubs="$(echo "$counts" | awk '{print $1}')"
  subs="$(echo "$counts" | awk '{print $2}')"
  pubs="${pubs:-0}"
  subs="${subs:-0}"

  if awk -v v="$pubs" -v m="$min_pubs" 'BEGIN { exit !(v >= m) }'; then
    pass "$desc publishers OK: $pubs >= $min_pubs"
  else
    fail "$desc publishers low: $pubs < $min_pubs"
  fi

  if awk -v v="$subs" -v m="$min_subs" 'BEGIN { exit !(v >= m) }'; then
    pass "$desc subscribers OK: $subs >= $min_subs"
  else
    if [ "$min_subs" -gt 0 ]; then
      fail "$desc subscribers low: $subs < $min_subs"
    else
      pass "$desc subscribers OK: $subs >= $min_subs"
    fi
  fi
}

check_topic_topology_soft() {
  local topic="$1"
  local min_pubs="$2"
  local min_subs="$3"
  local desc="$4"

  if ! topic_exists_retry "$topic" 3 0.4; then
    warn "$desc topology skipped; topic missing (soft): $topic"
    return 1
  fi

  local counts pubs subs
  counts="$(topic_info_counts "$topic" || true)"
  pubs="$(echo "$counts" | awk '{print $1}')"
  subs="$(echo "$counts" | awk '{print $2}')"
  pubs="${pubs:-0}"
  subs="${subs:-0}"

  if awk -v v="$pubs" -v m="$min_pubs" 'BEGIN { exit !(v >= m) }'; then
    pass "$desc publishers OK: $pubs >= $min_pubs"
  else
    warn "$desc publishers low (soft): $pubs < $min_pubs"
  fi

  if awk -v v="$subs" -v m="$min_subs" 'BEGIN { exit !(v >= m) }'; then
    pass "$desc subscribers OK: $subs >= $min_subs"
  else
    warn "$desc subscribers low (soft): $subs < $min_subs"
  fi
}

check_nodes() {
  section "Nodes"

  check_node_exists_substr "detection_autoware_bridge" "Autoware detection bridge"
  local i
  for i in $(seq 0 $((CAMERA_COUNT - 1))); do
    check_node_exists_substr "yolov8_detector${i}" "YOLO detector cam${i}"
    check_node_exists_substr "bytetrack${i}" "ByteTrack cam${i}"
  done
  check_optional_node_exists_substr "laserscan_to_pcl" "LaserScan->PointCloud node"
  check_optional_node_exists_substr "pointcloud_to_occupancy_grid_fallback" "Occupancy fallback"

  if [ "$EXPECT_TRAFFIC" = true ]; then
    check_optional_node_exists_substr "traffic_light_roi_detector" "Traffic light ROI detector"
    check_optional_node_exists_substr "traffic_light_classifier" "Traffic light classifier"
  fi

  if [ "$EXPECT_FUSION" = true ]; then
    check_optional_node_exists_substr "detection2d_to_tier4_rois" "ROI adapter"
    check_optional_node_exists_substr "roi_cluster_fusion" "ROI fusion"
    check_optional_node_exists_substr "euclidean_cluster" "Euclidean cluster"
  fi

  if [ "$USE_DUMMY_CAMERA" = true ]; then
    check_node_exists_substr "dummy_camera_publisher_0" "Dummy camera publisher"
  fi
  if [ "$USE_DUMMY_LIDAR" = true ]; then
    check_optional_node_exists_substr "dummy_lidar_publisher" "Dummy lidar publisher"
  fi
}

raw_yolo_topic_for_camera() {
  local idx="$1"
  echo "/yolov8/rois${idx}"
}

check_topic_topologies() {
  section "Topic Topology (pub/sub counts)"
  local raw_rois0
  raw_rois0="$(raw_yolo_topic_for_camera 0)"
  if topic_exists "$raw_rois0"; then
    check_topic_topology "$raw_rois0" 1 1 "Raw YOLO ROI detections (cam0)" || true
  else
    warn "Raw YOLO ROI topic missing (cam0): $raw_rois0"
  fi
  check_topic_topology "/rois0" 1 1 "Tracked ROI detections" || true
  check_topic_topology "/perception/object_recognition/detection/objects" 1 0 "DetectedObjects" || true
  check_topic_topology "/perception/object_recognition/tracking/objects" 1 0 "TrackedObjects" || true
  check_topic_topology "/perception/object_recognition/objects" 1 0 "PredictedObjects" || true

  if topic_exists "/perception/lidar/pointcloud"; then
    # occupancy + clustering should subscribe when fusion and occupancy fallback are on
    local min_subs=1
    [ "$EXPECT_FUSION" = true ] && min_subs=2
    check_topic_topology "/perception/lidar/pointcloud" 1 "$min_subs" "LiDAR pointcloud" || true
  fi
  if topic_exists "/perception/obstacle/pointcloud"; then
    check_topic_topology "/perception/obstacle/pointcloud" 1 0 "Planner obstacle pointcloud" || true
  fi
  if topic_exists "/perception/occupancy_grid"; then
    check_topic_topology "/perception/occupancy_grid" 1 0 "Occupancy grid" || true
  fi
  if [ "$EXPECT_FUSION" = true ] && topic_exists "/perception/fusion/rois0"; then
    if [ "$USE_DUMMY_CAMERA" = true ] || [ "$USE_DUMMY_LIDAR" = true ]; then
      check_topic_topology_soft "/perception/fusion/rois0" 1 1 "ROI fusion adapter output" || true
    else
      check_topic_topology "/perception/fusion/rois0" 1 1 "ROI fusion adapter output" || true
    fi
  fi
  if [ "$EXPECT_FUSION" = true ] && topic_exists "/perception/lidar/clusters"; then
    check_topic_topology "/perception/lidar/clusters" 1 1 "LiDAR clusters" || true
  fi
  if [ "$EXPECT_FUSION" = true ] && topic_exists "/perception/object_recognition/detection/fused_objects"; then
    check_topic_topology "/perception/object_recognition/detection/fused_objects" 1 0 "Fused objects" || true
  fi
}

count_word_in_file() {
  local file="$1"
  local word="$2"
  grep -Eo "\\b${word}\\b" "$file" 2>/dev/null | wc -l | tr -d ' '
}

check_pointcloud_sanity() {
  local topic="$1"
  local desc="$2"
  local file="$TEMP_DIR/$(echo "$topic" | tr '/' '_').echo.txt"
  [ -f "$file" ] || return 0

  if grep -qE "name:[[:space:]]*x" "$file" && grep -qE "name:[[:space:]]*y" "$file"; then
    pass "$desc fields include x/y"
  else
    fail "$desc missing x/y fields"
  fi
  if grep -qE "name:[[:space:]]*z" "$file"; then
    pass "$desc field includes z"
  else
    warn "$desc z field not found"
  fi

  local frame_id
  frame_id="$(extract_field_value "$file" "frame_id" | tr -d '\"')"
  [ -n "$frame_id" ] && pass "$desc frame_id: $frame_id" || warn "$desc frame_id parse failed"

  local width height point_step row_step
  width="$(extract_field_value "$file" "width" | tr -d ' ')"
  height="$(extract_field_value "$file" "height" | tr -d ' ')"
  point_step="$(extract_field_value "$file" "point_step" | tr -d ' ')"
  row_step="$(extract_field_value "$file" "row_step" | tr -d ' ')"
  [[ "$width" =~ ^[0-9]+$ ]] && [ "$width" -gt 0 ] && pass "$desc width > 0 ($width)" || warn "$desc width invalid ($width)"
  [[ "$height" =~ ^[0-9]+$ ]] && [ "$height" -gt 0 ] && pass "$desc height > 0 ($height)" || warn "$desc height invalid ($height)"
  [[ "$point_step" =~ ^[0-9]+$ ]] && [ "$point_step" -gt 0 ] && pass "$desc point_step > 0 ($point_step)" || warn "$desc point_step invalid ($point_step)"
  [[ "$row_step" =~ ^[0-9]+$ ]] && [ "$row_step" -gt 0 ] && pass "$desc row_step > 0 ($row_step)" || warn "$desc row_step invalid ($row_step)"
}

check_occupancy_sanity() {
  local topic="/perception/occupancy_grid"
  local file="$TEMP_DIR/_perception_occupancy_grid.echo.txt"
  [ -f "$file" ] || return 0

  local width height res frame
  width="$(extract_field_value "$file" "width" | tr -d ' ')"
  height="$(extract_field_value "$file" "height" | tr -d ' ')"
  res="$(extract_field_value "$file" "resolution" | tr -d ' ')"
  frame="$(extract_field_value "$file" "frame_id" | tr -d '\"')"

  [[ "$width" =~ ^[0-9]+$ ]] && [ "$width" -gt 0 ] && pass "Occupancy width > 0 ($width)" || fail "Occupancy width invalid ($width)"
  [[ "$height" =~ ^[0-9]+$ ]] && [ "$height" -gt 0 ] && pass "Occupancy height > 0 ($height)" || fail "Occupancy height invalid ($height)"
  if [ -n "$res" ] && awk -v r="$res" 'BEGIN { exit !(r > 0 && r <= 5.0) }'; then
    pass "Occupancy resolution sane ($res)"
  else
    fail "Occupancy resolution invalid ($res)"
  fi
  [ -n "$frame" ] && pass "Occupancy frame_id present ($frame)" || warn "Occupancy frame_id parse failed"

  # Best-effort occupied-cell signal check (especially useful with dummy lidar)
  local occupied_count
  occupied_count="$(count_word_in_file "$file" "100")"
  if [ "${occupied_count:-0}" -gt 0 ]; then
    pass "Occupancy grid has occupied cells (100 count=${occupied_count})"
  else
    if [ "$USE_DUMMY_LIDAR" = true ]; then
      pass "Occupancy grid sampled echo showed no occupied cells (dummy mode; echo may truncate/omit populated cells)"
    else
      warn "Occupancy grid has no occupied cells in sampled message"
    fi
  fi
}

check_detection_array_sanity() {
  local topic="$1"
  local desc="$2"
  local file="$TEMP_DIR/$(echo "$topic" | tr '/' '_').echo.txt"
  [ -f "$file" ] || return 0

  if grep -qE "detections:" "$file"; then
    pass "$desc contains detections array"
  else
    fail "$desc missing detections array"
  fi
  if grep -qE "bbox:" "$file"; then
    pass "$desc contains bbox fields"
  else
    if [ "$USE_DUMMY_CAMERA" = true ]; then
      pass "$desc bbox fields absent in sampled message (expected when dummy detections are empty)"
    else
      warn "$desc bbox fields not found (may be empty detections)"
    fi
  fi
}

check_tracked_roi_sanity() {
  local topic="/rois0"
  local file="$TEMP_DIR/_rois0.echo.txt"
  [ -f "$file" ] || return 0

  if grep -qE '^[[:space:]]*id:' "$file"; then
    pass "Tracked ROI detections include Detection2D.id (track IDs)"
  else
    if grep -qE '^[[:space:]]*detections:[[:space:]]*\[\]' "$file"; then
      pass "Tracked ROI sample empty; Detection2D.id not expected in sampled echo"
    else
      warn "Tracked ROI detections missing Detection2D.id (track IDs not visible in echo)"
    fi
  fi
}

check_bytetrack_chain_sanity() {
  local raw_topic
  raw_topic="$(raw_yolo_topic_for_camera 0)"
  local raw_file="$TEMP_DIR/$(echo "$raw_topic" | tr '/' '_').echo.txt"
  local tracked_file="$TEMP_DIR/_rois0.echo.txt"
  [ -f "$raw_file" ] || return 0
  [ -f "$tracked_file" ] || return 0

  local raw_count tracked_count
  raw_count="$(count_regex_in_file "$raw_file" '^[[:space:]]*bbox:')"
  tracked_count="$(count_regex_in_file "$tracked_file" '^[[:space:]]*bbox:')"

  if [[ "$raw_count" =~ ^[0-9]+$ ]]; then
    pass "Raw YOLO ROI sample bbox count: $raw_count"
  fi
  if [[ "$tracked_count" =~ ^[0-9]+$ ]]; then
    pass "ByteTrack ROI sample bbox count: $tracked_count"
  fi

  if [[ "$raw_count" =~ ^[0-9]+$ ]] && [[ "$tracked_count" =~ ^[0-9]+$ ]]; then
    if awk -v t="$tracked_count" -v r="$raw_count" 'BEGIN { exit !(t <= r + 2) }'; then
      pass "ByteTrack tracked ROI count is plausible vs raw YOLO (tracked=$tracked_count raw=$raw_count)"
    else
      warn "ByteTrack tracked ROI count suspicious vs raw YOLO (tracked=$tracked_count raw=$raw_count)"
    fi
  fi
}

check_bridge_output_sanity() {
  local tracked_file="$TEMP_DIR/_perception_object_recognition_tracking_objects.echo.txt"
  local predicted_file="$TEMP_DIR/_perception_object_recognition_objects.echo.txt"
  local detected_file="$TEMP_DIR/_perception_object_recognition_detection_objects.echo.txt"
  local tracked_empty=false predicted_empty=false detected_empty=false

  if [ -f "$tracked_file" ] && grep -qE '^[[:space:]]*objects:[[:space:]]*\[\]' "$tracked_file"; then
    tracked_empty=true
  fi
  if [ -f "$predicted_file" ] && grep -qE '^[[:space:]]*objects:[[:space:]]*\[\]' "$predicted_file"; then
    predicted_empty=true
  fi
  if [ -f "$detected_file" ] && grep -qE '^[[:space:]]*objects:[[:space:]]*\[\]' "$detected_file"; then
    detected_empty=true
  fi

  if [ -f "$tracked_file" ]; then
    if grep -qE 'object_id:' "$tracked_file"; then
      pass "TrackedObjects contains object_id (UUIDs)"
    else
      if [ "$tracked_empty" = true ]; then
        pass "TrackedObjects sample empty (object_id not observable)"
      else
        fail "TrackedObjects missing object_id"
      fi
    fi
    if grep -qE 'classification:' "$tracked_file"; then
      pass "TrackedObjects contains classification"
    else
      if [ "$tracked_empty" = true ]; then
        pass "TrackedObjects sample empty (classification not observable)"
      else
        fail "TrackedObjects missing classification"
      fi
    fi
  fi

  if [ -f "$predicted_file" ]; then
    if grep -qE 'object_id:' "$predicted_file"; then
      pass "PredictedObjects contains object_id (UUIDs)"
    else
      if [ "$predicted_empty" = true ]; then
        pass "PredictedObjects sample empty (object_id not observable)"
      else
        fail "PredictedObjects missing object_id"
      fi
    fi
    if grep -qE 'predicted_paths:' "$predicted_file"; then
      pass "PredictedObjects contains predicted_paths"
    else
      if [ "$predicted_empty" = true ]; then
        pass "PredictedObjects sample empty (predicted_paths not observable)"
      else
        warn "PredictedObjects missing predicted_paths"
      fi
    fi
  fi

  if [ -f "$detected_file" ]; then
    if grep -qE 'existence_probability:' "$detected_file"; then
      pass "DetectedObjects contains existence_probability"
    else
      if [ "$detected_empty" = true ]; then
        pass "DetectedObjects sample empty (existence_probability not observable)"
      else
        warn "DetectedObjects missing existence_probability"
      fi
    fi
  fi
}

check_laserscan_sanity() {
  local file="$TEMP_DIR/scan.echo.txt"
  [ -f "$file" ] || return 0

  if grep -qE "ranges:" "$file"; then
    pass "LaserScan message contains ranges"
  else
    fail "LaserScan message missing ranges"
  fi

  local angle_min angle_max range_min range_max
  angle_min="$(extract_field_value "$file" "angle_min" | tr -d ' ')"
  angle_max="$(extract_field_value "$file" "angle_max" | tr -d ' ')"
  range_min="$(extract_field_value "$file" "range_min" | tr -d ' ')"
  range_max="$(extract_field_value "$file" "range_max" | tr -d ' ')"

  if [ -n "$angle_min" ] && [ -n "$angle_max" ] && awk -v a="$angle_min" -v b="$angle_max" 'BEGIN { exit !(b > a) }'; then
    pass "LaserScan angle bounds sane ($angle_min .. $angle_max)"
  else
    warn "LaserScan angle bounds invalid ($angle_min .. $angle_max)"
  fi
  if [ -n "$range_min" ] && [ -n "$range_max" ] && awk -v a="$range_min" -v b="$range_max" 'BEGIN { exit !(b > a && a >= 0) }'; then
    pass "LaserScan range bounds sane ($range_min .. $range_max)"
  else
    warn "LaserScan range bounds invalid ($range_min .. $range_max)"
  fi
}

check_image_message_sanity() {
  local topic="$1"
  local desc="$2"
  local file="$TEMP_DIR/$(echo "$topic" | tr '/' '_').echo.txt"
  [ -f "$file" ] || return 0

  local width height encoding step frame
  width="$(extract_field_value "$file" "width" | tr -d ' ')"
  height="$(extract_field_value "$file" "height" | tr -d ' ')"
  encoding="$(extract_field_value "$file" "encoding" | tr -d '\"')"
  step="$(extract_field_value "$file" "step" | tr -d ' ')"
  frame="$(extract_field_value "$file" "frame_id" | tr -d '\"')"

  [[ "$width" =~ ^[0-9]+$ ]] && [ "$width" -gt 0 ] && pass "$desc width > 0 ($width)" || fail "$desc width invalid ($width)"
  [[ "$height" =~ ^[0-9]+$ ]] && [ "$height" -gt 0 ] && pass "$desc height > 0 ($height)" || fail "$desc height invalid ($height)"
  [ -n "$encoding" ] && pass "$desc encoding present ($encoding)" || fail "$desc encoding missing"
  [[ "$step" =~ ^[0-9]+$ ]] && [ "$step" -gt 0 ] && pass "$desc step > 0 ($step)" || warn "$desc step invalid ($step)"
  [ -n "$frame" ] && pass "$desc frame_id present ($frame)" || warn "$desc frame_id missing"
}

check_fusion_adapter_sanity() {
  local topic="/perception/fusion/rois0"
  local file="$TEMP_DIR/_perception_fusion_rois0.echo.txt"
  [ -f "$file" ] || return 0
  local empty_sample=false

  if grep -qE 'feature_objects:[[:space:]]*\[\]|objects:[[:space:]]*\[\]' "$file"; then
    empty_sample=true
  fi

  if grep -qE "feature:" "$file"; then
    pass "Fusion ROI adapter output contains feature field"
  else
    if [ "$empty_sample" = true ]; then
      pass "Fusion ROI adapter output sample empty (feature field not observable)"
    else
      warn "Fusion ROI adapter output missing feature field"
    fi
  fi
  if grep -qE "roi:" "$file"; then
    pass "Fusion ROI adapter output contains ROI field"
  else
    if [ "$empty_sample" = true ]; then
      pass "Fusion ROI adapter output sample empty (ROI field not observable)"
    else
      warn "Fusion ROI adapter output missing ROI field"
    fi
  fi
}

check_fused_objects_sanity() {
  local topic="/perception/object_recognition/detection/fused_objects"
  local file="$TEMP_DIR/_perception_object_recognition_detection_fused_objects.echo.txt"
  [ -f "$file" ] || return 0

  if grep -qE "classification:" "$file"; then
    pass "Fused objects message contains classification"
  else
    if grep -qE 'feature_objects:[[:space:]]*\[\]|objects:[[:space:]]*\[\]' "$file"; then
      pass "Fused objects sample empty (classification not observable)"
    else
      warn "Fused objects message missing classification (may be empty)"
    fi
  fi
}

check_cluster_message_sanity() {
  local file="$TEMP_DIR/_perception_lidar_clusters.echo.txt"
  [ -f "$file" ] || return 0

  if grep -qE "feature_objects:|objects:" "$file"; then
    pass "LiDAR clusters message contains object array"
  else
    warn "LiDAR clusters message object array not found"
  fi
  if grep -qE "feature:" "$file"; then
    pass "LiDAR clusters message contains feature field"
  else
    warn "LiDAR clusters message feature field not found"
  fi
}

check_traffic_output_sanity() {
  local file="$TEMP_DIR/_perception_traffic_light_recognition_traffic_signals.echo.txt"
  [ -f "$file" ] || return 0
  if grep -qE "traffic_light_groups:" "$file"; then
    pass "Traffic light output contains traffic_light_groups"
  else
    warn "Traffic light output missing traffic_light_groups"
  fi
}

check_message_content_sanity() {
  section "Message Content Sanity"
  check_detection_array_sanity "/yolov8/rois0" "Raw YOLO ROI detections"
  check_detection_array_sanity "/rois0" "Tracked ROI detections"
  check_tracked_roi_sanity
  check_bytetrack_chain_sanity
  check_laserscan_sanity
  check_pointcloud_sanity "/perception/lidar/pointcloud" "LiDAR pointcloud"
  check_pointcloud_sanity "/perception/obstacle/pointcloud" "Planner obstacle pointcloud"
  check_cluster_message_sanity
  check_occupancy_sanity
  check_fusion_adapter_sanity
  check_fused_objects_sanity
  check_traffic_output_sanity
  check_bridge_output_sanity
  local i img
  for i in $(seq 0 $((CAMERA_COUNT - 1))); do
    img="${CAMERA_IMAGE_TOPICS[$i]:-}"
    [ -n "$img" ] && check_image_message_sanity "$img" "Camera $i image"
  done
}

check_pipeline_log_health() {
  [ -n "$PIPELINE_LOG" ] || return 0
  [ -f "$PIPELINE_LOG" ] || return 0

  section "Pipeline Log Health"

  local hard_pat soft_pat hard_hits soft_hits
  hard_pat='Traceback|Segmentation fault|Fatal|FATAL|terminate called|core dumped|No module named|ImportError|RuntimeError'
  soft_pat='\\b(error|failed|exception|transform|extrapolation)\\b'

  hard_hits="$(grep -Ein "$hard_pat" "$PIPELINE_LOG" || true)"
  if [ -n "$hard_hits" ]; then
    fail "Hard error patterns found in pipeline log"
    echo "$hard_hits" | head -20
  else
    pass "No hard error patterns in pipeline log"
  fi

  # Filter out known benign warnings/messages emitted by the script itself.
  soft_hits="$(grep -Ein "$soft_pat" "$PIPELINE_LOG" | grep -Eiv 'not installed|not yet active|may need data|skipping roi_cluster_fusion|camera-only|warning|dummy|fallback watcher started' || true)"
  if [ -n "$soft_hits" ]; then
    warn "Suspicious soft error/failure lines found in pipeline log"
    echo "$soft_hits" | head -20
  else
    pass "No suspicious soft error/failure lines in pipeline log"
  fi
}

check_nuc_pipeline_report_in_log() {
  [ -n "$PIPELINE_LOG" ] || return 0
  [ -f "$PIPELINE_LOG" ] || return 0
  section "NUC Script Report (Log-derived)"

  if grep -q "NUC Perception Pipeline Running" "$PIPELINE_LOG"; then
    pass "NUC script reached final running banner"
  else
    warn "NUC script final running banner not found in log"
  fi

  if [ "$EXPECT_FUSION" = true ]; then
    if grep -q "ROI Cluster Fusion running" "$PIPELINE_LOG"; then
      pass "NUC script reports ROI Cluster Fusion running"
    elif grep -q "Fusion preflight failed" "$PIPELINE_LOG"; then
      fail "NUC script reports Fusion preflight failed"
    elif grep -qi "autoware_image_projection_based_fusion not installed" "$PIPELINE_LOG"; then
      warn "Fusion package missing according to pipeline log"
    else
      warn "Fusion status line not found in pipeline log"
    fi
  fi

  if grep -q "camera_info missing" "$PIPELINE_LOG"; then
    if [ "$USE_DUMMY_CAMERA" = true ]; then
      pass "camera_info missing at startup is expected in dummy camera mode (fallback may have recovered)"
    else
      warn "At least one camera_info topic was missing at startup (fallback may have recovered)"
    fi
  fi
}

source_envs() {
  # Many ROS setup scripts assume nounset is off; temporarily relax shell flags
  # so this test harness does not exit silently during environment sourcing.
  local flags="$-"
  set +e
  set +u

  source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
  source /autoware/install/setup.bash >/dev/null 2>&1 || true
  if [ -f "$MODULE_DIR/detection_ws/install/setup.bash" ]; then
    source "$MODULE_DIR/detection_ws/install/setup.bash" >/dev/null 2>&1 || true
  fi

  case "$flags" in
    *u*) set -u ;;
  esac
  case "$flags" in
    *e*) set -e ;;
  esac
}

start_pipeline_if_requested() {
  if [ "$START_PIPELINE" != true ]; then
    return 0
  fi

  section "Starting Pipeline"
  PIPELINE_LOG="$TEMP_DIR/nuc_docker_perception.log"

  local cmd=(bash "$SCRIPT_DIR/nuc_docker_perception.sh")
  if [ "${#PIPELINE_ARGS[@]}" -gt 0 ]; then
    cmd+=("${PIPELINE_ARGS[@]}")
  else
    cmd+=(--cameras "$CAMERA_COUNT")
  fi
  if [ "$USE_DUMMY_CAMERA" = true ]; then
    cmd+=(--dummy-camera)
  fi
  if [ "$USE_DUMMY_LIDAR" = true ]; then
    cmd+=(--dummy-lidar)
  fi
  if [ "$EXPECT_FUSION" = false ]; then
    cmd+=(--no-fusion)
  fi

  echo -e "${BLUE}Command:${NC} ${cmd[*]}"
  "${cmd[@]}" >"$PIPELINE_LOG" 2>&1 &
  PIPELINE_PID=$!
  echo -e "${BLUE}Pipeline PID:${NC} $PIPELINE_PID"
  echo -e "${BLUE}Waiting ${STARTUP_WAIT}s for startup...${NC}"
  sleep "$STARTUP_WAIT"

  if ! kill -0 "$PIPELINE_PID" 2>/dev/null; then
    fail "Pipeline process exited early"
    tail -120 "$PIPELINE_LOG" || true
    return 1
  fi
  pass "Pipeline process still running after startup wait"

  if [ "$DEEP_MODE" = true ]; then
    local deadline banner_seen=false
    deadline=$((SECONDS + PIPELINE_READY_WAIT))
    echo -e "${BLUE}Waiting up to ${PIPELINE_READY_WAIT}s for pipeline running banner (deep mode)...${NC}"
    while [ "$SECONDS" -lt "$deadline" ]; do
      if grep -q "NUC Perception Pipeline Running" "$PIPELINE_LOG" 2>/dev/null; then
        banner_seen=true
        break
      fi
      if ! kill -0 "$PIPELINE_PID" 2>/dev/null; then
        fail "Pipeline process died while waiting for running banner"
        tail -120 "$PIPELINE_LOG" || true
        return 1
      fi
      sleep 1
    done
    if [ "$banner_seen" = true ]; then
      pass "Pipeline running banner observed before deep checks"
      sleep 2
    else
      warn "Pipeline running banner not observed before deep checks timeout (${PIPELINE_READY_WAIT}s)"
    fi
  fi
}

print_context() {
  section "Environment"
  if command -v ros2 >/dev/null 2>&1; then
    pass "ros2 CLI found: $(command -v ros2)"
  else
    fail "ros2 CLI not found"
    return 1
  fi

  section "Packages"
  pkg_installed() {
    ros2 pkg prefix "$1" >/dev/null 2>&1
  }
  for pkg in pointcloud_to_laserscan tier4_perception_launch autoware_detection_autoware_bridge autoware_bytetrack autoware_tensorrt_yolox; do
    if pkg_installed "$pkg"; then
      pass "Package installed: $pkg"
    else
      fail "Package missing: $pkg"
    fi
  done

  for pkg in autoware_euclidean_cluster autoware_image_projection_based_fusion tf2_ros; do
    if pkg_installed "$pkg"; then
      pass "Optional package installed: $pkg"
    else
      warn "Optional package missing: $pkg"
    fi
  done
}

check_pipeline_pid_alive() {
  [ -n "${PIPELINE_PID:-}" ] || return 0
  if kill -0 "$PIPELINE_PID" 2>/dev/null; then
    pass "Pipeline PID alive ($PIPELINE_PID)"
  else
    fail "Pipeline PID not alive ($PIPELINE_PID)"
  fi
}

detect_camera_topics() {
  CAMERA_IMAGE_TOPICS=()
  CAMERA_INFO_TOPICS=()
  local topics
  topics="$(ros2 topic list 2>/dev/null || true)"
  local i img info alt
  for i in $(seq 0 $((CAMERA_COUNT - 1))); do
    img="/sensing/camera/camera${i}/image_raw"
    if ! echo "$topics" | grep -Fxq "$img"; then
      alt="$(echo "$topics" | grep -E "/camera${i}/image_raw$|^/camera/image_raw$" | head -1 || true)"
      [ -n "$alt" ] && img="$alt"
    fi
    info="${img%/image_raw}/camera_info"
    CAMERA_IMAGE_TOPICS[$i]="$img"
    CAMERA_INFO_TOPICS[$i]="$info"
  done
}

check_topics_and_messages() {
  section "Topic Presence"

  check_topic_exists "$(raw_yolo_topic_for_camera 0)" "Raw YOLO ROI detections cam0"
  check_topic_exists "/perception/object_recognition/detection/objects" "DetectedObjects"
  check_topic_exists "/perception/object_recognition/tracking/objects" "TrackedObjects"
  check_topic_exists "/perception/object_recognition/objects" "PredictedObjects"
  check_topic_exists "/rois0" "Tracked ROI detections"
  check_topic_exists_soft "/perception/lidar/pointcloud" "LiDAR pointcloud"
  check_topic_exists_soft "/perception/obstacle/pointcloud" "Planner obstacle pointcloud"
  check_topic_exists_soft "/perception/occupancy_grid" "Planner occupancy grid"

  if [ "$EXPECT_TRAFFIC" = true ]; then
    check_topic_exists "/perception/traffic_light_recognition/traffic_signals" "Traffic light output"
  fi

  if [ "$EXPECT_FUSION" = true ]; then
    check_topic_exists_soft "/perception/fusion/rois0" "ROI fusion adapter output"
    check_topic_exists_soft "/perception/lidar/clusters" "LiDAR clusters (ROI fusion input)"
    check_topic_exists_soft "/perception/object_recognition/detection/fused_objects" "ROI fused objects"
  fi

  section "Topic Types"
  if topic_exists "$(raw_yolo_topic_for_camera 0)"; then
    check_topic_type "$(raw_yolo_topic_for_camera 0)" "vision_msgs/msg/Detection2DArray" "Raw YOLO ROI detections cam0" || true
  fi
  check_topic_type "/rois0" "vision_msgs/msg/Detection2DArray" "Tracked ROI detections" || true
  check_topic_type "/perception/object_recognition/detection/objects" "autoware_perception_msgs/msg/DetectedObjects" "DetectedObjects" || true
  check_topic_type "/perception/object_recognition/tracking/objects" "autoware_perception_msgs/msg/TrackedObjects" "TrackedObjects" || true
  check_topic_type "/perception/object_recognition/objects" "autoware_perception_msgs/msg/PredictedObjects" "PredictedObjects" || true
  if topic_exists "/perception/occupancy_grid"; then
    check_topic_type "/perception/occupancy_grid" "nav_msgs/msg/OccupancyGrid" "Occupancy grid"
  else
    warn "Skipping occupancy grid type check; topic missing"
  fi
  if topic_exists "/perception/obstacle/pointcloud"; then
    check_topic_type "/perception/obstacle/pointcloud" "sensor_msgs/msg/PointCloud2" "Planner obstacle pointcloud"
  else
    warn "Skipping obstacle pointcloud type check; topic missing"
  fi

  if [ "$EXPECT_TRAFFIC" = true ] && topic_exists "/perception/traffic_light_recognition/traffic_signals"; then
    check_topic_type "/perception/traffic_light_recognition/traffic_signals" \
      "autoware_perception_msgs/msg/TrafficLightGroupArray" "Traffic light output" || true
  fi
  if [ "$EXPECT_FUSION" = true ] && topic_exists "/perception/fusion/rois0"; then
    check_topic_type "/perception/fusion/rois0" \
      "tier4_perception_msgs/msg/DetectedObjectsWithFeature" "ROI fusion adapter output" || true
  fi

  section "Message Flow (echo --once)"
  if topic_exists "$(raw_yolo_topic_for_camera 0)"; then
    check_topic_echo_once "$(raw_yolo_topic_for_camera 0)" "Raw YOLO ROI detections cam0" || true
  fi
  if [ "$USE_DUMMY_CAMERA" = true ]; then
    check_topic_echo_once_soft "/rois0" "Tracked ROI detections" || true
  else
    check_topic_echo_once "/rois0" "Tracked ROI detections" || true
  fi
  check_topic_echo_once "/perception/object_recognition/detection/objects" "DetectedObjects" || true
  if [ "$USE_DUMMY_CAMERA" = true ]; then
    check_topic_echo_once_soft "/perception/object_recognition/tracking/objects" "TrackedObjects" || true
    check_topic_echo_once_soft "/perception/object_recognition/objects" "PredictedObjects" || true
  else
    check_topic_echo_once "/perception/object_recognition/tracking/objects" "TrackedObjects" || true
    check_topic_echo_once "/perception/object_recognition/objects" "PredictedObjects" || true
  fi

  if topic_exists "/perception/lidar/pointcloud"; then
    check_topic_echo_once "/perception/lidar/pointcloud" "LiDAR pointcloud" || true
  fi
  if topic_exists "/perception/obstacle/pointcloud"; then
    check_topic_echo_once "/perception/obstacle/pointcloud" "Planner obstacle pointcloud" || true
  fi
  if topic_exists "/perception/occupancy_grid"; then
    check_topic_echo_once "/perception/occupancy_grid" "Planner occupancy grid" || true
  fi

  if [ "$EXPECT_TRAFFIC" = true ] && topic_exists "/perception/traffic_light_recognition/traffic_signals"; then
    check_topic_echo_once "/perception/traffic_light_recognition/traffic_signals" "Traffic light output" || true
  fi

  if [ "$EXPECT_FUSION" = true ]; then
    if topic_exists "/perception/fusion/rois0"; then
      check_topic_echo_once "/perception/fusion/rois0" "ROI fusion adapter output" || true
    fi
    if topic_exists "/perception/lidar/clusters"; then
      check_topic_echo_once "/perception/lidar/clusters" "LiDAR clusters" || true
    fi
    if topic_exists "/perception/object_recognition/detection/fused_objects"; then
      if [ "$REQUIRE_FUSION_MESSAGES" = true ]; then
        check_topic_echo_once "/perception/object_recognition/detection/fused_objects" "ROI fused objects" || true
      else
        check_topic_echo_once_soft "/perception/object_recognition/detection/fused_objects" "ROI fused objects" || true
      fi
    fi
  fi

  if [ "$ECHO_REPEAT" -gt 1 ]; then
    section "Message Flow Stability (repeated echo)"
    if topic_exists "$(raw_yolo_topic_for_camera 0)"; then
      check_topic_echo_repeat "$(raw_yolo_topic_for_camera 0)" "Raw YOLO ROI detections cam0" "$ECHO_REPEAT" hard || true
    fi
    if [ "$USE_DUMMY_CAMERA" = true ]; then
      check_topic_echo_repeat "/rois0" "Tracked ROI detections" "$ECHO_REPEAT" soft || true
    else
      check_topic_echo_repeat "/rois0" "Tracked ROI detections" "$ECHO_REPEAT" hard || true
    fi
    check_topic_echo_repeat "/perception/object_recognition/detection/objects" "DetectedObjects" "$ECHO_REPEAT" hard || true
    check_topic_echo_repeat "/perception/object_recognition/tracking/objects" "TrackedObjects" "$ECHO_REPEAT" hard || true
    if topic_exists "/perception/lidar/pointcloud"; then
      check_topic_echo_repeat "/perception/lidar/pointcloud" "LiDAR pointcloud" "$ECHO_REPEAT" hard || true
    elif topic_exists "$LIDAR_TOPIC"; then
      check_topic_echo_repeat "$LIDAR_TOPIC" "Raw LiDAR pointcloud" "$ECHO_REPEAT" soft || true
    fi
    if topic_exists "/perception/occupancy_grid"; then
      check_topic_echo_repeat "/perception/occupancy_grid" "Planner occupancy grid" "$ECHO_REPEAT" hard || true
    fi
    if [ "$EXPECT_FUSION" = true ] && topic_exists "/perception/fusion/rois0"; then
      check_topic_echo_repeat "/perception/fusion/rois0" "ROI fusion adapter output" "$ECHO_REPEAT" hard || true
    fi
    if [ "$EXPECT_FUSION" = true ] && topic_exists "/perception/object_recognition/detection/fused_objects"; then
      if [ "$REQUIRE_FUSION_MESSAGES" = true ]; then
        check_topic_echo_repeat "/perception/object_recognition/detection/fused_objects" "ROI fused objects" "$ECHO_REPEAT" hard || true
      else
        check_topic_echo_repeat "/perception/object_recognition/detection/fused_objects" "ROI fused objects" "$ECHO_REPEAT" soft || true
      fi
    fi
  fi
}

check_rates() {
  section "Rates (best effort)"
  if topic_exists "$(raw_yolo_topic_for_camera 0)"; then
    check_topic_rate "$(raw_yolo_topic_for_camera 0)" 1.0 "Raw YOLO ROI detections cam0"
  fi
  check_topic_rate "/rois0" 1.0 "Tracked ROI detections"
  check_topic_rate "/perception/object_recognition/detection/objects" 1.0 "DetectedObjects"
  check_topic_rate "/perception/object_recognition/tracking/objects" 1.0 "TrackedObjects"
  check_topic_rate "/perception/object_recognition/objects" 1.0 "PredictedObjects"
  if topic_exists "/perception/lidar/pointcloud"; then
    check_topic_rate "/perception/lidar/pointcloud" 1.0 "LiDAR pointcloud"
  fi
  if topic_exists "/perception/obstacle/pointcloud"; then
    check_topic_rate "/perception/obstacle/pointcloud" 1.0 "Planner obstacle pointcloud"
  fi
}

check_camera_info_and_tf() {
  section "CameraInfo / TF"
  detect_camera_topics
  local i img info echo_file width height dist_model cam_frame

  for i in $(seq 0 $((CAMERA_COUNT - 1))); do
    img="${CAMERA_IMAGE_TOPICS[$i]:-}"
    info="${CAMERA_INFO_TOPICS[$i]:-}"

    if [ -z "$img" ]; then
      warn "Camera $i topic resolution failed"
      continue
    fi

    if topic_exists_retry "$img" 3 0.5; then
      pass "Camera $i image topic exists: $img"
      local img_echo_file_sensor
      img_echo_file_sensor="$TEMP_DIR/$(echo "$img" | tr '/' '_').echo.txt"
      if topic_echo_once_sensor "$img" "$img_echo_file_sensor"; then
        pass "Camera $i image message received once: $img"
      else
        fail "Camera $i image no message within timeout: $img"
      fi
    else
      fail "Camera $i image topic missing: $img"
      continue
    fi

    if topic_exists_retry "$info" 3 0.5; then
      pass "Camera $i camera_info topic exists: $info"
    else
      fail "Camera $i camera_info topic missing: $info"
      continue
    fi

    echo_file="$TEMP_DIR/camera${i}_info.echo.txt"
    if topic_echo_once_sensor "$info" "$echo_file"; then
      pass "Camera $i camera_info message received"
    else
      if [ "$USE_DUMMY_CAMERA" = true ]; then
        warn "Camera $i camera_info no message (dummy mode; likely QoS/discovery timing)"
      else
        fail "Camera $i camera_info no message"
      fi
      continue
    fi

    width="$(extract_field_value "$echo_file" "width" | tr -d ' ')"
    height="$(extract_field_value "$echo_file" "height" | tr -d ' ')"
    dist_model="$(extract_field_value "$echo_file" "distortion_model" | tr -d '\"')"
    cam_frame="$(extract_field_value "$echo_file" "frame_id" | tr -d '\"')"

    [ -n "$width" ] && pass "Camera $i width reported: $width" || warn "Camera $i width not parsed"
    [ -n "$height" ] && pass "Camera $i height reported: $height" || warn "Camera $i height not parsed"
    [ -n "$dist_model" ] && pass "Camera $i distortion model: $dist_model" || warn "Camera $i distortion model not parsed"

    if grep -q "k:" "$echo_file"; then
      pass "Camera $i intrinsics matrix present (k)"
    else
      warn "Camera $i intrinsics matrix missing in echo output"
    fi

    local img_echo_file img_w img_h img_frame
    img_echo_file="$TEMP_DIR/$(echo "$img" | tr '/' '_').echo.txt"
    if [ -f "$img_echo_file" ]; then
      img_w="$(extract_field_value "$img_echo_file" "width" | tr -d ' ')"
      img_h="$(extract_field_value "$img_echo_file" "height" | tr -d ' ')"
      img_frame="$(extract_field_value "$img_echo_file" "frame_id" | tr -d '\"')"
      if [ -n "$width" ] && [ -n "$img_w" ] && [ "$width" = "$img_w" ]; then
        pass "Camera $i image/camera_info width match ($width)"
      else
        warn "Camera $i image/camera_info width mismatch (img=$img_w info=$width)"
      fi
      if [ -n "$height" ] && [ -n "$img_h" ] && [ "$height" = "$img_h" ]; then
        pass "Camera $i image/camera_info height match ($height)"
      else
        warn "Camera $i image/camera_info height mismatch (img=$img_h info=$height)"
      fi
      if [ -n "$cam_frame" ] && [ -n "$img_frame" ] && [ "$cam_frame" = "$img_frame" ]; then
        pass "Camera $i image/camera_info frame match ($cam_frame)"
      else
        warn "Camera $i image/camera_info frame mismatch (img=$img_frame info=$cam_frame)"
      fi
    else
      warn "Camera $i image echo snapshot missing for image/camera_info cross-check"
    fi

    if [ -n "$cam_frame" ]; then
      pass "Camera $i frame_id: $cam_frame"
      check_tf_echo "$BASE_FRAME" "$cam_frame" "Camera $i TF"
    else
      warn "Camera $i frame_id missing in CameraInfo"
    fi
  done

  if topic_exists "$SCAN_TOPIC"; then
    local scan_echo="$TEMP_DIR/scan.echo.txt"
    if topic_echo_once_sensor "$SCAN_TOPIC" "$scan_echo"; then
      pass "LaserScan message received: $SCAN_TOPIC"
      local scan_frame
      scan_frame="$(extract_field_value "$scan_echo" "frame_id" | tr -d '\"')"
      if [ -n "$scan_frame" ]; then
        pass "LaserScan frame_id: $scan_frame"
        check_tf_echo "$BASE_FRAME" "$scan_frame" "LaserScan TF"
      else
        warn "LaserScan frame_id parse failed"
      fi
    else
      warn "No LaserScan message on $SCAN_TOPIC (possible pointcloud-only LiDAR mode)"
    fi
  else
    warn "LaserScan topic not found: $SCAN_TOPIC (possible pointcloud-only LiDAR mode)"
  fi

  if topic_exists "$LIDAR_TOPIC"; then
    pass "Raw LiDAR pointcloud topic exists: $LIDAR_TOPIC"
  else
    if [ "$USE_DUMMY_LIDAR" = true ]; then
      pass "Raw LiDAR pointcloud topic not expected in dummy lidar mode ($LIDAR_TOPIC)"
    else
      warn "Raw LiDAR pointcloud topic not found: $LIDAR_TOPIC"
    fi
  fi
}

check_fusion_wiring_ready() {
  [ "$EXPECT_FUSION" = true ] || return 0
  section "Fusion Wiring Readiness"

  if ! topic_exists_retry "/perception/fusion/rois0" 3 0.4; then
    warn "Fusion adapter topic missing; skipping fusion wiring checks"
    return 0
  fi

  check_topic_type "/perception/fusion/rois0" "tier4_perception_msgs/msg/DetectedObjectsWithFeature" "Fusion ROI adapter output" || true
  check_topic_topology_soft "/perception/fusion/rois0" 1 1 "Fusion ROI adapter output" || true

  if topic_exists "/perception/lidar/clusters"; then
    check_topic_type_soft "/perception/lidar/clusters" "tier4_perception_msgs/msg/DetectedObjectsWithFeature" "LiDAR clusters"
    check_topic_topology "/perception/lidar/clusters" 1 1 "LiDAR clusters" || true
  else
    warn "LiDAR clusters topic missing; fusion likely disabled or clustering package missing"
  fi

  if topic_exists "/perception/object_recognition/detection/fused_objects"; then
    check_topic_type_soft "/perception/object_recognition/detection/fused_objects" \
      "tier4_perception_msgs/msg/DetectedObjectsWithFeature" "Fused objects output"
    check_topic_topology "/perception/object_recognition/detection/fused_objects" 1 0 "Fused objects output" || true
  else
    warn "Fused objects topic missing"
  fi
}

soak_monitor() {
  [ "$SOAK_SEC" -gt 0 ] || return 0
  section "Soak Stability (${SOAK_SEC}s)"
  local end now iteration=0
  local topic_miss=0 node_miss=0 pid_dead=0
  end=$((SECONDS + SOAK_SEC))

  while [ "$SECONDS" -lt "$end" ]; do
    iteration=$((iteration + 1))
    now=$((end - SECONDS))
    echo -e "${BLUE}[soak]${NC} iteration=$iteration remaining=${now}s"

    if [ -n "${PIPELINE_PID:-}" ] && ! kill -0 "$PIPELINE_PID" 2>/dev/null; then
      pid_dead=$((pid_dead + 1))
      fail "Soak: pipeline PID died during monitoring"
      break
    fi

    for t in \
      "/rois0" \
      "/perception/object_recognition/detection/objects" \
      "/perception/object_recognition/tracking/objects" \
      "/perception/object_recognition/objects"; do
      if ! topic_exists_retry "$t" 4 0.4; then
        topic_miss=$((topic_miss + 1))
        fail "Soak: required topic disappeared: $t"
      fi
    done

    if [ "$EXPECT_TRAFFIC" = true ] && ! topic_exists_retry "/perception/traffic_light_recognition/traffic_signals" 3 0.4; then
      topic_miss=$((topic_miss + 1))
      warn "Soak: traffic topic absent in this poll"
    fi

    if topic_exists_retry "/perception/occupancy_grid" 3 0.4; then
      :
    else
      topic_miss=$((topic_miss + 1))
      warn "Soak: occupancy topic absent in this poll"
    fi

    if [ "$EXPECT_FUSION" = true ] && topic_exists_retry "/perception/fusion/rois0" 3 0.4; then
      :
    elif [ "$EXPECT_FUSION" = true ]; then
      topic_miss=$((topic_miss + 1))
      warn "Soak: fusion adapter topic absent in this poll"
    fi

    if ! node_exists_substr_retry "detection_autoware_bridge" 3 0.4; then
      node_miss=$((node_miss + 1))
      fail "Soak: detection bridge node missing"
    fi
    if ! node_exists_substr_retry "bytetrack0" 3 0.4; then
      node_miss=$((node_miss + 1))
      fail "Soak: bytetrack0 node missing"
    fi

    sleep "$SOAK_POLL_SEC"
  done

  if [ "$pid_dead" -eq 0 ]; then
    pass "Soak: pipeline PID stayed alive"
  fi
  if [ "$topic_miss" -eq 0 ]; then
    pass "Soak: no topic disappearance observed"
  else
    warn "Soak: topic miss observations=$topic_miss"
  fi
  if [ "$node_miss" -eq 0 ]; then
    pass "Soak: key nodes stayed visible"
  else
    warn "Soak: node miss observations=$node_miss"
  fi
}

print_debug_hints() {
  section "Debug Commands"
  cat <<EOF
ros2 topic list | grep -E "perception|rois|camera_info|traffic|occupancy|scan"
ros2 topic hz /rois0
ros2 topic hz /perception/object_recognition/detection/objects
ros2 topic hz /perception/lidar/pointcloud
ros2 topic echo /sensing/camera/camera0/camera_info --once
ros2 topic echo /perception/occupancy_grid --once
ros2 topic echo /perception/object_recognition/detection/fused_objects --once
ros2 topic type /perception/object_recognition/detection/fused_objects
EOF
  if [ -n "$PIPELINE_LOG" ] && [ -f "$PIPELINE_LOG" ]; then
    echo ""
    echo "Pipeline log tail:"
    tail -80 "$PIPELINE_LOG" || true
  fi
}

main() {
  echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
  echo -e "${CYAN}  NUC Perception Runtime Test (Smoke + Integration)${NC}"
  echo -e "${CYAN}══════════════════════════════════════════════════════════${NC}"
  echo -e "${BLUE}Mode:${NC} $([ "$START_PIPELINE" = true ] && echo 'start+test' || echo 'check-only')"
  echo -e "${BLUE}Expect fusion:${NC} $EXPECT_FUSION"
  echo -e "${BLUE}Expect traffic:${NC} $EXPECT_TRAFFIC"
  echo -e "${BLUE}Cameras:${NC} $CAMERA_COUNT"
  echo -e "${BLUE}Base frame:${NC} $BASE_FRAME"
  echo -e "${BLUE}Dummy camera:${NC} $USE_DUMMY_CAMERA"
  echo -e "${BLUE}Dummy lidar:${NC} $USE_DUMMY_LIDAR"
  echo -e "${BLUE}Strict mode:${NC} $STRICT"
  echo -e "${BLUE}Deep mode:${NC} $DEEP_MODE"
  echo -e "${BLUE}Require fusion messages:${NC} $REQUIRE_FUSION_MESSAGES"
  echo -e "${BLUE}Echo repeat:${NC} $ECHO_REPEAT"
  echo -e "${BLUE}Soak sec:${NC} $SOAK_SEC"
  echo -e "${BLUE}Artifacts dir:${NC} $TEMP_DIR"

  # After setup/startup, keep collecting findings instead of exiting on first failing check.
  set +e
  source_envs
  check_script_syntax_smoke
  check_detection_stack_static_wiring
  print_context || true
  start_pipeline_if_requested || true
  check_pipeline_pid_alive
  snapshot_ros_state
  check_nodes
  check_topics_and_messages
  check_topic_topologies
  check_fusion_wiring_ready
  check_rates
  check_camera_info_and_tf
  check_message_content_sanity
  soak_monitor
  check_pipeline_log_health
  check_nuc_pipeline_report_in_log

  if [ "$STRICT" = true ] && [ "$WARN_COUNT" -gt 0 ]; then
    fail "Strict mode: warnings present ($WARN_COUNT)"
  fi

  section "Summary"
  echo -e "${GREEN}PASS:${NC} $PASS_COUNT"
  echo -e "${YELLOW}WARN:${NC} $WARN_COUNT"
  echo -e "${RED}FAIL:${NC} $FAIL_COUNT"

  if [ "$STRICT" = true ] && [ "$WARN_COUNT" -gt 0 ]; then
    echo -e "${RED}STRICT FAIL:${NC} warnings must be zero"
  fi

  if [ "$FAIL_COUNT" -gt 0 ]; then
    print_debug_hints
    exit 1
  fi

  echo -e "${GREEN}Runtime smoke test completed successfully.${NC}"
}

main "$@"
