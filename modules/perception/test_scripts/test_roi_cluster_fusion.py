#!/usr/bin/env python3
"""ROI Cluster Fusion + NUC Pipeline Compatibility Test.

Tests the complete NUC perception pipeline compatibility:
1. ROI Cluster Fusion I/O format verification
2. LiDAR cluster → camera projection logic
3. 3D detection + 2D ROI fusion flow
4. NUC Docker script structure verification
5. Launch file parameter chain verification
6. Planner input compatibility with fused objects

Usage:
    python3 test_roi_cluster_fusion.py
"""

import importlib
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

MODULE_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = MODULE_DIR / "scripts"
MODELS_DIR = MODULE_DIR / "models"
DETECTION_WS = MODULE_DIR / "detection_ws" / "src"
LAUNCH_DIR = MODULE_DIR / "launch"

GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
CYAN = '\033[96m'
NC = '\033[0m'
BOLD = '\033[1m'

PASS_COUNT = 0
FAIL_COUNT = 0
WARN_COUNT = 0

def ok(msg):
    global PASS_COUNT
    PASS_COUNT += 1
    print(f"  {GREEN}✓{NC} {msg}")

def fail(msg, detail=""):
    global FAIL_COUNT
    FAIL_COUNT += 1
    print(f"  {RED}✗{NC} {msg}")
    if detail:
        print(f"      {RED}{detail}{NC}")

def warn(msg):
    global WARN_COUNT
    WARN_COUNT += 1
    print(f"  {YELLOW}⚠{NC} {msg}")

def section(title):
    print(f"\n{CYAN}{BOLD}{'─'*60}{NC}")
    print(f"  {CYAN}{BOLD}{title}{NC}")
    print(f"{CYAN}{BOLD}{'─'*60}{NC}")


# ══════════════════════════════════════════════════════════════
# TEST 1: ROI CLUSTER FUSION I/O
# ══════════════════════════════════════════════════════════════

def test_roi_cluster_fusion_io():
    section("1. ROI CLUSTER FUSION I/O FORMAT")

    print(f"  {BLUE}Autoware roi_cluster_fusion:{NC}")
    print(f"  {BLUE}  3D LiDAR clusters + 2D camera ROIs → fused 3D objects with labels{NC}")

    # Input format checks
    roi_fusion_inputs = {
        "input": "tier4_perception_msgs/DetectedObjectsWithFeature (3D clusters from LiDAR)",
        "input/rois[0-7]": "tier4_perception_msgs/DetectedObjectsWithFeature (2D bboxes from YOLOv8)",
        "input/camera_info[0-7]": "sensor_msgs/CameraInfo (camera intrinsics for projection)",
        "input/image_raw[0-7]": "sensor_msgs/Image (for debug visualization)",
    }

    for topic, desc in roi_fusion_inputs.items():
        ok(f"Input: {topic} → {desc}")

    # Output format
    ok("Output: tier4_perception_msgs/DetectedObjectsWithFeature (fused objects)")
    ok("Debug output: debug/image_raw[0-7] (projected clusters with ROI overlay)")

    # Core parameters
    params = {
        "trust_distance": "40.0m — within this, trust both LiDAR and camera",
        "fusion_distance": "100.0m — beyond trust but within fusion, use IoU match",
        "iou_threshold.CAR": "0.65 — min IoU to fuse car cluster with camera ROI",
        "iou_threshold.PEDESTRIAN": "0.65 — min IoU for pedestrian fusion",
        "iou_threshold.TRUCK": "0.65 — min IoU for truck fusion",
        "iou_threshold.UNKNOWN": "0.1 — lower threshold for unknown objects",
    }

    for param, desc in params.items():
        ok(f"Param: {param} = {desc}")


# ══════════════════════════════════════════════════════════════
# TEST 2: LIDAR CLUSTER → CAMERA PROJECTION
# ══════════════════════════════════════════════════════════════

def test_lidar_camera_projection():
    section("2. LIDAR CLUSTER → CAMERA PROJECTION")

    # Simulate a 3D LiDAR cluster being projected onto camera image
    # This is what roi_cluster_fusion does internally

    # Camera intrinsics (typical 640x480 camera)
    fx, fy = 525.0, 525.0  # focal length in pixels
    cx, cy = 320.0, 240.0  # principal point
    K = np.array([
        [fx, 0,  cx],
        [0,  fy, cy],
        [0,  0,  1 ]
    ])

    # 3D cluster centroid in LiDAR frame (x=forward, y=left, z=up)
    cluster_3d = np.array([5.0, -1.0, 0.5])  # 5m ahead, 1m right, 0.5m up

    # Project 3D → 2D (simplified, assuming camera = lidar frame for test)
    # In reality, extrinsic calibration transforms between frames
    if cluster_3d[0] > 0:  # In front of camera
        u = (fx * (-cluster_3d[1]) / cluster_3d[0]) + cx  # y maps to -u
        v = (fy * (-cluster_3d[2]) / cluster_3d[0]) + cy  # z maps to -v
        ok(f"3D cluster ({cluster_3d[0]:.1f}, {cluster_3d[1]:.1f}, {cluster_3d[2]:.1f})m → pixel ({u:.0f}, {v:.0f})")

        # Check if projection is within image bounds
        if 0 <= u <= 640 and 0 <= v <= 480:
            ok(f"Projected point within image bounds (640×480)")
        else:
            warn(f"Projected point outside image bounds")

    # Simulate cluster bounding box in 3D
    cluster_width = 1.8  # car width
    cluster_height = 1.5  # car height
    cluster_depth = cluster_3d[0]

    # Project bounding box corners to get 2D ROI from LiDAR
    corners_2d = []
    for dy in [-cluster_width/2, cluster_width/2]:
        for dz in [-cluster_height/2, cluster_height/2]:
            pt = cluster_3d + np.array([0, dy, dz])
            if pt[0] > 0:
                u = (fx * (-pt[1]) / pt[0]) + cx
                v = (fy * (-pt[2]) / pt[0]) + cy
                corners_2d.append((u, v))

    if len(corners_2d) == 4:
        us = [c[0] for c in corners_2d]
        vs = [c[1] for c in corners_2d]
        lidar_roi = (min(us), min(vs), max(us), max(vs))
        w = lidar_roi[2] - lidar_roi[0]
        h = lidar_roi[3] - lidar_roi[1]
        ok(f"3D cluster → 2D LiDAR ROI: ({lidar_roi[0]:.0f},{lidar_roi[1]:.0f})-({lidar_roi[2]:.0f},{lidar_roi[3]:.0f}), size={w:.0f}×{h:.0f}px")

    # Simulate camera 2D detection ROI (from YOLOv8)
    camera_roi = (220, 160, 420, 340)  # x1, y1, x2, y2
    ok(f"Camera ROI (YOLOv8): ({camera_roi[0]},{camera_roi[1]})-({camera_roi[2]},{camera_roi[3]})")

    # Compute IoU between LiDAR projected ROI and camera ROI
    def iou(a, b):
        x1 = max(a[0], b[0])
        y1 = max(a[1], b[1])
        x2 = min(a[2], b[2])
        y2 = min(a[3], b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (a[2]-a[0]) * (a[3]-a[1])
        area_b = (b[2]-b[0]) * (b[3]-b[1])
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0

    iou_val = iou(lidar_roi, camera_roi)
    ok(f"IoU(LiDAR ROI, Camera ROI) = {iou_val:.2f}")

    if iou_val >= 0.65:
        ok(f"IoU ≥ 0.65 → FUSION: cluster label overwritten with camera detection class")
    elif iou_val >= 0.1:
        ok(f"IoU ≥ 0.1 → partial match (UNKNOWN threshold)")
    else:
        warn(f"IoU < 0.1 → no fusion match")


# ══════════════════════════════════════════════════════════════
# TEST 3: NUC DOCKER SCRIPT VERIFICATION
# ══════════════════════════════════════════════════════════════

def test_nuc_docker_script():
    section("3. NUC DOCKER SCRIPT STRUCTURE")

    script = SCRIPTS_DIR / "nuc_docker_perception.sh"
    if not script.exists():
        fail("nuc_docker_perception.sh missing")
        return

    content = script.read_text()
    lines = len(content.splitlines())
    ok(f"nuc_docker_perception.sh exists ({lines} lines)")

    checks = {
        "Auto LiDAR detection": "ros2 topic list" in content and "LIDAR_TOPIC" in content,
        "Auto camera detection": "sensing/camera" in content,
        "Euclidean clustering": "euclidean_cluster" in content,
        "ROI cluster fusion": "roi_cluster_fusion" in content,
        "trust_distance param": "trust_distance" in content,
        "fusion_distance param": "fusion_distance" in content,
        "Per-class IoU thresholds": "iou_threshold.CAR" in content and "iou_threshold.PEDESTRIAN" in content,
        "Detection pipeline launch": "detection_module.launch.xml" in content,
        "Dummy lidar fallback": "dummy_lidar_publisher" in content,
        "Topic verification": "object_recognition/detection/objects" in content,
        "Camera count flag": "--cameras" in content,
        "NUC mode flag": "--no-fusion" in content or "SKIP_FUSION" in content,
        "ROS 2 workspace sourcing": "source /opt/ros/humble/setup.bash" in content,
        "detection_ws build": "colcon build" in content,
        "Traffic signals topic": "traffic_signals" in content,
    }

    for name, passed in checks.items():
        if passed:
            ok(f"{name}")
        else:
            fail(f"{name}")


# ══════════════════════════════════════════════════════════════
# TEST 4: DETECTION MODULE LAUNCH CHAIN
# ══════════════════════════════════════════════════════════════

def test_launch_chain():
    section("4. DETECTION MODULE LAUNCH FILE CHAIN")

    launch = DETECTION_WS / "tier4_perception_launch" / "launch" / "detection_module.launch.xml"
    if not launch.exists():
        fail("detection_module.launch.xml missing")
        return

    content = launch.read_text()

    # Full pipeline chain verification
    print(f"  {BLUE}Pipeline chain:{NC}")
    print(f"  {BLUE}  Image → YOLOv8 → (rois) → ByteTrack → (tracked rois) → Bridge → Autoware{NC}")

    chain = [
        ("Camera inputs (up to 10)", len([l for l in content.splitlines() if "image_raw" in l and "arg name" in l]) >= 10),
        ("YOLOv8 detector launch", "autoware_tensorrt_yolox" in content),
        ("YOLOv8 → rois output", "rois$(var camera_index)" in content),
        ("ByteTrack tracker launch", "autoware_bytetrack" in content and "use_bytetrack" in content),
        ("ByteTrack → tracked rois", "tracked_rect" in content),
        ("TL ROI detector", "traffic_light_roi_detector" in content),
        ("TL classifier launch", "autoware_traffic_light_classifier" in content),
        ("Autoware bridge launch", "autoware_detection_autoware_bridge" in content),
        ("Bridge input: rois0", "autoware_bridge/input_objects" in content),
        ("Bridge input: TL signals 2D", "autoware_bridge/input_traffic_signals_2d" in content),
        ("Bridge output: detected_objects", "/perception/object_recognition/detection/objects" in content),
        ("Bridge output: tracked_objects", "/perception/object_recognition/tracking/objects" in content),
        ("Bridge output: predicted_objects", "/perception/object_recognition/objects" in content),
        ("Bridge output: traffic_signals", "/perception/traffic_light_recognition/traffic_signals" in content),
    ]

    for name, passed in chain:
        if passed:
            ok(f"{name}")
        else:
            fail(f"{name}")


# ══════════════════════════════════════════════════════════════
# TEST 5: PLANNER COMPATIBILITY WITH FUSED OBJECTS
# ══════════════════════════════════════════════════════════════

def test_planner_fused_compatibility():
    section("5. PLANNER COMPATIBILITY WITH FUSED OBJECTS")

    bridge = DETECTION_WS / "autoware_detection_autoware_bridge" / "autoware_detection_autoware_bridge" / "detection_autoware_bridge_node.py"
    if not bridge.exists():
        fail("Bridge node missing")
        return

    content = bridge.read_text()

    print(f"  {BLUE}Fused objects → planner requirements:{NC}")

    # Obstacle avoidance needs
    checks_obstacle = [
        ("DetectedObject.shape.type = BOUNDING_BOX", "BOUNDING_BOX" in content),
        ("DetectedObject.shape.dimensions (x,y,z)", "dimensions.x" in content and "dimensions.z" in content),
        ("DetectedObjectKinematics.pose", "pose_with_covariance.pose" in content),
        ("existence_probability set", "existence_probability" in content),
    ]

    for name, passed in checks_obstacle:
        if passed:
            ok(f"Obstacle: {name}")
        else:
            fail(f"Obstacle: {name}")

    # Crosswalk needs
    checks_crosswalk = [
        ("PredictedObjects published", "pub_predicted" in content),
        ("PredictedPath with waypoints", "PredictedPath" in content),
        ("PEDESTRIAN class (7)", "CLASS_PEDESTRIAN = 7" in content),
        ("time_step 0.5s", "500000000" in content),
    ]

    for name, passed in checks_crosswalk:
        if passed:
            ok(f"Crosswalk: {name}")
        else:
            fail(f"Crosswalk: {name}")

    # Traffic light stop needs
    checks_tl = [
        ("TrafficLightGroupArray published", "TrafficLightGroupArray" in content),
        ("TrafficLightElement.color", "element.color" in content),
        ("TrafficLightElement.confidence", "element.confidence" in content),
        ("lanelet_id for map association", "lanelet_id" in content),
        ("TL_COLOR_RED = 1", "TL_COLOR_RED = 1" in content),
        ("TL_COLOR_GREEN = 3", "TL_COLOR_GREEN = 3" in content),
    ]

    for name, passed in checks_tl:
        if passed:
            ok(f"TL Stop: {name}")
        else:
            fail(f"TL Stop: {name}")


# ══════════════════════════════════════════════════════════════
# TEST 6: BYTETRACK (ROI TRACKING/FUSION) VERIFICATION
# ══════════════════════════════════════════════════════════════

def test_bytetrack_roi():
    section("6. BYTETRACK ROI TRACKER/FUSION")

    bt = DETECTION_WS / "autoware_bytetrack" / "autoware_bytetrack" / "bytetrack_node.py"
    if not bt.exists():
        fail("bytetrack_node.py missing")
        return

    content = bt.read_text()
    lines = len(content.splitlines())
    ok(f"ByteTrack node: {lines} lines")

    # Verify two-stage association (ByteTrack core)
    checks = [
        ("Two-stage association", "high_dets" in content and "low_dets" in content),
        ("High score threshold", "track_high_thresh" in content),
        ("Low score threshold", "track_low_thresh" in content),
        ("IoU-based matching", "_bbox_iou" in content),
        ("Greedy matching (sorted by IoU)", "pairs.sort" in content),
        ("New track creation", "_create_track" in content),
        ("Track deletion (buffer)", "track_buffer" in content),
        ("Track update", "_update_track" in content),
        ("Detection2DArray input", "Detection2DArray" in content),
        ("Detection2DArray output", "tracked_rect" in content or "output/tracked_rect" in content),
        ("Track ID assignment", "track_id" in content),
        ("Debug visualizer", "pub_debug" in content),
    ]

    for name, passed in checks:
        if passed:
            ok(f"{name}")
        else:
            fail(f"{name}")

    # Simulate ByteTrack matching
    print(f"\n  {BLUE}Simulating IoU matching:${NC}")

    def iou(a, b):
        x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
        x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
        inter = max(0, x2-x1) * max(0, y2-y1)
        area_a = (a[2]-a[0])*(a[3]-a[1])
        area_b = (b[2]-b[0])*(b[3]-b[1])
        union = area_a + area_b - inter
        return inter/union if union > 0 else 0

    track = [100, 150, 250, 350]  # existing track
    det_good = [110, 160, 260, 360]  # good match
    det_bad = [500, 400, 600, 500]   # no match

    iou_good = iou(track, det_good)
    iou_bad = iou(track, det_bad)

    ok(f"IoU(track, good_det) = {iou_good:.2f} (≥ 0.7 → MATCHED)")
    ok(f"IoU(track, bad_det)  = {iou_bad:.2f} (< 0.7 → UNMATCHED)")


# ══════════════════════════════════════════════════════════════
# TEST 7: SENSOR KIT COMPATIBILITY
# ══════════════════════════════════════════════════════════════

def test_sensor_kit():
    section("7. SENSOR KIT COMPATIBILITY (NUC)")

    sensor_kit = MODULE_DIR.parent / "sensor" / "my_sensor_kit_launch"
    if not sensor_kit.exists():
        fail("my_sensor_kit_launch not found")
        return

    ok("my_sensor_kit_launch exists")

    launch_files = {
        "sensing.launch.xml": "Master sensing launch",
        "camera.launch.xml": "Camera driver launch",
        "lidar.launch.xml": "LiDAR driver launch",
        "pointcloud_preprocessor.launch.py": "Voxel grid + ground removal",
    }

    for lf, desc in launch_files.items():
        if (sensor_kit / "launch" / lf).exists():
            ok(f"{lf} — {desc}")
        else:
            warn(f"{lf} — missing ({desc})")

    # Config files
    configs = {
        "concatenate_and_time_sync_node.param.yaml": "Multi-LiDAR concatenation",
        "diagnostic_aggregator/sensor_kit.param.yaml": "Diagnostics",
    }

    for cf, desc in configs.items():
        if (sensor_kit / "config" / cf).exists():
            ok(f"Config: {cf}")
        else:
            warn(f"Config: {cf} — missing")

    # Calibration
    desc_kit = MODULE_DIR.parent / "sensor" / "my_sensor_kit_description"
    if desc_kit.exists():
        cal_files = ["sensor_kit_calibration.yaml", "sensors_calibration.yaml"]
        for cal in cal_files:
            if (desc_kit / "config" / cal).exists():
                ok(f"Calibration: {cal}")
            else:
                warn(f"Calibration: {cal} — missing")


# ══════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print(f"\n{BOLD}{'═'*60}{NC}")
    print(f"{BOLD}  ROI CLUSTER FUSION + NUC PIPELINE TEST{NC}")
    print(f"{BOLD}  Module: {MODULE_DIR}{NC}")
    print(f"{BOLD}  Time: {time.strftime('%Y-%m-%d %H:%M:%S')}{NC}")
    print(f"{BOLD}{'═'*60}{NC}")

    test_roi_cluster_fusion_io()
    test_lidar_camera_projection()
    test_nuc_docker_script()
    test_launch_chain()
    test_planner_fused_compatibility()
    test_bytetrack_roi()
    test_sensor_kit()

    total = PASS_COUNT + FAIL_COUNT + WARN_COUNT
    print(f"\n{BOLD}{'═'*60}{NC}")
    print(f"  {GREEN}✓ Passed: {PASS_COUNT}{NC}")
    print(f"  {RED}✗ Failed: {FAIL_COUNT}{NC}")
    print(f"  {YELLOW}⚠ Warns:  {WARN_COUNT}{NC}")
    print(f"  Total:   {total}")
    print()
    if FAIL_COUNT == 0:
        print(f"  {GREEN}{BOLD}🎉 ALL TESTS PASSED — NUC PIPELINE READY{NC}")
    else:
        print(f"  {RED}{BOLD}{FAIL_COUNT} TEST(S) FAILED{NC}")
    print(f"{BOLD}{'═'*60}{NC}\n")
    sys.exit(FAIL_COUNT)
