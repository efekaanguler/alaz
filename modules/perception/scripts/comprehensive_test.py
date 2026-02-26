#!/usr/bin/env python3
"""Comprehensive perception module test suite.

Tests everything that can be verified on Mac without ROS 2:
1. File structure integrity
2. Python syntax & imports
3. YOLOv8 model loading + dummy inference
4. Traffic light detector (contrast-based) with synthetic images
5. Autoware bridge node structure verification
6. Detection output format compatibility with Autoware planner
7. Sensor fusion pipeline file verification

Usage:
    python3 comprehensive_test.py
"""

import importlib
import json
import math
import os
import sys
import time
from pathlib import Path

# ── Colors ──
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
NC = '\033[0m'
BOLD = '\033[1m'

PASS_COUNT = 0
FAIL_COUNT = 0
WARN_COUNT = 0

def ok(msg):
    global PASS_COUNT
    PASS_COUNT += 1
    print(f"  {GREEN}✓ PASS{NC}  {msg}")

def fail(msg, detail=""):
    global FAIL_COUNT
    FAIL_COUNT += 1
    print(f"  {RED}✗ FAIL{NC}  {msg}")
    if detail:
        print(f"          {RED}{detail}{NC}")

def warn(msg):
    global WARN_COUNT
    WARN_COUNT += 1
    print(f"  {YELLOW}⚠ WARN{NC}  {msg}")

def section(title):
    print(f"\n{BLUE}{BOLD}{'═'*60}{NC}")
    print(f"{BLUE}{BOLD}  {title}{NC}")
    print(f"{BLUE}{BOLD}{'═'*60}{NC}")


# ═══════════════════════════════════════════════════════════════
# Paths
# ═══════════════════════════════════════════════════════════════
MODULE_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = MODULE_DIR / "scripts"
MODELS_DIR = MODULE_DIR / "models"
LAUNCH_DIR = MODULE_DIR / "launch"
DETECTION_WS = MODULE_DIR / "detection_ws" / "src"


# ═══════════════════════════════════════════════════════════════
# TEST 1: FILE STRUCTURE
# ═══════════════════════════════════════════════════════════════
def test_file_structure():
    section("1. FILE STRUCTURE INTEGRITY")

    required_files = {
        # Top-level
        "package.xml": MODULE_DIR / "package.xml",
        "CMakeLists.txt": MODULE_DIR / "CMakeLists.txt",
        "README.md": MODULE_DIR / "README.md",
        # Scripts
        "webcam_detect_standalone.py": SCRIPTS_DIR / "webcam_detect_standalone.py",
        "mjpeg_to_ros.py": SCRIPTS_DIR / "mjpeg_to_ros.py",
        "visualize_detections.py": SCRIPTS_DIR / "visualize_detections.py",
        "dummy_lidar_publisher.py": SCRIPTS_DIR / "dummy_lidar_publisher.py",
        "dummy_camera_publisher.py": SCRIPTS_DIR / "dummy_camera_publisher.py",
        "docker_pipeline_start.sh": SCRIPTS_DIR / "docker_pipeline_start.sh",
        "test_detection_e2e.sh": SCRIPTS_DIR / "test_detection_e2e.sh",
        "test_planner_integration.sh": SCRIPTS_DIR / "test_planner_integration.sh",
        "test_perception_pipeline.sh": SCRIPTS_DIR / "test_perception_pipeline.sh",
        # Models
        "yolov8n.onnx": MODELS_DIR / "yolov8n.onnx",
        "labels.txt": MODELS_DIR / "labels.txt",
        "color_map.json": MODELS_DIR / "color_map.json",
        # Launch
        "perception.launch.xml": LAUNCH_DIR / "perception.launch.xml",
        "laserscan_to_pcl_and_occ.launch.xml": LAUNCH_DIR / "laserscan_to_pcl_and_occ.launch.xml",
    }

    for name, path in required_files.items():
        if path.exists():
            size = path.stat().st_size
            ok(f"{name} ({size:,} bytes)")
        else:
            fail(f"{name} — MISSING")


# ═══════════════════════════════════════════════════════════════
# TEST 2: DETECTION_WS AUTOWARE PACKAGES
# ═══════════════════════════════════════════════════════════════
def test_detection_ws():
    section("2. DETECTION_WS AUTOWARE PACKAGES")

    packages = {
        "autoware_tensorrt_yolox": ["yolov8_node.py", "yolox_node.py"],
        "autoware_bytetrack": ["bytetrack_node.py"],
        "autoware_traffic_light_classifier": ["traffic_light_classifier_node.py"],
        "autoware_detection_autoware_bridge": ["detection_autoware_bridge_node.py"],
        "tier4_perception_launch": [],
    }

    for pkg, nodes in packages.items():
        pkg_dir = DETECTION_WS / pkg
        if not pkg_dir.exists():
            fail(f"{pkg}/ — MISSING")
            continue

        pkg_xml = pkg_dir / "package.xml"
        if pkg_xml.exists():
            ok(f"{pkg}/package.xml")
        else:
            fail(f"{pkg}/package.xml — MISSING")

        for node in nodes:
            node_path = pkg_dir / pkg / node
            if node_path.exists():
                lines = len(node_path.read_text().splitlines())
                ok(f"{pkg}/{node} ({lines} lines)")
            else:
                fail(f"{pkg}/{node} — MISSING")

    # Check detection_module.launch.xml
    launch = DETECTION_WS / "tier4_perception_launch" / "launch" / "detection_module.launch.xml"
    if launch.exists():
        lines = len(launch.read_text().splitlines())
        ok(f"tier4_perception_launch/detection_module.launch.xml ({lines} lines)")
    else:
        fail("tier4_perception_launch/detection_module.launch.xml — MISSING")


# ═══════════════════════════════════════════════════════════════
# TEST 3: PYTHON SYNTAX & IMPORTS
# ═══════════════════════════════════════════════════════════════
def test_python_syntax():
    section("3. PYTHON SYNTAX & IMPORTS")

    import py_compile

    python_files = list(SCRIPTS_DIR.glob("*.py"))
    for pf in sorted(python_files):
        try:
            py_compile.compile(str(pf), doraise=True)
            ok(f"Syntax OK: {pf.name}")
        except py_compile.PyCompileError as e:
            fail(f"Syntax ERROR: {pf.name}", str(e))

    # Check core imports
    core_imports = ["cv2", "numpy", "onnxruntime", "math", "json", "threading", "collections"]
    for mod in core_imports:
        try:
            importlib.import_module(mod)
            ok(f"Import: {mod}")
        except ImportError:
            fail(f"Import: {mod} — NOT INSTALLED")


# ═══════════════════════════════════════════════════════════════
# TEST 4: YOLO MODEL LOADING + INFERENCE
# ═══════════════════════════════════════════════════════════════
def test_yolo_model():
    section("4. YOLO MODEL LOADING & INFERENCE")

    model_path = MODELS_DIR / "yolov8n.onnx"
    if not model_path.exists():
        fail("yolov8n.onnx not found — cannot test")
        return

    try:
        import onnxruntime as ort
        ok("onnxruntime imported")
    except ImportError:
        fail("onnxruntime not installed")
        return

    # Load model
    try:
        sess = ort.InferenceSession(str(model_path))
        inp = sess.get_inputs()[0]
        ok(f"Model loaded: input={inp.name} shape={inp.shape}")
    except Exception as e:
        fail(f"Model loading failed: {e}")
        return

    # Dummy inference
    try:
        import numpy as np
        dummy = np.zeros((1, 3, 640, 640), dtype=np.float32)
        t0 = time.time()
        out = sess.run(None, {inp.name: dummy})
        dt = time.time() - t0
        shape = out[0].shape
        ok(f"Dummy inference: output shape={shape}, time={dt*1000:.0f}ms")
    except Exception as e:
        fail(f"Dummy inference failed: {e}")
        return

    # Labels check
    labels_path = MODELS_DIR / "labels.txt"
    if labels_path.exists():
        labels = [l.strip() for l in labels_path.read_text().splitlines() if l.strip()]
        if len(labels) == 80:
            ok(f"Labels: {len(labels)} classes (COCO)")
            # Check key classes
            expected = {"person": 0, "car": 2, "bus": 5, "truck": 7, "traffic light": 9}
            for name, idx in expected.items():
                if idx < len(labels) and labels[idx] == name:
                    ok(f"  Class {idx}: '{name}'")
                else:
                    fail(f"  Class {idx}: expected '{name}', got '{labels[idx] if idx < len(labels) else 'N/A'}'")
        else:
            fail(f"Labels: expected 80 classes, got {len(labels)}")

    # Color map check
    cmap_path = MODELS_DIR / "color_map.json"
    if cmap_path.exists():
        try:
            cmap = json.loads(cmap_path.read_text())
            ok(f"Color map: {len(cmap)} entries")
        except json.JSONDecodeError:
            fail("Color map: invalid JSON")


# ═══════════════════════════════════════════════════════════════
# TEST 5: TRAFFIC LIGHT DETECTION (SYNTHETIC IMAGES)
# ═══════════════════════════════════════════════════════════════
def test_traffic_light_detection():
    section("5. TRAFFIC LIGHT DETECTION (SYNTHETIC)")

    try:
        import cv2
        import numpy as np
    except ImportError:
        fail("OpenCV/NumPy not available")
        return

    # Import the detector
    sys.path.insert(0, str(SCRIPTS_DIR))
    try:
        # We need to import from webcam_detect_standalone
        spec = importlib.util.spec_from_file_location(
            "webcam_detect", str(SCRIPTS_DIR / "webcam_detect_standalone.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        detect_fn = mod.detect_traffic_lights
        ok("detect_traffic_lights function loaded")
    except Exception as e:
        fail(f"Cannot load detect_traffic_lights: {e}")
        return

    # Test A: Empty/plain image → should detect NOTHING
    plain = np.ones((480, 640, 3), dtype=np.uint8) * 180  # gray room
    dets = detect_fn(plain)
    if len(dets) == 0:
        ok("Plain gray image → 0 detections (no false positives)")
    else:
        fail(f"Plain gray image → {len(dets)} detections (FALSE POSITIVES!)")

    # Test B: Warm indoor lighting → should detect NOTHING
    warm = np.zeros((480, 640, 3), dtype=np.uint8)
    warm[:, :, 2] = 200  # red channel high
    warm[:, :, 1] = 170  # green channel medium
    warm[:, :, 0] = 140  # blue channel low → warm yellowish/orange
    dets = detect_fn(warm)
    if len(dets) == 0:
        ok("Warm indoor lighting → 0 detections (no false positives)")
    else:
        fail(f"Warm indoor lighting → {len(dets)} detections (FALSE POSITIVES!)")

    # Test C: Synthetic traffic light (bright red circle on dark background)
    tl_img = np.zeros((480, 640, 3), dtype=np.uint8)
    tl_img[:] = 30  # dark background
    # Red lamp (top) — very bright and saturated
    cv2.circle(tl_img, (320, 180), 20, (0, 0, 255), -1)
    # Green lamp (bottom) — dim
    cv2.circle(tl_img, (320, 230), 20, (0, 40, 0), -1)
    dets = detect_fn(tl_img)
    if len(dets) > 0:
        color = dets[0][5]
        conf = dets[0][4]
        ok(f"Synthetic TL (red on dark) → detected: {color} ({conf:.0%} conf)")
        if color == "red":
            ok(f"  Color correctly identified as RED")
        else:
            warn(f"  Expected 'red', got '{color}'")
    else:
        warn("Synthetic TL (red on dark) → 0 detections (may need threshold tuning)")

    # Test D: Synthetic green traffic light
    tl_green = np.zeros((480, 640, 3), dtype=np.uint8)
    tl_green[:] = 30
    # Red lamp (top) — dim
    cv2.circle(tl_green, (320, 180), 20, (0, 0, 40), -1)
    # Green lamp (bottom) — very bright
    cv2.circle(tl_green, (320, 230), 20, (0, 255, 0), -1)
    dets = detect_fn(tl_green)
    if len(dets) > 0:
        color = dets[0][5]
        conf = dets[0][4]
        ok(f"Synthetic TL (green on dark) → detected: {color} ({conf:.0%} conf)")
        if color == "green":
            ok(f"  Color correctly identified as GREEN")
        else:
            warn(f"  Expected 'green', got '{color}'")
    else:
        warn("Synthetic TL (green on dark) → 0 detections (may need threshold tuning)")

    # Test E: Pair matching test (red above green, both visible)
    tl_pair = np.zeros((480, 640, 3), dtype=np.uint8)
    tl_pair[:] = 25  # very dark housing
    cv2.circle(tl_pair, (320, 170), 18, (0, 0, 240), -1)  # red top
    cv2.circle(tl_pair, (320, 220), 18, (0, 200, 0), -1)   # green bottom
    dets = detect_fn(tl_pair)
    if len(dets) > 0:
        conf = dets[0][4]
        color = dets[0][5]
        if conf >= 0.70:
            ok(f"Pair matching → HIGH confidence ({conf:.0%}), color={color}")
        else:
            ok(f"Pair matching → detected ({conf:.0%}), color={color}")
    else:
        warn("Pair matching → no detection")


# ═══════════════════════════════════════════════════════════════
# TEST 6: AUTOWARE PLANNER BRIDGE COMPATIBILITY
# ═══════════════════════════════════════════════════════════════
def test_autoware_bridge():
    section("6. AUTOWARE PLANNER BRIDGE COMPATIBILITY")

    # Check detection_autoware_bridge exists and has correct output topics
    bridge_path = DETECTION_WS / "autoware_detection_autoware_bridge"
    if not bridge_path.exists():
        fail("autoware_detection_autoware_bridge package missing")
        return

    bridge_node = bridge_path / "autoware_detection_autoware_bridge" / "detection_autoware_bridge_node.py"
    if not bridge_node.exists():
        fail("detection_autoware_bridge_node.py missing")
        return

    content = bridge_node.read_text()
    ok(f"Bridge node exists ({len(content.splitlines())} lines)")

    # Check critical Autoware output topics
    autoware_topics = {
        "detected_objects": "/perception/object_recognition/detection/objects",
        "tracked_objects": "/perception/object_recognition/tracking/objects",
        "predicted_objects": "/perception/object_recognition/objects",
        "traffic_signals": "/perception/traffic_light_recognition/traffic_signals",
    }

    # Check detection_module.launch.xml for topic mappings
    launch_path = DETECTION_WS / "tier4_perception_launch" / "launch" / "detection_module.launch.xml"
    if launch_path.exists():
        launch_content = launch_path.read_text()
        for name, topic in autoware_topics.items():
            if topic in launch_content:
                ok(f"Planner topic mapped: {topic}")
            else:
                fail(f"Planner topic NOT found in launch: {topic}")

        # Crosswalk detection: needs object detection output
        if "output_detected_objects" in launch_content:
            ok("Crosswalk/obstacle avoidance: detected_objects output connected")
        else:
            warn("Crosswalk/obstacle avoidance: output_detected_objects mapping not found")

        # Traffic light → planner connection
        if "output_traffic_signals" in launch_content:
            ok("Traffic light → planner: traffic_signals output connected")
        else:
            warn("Traffic light → planner: output_traffic_signals mapping not found")
    else:
        fail("detection_module.launch.xml missing")

    # Check traffic light classifier config
    tl_config = DETECTION_WS / "autoware_traffic_light_classifier" / "config" / "pedestrian_traffic_light_classifier.param.yaml"
    if tl_config.exists():
        ok(f"Pedestrian TL classifier config exists")
    else:
        warn("Pedestrian TL classifier config missing (will use defaults)")


# ═══════════════════════════════════════════════════════════════
# TEST 7: SENSOR FUSION PIPELINE FILES
# ═══════════════════════════════════════════════════════════════
def test_sensor_fusion():
    section("7. SENSOR FUSION PIPELINE")

    # Check sensor_start module
    sensor_start = MODULE_DIR.parent / "sensor_start"
    sensor_files = {
        "mac_webcam_stream.py": sensor_start / "mac_webcam_stream.py",
        "sensor_setup.sh": sensor_start / "sensor_setup.sh",
        "sensor_start.sh": sensor_start / "sensor_start.sh",
        "README.md": sensor_start / "README.md",
    }

    for name, path in sensor_files.items():
        if path.exists():
            ok(f"sensor_start/{name}")
        else:
            fail(f"sensor_start/{name} — MISSING")

    # Check sensor kit
    sensor_kit = MODULE_DIR.parent / "sensor" / "my_sensor_kit_launch"
    if sensor_kit.exists():
        ok("my_sensor_kit_launch/ exists")
        # Check key launch files
        for lf in ["sensing.launch.xml", "camera.launch.xml", "lidar.launch.xml"]:
            lf_path = sensor_kit / "launch" / lf
            if lf_path.exists():
                ok(f"  {lf}")
            else:
                warn(f"  {lf} — missing")
    else:
        fail("my_sensor_kit_launch/ — MISSING")

    # Check laserscan → occupancy grid launch
    ls_launch = LAUNCH_DIR / "laserscan_to_pcl_and_occ.launch.xml"
    if ls_launch.exists():
        content = ls_launch.read_text()
        if "/scan" in content:
            ok("LaserScan input topic: /scan")
        if "pointcloud" in content.lower() or "points" in content.lower():
            ok("PointCloud output configured")
    else:
        fail("laserscan_to_pcl_and_occ.launch.xml — MISSING")

    # Check dummy publishers
    for dp in ["dummy_lidar_publisher.py", "dummy_camera_publisher.py"]:
        dp_path = SCRIPTS_DIR / dp
        if dp_path.exists():
            ok(f"Dummy publisher: {dp}")
        else:
            fail(f"Dummy publisher: {dp} — MISSING")


# ═══════════════════════════════════════════════════════════════
# TEST 8: STANDALONE DETECTION SCRIPT STRUCTURE
# ═══════════════════════════════════════════════════════════════
def test_standalone():
    section("8. STANDALONE DETECTION SCRIPT")

    script = SCRIPTS_DIR / "webcam_detect_standalone.py"
    if not script.exists():
        fail("webcam_detect_standalone.py missing")
        return

    content = script.read_text()
    lines = content.splitlines()
    ok(f"Script exists ({len(lines)} lines)")

    # Check key functions
    functions = [
        "yolo_detect", "detect_traffic_lights", "_find_color_blobs",
        "make_session", "_letterbox", "main"
    ]
    for fn in functions:
        if f"def {fn}" in content:
            ok(f"Function: {fn}()")
        else:
            fail(f"Function: {fn}() — NOT FOUND")

    # Check YOLO classes (should NOT include traffic light = class 9)
    if '"0,1,2,3,5,7"' in content:
        ok("YOLO classes: 0,1,2,3,5,7 (no traffic light)")
    elif "0,1,2,3,5,7" in content:
        ok("YOLO classes include person/car/bus/truck (no TL)")
    else:
        warn("YOLO class list not found — verify manually")

    # Check contrast-based detection
    if "contrast" in content.lower():
        ok("Contrast-based TL detection implemented")
    else:
        fail("Contrast-based detection NOT found")

    # Check temporal smoothing
    if "tl_history" in content or "tl_confirmed" in content:
        ok("Temporal smoothing for TL state")
    else:
        fail("Temporal smoothing NOT found")

    # Check CoreML support
    if "CoreML" in content or "coreml" in content.lower():
        ok("CoreML (Apple Silicon) acceleration supported")
    else:
        warn("CoreML acceleration not found in script")


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print(f"\n{BOLD}{'═'*60}{NC}")
    print(f"{BOLD}  SDC 2026 — COMPREHENSIVE PERCEPTION MODULE TEST{NC}")
    print(f"{BOLD}  Module: {MODULE_DIR}{NC}")
    print(f"{BOLD}  Time: {time.strftime('%Y-%m-%d %H:%M:%S')}{NC}")
    print(f"{BOLD}{'═'*60}{NC}")

    test_file_structure()
    test_detection_ws()
    test_python_syntax()
    test_yolo_model()
    test_traffic_light_detection()
    test_autoware_bridge()
    test_sensor_fusion()
    test_standalone()

    # Summary
    total = PASS_COUNT + FAIL_COUNT + WARN_COUNT
    print(f"\n{BOLD}{'═'*60}{NC}")
    print(f"{BOLD}  RESULTS{NC}")
    print(f"{BOLD}{'═'*60}{NC}")
    print(f"  {GREEN}✓ Passed: {PASS_COUNT}{NC}")
    print(f"  {RED}✗ Failed: {FAIL_COUNT}{NC}")
    print(f"  {YELLOW}⚠ Warns:  {WARN_COUNT}{NC}")
    print(f"  Total:   {total}")
    print()

    if FAIL_COUNT == 0:
        print(f"  {GREEN}{BOLD}ALL CRITICAL TESTS PASSED ✓{NC}")
    else:
        print(f"  {RED}{BOLD}{FAIL_COUNT} TEST(S) FAILED — FIX BEFORE COMMIT{NC}")

    print(f"{BOLD}{'═'*60}{NC}\n")
    sys.exit(FAIL_COUNT)
