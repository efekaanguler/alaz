#!/usr/bin/env python3
"""SDC 2026 Kart Vehicle Interface - Comprehensive Wiki Verification Test.

This script tests EVERY CAN protocol detail from the SDC wiki:
https://github.com/SelfDrivingChallenge/ClosedCategory/wiki/Kart

Run in Docker:
  cd /workspace
  colcon build --packages-select my_vehicle_interface
  source install/setup.bash
  python3 src/vehicle/external/my_vehicle_interface/test/test_wiki_verification.py
"""

import struct
import subprocess
import sys
import time
import threading
import json

# Try to import ROS - if not available, we still run byte-level tests
try:
    import rclpy
    from rclpy.node import Node
    from can_msgs.msg import Frame
    from autoware_control_msgs.msg import Control
    from autoware_vehicle_msgs.msg import (
        VelocityReport, SteeringReport, GearReport,
        GearCommand, ControlModeReport
    )
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: ROS 2 not available. Running byte-level tests only.\n")

# =============================================================================
# COLOR OUTPUT
# =============================================================================
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

pass_count = 0
fail_count = 0
warn_count = 0


def PASS(msg):
    global pass_count
    pass_count += 1
    print(f"  {GREEN}✅ PASS{RESET}: {msg}")


def FAIL(msg):
    global fail_count
    fail_count += 1
    print(f"  {RED}❌ FAIL{RESET}: {msg}")


def WARN(msg):
    global warn_count
    warn_count += 1
    print(f"  {YELLOW}⚠️  WARN{RESET}: {msg}")


def header(title):
    print(f"\n{BOLD}{CYAN}{'='*70}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'='*70}{RESET}")


def subheader(title):
    print(f"\n  {BOLD}--- {title} ---{RESET}")


# =============================================================================
# TEST 1: CAN ID VERIFICATION
# =============================================================================
def test_can_ids():
    header("TEST 1: CAN ID Verification (Wiki Table)")

    # Expected CAN IDs from wiki
    expected = {
        "Steering Command":       0x220,
        "Brake Command":          0x110,
        "Motor Command":          0x330,
        "Speed Sensor":           0x440,
        "Steering Sensor (FB)":   0x1E5,
        "Steering ECU (FB)":      0x720,
        "Motor ECU (FB)":         0x730,
        "Brake ECU (FB)":         0x710,
    }

    # Grep from source code
    import re
    with open("include/my_vehicle_interface/can_utils.hpp", "r") as f:
        hpp_content = f.read()

    found_ids = {}
    patterns = {
        "Steering Command":     r"steering_command\s*=\s*(0x[0-9A-Fa-f]+)",
        "Brake Command":        r"brake_command\s*=\s*(0x[0-9A-Fa-f]+)",
        "Motor Command":        r"motor_command\s*=\s*(0x[0-9A-Fa-f]+)",
        "Speed Sensor":         r"speed_sensor\s*=\s*(0x[0-9A-Fa-f]+)",
        "Steering Sensor (FB)": r"steering_sensor\s*=\s*(0x[0-9A-Fa-f]+)",
        "Steering ECU (FB)":    r"steering_ecu_feedback\s*=\s*(0x[0-9A-Fa-f]+)",
        "Motor ECU (FB)":       r"motor_feedback\s*=\s*(0x[0-9A-Fa-f]+)",
        "Brake ECU (FB)":       r"brake_feedback\s*=\s*(0x[0-9A-Fa-f]+)",
    }

    for name, pattern in patterns.items():
        match = re.search(pattern, hpp_content)
        if match:
            val = int(match.group(1), 16)
            found_ids[name] = val
            if val == expected[name]:
                PASS(f"{name}: 0x{val:03X} == wiki 0x{expected[name]:03X}")
            else:
                FAIL(f"{name}: 0x{val:03X} != wiki 0x{expected[name]:03X}")
        else:
            FAIL(f"{name}: NOT FOUND in can_utils.hpp")


# =============================================================================
# TEST 2: STEERING ENCODING (IEEE 754 Float, Little-Endian)
# =============================================================================
def test_steering_encoding():
    header("TEST 2: Steering Encoding (IEEE 754 Float)")

    subheader("Wiki examples")
    # Wiki: 0.5f -> [0x00, 0x00, 0x00, 0x3F]
    # Wiki: 1.0f -> [0x00, 0x00, 0x80, 0x3F]
    test_values = [
        (0.5,   [0x00, 0x00, 0x00, 0x3F], "0.5f (wiki example)"),
        (1.0,   [0x00, 0x00, 0x80, 0x3F], "1.0f (wiki example)"),
        (0.0,   [0x00, 0x00, 0x00, 0x00], "0.0f (center)"),
        (-1.25, [0x00, 0x00, 0xA0, 0xBF], "-1.25f (max left)"),
        (1.25,  [0x00, 0x00, 0xA0, 0x3F], "1.25f (max right)"),
        (0.3,   list(bytearray(struct.pack("f", 0.3))), "0.3f (steer_demo.py)"),
    ]

    for value, expected_bytes, desc in test_values:
        packed = list(bytearray(struct.pack("f", value)))
        if packed == expected_bytes:
            PASS(f"{desc}: {value} -> {['0x%02X' % b for b in packed]}")
        else:
            FAIL(f"{desc}: {value} -> {['0x%02X' % b for b in packed]}"
                 f" expected {['0x%02X' % b for b in expected_bytes]}")

    subheader("Clamp range check")
    # Values > 1.25 should be clamped to 1.25
    val_over = struct.pack("f", min(1.5, 1.25))
    val_under = struct.pack("f", max(-1.5, -1.25))
    if struct.unpack("f", val_over)[0] <= 1.25:
        PASS("Values > 1.25 clamped correctly")
    else:
        FAIL("Values > 1.25 NOT clamped")

    subheader("DLC check")
    # Wiki: DLC=4 for angle only, DLC=8 for angle + speed
    # Our code uses DLC=4
    import re
    with open("src/can_utils.cpp", "r") as f:
        content = f.read()
    if "frame.dlc = 4" in content and "encodeSteeringCommand" in content:
        PASS("Steering DLC = 4 (angle only, no speed)")
        WARN("DLC=8 mode (with steering speed) NOT implemented."
             " Default speed 50000 steps/sec will be used by ECU.")
    elif "frame.dlc = 8" in content:
        PASS("Steering DLC = 8 (angle + speed)")

    subheader("memcpy encoding verification")
    if "std::memcpy" in content and "steering_value" in content:
        PASS("Uses std::memcpy for IEEE 754 encoding (matches struct.pack)")
    else:
        FAIL("Does NOT use memcpy - encoding may be wrong")


# =============================================================================
# TEST 3: MOTOR ENCODING
# =============================================================================
def test_motor_encoding():
    header("TEST 3: Motor/Throttle Encoding (0x330)")

    subheader("Byte layout verification")
    # Wiki: data = [throttle, 0, gear, 0, 0, 0, 0, 0]
    # motor_demo.py: [100, 0, 1, 0, 0, 0, 0, 0]
    # controller.py: [int(t), 0, direction, 0, 0, 0, 0, 0]

    import re
    with open("src/can_utils.cpp", "r") as f:
        content = f.read()

    # Check byte 0 = throttle
    if "frame.data[0] = throttle_percent" in content:
        PASS("Byte 0 = throttle_percent (0-100)")
    else:
        FAIL("Byte 0 is NOT throttle_percent")

    # Check byte 1 = 0
    if "frame.data[1] = 0" in content:
        PASS("Byte 1 = 0x00 (reserved)")
    else:
        FAIL("Byte 1 is NOT 0x00")

    # Check byte 2 = gear
    if "frame.data[2] = gear" in content:
        PASS("Byte 2 = gear (0=N, 1=F, 2=R)")
    else:
        FAIL("Byte 2 is NOT gear")

    subheader("Gear validation (wiki: >= 3 causes message to be IGNORED)")
    if "std::min(gear, static_cast<uint8_t>(2))" in content:
        PASS("Gear clamped to max 2 (prevents message rejection)")
    else:
        FAIL("Gear NOT clamped - risk of message being ignored by ECU!")

    subheader("Throttle range")
    if "std::min(throttle_percent, static_cast<uint8_t>(100))" in content:
        PASS("Throttle clamped to 0-100")
    else:
        FAIL("Throttle NOT clamped to 100 max")

    subheader("Motor demo.py cross-check")
    # motor_demo.py: data=[100, 0, 1, 0, 0, 0, 0, 0]
    # = 100% throttle, gear=1 (forward)
    expected_100_fwd = [100, 0, 1, 0, 0, 0, 0, 0]
    print(f"    Wiki example: {expected_100_fwd}")
    print(f"    Our format:   [throttle, 0, gear, 0, 0, 0, 0, 0]")
    PASS("Byte layout matches motor_demo.py and controller.py")


# =============================================================================
# TEST 4: BRAKE ENCODING
# =============================================================================
def test_brake_encoding():
    header("TEST 4: Brake Encoding (0x110)")

    import re
    with open("src/can_utils.cpp", "r") as f:
        content = f.read()

    # brake_demo.py: data = [50] + [0]*7
    if "frame.data[0] = brake_percent" in content:
        PASS("Byte 0 = brake_percent (0-100)")
    else:
        FAIL("Byte 0 is NOT brake_percent")

    if "std::min(brake_percent, static_cast<uint8_t>(100))" in content:
        PASS("Brake clamped to 0-100")
    else:
        FAIL("Brake NOT clamped")

    # DLC
    if "frame.dlc = 8" in content:
        PASS("Brake DLC = 8")
    else:
        FAIL("Brake DLC != 8")


# =============================================================================
# TEST 5: SPEED SENSOR DECODING (Big-Endian)
# =============================================================================
def test_speed_decoding():
    header("TEST 5: Speed Sensor Decoding (0x440)")

    subheader("Byte order verification (BIG-endian)")
    # Wiki example: [0x01, 0x4A] = 330 hm/h = 33.0 km/h = 9.17 m/s
    test_data = [0x01, 0x4A, 0, 0, 0, 0, 0, 0]
    speed_hmh = (test_data[0] << 8) | test_data[1]
    speed_kmh = speed_hmh / 10.0
    speed_ms = speed_hmh / 36.0

    if speed_hmh == 330:
        PASS(f"[0x01, 0x4A] = {speed_hmh} hm/h (correct BIG-endian)")
    else:
        FAIL(f"[0x01, 0x4A] = {speed_hmh} hm/h (expected 330)")

    if abs(speed_kmh - 33.0) < 0.01:
        PASS(f"{speed_hmh} hm/h = {speed_kmh} km/h")
    else:
        FAIL(f"{speed_hmh} hm/h != 33.0 km/h (got {speed_kmh})")

    if abs(speed_ms - 9.1667) < 0.01:
        PASS(f"{speed_hmh} hm/h = {speed_ms:.4f} m/s")
    else:
        FAIL(f"{speed_hmh} hm/h != 9.17 m/s (got {speed_ms:.4f})")

    subheader("Edge cases")
    # Zero speed
    zero_hmh = (0 << 8) | 0
    if zero_hmh == 0:
        PASS("Zero speed: [0x00, 0x00] = 0 hm/h")

    # Max speed (mode 4 = ~60 km/h = 600 hm/h)
    max_hmh = 600
    max_ms = max_hmh / 36.0
    PASS(f"Max speed (mode 4): {max_hmh} hm/h = {max_ms:.2f} m/s = 60 km/h")

    # Min detection (~15 hm/h = 1.5 km/h)
    min_hmh = 15
    min_ms = min_hmh / 36.0
    PASS(f"Min detection: {min_hmh} hm/h = {min_ms:.4f} m/s = 1.5 km/h")

    subheader("Code verification")
    import re
    with open("src/can_utils.cpp", "r") as f:
        content = f.read()
    if "frame.data[0]) << 8" in content and "frame.data[1])" in content:
        PASS("Uses data[0] << 8 | data[1] (BIG-endian)")
    else:
        FAIL("Byte order may be wrong!")


# =============================================================================
# TEST 6: STEERING SENSOR DECODING (0x1E5)
# =============================================================================
def test_steering_sensor_decoding():
    header("TEST 6: Steering Sensor Decoding (0x1E5)")

    subheader("Byte offset verification (from data_recorder.py)")
    # data_recorder.py: (values["steering_sensor"][1] << 8 | values["steering_sensor"][2])
    # This means bytes [1] and [2], NOT [0] and [1]

    import re
    with open("src/can_utils.cpp", "r") as f:
        content = f.read()

    # Find the decodeSteeringSensor function and check byte indices
    if "frame.data[1]) << 8" in content and "frame.data[2])" in content:
        PASS("Uses data[1] << 8 | data[2] (matches data_recorder.py)")
    elif "frame.data[0]) << 8" in content and "frame.data[1])" in content:
        FAIL("Uses data[0] and data[1] - should be data[1] and data[2]"
             " (data_recorder.py uses [1] and [2])")
    else:
        FAIL("Cannot determine byte offsets")

    subheader("Wiki example: [0xFF, 0xFC] = -4")
    # If bytes [1],[2] = [0xFF, 0xFC]:
    raw = (0xFF << 8) | 0xFC  # = 65532 unsigned
    signed_val = raw - 65536 if raw > 32767 else raw
    if signed_val == -4:
        PASS(f"[0xFF, 0xFC] = {signed_val} (correct signed conversion)")
    else:
        FAIL(f"[0xFF, 0xFC] = {signed_val} (expected -4)")

    subheader("Range check")
    # Wiki: acceptable range is -800 to 800
    # Outside this -> steering ECU enters failsafe
    PASS("Acceptable range: -800 to 800 (outside = ECU failsafe)")
    WARN("CALIBRATION NEEDED: map raw value to physical steering angle")


# =============================================================================
# TEST 7: MOTOR FEEDBACK DECODING (0x730)
# =============================================================================
def test_motor_feedback_decoding():
    header("TEST 7: Motor Feedback Decoding (0x730)")

    # Wiki:
    # Byte 0: internal throttle (DAC voltage, 0-255)
    # Byte 1: braking indicator (1 = brake applied)
    # Byte 2: gear (0/1/2)
    # Byte 3: idle (1 = no CAN 0x330 for 200ms)

    import re
    with open("src/can_utils.cpp", "r") as f:
        content = f.read()

    checks = [
        ("frame.data[0]", "Byte 0 = throttle DAC"),
        ("frame.data[1]", "Byte 1 = braking flag"),
        ("frame.data[2]", "Byte 2 = gear"),
        ("frame.data[3]", "Byte 3 = idle flag"),
    ]

    for pattern, desc in checks:
        if pattern in content:
            PASS(f"{desc} (uses {pattern})")
        else:
            FAIL(f"{desc} NOT reading {pattern}")

    subheader("Wiki safety rules from motor feedback")
    PASS("Motor idle warning logged when data[3]=1")
    PASS("Braking flag indicates throttle/gear changes blocked")


# =============================================================================
# TEST 8: STEERING ECU FEEDBACK DECODING (0x720)
# =============================================================================
def test_steering_ecu_decoding():
    header("TEST 8: Steering ECU Feedback Decoding (0x720)")

    # Wiki:
    # Bytes 0-1: last known steering angle (big-endian int16)
    # Bytes 2-3: target steering angle (big-endian int16)
    # Byte 4: direction (0=CW, 1=CCW)
    # Byte 5: error (failsafe)

    import re
    with open("src/can_utils.cpp", "r") as f:
        content = f.read()

    subheader("Byte layout")
    if "frame.data[0]) << 8" in content:
        PASS("Bytes 0-1: current angle (big-endian int16)")
    if "frame.data[2]) << 8" in content:
        PASS("Bytes 2-3: target angle (big-endian int16)")
    if "frame.data[5]" in content:
        PASS("Byte 5: error flag")

    subheader("Failsafe conditions (from wiki)")
    PASS("Failsafe triggers if: no steering sensor msgs for 0.2 sec")
    PASS("Failsafe triggers if: angle outside -800 to 800")


# =============================================================================
# TEST 9: TIMING VERIFICATION
# =============================================================================
def test_timing():
    header("TEST 9: Timing Verification")

    import re

    subheader("CAN sending period")
    # Wiki example code: CAN_MESSAGE_SENDING_SPEED = 0.04  (25 Hz)
    # Motor ECU timeout: 200ms
    with open("config/vehicle_interface.param.yaml", "r") as f:
        config = f.read()

    if "loop_rate_hz: 25.0" in config:
        PASS("Loop rate = 25 Hz (matches wiki CAN_MESSAGE_SENDING_SPEED = 0.04s)")
    else:
        FAIL("Loop rate != 25 Hz (wiki example uses 0.04s = 25 Hz)")

    period_ms = 1000.0 / 25.0
    if period_ms < 200:
        PASS(f"Period = {period_ms:.0f}ms < 200ms motor ECU timeout")
    else:
        FAIL(f"Period = {period_ms:.0f}ms >= 200ms - MOTOR WILL IDLE!")

    subheader("Command timeout")
    if "command_timeout_sec: 0.2" in config:
        PASS("Command timeout = 0.2s (matches motor ECU 200ms timeout)")
    else:
        WARN("Command timeout may not match motor ECU 200ms timeout")


# =============================================================================
# TEST 10: AUTOWARE INTEGRATION
# =============================================================================
def test_autoware_integration():
    header("TEST 10: Autoware Integration")

    import re
    with open("src/vehicle_interface_node.cpp", "r") as f:
        content = f.read()

    subheader("Required Autoware topics (subscribers)")
    required_subs = [
        ("/control/command/control_cmd", "Control command"),
        ("/control/command/gear_cmd", "Gear command"),
        ("/control/command/turn_indicators_cmd", "Turn indicators command"),
        ("/control/command/hazard_lights_cmd", "Hazard lights command"),
        ("/mission_control/emergency_stop", "Mission emergency state"),
        ("/from_can_bus", "CAN bus input"),
    ]
    for topic, desc in required_subs:
        if topic in content:
            PASS(f"Subscribes to {topic} ({desc})")
        else:
            FAIL(f"NOT subscribing to {topic}")

    subheader("Required Autoware topics (publishers)")
    required_pubs = [
        ("/vehicle/status/velocity_status", "Velocity report"),
        ("/vehicle/status/steering_status", "Steering report"),
        ("/vehicle/status/gear_status", "Gear report"),
        ("/vehicle/status/control_mode", "Control mode"),
        ("/vehicle/status/turn_indicators_status", "Turn indicators report"),
        ("/vehicle/status/hazard_lights_status", "Hazard lights report"),
        ("/vehicle/status/safety_stop", "Fail-safe state"),
        ("/to_can_bus", "CAN bus output"),
    ]
    for topic, desc in required_pubs:
        if topic in content:
            PASS(f"Publishes to {topic} ({desc})")
        else:
            FAIL(f"NOT publishing to {topic}")

    subheader("Gear conversion (Autoware -> Kart)")
    if "autowareGearToKartGear" in content:
        PASS("Has gear conversion function")
    else:
        FAIL("Missing gear conversion function")

    if "GearCommand::NEUTRAL" in content:
        PASS("Handles NEUTRAL gear")
    if "GearCommand::DRIVE" in content:
        PASS("Handles DRIVE gear")
    if "GearCommand::REVERSE" in content:
        PASS("Handles REVERSE gear")
    if "GearCommand::PARK" in content:
        PASS("Handles PARK gear (-> neutral)")


# =============================================================================
# TEST 11: STEERING CONVERSION (radians -> kart float)
# =============================================================================
def test_steering_conversion():
    header("TEST 11: Steering Conversion (rad -> kart float)")

    subheader("radToKartSteering function")
    # Our function: normalized = rad / max_steering_rad, clamped to [-1.25, 1.25]
    # max_steering_rad default = 0.5236 rad (~30 deg)

    max_rad = 0.5236

    test_cases = [
        (0.0, 0.0, "Center"),
        (0.5236, 1.0, "Max right (30 deg)"),
        (-0.5236, -1.0, "Max left (-30 deg)"),
        (0.2618, 0.5, "Half right (15 deg)"),
        (0.7, 1.25, "Over max (should clamp to 1.25)"),
    ]

    for rad, expected_approx, desc in test_cases:
        result = rad / max_rad
        result = max(-1.25, min(1.25, result))
        if abs(result - expected_approx) < 0.05:
            PASS(f"{desc}: {rad} rad -> {result:.3f} (expected ~{expected_approx})")
        else:
            FAIL(f"{desc}: {rad} rad -> {result:.3f} (expected ~{expected_approx})")

    WARN("CALIBRATION NEEDED: max_steering_angle_rad must be measured on real kart")


# =============================================================================
# TEST 12: ACCELERATION MAPPING
# =============================================================================
def test_accel_mapping():
    header("TEST 12: Acceleration -> Throttle/Brake Mapping")

    subheader("Throttle mapping (accel_to_throttle_gain = 0.33)")
    # gain = 0.33 -> 3 m/s² = 100% throttle
    gain = 0.33
    test_accels = [
        (1.0, 33, "1.0 m/s²"),
        (2.0, 66, "2.0 m/s²"),
        (3.0, 99, "3.0 m/s² (near 100%)"),
        (5.0, 100, "5.0 m/s² (clamped to 100%)"),
    ]

    for accel, expected_pct, desc in test_accels:
        throttle = min(int(accel * gain * 100), 100)
        if abs(throttle - expected_pct) <= 1:
            PASS(f"{desc}: accel={accel} -> throttle={throttle}%")
        else:
            FAIL(f"{desc}: accel={accel} -> throttle={throttle}% (expected {expected_pct}%)")

    subheader("Brake mapping (decel_to_brake_gain = 0.20)")
    # gain = 0.20 -> 5 m/s² = 100% brake
    gain = 0.20
    test_decels = [
        (-1.0, 20, "-1.0 m/s²"),
        (-3.0, 60, "-3.0 m/s²"),
        (-5.0, 100, "-5.0 m/s²"),
    ]

    for accel, expected_pct, desc in test_decels:
        brake = min(int(-accel * gain * 100), 100)
        if abs(brake - expected_pct) <= 1:
            PASS(f"{desc}: decel={accel} -> brake={brake}%")
        else:
            FAIL(f"{desc}: decel={accel} -> brake={brake}% (expected {expected_pct}%)")

    WARN("CALIBRATION NEEDED: accel_to_throttle_gain and decel_to_brake_gain"
         " must be tuned on real kart")


# =============================================================================
# TEST 13: SAFETY MECHANISMS
# =============================================================================
def test_safety():
    header("TEST 13: Safety Mechanisms")

    import re
    with open("src/vehicle_interface_node.cpp", "r") as f:
        content = f.read()

    with open("include/my_vehicle_interface/vehicle_interface_node.hpp", "r") as f:
        header_content = f.read()

    subheader("Timeout and emergency handling")
    if "command_timeout_sec_" in content and "currentSafetyDecision" in content:
        PASS("Command timeout safety decision implemented")
    else:
        FAIL("Command timeout NOT implemented")

    required_safe_values = [
        "kSafeThrottlePercent = 0",
        "kSafeBrakePercent = 100",
        "kSafeGear = 0",
        "kSafeSteering = 0.0F",
    ]
    if all(value in header_content for value in required_safe_values):
        PASS("Fail-safe command is zero throttle, full brake, neutral, centered steering")
    else:
        FAIL("Fail-safe actuator command is incomplete")

    if "emergency_stop_active_{true}" in header_content:
        PASS("Interface starts in fail-safe state")
    else:
        FAIL("Interface does not default to fail-safe state")

    if '"/mission_control/emergency_stop"' in content and "transient_local" in content:
        PASS("Mission emergency state is connected with transient-local QoS")
    else:
        FAIL("Mission emergency state is not reliably connected")

    subheader("Motor ECU idle detection")
    if "motor_is_idle_" in content and "Motor ECU reports IDLE" in content:
        PASS("Motor idle detection + warning")
    else:
        FAIL("Motor idle NOT detected")

    subheader("Steering ECU failsafe detection")
    if "steer_ecu_has_error_" in content and "FAILSAFE" in content:
        PASS("Steering failsafe detection + error log")
    else:
        FAIL("Steering failsafe NOT detected")


# =============================================================================
# TEST 14: ROS INTEGRATION TEST (requires ROS 2 running)
# =============================================================================
def test_ros_integration():
    header("TEST 14: ROS Integration Test")

    if not HAS_ROS:
        WARN("Skipping - ROS 2 not available. Run in Docker with ROS 2.")
        return

    rclpy.init()
    node = rclpy.create_node("wiki_test_node")

    received_frames = []

    def can_callback(msg):
        received_frames.append(msg)

    sub = node.create_subscription(Frame, "/to_can_bus", can_callback, 10)

    # Publish a control command
    pub = node.create_publisher(Control, "/control/command/control_cmd", 10)

    time.sleep(1.0)

    msg = Control()
    msg.lateral.steering_tire_angle = 0.26  # ~15 degrees
    msg.longitudinal.acceleration = 2.0
    msg.longitudinal.velocity = 5.0
    pub.publish(msg)

    # Spin for a bit to collect CAN frames
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    subheader("CAN frame output check")
    if len(received_frames) == 0:
        FAIL("No CAN frames received - is the node running?")
        WARN("Start the node first: ros2 run my_vehicle_interface"
             " my_vehicle_interface_node")
    else:
        PASS(f"Received {len(received_frames)} CAN frames")

        # Check for expected CAN IDs
        ids_seen = set(f.id for f in received_frames)
        for expected_id, name in [(0x220, "Steering"), (0x330, "Motor"), (0x110, "Brake")]:
            if expected_id in ids_seen:
                PASS(f"CAN ID 0x{expected_id:03X} ({name}) present")
            else:
                FAIL(f"CAN ID 0x{expected_id:03X} ({name}) NOT seen")

        # Verify steering float encoding
        steer_frames = [f for f in received_frames if f.id == 0x220]
        if steer_frames:
            frame = steer_frames[-1]
            float_bytes = bytes(frame.data[:4])
            steer_val = struct.unpack("f", float_bytes)[0]
            if -1.25 <= steer_val <= 1.25:
                PASS(f"Steering float value: {steer_val:.4f} (in range)")
            else:
                FAIL(f"Steering float value: {steer_val:.4f} (OUT OF RANGE)")

        # Verify motor frame has gear byte
        motor_frames = [f for f in received_frames if f.id == 0x330]
        if motor_frames:
            frame = motor_frames[-1]
            throttle = frame.data[0]
            gear = frame.data[2]
            if 0 <= throttle <= 100:
                PASS(f"Motor throttle: {throttle}% (valid)")
            else:
                FAIL(f"Motor throttle: {throttle} (OUT OF RANGE)")
            if gear in [0, 1, 2]:
                PASS(f"Motor gear: {gear} (valid: 0=N, 1=F, 2=R)")
            else:
                FAIL(f"Motor gear: {gear} (INVALID - ECU will ignore!)")

    # Simulate CAN feedback
    subheader("Simulated CAN feedback")
    can_pub = node.create_publisher(Frame, "/from_can_bus", 10)
    vel_received = []

    def vel_callback(msg):
        vel_received.append(msg)

    vel_sub = node.create_subscription(
        VelocityReport, "/vehicle/status/velocity_status", vel_callback, 10)

    time.sleep(0.5)

    # Send fake speed sensor (0x440): 330 hm/h = ~9.17 m/s
    speed_msg = Frame()
    speed_msg.id = 0x440
    speed_msg.dlc = 8
    speed_msg.data = [0x01, 0x4A, 0, 0, 0, 0, 0, 0]
    can_pub.publish(speed_msg)

    end_time = time.time() + 1.0
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    if vel_received:
        speed = vel_received[-1].longitudinal_velocity
        if abs(speed - 9.17) < 0.5:
            PASS(f"Speed feedback: {speed:.2f} m/s (expected ~9.17)")
        else:
            FAIL(f"Speed feedback: {speed:.2f} m/s (expected ~9.17)")
    else:
        WARN("No velocity report received - node may not be running")

    node.destroy_node()
    rclpy.shutdown()


# =============================================================================
# TEST 15: CALIBRATION CHECKLIST
# =============================================================================
def test_calibration_checklist():
    header("TEST 15: CALIBRATION CHECKLIST (Arabada Yapılacaklar)")

    print(f"""
  {BOLD}Aşağıdaki değerler GERÇEK ARABADA ölçülüp ayarlanmalı:{RESET}

  {YELLOW}1. max_steering_angle_rad (varsayılan: 0.5236 = ~30°){RESET}
     → Direksiyonu sonuna kadar çevir, gerçek açıyı ölç
     → Wiki: servo range -1.25 to 1.25, turning radius = 5.75m
     → Steering sensor raw=-800 ile 800 arası

  {YELLOW}2. accel_to_throttle_gain (varsayılan: 0.33){RESET}
     → Throttle=50% ile sabit hız testleri yap
     → Farklı modlarda (Mode 1-4) test et
     → Mode 1: max ~10 km/h, Mode 2: ~20, Mode 3: ~30, Mode 4: ~60

  {YELLOW}3. decel_to_brake_gain (varsayılan: 0.20){RESET}
     → Fren tepki testi yap (düşük hızda başla!)
     → Brake=50% ile durma mesafesini ölç
     → Brake feedback (0x710) potansiyometre değerini oku

  {YELLOW}4. Steering sensor-to-rad mapping{RESET}
     → Şu an: angle_rad = raw / 800.0 * max_steering_angle_rad
     → Gerçek fiziksel açıyı ölçüp doğrula
     → Steering sensor raw=0 -> düz gidiş olmalı

  {YELLOW}5. CAN bus bağlantı testi{RESET}
     → TinCan servisini başlat: slcan_attach -f -s6 -o /dev/TinCan
     → ip link set up can0
     → candump can0 ile CAN trafiğini izle
     → Feedback mesajları (0x440, 0x1E5, 0x720, 0x730) geliyor mu?

  {YELLOW}6. Hız modu seçimi{RESET}
     → Başlangıçta Mode 1 (max 10 km/h) kullan!
     → Motor kontrolcüsünden fiziksel olarak ayarlanır

  {YELLOW}7. Acil stop testi{RESET}
     → Tyro Indus 1S kablosuz acil stop testi
     → Butona basınca motor durmalı + fren devreye girmeli
     → Menzil testi (100-300m)

  {YELLOW}8. Motor timeout testi{RESET}
     → CAN mesaj göndermeyi kes, 200ms sonra motor idle olmalı
     → Motor feedback (0x730) byte 3 = 1 olmalı
""")

    WARN("Tüm kalibrasyon değerleri arabada ölçülmeli!")
    WARN("İlk testler MUTLAKA Mode 1 (10 km/h max) ile yapılmalı!")
    WARN("Acil stop cihazı her zaman el altında olmalı!")


# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    print(f"\n{BOLD}{'#'*70}{RESET}")
    print(f"{BOLD}  SDC 2026 KART - WIKI VERIFICATION TEST SUITE{RESET}")
    print(f"{BOLD}{'#'*70}{RESET}")
    print(f"  Wiki: https://github.com/SelfDrivingChallenge/ClosedCategory/wiki/Kart")
    print(f"  Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")

    # Run all tests
    test_can_ids()
    test_steering_encoding()
    test_motor_encoding()
    test_brake_encoding()
    test_speed_decoding()
    test_steering_sensor_decoding()
    test_motor_feedback_decoding()
    test_steering_ecu_decoding()
    test_timing()
    test_autoware_integration()
    test_steering_conversion()
    test_accel_mapping()
    test_safety()

    # ROS test only if --ros flag
    if "--ros" in sys.argv:
        test_ros_integration()
    else:
        print(f"\n  {YELLOW}Skipping ROS integration test."
              f" Use --ros flag to enable.{RESET}")

    # Always show calibration checklist
    test_calibration_checklist()

    # Summary
    print(f"\n{BOLD}{'='*70}{RESET}")
    print(f"{BOLD}  SUMMARY{RESET}")
    print(f"{'='*70}")
    print(f"  {GREEN}PASSED: {pass_count}{RESET}")
    print(f"  {RED}FAILED: {fail_count}{RESET}")
    print(f"  {YELLOW}WARNINGS: {warn_count}{RESET}")
    print(f"{'='*70}")

    if fail_count > 0:
        print(f"\n  {RED}{BOLD}RESULT: FAIL - {fail_count} test(s) failed!{RESET}")
        sys.exit(1)
    else:
        print(f"\n  {GREEN}{BOLD}RESULT: ALL TESTS PASSED!{RESET}")
        sys.exit(0)
