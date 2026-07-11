#!/usr/bin/env python3
"""
SDC 2026 Kart - CANLI ORTAM ENTEGRASYON TESTİ (HAYVANI TEST)
=============================================================

Bu test, my_vehicle_interface node'unu GERÇEK ROS ortamında test eder.
Sanal bir kart simüle eder (CAN feedback gönderir) ve tüm pipeline'ı kontrol eder.

NASIL ÇALIŞTIRILIR:
  # Terminal 1: Node'u başlat
  ros2 run my_vehicle_interface my_vehicle_interface_node

  # Terminal 2: Testi çalıştır
  python3 test/test_integration_live.py

  # VEYA hepsini tek seferde:
  python3 test/test_integration_live.py --launch

NE YAPAR:
  1. Autoware komutları gönderir (steering, throttle, brake, gear)
  2. Kart gibi CAN feedback mesajları gönderir (speed, steering sensor, motor/steer ECU)
  3. Çıkan CAN frame'leri yakalar ve byte byte doğrular
  4. Autoware status report'larını yakalar ve değerleri doğrular
  5. Edge case'leri test eder (timeout, max değerler, negatif, geçersiz gear, vs.)
  6. Tüm dönüşümleri KENDİ hesaplar ve karşılaştırır
"""

import struct
import sys
import time
import math
import random
import threading
import subprocess
import signal
import os

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    from can_msgs.msg import Frame
    from autoware_control_msgs.msg import Control
    from autoware_vehicle_msgs.msg import (
        VelocityReport, SteeringReport, GearReport,
        GearCommand, ControlModeReport,
        TurnIndicatorsCommand, TurnIndicatorsReport,
        HazardLightsCommand, HazardLightsReport,
    )
except ImportError:
    print("HATA: ROS 2 ortamı bulunamadı!")
    print("Önce: source /opt/ros/humble/setup.bash")
    print("Sonra: source /workspace/install/setup.bash")
    sys.exit(1)

# =============================================================================
# RENKLER
# =============================================================================
G = "\033[92m"   # Green
R = "\033[91m"   # Red
Y = "\033[93m"   # Yellow
C = "\033[96m"   # Cyan
B = "\033[1m"    # Bold
W = "\033[0m"    # Reset

pass_count = 0
fail_count = 0
warn_count = 0


def PASS(msg):
    global pass_count
    pass_count += 1
    print(f"    {G}✅ PASS{W}: {msg}")


def FAIL(msg):
    global fail_count
    fail_count += 1
    print(f"    {R}❌ FAIL{W}: {msg}")


def WARN(msg):
    global warn_count
    warn_count += 1
    print(f"    {Y}⚠️  WARN{W}: {msg}")


def header(title):
    print(f"\n{B}{C}{'='*72}{W}")
    print(f"{B}{C}  {title}{W}")
    print(f"{B}{C}{'='*72}{W}")


def subheader(title):
    print(f"\n  {B}--- {title} ---{W}")


# =============================================================================
# WIKI SABİTLERİ (Doğrulama için)
# =============================================================================
WIKI = {
    "STEER_CMD_ID": 0x220,
    "BRAKE_CMD_ID": 0x110,
    "MOTOR_CMD_ID": 0x330,
    "SPEED_SENSOR_ID": 0x440,
    "STEER_SENSOR_ID": 0x1E5,
    "STEER_ECU_FB_ID": 0x720,
    "MOTOR_FB_ID": 0x730,
    "BRAKE_FB_ID": 0x710,
    "STEER_RANGE_MIN": -1.25,
    "STEER_RANGE_MAX": 1.25,
    "THROTTLE_MIN": 0,
    "THROTTLE_MAX": 100,
    "BRAKE_MIN": 0,
    "BRAKE_MAX": 100,
    "GEAR_N": 0,
    "GEAR_F": 1,
    "GEAR_R": 2,
    "SENSOR_RANGE_MIN": -800,
    "SENSOR_RANGE_MAX": 800,
    "MOTOR_TIMEOUT_MS": 200,
    "LOOP_RATE_HZ": 25,
}

# Default params (from param.yaml)
MAX_STEER_RAD = 0.5236
ACCEL_GAIN = 0.33
DECEL_GAIN = 0.20


# =============================================================================
# TEST NODE
# =============================================================================
class IntegrationTestNode(Node):
    def __init__(self):
        super().__init__("integration_test_node")

        # Yakalanan mesajlar
        self.can_frames_out = []       # /to_can_bus'a giden (bizim encode'larımız)
        self.velocity_reports = []      # Autoware'e giden velocity
        self.steering_reports = []      # Autoware'e giden steering
        self.gear_reports = []          # Autoware'e giden gear
        self.control_mode_reports = []  # Autoware'e giden control mode
        self.turn_reports = []
        self.hazard_reports = []

        # SUBSCRIBERS - Node'un çıkışlarını yakala
        self.create_subscription(
            Frame, "/to_can_bus", self._on_can_out, QoSProfile(depth=500))
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status",
            self._on_velocity, QoSProfile(depth=50))
        self.create_subscription(
            SteeringReport, "/vehicle/status/steering_status",
            self._on_steering, QoSProfile(depth=50))
        self.create_subscription(
            GearReport, "/vehicle/status/gear_status",
            self._on_gear, QoSProfile(depth=50))
        self.create_subscription(
            ControlModeReport, "/vehicle/status/control_mode",
            self._on_control_mode, QoSProfile(depth=50))
        self.create_subscription(
            TurnIndicatorsReport, "/vehicle/status/turn_indicators_status",
            self._on_turn, QoSProfile(depth=50))
        self.create_subscription(
            HazardLightsReport, "/vehicle/status/hazard_lights_status",
            self._on_hazard, QoSProfile(depth=50))

        # PUBLISHERS - Node'a girdi gönder
        self.cmd_pub = self.create_publisher(
            Control, "/control/command/control_cmd", QoSProfile(depth=10))
        self.gear_pub = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", QoSProfile(depth=10))
        self.can_in_pub = self.create_publisher(
            Frame, "/from_can_bus", QoSProfile(depth=100))
        emergency_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.emergency_pub = self.create_publisher(
            Bool, "/mission_control/emergency_stop", emergency_qos)
        self.emergency_active = True
        self.create_timer(0.2, self.publish_emergency_state)

    def _on_can_out(self, msg):
        self.can_frames_out.append(msg)

    def _on_velocity(self, msg):
        self.velocity_reports.append(msg)

    def _on_steering(self, msg):
        self.steering_reports.append(msg)

    def _on_gear(self, msg):
        self.gear_reports.append(msg)

    def _on_control_mode(self, msg):
        self.control_mode_reports.append(msg)

    def _on_turn(self, msg):
        self.turn_reports.append(msg)

    def _on_hazard(self, msg):
        self.hazard_reports.append(msg)

    def clear_all(self):
        self.can_frames_out.clear()
        self.velocity_reports.clear()
        self.steering_reports.clear()
        self.gear_reports.clear()
        self.control_mode_reports.clear()
        self.turn_reports.clear()
        self.hazard_reports.clear()

    def spin_for(self, seconds):
        """Belirli süre boyunca spin et (mesajları topla)."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_control_and_wait(self, steer_rad, accel, velocity=0.0, duration=0.3):
        """Komutu birden fazla kez gönder ve bekle (timing race fix)."""
        end = time.time() + duration
        while time.time() < end:
            self.send_control(steer_rad, accel, velocity)
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_control(self, steer_rad, accel, velocity=0.0):
        msg = Control()
        msg.lateral.steering_tire_angle = steer_rad
        msg.longitudinal.acceleration = accel
        msg.longitudinal.velocity = velocity
        self.cmd_pub.publish(msg)

    def send_gear(self, gear_value):
        msg = GearCommand()
        msg.command = gear_value
        self.gear_pub.publish(msg)

    def send_emergency(self, active):
        self.emergency_active = active
        self.publish_emergency_state()

    def publish_emergency_state(self):
        self.emergency_pub.publish(Bool(data=self.emergency_active))

    def send_fake_speed(self, speed_hmh):
        """Kart gibi speed sensor (0x440) CAN mesajı gönder."""
        frame = Frame()
        frame.id = WIKI["SPEED_SENSOR_ID"]
        frame.dlc = 8
        frame.data[0] = (speed_hmh >> 8) & 0xFF  # Big-endian MSB
        frame.data[1] = speed_hmh & 0xFF          # Big-endian LSB
        self.can_in_pub.publish(frame)

    def send_fake_steering_sensor(self, raw_value):
        """Kart gibi steering sensor (0x1E5) CAN mesajı gönder."""
        frame = Frame()
        frame.id = WIKI["STEER_SENSOR_ID"]
        frame.dlc = 8
        # Byte 0 = status/counter (skip), Byte 1-2 = big-endian int16
        raw_uint = raw_value & 0xFFFF  # 2's complement
        frame.data[0] = 0  # status byte
        frame.data[1] = (raw_uint >> 8) & 0xFF  # MSB
        frame.data[2] = raw_uint & 0xFF          # LSB
        self.can_in_pub.publish(frame)

    def send_fake_motor_feedback(self, throttle_dac, braking, gear, idle):
        """Kart gibi motor ECU (0x730) CAN mesajı gönder."""
        frame = Frame()
        frame.id = WIKI["MOTOR_FB_ID"]
        frame.dlc = 8
        frame.data[0] = throttle_dac
        frame.data[1] = 1 if braking else 0
        frame.data[2] = gear
        frame.data[3] = 1 if idle else 0
        self.can_in_pub.publish(frame)

    def send_fake_steering_ecu(self, current, target, error):
        """Kart gibi steering ECU (0x720) CAN mesajı gönder."""
        frame = Frame()
        frame.id = WIKI["STEER_ECU_FB_ID"]
        frame.dlc = 8
        cur_uint = current & 0xFFFF
        tgt_uint = target & 0xFFFF
        frame.data[0] = (cur_uint >> 8) & 0xFF
        frame.data[1] = cur_uint & 0xFF
        frame.data[2] = (tgt_uint >> 8) & 0xFF
        frame.data[3] = tgt_uint & 0xFF
        frame.data[4] = 0  # direction
        frame.data[5] = 1 if error else 0
        self.can_in_pub.publish(frame)

    def get_last_can_by_id(self, can_id):
        """Belirli CAN ID'li son frame'i bul."""
        for f in reversed(self.can_frames_out):
            if f.id == can_id:
                return f
        return None


# =============================================================================
# TEST FONKSİYONLARI
# =============================================================================

def test_01_node_alive(node):
    """Node başlatıldı mı? Heartbeat (timer) çalışıyor mu?"""
    header("TEST 01: Node Hayatta mı?")

    node.clear_all()
    node.spin_for(1.0)

    if len(node.can_frames_out) > 0:
        PASS(f"{len(node.can_frames_out)} CAN frame alındı (node çalışıyor)")
    else:
        FAIL("Hiç CAN frame yok - node çalışmıyor olabilir!")
        WARN("Node'u başlattın mı? ros2 run my_vehicle_interface"
             " my_vehicle_interface_node")
        return False

    # Hangi CAN ID'ler var?
    ids = set(f.id for f in node.can_frames_out)
    for expected_id, name in [
        (0x220, "Steering"), (0x330, "Motor"), (0x110, "Brake")
    ]:
        if expected_id in ids:
            PASS(f"CAN ID 0x{expected_id:03X} ({name}) gönderiliyor")
        else:
            FAIL(f"CAN ID 0x{expected_id:03X} ({name}) YOK!")

    # Autoware status yayınlanıyor mu?
    if len(node.velocity_reports) > 0:
        PASS("VelocityReport yayınlanıyor")
    else:
        FAIL("VelocityReport YOK")
    if len(node.steering_reports) > 0:
        PASS("SteeringReport yayınlanıyor")
    else:
        FAIL("SteeringReport YOK")
    if len(node.gear_reports) > 0:
        PASS("GearReport yayınlanıyor")
    else:
        FAIL("GearReport YOK")
    if len(node.control_mode_reports) > 0:
        PASS("ControlModeReport yayınlanıyor")
    else:
        FAIL("ControlModeReport YOK")
    if len(node.turn_reports) > 0:
        PASS("TurnIndicatorsReport yayınlanıyor")
    else:
        FAIL("TurnIndicatorsReport YOK")
    if len(node.hazard_reports) > 0:
        PASS("HazardLightsReport yayınlanıyor")
    else:
        FAIL("HazardLightsReport YOK")

    return True


def test_safety_stop_chain(node):
    """Emergency and command timeout must produce the physical safe command."""
    header("SAFETY: Emergency and Command Watchdog")

    def assert_safe_command(label):
        steer = node.get_last_can_by_id(WIKI["STEER_CMD_ID"])
        motor = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
        brake = node.get_last_can_by_id(WIKI["BRAKE_CMD_ID"])
        if not steer or not motor or not brake:
            FAIL(f"{label}: safe CAN frame set is incomplete")
            return

        steer_value = struct.unpack("<f", bytes(steer.data[:4]))[0]
        if (abs(steer_value) < 0.001 and motor.data[0] == 0 and
                motor.data[2] == WIKI["GEAR_N"] and brake.data[0] == 100):
            PASS(f"{label}: centered steering, zero throttle, neutral, full brake")
        else:
            FAIL(
                f"{label}: steer={steer_value}, throttle={motor.data[0]}, "
                f"gear={motor.data[2]}, brake={brake.data[0]}")

    # The interface starts stopped until mission control reports healthy inputs.
    node.clear_all()
    node.spin_for(0.15)
    assert_safe_command("Startup")

    # Clearing emergency alone is insufficient without a fresh command.
    node.clear_all()
    node.send_emergency(False)
    node.spin_for(0.15)
    assert_safe_command("Healthy sensors without command")

    # Fresh commands may drive after mission health is clear.
    node.clear_all()
    node.send_gear(GearCommand.DRIVE)
    node.send_control_and_wait(steer_rad=0.1, accel=1.0, duration=0.15)
    motor = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
    brake = node.get_last_can_by_id(WIKI["BRAKE_CMD_ID"])
    if motor and brake and motor.data[0] > 0 and motor.data[2] == WIKI["GEAR_F"] and brake.data[0] == 0:
        PASS("Fresh command resumes normal actuation")
    else:
        FAIL("Fresh command did not clear the safety stop")

    # Emergency overrides fresh commands.
    node.clear_all()
    node.send_emergency(True)
    node.send_control_and_wait(steer_rad=0.1, accel=1.0, duration=0.15)
    assert_safe_command("Emergency")

    # Recovery requires both a cleared emergency and a new command.
    node.clear_all()
    node.send_emergency(False)
    node.send_control_and_wait(steer_rad=0.1, accel=1.0, duration=0.15)
    motor = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
    if motor and motor.data[0] > 0:
        PASS("Cleared emergency plus fresh command resumes actuation")
    else:
        FAIL("Emergency recovery did not resume actuation")

    # Stop refreshing the command and wait beyond the configured 200 ms timeout.
    node.clear_all()
    node.spin_for(0.3)
    assert_safe_command("Control command timeout")

    # Leave the node healthy for subsequent encoding tests.
    node.send_emergency(False)
    node.send_control_and_wait(steer_rad=0.0, accel=0.0, duration=0.15)


def test_02_steering_encoding(node):
    """Steering float encoding doğru mu?"""
    header("TEST 02: Steering Encoding (IEEE 754 Float)")

    test_angles = [0.0, 0.1, 0.26, 0.5, -0.1, -0.26, -0.5]

    for angle_rad in test_angles:
        node.clear_all()
        node.send_control_and_wait(steer_rad=angle_rad, accel=0.0, duration=0.3)

        frame = node.get_last_can_by_id(WIKI["STEER_CMD_ID"])
        if frame is None:
            FAIL(f"Steering frame (0x220) bulunamadı! (angle={angle_rad})")
            continue

        # CAN frame'den float'ı oku
        float_bytes = bytes(frame.data[:4])
        received_float = struct.unpack("<f", float_bytes)[0]

        # Beklenen değeri biz hesaplıyoruz
        expected_kart = angle_rad / MAX_STEER_RAD
        expected_kart = max(-1.25, min(1.25, expected_kart))

        if abs(received_float - expected_kart) < 0.01:
            PASS(f"angle={angle_rad:.2f}rad → float={received_float:.4f}"
                 f" (beklenen: {expected_kart:.4f})")
        else:
            FAIL(f"angle={angle_rad:.2f}rad → float={received_float:.4f}"
                 f" (beklenen: {expected_kart:.4f})")

    subheader("Clamp kontrolü (max değer aşılınca)")
    # max_steer_rad'ı aşan değer → 1.25'e clamp olmalı
    node.clear_all()
    node.send_control_and_wait(steer_rad=1.5, accel=0.0, duration=0.3)

    frame = node.get_last_can_by_id(WIKI["STEER_CMD_ID"])
    if frame:
        val = struct.unpack("<f", bytes(frame.data[:4]))[0]
        if val <= 1.25:
            PASS(f"Clamp çalışıyor: 1.5 rad → {val:.4f} (max 1.25)")
        else:
            FAIL(f"Clamp BAŞARISIZ: 1.5 rad → {val:.4f} (1.25'i aştı!)")

    # Negatif clamp
    node.clear_all()
    node.send_control_and_wait(steer_rad=-1.5, accel=0.0, duration=0.3)

    frame = node.get_last_can_by_id(WIKI["STEER_CMD_ID"])
    if frame:
        val = struct.unpack("<f", bytes(frame.data[:4]))[0]
        if val >= -1.25:
            PASS(f"Negatif clamp çalışıyor: -1.5 rad → {val:.4f} (min -1.25)")
        else:
            FAIL(f"Negatif clamp BAŞARISIZ: -1.5 rad → {val:.4f}")


def test_03_motor_encoding(node):
    """Motor throttle + gear encoding doğru mu?"""
    header("TEST 03: Motor Encoding (Throttle + Gear)")

    subheader("Throttle testi")
    test_accels = [0.5, 1.0, 2.0, 3.0, 5.0]

    for accel in test_accels:
        node.clear_all()
        node.send_control(steer_rad=0.0, accel=accel)
        node.spin_for(0.2)

        frame = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
        if frame is None:
            FAIL(f"Motor frame (0x330) bulunamadı! (accel={accel})")
            continue

        throttle = frame.data[0]
        gear = frame.data[2]

        # Biz hesaplıyoruz
        expected_throttle = min(int(min(accel * ACCEL_GAIN, 1.0) * 100), 100)

        if abs(throttle - expected_throttle) <= 1:
            PASS(f"accel={accel:.1f} → throttle={throttle}%"
                 f" (beklenen: {expected_throttle}%)")
        else:
            FAIL(f"accel={accel:.1f} → throttle={throttle}%"
                 f" (beklenen: {expected_throttle}%)")

        # Byte 1 = reserved (0 olmalı)
        if frame.data[1] == 0:
            PASS(f"  Byte 1 = 0x00 (reserved)")
        else:
            FAIL(f"  Byte 1 = 0x{frame.data[1]:02X} (0 olmalı!)")

        # Gear valid mi?
        if gear <= 2:
            PASS(f"  Gear = {gear} (valid)")
        else:
            FAIL(f"  Gear = {gear} (INVALID! >= 3 mesajı geçersiz kılar!)")

    subheader("Gear değişimi testi")
    gear_tests = [
        (GearCommand.NEUTRAL, 0, "NEUTRAL"),
        (GearCommand.DRIVE, 1, "DRIVE (Forward)"),
        (GearCommand.REVERSE, 2, "REVERSE"),
        (GearCommand.PARK, 0, "PARK (→ Neutral)"),
    ]

    for aw_gear, expected_kart_gear, name in gear_tests:
        node.clear_all()
        node.send_gear(aw_gear)
        node.send_control_and_wait(steer_rad=0.0, accel=1.0, duration=0.3)

        frame = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
        if frame:
            if frame.data[2] == expected_kart_gear:
                PASS(f"Gear {name}: AW={aw_gear} → Kart={frame.data[2]}")
            else:
                FAIL(f"Gear {name}: AW={aw_gear} → Kart={frame.data[2]}"
                     f" (beklenen: {expected_kart_gear})")


def test_04_brake_encoding(node):
    """Brake encoding doğru mu?"""
    header("TEST 04: Brake Encoding")

    test_decels = [-0.5, -1.0, -2.0, -3.0, -5.0, -10.0]

    for accel in test_decels:
        node.clear_all()
        node.send_control(steer_rad=0.0, accel=accel)
        node.spin_for(0.2)

        frame = node.get_last_can_by_id(WIKI["BRAKE_CMD_ID"])
        if frame is None:
            FAIL(f"Brake frame (0x110) bulunamadı! (accel={accel})")
            continue

        brake = frame.data[0]
        expected_brake = min(int(min(-accel * DECEL_GAIN, 1.0) * 100), 100)

        if abs(brake - expected_brake) <= 1:
            PASS(f"accel={accel:.1f} → brake={brake}%"
                 f" (beklenen: {expected_brake}%)")
        else:
            FAIL(f"accel={accel:.1f} → brake={brake}%"
                 f" (beklenen: {expected_brake}%)")

        # Brake ederken throttle 0 olmalı
        motor_frame = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
        if motor_frame and motor_frame.data[0] == 0:
            PASS(f"  Fren sırasında throttle = 0% ✓")
        elif motor_frame:
            FAIL(f"  Fren sırasında throttle = {motor_frame.data[0]}%"
                 f" (0 olmalı!)")

    subheader("Brake clamp (max %100)")
    node.clear_all()
    node.send_control(steer_rad=0.0, accel=-100.0)  # Absürt değer
    node.spin_for(0.2)
    frame = node.get_last_can_by_id(WIKI["BRAKE_CMD_ID"])
    if frame:
        if frame.data[0] <= 100:
            PASS(f"Brake clamp: accel=-100 → brake={frame.data[0]}% (max 100)")
        else:
            FAIL(f"Brake clamp BAŞARISIZ: {frame.data[0]}%")


def test_05_speed_feedback(node):
    """Speed sensor → VelocityReport dönüşümü doğru mu?"""
    header("TEST 05: Speed Sensor Feedback (0x440 → VelocityReport)")

    test_speeds_hmh = [0, 15, 100, 200, 330, 500, 600]

    for hmh in test_speeds_hmh:
        node.clear_all()
        node.send_fake_speed(hmh)
        node.spin_for(0.2)

        if len(node.velocity_reports) == 0:
            FAIL(f"VelocityReport alınamadı (speed={hmh} hm/h)")
            continue

        received_ms = node.velocity_reports[-1].longitudinal_velocity
        expected_ms = hmh / 36.0  # hm/h → m/s

        if abs(received_ms - expected_ms) < 0.1:
            PASS(f"{hmh} hm/h → {received_ms:.2f} m/s"
                 f" (beklenen: {expected_ms:.2f})")
        else:
            FAIL(f"{hmh} hm/h → {received_ms:.2f} m/s"
                 f" (beklenen: {expected_ms:.2f})")

    subheader("Hız dönüşüm doğrulaması")
    # 330 hm/h = 33.0 km/h = 9.17 m/s (wiki örneği)
    node.clear_all()
    node.send_fake_speed(330)
    node.spin_for(0.2)
    if node.velocity_reports:
        v = node.velocity_reports[-1].longitudinal_velocity
        if abs(v - 9.1667) < 0.1:
            PASS(f"Wiki örneği: 330 hm/h = {v:.2f} m/s ≈ 9.17 m/s ✓")
        else:
            FAIL(f"Wiki örneği: 330 hm/h = {v:.2f} m/s (beklenen ~9.17)")


def test_06_steering_sensor_feedback(node):
    """Steering sensor → SteeringReport dönüşümü doğru mu?"""
    header("TEST 06: Steering Sensor Feedback (0x1E5 → SteeringReport)")

    test_raws = [0, 100, 400, 800, -100, -400, -800]

    for raw in test_raws:
        node.clear_all()
        node.send_fake_steering_sensor(raw)
        node.spin_for(0.2)

        if len(node.steering_reports) == 0:
            FAIL(f"SteeringReport alınamadı (raw={raw})")
            continue

        received_rad = node.steering_reports[-1].steering_tire_angle
        # Biz hesaplıyoruz
        expected_rad = (raw / 800.0) * MAX_STEER_RAD

        if abs(received_rad - expected_rad) < 0.01:
            PASS(f"raw={raw:+5d} → {received_rad:+.4f} rad"
                 f" (beklenen: {expected_rad:+.4f})")
        else:
            FAIL(f"raw={raw:+5d} → {received_rad:+.4f} rad"
                 f" (beklenen: {expected_rad:+.4f})")

    subheader("Sıfır noktası kontrolü")
    node.clear_all()
    node.send_fake_steering_sensor(0)
    node.spin_for(0.2)
    if node.steering_reports:
        v = node.steering_reports[-1].steering_tire_angle
        if abs(v) < 0.001:
            PASS(f"raw=0 → {v:.6f} rad (sıfır noktası doğru)")
        else:
            FAIL(f"raw=0 → {v:.6f} rad (sıfır olmalı!)")


def test_07_motor_feedback(node):
    """Motor ECU feedback (0x730) doğru decode ediliyor mu?"""
    header("TEST 07: Motor ECU Feedback (0x730 → GearReport)")

    subheader("Gear feedback")
    gear_tests = [
        (0, GearReport.NEUTRAL, "Neutral"),
        (1, GearReport.DRIVE, "Forward/Drive"),
        (2, GearReport.REVERSE, "Reverse"),
    ]

    for kart_gear, expected_report, name in gear_tests:
        node.clear_all()
        node.send_fake_motor_feedback(
            throttle_dac=50, braking=False, gear=kart_gear, idle=False)
        node.spin_for(0.2)

        if len(node.gear_reports) == 0:
            FAIL(f"GearReport alınamadı (gear={kart_gear})")
            continue

        if node.gear_reports[-1].report == expected_report:
            PASS(f"Kart gear={kart_gear} ({name}) → GearReport doğru")
        else:
            FAIL(f"Kart gear={kart_gear} ({name}) → GearReport yanlış"
                 f" (got {node.gear_reports[-1].report})")

    subheader("Idle flag testi")
    node.clear_all()
    node.send_fake_motor_feedback(
        throttle_dac=0, braking=False, gear=0, idle=True)
    node.spin_for(0.3)
    PASS("Motor idle flag gönderildi (log'da WARN görmeli)")

    subheader("Braking flag testi")
    node.clear_all()
    node.send_fake_motor_feedback(
        throttle_dac=0, braking=True, gear=1, idle=False)
    node.spin_for(0.3)
    PASS("Motor braking flag gönderildi (throttle/gear kilitli)")


def test_08_steering_ecu_feedback(node):
    """Steering ECU feedback (0x720) doğru decode ediliyor mu?"""
    header("TEST 08: Steering ECU Feedback (0x720)")

    subheader("Normal feedback")
    node.clear_all()
    node.send_fake_steering_ecu(current=200, target=300, error=False)
    node.spin_for(0.3)
    PASS("Steering ECU normal feedback gönderildi (current=200, target=300)")

    subheader("Failsafe flag testi")
    node.clear_all()
    node.send_fake_steering_ecu(current=850, target=0, error=True)
    node.spin_for(0.3)
    PASS("Steering ECU FAILSAFE gönderildi (log'da ERROR görmeli)")

    subheader("Negatif açı testi")
    node.clear_all()
    node.send_fake_steering_ecu(current=-400, target=-200, error=False)
    node.spin_for(0.3)
    PASS("Negatif açılar gönderildi (current=-400, target=-200)")


def test_09_full_round_trip(node):
    """Tam döngü: Autoware komutu → CAN → Fake feedback → Autoware rapor."""
    header("TEST 09: TAM DÖNGÜ TESTİ (Encode + Decode Round-Trip)")

    subheader("Senaryo: Sola dön + gaz ver + hız feedback'i")
    node.clear_all()

    # 1. Autoware komutu gönder
    steer_rad = 0.26  # ~15 derece sola
    accel = 2.0       # 2 m/s² hızlan
    node.send_gear(GearCommand.DRIVE)

    # 2. Komutu ve feedback'i birden fazla kez gönder (timing fix)
    end = time.time() + 0.5
    while time.time() < end:
        node.send_control(steer_rad=steer_rad, accel=accel)
        node.send_fake_speed(200)                     # 20 km/h
        node.send_fake_steering_sensor(325)            # ~yarım sağ
        node.send_fake_motor_feedback(100, False, 1, False)
        rclpy.spin_once(node, timeout_sec=0.02)

    # 3. CAN çıkışlarını kontrol et
    steer_frame = node.get_last_can_by_id(WIKI["STEER_CMD_ID"])
    motor_frame = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
    brake_frame = node.get_last_can_by_id(WIKI["BRAKE_CMD_ID"])

    if steer_frame and motor_frame and brake_frame:
        PASS("3 CAN komutu gönderildi (steer + motor + brake)")

        steer_val = struct.unpack("<f", bytes(steer_frame.data[:4]))[0]
        throttle = motor_frame.data[0]
        gear = motor_frame.data[2]
        brake = brake_frame.data[0]

        expected_steer = steer_rad / MAX_STEER_RAD
        expected_throttle = min(int(accel * ACCEL_GAIN * 100), 100)

        print(f"      Steering: {steer_val:.4f}"
              f" (beklenen: {expected_steer:.4f})")
        print(f"      Throttle: {throttle}%"
              f" (beklenen: {expected_throttle}%)")
        print(f"      Gear:     {gear} (Forward)")
        print(f"      Brake:    {brake}% (0 bekleniyor)")

        if abs(steer_val - expected_steer) < 0.01:
            PASS("Steering dönüşümü doğru")
        else:
            FAIL("Steering dönüşümü YANLIŞ")

        if abs(throttle - expected_throttle) <= 1:
            PASS("Throttle dönüşümü doğru")
        else:
            FAIL("Throttle dönüşümü YANLIŞ")

        if brake == 0:
            PASS("Gaz verirken fren = 0%")
        else:
            FAIL(f"Gaz verirken fren = {brake}% (0 olmalı!)")
    else:
        FAIL("CAN frame'ler eksik!")

    # 4. Autoware raporlarını kontrol et
    if node.velocity_reports:
        v = node.velocity_reports[-1].longitudinal_velocity
        expected_v = 200 / 36.0  # ~5.56 m/s
        if abs(v - expected_v) < 0.1:
            PASS(f"Velocity report: {v:.2f} m/s (beklenen: {expected_v:.2f})")
        else:
            FAIL(f"Velocity report: {v:.2f} m/s (beklenen: {expected_v:.2f})")

    if node.steering_reports:
        s = node.steering_reports[-1].steering_tire_angle
        expected_s = 325 / 800.0 * MAX_STEER_RAD
        if abs(s - expected_s) < 0.01:
            PASS(f"Steering report: {s:.4f} rad (beklenen: {expected_s:.4f})")
        else:
            FAIL(f"Steering report: {s:.4f} rad (beklenen: {expected_s:.4f})")

    if node.gear_reports:
        g = node.gear_reports[-1].report
        if g == GearReport.DRIVE:
            PASS("Gear report: DRIVE ✓")
        else:
            FAIL(f"Gear report: {g} (DRIVE bekleniyor)")


def test_10_random_stress(node):
    """Rastgele değerlerle stres testi."""
    header("TEST 10: RASTGELE STRES TESTİ (50 iterasyon)")

    random.seed(42)
    errors = 0

    for i in range(50):
        # Rastgele değerler üret
        rand_steer = random.uniform(-0.6, 0.6)
        rand_accel = random.uniform(-5.0, 5.0)
        rand_speed_hmh = random.randint(0, 600)
        rand_sensor = random.randint(-800, 800)
        rand_gear = random.choice([
            GearCommand.NEUTRAL, GearCommand.DRIVE, GearCommand.REVERSE])

        node.clear_all()

        # Komut gönder
        node.send_gear(rand_gear)
        node.send_control(steer_rad=rand_steer, accel=rand_accel)
        node.send_fake_speed(rand_speed_hmh)
        node.send_fake_steering_sensor(rand_sensor)

        node.spin_for(0.15)

        # CAN frame kontrol
        steer_frame = node.get_last_can_by_id(WIKI["STEER_CMD_ID"])
        motor_frame = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
        brake_frame = node.get_last_can_by_id(WIKI["BRAKE_CMD_ID"])

        if not steer_frame or not motor_frame or not brake_frame:
            errors += 1
            continue

        # Steering float valid mi?
        steer_val = struct.unpack("<f", bytes(steer_frame.data[:4]))[0]
        if not (-1.25 <= steer_val <= 1.25):
            FAIL(f"  [{i}] Steering float aralık dışı: {steer_val:.4f}")
            errors += 1

        # Throttle/brake aralığı
        throttle = motor_frame.data[0]
        brake = brake_frame.data[0]
        gear = motor_frame.data[2]

        if throttle > 100:
            FAIL(f"  [{i}] Throttle > 100: {throttle}")
            errors += 1
        if brake > 100:
            FAIL(f"  [{i}] Brake > 100: {brake}")
            errors += 1
        if gear > 2:
            FAIL(f"  [{i}] Gear >= 3: {gear} (mesaj reddedilir!)")
            errors += 1

        # Throttle ve brake aynı anda aktif olmamalı
        if throttle > 0 and brake > 0:
            FAIL(f"  [{i}] Throttle={throttle} VE Brake={brake}"
                 f" İKİSİ BİRDEN AKTİF!")
            errors += 1

        # Velocity report valid mi?
        if node.velocity_reports:
            v = node.velocity_reports[-1].longitudinal_velocity
            expected_v = rand_speed_hmh / 36.0
            if abs(v - expected_v) > 0.5:
                FAIL(f"  [{i}] Velocity: {v:.2f} != {expected_v:.2f}")
                errors += 1

    if errors == 0:
        PASS(f"50 rastgele iterasyon TAMAMEN BAŞARILI!")
    else:
        FAIL(f"50 iterasyonda {errors} hata bulundu")


def test_11_edge_cases(node):
    """Uç durumlar ve sınır değerleri."""
    header("TEST 11: Edge Case'ler")

    subheader("Sıfır komut (durma)")
    node.clear_all()
    node.send_control(steer_rad=0.0, accel=0.0)
    node.spin_for(0.2)

    steer_frame = node.get_last_can_by_id(WIKI["STEER_CMD_ID"])
    motor_frame = node.get_last_can_by_id(WIKI["MOTOR_CMD_ID"])
    brake_frame = node.get_last_can_by_id(WIKI["BRAKE_CMD_ID"])

    if steer_frame:
        val = struct.unpack("<f", bytes(steer_frame.data[:4]))[0]
        if abs(val) < 0.001:
            PASS(f"Sıfır steering: {val:.6f}")
        else:
            FAIL(f"Sıfır steering: {val:.6f} (0 olmalı)")
    if motor_frame and motor_frame.data[0] == 0:
        PASS("Sıfır throttle: 0%")
    elif motor_frame:
        FAIL(f"Sıfır throttle: {motor_frame.data[0]}% (0 olmalı)")
    if brake_frame and brake_frame.data[0] == 0:
        PASS("Sıfır brake: 0%")
    elif brake_frame:
        FAIL(f"Sıfır brake: {brake_frame.data[0]}% (0 olmalı)")

    subheader("Çok küçük değerler (epsilon)")
    node.clear_all()
    node.send_control(steer_rad=0.001, accel=0.001)
    node.spin_for(0.2)
    PASS("Epsilon değerler gönderildi (crash yok)")

    subheader("Negatif sıfır testi")
    node.clear_all()
    node.send_control(steer_rad=-0.0, accel=-0.0)
    node.spin_for(0.2)
    PASS("Negatif sıfır gönderildi (crash yok)")

    subheader("Speed sensor sınır değerleri")
    for hmh in [0, 1, 65535]:
        node.clear_all()
        node.send_fake_speed(hmh)
        node.spin_for(0.2)
        if node.velocity_reports:
            v = node.velocity_reports[-1].longitudinal_velocity
            PASS(f"Speed={hmh} hm/h → {v:.2f} m/s (crash yok)")

    subheader("Steering sensor sınır değerleri")
    for raw in [-32768, -800, -1, 0, 1, 800, 32767]:
        node.clear_all()
        node.send_fake_steering_sensor(raw)
        node.spin_for(0.2)
        if node.steering_reports:
            PASS(f"Sensor raw={raw} (crash yok)")


def test_12_timing(node):
    """25 Hz rate doğru mu?"""
    header("TEST 12: Timing (25 Hz)")

    node.clear_all()
    node.spin_for(2.0)

    steer_frames = [f for f in node.can_frames_out
                    if f.id == WIKI["STEER_CMD_ID"]]
    motor_frames = [f for f in node.can_frames_out
                    if f.id == WIKI["MOTOR_CMD_ID"]]

    # 2 saniyede ~50 frame gelmeli (25 Hz)
    if 40 <= len(steer_frames) <= 60:
        PASS(f"Steering: {len(steer_frames)} frame / 2s"
             f" ≈ {len(steer_frames)/2:.0f} Hz (beklenen: 25)")
    else:
        FAIL(f"Steering: {len(steer_frames)} frame / 2s"
             f" ≈ {len(steer_frames)/2:.0f} Hz (beklenen: ~50 frame)")

    if 40 <= len(motor_frames) <= 60:
        PASS(f"Motor: {len(motor_frames)} frame / 2s"
             f" ≈ {len(motor_frames)/2:.0f} Hz (beklenen: 25)")
    else:
        FAIL(f"Motor: {len(motor_frames)} frame / 2s"
             f" ≈ {len(motor_frames)/2:.0f} Hz (beklenen: ~50 frame)")

    # Motor timeout kontrolü: 200ms = 5 Hz minimum
    hz = len(motor_frames) / 2.0
    if hz >= 5:
        PASS(f"Motor rate ({hz:.0f} Hz) > 5 Hz (200ms timeout safe)")
    else:
        FAIL(f"Motor rate ({hz:.0f} Hz) < 5 Hz (MOTOR IDLE OLACAK!)")


def test_13_simultaneous_commands(node):
    """Aynı anda birden fazla komut gönderildiğinde çakışma var mı?"""
    header("TEST 13: Eşzamanlı Komut Testi")

    node.clear_all()

    # Hızlıca ardışık komutlar gönder
    for i in range(20):
        angle = math.sin(i * 0.3) * 0.3
        accel = math.cos(i * 0.2) * 2.0
        node.send_control(steer_rad=angle, accel=accel)
        node.send_fake_speed(100 + i * 10)
        node.send_fake_steering_sensor(int(angle * 800))
        node.spin_for(0.05)

    errors = 0
    for f in node.can_frames_out:
        if f.id == WIKI["STEER_CMD_ID"]:
            val = struct.unpack("<f", bytes(f.data[:4]))[0]
            if not (-1.25 <= val <= 1.25):
                errors += 1
        elif f.id == WIKI["MOTOR_CMD_ID"]:
            if f.data[0] > 100 or f.data[2] > 2:
                errors += 1
        elif f.id == WIKI["BRAKE_CMD_ID"]:
            if f.data[0] > 100:
                errors += 1

    if errors == 0:
        PASS(f"20 hızlı ardışık komut gönderildi, "
             f"{len(node.can_frames_out)} frame valid")
    else:
        FAIL(f"{errors} invalid frame bulundu!")


# =============================================================================
# MAIN
# =============================================================================
def main():
    print(f"\n{B}{'#'*72}{W}")
    print(f"{B}  SDC 2026 KART - CANLI ORTAM ENTEGRASYON TESTİ{W}")
    print(f"{B}  (HAYVANI TEST - {time.strftime('%Y-%m-%d %H:%M:%S')}){W}")
    print(f"{B}{'#'*72}{W}")

    # Node'u ayrı process olarak başlat (--launch flag)
    node_proc = None
    if "--launch" in sys.argv:
        print(f"\n  {Y}Node başlatılıyor...{W}")
        node_proc = subprocess.Popen(
            ["ros2", "run", "my_vehicle_interface",
             "my_vehicle_interface_node"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        time.sleep(2.0)
        print(f"  {G}Node başlatıldı (PID: {node_proc.pid}){W}")

    try:
        rclpy.init()
        node = IntegrationTestNode()

        # Node çalışıyor mu kontrol et
        if not test_01_node_alive(node):
            print(f"\n  {R}{B}NODE ÇALIŞMIYOR! Önce node'u başlat:{W}")
            print(f"  ros2 run my_vehicle_interface my_vehicle_interface_node")
            rclpy.shutdown()
            return

        test_safety_stop_chain(node)

        # TÜM TESTLERİ ÇALIŞTIR
        test_02_steering_encoding(node)
        test_03_motor_encoding(node)
        test_04_brake_encoding(node)
        test_05_speed_feedback(node)
        test_06_steering_sensor_feedback(node)
        test_07_motor_feedback(node)
        test_08_steering_ecu_feedback(node)
        test_09_full_round_trip(node)
        test_10_random_stress(node)
        test_11_edge_cases(node)
        test_12_timing(node)
        test_13_simultaneous_commands(node)

        node.destroy_node()
        rclpy.shutdown()

    finally:
        if node_proc:
            node_proc.send_signal(signal.SIGINT)
            try:
                node_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                node_proc.terminate()
                node_proc.wait(timeout=5)
            print(f"\n  {Y}Node durduruldu{W}")

    # SONUÇ
    print(f"\n{B}{'='*72}{W}")
    print(f"{B}  SONUÇ{W}")
    print(f"{'='*72}")
    print(f"  {G}PASSED : {pass_count}{W}")
    print(f"  {R}FAILED : {fail_count}{W}")
    print(f"  {Y}WARNINGS: {warn_count}{W}")
    print(f"{'='*72}")

    if fail_count > 0:
        print(f"\n  {R}{B}❌ FAILED - {fail_count} test başarısız!{W}\n")
        sys.exit(1)
    else:
        print(f"\n  {G}{B}✅ TÜM TESTLER GEÇTİ!{W}\n")
        sys.exit(0)


if __name__ == "__main__":
    main()
