#!/usr/bin/env python3
"""
Mission Control test harness.

Imitates the sensor / localization / Autoware messages that
mission_control's mode_start / mode_run / mode_emergency / mode_park nodes
subscribe to, so you can drive the state machine through
START -> PAUSE -> RUN -> PAUSE -> PARK -> EMERGENCY (and back) without any
real sensors or a running Autoware stack. It also echoes everything
mission_control publishes (/vehicle_mode, /mission_control/emergency_stop,
/autoware/engage, /control/command/gear_cmd) so you can watch the
transitions happen live.

Run (after sourcing your workspace, with mission_control already running
in another terminal):

    chmod +x mission_control_test_harness.py
    ./mission_control_test_harness.py

or:

    python3 mission_control_test_harness.py

Then drive it from the interactive menu.

Topic map (must match your mission_control config):
  Publishes (stimuli mission_control listens to):
    /sensing/scan                        sensor_msgs/LaserScan
    /sensing/image_raw                   sensor_msgs/Image
    /odom                                nav_msgs/Odometry
    /localization/kinematic_state        nav_msgs/Odometry        (emergency watchdog heartbeat)
    /localization/initialization_state   LocalizationInitializationState (transient_local, one-shot)
    /mission_control/goal_array          geometry_msgs/PoseArray
    /api/routing/state                   RouteState               (transient_local, one-shot)
    /control/command/control_cmd         autoware_control_msgs/Control
    /api/autoware/get/emergency          std_msgs/Bool
    /mission_control/manual_park         std_msgs/Bool
    /mission_control/manual_resume       std_msgs/Bool
    /sensing/gnss/fix, /sensing/imu/data sensor_msgs/NavSatFix, Imu
                                          (GNSS_TOPIC / IMU_TOPIC are "" by
                                           default in mission_control, so
                                           these checks are disabled unless
                                           you re-enable them - published
                                           here only for completeness)

  Subscribes (mission_control's own output, for observation):
    /vehicle_mode                        std_msgs/UInt8
    /mission_control/emergency_stop      std_msgs/Bool
    /autoware/engage                     std_msgs/Bool
    /control/command/gear_cmd            autoware_vehicle_msgs/GearCommand
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, UInt8
from sensor_msgs.msg import LaserScan, Image, NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, RouteState
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand

MODE_NAMES = {
    0: "START",
    1: "RUN",
    2: "PAUSE",
    3: "PARK",
    4: "EMERGENCY",
}

ROUTE_STATE_NAMES = {
    0: "UNKNOWN",
    1: "UNSET",
    2: "SET",
    3: "ARRIVED",
    4: "CHANGING",
}

GEAR_NAMES = {
    0: "NONE",
    1: "NEUTRAL",
    2: "DRIVE",
    20: "REVERSE",
    22: "PARK",
    23: "LOW",
}

# Autoware ADAPI status topics (initialization_state, routing/state) are
# normally published transient_local so late subscribers get the last value.
# Matching that here so the harness behaves like the real stack.
LATCHED_QOS = QoSProfile(depth=1)
LATCHED_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
LATCHED_QOS.reliability = QoSReliabilityPolicy.RELIABLE

# mission_control's EmergencyMode/StartMode use plain rclcpp::QoS(10), which
# defaults to RELIABLE - keep these matching so messages actually arrive.
HEARTBEAT_PERIOD_S = 0.2  # 5 Hz, comfortably under the 1.0s TIMEOUT


class MissionControlHarness(Node):
    def __init__(self):
        super().__init__('mission_control_test_harness')

        # ---- Publishers: stimuli mission_control subscribes to ----
        self.lidar_pub = self.create_publisher(LaserScan, '/sensing/scan', 10)
        self.camera_pub = self.create_publisher(Image, '/sensing/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.kinematic_pub = self.create_publisher(Odometry, '/localization/kinematic_state', 10)
        self.init_state_pub = self.create_publisher(
            LocalizationInitializationState, '/localization/initialization_state', LATCHED_QOS)
        self.goal_array_pub = self.create_publisher(PoseArray, '/mission_control/goal_array', 10)
        self.route_state_pub = self.create_publisher(RouteState, '/api/routing/state', LATCHED_QOS)
        self.control_cmd_pub = self.create_publisher(Control, '/control/command/control_cmd', 10)
        self.autoware_emergency_pub = self.create_publisher(Bool, '/api/autoware/get/emergency', 10)
        self.manual_park_pub = self.create_publisher(Bool, '/mission_control/manual_park', 10)
        self.manual_resume_pub = self.create_publisher(Bool, '/mission_control/manual_resume', 10)

        # GNSS/IMU checks are disabled by default (empty topic strings in
        # mission_control), included here in case you re-enable them later.
        self.gnss_pub = self.create_publisher(NavSatFix, '/sensing/gnss/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/sensing/imu/data', 10)

        # ---- Subscribers: mission_control's own output, for observation ----
        self.create_subscription(UInt8, '/vehicle_mode', self.on_vehicle_mode, 10)
        self.create_subscription(Bool, '/mission_control/emergency_stop', self.on_emergency_stop, 10)
        self.create_subscription(Bool, '/autoware/engage', self.on_engage, 10)
        self.create_subscription(GearCommand, '/control/command/gear_cmd', self.on_gear, 10)
        self.create_subscription(UInt8, '/mission_control/route_state_debug', self.on_route_state_debug, 10)

        self.last_mode = None

        # Heartbeat state - all "healthy" by default; toggle off from the
        # menu to simulate a dropped sensor / stale localization.
        self.healthy = {
            'lidar': True,
            'camera': True,
            'odom': True,
            'kinematic': True,
        }

        self.create_timer(HEARTBEAT_PERIOD_S, self.publish_heartbeats)

    # ------------------------------------------------------------------
    # Continuous heartbeats
    # ------------------------------------------------------------------
    def publish_heartbeats(self):
        now = self.get_clock().now().to_msg()

        if self.healthy['lidar']:
            msg = LaserScan()
            msg.header.stamp = now
            msg.header.frame_id = 'base_link'
            self.lidar_pub.publish(msg)

        if self.healthy['camera']:
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = 'camera_link'
            self.camera_pub.publish(msg)

        if self.healthy['odom']:
            msg = Odometry()
            msg.header.stamp = now
            msg.header.frame_id = 'odom'
            self.odom_pub.publish(msg)

        if self.healthy['kinematic']:
            msg = Odometry()
            msg.header.stamp = now
            msg.header.frame_id = 'map'
            self.kinematic_pub.publish(msg)

    # ------------------------------------------------------------------
    # One-shot / triggered stimuli
    # ------------------------------------------------------------------
    def send_localization_initialized(self):
        msg = LocalizationInitializationState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = LocalizationInitializationState.INITIALIZED
        self.init_state_pub.publish(msg)
        self.get_logger().info('Published: /localization/initialization_state = INITIALIZED')

    def send_localization_uninitialized(self):
        msg = LocalizationInitializationState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = LocalizationInitializationState.UNINITIALIZED
        self.init_state_pub.publish(msg)
        self.get_logger().info('Published: /localization/initialization_state = UNINITIALIZED')

    def send_goal_array(self, n=2):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for i in range(n):
            p = Pose()
            p.position.x = float(i * 5)
            p.position.y = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.goal_array_pub.publish(msg)
        self.get_logger().info(f'Published: /mission_control/goal_array with {n} goal(s)')

    def send_goal_reached(self):
        msg = RouteState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = RouteState.ARRIVED
        self.route_state_pub.publish(msg)
        self.get_logger().info('Published: /api/routing/state = ARRIVED')

    def send_route_set(self):
        msg = RouteState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = RouteState.SET
        self.route_state_pub.publish(msg)
        self.get_logger().info('Published: /api/routing/state = SET')

    def send_route_changing(self):
        msg = RouteState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = RouteState.CHANGING
        self.route_state_pub.publish(msg)
        self.get_logger().info('Published: /api/routing/state = CHANGING')

    def send_route_unset(self):
        msg = RouteState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = RouteState.UNSET
        self.route_state_pub.publish(msg)
        self.get_logger().info('Published: /api/routing/state = UNSET')

    def send_control_cmd(self, velocity=2.0, steering=0.0):
        msg = Control()
        msg.stamp = self.get_clock().now().to_msg()
        msg.longitudinal.velocity = velocity
        msg.lateral.steering_tire_angle = steering
        self.control_cmd_pub.publish(msg)
        self.get_logger().info(f'Published: /control/command/control_cmd v={velocity} steer={steering}')

    def send_autoware_emergency(self, flag: bool):
        msg = Bool()
        msg.data = flag
        self.autoware_emergency_pub.publish(msg)
        self.get_logger().info(f'Published: /api/autoware/get/emergency = {flag}')

    def send_manual_park(self):
        msg = Bool()
        msg.data = True
        self.manual_park_pub.publish(msg)
        self.get_logger().info('Published: /mission_control/manual_park = True')

    def send_manual_resume(self):
        msg = Bool()
        msg.data = True
        self.manual_resume_pub.publish(msg)
        self.get_logger().info('Published: /mission_control/manual_resume = True')

    def toggle_sensor(self, name):
        self.healthy[name] = not self.healthy[name]
        state = 'HEALTHY (publishing)' if self.healthy[name] else 'STALE (stopped publishing)'
        self.get_logger().warn(f'{name} heartbeat is now {state}')

    # ------------------------------------------------------------------
    # Echo callbacks - what mission_control itself is doing
    # ------------------------------------------------------------------
    def on_vehicle_mode(self, msg: UInt8):
        name = MODE_NAMES.get(msg.data, f'UNKNOWN({msg.data})')
        if name != self.last_mode:
            self.get_logger().info(f'>>> vehicle_mode changed: {self.last_mode} -> {name}')
            self.last_mode = name

    def on_emergency_stop(self, msg: Bool):
        self.get_logger().info(f'    /mission_control/emergency_stop = {msg.data}')

    def on_engage(self, msg: Bool):
        self.get_logger().info(f'    /autoware/engage = {msg.data}')

    def on_gear(self, msg: GearCommand):
        name = GEAR_NAMES.get(msg.command, str(msg.command))
        self.get_logger().info(f'    /control/command/gear_cmd = {name}')

    def on_route_state_debug(self, msg: UInt8):
        name = ROUTE_STATE_NAMES.get(msg.data, f'UNKNOWN({msg.data})')
        self.get_logger().info(f'    /mission_control/route_state_debug = {name}')


MENU = """
========== Mission Control Test Harness ==========
 1) Publish localization INITIALIZED          (needed to leave START)
 0) Publish localization UNINITIALIZED
 2) Send goal array (2 goals)                 (needed for PAUSE -> RUN)
 3) Send empty goal array                     (forces RUN -> PAUSE)
 4) Simulate goal reached (route_state ARRIVED)
 4s) Simulate route_state SET
 4c) Simulate route_state CHANGING             (send_next_goal should refuse while this holds)
 4u) Simulate route_state UNSET
 5) Publish control_cmd (v=2.0, steer=0.0)
 6) Toggle lidar heartbeat            (currently {lidar})
 7) Toggle camera heartbeat           (currently {camera})
 8) Toggle odom heartbeat             (currently {odom})
 9) Toggle kinematic_state heartbeat  (currently {kinematic})   <- emergency watchdog
 e) Trigger Autoware-reported emergency (/api/autoware/get/emergency = true)
 c) Clear Autoware-reported emergency
 p) Manual park (operator override)
 r) Manual resume (release manual park)
 q) Quit
====================================================
> """


def input_loop(node: MissionControlHarness):
    while rclpy.ok():
        h = node.healthy
        try:
            choice = input(MENU.format(
                lidar='ON' if h['lidar'] else 'OFF',
                camera='ON' if h['camera'] else 'OFF',
                odom='ON' if h['odom'] else 'OFF',
                kinematic='ON' if h['kinematic'] else 'OFF',
            )).strip().lower()
        except EOFError:
            break

        if choice == '1':
            node.send_localization_initialized()
        elif choice == '0':
            node.send_localization_uninitialized()
        elif choice == '2':
            node.send_goal_array(2)
        elif choice == '3':
            node.send_goal_array(0)
        elif choice == '4':
            node.send_goal_reached()
        elif choice == '4s':
            node.send_route_set()
        elif choice == '4c':
            node.send_route_changing()
        elif choice == '4u':
            node.send_route_unset()
        elif choice == '5':
            node.send_control_cmd()
        elif choice == '6':
            node.toggle_sensor('lidar')
        elif choice == '7':
            node.toggle_sensor('camera')
        elif choice == '8':
            node.toggle_sensor('odom')
        elif choice == '9':
            node.toggle_sensor('kinematic')
        elif choice == 'e':
            node.send_autoware_emergency(True)
        elif choice == 'c':
            node.send_autoware_emergency(False)
        elif choice == 'p':
            node.send_manual_park()
        elif choice == 'r':
            node.send_manual_resume()
        elif choice == 'q':
            rclpy.shutdown()
            break
        else:
            print('Unknown option.')


def main():
    rclpy.init()
    node = MissionControlHarness()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        input_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()