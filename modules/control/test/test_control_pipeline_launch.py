#!/usr/bin/env python3

import math
import os
import struct
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    OperationModeState,
)
from autoware_control_msgs.msg import Control
from autoware_internal_planning_msgs.srv import SetWaypointRoute
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_system_msgs.msg import AutowareState
from autoware_vehicle_msgs.msg import ControlModeReport, Engage
from can_msgs.msg import Frame
from geometry_msgs.msg import AccelWithCovarianceStamped, PoseArray
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def _launch(package, filename, arguments=None):
    path = os.path.join(get_package_share_directory(package), "launch", filename)
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                _launch(
                    "control",
                    "control.launch.py",
                    {"enable_safety_checks": "false"},
                ),
                _launch(
                    "my_vehicle_interface",
                    "vehicle_interface.launch.xml",
                    {"software_test_mode": "true"},
                ),
                _launch("mission_control", "mission_control.launch.xml"),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestControlPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("control_pipeline_test")
        qos = QoSProfile(depth=20)
        state_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.trajectory_pub = self.node.create_publisher(
            Trajectory, "/planning/trajectory", qos
        )
        self.kinematic_pub = self.node.create_publisher(
            Odometry, "/localization/kinematic_state", qos
        )
        self.wheel_odom_pub = self.node.create_publisher(Odometry, "/odom", qos)
        self.acceleration_pub = self.node.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", qos
        )
        self.autoware_state_pub = self.node.create_publisher(
            AutowareState, "/autoware/state", qos
        )
        self.localization_state_pub = self.node.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            state_qos,
        )
        self.goal_pub = self.node.create_publisher(
            PoseArray, "/mission_control/goal_array", qos
        )
        self.can_feedback_pub = self.node.create_publisher(
            Frame, "/from_can_bus", qos
        )
        self.scan_pub = self.node.create_publisher(
            LaserScan, "/sensing/scan", qos_profile_sensor_data
        )
        self.image_pub = self.node.create_publisher(
            Image, "/sensing/image_raw", qos_profile_sensor_data
        )

        self.control_commands = []
        self.raw_control_commands = []
        self.gate_operation_modes = []
        self.control_mode_reports = []
        self.can_commands = []
        self.engage_messages = []
        self.node.create_subscription(
            Control,
            "/control/trajectory_follower/control_cmd",
            self.raw_control_commands.append,
            qos,
        )
        self.node.create_subscription(
            Control,
            "/control/command/control_cmd",
            self.control_commands.append,
            qos,
        )
        self.node.create_subscription(
            Frame, "/to_can_bus", self.can_commands.append, QoSProfile(depth=500)
        )
        self.node.create_subscription(
            Engage, "/autoware/engage", self.engage_messages.append, qos
        )
        self.node.create_subscription(
            OperationModeState,
            "/control/vehicle_cmd_gate/operation_mode",
            self.gate_operation_modes.append,
            qos,
        )
        self.node.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            self.control_mode_reports.append,
            qos,
        )
        self.route_requests = []
        self.route_service = self.node.create_service(
            SetWaypointRoute,
            "/planning/set_waypoint_route",
            self._accept_route,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _accept_route(self, request, response):
        self.route_requests.append(request)
        response.status.success = True
        response.status.message = "mock route accepted"
        return response

    def _publish_inputs(self, publish_goal=True):
        stamp = self.node.get_clock().now().to_msg()

        trajectory = Trajectory()
        trajectory.header.stamp = stamp
        trajectory.header.frame_id = "map"
        for index in range(40):
            x = index * 0.5
            y = 0.015 * x * x
            yaw = math.atan(0.03 * x)
            point = TrajectoryPoint()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.orientation.z = math.sin(yaw / 2.0)
            point.pose.orientation.w = math.cos(yaw / 2.0)
            point.longitudinal_velocity_mps = 2.0
            point.acceleration_mps2 = 0.5
            point.front_wheel_angle_rad = 0.08
            point.time_from_start.sec = index // 4
            point.time_from_start.nanosec = (index % 4) * 250_000_000
            trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.kinematic_pub.publish(odom)

        wheel_odom = Odometry()
        wheel_odom.header.stamp = stamp
        wheel_odom.header.frame_id = "odom"
        wheel_odom.child_frame_id = "base_link"
        wheel_odom.pose.pose.orientation.w = 1.0
        self.wheel_odom_pub.publish(wheel_odom)

        acceleration = AccelWithCovarianceStamped()
        acceleration.header.stamp = stamp
        acceleration.header.frame_id = "base_link"
        self.acceleration_pub.publish(acceleration)

        autoware_state = AutowareState()
        autoware_state.stamp = stamp
        autoware_state.state = AutowareState.DRIVING
        self.autoware_state_pub.publish(autoware_state)

        localization_state = LocalizationInitializationState()
        localization_state.stamp = stamp
        localization_state.state = LocalizationInitializationState.INITIALIZED
        self.localization_state_pub.publish(localization_state)

        if publish_goal:
            goals = PoseArray()
            goals.header.stamp = stamp
            goals.header.frame_id = "map"
            goals.poses.append(trajectory.points[-1].pose)
            self.goal_pub.publish(goals)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "lidar_link"
        scan.angle_min = -1.0
        scan.angle_max = 1.0
        scan.angle_increment = 1.0
        scan.range_min = 0.1
        scan.range_max = 20.0
        scan.ranges = [10.0, 10.0, 10.0]
        self.scan_pub.publish(scan)

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "camera_center_link"
        image.height = 1
        image.width = 1
        image.encoding = "rgb8"
        image.step = 3
        image.data = [0, 0, 0]
        self.image_pub.publish(image)

        motor_feedback = Frame()
        motor_feedback.id = 0x730
        motor_feedback.dlc = 8
        motor_feedback.data[2] = 1
        self.can_feedback_pub.publish(motor_feedback)

        speed_feedback = Frame()
        speed_feedback.id = 0x440
        speed_feedback.dlc = 8
        self.can_feedback_pub.publish(speed_feedback)

        steering_feedback = Frame()
        steering_feedback.id = 0x1E5
        steering_feedback.dlc = 8
        self.can_feedback_pub.publish(steering_feedback)

    def test_positive_control_reaches_can(self):
        discovery_deadline = time.monotonic() + 5.0
        while time.monotonic() < discovery_deadline:
            self._publish_inputs(publish_goal=False)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self._publish_inputs(publish_goal=True)

        deadline = time.monotonic() + 30.0
        positive_control = None
        positive_motor = None
        nonzero_steering = None

        while time.monotonic() < deadline:
            self._publish_inputs(publish_goal=False)
            rclpy.spin_once(self.node, timeout_sec=0.05)

            for command in reversed(self.control_commands):
                if (
                    command.longitudinal.velocity > 0.1
                    and command.longitudinal.acceleration > 0.01
                    and abs(command.lateral.steering_tire_angle) > 0.001
                ):
                    positive_control = command
                    break

            for frame in reversed(self.can_commands):
                if frame.id == 0x330 and frame.data[0] > 0 and frame.data[2] == 1:
                    positive_motor = frame
                if frame.id == 0x220 and frame.dlc >= 4:
                    steering = struct.unpack("<f", bytes(frame.data[:4]))[0]
                    if abs(steering) > 0.001:
                        nonzero_steering = steering

            if positive_control and positive_motor and nonzero_steering:
                break

        raw_summary = self._summarize_controls(self.raw_control_commands)
        gated_summary = self._summarize_controls(self.control_commands)
        gate_mode_summary = [
            (
                message.mode,
                message.is_autoware_control_enabled,
                message.is_in_transition,
                message.is_autonomous_mode_available,
            )
            for message in self.gate_operation_modes[-5:]
        ]
        diagnostics = (
            f"raw={raw_summary}, gated={gated_summary}, "
            f"gate_operation_modes={gate_mode_summary}, "
            f"control_modes={[message.mode for message in self.control_mode_reports[-5:]]}, "
            f"engage={[message.engage for message in self.engage_messages[-5:]]}, "
            f"can_frames={len(self.can_commands)}"
        )

        self.assertTrue(
            any(message.engage for message in self.engage_messages),
            "Operation Mode Manager never published engage=true",
        )
        self.assertTrue(
            self.route_requests,
            "Mission Control never called /planning/set_waypoint_route",
        )
        self.assertTrue(
            any(
                message.mode == ControlModeReport.AUTONOMOUS
                for message in self.control_mode_reports
            ),
            "Operation Mode Manager never moved the vehicle interface to AUTONOMOUS; "
            f"{diagnostics}",
        )
        self.assertIsNotNone(
            positive_control,
            "No positive velocity/acceleration/steering command reached the gate "
            f"output; {diagnostics}",
        )
        self.assertIsNotNone(
            positive_motor,
            "No positive throttle DRIVE frame reached /to_can_bus",
        )
        self.assertIsNotNone(
            nonzero_steering,
            "No non-zero steering frame reached /to_can_bus",
        )

    @staticmethod
    def _summarize_controls(commands):
        if not commands:
            return {"count": 0}
        return {
            "count": len(commands),
            "max_velocity": max(command.longitudinal.velocity for command in commands),
            "max_acceleration": max(
                command.longitudinal.acceleration for command in commands
            ),
            "max_abs_steering": max(
                abs(command.lateral.steering_tire_angle) for command in commands
            ),
            "last": (
                commands[-1].longitudinal.velocity,
                commands[-1].longitudinal.acceleration,
                commands[-1].lateral.steering_tire_angle,
            ),
        }
