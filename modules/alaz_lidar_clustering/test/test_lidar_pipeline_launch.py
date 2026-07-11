#!/usr/bin/env python3

import math
import time
import unittest

from autoware_perception_msgs.msg import DetectedObjects
import launch
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    lidar_node = Node(
        package='alaz_lidar_clustering',
        executable='alaz_lidar_2d',
        name='alaz_lidar_2d_test',
        output='screen',
        parameters=[
            {
                'scan_topic': '/test/sensing/scan',
                'output_topic': '/test/perception/detection/objects',
                'max_range': 10.0,
                'cluster_tolerance': 0.4,
                'min_cluster_size': 3,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [lidar_node, launch_testing.actions.ReadyToTest()]
        ),
        {'lidar_node': lidar_node},
    )


class TestLidarPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('lidar_pipeline_test')
        self.scan_publisher = self.node.create_publisher(
            LaserScan, '/test/sensing/scan', qos_profile_sensor_data
        )
        self.messages = []
        self.subscription = self.node.create_subscription(
            DetectedObjects,
            '/test/perception/detection/objects',
            self.messages.append,
            10,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _publish_cluster(self):
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -0.2
        scan.angle_max = 0.2
        scan.angle_increment = 0.05
        scan.range_min = 0.1
        scan.range_max = 20.0
        scan.ranges = [math.inf, 2.02, 2.0, 1.98, 2.0, 2.02, math.inf, math.inf, math.inf]
        self.scan_publisher.publish(scan)

    def test_sensor_qos_and_metric_detection_output(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not self.messages:
            self._publish_cluster()
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.messages, 'No DetectedObjects message was published')
        message = self.messages[-1]
        self.assertEqual(message.header.frame_id, 'lidar_link')
        self.assertEqual(len(message.objects), 1)

        detected = message.objects[0]
        position = detected.kinematics.pose_with_covariance.pose.position
        self.assertGreater(position.x, 1.8)
        self.assertLess(position.x, 2.2)
        self.assertLess(abs(position.y), 0.4)
        self.assertGreater(detected.shape.dimensions.x, 0.0)
        self.assertLess(detected.shape.dimensions.x, 1.0)
        self.assertGreater(detected.shape.dimensions.y, 0.0)
        self.assertLess(detected.shape.dimensions.y, 1.0)
        self.assertFalse(detected.kinematics.has_twist)


@launch_testing.post_shutdown_test()
class TestLidarPipelineShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info, lidar_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=lidar_node)
