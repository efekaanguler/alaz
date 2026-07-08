#!/usr/bin/env python3
"""Dummy LaserScan publisher for testing the perception → planning pipeline.

Publishes synthetic LaserScan data on /scan topic so that the
laserscan_to_pcl_and_occ pipeline can be tested without real hardware.

Simulates a simple scene with:
  - A wall at ~3m ahead (front 60° arc)
  - Open space on the sides
  - Optional moving obstacle

Usage:
    python3 dummy_lidar_publisher.py
    python3 dummy_lidar_publisher.py --rate 10 --obstacle
"""

import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import yaml
import os

def get_default_topic():
    default_topic = '/sensing/lidar/top/scan'
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(script_dir, "../../global_bringup/config/topics.yaml")
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            topics = data.get('topics', {})
            if 'input_laserscan' in topics:
                return topics['input_laserscan']
            return default_topic
    except Exception:
        return default_topic


class DummyLidarPublisher(Node):
    def __init__(self, rate=10.0, add_obstacle=False, topic=None):
        super().__init__('dummy_lidar_publisher')
        if topic is None:
            topic = get_default_topic()
        self.pub = self.create_publisher(LaserScan, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_scan)
        self.add_obstacle = add_obstacle
        self.frame_count = 0
        self.get_logger().info(
            f'Dummy lidar publisher started at {rate} Hz on {topic}, '
            f'obstacle={"on" if add_obstacle else "off"}'
        )

    def publish_scan(self):
        if not rclpy.ok():
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'

        # Scan parameters (typical 2D lidar)
        msg.angle_min = -math.pi           # -180°
        msg.angle_max = math.pi            # +180°
        num_readings = 720                  # 0.5° resolution
        msg.angle_increment = (msg.angle_max - msg.angle_min) / num_readings
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0

        ranges = []
        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment

            # Default: open space at max range
            r = msg.range_max

            # Front wall at ~3m (±30° from center, i.e. angle near 0)
            if -math.pi / 6 < angle < math.pi / 6:
                r = 3.0 + 0.2 * math.sin(angle * 10)  # slight waviness

            # Side walls at ~5m (±60° to ±90°)
            if (math.pi / 3 < abs(angle) < math.pi / 2):
                r = 5.0

            # Optional moving obstacle (sweeps left-right)
            if self.add_obstacle:
                obs_angle = 0.5 * math.sin(self.frame_count * 0.05)  # oscillate
                obs_dist = 1.5
                if abs(angle - obs_angle) < 0.1:
                    r = min(r, obs_dist)

            ranges.append(r)

        msg.ranges = ranges
        msg.intensities = [100.0] * num_readings

        try:
            self.pub.publish(msg)
        except Exception:
            return
        self.frame_count += 1

        if self.frame_count % 50 == 0:
            self.get_logger().info(f'Published {self.frame_count} scans')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz')
    ap.add_argument('--obstacle', action='store_true', help='Add moving obstacle')
    ap.add_argument('--topic', type=str, default=None, help='ROS 2 topic to publish to')
    args = ap.parse_args()

    rclpy.init()
    node = DummyLidarPublisher(rate=args.rate, add_obstacle=args.obstacle, topic=args.topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
