#!/usr/bin/env python3

"""
Convert PointCloud2 to LaserScan.

Subscribes to:
  /carla/ego_vehicle/lidar_2d (sensor_msgs/msg/PointCloud2)

Publishes:
  /scan (sensor_msgs/msg/LaserScan)
"""

import math
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2


class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')
        
        # Parameters
        self.declare_parameter('input_topic', '/carla/ego_vehicle/lidar_2d')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('min_height', -3.0)
        self.declare_parameter('max_height', 3.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0087)  # ~0.5 degrees
        self.declare_parameter('range_min', 0.5)
        self.declare_parameter('range_max', 100.0)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        
        # Calculate number of beams
        self.num_beams = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # Subscriber and Publisher
        self.pc_sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )
        
        self.scan_pub = self.create_publisher(LaserScan, output_topic, 10)
        
        self.get_logger().info(f'PointCloud to LaserScan converter started')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Height range: [{self.min_height}, {self.max_height}]')
        self.get_logger().info(f'  Angle range: [{self.angle_min:.2f}, {self.angle_max:.2f}] rad')
        self.get_logger().info(f'  Number of beams: {self.num_beams}')
    
    def pointcloud_callback(self, cloud_msg: PointCloud2):
        """Convert PointCloud2 to LaserScan."""
        
        # Initialize LaserScan message
        scan = LaserScan()
        scan.header = cloud_msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Initialize ranges with infinity
        ranges = [float('inf')] * self.num_beams
        
        # Read point cloud data
        try:
            for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point
                
                # Filter by height
                if z < self.min_height or z > self.max_height:
                    continue
                
                # Calculate range and angle
                range_val = math.sqrt(x * x + y * y)
                
                # Check range limits
                if range_val < self.range_min or range_val > self.range_max:
                    continue
                
                # Calculate angle (atan2 returns [-pi, pi])
                angle = math.atan2(y, x)
                
                # Check if angle is within bounds
                if angle < self.angle_min or angle > self.angle_max:
                    continue
                
                # Find corresponding beam index
                beam_index = int((angle - self.angle_min) / self.angle_increment)
                
                # Clamp index to valid range
                if beam_index < 0:
                    beam_index = 0
                elif beam_index >= self.num_beams:
                    beam_index = self.num_beams - 1
                
                # Keep minimum range for this beam (closest obstacle)
                if range_val < ranges[beam_index]:
                    ranges[beam_index] = range_val
        
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
            return
        
        scan.ranges = ranges
        scan.intensities = []  # No intensity data
        
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
