#!/usr/bin/env python3
"""MJPEG webcam stream → ROS 2 sensor_msgs/Image publisher.

Captures frames from the Mac webcam and publishes them as ROS 2 Image messages.
This bridges the Mac camera (which can't be accessed inside Docker) to the
ROS 2 perception pipeline running in the container.

Usage (on Mac, outside Docker):
    source /opt/ros/humble/setup.bash   # or your ROS 2 workspace
    python3 mjpeg_to_ros.py

The node publishes to /sensing/image_raw at ~30 fps by default.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        # Parameters
        self.declare_parameter('device', '0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', '/sensing/image_raw')
        self.declare_parameter('frame_id', 'camera_center_link')

        raw_device = self.get_parameter('device').value
        try:
            # Eğer sayıysa (0, 1 gibi) int'e çevir
            device = int(raw_device)
        except ValueError:
            # Eğer yazıysa (/dev/video0 gibi) olduğu gibi bırak
            device = raw_device

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Open camera
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {device}')
            raise SystemExit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Camera opened: {actual_w}x{actual_h}')

        # Publisher
        self.pub = self.create_publisher(Image, topic, 10)
        self.get_logger().info(f'Publishing to {topic} at {fps} fps')

        # Timer
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Failed to read frame')
            return

        # Convert OpenCV BGR frame to ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
