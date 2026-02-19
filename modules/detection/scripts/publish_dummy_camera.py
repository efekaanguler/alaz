#!/usr/bin/env python3
"""Publish synthetic camera frames for perception pipeline testing."""

import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DummyCameraPublisher(Node):
    def __init__(self, topic: str, width: int, height: int, fps: float) -> None:
        super().__init__('dummy_camera_publisher')
        self.topic = topic
        self.width = width
        self.height = height
        self.tick = 0

        self.pub = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(1.0 / fps, self._on_timer)
        self.get_logger().info(
            f'publishing synthetic images: topic={topic}, size={width}x{height}, fps={fps}'
        )

    def _make_frame(self) -> np.ndarray:
        x = np.arange(self.width, dtype=np.uint16)
        y = np.arange(self.height, dtype=np.uint16)[:, None]

        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:, :, 0] = ((x + self.tick) % 256).astype(np.uint8)
        frame[:, :, 1] = ((y + (2 * self.tick)) % 256).astype(np.uint8)
        frame[:, :, 2] = ((x[None, :] // 2 + y // 2 + (3 * self.tick)) % 256).astype(np.uint8)
        return frame

    def _on_timer(self) -> None:
        frame = self._make_frame()

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera0'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)
        self.tick = (self.tick + 1) % 10000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Publish synthetic images to a ROS topic.')
    parser.add_argument(
        '--topic',
        default='/sensing/camera/camera0/image_raw',
        help='Output image topic',
    )
    parser.add_argument('--width', type=int, default=640, help='Image width')
    parser.add_argument('--height', type=int, default=640, help='Image height')
    parser.add_argument('--fps', type=float, default=5.0, help='Publish rate')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = DummyCameraPublisher(args.topic, args.width, args.height, args.fps)
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
