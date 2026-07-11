#!/usr/bin/env python3
"""Dummy Camera Image publisher for testing the detection pipeline.

Publishes synthetic camera images on /sensing/image_raw
so that the YOLOv8 detection pipeline can be tested without real hardware.

Generates a simple test pattern with colored rectangles that simulate
objects the detector should recognize.

Usage:
    python3 dummy_camera_publisher.py
    python3 dummy_camera_publisher.py --width 640 --height 480 --fps 15
    python3 dummy_camera_publisher.py --topic /sensing/camera/camera1/image_raw --frame-id camera1_link
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DummyCameraPublisher(Node):
    def __init__(
        self,
        width=640,
        height=480,
        fps=15.0,
        topic='/sensing/image_raw',
        frame_id='camera_center_link',
        node_name='dummy_camera_publisher',
        phase=0.0,
    ):
        super().__init__(node_name)
        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / fps, self.publish_image)
        self.width = width
        self.height = height
        self.frame_id = frame_id
        self.phase = phase
        self.frame_count = 0
        self.get_logger().info(
            f'Dummy camera publisher started: topic={topic}, frame_id={frame_id}, '
            f'{width}x{height} at {fps} fps'
        )

    def _generate_frame(self):
        """Generate a synthetic test frame with colored objects."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Background gradient (sky/road)
        for y in range(self.height):
            ratio = y / self.height
            if ratio < 0.5:
                # Sky (dark blue gradient)
                frame[y, :] = [int(180 * ratio), int(100 * ratio), int(30 + 50 * ratio)]
            else:
                # Road (dark gray)
                gray = int(40 + 30 * (ratio - 0.5))
                frame[y, :] = [gray, gray, gray]

        # Simulated traffic light (red circle on dark background)
        tl_x, tl_y = 300, 100
        # Housing (dark rectangle)
        cv2.rectangle(frame, (tl_x - 20, tl_y - 50), (tl_x + 20, tl_y + 50), (20, 20, 20), -1)
        # Red lamp (top)
        cv2.circle(frame, (tl_x, tl_y - 20), 12, (0, 0, 255), -1)
        # Green lamp (bottom, dim)
        cv2.circle(frame, (tl_x, tl_y + 20), 12, (0, 60, 0), -1)

        # Simulated person (simple rectangle)
        px = 150 + int(50 * np.sin(self.frame_count * 0.02 + self.phase))
        cv2.rectangle(frame, (px, 200), (px + 40, 350), (50, 80, 200), -1)  # body
        cv2.circle(frame, (px + 20, 190), 15, (150, 120, 100), -1)  # head

        # Simulated car (blue rectangle)
        cx = 400 + int(30 * np.sin(self.frame_count * 0.03 + self.phase * 0.5))
        cv2.rectangle(frame, (cx, 280), (cx + 120, 370), (180, 100, 30), -1)  # body
        cv2.rectangle(frame, (cx + 10, 260), (cx + 110, 280), (150, 80, 20), -1)  # roof

        # Frame counter
        cv2.putText(frame, f'Frame {self.frame_count}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        return frame

    def publish_image(self):
        if not rclpy.ok():
            return
        frame = self._generate_frame()

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()

        try:
            self.pub.publish(msg)
        except Exception:
            # Common during shutdown (context invalid / publisher already torn down).
            return
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--topic', type=str, default='/sensing/image_raw')
    ap.add_argument('--frame-id', type=str, default='camera_center_link')
    ap.add_argument('--node-name', type=str, default='dummy_camera_publisher')
    ap.add_argument('--phase', type=float, default=0.0, help='Animation phase offset (radians)')
    args = ap.parse_args()

    rclpy.init()
    node = DummyCameraPublisher(
        width=args.width,
        height=args.height,
        fps=args.fps,
        topic=args.topic,
        frame_id=args.frame_id,
        node_name=args.node_name,
        phase=args.phase,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (Exception, KeyboardInterrupt):
                pass


if __name__ == '__main__':
    main()
