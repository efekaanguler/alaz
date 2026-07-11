#!/usr/bin/env python3
"""Visualize detection results from the perception pipeline.

Subscribes to detection topics and draws bounding boxes on the camera image.
Useful for debugging and verifying the detection pipeline output.

Usage:
    ros2 run perception visualize_detections.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2DArray


class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__('detection_visualizer')

        self.declare_parameter('image_topic', '/sensing/image_raw')
        self.declare_parameter('detection_topic', '/rois0')

        image_topic = self.get_parameter('image_topic').value
        det_topic = self.get_parameter('detection_topic').value

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.det2d_sub = self.create_subscription(
            Detection2DArray, det_topic, self.det2d_callback, 10)

        self.latest_image = None
        self.latest_dets = []

        self.get_logger().info(f'Subscribed to {image_topic} and {det_topic}')

        # Display timer (30 fps)
        self.timer = self.create_timer(1.0 / 30.0, self.display_callback)

        # Color map for classes
        self.colors = {
            'person': (56, 56, 255),
            'car': (31, 112, 255),
            'bus': (255, 144, 30),
            'truck': (180, 105, 255),
            'bicycle': (0, 200, 200),
            'motorbike': (0, 180, 180),
            'traffic_light_red': (0, 0, 255),
            'traffic_light_green': (0, 220, 0),
        }

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        try:
            if msg.encoding == 'bgr8':
                self.latest_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self.latest_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def det2d_callback(self, msg):
        detections = []
        for det in msg.detections:
            label = 'unknown'
            score = 0.0
            if det.results:
                hyp = det.results[0].hypothesis if hasattr(det.results[0], 'hypothesis') else det.results[0]
                label = str(getattr(hyp, 'class_id', getattr(hyp, 'id', 'unknown')))
                score = float(getattr(hyp, 'score', 0.0))

            center = det.bbox.center
            if hasattr(center, 'position'):
                cx = float(center.position.x)
                cy = float(center.position.y)
            else:
                cx = float(center.x)
                cy = float(center.y)
            w = float(det.bbox.size_x)
            h = float(det.bbox.size_y)
            detections.append({
                'label': label,
                'score': score,
                'x1': cx - w * 0.5,
                'y1': cy - h * 0.5,
                'x2': cx + w * 0.5,
                'y2': cy + h * 0.5,
            })
        self.latest_dets = detections

    def display_callback(self):
        if self.latest_image is None:
            return

        canvas = self.latest_image.copy()

        for det in self.latest_dets:
            label = det.get('label', 'unknown')
            score = det.get('score', 0.0)
            x1 = int(det.get('x1', 0))
            y1 = int(det.get('y1', 0))
            x2 = int(det.get('x2', 0))
            y2 = int(det.get('y2', 0))

            color = self.colors.get(label, (56, 200, 56))
            txt = f'{label} {score:.0%}'

            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(canvas, (x1, y1-th-6), (x1+tw+4, y1), color, -1)
            cv2.putText(canvas, txt, (x1+2, y1-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        n = len(self.latest_dets)
        cv2.putText(canvas, f'Dets: {n}', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 230), 2, cv2.LINE_AA)

        cv2.imshow('Detection Visualizer', canvas)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizer()   # kendi node class adın
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
