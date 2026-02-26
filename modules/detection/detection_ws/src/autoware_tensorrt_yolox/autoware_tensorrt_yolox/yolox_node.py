#!/usr/bin/env python3
from pathlib import Path
from typing import Tuple
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from tier4_perception_msgs.msg import DetectedObjectsWithFeature, DetectedObjectWithFeature
from autoware_perception_msgs.msg import DetectedObject, ObjectClassification, Shape

try:
    import onnxruntime as ort
except ImportError:
    ort = None

# COCO class_id -> Autoware label
COCO_TO_AUTOWARE = {
    0:  ObjectClassification.PEDESTRIAN,
    1:  ObjectClassification.BICYCLE,
    2:  ObjectClassification.CAR,
    3:  ObjectClassification.MOTORCYCLE,
    5:  ObjectClassification.BUS,
    7:  ObjectClassification.TRUCK,
}

def _letterbox(image, new_shape, color=(114, 114, 114)):
    shape = image.shape[:2]
    new_h, new_w = new_shape
    ratio = min(new_h / shape[0], new_w / shape[1])
    resized_w = int(round(shape[1] * ratio))
    resized_h = int(round(shape[0] * ratio))
    pad_w = (new_w - resized_w) / 2.0
    pad_h = (new_h - resized_h) / 2.0
    if (shape[1], shape[0]) != (resized_w, resized_h):
        image = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    top = int(round(pad_h - 0.1))
    bottom = int(round(pad_h + 0.1))
    left = int(round(pad_w - 0.1))
    right = int(round(pad_w + 0.1))
    return cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color), ratio, pad_w, pad_h


class YoloXNode(Node):
    def __init__(self):
        super().__init__('yolox_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('label_path', '')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('score_threshold', 0.30)
        self.declare_parameter('nms_threshold', 0.45)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.label_path = self.get_parameter('label_path').get_parameter_value().string_value
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value
        self.score_threshold = self.get_parameter('score_threshold').get_parameter_value().double_value
        self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value

        if ort is None:
            raise RuntimeError("pip install onnxruntime")
        self.session = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

        self.bridge = CvBridge()
        self.pub = self.create_publisher(DetectedObjectsWithFeature, '~/output/objects', 10)
        self.sub = self.create_subscription(Image, '~/input/image', self._on_image, 10)
        self.get_logger().info(f"YOLOX Node ONLINE: {self.model_path}")

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detections = self._infer(frame)

            out = DetectedObjectsWithFeature()
            out.header = msg.header

            for (x1, y1, x2, y2, score, class_id) in detections:
                owf = DetectedObjectWithFeature()

                det = DetectedObject()
                det.existence_probability = float(score)

                cls = ObjectClassification()
                cls.label = COCO_TO_AUTOWARE.get(int(class_id), ObjectClassification.UNKNOWN)
                cls.probability = float(score)
                det.classification.append(cls)

                det.kinematics.pose_with_covariance.pose.position.x = float((x1 + x2) / 2)
                det.kinematics.pose_with_covariance.pose.position.y = float((y1 + y2) / 2)
                det.kinematics.pose_with_covariance.pose.position.z = 0.0
                det.kinematics.pose_with_covariance.pose.orientation.w = 1.0

                det.shape.type = Shape.BOUNDING_BOX
                det.shape.dimensions.x = float(x2 - x1)
                det.shape.dimensions.y = float(y2 - y1)
                det.shape.dimensions.z = 1.0

                owf.object = det
                owf.feature.roi.x_offset = int(max(x1, 0))
                owf.feature.roi.y_offset = int(max(y1, 0))
                owf.feature.roi.width = int(x2 - x1)
                owf.feature.roi.height = int(y2 - y1)

                out.feature_objects.append(owf)

            self.pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def _infer(self, frame):
        img, ratio, pad_w, pad_h = _letterbox(frame, (self.input_height, self.input_width))
        blob = img.astype(np.float32).transpose(2, 0, 1)
        blob = np.expand_dims(blob, 0) / 255.0

        preds = self.session.run(None, {self.input_name: blob})[0]
        if preds.shape[0] == 1:
            preds = preds[0]
        if preds.shape[0] < preds.shape[1] and preds.shape[0] < 100:
            preds = preds.transpose(1, 0)

        if preds.shape[1] == 84:  # YOLOv8
            boxes = preds[:, :4]
            cls = preds[:, 4:]
            class_ids = np.argmax(cls, axis=1)
            scores = cls[np.arange(cls.shape[0]), class_ids]
        else:  # YOLOX
            boxes = preds[:, :4]
            obj_scores = preds[:, 4]
            class_scores = preds[:, 5:]
            class_ids = np.argmax(class_scores, axis=1)
            scores = obj_scores * class_scores[np.arange(class_scores.shape[0]), class_ids]

        keep = scores > self.score_threshold
        boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]

        x1 = (boxes[:, 0] - boxes[:, 2] / 2 - pad_w) / ratio
        y1 = (boxes[:, 1] - boxes[:, 3] / 2 - pad_h) / ratio
        w = boxes[:, 2] / ratio
        h = boxes[:, 3] / ratio

        if len(scores) == 0:
            return []

        nms_idxs = cv2.dnn.NMSBoxes(
            [[float(x), float(y), float(bw), float(bh)] for x, y, bw, bh in zip(x1, y1, w, h)],
            scores.tolist(), self.score_threshold, self.nms_threshold
        )

        results = []
        if len(nms_idxs) > 0:
            for i in nms_idxs.flatten():
                results.append((x1[i], y1[i], x1[i] + w[i], y1[i] + h[i], scores[i], class_ids[i]))
        return results


def main(args=None):
    rclpy.init(args=args)
    node = YoloXNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
