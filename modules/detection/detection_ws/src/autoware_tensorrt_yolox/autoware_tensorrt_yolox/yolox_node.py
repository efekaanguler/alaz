#!/usr/bin/env python3

import json
from pathlib import Path
from typing import List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    import onnxruntime as ort
except ImportError:
    ort = None

def _letterbox(image: np.ndarray, new_shape: Tuple[int, int], color: Tuple[int, int, int]) -> Tuple[np.ndarray, float, float, float]:
    shape = image.shape[:2]
    new_h, new_w = new_shape
    ratio = min(new_h / shape[0], new_w / shape[1])
    resized_w, resized_h = int(round(shape[1] * ratio)), int(round(shape[0] * ratio))
    pad_w, pad_h = (new_w - resized_w) / 2.0, (new_h - resized_h) / 2.0
    if (shape[1], shape[0]) != (resized_w, resized_h):
        image = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
    left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
    return cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color), ratio, pad_w, pad_h

def _set_bbox(bbox: BoundingBox2D, cx: float, cy: float, w: float, h: float) -> None:
    bbox.size_x, bbox.size_y = float(w), float(h)
    if hasattr(bbox.center, 'position'):
        bbox.center.position.x, bbox.center.position.y = float(cx), float(cy)
    else:
        bbox.center.x, bbox.center.y = float(cx), float(cy)

def _set_hypothesis(result: ObjectHypothesisWithPose, class_id: int, score: float) -> None:
    if hasattr(result, 'hypothesis'):
        result.hypothesis.class_id, result.hypothesis.score = str(class_id), float(score)
    else:
        result.id, result.score = str(class_id), float(score)

class YoloXNode(Node):
    def __init__(self) -> None:
        super().__init__('yolox_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('label_path', '')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('score_threshold', 0.35)
        self.declare_parameter('nms_threshold', 0.45)
        
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.label_path = self.get_parameter('label_path').get_parameter_value().string_value
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value
        self.score_threshold = self.get_parameter('score_threshold').get_parameter_value().double_value
        self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value

        self.labels = self._load_labels(self.label_path)
        
        if ort is None: raise RuntimeError("pip install onnxruntime is required")
        self.ort_session = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
        self.ort_input_name = self.ort_session.get_inputs()[0].name

        self.bridge, self.pub = CvBridge(), self.create_publisher(Detection2DArray, '~/output/objects', 10)
        self.sub = self.create_subscription(Image, '~/input/image', self._on_image, 10)
        self.get_logger().info(f"YOLOX Node ONLINE: {self.model_path}")

    def _load_labels(self, path):
        if not path or not Path(path).exists(): return []
        with open(path, 'r') as f: return [l.strip() for l in f if l.strip()]

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detections = self._infer(frame)
            out = Detection2DArray(header=msg.header)
            for i, (x1, y1, x2, y2, score, class_id) in enumerate(detections):
                det = Detection2D(header=msg.header)
                _set_bbox(det.bbox, (x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1)
                res = ObjectHypothesisWithPose()
                _set_hypothesis(res, class_id, score)
                det.results.append(res)
                out.detections.append(det)
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def _infer(self, frame):
        img, ratio, pad_w, pad_h = _letterbox(frame, (self.input_height, self.input_width), (114, 114, 114))
        blob = img.astype(np.float32).transpose(2, 0, 1)
        blob = np.expand_dims(blob, axis=0) / 255.0

        preds = self.ort_session.run(None, {self.ort_input_name: blob})[0]
        if preds.shape[0] == 1: preds = preds[0]
        if preds.shape[0] < preds.shape[1] and preds.shape[0] < 100: preds = preds.transpose(1, 0)
        
        # Hibrit Decode: YOLOv8 (84) veya YOLOX (85+)
        if preds.shape[1] == 84:
            boxes, cls = preds[:, :4], preds[:, 4:]
            class_ids = np.argmax(cls, axis=1)
            scores = cls[np.arange(cls.shape[0]), class_ids]
        else:
            boxes, obj_scores, class_scores = preds[:, :4], preds[:, 4], preds[:, 5:]
            class_ids = np.argmax(class_scores, axis=1)
            scores = obj_scores * class_scores[np.arange(class_scores.shape[0]), class_ids]

        keep = scores > self.score_threshold
        boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]

        x1 = (boxes[:, 0] - boxes[:, 2]/2 - pad_w) / ratio
        y1 = (boxes[:, 1] - boxes[:, 3]/2 - pad_h) / ratio
        w, h = boxes[:, 2] / ratio, boxes[:, 3] / ratio
        
        nms_idxs = cv2.dnn.NMSBoxes([[float(x), float(y), float(width), float(height)] for x,y,width,height in zip(x1, y1, w, h)], 
                                    scores.tolist(), self.score_threshold, self.nms_threshold)
        
        results = []
        if len(nms_idxs) > 0:
            for i in nms_idxs.flatten():
                results.append((x1[i], y1[i], x1[i]+w[i], y1[i]+h[i], scores[i], class_ids[i]))
        return results

def main(args=None):
    rclpy.init(args=args)
    node = YoloXNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()
