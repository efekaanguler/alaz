#!/usr/bin/env python3

from threading import Lock
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover
    ort = None


def _bbox_to_xyxy(det: Detection2D) -> Tuple[float, float, float, float]:
    if hasattr(det.bbox.center, 'position'):
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
    else:
        cx = float(det.bbox.center.x)
        cy = float(det.bbox.center.y)

    w = float(det.bbox.size_x)
    h = float(det.bbox.size_y)

    x1 = cx - (w * 0.5)
    y1 = cy - (h * 0.5)
    x2 = cx + (w * 0.5)
    y2 = cy + (h * 0.5)
    return x1, y1, x2, y2


def _set_bbox(det: Detection2D, x1: float, y1: float, x2: float, y2: float) -> None:
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    w = max(0.0, x2 - x1)
    h = max(0.0, y2 - y1)

    det.bbox.size_x = float(w)
    det.bbox.size_y = float(h)

    if hasattr(det.bbox.center, 'position'):
        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
    else:
        det.bbox.center.x = float(cx)
        det.bbox.center.y = float(cy)

    if hasattr(det.bbox.center, 'theta'):
        det.bbox.center.theta = 0.0


def _set_hypothesis(result: ObjectHypothesisWithPose, class_id: str, score: float) -> None:
    if hasattr(result, 'hypothesis'):
        if hasattr(result.hypothesis, 'class_id'):
            result.hypothesis.class_id = class_id
        elif hasattr(result.hypothesis, 'id'):
            result.hypothesis.id = 0
        result.hypothesis.score = float(score)
    else:
        if hasattr(result, 'class_id'):
            result.class_id = class_id
        elif hasattr(result, 'id'):
            result.id = 0
        if hasattr(result, 'score'):
            result.score = float(score)


class TrafficLightClassifierNode(Node):
    def __init__(self) -> None:
        super().__init__('traffic_light_classifier')

        self.declare_parameter('model_path', '')
        self.declare_parameter('label_path', '')
        self.declare_parameter('build_only', False)
        self.declare_parameter('use_onnx', True)
        self.declare_parameter('onnx_input_height', 96)
        self.declare_parameter('onnx_input_width', 96)
        self.declare_parameter('min_color_ratio', 0.01)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.label_path = self.get_parameter('label_path').get_parameter_value().string_value
        self.build_only = bool(self.get_parameter('build_only').get_parameter_value().bool_value)
        self.use_onnx = bool(self.get_parameter('use_onnx').get_parameter_value().bool_value)
        self.default_input_h = int(self.get_parameter('onnx_input_height').get_parameter_value().integer_value)
        self.default_input_w = int(self.get_parameter('onnx_input_width').get_parameter_value().integer_value)
        self.min_color_ratio = float(self.get_parameter('min_color_ratio').get_parameter_value().double_value)

        self.labels = self._load_labels(self.label_path)

        self.ort_session = None
        self.ort_input_name = ''
        self.onnx_channels_first = True
        self.onnx_channels = 3
        self.onnx_input_h = self.default_input_h
        self.onnx_input_w = self.default_input_w

        self._init_onnx()

        if self.build_only:
            self.get_logger().info('build_only=true, exiting after model initialization')
            self.create_timer(0.2, self._shutdown_once)
            return

        self.bridge = CvBridge()
        self.image_lock = Lock()
        self.latest_image = None

        self.sub_image = self.create_subscription(Image, '~/input/image', self._on_image, 10)
        self.sub_rois = self.create_subscription(Detection2DArray, '~/input/rois', self._on_rois, 10)
        self.pub_signals = self.create_publisher(Detection2DArray, '~/output/traffic_signals', 10)

        backend = 'onnx' if self.ort_session is not None else 'hsv_fallback'
        self.get_logger().info(
            f'traffic light classifier started, backend={backend}, '
            f'model_path={self.model_path if self.model_path else "<none>"}'
        )

    def _shutdown_once(self) -> None:
        rclpy.shutdown()

    def _load_labels(self, label_path: str) -> List[str]:
        if label_path == '':
            return ['unknown', 'green', 'yellow', 'red']

        try:
            with open(label_path, 'r', encoding='utf-8') as f:
                labels = [line.strip() for line in f if line.strip()]
            if len(labels) > 0:
                return labels
        except OSError:
            self.get_logger().warning(f'label file not found: {label_path}')

        return ['unknown', 'green', 'yellow', 'red']

    def _init_onnx(self) -> None:
        if not self.use_onnx:
            self.get_logger().info('use_onnx=false, classifier will use HSV fallback')
            return

        if ort is None:
            self.get_logger().warning('onnxruntime is not installed, classifier will use HSV fallback')
            return

        if self.model_path == '':
            self.get_logger().warning('model_path is empty, classifier will use HSV fallback')
            return

        try:
            providers = ['CPUExecutionProvider']
            self.ort_session = ort.InferenceSession(self.model_path, providers=providers)
            input_meta = self.ort_session.get_inputs()[0]
            self.ort_input_name = input_meta.name
            self._parse_onnx_input_shape(input_meta.shape)
            self.get_logger().info(
                f'ONNX model loaded: input=({self.onnx_input_h}x{self.onnx_input_w}), '
                f'channels={self.onnx_channels}, channels_first={self.onnx_channels_first}'
            )
        except Exception as ex:  # pylint: disable=broad-except
            self.ort_session = None
            self.get_logger().warning(f'failed to initialize ONNX model, using HSV fallback: {ex}')

    def _parse_onnx_input_shape(self, shape) -> None:
        if not isinstance(shape, list) or len(shape) != 4:
            return

        if isinstance(shape[1], int) and shape[1] in (1, 3):
            self.onnx_channels_first = True
            self.onnx_channels = shape[1]
            if isinstance(shape[2], int) and shape[2] > 0:
                self.onnx_input_h = shape[2]
            if isinstance(shape[3], int) and shape[3] > 0:
                self.onnx_input_w = shape[3]
            return

        if isinstance(shape[3], int) and shape[3] in (1, 3):
            self.onnx_channels_first = False
            self.onnx_channels = shape[3]
            if isinstance(shape[1], int) and shape[1] > 0:
                self.onnx_input_h = shape[1]
            if isinstance(shape[2], int) and shape[2] > 0:
                self.onnx_input_w = shape[2]

    def _on_image(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as ex:  # pylint: disable=broad-except
            self.get_logger().warning(f'image conversion failed: {ex}')
            return

        with self.image_lock:
            self.latest_image = image

    def _on_rois(self, msg: Detection2DArray) -> None:
        with self.image_lock:
            if self.latest_image is None:
                return
            frame = self.latest_image.copy()

        out = Detection2DArray()
        out.header = msg.header

        height, width = frame.shape[:2]

        for i, det in enumerate(msg.detections):
            x1, y1, x2, y2 = _bbox_to_xyxy(det)
            ix1 = int(np.clip(np.floor(x1), 0, width - 1))
            iy1 = int(np.clip(np.floor(y1), 0, height - 1))
            ix2 = int(np.clip(np.ceil(x2), 0, width - 1))
            iy2 = int(np.clip(np.ceil(y2), 0, height - 1))

            classified_det = Detection2D()
            classified_det.header = msg.header
            _set_bbox(classified_det, float(ix1), float(iy1), float(ix2), float(iy2))

            if hasattr(classified_det, 'id'):
                source_id = ''
                if hasattr(det, 'id'):
                    source_id = str(det.id)
                classified_det.id = source_id if source_id != '' else str(i)

            if ix2 <= ix1 or iy2 <= iy1:
                label, score = 'unknown', 0.0
            else:
                crop = frame[iy1:iy2, ix1:ix2]
                label, score = self._classify(crop)

            result = ObjectHypothesisWithPose()
            _set_hypothesis(result, label, score)
            classified_det.results.append(result)
            out.detections.append(classified_det)

        self.pub_signals.publish(out)

    def _classify(self, crop: np.ndarray) -> Tuple[str, float]:
        if crop.size == 0:
            return 'unknown', 0.0

        if self.ort_session is not None:
            try:
                return self._classify_with_onnx(crop)
            except Exception as ex:  # pylint: disable=broad-except
                self.get_logger().warning(f'onnx inference failed, using HSV fallback: {ex}')

        return self._classify_with_hsv(crop)

    def _classify_with_onnx(self, crop: np.ndarray) -> Tuple[str, float]:
        resized = cv2.resize(crop, (self.onnx_input_w, self.onnx_input_h), interpolation=cv2.INTER_LINEAR)

        if self.onnx_channels == 1:
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            data = gray.astype(np.float32) / 255.0
            if self.onnx_channels_first:
                data = data[np.newaxis, :, :]
            else:
                data = data[:, :, np.newaxis]
        else:
            data = resized.astype(np.float32) / 255.0
            if self.onnx_channels_first:
                data = np.transpose(data, (2, 0, 1))

        data = np.expand_dims(data, axis=0)

        outputs = self.ort_session.run(None, {self.ort_input_name: data})
        if len(outputs) == 0:
            raise RuntimeError('onnx output is empty')

        logits = np.array(outputs[0]).reshape(-1)
        if logits.size == 0:
            raise RuntimeError('onnx logits are empty')

        probs = np.exp(logits - np.max(logits))
        denom = float(np.sum(probs))
        if denom <= 0.0:
            raise RuntimeError('invalid softmax denominator')
        probs /= denom

        idx = int(np.argmax(probs))
        score = float(probs[idx])
        label = self.labels[idx] if idx < len(self.labels) else f'class_{idx}'
        return label, score

    def _classify_with_hsv(self, crop: np.ndarray) -> Tuple[str, float]:
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        area = float(max(1, crop.shape[0] * crop.shape[1]))

        red_1 = cv2.inRange(hsv, (0, 80, 60), (12, 255, 255))
        red_2 = cv2.inRange(hsv, (160, 80, 60), (180, 255, 255))
        yellow = cv2.inRange(hsv, (15, 80, 60), (42, 255, 255))
        green = cv2.inRange(hsv, (40, 80, 60), (95, 255, 255))

        red_ratio = max(float(cv2.countNonZero(red_1)), float(cv2.countNonZero(red_2))) / area
        yellow_ratio = float(cv2.countNonZero(yellow)) / area
        green_ratio = float(cv2.countNonZero(green)) / area

        candidates = {
            'red': red_ratio,
            'yellow': yellow_ratio,
            'green': green_ratio,
        }

        label = max(candidates, key=candidates.get)
        score = candidates[label]

        if score < self.min_color_ratio:
            return 'unknown', score

        return label, score


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficLightClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
