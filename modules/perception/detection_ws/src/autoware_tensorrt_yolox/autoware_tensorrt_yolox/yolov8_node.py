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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover
    ort = None


def _letterbox(image: np.ndarray, new_shape: Tuple[int, int], color: Tuple[int, int, int]) -> Tuple[np.ndarray, float, float, float]:
    shape = image.shape[:2]  # (h, w)
    if shape[0] == 0 or shape[1] == 0:
        raise ValueError('input image has invalid shape')

    new_h, new_w = new_shape
    ratio = min(new_h / shape[0], new_w / shape[1])

    resized_w = int(round(shape[1] * ratio))
    resized_h = int(round(shape[0] * ratio))

    pad_w = new_w - resized_w
    pad_h = new_h - resized_h

    pad_w /= 2.0
    pad_h /= 2.0

    if (shape[1], shape[0]) != (resized_w, resized_h):
        image = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

    top = int(round(pad_h - 0.1))
    bottom = int(round(pad_h + 0.1))
    left = int(round(pad_w - 0.1))
    right = int(round(pad_w + 0.1))

    bordered = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return bordered, ratio, pad_w, pad_h


def _set_bbox(bbox: BoundingBox2D, cx: float, cy: float, w: float, h: float) -> None:
    bbox.size_x = float(w)
    bbox.size_y = float(h)

    if hasattr(bbox.center, 'position'):
        bbox.center.position.x = float(cx)
        bbox.center.position.y = float(cy)
    elif hasattr(bbox.center, 'x'):
        bbox.center.x = float(cx)
        bbox.center.y = float(cy)

    if hasattr(bbox.center, 'theta'):
        bbox.center.theta = 0.0


def _set_hypothesis(result: ObjectHypothesisWithPose, class_id: int, score: float) -> None:
    if hasattr(result, 'hypothesis'):
        if hasattr(result.hypothesis, 'class_id'):
            result.hypothesis.class_id = str(class_id)
        elif hasattr(result.hypothesis, 'id'):
            result.hypothesis.id = int(class_id)
        result.hypothesis.score = float(score)
    else:
        if hasattr(result, 'class_id'):
            result.class_id = str(class_id)
        elif hasattr(result, 'id'):
            result.id = int(class_id)
        if hasattr(result, 'score'):
            result.score = float(score)


class YoloV8Node(Node):
    def __init__(self) -> None:
        super().__init__('yolov8_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('label_path', '')
        self.declare_parameter('color_map_path', '')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('score_threshold', 0.35)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        # Keep this dynamically typed so launch can pass int, string, or arrays.
        self.declare_parameter(
            'class_allowlist',
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.input_width = int(self.get_parameter('input_width').get_parameter_value().integer_value)
        self.input_height = int(self.get_parameter('input_height').get_parameter_value().integer_value)
        self.score_threshold = float(self.get_parameter('score_threshold').get_parameter_value().double_value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').get_parameter_value().double_value)
        requested_device = self.get_parameter('device').get_parameter_value().string_value.lower()
        self.device = self._resolve_device(requested_device)
        self.class_allowlist_raw = self._class_allowlist_to_text(self.get_parameter('class_allowlist').get_parameter_value())

        label_path = self.get_parameter('label_path').get_parameter_value().string_value
        color_map_path = self.get_parameter('color_map_path').get_parameter_value().string_value
        self.labels = self._load_labels(label_path)
        self.color_map = self._load_color_map(color_map_path)
        self.allowed_class_ids = self._parse_class_allowlist(self.class_allowlist_raw)

        if not Path(self.model_path).is_file():
            raise RuntimeError(f'model file not found: {self.model_path}')

        self.net = None
        self.ort_session = None
        self.ort_input_name = ''
        self.inference_backend = 'unknown'
        self._initialize_inference_backend()

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, '~/output/objects', 10)
        self.sub = self.create_subscription(
            Image, '~/input/image', self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Loaded model={self.model_path}, backend={self.inference_backend}, '
            f'input_size=({self.input_width},{self.input_height}), '
            f'thresholds=(score={self.score_threshold}, nms={self.nms_threshold}), device={self.device}, '
            f'class_allowlist={self.class_allowlist_raw if self.class_allowlist_raw else "ALL"}'
        )

    def _class_allowlist_to_text(self, value) -> str:
        if value.type == ParameterType.PARAMETER_NOT_SET:
            return '0,2'
        if value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        if value.type == ParameterType.PARAMETER_INTEGER:
            return str(value.integer_value)
        if value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return ','.join(value.string_array_value)
        if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return ','.join(str(v) for v in value.integer_array_value)

        self.get_logger().warning('Unsupported class_allowlist type, using default "0,2"')
        return '0,2'

    def _parse_class_allowlist(self, raw: str) -> Optional[Set[int]]:
        text = raw.strip()
        if text == '':
            return None

        label_to_id = {label.lower(): idx for idx, label in enumerate(self.labels)}
        allow_ids: Set[int] = set()

        for token in text.split(','):
            value = token.strip()
            if value == '':
                continue

            try:
                allow_ids.add(int(value))
                continue
            except ValueError:
                pass

            label_id = label_to_id.get(value.lower())
            if label_id is None:
                self.get_logger().warning(f'class token "{value}" is invalid and ignored')
                continue

            allow_ids.add(label_id)

        if len(allow_ids) == 0:
            self.get_logger().warning('class_allowlist has no valid entries, filtering is disabled')
            return None

        self.get_logger().info(f'class filter ids: {sorted(allow_ids)}')
        return allow_ids

    def _configure_backend(self) -> None:
        if self.net is None:
            raise RuntimeError('OpenCV DNN backend is not initialized')

        if self.device != 'cuda':
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            return

        try:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
            self.get_logger().info('Using CUDA backend for OpenCV DNN.')
        except cv2.error:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().warning('CUDA backend unavailable, falling back to CPU.')

    def _resolve_device(self, requested: str) -> str:
        if requested not in {'auto', 'cpu', 'cuda'}:
            self.get_logger().warning(
                f'unsupported device "{requested}", falling back to auto selection'
            )
            requested = 'auto'

        if requested != 'auto':
            return requested

        if ort is not None and 'CUDAExecutionProvider' in ort.get_available_providers():
            return 'cuda'

        try:
            if hasattr(cv2, 'cuda') and cv2.cuda.getCudaEnabledDeviceCount() > 0:
                return 'cuda'
        except cv2.error:
            pass

        return 'cpu'

    def _initialize_inference_backend(self) -> None:
        if ort is not None:
            try:
                providers = ['CPUExecutionProvider']
                if self.device == 'cuda':
                    available = set(ort.get_available_providers())
                    if 'CUDAExecutionProvider' in available:
                        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
                    else:
                        raise RuntimeError('ONNX Runtime CUDA provider unavailable')

                self.ort_session = ort.InferenceSession(self.model_path, providers=providers)
                self.ort_input_name = self.ort_session.get_inputs()[0].name
                self.inference_backend = 'onnxruntime'
                self.get_logger().info(f'Using ONNX Runtime backend with providers={providers}.')
                return
            except Exception as ex:  # pylint: disable=broad-except
                self.ort_session = None
                self.get_logger().warning(
                    f'ONNX Runtime init failed, falling back to OpenCV DNN: {ex}'
                )
        else:
            self.get_logger().warning('onnxruntime is not installed, using OpenCV DNN backend.')

        self.net = cv2.dnn.readNet(self.model_path)
        self._configure_backend()
        self.inference_backend = 'opencv_dnn'

    def _load_labels(self, label_path: str) -> List[str]:
        if not label_path:
            return []

        p = Path(label_path)
        if not p.is_file():
            self.get_logger().warning(f'label file not found: {label_path}')
            return []

        with p.open('r', encoding='utf-8') as f:
            labels = [line.strip() for line in f if line.strip()]
        return labels

    def _load_color_map(self, color_map_path: str) -> dict:
        if not color_map_path:
            return {}

        p = Path(color_map_path)
        if not p.is_file():
            self.get_logger().warning(f'color map file not found: {color_map_path}')
            return {}

        try:
            with p.open('r', encoding='utf-8') as f:
                raw = json.load(f)

            if isinstance(raw, dict):
                return raw
            return {}
        except (OSError, json.JSONDecodeError):
            self.get_logger().warning(f'failed to parse color map file: {color_map_path}')
            return {}

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as ex:  # pylint: disable=broad-except
            self.get_logger().error(f'cv_bridge conversion failed: {ex}')
            return

        try:
            detections = self._infer(frame)
        except Exception as ex:  # pylint: disable=broad-except
            self.get_logger().error(f'inference failed: {ex}')
            return

        out = Detection2DArray()
        out.header = msg.header

        for i, (x1, y1, x2, y2, score, class_id) in enumerate(detections):
            det = Detection2D()
            det.header = msg.header
            _set_bbox(det.bbox, (x1 + x2) * 0.5, (y1 + y2) * 0.5, max(0.0, x2 - x1), max(0.0, y2 - y1))

            result = ObjectHypothesisWithPose()
            _set_hypothesis(result, class_id, score)
            det.results.append(result)

            if hasattr(det, 'id'):
                det.id = f'{class_id}_{i}'

            out.detections.append(det)

        self.pub.publish(out)

    def _infer(self, frame: np.ndarray) -> List[Tuple[float, float, float, float, float, int]]:
        image, ratio, pad_w, pad_h = _letterbox(
            frame,
            new_shape=(self.input_height, self.input_width),
            color=(114, 114, 114),
        )

        blob = cv2.dnn.blobFromImage(
            image,
            scalefactor=1.0 / 255.0,
            size=(self.input_width, self.input_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        ).astype(np.float32, copy=False)

        raw = self._run_network(blob)
        pred = self._normalize_prediction(raw)

        # Handle both [N, C] and [C, N] layouts.
        if pred.shape[0] <= 128 and pred.shape[0] < pred.shape[1]:
            pred = pred.transpose(1, 0)

        if pred.shape[1] < 6:
            raise RuntimeError(f'unsupported output channels: {pred.shape[1]}')

        boxes = pred[:, :4]

        if pred.shape[1] >= 85:
            obj = pred[:, 4]
            cls = pred[:, 5:]
            cls_ids = np.argmax(cls, axis=1)
            cls_scores = cls[np.arange(cls.shape[0]), cls_ids]
            scores = obj * cls_scores
        elif pred.shape[1] == 84:
            cls = pred[:, 4:]
            cls_ids = np.argmax(cls, axis=1)
            scores = cls[np.arange(cls.shape[0]), cls_ids]
        else:
            scores = pred[:, 4]
            cls_ids = np.rint(pred[:, 5]).astype(int)

        keep = scores >= self.score_threshold
        if self.allowed_class_ids is not None:
            allowed_ids = np.array(sorted(self.allowed_class_ids), dtype=np.int32)
            keep = np.logical_and(keep, np.isin(cls_ids, allowed_ids))

        if not np.any(keep):
            return []

        boxes = boxes[keep]
        scores = scores[keep]
        cls_ids = cls_ids[keep]

        # cx, cy, w, h -> x1, y1, x2, y2 in model-input space.
        x1 = boxes[:, 0] - (boxes[:, 2] * 0.5)
        y1 = boxes[:, 1] - (boxes[:, 3] * 0.5)
        x2 = boxes[:, 0] + (boxes[:, 2] * 0.5)
        y2 = boxes[:, 1] + (boxes[:, 3] * 0.5)

        # Undo letterbox transform.
        x1 = (x1 - pad_w) / ratio
        y1 = (y1 - pad_h) / ratio
        x2 = (x2 - pad_w) / ratio
        y2 = (y2 - pad_h) / ratio

        x1 = np.clip(x1, 0, frame.shape[1] - 1)
        y1 = np.clip(y1, 0, frame.shape[0] - 1)
        x2 = np.clip(x2, 0, frame.shape[1] - 1)
        y2 = np.clip(y2, 0, frame.shape[0] - 1)

        nms_boxes = []
        for i in range(len(scores)):
            w = max(0.0, x2[i] - x1[i])
            h = max(0.0, y2[i] - y1[i])
            nms_boxes.append([float(x1[i]), float(y1[i]), float(w), float(h)])

        idxs = cv2.dnn.NMSBoxes(nms_boxes, scores.tolist(), self.score_threshold, self.nms_threshold)
        if len(idxs) == 0:
            return []

        idxs = np.array(idxs).reshape(-1)

        detections: List[Tuple[float, float, float, float, float, int]] = []
        for idx in idxs:
            detections.append(
                (
                    float(x1[idx]),
                    float(y1[idx]),
                    float(x2[idx]),
                    float(y2[idx]),
                    float(scores[idx]),
                    int(cls_ids[idx]),
                )
            )

        return detections

    def _run_network(self, blob: np.ndarray):
        if self.ort_session is not None:
            outputs = self.ort_session.run(None, {self.ort_input_name: blob})
            if len(outputs) == 0:
                raise RuntimeError('onnxruntime output is empty')
            return outputs[0]

        if self.net is None:
            raise RuntimeError('no inference backend is initialized')

        self.net.setInput(blob)
        return self.net.forward()

    def _normalize_prediction(self, raw) -> np.ndarray:
        if isinstance(raw, Sequence) and not isinstance(raw, np.ndarray):
            if len(raw) == 0:
                raise RuntimeError('model output is empty')
            raw = raw[0]

        pred = np.asarray(raw)

        while pred.ndim > 2 and pred.shape[0] == 1:
            pred = pred[0]

        if pred.ndim != 2:
            raise RuntimeError(f'unexpected output shape: {pred.shape}')

        return pred


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloV8Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
