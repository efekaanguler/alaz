#!/usr/bin/env python3
"""Convert vision_msgs/Detection2DArray to tier4_perception_msgs/DetectedObjectsWithFeature.

`roi_cluster_fusion` expects ROI inputs as `tier4_perception_msgs/DetectedObjectsWithFeature`,
but this module's YOLOv8 + ByteTrack pipeline publishes `vision_msgs/Detection2DArray`.
This adapter bridges that gap by converting 2D bboxes into tier4 ROI feature objects.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray

from autoware_perception_msgs.msg import ObjectClassification

try:
    from tier4_perception_msgs.msg import DetectedObjectWithFeature, DetectedObjectsWithFeature, Feature
except Exception as e:  # pragma: no cover - runtime environment dependent
    DetectedObjectWithFeature = None
    DetectedObjectsWithFeature = None
    Feature = None
    IMPORT_ERROR = e
else:
    IMPORT_ERROR = None


CLASS_UNKNOWN = 0
CLASS_CAR = 1
CLASS_TRUCK = 2
CLASS_BUS = 3
CLASS_TRAILER = 4
CLASS_MOTORCYCLE = 5
CLASS_BICYCLE = 6
CLASS_PEDESTRIAN = 7
CLASS_ANIMAL = 8


def _extract_hypothesis(det: Detection2D) -> Tuple[str, float]:
    if len(det.results) == 0:
        return "unknown", 0.0
    hyp = det.results[0].hypothesis if hasattr(det.results[0], "hypothesis") else det.results[0]
    class_id = getattr(hyp, "class_id", getattr(hyp, "id", "unknown"))
    score = getattr(hyp, "score", 0.0)
    return str(class_id), float(score)


def _bbox_xywh(det: Detection2D) -> Tuple[float, float, float, float]:
    if hasattr(det.bbox.center, "position"):
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
    else:
        cx = float(det.bbox.center.x)
        cy = float(det.bbox.center.y)
    w = max(float(det.bbox.size_x), 0.0)
    h = max(float(det.bbox.size_y), 0.0)
    return cx, cy, w, h


def _clamp_int(v: float) -> int:
    if not math.isfinite(v):
        return 0
    return max(0, int(round(v)))


def _map_object_label(class_id: str) -> int:
    raw = class_id.strip().lower()
    word_map = {
        "person": CLASS_PEDESTRIAN,
        "pedestrian": CLASS_PEDESTRIAN,
        "bicycle": CLASS_BICYCLE,
        "bike": CLASS_BICYCLE,
        "motorbike": CLASS_MOTORCYCLE,
        "motorcycle": CLASS_MOTORCYCLE,
        "car": CLASS_CAR,
        "truck": CLASS_TRUCK,
        "bus": CLASS_BUS,
        "trailer": CLASS_TRAILER,
        "animal": CLASS_ANIMAL,
    }
    if raw in word_map:
        return word_map[raw]

    # COCO IDs as strings from YOLO wrappers
    coco_map = {
        "0": CLASS_PEDESTRIAN,  # person
        "1": CLASS_BICYCLE,     # bicycle
        "2": CLASS_CAR,         # car
        "3": CLASS_MOTORCYCLE,  # motorcycle
        "5": CLASS_BUS,         # bus
        "7": CLASS_TRUCK,       # truck
    }
    return coco_map.get(raw, CLASS_UNKNOWN)


class Detection2DToTier4Rois(Node):
    def __init__(self) -> None:
        super().__init__("detection2d_to_tier4_rois")

        if (
            DetectedObjectWithFeature is None
            or DetectedObjectsWithFeature is None
            or Feature is None
        ):
            raise RuntimeError(f"tier4_perception_msgs import failed: {IMPORT_ERROR}")

        self.declare_parameter("copy_track_id_to_uuid", False)
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")

        input_topic = str(self.get_parameter("input_topic").value).strip() or "~/input/detections"
        output_topic = str(self.get_parameter("output_topic").value).strip() or "~/output/rois"

        self.sub = self.create_subscription(
            Detection2DArray, input_topic, self._on_detections, 10
        )
        self.pub = self.create_publisher(
            DetectedObjectsWithFeature, output_topic, 10
        )
        self.msg_count = 0
        self.get_logger().info(
            "Detection2DArray -> tier4_perception_msgs/DetectedObjectsWithFeature adapter started "
            f"(in={getattr(self.sub, 'topic_name', input_topic)}, out={getattr(self.pub, 'topic_name', output_topic)})"
        )

    def _on_detections(self, msg: Detection2DArray) -> None:
        if not rclpy.ok():
            return
        out = DetectedObjectsWithFeature()
        out.header = msg.header

        arr = self._get_feature_array(out)
        if arr is None:
            self.get_logger().error("Unsupported DetectedObjectsWithFeature layout; no feature array field")
            return

        for det in msg.detections:
            arr.append(self._convert_detection(det))

        try:
            self.pub.publish(out)
        except Exception:
            return
        self.msg_count += 1
        if self.msg_count == 1 or self.msg_count % 200 == 0:
            self.get_logger().info(
                f"Published {self.msg_count} ROI arrays, last count={len(msg.detections)}"
            )

    def _convert_detection(self, det: Detection2D):
        obj_with_feature = DetectedObjectWithFeature()

        class_id, score = _extract_hypothesis(det)
        cx, cy, w, h = _bbox_xywh(det)
        x1 = cx - w * 0.5
        y1 = cy - h * 0.5

        self._fill_object(obj_with_feature, class_id, score, w, h)
        self._fill_feature_roi(obj_with_feature, x1, y1, w, h)
        return obj_with_feature

    def _fill_object(self, obj_with_feature, class_id: str, score: float, w: float, h: float) -> None:
        if not hasattr(obj_with_feature, "object"):
            return

        obj = obj_with_feature.object

        # existence_probability is commonly required downstream.
        if hasattr(obj, "existence_probability"):
            obj.existence_probability = float(max(0.0, min(1.0, score)))

        if hasattr(obj, "classification"):
            cls = ObjectClassification()
            cls.label = _map_object_label(class_id)
            cls.probability = float(max(0.0, min(1.0, score)))
            obj.classification.append(cls)

        # Fill a minimal shape if available; ROI fusion mostly uses ROI + classification.
        if hasattr(obj, "shape"):
            shape = obj.shape
            if hasattr(shape, "type") and hasattr(shape.__class__, "BOUNDING_BOX"):
                try:
                    shape.type = shape.__class__.BOUNDING_BOX
                except Exception:
                    pass
            if hasattr(shape, "dimensions"):
                if hasattr(shape.dimensions, "x"):
                    shape.dimensions.x = max(float(w), 0.01)
                if hasattr(shape.dimensions, "y"):
                    shape.dimensions.y = max(float(h), 0.01)
                if hasattr(shape.dimensions, "z"):
                    shape.dimensions.z = 1.0

    def _fill_feature_roi(self, obj_with_feature, x: float, y: float, w: float, h: float) -> None:
        feature = getattr(obj_with_feature, "feature", None)
        if feature is None:
            return
        roi = getattr(feature, "roi", None)
        if roi is None:
            return
        if hasattr(roi, "x_offset"):
            roi.x_offset = _clamp_int(x)
        if hasattr(roi, "y_offset"):
            roi.y_offset = _clamp_int(y)
        if hasattr(roi, "width"):
            roi.width = _clamp_int(w)
        if hasattr(roi, "height"):
            roi.height = _clamp_int(h)
        if hasattr(roi, "do_rectify"):
            roi.do_rectify = False

    def _get_feature_array(self, msg: DetectedObjectsWithFeature) -> Optional[list]:
        for field in ("feature_objects", "objects"):
            if hasattr(msg, field):
                arr = getattr(msg, field)
                try:
                    len(arr)
                    return arr
                except Exception:
                    continue
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Detection2DToTier4Rois()
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


if __name__ == "__main__":
    main()
