#!/usr/bin/env python3

import uuid as uuidlib
from typing import Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rclpy.node import Node

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjectKinematics,
    DetectedObjects,
    ObjectClassification,
    PredictedObject,
    PredictedObjectKinematics,
    PredictedObjects,
    PredictedPath,
    Shape,
    TrackedObject,
    TrackedObjectKinematics,
    TrackedObjects,
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from unique_identifier_msgs.msg import UUID
from vision_msgs.msg import Detection2D, Detection2DArray

# autoware_perception_msgs/ObjectClassification labels
CLASS_UNKNOWN = 0
CLASS_CAR = 1
CLASS_TRUCK = 2
CLASS_BUS = 3
CLASS_TRAILER = 4
CLASS_MOTORCYCLE = 5
CLASS_BICYCLE = 6
CLASS_PEDESTRIAN = 7
CLASS_ANIMAL = 8

# autoware_perception_msgs/TrafficLightElement constants
TL_COLOR_UNKNOWN = 0
TL_COLOR_RED = 1
TL_COLOR_AMBER = 2
TL_COLOR_GREEN = 3
TL_COLOR_WHITE = 4
TL_SHAPE_CIRCLE = 1
TL_STATUS_UNKNOWN = 0
TL_STATUS_SOLID_ON = 2

KINEMATICS_UNAVAILABLE = 0


class DetectionAutowareBridge(Node):
    def __init__(self) -> None:
        super().__init__('detection_autoware_bridge')

        self.publish_detected_objects = bool(self.declare_parameter('publish_detected_objects', True).value)
        self.publish_tracked_objects = bool(self.declare_parameter('publish_tracked_objects', True).value)
        self.publish_predicted_objects = bool(self.declare_parameter('publish_predicted_objects', True).value)
        self.publish_traffic_signals = bool(self.declare_parameter('publish_traffic_signals', True).value)

        self.sub_objects = self.create_subscription(
            Detection2DArray,
            '~/input/tracked_detections',
            self._on_object_detections,
            10,
        )
        self.sub_traffic = self.create_subscription(
            Detection2DArray,
            '~/input/traffic_signals_2d',
            self._on_traffic_signals,
            10,
        )

        self.pub_detected = self.create_publisher(DetectedObjects, '~/output/detected_objects', 10)
        self.pub_tracked = self.create_publisher(TrackedObjects, '~/output/tracked_objects', 10)
        self.pub_predicted = self.create_publisher(PredictedObjects, '~/output/predicted_objects', 10)
        self.pub_traffic = self.create_publisher(TrafficLightGroupArray, '~/output/traffic_signals', 10)

        self.get_logger().info(
            'bridge started: detected=%s tracked=%s predicted=%s traffic=%s'
            % (
                self.publish_detected_objects,
                self.publish_tracked_objects,
                self.publish_predicted_objects,
                self.publish_traffic_signals,
            )
        )

    def _on_object_detections(self, msg: Detection2DArray) -> None:
        detected_msg = DetectedObjects()
        detected_msg.header = msg.header

        tracked_msg = TrackedObjects()
        tracked_msg.header = msg.header

        predicted_msg = PredictedObjects()
        predicted_msg.header = msg.header

        for idx, det in enumerate(msg.detections):
            class_id, score = _extract_hypothesis(det)
            score = _clamp01(score)
            classification = _to_object_classification(class_id, score)
            pose = _to_pose(det)
            shape = _to_shape(det)
            object_id = _to_uuid(det, idx, class_id)

            if self.publish_detected_objects:
                d_obj = DetectedObject()
                d_obj.existence_probability = score
                d_obj.classification.append(classification)
                d_obj.shape = shape
                d_obj.kinematics = _to_detected_kinematics(pose)
                detected_msg.objects.append(d_obj)

            if self.publish_tracked_objects:
                t_obj = TrackedObject()
                t_obj.object_id = object_id
                t_obj.existence_probability = score
                t_obj.classification.append(classification)
                t_obj.shape = shape
                t_obj.kinematics = _to_tracked_kinematics(pose)
                tracked_msg.objects.append(t_obj)

            if self.publish_predicted_objects:
                p_obj = PredictedObject()
                p_obj.object_id = object_id
                p_obj.existence_probability = score
                p_obj.classification.append(classification)
                p_obj.shape = shape
                p_obj.kinematics = _to_predicted_kinematics(pose, score)
                predicted_msg.objects.append(p_obj)

        if self.publish_detected_objects:
            self.pub_detected.publish(detected_msg)
        if self.publish_tracked_objects:
            self.pub_tracked.publish(tracked_msg)
        if self.publish_predicted_objects:
            self.pub_predicted.publish(predicted_msg)

    def _on_traffic_signals(self, msg: Detection2DArray) -> None:
        if not self.publish_traffic_signals:
            return

        out = TrafficLightGroupArray()
        if hasattr(out, 'stamp'):
            out.stamp = msg.header.stamp
        elif hasattr(out, 'header'):
            out.header = msg.header

        for idx, det in enumerate(msg.detections):
            class_id, score = _extract_hypothesis(det)
            group = TrafficLightGroup()
            _set_traffic_light_group_id(group, _lanelet_id(det, idx))
            group.elements.append(_to_traffic_light_element(class_id, _clamp01(score)))
            out.traffic_light_groups.append(group)

        self.pub_traffic.publish(out)


def _extract_hypothesis(det: Detection2D) -> Tuple[str, float]:
    if len(det.results) == 0:
        return 'unknown', 0.0

    hyp = det.results[0].hypothesis
    return str(hyp.class_id), float(hyp.score)


def _to_pose(det: Detection2D) -> Pose:
    pose = Pose()
    pose.position.x = float(det.bbox.center.position.x)
    pose.position.y = float(det.bbox.center.position.y)
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose


def _to_shape(det: Detection2D) -> Shape:
    shape = Shape()
    shape.type = Shape.BOUNDING_BOX
    shape.dimensions.x = max(float(det.bbox.size_x), 0.01)
    shape.dimensions.y = max(float(det.bbox.size_y), 0.01)
    shape.dimensions.z = 1.0
    return shape


def _to_detected_kinematics(pose: Pose) -> DetectedObjectKinematics:
    kinematics = DetectedObjectKinematics()
    kinematics.pose_with_covariance.pose = pose
    kinematics.has_position_covariance = False
    try:
        kinematics.orientation_availability = KINEMATICS_UNAVAILABLE
    except AttributeError:
        pass  # Field not available in this msg version
    kinematics.has_twist = False
    kinematics.has_twist_covariance = False
    return kinematics


def _to_tracked_kinematics(pose: Pose) -> TrackedObjectKinematics:
    kinematics = TrackedObjectKinematics()
    kinematics.pose_with_covariance.pose = pose
    try:
        kinematics.orientation_availability = KINEMATICS_UNAVAILABLE
    except AttributeError:
        pass  # Field not available in this msg version
    kinematics.is_stationary = True
    return kinematics


def _to_predicted_kinematics(pose: Pose, score: float) -> PredictedObjectKinematics:
    kinematics = PredictedObjectKinematics()
    kinematics.initial_pose_with_covariance.pose = pose
    try:
        kinematics.orientation_availability = KINEMATICS_UNAVAILABLE
    except AttributeError:
        pass  # Field not available in this msg version

    path = PredictedPath()
    path.path.append(pose)
    path.time_step = Duration(sec=0, nanosec=500000000)
    path.confidence = score

    kinematics.predicted_paths.append(path)
    return kinematics


def _to_object_classification(class_id: str, score: float) -> ObjectClassification:
    cls = ObjectClassification()
    cls.label = _map_object_label(class_id)
    cls.probability = score
    return cls


def _map_object_label(class_id: str) -> int:
    raw = class_id.strip().lower()

    word_map = {
        'person': CLASS_PEDESTRIAN,
        'pedestrian': CLASS_PEDESTRIAN,
        'human': CLASS_PEDESTRIAN,
        'car': CLASS_CAR,
        'truck': CLASS_TRUCK,
        'bus': CLASS_BUS,
        'trailer': CLASS_TRAILER,
        'motorcycle': CLASS_MOTORCYCLE,
        'bicycle': CLASS_BICYCLE,
        'bike': CLASS_BICYCLE,
        'animal': CLASS_ANIMAL,
    }
    if raw in word_map:
        return word_map[raw]

    coco_map = {
        0: CLASS_PEDESTRIAN,
        1: CLASS_BICYCLE,
        2: CLASS_CAR,
        3: CLASS_MOTORCYCLE,
        5: CLASS_BUS,
        7: CLASS_TRUCK,
    }
    try:
        return coco_map.get(int(raw), CLASS_UNKNOWN)
    except ValueError:
        return CLASS_UNKNOWN


def _to_uuid(det: Detection2D, index: int, class_id: str) -> UUID:
    token = ''
    if hasattr(det, 'id'):
        token = str(det.id).strip()

    if token == '':
        token = (
            f'{index}:{class_id}:'
            f'{float(det.bbox.center.position.x):.3f}:'
            f'{float(det.bbox.center.position.y):.3f}'
        )

    uid = uuidlib.uuid5(uuidlib.NAMESPACE_URL, token)
    out = UUID()
    out.uuid = list(uid.bytes)
    return out


def _lanelet_id(det: Detection2D, index: int) -> int:
    if hasattr(det, 'id'):
        raw = str(det.id).strip()
        try:
            return int(raw)
        except ValueError:
            pass

    return -(index + 1)


def _set_traffic_light_group_id(group: TrafficLightGroup, value: int) -> None:
    """Handle message version differences in TrafficLightGroup ID field name."""
    for attr in ('lanelet_id', 'traffic_light_group_id', 'map_primitive_id', 'id'):
        if hasattr(group, attr):
            setattr(group, attr, int(value))
            return


def _to_traffic_light_element(class_id: str, score: float) -> TrafficLightElement:
    label = class_id.strip().lower()

    if label in {'red', 'stop'}:
        color = TL_COLOR_RED
    elif label in {'yellow', 'amber', 'orange'}:
        color = TL_COLOR_AMBER
    elif label in {'green', 'go'}:
        color = TL_COLOR_GREEN
    elif label in {'white'}:
        color = TL_COLOR_WHITE
    else:
        color = TL_COLOR_UNKNOWN

    element = TrafficLightElement()
    element.color = color
    element.shape = TL_SHAPE_CIRCLE
    element.status = TL_STATUS_SOLID_ON if color != TL_COLOR_UNKNOWN else TL_STATUS_UNKNOWN
    element.confidence = score
    return element


def _clamp01(value: float) -> float:
    if value < 0.0:
        return 0.0
    if value > 1.0:
        return 1.0
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DetectionAutowareBridge()
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
