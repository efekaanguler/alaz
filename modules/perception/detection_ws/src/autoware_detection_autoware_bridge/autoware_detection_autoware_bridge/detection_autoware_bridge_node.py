#!/usr/bin/env python3

import math
import uuid as uuidlib
from typing import Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

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

# Typical object heights (metres) used for ground-plane depth estimation
# when no depth sensor is available.
_DEFAULT_OBJECT_HEIGHTS = {
    CLASS_PEDESTRIAN: 1.7,
    CLASS_BICYCLE: 1.5,
    CLASS_MOTORCYCLE: 1.5,
    CLASS_CAR: 1.5,
    CLASS_TRUCK: 3.0,
    CLASS_BUS: 3.2,
    CLASS_TRAILER: 3.5,
    CLASS_ANIMAL: 0.6,
    CLASS_UNKNOWN: 1.0,
}

# Typical object widths (metres) for metric shape estimation
_DEFAULT_OBJECT_WIDTHS = {
    CLASS_PEDESTRIAN: 0.6,
    CLASS_BICYCLE: 0.6,
    CLASS_MOTORCYCLE: 0.8,
    CLASS_CAR: 1.8,
    CLASS_TRUCK: 2.5,
    CLASS_BUS: 2.5,
    CLASS_TRAILER: 2.5,
    CLASS_ANIMAL: 0.5,
    CLASS_UNKNOWN: 1.0,
}

# Typical object lengths (metres) for metric shape estimation
_DEFAULT_OBJECT_LENGTHS = {
    CLASS_PEDESTRIAN: 0.6,
    CLASS_BICYCLE: 1.8,
    CLASS_MOTORCYCLE: 2.0,
    CLASS_CAR: 4.5,
    CLASS_TRUCK: 8.0,
    CLASS_BUS: 10.0,
    CLASS_TRAILER: 12.0,
    CLASS_ANIMAL: 0.8,
    CLASS_UNKNOWN: 1.0,
}

# Minimum depth (metres) to avoid publishing objects "at camera"
_MIN_DEPTH = 0.5
# Maximum depth (metres) beyond which ground-plane estimates are unreliable
_MAX_DEPTH = 50.0


class DetectionAutowareBridge(Node):
    def __init__(self) -> None:
        super().__init__('detection_autoware_bridge')

        # --- publication toggles ---
        self.publish_detected_objects = bool(self.declare_parameter('publish_detected_objects', False).value)
        self.publish_tracked_objects = bool(self.declare_parameter('publish_tracked_objects', False).value)
        self.publish_predicted_objects = bool(self.declare_parameter('publish_predicted_objects', False).value)
        self.publish_traffic_signals = bool(self.declare_parameter('publish_traffic_signals', False).value)

        # --- camera mounting parameters ---
        # Height of camera above the ground plane (metres).
        self._camera_height = float(self.declare_parameter('camera_height', 0.39).value)
        # Pitch angle of camera relative to horizontal (radians, positive = tilted down).
        self._camera_pitch = float(self.declare_parameter('camera_pitch', 0.0).value)
        # Output frame_id for 3D objects (should be the vehicle base frame).
        self._output_frame = str(self.declare_parameter('output_frame', 'base_link').value)
        # Minimum confidence to publish an object
        self._min_confidence = float(self.declare_parameter('min_confidence', 0.3).value)

        # --- camera intrinsics (updated via CameraInfo subscription) ---
        self._fx: Optional[float] = None
        self._fy: Optional[float] = None
        self._cx: Optional[float] = None
        self._cy: Optional[float] = None
        self._intrinsics_warned = False
        self._intrinsics_received = False

        # --- subscriptions ---
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '~/input/camera_info',
            self._on_camera_info,
            10,
        )
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

        # --- publishers ---
        self.pub_detected = self.create_publisher(DetectedObjects, '~/output/detected_objects', 10)
        self.pub_tracked = self.create_publisher(TrackedObjects, '~/output/tracked_objects', 10)
        self.pub_predicted = self.create_publisher(PredictedObjects, '~/output/predicted_objects', 10)
        self.pub_traffic = self.create_publisher(TrafficLightGroupArray, '~/output/traffic_signals', 10)

        self.get_logger().info(
            'bridge started: detected=%s tracked=%s predicted=%s traffic=%s '
            'camera_height=%.2f camera_pitch=%.3f output_frame=%s'
            % (
                self.publish_detected_objects,
                self.publish_tracked_objects,
                self.publish_predicted_objects,
                self.publish_traffic_signals,
                self._camera_height,
                self._camera_pitch,
                self._output_frame,
            )
        )
        self.get_logger().warn(
            'Waiting for CameraInfo on ~/input/camera_info before publishing 3D objects. '
            'Ensure your camera is calibrated (fx, fy != 1.0) for accurate depth estimation.'
        )

    # ------------------------------------------------------------------
    # CameraInfo callback
    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Extract intrinsics from CameraInfo (3x3 K matrix)."""
        if len(msg.k) < 9:
            return
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

        if not self._intrinsics_received:
            self._intrinsics_received = True
            self.get_logger().info(
                'CameraInfo received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f'
                % (self._fx, self._fy, self._cx, self._cy)
            )

        # Warn once if intrinsics look like placeholders
        if not self._intrinsics_warned and (abs(self._fx - 1.0) < 0.01 or abs(self._fy - 1.0) < 0.01):
            self._intrinsics_warned = True
            self.get_logger().error(
                'Camera intrinsics appear to be UNCALIBRATED (fx=%.2f, fy=%.2f). '
                '3D depth estimates will be incorrect. Calibrate your camera and update '
                'the camera_info YAML file.' % (self._fx, self._fy)
            )

    # ------------------------------------------------------------------
    # Object detection callback
    # ------------------------------------------------------------------
    def _on_object_detections(self, msg: Detection2DArray) -> None:
        if not self._intrinsics_received:
            return  # Don't publish invalid objects until we have camera intrinsics

        detected_msg = DetectedObjects()
        detected_msg.header = msg.header
        detected_msg.header.frame_id = self._output_frame

        tracked_msg = TrackedObjects()
        tracked_msg.header = msg.header
        tracked_msg.header.frame_id = self._output_frame

        predicted_msg = PredictedObjects()
        predicted_msg.header = msg.header
        predicted_msg.header.frame_id = self._output_frame

        for idx, det in enumerate(msg.detections):
            class_id, score = _extract_hypothesis(det)
            score = _clamp01(score)

            if score < self._min_confidence:
                continue

            classification = _to_object_classification(class_id, score)
            label = classification.label

            # Ground-plane 3D projection
            result = self._project_to_ground(det, label)
            if result is None:
                continue  # Could not project (object above horizon, etc.)

            depth, x_vehicle, y_vehicle = result

            pose = _make_pose(x_vehicle, y_vehicle, 0.0)
            shape = _estimate_shape(label, depth, det, self._fx, self._fy)
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

    def _project_to_ground(
        self, det: Detection2D, label: int
    ) -> Optional[Tuple[float, float, float]]:
        """
        Project a 2D bounding box onto the ground plane using the pinhole camera
        model and known camera height.

        Uses the bottom-centre of the bounding box as the ground-contact point.
        Returns (depth, x_vehicle, y_vehicle) in the vehicle base_link frame,
        or None if projection is invalid.

        Vehicle frame convention (ROS/Autoware):
            x = forward, y = left, z = up
        Camera frame convention (OpenCV):
            x = right, y = down, z = forward
        """
        fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        # Bottom-centre of bbox = assumed ground-contact point
        u = float(det.bbox.center.position.x)
        v_bottom = float(det.bbox.center.position.y) + float(det.bbox.size_y) / 2.0

        # Ray direction in camera frame (before pitch correction)
        ray_y = (v_bottom - cy) / fy
        ray_x = (u - cx) / fx

        # Apply camera pitch: rotate ray around camera X axis
        # After pitch rotation, the "down" component in world frame is:
        #   ray_y_world = ray_y * cos(pitch) + 1.0 * sin(pitch)
        cos_p = math.cos(self._camera_pitch)
        sin_p = math.sin(self._camera_pitch)
        ray_y_world = ray_y * cos_p + sin_p  # component pointing "down" in world
        ray_z_world = -ray_y * sin_p + cos_p  # component pointing "forward" in world

        # Object must be below the camera (ground contact below horizon)
        if ray_y_world <= 0.01:
            return None  # Object is at or above horizon line

        # Scale so that the ray reaches the ground plane (camera_height below camera)
        scale = self._camera_height / ray_y_world

        # 3D position in camera world-aligned frame
        depth = scale * ray_z_world  # forward distance
        lateral = scale * ray_x      # rightward in camera frame

        if depth < _MIN_DEPTH or depth > _MAX_DEPTH:
            return None

        # Convert to vehicle frame: x=forward, y=left
        x_vehicle = depth
        y_vehicle = -lateral  # camera right → vehicle left is negative

        return (depth, x_vehicle, y_vehicle)

    # ------------------------------------------------------------------
    # Traffic signals callback (unchanged)
    # ------------------------------------------------------------------
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


# ======================================================================
# Helper functions
# ======================================================================

def _extract_hypothesis(det: Detection2D) -> Tuple[str, float]:
    if len(det.results) == 0:
        return 'unknown', 0.0

    hyp = det.results[0].hypothesis
    return str(hyp.class_id), float(hyp.score)


def _make_pose(x: float, y: float, z: float) -> Pose:
    """Create a Pose at (x, y, z) with identity orientation."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose


def _estimate_shape(
    label: int, depth: float, det: Detection2D,
    fx: float, fy: float
) -> Shape:
    """
    Estimate metric object dimensions from depth and class priors.

    Width is estimated from the pixel bounding box width and depth via the
    pinhole model: width_m = (pixel_width * depth) / fx.
    Height and length use class-typical priors since monocular estimation
    from a single frame is unreliable for those dimensions.
    """
    shape = Shape()
    shape.type = Shape.BOUNDING_BOX

    # Estimate width from pixel width + depth (this is geometrically valid)
    pixel_width = max(float(det.bbox.size_x), 1.0)
    estimated_width = (pixel_width * depth) / fx if fx > 1.0 else _DEFAULT_OBJECT_WIDTHS.get(label, 1.0)

    # Clamp width to reasonable range based on class
    max_reasonable_width = _DEFAULT_OBJECT_WIDTHS.get(label, 3.0) * 2.5
    estimated_width = min(estimated_width, max_reasonable_width)
    estimated_width = max(estimated_width, 0.1)

    # Use class priors for length and height (monocular can't estimate these reliably)
    shape.dimensions.x = _DEFAULT_OBJECT_LENGTHS.get(label, 1.0)  # length (along vehicle x)
    shape.dimensions.y = estimated_width                            # width (along vehicle y)
    shape.dimensions.z = _DEFAULT_OBJECT_HEIGHTS.get(label, 1.0)  # height

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
    # Do not assume stationary — we don't have velocity info from monocular camera.
    # Let downstream tracking/fusion determine motion state.
    kinematics.is_stationary = False
    return kinematics


def _to_predicted_kinematics(pose: Pose, score: float) -> PredictedObjectKinematics:
    kinematics = PredictedObjectKinematics()
    kinematics.initial_pose_with_covariance.pose = pose
    try:
        kinematics.orientation_availability = KINEMATICS_UNAVAILABLE
    except AttributeError:
        pass  # Field not available in this msg version

    # Without velocity estimation, predict the object stays in place.
    # This is more honest than the old single-pose "path" — we explicitly
    # set low confidence to signal that this is not a real prediction.
    path = PredictedPath()
    path.path.append(pose)
    path.time_step = Duration(sec=0, nanosec=500000000)
    path.confidence = min(score * 0.5, 0.5)  # Low confidence: camera-only, no velocity

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
