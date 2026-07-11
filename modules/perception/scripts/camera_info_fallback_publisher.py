#!/usr/bin/env python3
"""Publish fallback CameraInfo for an image topic and optional static TF.

Use this when a camera driver publishes `image_raw` but not `camera_info`.
The node republishes a calibrated `sensor_msgs/CameraInfo` on every incoming
image with matching timestamps so ROI projection/fusion nodes can synchronize.
"""

import argparse
import math
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover - optional dependency in runtime image
    yaml = None

try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
except Exception:  # pragma: no cover - optional if tf2_ros is unavailable
    TransformStamped = None
    StaticTransformBroadcaster = None


def _parse_csv_floats(raw: str) -> List[float]:
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    return [float(p) for p in parts]


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def _load_camera_info_yaml(path: str) -> Optional[dict]:
    if yaml is None:
        return None
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class CameraInfoFallbackPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("camera_info_fallback_publisher")
        self._args = args
        self._pub = self.create_publisher(CameraInfo, args.camera_info_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            Image, args.image_topic, self._on_image, qos_profile_sensor_data
        )
        self._frames = 0
        self._warned_no_yaml = False
        self._tf_published = False
        self._static_tf = None

        self._cfg = self._build_config(args)
        if args.publish_static_tf:
            if StaticTransformBroadcaster is None or TransformStamped is None:
                self.get_logger().warn("tf2_ros not available; static TF fallback disabled")
            else:
                self._static_tf = StaticTransformBroadcaster(self)

        self.get_logger().info(
            f"Fallback CameraInfo: image={args.image_topic} -> camera_info={args.camera_info_topic}"
        )

    def _build_config(self, args: argparse.Namespace) -> dict:
        cfg = {
            "image_width": int(args.width),
            "image_height": int(args.height),
            "camera_name": args.camera_name,
            "distortion_model": args.distortion_model,
            "distortion_coefficients": {"data": _parse_csv_floats(args.distortion)},
            "camera_matrix": {
                "data": [
                    float(args.fx),
                    0.0,
                    float(args.cx),
                    0.0,
                    float(args.fy),
                    float(args.cy),
                    0.0,
                    0.0,
                    1.0,
                ]
            },
            "rectification_matrix": {"data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
            "projection_matrix": {
                "data": [
                    float(args.fx),
                    0.0,
                    float(args.cx),
                    0.0,
                    0.0,
                    float(args.fy),
                    float(args.cy),
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                ]
            },
        }

        if args.camera_info_yaml:
            if yaml is None:
                if not self._warned_no_yaml:
                    self.get_logger().warn(
                        f"PyYAML not available; cannot load {args.camera_info_yaml}. Using built-in defaults"
                    )
                    self._warned_no_yaml = True
            else:
                try:
                    loaded = _load_camera_info_yaml(args.camera_info_yaml)
                    if loaded:
                        cfg.update(loaded)
                        self.get_logger().info(f"Loaded camera calibration YAML: {args.camera_info_yaml}")
                except Exception as e:  # pragma: no cover - runtime environment dependent
                    self.get_logger().warn(f"Failed to load camera info YAML ({args.camera_info_yaml}): {e}")

        return cfg

    def _on_image(self, image: Image) -> None:
        if not rclpy.ok():
            return
        msg = CameraInfo()
        msg.header = image.header

        frame_id_fallback = self._args.frame_id.strip()
        if not msg.header.frame_id:
            msg.header.frame_id = frame_id_fallback or "camera"

        if self._args.width > 0:
            msg.width = int(self._cfg.get("image_width", self._args.width))
        else:
            msg.width = int(image.width)

        if self._args.height > 0:
            msg.height = int(self._cfg.get("image_height", self._args.height))
        else:
            msg.height = int(image.height)

        msg.distortion_model = str(self._cfg.get("distortion_model", "plumb_bob"))
        msg.d = [float(v) for v in self._cfg.get("distortion_coefficients", {}).get("data", [0.0] * 5)]
        msg.k = [float(v) for v in self._cfg.get("camera_matrix", {}).get("data", [])]
        msg.r = [float(v) for v in self._cfg.get("rectification_matrix", {}).get("data", [])]
        msg.p = [float(v) for v in self._cfg.get("projection_matrix", {}).get("data", [])]

        if len(msg.k) != 9:
            msg.k = [1.0, 0.0, msg.width / 2.0, 0.0, 1.0, msg.height / 2.0, 0.0, 0.0, 1.0]
        if len(msg.r) != 9:
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        if len(msg.p) != 12:
            msg.p = [1.0, 0.0, msg.width / 2.0, 0.0, 0.0, 1.0, msg.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        try:
            self._pub.publish(msg)
        except Exception:
            return
        self._publish_static_tf_once(msg.header.frame_id)

        self._frames += 1
        if self._frames == 1 or self._frames % 300 == 0:
            self.get_logger().info(
                f"Published CameraInfo {self._frames}x on {self._args.camera_info_topic} "
                f"(frame={msg.header.frame_id}, size={msg.width}x{msg.height})"
            )

    def _publish_static_tf_once(self, child_frame: str) -> None:
        if self._tf_published or self._static_tf is None:
            return
        if not child_frame:
            return

        base_frame = self._args.base_frame.strip() or "base_link"
        if base_frame == child_frame:
            self._tf_published = True
            return
        if not rclpy.ok():
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = base_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = float(self._args.x)
        tf_msg.transform.translation.y = float(self._args.y)
        tf_msg.transform.translation.z = float(self._args.z)
        qx, qy, qz, qw = _quat_from_rpy(
            float(self._args.roll),
            float(self._args.pitch),
            float(self._args.yaw),
        )
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        try:
            self._static_tf.sendTransform(tf_msg)
        except Exception:
            return
        self._tf_published = True
        self.get_logger().info(
            f"Published fallback static TF {base_frame} -> {child_frame} "
            f"(xyz={self._args.x},{self._args.y},{self._args.z} rpy={self._args.roll},{self._args.pitch},{self._args.yaw})"
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image-topic", required=True, help="Input image topic")
    ap.add_argument("--camera-info-topic", required=True, help="Output CameraInfo topic")
    ap.add_argument("--camera-name", default="logitech_720p")
    ap.add_argument("--camera-info-yaml", default="", help="ROS camera_info YAML path (optional)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fx", type=float, default=1.0)
    ap.add_argument("--fy", type=float, default=1.0)
    ap.add_argument("--cx", type=float, default=640.0)
    ap.add_argument("--cy", type=float, default=360.0)
    ap.add_argument("--distortion-model", default="plumb_bob")
    ap.add_argument("--distortion", default="0,0,0,0,0", help="Comma-separated distortion coeffs")
    ap.add_argument("--frame-id", default="", help="Used only if image header frame_id is empty")

    ap.add_argument("--publish-static-tf", action="store_true")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--x", type=float, default=0.98)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.39)
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = CameraInfoFallbackPublisher(args)
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


if __name__ == "__main__":
    main()
