#!/usr/bin/env python3
"""Publish a fallback static TF for the LaserScan frame.

This helps `laserscan_to_pointcloud_node` transform `/sensing/scan` into `base_link`
when the sensor kit TF tree is missing.
"""

import argparse
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
except Exception as e:  # pragma: no cover - runtime environment dependent
    TransformStamped = None
    StaticTransformBroadcaster = None
    IMPORT_ERROR = e
else:
    IMPORT_ERROR = None


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


class ScanStaticTfFallback(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("scan_static_tf_fallback")
        if StaticTransformBroadcaster is None or TransformStamped is None:
            raise RuntimeError(f"tf2_ros unavailable: {IMPORT_ERROR}")

        self._args = args
        self._broadcaster = StaticTransformBroadcaster(self)
        self._published = False

        self._sub = self.create_subscription(
            LaserScan, args.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Waiting for scan on {args.scan_topic} to publish fallback TF {args.base_frame} -> <scan_frame>"
        )

        if args.child_frame:
            self._publish_once(args.child_frame)

    def _on_scan(self, msg: LaserScan) -> None:
        if self._published:
            return
        child = self._args.child_frame.strip() or msg.header.frame_id.strip() or "lidar_link"
        self._publish_once(child)

    def _publish_once(self, child_frame: str) -> None:
        if self._published:
            return
        base_frame = self._args.base_frame.strip() or "base_link"
        if base_frame == child_frame:
            self._published = True
            self.get_logger().info("Scan frame already equals base frame; no TF published")
            return
        if not rclpy.ok():
            return

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(self._args.x)
        msg.transform.translation.y = float(self._args.y)
        msg.transform.translation.z = float(self._args.z)
        qx, qy, qz, qw = _quat_from_rpy(
            float(self._args.roll), float(self._args.pitch), float(self._args.yaw)
        )
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        try:
            self._broadcaster.sendTransform(msg)
        except Exception:
            return
        self._published = True
        self.get_logger().info(
            f"Published fallback static TF {base_frame} -> {child_frame} "
            f"(xyz={self._args.x},{self._args.y},{self._args.z} rpy={self._args.roll},{self._args.pitch},{self._args.yaw})"
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan-topic", default="/sensing/scan")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--child-frame", default="", help="Optional; if empty uses first scan header.frame_id")
    ap.add_argument("--x", type=float, default=1.36)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.17)
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    args = ap.parse_args()

    rclpy.init()
    node = ScanStaticTfFallback(args)
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
