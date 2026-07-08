#!/usr/bin/env python3
"""Convert PointCloud2 to a simple 2D OccupancyGrid for planner fallback.

This is a lightweight fallback when the full Autoware occupancy map stack is
not available in the Docker image. It also republishes the pointcloud to a
planner-friendly topic.
"""

import argparse
import math
import struct
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudToOccupancyGrid(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pointcloud_to_occupancy_grid_fallback")
        self._args = args
        self._sub = self.create_subscription(
            PointCloud2, args.input_topic, self._on_cloud, qos_profile_sensor_data
        )
        self._pub_occ = self.create_publisher(OccupancyGrid, args.occupancy_topic, 10)
        self._pub_cloud = None
        if args.pointcloud_output_topic:
            self._pub_cloud = self.create_publisher(PointCloud2, args.pointcloud_output_topic, qos_profile_sensor_data)
        self._frames = 0
        self.get_logger().info(
            f"Occupancy fallback: {args.input_topic} -> {args.occupancy_topic}"
            + (f" and {args.pointcloud_output_topic}" if args.pointcloud_output_topic else "")
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not rclpy.ok():
            return
        if self._pub_cloud is not None:
            try:
                self._pub_cloud.publish(msg)
            except Exception:
                return

        occ = OccupancyGrid()
        occ.header = msg.header
        if self._args.frame_id:
            occ.header.frame_id = self._args.frame_id

        occ.info.resolution = float(self._args.resolution)
        occ.info.width = int(self._args.width)
        occ.info.height = int(self._args.height)
        occ.info.origin.position.x = float(self._args.origin_x)
        occ.info.origin.position.y = float(self._args.origin_y)
        occ.info.origin.position.z = 0.0
        occ.info.origin.orientation.x = 0.0
        occ.info.origin.orientation.y = 0.0
        occ.info.origin.orientation.z = 0.0
        occ.info.origin.orientation.w = 1.0

        default_val = -1 if self._args.unknown_default else 0
        grid = [default_val] * (occ.info.width * occ.info.height)
        points_seen = 0
        points_kept = 0

        for x, y, z in self._iter_xyz(msg):
            points_seen += 1
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            if z < self._args.min_z or z > self._args.max_z:
                continue

            gx = math.floor((x - self._args.origin_x) / self._args.resolution)
            gy = math.floor((y - self._args.origin_y) / self._args.resolution)
            if 0 <= gx < self._args.width and 0 <= gy < self._args.height:
                idx = gy * self._args.width + gx
                grid[idx] = 100
                points_kept += 1

        occ.data = grid
        try:
            self._pub_occ.publish(occ)
        except Exception:
            return

        self._frames += 1
        if self._frames == 1 or self._frames % 100 == 0:
            self.get_logger().info(
                f"Published occupancy {self._frames}x ({points_kept}/{points_seen} pts kept, "
                f"grid={self._args.width}x{self._args.height}@{self._args.resolution}m)"
            )

    def _iter_xyz(self, msg: PointCloud2):
        x_off = y_off = z_off = None
        for f in msg.fields:
            if f.name == "x":
                x_off = f.offset
            elif f.name == "y":
                y_off = f.offset
            elif f.name == "z":
                z_off = f.offset

        if x_off is None or y_off is None:
            return
        if z_off is None:
            z_off = -1

        endian = ">" if msg.is_bigendian else "<"
        unpack_f32 = struct.Struct(endian + "f").unpack_from

        data = msg.data
        point_step = msg.point_step
        row_step = msg.row_step if msg.row_step > 0 else msg.point_step * msg.width
        width = msg.width
        height = msg.height

        for r in range(height):
            row_base = r * row_step
            for c in range(width):
                base = row_base + c * point_step
                try:
                    x = unpack_f32(data, base + x_off)[0]
                    y = unpack_f32(data, base + y_off)[0]
                    z = unpack_f32(data, base + z_off)[0] if z_off >= 0 else 0.0
                except struct.error:
                    continue
                yield x, y, z


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-topic", default="/perception/lidar/pointcloud")
    ap.add_argument("--occupancy-topic", default="/perception/occupancy_grid")
    ap.add_argument(
        "--pointcloud-output-topic",
        default="/perception/obstacle/pointcloud",
        help="Optional pointcloud relay for planner compatibility",
    )
    ap.add_argument("--frame-id", default="base_link", help="Override output OccupancyGrid frame_id")
    ap.add_argument("--resolution", type=float, default=0.2)
    ap.add_argument("--width", type=int, default=200)
    ap.add_argument("--height", type=int, default=200)
    ap.add_argument("--origin-x", type=float, default=-20.0)
    ap.add_argument("--origin-y", type=float, default=-20.0)
    ap.add_argument("--min-z", type=float, default=-2.5)
    ap.add_argument("--max-z", type=float, default=2.5)
    ap.add_argument("--unknown-default", action="store_true", help="Fill unseen cells with -1 instead of 0")
    args = ap.parse_args()

    rclpy.init()
    node = PointCloudToOccupancyGrid(args)
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
