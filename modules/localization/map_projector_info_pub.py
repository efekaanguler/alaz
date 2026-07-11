#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from autoware_map_msgs.msg import MapProjectorInfo
from geographic_msgs.msg import GeoPoint

import os
import yaml
from ament_index_python.packages import get_package_share_directory

class MapProjectorInfoPub(Node):
    def __init__(self):
        super().__init__("map_projector_info_pub")

        self.declare_parameter("lanelet2_map_path", "")
        map_path = self.get_parameter("lanelet2_map_path").get_parameter_value().string_value

        path = ""
        if map_path:
            path = os.path.join(os.path.dirname(map_path), "map_projector_info.yaml")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MapProjectorInfo, "/map/map_projector_info", qos)

        msg = MapProjectorInfo()
        msg.projector_type = MapProjectorInfo.LOCAL
        msg.vertical_datum = MapProjectorInfo.WGS84
        msg.mgrs_grid = ""
        msg.map_origin = GeoPoint(latitude=0.0, longitude=0.0, altitude=0.0)
        msg.scale_factor = 1.0

        if path and os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    data = yaml.safe_load(f)
                    if 'projector_type' in data:
                        if data['projector_type'].upper() == "MGRS":
                            msg.projector_type = MapProjectorInfo.MGRS
                        elif data['projector_type'].upper() == "LOCAL":
                            msg.projector_type = MapProjectorInfo.LOCAL
                    if 'map_origin' in data:
                        msg.map_origin.latitude = float(data['map_origin'].get('latitude', 0.0))
                        msg.map_origin.longitude = float(data['map_origin'].get('longitude', 0.0))
                        msg.map_origin.altitude = float(data['map_origin'].get('altitude', 0.0))
                self.get_logger().info(f"Loaded projector info from {path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load projector info from {path}: {e}")
        else:
            self.get_logger().info("No valid map_projector_info_path provided. Using default Local/WGS84 zero origin.")

        self.msg = msg

        # publish immediately and keep publishing (safe)
        self.pub.publish(self.msg)
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info("Publishing /map/map_projector_info (Local, WGS84) TRANSIENT_LOCAL.")

    def _tick(self):
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapProjectorInfoPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
