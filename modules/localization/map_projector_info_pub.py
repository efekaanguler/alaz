#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from autoware_map_msgs.msg import MapProjectorInfo
from geographic_msgs.msg import GeoPoint

class MapProjectorInfoPub(Node):
    def __init__(self):
        super().__init__("map_projector_info_pub")

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

        self.msg = msg

        # publish immediately and keep publishing (safe)
        self.pub.publish(self.msg)
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info("Publishing /map/map_projector_info (Local, WGS84) TRANSIENT_LOCAL.")

    def _tick(self):
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    rclpy.spin(MapProjectorInfoPub())
    rclpy.shutdown()

if __name__ == "__main__":
    main()