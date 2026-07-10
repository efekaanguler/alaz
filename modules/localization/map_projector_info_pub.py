#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from autoware_map_msgs.msg import MapProjectorInfo
from geographic_msgs.msg import GeoPoint

class MapProjectorInfoPub(Node):
    def __init__(self):
        super().__init__("map_projector_info_pub")

        self.declare_parameter("map_projector_info_topic", "/map/map_projector_info")
        self.declare_parameter("projector_type", "LOCAL")
        self.declare_parameter("vertical_datum", "WGS84")
        self.declare_parameter("mgrs_grid", "")
        self.declare_parameter("map_origin_latitude", 0.0)
        self.declare_parameter("map_origin_longitude", 0.0)
        self.declare_parameter("map_origin_altitude", 0.0)
        self.declare_parameter("scale_factor", 1.0)

        topic = self.get_parameter("map_projector_info_topic").value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MapProjectorInfo, topic, qos)

        msg = MapProjectorInfo()
        projector_type = str(self.get_parameter("projector_type").value).upper()
        vertical_datum = str(self.get_parameter("vertical_datum").value).upper()
        msg.projector_type = getattr(MapProjectorInfo, projector_type, MapProjectorInfo.LOCAL)
        msg.vertical_datum = getattr(MapProjectorInfo, vertical_datum, MapProjectorInfo.WGS84)
        msg.mgrs_grid = str(self.get_parameter("mgrs_grid").value)
        msg.map_origin = GeoPoint(
            latitude=float(self.get_parameter("map_origin_latitude").value),
            longitude=float(self.get_parameter("map_origin_longitude").value),
            altitude=float(self.get_parameter("map_origin_altitude").value),
        )
        msg.scale_factor = float(self.get_parameter("scale_factor").value)

        self.msg = msg

        # publish immediately and keep publishing (safe)
        self.pub.publish(self.msg)
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Publishing {topic} ({projector_type}, {vertical_datum}) TRANSIENT_LOCAL.")

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
