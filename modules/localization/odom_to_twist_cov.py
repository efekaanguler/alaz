#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

class OdomToTwistCov(Node):
    def __init__(self):
        super().__init__("odom_to_twist_cov")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("twist_topic", "/localization/twist_estimator/twist_with_covariance")
        self.declare_parameter("frame_id", "base_link")

        odom_topic = self.get_parameter("odom_topic").value
        twist_topic = self.get_parameter("twist_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(TwistWithCovarianceStamped, twist_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, 10)

    def cb(self, msg: Odometry):
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.twist.twist = msg.twist.twist
        out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTwistCov()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
