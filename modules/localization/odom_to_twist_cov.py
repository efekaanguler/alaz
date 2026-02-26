#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

class OdomToTwistCov(Node):
    def __init__(self):
        super().__init__("odom_to_twist_cov")
        self.declare_parameter("odom_topic", "/vehicle/odometry")
        self.declare_parameter("twist_topic", "/localization/twist_estimator/twist_with_covariance")

        odom_topic = self.get_parameter("odom_topic").value
        twist_topic = self.get_parameter("twist_topic").value

        self.pub = self.create_publisher(TwistWithCovarianceStamped, twist_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, 10)

    def cb(self, msg: Odometry):
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.twist.twist = msg.twist.twist
        out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)

def main():
    rclpy.init()
    node = OdomToTwistCov()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()