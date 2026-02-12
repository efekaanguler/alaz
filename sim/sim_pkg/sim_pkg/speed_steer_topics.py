#!/usr/bin/env python3

"""
Republish vehicle speed and steering angle from CARLA bridge.

Subscribes to:
  /carla/ego_vehicle/vehicle_status (carla_msgs/msg/CarlaEgoVehicleStatus)

Publishes:
  /speed (std_msgs/msg/Float32) - vehicle speed in km/h
  /steering_angle (std_msgs/msg/Float32) - steering angle in radians
"""

import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaEgoVehicleStatus
from std_msgs.msg import Float32


class SpeedSteerRepublisher(Node):
    def __init__(self):
        super().__init__('speed_steer_republisher')
        
        # Parameters
        self.declare_parameter('max_steering_angle', 1.22)  # radians (~70 degrees)
        self.declare_parameter('input_topic', '/carla/ego_vehicle/vehicle_status')
        self.declare_parameter('speed_topic', '/speed')
        self.declare_parameter('steering_topic', '/steering_angle')
        
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        input_topic = self.get_parameter('input_topic').value
        speed_topic = self.get_parameter('speed_topic').value
        steering_topic = self.get_parameter('steering_topic').value
        
        # Subscriber
        self.vehicle_status_sub = self.create_subscription(
            CarlaEgoVehicleStatus,
            input_topic,
            self.vehicle_status_callback,
            10
        )
        
        # Publishers
        self.speed_pub = self.create_publisher(Float32, speed_topic, 10)
        self.steering_pub = self.create_publisher(Float32, steering_topic, 10)
        
        self.get_logger().info(f'Speed/Steer Republisher started')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output speed: {speed_topic} (hm/h)')
        self.get_logger().info(f'  Output steering: {steering_topic} (radians)')
        self.get_logger().info(f'  Max steering angle: {self.max_steering_angle:.3f} rad')
    
    def vehicle_status_callback(self, msg: CarlaEgoVehicleStatus):
        """Convert and republish speed and steering angle."""
        
        # Speed: m/s -> km/h
        speed = msg.velocity * 36
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        
        # Steering: normalized (-1 to 1) -> radians
        # Negative values = left, positive = right
        steering_rad = msg.control.steer * self.max_steering_angle
        steering_msg = Float32()
        steering_msg.data = steering_rad
        self.steering_pub.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedSteerRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
