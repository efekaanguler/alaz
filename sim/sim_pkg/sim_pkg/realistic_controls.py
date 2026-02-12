#!/usr/bin/env python3

"""
Vehicle Control Converter with Servo-like Steering

Subscribes to direct control inputs and converts them to vehicle control commands.

Input Topics:
  /control/throttle (std_msgs/msg/Float32) - Direct throttle value (0.0-1.0)
  /control/brake (std_msgs/msg/Float32) - Direct brake value (0.0-1.0)
  /control/steering (std_msgs/msg/Int16) - Steering rate command (-255 to +255)

Output:
  /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/msg/CarlaEgoVehicleControl)

Steering Behavior:
  - Input value determines rotation speed (servo-like behavior)
  - -255: Fast left turn
  - -1: Slow left turn
  - 0: Straight (maintain current angle, then center)
  - +1: Slow right turn
  - +255: Fast right turn
  - Steering angle integrates over time based on the command
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int16
from carla_msgs.msg import CarlaEgoVehicleControl


class VehicleControlConverter(Node):
    def __init__(self):
        super().__init__('vehicle_control_converter')
        
        # Parameters
        self.declare_parameter('control_topic', '/carla/ego_vehicle/vehicle_control_cmd')
        self.declare_parameter('throttle_topic', '/control/throttle')
        self.declare_parameter('brake_topic', '/control/brake')
        self.declare_parameter('steering_topic', '/control/steering')
        
        # Steering servo parameters
        self.declare_parameter('max_steering_angle', 1.0)  # Maximum steering angle (normalized -1.0 to 1.0)
        self.declare_parameter('steering_speed_scale', 0.02)  # Speed scale factor (angle change per command unit per second)
        self.declare_parameter('return_to_center_speed', 0.5)  # Speed to return to center when command is 0
        self.declare_parameter('steering_deadzone', 5)  # Command deadzone (ignore commands below this value)
        
        # Deadzone parameters for throttle/brake
        self.declare_parameter('throttle_deadzone', 0.02)
        self.declare_parameter('brake_deadzone', 0.02)
        
        # Timeout parameter (seconds)
        self.declare_parameter('timeout', 0.5)
        
        # Publishing rate
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # Get parameters
        control_topic = self.get_parameter('control_topic').value
        throttle_topic = self.get_parameter('throttle_topic').value
        brake_topic = self.get_parameter('brake_topic').value
        steering_topic = self.get_parameter('steering_topic').value
        
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_speed_scale = self.get_parameter('steering_speed_scale').value
        self.return_to_center_speed = self.get_parameter('return_to_center_speed').value
        self.steering_cmd_deadzone = self.get_parameter('steering_deadzone').value
        
        self.throttle_deadzone = self.get_parameter('throttle_deadzone').value
        self.brake_deadzone = self.get_parameter('brake_deadzone').value
        
        self.timeout = self.get_parameter('timeout').value
        publish_rate = self.get_parameter('publish_rate').value
        self.dt = 1.0 / publish_rate  # Time step for integration
        
        # Control state
        self.throttle = 0.0
        self.brake = 0.0
        self.steering_command = 0  # Steering rate command (-255 to +255)
        self.current_steering_angle = 0.0  # Current steering angle (-1.0 to +1.0)
        
        # Last update times
        self.throttle_last_update = self.get_clock().now()
        self.brake_last_update = self.get_clock().now()
        self.steering_last_update = self.get_clock().now()
        
        # Subscribers
        self.throttle_sub = self.create_subscription(
            Float32,
            throttle_topic,
            self.throttle_callback,
            10
        )
        
        self.brake_sub = self.create_subscription(
            Float32,
            brake_topic,
            self.brake_callback,
            10
        )
        
        self.steering_sub = self.create_subscription(
            Int16,
            steering_topic,
            self.steering_callback,
            10
        )
        
        # Publisher
        self.control_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            control_topic,
            10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_control)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Vehicle Control Converter Started (Servo Steering)')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Subscribing to:')
        self.get_logger().info(f'  Throttle: {throttle_topic} (0.0-1.0)')
        self.get_logger().info(f'  Brake:    {brake_topic} (0.0-1.0)')
        self.get_logger().info(f'  Steering: {steering_topic} (-255 to +255)')
        self.get_logger().info(f'Publishing to: {control_topic}')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
        self.get_logger().info(f'Steering config:')
        self.get_logger().info(f'  Max angle: ±{self.max_steering_angle:.2f}')
        self.get_logger().info(f'  Speed scale: {self.steering_speed_scale:.4f}')
        self.get_logger().info(f'  Return speed: {self.return_to_center_speed:.2f}')
        self.get_logger().info(f'  Deadzone: ±{self.steering_cmd_deadzone}')
        self.get_logger().info(f'Timeout: {self.timeout} s')
        self.get_logger().info('='*60)

    
    def apply_deadzone(self, value, deadzone):
        """Apply deadzone to filter small values."""
        if abs(value) < deadzone:
            return 0.0
        return value
    
    def throttle_callback(self, msg: Float32):
        """Handle throttle input (0.0-1.0)."""
        value = max(0.0, min(1.0, msg.data))  # Clamp to valid range
        self.throttle = self.apply_deadzone(value, self.throttle_deadzone)
        self.throttle_last_update = self.get_clock().now()
        
        self.get_logger().debug(f'Throttle: {self.throttle:.3f}')
    
    def brake_callback(self, msg: Float32):
        """Handle brake input (0.0-1.0)."""
        value = max(0.0, min(1.0, msg.data))  # Clamp to valid range
        self.brake = self.apply_deadzone(value, self.brake_deadzone)
        self.brake_last_update = self.get_clock().now()
        
        self.get_logger().debug(f'Brake: {self.brake:.3f}')
    
    def steering_callback(self, msg: Int16):
        """Handle steering rate command (-255 to +255)."""
        command = max(-255, min(255, msg.data))  # Clamp to valid range
        
        # Apply deadzone
        if abs(command) < self.steering_cmd_deadzone:
            self.steering_command = 0
        else:
            self.steering_command = command
        
        self.steering_last_update = self.get_clock().now()
        
        self.get_logger().debug(f'Steering command: {self.steering_command}')
    
    def update_steering(self):
        """Update steering angle based on rate command (servo-like behavior)."""
        if self.steering_command != 0:
            # Active steering command: integrate based on command magnitude
            # Normalize command to [-1.0, 1.0] and scale by speed factor
            normalized_command = self.steering_command / 255.0
            steering_rate = normalized_command * self.steering_speed_scale / self.dt
            
            # Update angle
            self.current_steering_angle += steering_rate * self.dt
        else:
            # No command: slowly return to center
            if abs(self.current_steering_angle) > 0.001:
                direction = -1.0 if self.current_steering_angle > 0 else 1.0
                return_rate = direction * self.return_to_center_speed
                
                # Move toward center
                self.current_steering_angle += return_rate * self.dt
                
                # Clamp to not overshoot center
                if abs(self.current_steering_angle) < abs(return_rate * self.dt):
                    self.current_steering_angle = 0.0
            else:
                self.current_steering_angle = 0.0
        
        # Clamp to maximum angle
        self.current_steering_angle = max(-self.max_steering_angle, 
                                         min(self.max_steering_angle, 
                                             self.current_steering_angle))

    
    def check_timeout(self):
        """Check if any signal has timed out and reset to safe values."""
        now = self.get_clock().now()
        timeout_duration = rclpy.duration.Duration(seconds=self.timeout)
        
        if (now - self.throttle_last_update) > timeout_duration:
            if self.throttle != 0.0:
                self.get_logger().warn('Throttle signal timeout - setting to 0')
                self.throttle = 0.0
        
        if (now - self.brake_last_update) > timeout_duration:
            if self.brake != 0.0:
                self.get_logger().warn('Brake signal timeout - setting to 0')
                self.brake = 0.0
        
        if (now - self.steering_last_update) > timeout_duration:
            if self.steering_command != 0:
                self.get_logger().warn('Steering signal timeout - setting command to 0')
                self.steering_command = 0
    
    def publish_control(self):
        """Publish control command at fixed rate."""
        self.check_timeout()
        
        # Update steering angle based on rate command
        self.update_steering()
        
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle = float(self.throttle)
        msg.steer = float(self.current_steering_angle)
        msg.brake = float(self.brake)
        msg.hand_brake = False
        msg.reverse = False
        msg.gear = 1
        msg.manual_gear_shift = False
        
        self.control_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
