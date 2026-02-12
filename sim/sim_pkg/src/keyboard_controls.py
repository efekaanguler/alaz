#!/usr/bin/env python3

"""
Keyboard control for CARLA ego vehicle.

Controls:
  W - Throttle (accelerate)
  S - Brake
  A - Steer left
  D - Steer right
  Space - Hand brake
  Q - Quit

Publishes to:
  /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/msg/CarlaEgoVehicleControl)
"""

import sys
import threading

import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaEgoVehicleControl

try:
    from pynput import keyboard
    from pynput.keyboard import Key
except ImportError:
    print("ERROR: pynput not installed!")
    print("Install with: pip3 install pynput")
    sys.exit(1)


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Parameters
        self.declare_parameter('control_topic', '/carla/ego_vehicle/vehicle_control_cmd')
        self.declare_parameter('throttle_value', 0.6)
        self.declare_parameter('brake_value', 0.8)
        self.declare_parameter('steer_value', 0.5)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        control_topic = self.get_parameter('control_topic').value
        self.throttle_val = self.get_parameter('throttle_value').value
        self.brake_val = self.get_parameter('brake_value').value
        self.steer_val = self.get_parameter('steer_value').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher
        self.control_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            control_topic,
            10
        )
        
        # Control state
        self.lock = threading.Lock()
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.hand_brake = False
        self.running = True
        
        # Pressed keys tracking
        self.pressed_keys = set()
        
        # Timer for publishing control commands
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_control)
        
        # Start keyboard listener in separate thread
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.listener.start()
        
        self.get_logger().info('='*50)
        self.get_logger().info('Keyboard Controller Started')
        self.get_logger().info('='*50)
        self.get_logger().info('Controls:')
        self.get_logger().info('  W     - Throttle (accelerate)')
        self.get_logger().info('  S     - Brake')
        self.get_logger().info('  A     - Steer left')
        self.get_logger().info('  D     - Steer right')
        self.get_logger().info('  Space - Hand brake')
        self.get_logger().info('  Q     - Quit')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Publishing to: {control_topic}')
        self.get_logger().info(f'Throttle: {self.throttle_val}, Brake: {self.brake_val}, Steer: {self.steer_val}')
        self.get_logger().info('='*50)
    
    def on_key_press(self, key):
        """Handle key press events."""
        try:
            # Handle character keys
            if hasattr(key, 'char') and key.char:
                self.pressed_keys.add(key.char.lower())
                
                # Quit on 'q'
                if key.char.lower() == 'q':
                    self.get_logger().info('Quitting...')
                    self.running = False
                    self.listener.stop()
                    rclpy.shutdown()
            
            # Handle special keys
            elif key == Key.space:
                with self.lock:
                    self.hand_brake = True
                self.get_logger().info('Hand brake: ON')
        
        except AttributeError:
            pass
        
        self.update_control_state()
    
    def on_key_release(self, key):
        """Handle key release events."""
        try:
            # Handle character keys
            if hasattr(key, 'char') and key.char:
                self.pressed_keys.discard(key.char.lower())
            
            # Handle special keys
            elif key == Key.space:
                with self.lock:
                    self.hand_brake = False
                self.get_logger().info('Hand brake: OFF')
        
        except AttributeError:
            pass
        
        self.update_control_state()
    
    def update_control_state(self):
        """Update control values based on pressed keys."""
        with self.lock:
            # Throttle
            if 'w' in self.pressed_keys:
                self.throttle = self.throttle_val
            else:
                self.throttle = 0.0
            
            # Brake
            if 's' in self.pressed_keys:
                self.brake = self.brake_val
            else:
                self.brake = 0.0
            
            # Steering
            steer = 0.0
            if 'a' in self.pressed_keys:
                steer -= self.steer_val  # Left (negative)
            if 'd' in self.pressed_keys:
                steer += self.steer_val  # Right (positive)
            
            # Clamp steering to [-1, 1]
            self.steer = max(-1.0, min(1.0, steer))
    
    def publish_control(self):
        """Publish control command at fixed rate."""
        if not self.running:
            return
        
        with self.lock:
            msg = CarlaEgoVehicleControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.throttle = float(self.throttle)
            msg.steer = float(self.steer)
            msg.brake = float(self.brake)
            msg.hand_brake = self.hand_brake
            msg.reverse = False
            msg.gear = 1
            msg.manual_gear_shift = False
        
        self.control_pub.publish(msg)
    
    def shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down keyboard controller...')
        self.running = False
        if self.listener.is_alive():
            self.listener.stop()
        
        # Send zero control command
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle = 0.0
        msg.steer = 0.0
        msg.brake = 0.0
        msg.hand_brake = False
        msg.reverse = False
        msg.gear = 1
        msg.manual_gear_shift = False
        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
