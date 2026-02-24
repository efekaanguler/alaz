import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import can
import struct
from threading import Thread
from time import sleep

# CAN IDs (example, adjust as needed)
ACC_ID = 0x330
STEER_ID = 0x220
BRAKE_ID = 0x110
STEER_FEEDBACK_ID = 0x120
SPEED_FEEDBACK_ID = 0x130

class CANBridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_can_bridge_node')
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)

        # Subscribers for control commands
        self.create_subscription(Int32, 'throttle_cmd', self.throttle_callback, 10)
        self.create_subscription(Int32, 'brake_cmd', self.brake_callback, 10)
        self.create_subscription(Float32, 'steering_cmd', self.steering_callback, 10)

        # Publishers for feedback
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.speed_pub = self.create_publisher(Float32, 'vehicle_speed', 10)

        # Command state
        self._throttle = 0
        self._brake = 0
        self._steering = 0.0
        self._direction = 1  # 1: D, 0: N, 2: R

        # Start CAN send/receive threads
        self._send_thread = Thread(target=self._send_loop, daemon=True)
        self._recv_thread = Thread(target=self._recv_loop, daemon=True)
        self._send_thread.start()
        self._recv_thread.start()

    def throttle_callback(self, msg):
        self._throttle = max(0, min(100, msg.data))

    def brake_callback(self, msg):
        self._brake = max(0, min(100, msg.data))

    def steering_callback(self, msg):
        self._steering = float(msg.data)

    def _send_loop(self):
        while rclpy.ok():
            # Send throttle (acceleration)
            acc_msg = can.Message(arbitration_id=ACC_ID, is_extended_id=False,
                                  data=[int(self._throttle), 0, self._direction, 0, 0, 0, 0, 0])
            self.bus.send(acc_msg, timeout=0.01)
            # Send brake
            brake_msg = can.Message(arbitration_id=BRAKE_ID, is_extended_id=False,
                                    data=[int(self._brake), 0, 0, 0, 0, 0, 0, 0])
            self.bus.send(brake_msg, timeout=0.01)
            # Send steering
            steer_bytes = list(bytearray(struct.pack('f', self._steering))) + [0, 0, 195, 0]
            steer_msg = can.Message(arbitration_id=STEER_ID, is_extended_id=False, data=steer_bytes)
            self.bus.send(steer_msg, timeout=0.01)
            sleep(0.04)  # 100Hz

    def _recv_loop(self):
        while rclpy.ok():
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            if msg.arbitration_id == STEER_FEEDBACK_ID:
                # Assume steering angle is float in first 4 bytes
                angle = struct.unpack('f', bytes(msg.data[:4]))[0]
                self.steering_pub.publish(Float32(data=angle))
            elif msg.arbitration_id == SPEED_FEEDBACK_ID:
                # Assume speed is float in first 4 bytes
                speed = struct.unpack('f', bytes(msg.data[:4]))[0]
                self.speed_pub.publish(Float32(data=speed))


def main():
    rclpy.init()
    node = CANBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
