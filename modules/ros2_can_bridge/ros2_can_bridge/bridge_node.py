#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import can
import struct

CAN_MSG_SENDING_SPEED = 0.04  # 25 Hz (script böyle). Yorumunda 100Hz yazmış ama 0.04 = 25Hz.

ACC_ID = 0x330
STEER_ID = 0x220
BRAKE_ID = 0x110

STEER_FEEDBACK_ID = 0x120
SPEED_FEEDBACK_ID = 0x130


class CANBridgeNode(Node):
    def __init__(self):
        super().__init__("ros2_can_bridge_node")

        # socketcan bitrate burada set edilmez; ip link ile ayarlanır
        self.bus = can.Bus(interface="socketcan", channel="can0")

        # state
        self.throttle = 0
        self.brake = 0
        self.steering = 0.0
        self.direction = 1   # 0=N,1=D,2=R
        self.active = 1

        # ROS I/O
        self.create_subscription(Int32, "throttle_cmd", self.cb_throttle, 10)
        self.create_subscription(Int32, "brake_cmd", self.cb_brake, 10)
        self.create_subscription(Float32, "steering_cmd", self.cb_steer, 10)
        self.create_subscription(Int32, "gear_cmd", self.cb_gear, 10)
        self.create_subscription(Int32, "active_cmd", self.cb_active, 10)

        self.steering_pub = self.create_publisher(Float32, "steering_angle", 10)
        self.speed_pub = self.create_publisher(Float32, "vehicle_speed", 10)

        # periodic CAN tasks (controller script gibi)
        self.brk_msg = can.Message(arbitration_id=BRAKE_ID, data=[0,0,0,0,0,0,0,0], is_extended_id=False)
        self.steer_msg = can.Message(arbitration_id=STEER_ID, data=[0,0,0,0,0,0,195,0], is_extended_id=False)
        self.acc_msg = can.Message(arbitration_id=ACC_ID, data=[0,0,1,0,0,0,0,0], is_extended_id=False)

        self.brk_task = self.bus.send_periodic(self.brk_msg, CAN_MSG_SENDING_SPEED)
        self.steer_task = self.bus.send_periodic(self.steer_msg, CAN_MSG_SENDING_SPEED)
        self.acc_task = self.bus.send_periodic(self.acc_msg, CAN_MSG_SENDING_SPEED)

        # CAN recv timer
        self.create_timer(0.01, self.poll_can)

        self.get_logger().info("CAN bridge started (periodic tasks running).")

    def cb_throttle(self, msg: Int32):
        v = int(msg.data)
        if v < 0: v = 0
        if v > 100: v = 100
        self.throttle = v
        self.update_can()

    def cb_brake(self, msg: Int32):
        v = int(msg.data)
        if v < 0: v = 0
        if v > 100: v = 100
        self.brake = v
        self.update_can()

    def cb_steer(self, msg: Float32):
        self.steering = float(msg.data)
        self.update_can()

    def cb_gear(self, msg: Int32):
        d = int(msg.data)
        if d not in (0, 1, 2):
            return
        self.direction = d
        self.update_can()

    def cb_active(self, msg: Int32):
        self.active = 1 if int(msg.data) != 0 else 0
        if self.active:
            self.acc_task.start()
            self.steer_task.start()
            self.brk_task.start()
            self.get_logger().info("Controls activated")
        else:
            self.acc_task.stop()
            self.steer_task.stop()
            self.brk_task.stop()
            self.get_logger().info("Controls deactivated")

    def update_can(self):
        if not self.active:
            return

        self.acc_msg.data = [int(self.throttle), 0, int(self.direction), 0, 0, 0, 0, 0]
        self.acc_task.modify_data(self.acc_msg)

        steer_bytes = list(struct.pack("<f", float(self.steering))) + [0, 0, 195, 0]
        self.steer_msg.data = steer_bytes
        self.steer_task.modify_data(self.steer_msg)

        self.brk_msg.data = [int(self.brake), 0, 0, 0, 0, 0, 0, 0]
        self.brk_task.modify_data(self.brk_msg)

    def poll_can(self):
        # non-blocking poll
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            if msg.arbitration_id == STEER_FEEDBACK_ID and len(msg.data) >= 4:
                angle = struct.unpack("<f", bytes(msg.data[:4]))[0]
                self.steering_pub.publish(Float32(data=float(angle)))
            if msg.arbitration_id == SPEED_FEEDBACK_ID and len(msg.data) >= 4:
                speed = struct.unpack("<f", bytes(msg.data[:4]))[0]
                self.speed_pub.publish(Float32(data=float(speed)))


def main():
    rclpy.init()
    node = CANBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()