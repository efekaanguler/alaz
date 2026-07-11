#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import can


class CANBridgeNode(Node):
    def __init__(self):
        super().__init__("ros2_can_bridge_node")

        self.declare_parameter("interface", "can0")
        self.declare_parameter("channel_type", "socketcan")

        channel = self.get_parameter("interface").value
        channel_type = self.get_parameter("channel_type").value

        try:
            self.bus = can.Bus(interface=channel_type, channel=channel)
        except Exception as exc:
            self.get_logger().fatal(
                f"Cannot open CAN transport {channel_type}:{channel}: {exc}"
            )
            raise

        self.to_can_sub = self.create_subscription(
            Frame, "/to_can_bus", self.on_to_can_bus, 100
        )
        self.from_can_pub = self.create_publisher(Frame, "/from_can_bus", 100)

        self.create_timer(0.01, self.poll_can)

        self.get_logger().info(f"CAN transport bridge started on {channel_type}:{channel}")

    def on_to_can_bus(self, msg: Frame):
        data = bytes(msg.data[: msg.dlc])
        can_msg = can.Message(
            arbitration_id=msg.id,
            data=data,
            is_extended_id=msg.is_extended,
            is_remote_frame=msg.is_rtr,
        )
        try:
            self.bus.send(can_msg)
        except can.CanError as exc:
            self.get_logger().error(f"Failed to send CAN frame 0x{msg.id:X}: {exc}")

    def poll_can(self):
        for _ in range(100):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            frame = Frame()
            frame.id = msg.arbitration_id
            frame.is_extended = msg.is_extended_id
            frame.is_rtr = msg.is_remote_frame
            frame.is_error = msg.is_error_frame
            frame.dlc = msg.dlc
            data = list(msg.data)[:8]
            frame.data = data + [0] * (8 - len(data))
            self.from_can_pub.publish(frame)

    def destroy_node(self):
        try:
            self.bus.shutdown()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = CANBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
