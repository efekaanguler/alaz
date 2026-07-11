import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

class VehicleSimulatorNode(Node):
    def __init__(self):
        super().__init__('vehicle_simulator_node')

        self.declare_parameter('throttle_topic', '/simulation/pseudo_vehicle/throttle_cmd')
        self.declare_parameter('brake_topic', '/simulation/pseudo_vehicle/brake_cmd')
        self.declare_parameter('steering_cmd_topic', '/simulation/pseudo_vehicle/steering_cmd')
        self.declare_parameter('speed_topic', '/simulation/pseudo_vehicle/vehicle_speed')
        self.declare_parameter('steering_topic', '/simulation/pseudo_vehicle/steering_angle')

        throttle_topic = self.get_parameter('throttle_topic').get_parameter_value().string_value
        brake_topic = self.get_parameter('brake_topic').get_parameter_value().string_value
        steering_cmd_topic = self.get_parameter('steering_cmd_topic').get_parameter_value().string_value
        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        steering_topic = self.get_parameter('steering_topic').get_parameter_value().string_value

        self.create_subscription(Int32, throttle_topic, self.throttle_cb, 10)
        self.create_subscription(Int32, brake_topic, self.brake_cb, 10)
        self.create_subscription(Float32, steering_cmd_topic, self.steering_cb, 10)

        self.speed_pub = self.create_publisher(Float32, speed_topic, 10)
        self.steer_pub = self.create_publisher(Float32, steering_topic, 10)

        self.current_speed = 0.0
        self.current_steering = 0.0

        self.cmd_throttle = 0
        self.cmd_brake = 0
        self.cmd_steering = 0.0

        self.max_accel = 3.0
        self.max_decel = 8.0
        self.friction_decel = 0.5
        self.steer_speed = 2.0

        self.dt = 0.05 # 20 Hz
        self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f'Araç simülasyonu başladı: speed={speed_topic}, steering={steering_topic}'
        )

    def throttle_cb(self, msg):
        self.cmd_throttle = max(0, min(100, msg.data))

    def brake_cb(self, msg):
        self.cmd_brake = max(0, min(100, msg.data))

    def steering_cb(self, msg):
        self.cmd_steering = max(-1.0, min(1.0, msg.data))

    def timer_callback(self):
        if self.cmd_brake > 0:
            decel = (self.cmd_brake / 100.0) * self.max_decel
            self.current_speed -= decel * self.dt
        elif self.cmd_throttle > 0:
            accel = (self.cmd_throttle / 100.0) * self.max_accel
            self.current_speed += accel * self.dt
        else:
            self.current_speed -= self.friction_decel * self.dt

        if self.current_speed < 0.0:
            self.current_speed = 0.0

        steer_diff = self.cmd_steering - self.current_steering
        step = self.steer_speed * self.dt

        if abs(steer_diff) < step:
            self.current_steering = self.cmd_steering
        else:
            self.current_steering += step if steer_diff > 0 else -step

        speed_msg = Float32()
        speed_msg.data = float(self.current_speed)
        self.speed_pub.publish(speed_msg)

        steer_msg = Float32()
        steer_msg.data = float(self.current_steering)
        self.steer_pub.publish(steer_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
