import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class AutoPseudoPublisher(Node):
    def __init__(self):
        super().__init__('auto_pseudo_node')

        self.declare_parameter('speed_topic', '/simulation/pseudo_vehicle/vehicle_speed')
        self.declare_parameter('steering_topic', '/simulation/pseudo_vehicle/steering_angle')
        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        steering_topic = self.get_parameter('steering_topic').get_parameter_value().string_value

        self.speed_pub = self.create_publisher(Float32, speed_topic, 10)
        self.steer_pub = self.create_publisher(Float32, steering_topic, 10)

        # 20 Hz'de yayın yapacak timer (saniyede 20 mesaj)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = time.time()

        self.get_logger().info(
            f'Otomatik pseudo veri yayını başladı: speed={speed_topic}, steering={steering_topic}'
        )

    def timer_callback(self):
        # Düğüm başladığından beri geçen süre
        current_time = time.time() - self.start_time

        # GERÇEKÇİ SENARYO ÜRETİMİ (Sinüs dalgaları ile yumuşak geçişler)

        # Hız: 0 ile 15 m/s (yaklaşık 54 km/s) arasında yavaşça artıp azalır
        # math.sin(current_time / 5.0) periyodu yavaşlatır
        speed_val = 7.5 + 7.5 * math.sin(current_time / 5.0)

        # Direksiyon: -0.8 ile 0.8 (tam sol - tam sağ) arasında yumuşak dönüşler yapar
        # Hızdan farklı bir periyot (/ 3.0) kullanarak daha gerçekçi, asimetrik bir sürüş hissi veriyoruz
        steer_val = 0.8 * math.sin(current_time / 3.0)

        # Verileri yayınla
        speed_msg = Float32()
        speed_msg.data = float(speed_val)
        self.speed_pub.publish(speed_msg)

        steer_msg = Float32()
        steer_msg.data = float(steer_val)
        self.steer_pub.publish(steer_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoPseudoPublisher()   # kendi node class adın
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
