#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import tty
import termios

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.speed_pub = self.create_publisher(Float32, '/vehicle_speed', 10)
        self.steer_pub = self.create_publisher(Float32, '/steering_angle', 10)
        self.speed = 0.0
        self.steer = 0.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        print("\n=== ALAZ Manuel Kontrol Aktif ===")
        print("w: İleri (+) | s: Geri (-)")
        print("a: Sol (+)   | d: Sağ (-)")
        print("Space: DUR   | q: Çıkış")
        print("---------------------------------")
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    self.speed = round(min(self.speed + 0.2, 2.0), 2)
                elif key == 's':
                    self.speed = round(max(self.speed - 0.2, -2.0), 2)
                elif key == 'a':
                    self.steer = round(min(self.steer + 0.1, 1.0), 2)
                elif key == 'd':
                    self.steer = round(max(self.steer - 0.1, -1.0), 2)
                elif key == ' ':
                    self.speed = 0.0
                    self.steer = 0.0
                elif key == 'q':
                    print("\nÇıkış yapılıyor...")
                    break

                self.speed_pub.publish(Float32(data=float(self.speed)))
                self.steer_pub.publish(Float32(data=float(self.steer)))
                sys.stdout.write(f"\r[KOMUT] Hız: {self.speed:.2f} | Direksiyon: {self.steer:.2f}    ")
                sys.stdout.flush()
        except Exception as e:
            print(f"\nHata: {e}")

def main():
    rclpy.init()
    node = Teleop()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
