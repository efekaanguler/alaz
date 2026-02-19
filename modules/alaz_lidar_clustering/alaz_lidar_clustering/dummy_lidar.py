import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import math
import time

class DummyLidar(Node):
    def __init__(self):
        super().__init__('dummy_lidar_publisher')
        # Senin kodunun dinlediği topic ismine veri basıyoruz
        self.publisher_ = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('📡 Sanal Lidar Testi Başladı! Veri gönderiliyor...')

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.height = 1
        msg.width = 1000
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12000
        msg.is_dense = True
        
        points = []
        t = time.time()
        # Hareket eden sanal engel
        cx = 5.0 + 2.0 * math.sin(t)
        cy = 0.0
        
        for i in range(1000):
            if i < 200: # Engel kümesi
                x = cx + np.random.normal(0, 0.2)
                y = cy + np.random.normal(0, 0.2)
                z = np.random.normal(0, 0.5)
            else: # Rastgele gürültü
                x = np.random.uniform(-10, 10)
                y = np.random.uniform(-10, 10)
                z = np.random.uniform(-1, 1)
            
            points.append(struct.pack('fff', x, y, z))

        msg.data = b''.join(points)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
