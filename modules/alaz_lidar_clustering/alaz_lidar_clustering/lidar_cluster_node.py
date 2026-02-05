import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np

class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visual_markers', 10)
        self.get_logger().info('Alaz Lidar Real Data Mode Active')

    def listener_callback(self, msg):
        # Veriyi Oku
        gen = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        points_list = list(gen)
        
        if len(points_list) == 0:
            return

        # Downsampling (Hız için)
        points = np.array([[p[0], p[1], p[2]] for p in points_list[::10]])

        # --- KRİTİK DÜZELTME BURADA ---
        # İçinde 'inf' (sonsuz) veya çok büyük sayı olan noktaları temizle
        points = points[np.isfinite(points).all(axis=1)]

        if len(points) == 0:
            return

        # Kümeleme
        try:
            clustering = DBSCAN(eps=0.7, min_samples=15).fit(points)
        except Exception as e:
            self.get_logger().warn(f'Clustering error: {e}')
            return

        labels = clustering.labels_
        marker_array = MarkerArray()
        unique_labels = set(labels)
        
        cluster_id = 0
        for label in unique_labels:
            if label == -1:
                continue
                
            cluster_points = points[labels == label]
            
            if len(cluster_points) == 0:
                continue

            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            
            center_x = float((min_pt[0] + max_pt[0]) / 2.0)
            center_y = float((min_pt[1] + max_pt[1]) / 2.0)
            center_z = float((min_pt[2] + max_pt[2]) / 2.0)
            
            dim_x = float(max(max_pt[0] - min_pt[0], 0.1))
            dim_y = float(max(max_pt[1] - min_pt[1], 0.1))
            dim_z = float(max(max_pt[2] - min_pt[2], 0.1))
            
            marker = Marker()
            marker.header.frame_id = "velodyne_top"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = center_z
            
            marker.scale.x = dim_x
            marker.scale.y = dim_y
            marker.scale.z = dim_z
            
            marker.color.a = 0.6
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            cluster_id += 1

        self.marker_pub.publish(marker_array)
        
        n_clusters = len(unique_labels) - (1 if -1 in labels else 0)
        if n_clusters > 0:
            self.get_logger().info(f'Detected {n_clusters} obstacles')

def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
