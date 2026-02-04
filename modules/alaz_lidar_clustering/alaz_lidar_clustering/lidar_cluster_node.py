
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

            '/sensing/lidar/top/pointcloud_raw',

            self.listener_callback,

            10)

            

        self.marker_pub = self.create_publisher(MarkerArray, '/visual_markers', 10)

        self.get_logger().info('🚀 Alaz Lidar: Kutu Çizme Modu Aktif!')


    def listener_callback(self, msg):

        gen = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)

        points_list = list(gen)

        

        if len(points_list) == 0:

            return


        points = np.array([[p[0], p[1], p[2]] for p in points_list])


        clustering = DBSCAN(eps=0.5, min_samples=10).fit(points)

        labels = clustering.labels_


        marker_array = MarkerArray()

        unique_labels = set(labels)

        

        cluster_id = 0

        for label in unique_labels:

            if label == -1: 

                continue

                

            cluster_points = points[labels == label]

            

            min_pt = np.min(cluster_points, axis=0)

            max_pt = np.max(cluster_points, axis=0)

            

            # --- DÜZELTME BURADA ---

            # NumPy sayılarını saf Python float'ına çeviriyoruz

            center_x = float((min_pt[0] + max_pt[0]) / 2.0)

            center_y = float((min_pt[1] + max_pt[1]) / 2.0)

            center_z = float((min_pt[2] + max_pt[2]) / 2.0)

            

            dim_x = float(max(max_pt[0] - min_pt[0], 0.1))

            dim_y = float(max(max_pt[1] - min_pt[1], 0.1))

            dim_z = float(max(max_pt[2] - min_pt[2], 0.1))

            

            marker = Marker()

            marker.header.frame_id = "map"

            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "obstacles"

            marker.id = cluster_id

            marker.type = Marker.CUBE

            marker.action = Marker.ADD

            

            # Artık bunlar saf float, ROS kızmayacak:

            marker.pose.position.x = center_x

            marker.pose.position.y = center_y

            marker.pose.position.z = center_z

            

            marker.scale.x = dim_x

            marker.scale.y = dim_y

            marker.scale.z = dim_z

            

            marker.color.a = 0.5

            marker.color.r = 0.0

            marker.color.g = 1.0

            marker.color.b = 0.0

            

            marker_array.markers.append(marker)

            cluster_id += 1


        self.marker_pub.publish(marker_array)

        n_clusters = len(unique_labels) - (1 if -1 in labels else 0)

        self.get_logger().info(f'📦 {n_clusters} engel kutusu çizildi!')


def main(args=None):

    rclpy.init(args=args)

    node = LidarClusterNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()

