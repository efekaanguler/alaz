import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from autoware_perception_msgs.msg import DetectedObjects, DetectedObject, ObjectClassification, Shape
import numpy as np

class AlazLidar2D(Node):
    def __init__(self):
        super().__init__('alaz_lidar_2d')

        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('scan_topic', '/sensing/lidar/top/scan')
        self.declare_parameter('cluster_tolerance', 0.8)
        self.declare_parameter('min_cluster_size', 3)

        self.max_range = self.get_parameter('max_range').value
        self.tolerance = self.get_parameter('cluster_tolerance').value
        self.min_size = self.get_parameter('min_cluster_size').value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/alaz/visual_markers', 10)
        self.autoware_pub = self.create_publisher(DetectedObjects, '/perception/object_recognition/detection/clusters', 10)

        self.get_logger().info(f'Alaz 2D Lidar Aktif | Topic: {scan_topic} | Tol: {self.tolerance}m')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid_indices = np.isfinite(ranges) & (ranges > 0.2) & (ranges < self.max_range)

        if np.sum(valid_indices) < self.min_size:
            return

        valid_ranges = ranges[valid_indices]
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid_angles = angles[valid_indices]

        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((x, y))

        clusters = []
        if len(points) > 0:
            current_cluster = [points[0]]
            for i in range(1, len(points)):
                dist = np.linalg.norm(points[i] - points[i-1])
                if dist < self.tolerance:
                    current_cluster.append(points[i])
                else:
                    if len(current_cluster) >= self.min_size:
                        clusters.append(np.array(current_cluster))
                    current_cluster = [points[i]]
            if len(current_cluster) >= self.min_size:
                clusters.append(np.array(current_cluster))

        marker_array = MarkerArray()
        autoware_msg = DetectedObjects()
        autoware_msg.header = msg.header

        for cluster_id, cluster_points in enumerate(clusters):
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            center_x = (min_pt[0] + max_pt[0]) / 2.0
            center_y = (min_pt[1] + max_pt[1]) / 2.0
            dim_x = max(max_pt[0] - min_pt[0], 0.1)
            dim_y = max(max_pt[1] - min_pt[1], 0.1)

            marker = Marker()
            marker.header = msg.header
            marker.ns = "obstacles"
            marker.id = cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center_x)
            marker.pose.position.y = float(center_y)
            marker.pose.position.z = 0.0
            marker.scale.x = float(dim_x)
            marker.scale.y = float(dim_y)
            marker.scale.z = 0.5
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            p_obj = DetectedObject()
            p_obj.existence_probability = 1.0
            classification = ObjectClassification()
            classification.label = ObjectClassification.UNKNOWN
            classification.probability = 1.0
            p_obj.classification.append(classification)
            p_obj.kinematics.pose_with_covariance.pose.position.x = float(center_x)
            p_obj.kinematics.pose_with_covariance.pose.position.y = float(center_y)
            p_obj.kinematics.pose_with_covariance.pose.position.z = 0.0
            p_obj.shape.type = Shape.BOUNDING_BOX
            p_obj.shape.dimensions.x = float(dim_x)
            p_obj.shape.dimensions.y = float(dim_y)
            p_obj.shape.dimensions.z = 1.0
            autoware_msg.objects.append(p_obj)

        self.marker_pub.publish(marker_array)
        self.autoware_pub.publish(autoware_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AlazLidar2D()   # kendi node class adın
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
