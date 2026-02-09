import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# --- AUTOWARE MESAJLARI ---
from autoware_perception_msgs.msg import PredictedObjects, PredictedObject, ObjectClassification, Shape
from unique_identifier_msgs.msg import UUID
from geometry_msgs.msg import Point32

import numpy as np
from sklearn.cluster import DBSCAN
import uuid # Rastgele kimlik olusturmak icin

class AlazLidar2D(Node):
    def __init__(self):
        super().__init__('alaz_lidar_2d')

        # --- PARAMETRELER ---
        self.declare_parameter('max_range', 6.0) # Kural: 6 Metre
        self.declare_parameter('scan_topic', '/scan')
        
        self.max_range = self.get_parameter('max_range').value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # --- ABONELİK ---
        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        # --- YAYINCILAR ---
        # 1. Bizim gozumuz (Rviz icin gorsel kutular)
        self.marker_pub = self.create_publisher(MarkerArray, '/alaz/visual_markers', 10)
        
        # 2. Otonom Aracin Beyni (Autoware icin veriler)
        self.autoware_pub = self.create_publisher(PredictedObjects, '/perception/object_recognition/objects', 10)

        self.get_logger().info(f'Alaz 2D Lidar Node (Autoware Mode) Aktif! Menzil: {self.max_range}m')

    def scan_callback(self, msg):
        # 1. VERI HAZIRLIGI (Polar -> Cartesian)
        ranges = np.array(msg.ranges)
        # Menzil filtresi (0.2m'den yakinlari ve 6m'den uzaklari at)
        valid_indices = np.isfinite(ranges) & (ranges > 0.2) & (ranges < self.max_range)
        
        if np.sum(valid_indices) < 3: # En az 3 nokta yoksa islem yapma
            return

        valid_ranges = ranges[valid_indices]
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid_angles = angles[valid_indices]

        # X ve Y hesapla
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((x, y))

        # Downsampling (Islemciyi yormamak icin her 2 noktadan 1'ini al)
        points = points[::2]
        
        if len(points) == 0:
            return

        # 2. KUMELEME (DBSCAN)
        try:
            # eps: 0.5m (noktalar arasi maks mesafe), min_samples: 4 nokta
            clustering = DBSCAN(eps=1.2, min_samples=4).fit(points)
        except Exception as e:
            self.get_logger().warn(f'Clustering Hatasi: {e}')
            return

        labels = clustering.labels_
        unique_labels = set(labels)

        # Mesaj Paketlerini Hazirla
        marker_array = MarkerArray()
        
        autoware_msg = PredictedObjects()
        autoware_msg.header = msg.header # Lidar zamaniyla ayni olsun

        cluster_id = 0
        
        for label in unique_labels:
            if label == -1: # Gurultuyu atla
                continue

            # Kume noktalarini cek
            cluster_points = points[labels == label]
            if len(cluster_points) < 3:
                continue

            # Kutunun sinirlarini bul
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            
            # Merkez ve Boyutlar
            center_x = (min_pt[0] + max_pt[0]) / 2.0
            center_y = (min_pt[1] + max_pt[1]) / 2.0
            dim_x = max(max_pt[0] - min_pt[0], 0.1)
            dim_y = max(max_pt[1] - min_pt[1], 0.1)
            
            # --- A) GORSEL YAYIN (RVIZ MARKER) ---
            marker = Marker()
            marker.header = msg.header
            marker.ns = "obstacles"
            marker.id = cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.0
            marker.scale.x = float(dim_x)
            marker.scale.y = float(dim_y)
            marker.scale.z = 0.5 # Gorsel yukseklik
            marker.color.a = 0.8
            marker.color.r = 1.0 # KIRMIZI
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            # --- B) AUTOWARE YAYINI (PREDICTED OBJECT) ---
            p_obj = PredictedObject()
            
            # 1. Kimlik (UUID)
            u = uuid.uuid4()
            uuid_msg = UUID()
            uuid_msg.uuid = list(u.bytes) # Python UUID -> ROS UUID
            p_obj.object_id = uuid_msg

            # 2. Olasilik (Bu kesin bir engeldir)
            p_obj.existence_probability = 1.0

            # 3. Siniflandirma (UNKNOWN = 0)
            classification = ObjectClassification()
            classification.label = ObjectClassification.UNKNOWN
            classification.probability = 1.0
            p_obj.classification.append(classification)

            # 4. Pozisyon (Kinematics)
            p_obj.kinematics.initial_pose_with_covariance.pose.position.x = float(center_x)
            p_obj.kinematics.initial_pose_with_covariance.pose.position.y = float(center_y)
            p_obj.kinematics.initial_pose_with_covariance.pose.position.z = 0.0
            
            # 5. Sekil (Shape)
            p_obj.shape.type = Shape.BOUNDING_BOX
            p_obj.shape.dimensions.x = float(dim_x)
            p_obj.shape.dimensions.y = float(dim_y)
            p_obj.shape.dimensions.z = 1.0 # Standart engel yuksekligi
            
            autoware_msg.objects.append(p_obj)
            
            cluster_id += 1

        # Iki mesaji da yayinla
        self.marker_pub.publish(marker_array)
        self.autoware_pub.publish(autoware_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AlazLidar2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
