import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


from autoware_perception_msgs.msg import PredictedObjects, PredictedObject, ObjectClassification, Shape
from unique_identifier_msgs.msg import UUID
from geometry_msgs.msg import Point32

import numpy as np
from sklearn.cluster import DBSCAN
import uuid 

class AlazLidar2D(Node):
    def __init__(self):
        super().__init__('alaz_lidar_2d')

  
        self.declare_parameter('max_range', 6.0)  #lidar menzili
        self.declare_parameter('scan_topic', '/scan')
        
        self.max_range = self.get_parameter('max_range').value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

    
        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback, #lidar veri yolladığında fonk çalıştır
            qos_profile_sensor_data
        )

   
        self.marker_pub = self.create_publisher(MarkerArray, '/alaz/visual_markers', 10) #rvize yollamak için
        
 
        self.autoware_pub = self.create_publisher(PredictedObjects, '/perception/object_recognition/objects', 10) #araca yollamak için

        self.get_logger().info(f'Alaz 2D Lidar Node (Autoware Mode) Aktif Menzil: {self.max_range}m')

    def scan_callback(self, msg):
  
        ranges = np.array(msg.ranges)
   
        valid_indices = np.isfinite(ranges) & (ranges > 0.2) & (ranges < self.max_range) #temizlik
        
        if np.sum(valid_indices) < 3: 
            return

        valid_ranges = ranges[valid_indices]
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid_angles = angles[valid_indices]

  
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((x, y))
        #polar kartezyen dönüşüm 


        points = points[::2] #her iki nktadan biri
        
        if len(points) == 0:
            return


        try:
    
            clustering = DBSCAN(eps=1.2, min_samples=4).fit(points) #noktalar arası mesafe 1.2 yapıştır engel sayen az 4 nokta yan yana gelirse engel
        except Exception as e:
            self.get_logger().warn(f'Clustering Hatasi: {e}')
            return

        labels = clustering.labels_
        unique_labels = set(labels)


        marker_array = MarkerArray()
        
        autoware_msg = PredictedObjects()
        autoware_msg.header = msg.header 
        cluster_id = 0
        
        for label in unique_labels:
            if label == -1: 
                continue

         
            cluster_points = points[labels == label]
            if len(cluster_points) < 3:
                continue

          
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            
        #kutu 
            center_x = (min_pt[0] + max_pt[0]) / 2.0
            center_y = (min_pt[1] + max_pt[1]) / 2.0
            dim_x = max(max_pt[0] - min_pt[0], 0.1)
            dim_y = max(max_pt[1] - min_pt[1], 0.1)
            
        #rviz için
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
            marker.scale.z = 0.5 
            marker.color.a = 0.8
            marker.color.r = 1.0 
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

      #autoware için mesaj!!!
            p_obj = PredictedObject()
            
        #rastgele kimlik
            u = uuid.uuid4()
            uuid_msg = UUID()
            uuid_msg.uuid = list(u.bytes) 
            p_obj.object_id = uuid_msg

           #bu bi engel
            p_obj.existence_probability = 1.0

        
            classification = ObjectClassification()
            classification.label = ObjectClassification.UNKNOWN
            classification.probability = 1.0
            p_obj.classification.append(classification)

          
            p_obj.kinematics.initial_pose_with_covariance.pose.position.x = float(center_x)
            p_obj.kinematics.initial_pose_with_covariance.pose.position.y = float(center_y)
            p_obj.kinematics.initial_pose_with_covariance.pose.position.z = 0.0
            
     
            p_obj.shape.type = Shape.BOUNDING_BOX
            p_obj.shape.dimensions.x = float(dim_x)
            p_obj.shape.dimensions.y = float(dim_y)
            p_obj.shape.dimensions.z = 1.0 
            
            autoware_msg.objects.append(p_obj)
            
            cluster_id += 1

      #gönder
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
