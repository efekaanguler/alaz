from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    perception_share = get_package_share_directory('perception')

    laserscan_to_pcl_launch_path = os.path.join(perception_share, 'launch', 'laserscan_to_pcl_and_occ.launch.xml')

    return LaunchDescription([
        # Fix H2: Instead of a dead relay from a non-existent pointcloud_raw,
        # actually launch the LaserScan-to-PointCloud2 converter here.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(laserscan_to_pcl_launch_path),
            launch_arguments={
                'scan_topic': '/sensing/lidar/top/scan',
                'points_topic': '/sensing/lidar/concatenated/pointcloud',
                'frame_id': 'lidar_link'
            }.items(),
        ),
    ])
