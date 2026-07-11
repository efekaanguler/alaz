from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    perception_share = get_package_share_directory('perception')
    dummy_lidar_script = os.path.join(perception_share, '..', '..', 'lib', 'perception', 'dummy_lidar_publisher.py')
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dummy_lidar_static_tf',
            arguments=['1.36', '0.0', '0.17', '0.0', '0.0', '0.0',
                       'base_link', 'lidar_link'],
        ),
        ExecuteProcess(
            cmd=['python3', dummy_lidar_script, '--rate', '10.0', '--obstacle'],
            output='screen'
        )
    ])
