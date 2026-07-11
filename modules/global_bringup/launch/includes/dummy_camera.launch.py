from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    perception_share = get_package_share_directory('perception')
    dummy_camera_script = os.path.join(perception_share, '..', '..', 'lib', 'perception', 'dummy_camera_publisher.py')
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dummy_camera_static_tf',
            arguments=['0.98', '0.0', '0.39', '0.0', '0.0', '0.0',
                       'base_link', 'camera_center_link'],
        ),
        ExecuteProcess(
            cmd=['python3', dummy_camera_script, '--width', '1280',
                 '--height', '720'],
            output='screen'
        )
    ])
