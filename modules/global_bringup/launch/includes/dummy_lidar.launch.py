from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    perception_share = get_package_share_directory('perception')
    dummy_lidar_script = os.path.join(perception_share, '..', '..', 'lib', 'perception', 'dummy_lidar_publisher.py')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', dummy_lidar_script, '--rate', '10.0', '--obstacle'],
            output='screen'
        )
    ])
