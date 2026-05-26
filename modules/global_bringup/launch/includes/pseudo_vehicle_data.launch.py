from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pseudo_vehicle_data',
            executable='auto_pseudo',
            name='auto_pseudo_node',
            output='screen'
        )
    ])
