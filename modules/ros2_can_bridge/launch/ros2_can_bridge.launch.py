from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_can_bridge',
            executable='bridge_node.py',
            name='ros2_can_bridge_node',
            output='screen',
        ),
    ])
