from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_can_bridge',
            executable='ros2_can_bridge_node',
            name='ros2_can_bridge_node',
            output='screen',
        ),
    ])
