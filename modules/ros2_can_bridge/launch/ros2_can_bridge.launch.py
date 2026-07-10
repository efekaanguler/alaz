from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='ros2_can_bridge',
            executable='bridge_node.py',
            name='ros2_can_bridge_node',
            output='screen',
            parameters=[{
                'interface': 'can0',
                'channel_type': 'socketcan',
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
