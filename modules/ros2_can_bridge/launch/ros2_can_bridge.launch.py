from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    interface = LaunchConfiguration('interface')
    channel_type = LaunchConfiguration('channel_type')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('interface', default_value='can0'),
        DeclareLaunchArgument('channel_type', default_value='socketcan'),
        Node(
            package='ros2_can_bridge',
            executable='bridge_node.py',
            name='ros2_can_bridge_node',
            output='screen',
            parameters=[{
                'interface': interface,
                'channel_type': channel_type,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
