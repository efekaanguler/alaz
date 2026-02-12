from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Parametre: keyboard kontrolü aktif mi?
        DeclareLaunchArgument(
            'keyboard',
            default_value='false',
            description='Enable keyboard controls'
        ),
        
        # Her zaman çalışan node'lar
        Node(
            package='sim_pkg',
            executable='pointcloud_to_laserscan',
            name='pointcloud_to_laserscan',
            output='screen'
        ),
        
        Node(
            package='sim_pkg',
            executable='speed_steer_topics',
            name='speed_steer_topics',
            output='screen'
        ),
        
        Node(
            package='sim_pkg',
            executable='realistic_controls',
            name='realistic_controls',
            output='screen'
        ),
        
        # Sadece keyboard:=true ile çalışır
        Node(
            package='sim_pkg',
            executable='keyboard_controls',
            name='keyboard_controls',
            output='screen',
            condition=IfCondition(LaunchConfiguration('keyboard'))
        ),
    ])
