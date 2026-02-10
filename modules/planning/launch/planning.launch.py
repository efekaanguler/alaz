from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    planning_param_path = os.path.join(
        get_package_share_directory("planning"), "config", "planning.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="planning_params",
                default_value=planning_param_path,
                description="Path to planning parameters file",
            ),
            Node(
                package="behavior_path_planner",
                executable="behavior_path_planner_node",
                name="behavior_path_planner",
                parameters=[LaunchConfiguration("planning_params")],
                remappings=[],
                output="screen",
            ),
            Node(
                package="motion_velocity_smoother",
                executable="motion_velocity_smoother_node",
                name="motion_velocity_smoother",
                parameters=[LaunchConfiguration("planning_params")],
                remappings=[],
                output="screen",
            ),
        ]
    )
