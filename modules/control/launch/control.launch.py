from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    control_param_path = os.path.join(
        get_package_share_directory("control"), "config", "control.yaml"
    )
    controller_param_path = os.path.join(
        get_package_share_directory("control"), "config", "controller.param.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                name="control_params",
                default_value=control_param_path,
                description="Path to control parameters file",
            ),
            DeclareLaunchArgument(
                name="controller_params",
                default_value=controller_param_path,
                description="Path to controller parameters file",
            ),
            Node(
                package="autoware_trajectory_follower_node",
                executable="controller_node_exe",
                name="trajectory_follower",
                parameters=[
                    LaunchConfiguration("control_params"),
                    LaunchConfiguration("controller_params"),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[],
                output="screen",
            ),
        ]
    )
