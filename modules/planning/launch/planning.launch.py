from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    behavior_param_path = os.path.join(
        get_package_share_directory("planning"), "config", "behavior_path_planner.param.yaml"
    )
    velocity_smoother_param_path = os.path.join(
        get_package_share_directory("planning"), "config", "default_velocity_smoother.param.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="behavior_path_planner_params",
                default_value=behavior_param_path,
                description="Path to behavior path planner parameters file",
            ),
            DeclareLaunchArgument(
                name="velocity_smoother_params",
                default_value=velocity_smoother_param_path,
                description="Path to velocity smoother parameters file",
            ),
            DeclareLaunchArgument(
                name="use_dummy_scenario",
                default_value="false",
                description="Publish a dummy scenario message for standalone planner testing",
            ),
            Node(
                package="autoware_behavior_path_planner",
                executable="autoware_behavior_path_planner_node",
                name="autoware_behavior_path_planner",
                parameters=[LaunchConfiguration("behavior_path_planner_params")],
                remappings=[],
                output="screen",
            ),
            Node(
                package="autoware_velocity_smoother",
                executable="velocity_smoother_node",
                name="velocity_smoother",
                parameters=[LaunchConfiguration("velocity_smoother_params")],
                remappings=[],
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration("use_dummy_scenario")),
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "-r",
                    "1",
                    "/autoware_behavior_path_planner/input/scenario",
                    "autoware_internal_planning_msgs/msg/Scenario",
                    "{}",
                ],
                output="screen",
            ),
        ]
    )
