#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    autoware_launch_share = get_package_share_directory("autoware_launch")
    tier4_control_share = get_package_share_directory("tier4_control_launch")
    vehicle_share = get_package_share_directory("rdw_vehicle_description")
    this_share = get_package_share_directory("control")

    control_launch = os.path.join(
        tier4_control_share, "launch", "control.launch.xml"
    )
    global_params_launch = os.path.join(
        get_package_share_directory("autoware_global_parameter_loader"),
        "launch",
        "global_params.launch.py",
    )
    control_config = os.path.join(autoware_launch_share, "config", "control")
    trajectory_config = os.path.join(control_config, "trajectory_follower")

    enable_safety_checks = LaunchConfiguration("enable_safety_checks")

    control_arguments = {
        "lateral_controller_mode": "mpc",
        "longitudinal_controller_mode": "pid",
        "check_external_emergency_heartbeat": "false",
        "launch_external_cmd_selector": "true",
        "launch_external_cmd_converter": "false",
        "launch_lane_departure_checker": enable_safety_checks,
        "launch_control_validator": enable_safety_checks,
        "launch_autonomous_emergency_braking": enable_safety_checks,
        "launch_collision_detector": enable_safety_checks,
        "launch_obstacle_collision_checker": "false",
        "launch_predicted_path_checker": "false",
        "launch_control_evaluator": enable_safety_checks,
        "use_aeb_autoware_state_check": "true",
        "input_objects_topic_name": LaunchConfiguration("input_objects_topic"),
        "input_pointcloud_topic_name": LaunchConfiguration(
            "input_pointcloud_topic"
        ),
        "vehicle_param_file": os.path.join(
            vehicle_share, "config", "vehicle_info.param.yaml"
        ),
        "nearest_search_param_path": os.path.join(
            control_config, "common", "nearest_search.param.yaml"
        ),
        "trajectory_follower_node_param_path": os.path.join(
            trajectory_config, "trajectory_follower_node.param.yaml"
        ),
        "lat_controller_param_path": os.path.join(
            trajectory_config, "lateral", "mpc.param.yaml"
        ),
        "lon_controller_param_path": os.path.join(
            trajectory_config, "longitudinal", "pid.param.yaml"
        ),
        "vehicle_cmd_gate_param_path": os.path.join(
            this_share, "config", "vehicle_cmd_gate.param.yaml"
        ),
        "lane_departure_checker_param_path": os.path.join(
            control_config,
            "lane_departure_checker",
            "lane_departure_checker.param.yaml",
        ),
        "control_validator_param_path": os.path.join(
            control_config, "control_validator", "control_validator.param.yaml"
        ),
        "operation_mode_transition_manager_param_path": os.path.join(
            control_config,
            "operation_mode_transition_manager",
            "operation_mode_transition_manager.param.yaml",
        ),
        "shift_decider_param_path": os.path.join(
            control_config, "shift_decider", "shift_decider.param.yaml"
        ),
        "obstacle_collision_checker_param_path": os.path.join(
            control_config,
            "obstacle_collision_checker",
            "obstacle_collision_checker.param.yaml",
        ),
        "external_cmd_selector_param_path": os.path.join(
            control_config,
            "external_cmd_selector",
            "external_cmd_selector.param.yaml",
        ),
        "aeb_param_path": os.path.join(
            control_config,
            "autoware_autonomous_emergency_braking",
            "autonomous_emergency_braking.param.yaml",
        ),
        "predicted_path_checker_param_path": os.path.join(
            control_config,
            "predicted_path_checker",
            "predicted_path_checker.param.yaml",
        ),
        "collision_detector_param_path": os.path.join(
            control_config,
            "autoware_collision_detector",
            "collision_detector.param.yaml",
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("vehicle_model", default_value="rdw_vehicle"),
            DeclareLaunchArgument(
                "enable_safety_checks",
                default_value="true",
                description="Enable map, perception, and trajectory safety checkers",
            ),
            DeclareLaunchArgument(
                "input_objects_topic",
                default_value="/perception/object_recognition/objects",
            ),
            DeclareLaunchArgument(
                "input_pointcloud_topic",
                default_value="/perception/obstacle/pointcloud",
            ),
            GroupAction(
                [
                    SetParameter(
                        name="use_sim_time",
                        value=LaunchConfiguration("use_sim_time"),
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(global_params_launch),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "vehicle_model": LaunchConfiguration("vehicle_model"),
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(control_launch),
                        launch_arguments=control_arguments.items(),
                    ),
                ]
            ),
        ]
    )
