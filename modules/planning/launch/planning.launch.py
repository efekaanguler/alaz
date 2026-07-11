#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    autoware_planning = os.path.join(
        get_package_share_directory("autoware_launch"),
        "launch",
        "components",
        "tier4_planning_component.launch.xml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("vehicle_model", default_value="rdw_vehicle"),
            DeclareLaunchArgument(
                "pointcloud_container_name", default_value="/pointcloud_container"
            ),
            DeclareLaunchArgument("is_simulation", default_value="false"),
            DeclareLaunchArgument(
                "enable_all_modules_auto_mode",
                default_value="true",
                description="Allow planning scene modules to activate without RTC UI",
            ),
            DeclareLaunchArgument(
                "input_objects_topic",
                default_value="/perception/object_recognition/objects",
            ),
            DeclareLaunchArgument(
                "input_pointcloud_topic",
                default_value="/perception/obstacle/pointcloud",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(autoware_planning),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "vehicle_model": LaunchConfiguration("vehicle_model"),
                    "is_simulation": LaunchConfiguration("is_simulation"),
                    "enable_all_modules_auto_mode": LaunchConfiguration(
                        "enable_all_modules_auto_mode"
                    ),
                    "planning_input_objects_topic_name": LaunchConfiguration(
                        "input_objects_topic"
                    ),
                    "planning_input_pointcloud_topic_name": LaunchConfiguration(
                        "input_pointcloud_topic"
                    ),
                }.items(),
            ),
        ]
    )
