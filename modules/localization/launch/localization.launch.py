#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    this_share = get_package_share_directory("localization")
    map_loader_share = get_package_share_directory("autoware_map_loader")
    autoware_launch_share = get_package_share_directory("autoware_launch")

    map_launch = os.path.join(
        map_loader_share, "launch", "lanelet2_map_loader.launch.xml"
    )
    localization_component = os.path.join(
        autoware_launch_share,
        "launch",
        "components",
        "tier4_localization_component.launch.xml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic = LaunchConfiguration("odom_topic")
    twist_cov_topic = LaunchConfiguration("twist_cov_topic")
    src_image = LaunchConfiguration("src_image")
    src_info = LaunchConfiguration("src_info")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("vehicle_model", default_value="rdw_vehicle"),
            DeclareLaunchArgument("system_run_mode", default_value="online"),
            DeclareLaunchArgument("pose_source", default_value="yabloc"),
            DeclareLaunchArgument(
                "twist_source",
                default_value="external",
                description="Wheel odometry supplies the twist estimate",
            ),
            DeclareLaunchArgument("gnss_enabled", default_value="false"),
            DeclareLaunchArgument("data_path", default_value="/autoware_data"),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="[63.5139, 2.6648, 0.0, 0.0, 0.0, 1.0, 0.0]",
            ),
            DeclareLaunchArgument(
                "lanelet2_map_path", default_value="/workspace/maps/map.osm"
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("src_image", default_value="/sensing/image_raw"),
            DeclareLaunchArgument("src_info", default_value="/sensing/camera_info"),
            DeclareLaunchArgument(
                "twist_cov_topic",
                default_value="/localization/twist_estimator/twist_with_covariance",
            ),
            DeclareLaunchArgument(
                "input_pointcloud",
                default_value="/perception/lidar/pointcloud",
            ),
            DeclareLaunchArgument(
                "localization_pointcloud_container_name",
                default_value="/pointcloud_container",
            ),
            Node(
                package="localization",
                executable="map_projector_info_pub.py",
                name="map_projector_info_pub",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(map_launch),
                launch_arguments={
                    "lanelet2_map_path": LaunchConfiguration("lanelet2_map_path"),
                    "lanelet2_map_topic": "/map/vector_map",
                    "lanelet2_map_marker_topic": "/map/vector_map_marker",
                }.items(),
            ),
            Node(
                package="localization",
                executable="odom_to_twist_cov.py",
                name="odom_to_twist_cov",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_topic": odom_topic,
                        "twist_topic": twist_cov_topic,
                        "frame_id": "base_link",
                    }
                ],
            ),
            GroupAction(
                [
                    SetRemap(
                        src="/sensing/camera/traffic_light/image_raw",
                        dst=src_image,
                    ),
                    SetRemap(
                        src="/sensing/camera/traffic_light/camera_info",
                        dst=src_info,
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(localization_component),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "vehicle_model": LaunchConfiguration("vehicle_model"),
                            "pose_source": LaunchConfiguration("pose_source"),
                            "twist_source": LaunchConfiguration("twist_source"),
                            "initial_pose": LaunchConfiguration("initial_pose"),
                            "data_path": LaunchConfiguration("data_path"),
                            "input_pointcloud": LaunchConfiguration(
                                "input_pointcloud"
                            ),
                            "localization_pointcloud_container_name": LaunchConfiguration(
                                "localization_pointcloud_container_name"
                            ),
                        }.items(),
                    ),
                ]
            ),
        ]
    )
