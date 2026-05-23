#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Inputs
    lanelet2_map_path = LaunchConfiguration("lanelet2_map_path")
    odom_topic = LaunchConfiguration("odom_topic")
    src_image = LaunchConfiguration("src_image")
    src_info = LaunchConfiguration("src_info")

    # Topics
    twist_cov_topic = LaunchConfiguration("twist_cov_topic")
    projected_cloud = "/projected_line_segments_cloud"

    # YabLoc expected map topics
    ll2_road_marking = "/localization/pose_estimator/yabloc/map/ll2_road_marking"
    ll2_bounding_box = "/localization/pose_estimator/yabloc/map/ll2_bounding_box"

    # Package shares
    img_share = get_package_share_directory("yabloc_image_processing")
    pf_share = get_package_share_directory("yabloc_particle_filter")
    common_share = get_package_share_directory("yabloc_common")
    map_loader_share = get_package_share_directory("autoware_map_loader")

    # Launch files
    img_launch = os.path.join(img_share, "launch", "yabloc_image_processing.launch.xml")
    pf_launch = os.path.join(pf_share, "launch", "yabloc_particle_filter.launch.xml")
    map_launch = os.path.join(map_loader_share, "launch", "lanelet2_map_loader.launch.xml")

    # Default param yaml paths
    default_undistort = os.path.join(img_share, "config", "undistort.param.yaml")
    default_graph_seg = os.path.join(img_share, "config", "graph_segment.param.yaml")
    default_seg_filter = os.path.join(img_share, "config", "segment_filter.param.yaml")

    default_predictor = os.path.join(pf_share, "config", "predictor.param.yaml")
    default_cam_corr = os.path.join(pf_share, "config", "camera_particle_corrector.param.yaml")
    default_gnss_corr = os.path.join(pf_share, "config", "gnss_particle_corrector.param.yaml")

    default_ll2_decomposer = os.path.join(common_share, "config", "ll2_decomposer.param.yaml")

    # LaunchConfiguration for param paths
    undistort_param_path = LaunchConfiguration("undistort_param_path")
    graph_segment_param_path = LaunchConfiguration("graph_segment_param_path")
    segment_filter_param_path = LaunchConfiguration("segment_filter_param_path")

    predictor_param_path = LaunchConfiguration("predictor_param_path")
    camera_particle_corrector_param_path = LaunchConfiguration("camera_particle_corrector_param_path")
    gnss_particle_corrector_param_path = LaunchConfiguration("gnss_particle_corrector_param_path")

    ll2_decomposer_param_path = LaunchConfiguration("ll2_decomposer_param_path")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # Map path (your file)
        DeclareLaunchArgument("lanelet2_map_path", default_value="/workspace/maps/map.osm"),

        # Your topics
        DeclareLaunchArgument("odom_topic", default_value="/vehicle/odometry"),
        DeclareLaunchArgument("src_image", default_value="/sensing/camera/image"),
        DeclareLaunchArgument("src_info", default_value="/sensing/camera/camera_info"),
        DeclareLaunchArgument("twist_cov_topic", default_value="/localization/twist_estimator/twist_with_covariance"),

        # Params: image_processing
        DeclareLaunchArgument("undistort_param_path", default_value=default_undistort),
        DeclareLaunchArgument("graph_segment_param_path", default_value=default_graph_seg),
        DeclareLaunchArgument("segment_filter_param_path", default_value=default_seg_filter),

        # Params: particle_filter
        DeclareLaunchArgument("predictor_param_path", default_value=default_predictor),
        DeclareLaunchArgument("camera_particle_corrector_param_path", default_value=default_cam_corr),
        DeclareLaunchArgument("gnss_particle_corrector_param_path", default_value=default_gnss_corr),

        # Params: ll2_decomposer
        DeclareLaunchArgument("ll2_decomposer_param_path", default_value=default_ll2_decomposer),

        # 0) Publish /map/map_projector_info permanently (Local)
        Node(
            package="localization",
            executable="map_projector_info_pub.py",
            name="map_projector_info_pub",
            output="screen",
        ),

        # 1) Lanelet2 map loader -> publish exactly /map/vector_map (requires projector info)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(map_launch),
            launch_arguments={
                "lanelet2_map_path": lanelet2_map_path,
                "lanelet2_map_topic": "/map/vector_map",
                "lanelet2_map_marker_topic": "/map/vector_map_marker",
            }.items(),
        ),

        # 2) Odometry -> TwistWithCovarianceStamped
        Node(
            package="localization",
            executable="odom_to_twist_cov.py",
            name="odom_to_twist_cov",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "odom_topic": odom_topic,
                "twist_topic": twist_cov_topic,
            }],
        ),

        # 3) LaneletMapBin (/map/vector_map) -> ll2 clouds for YabLoc
        Node(
            package="yabloc_common",
            executable="yabloc_ll2_decomposer_node",
            name="ll2_decomposer",
            output="screen",
            parameters=[ll2_decomposer_param_path, {"use_sim_time": use_sim_time}],
            remappings=[
                ("input/vector_map", "/map/vector_map"),
                ("output/ll2_road_marking", ll2_road_marking),
                ("output/ll2_bounding_box", ll2_bounding_box),
            ],
        ),

        # 3b) Relay: namespace prefix workaround for ll2_decomposer topic mismatch
        Node(
            package="topic_tools",
            executable="relay",
            name="vector_map_relay",
            output="screen",
            arguments=["/map/vector_map", "/ll2_decomposer/input/vector_map"],
        ),

        # 4) YabLoc image processing
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(img_launch),
            launch_arguments={
                "src_image": src_image,
                "src_info": src_info,
                "undistort_param_path": undistort_param_path,
                "graph_segment_param_path": graph_segment_param_path,
                "segment_filter_param_path": segment_filter_param_path,
            }.items(),
        ),

        # 5) YabLoc particle filter
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(pf_launch),
            launch_arguments={
                "twist_cov_for_prediction": twist_cov_topic,
                "input_projected_line_segments_cloud": projected_cloud,
                "input_ll2_road_marking": ll2_road_marking,
                "input_ll2_bounding_box": ll2_bounding_box,
                "predictor_param_path": predictor_param_path,
                "camera_particle_corrector_param_path": camera_particle_corrector_param_path,
                "gnss_particle_corrector_param_path": gnss_particle_corrector_param_path,
            }.items(),
        ),

        # 6) YabLoc'u başlatmak için initialpose3d pub (5 sn sonra, node'lar ayağa kalksın)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "topic", "pub", "--once",
                        "/initialpose3d",
                        "geometry_msgs/msg/PoseWithCovarianceStamped",
                        '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068]}}',
                    ],
                    output="screen",
                )
            ],
        ),
    ])
