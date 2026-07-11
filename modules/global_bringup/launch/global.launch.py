#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _load_yaml(path: str) -> dict:
    """Load a required bringup configuration."""
    if not os.path.exists(path):
        raise FileNotFoundError(f"Required bringup config does not exist: {path}")
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def generate_launch_description():
    # Package paths
    this_share = get_package_share_directory("global_bringup")
    cfg_dir = os.path.join(this_share, "config")
    packages_cfg = _load_yaml(os.path.join(cfg_dir, "bringup.yaml"))

    actions = []
    actions.append(
        DeclareLaunchArgument(
            "vehicle_model",
            default_value="rdw_vehicle",
            description="Default vehicle model name",
        )
    )
    actions.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock for included launch files",
        )
    )
    actions.append(
        DeclareLaunchArgument(
            "launch_sensor_drivers",
            default_value="true",
            description="Launch physical camera and LiDAR drivers",
        )
    )
    actions.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="SocketCAN interface or python-can channel",
        )
    )
    actions.append(
        DeclareLaunchArgument(
            "can_channel_type",
            default_value="socketcan",
            description="python-can transport; virtual is for software tests only",
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_sensor_drivers = LaunchConfiguration("launch_sensor_drivers")
    can_interface = LaunchConfiguration("can_interface")
    can_channel_type = LaunchConfiguration("can_channel_type")

    # ========== PACKAGES ==========
    for pkg_name, pkg_cfg in packages_cfg.get("packages", {}).items():
        if not pkg_cfg.get("enabled", False):
            continue
        launch_file = pkg_cfg.get("launch_file")
        if not launch_file:
            continue
        pkg_share = get_package_share_directory(pkg_name)
        launch_path = os.path.join(pkg_share, "launch", launch_file)
        if not os.path.exists(launch_path):
            raise FileNotFoundError(
                f"Enabled package '{pkg_name}' has no launch file: {launch_path}"
            )

        pkg_args = dict(pkg_cfg.get("arguments", {}))
        pkg_args.setdefault("use_sim_time", use_sim_time)
        if pkg_name == "rdw_sensor_kit_launch":
            pkg_args.setdefault("launch_driver", launch_sensor_drivers)
        if pkg_name == "ros2_can_bridge":
            pkg_args.setdefault("interface", can_interface)
            pkg_args.setdefault("channel_type", can_channel_type)
        actions.append(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(launch_path),
                        launch_arguments=pkg_args.items(),
                    )
                ],
                scoped=True,
            )
        )
    
    return LaunchDescription(actions)
