#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _load_yaml(path: str) -> dict:
    """Load YAML file safely."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def _get_nested(cfg: dict, key: str, default):
    """Get nested dictionary value using dot notation."""
    keys = key.split(".")
    cur = cfg
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur if type(cur) == type(default) else default


def generate_launch_description():
    # Package paths
    this_share = get_package_share_directory("global_bringup")
    cfg_dir = os.path.join(this_share, "config")
    includes_dir = os.path.join(this_share, "launch", "includes")
    
    # Load all configuration files
    autoware_cfg = _load_yaml(os.path.join(cfg_dir, "autoware_args.yaml"))
    packages_cfg = _load_yaml(os.path.join(cfg_dir, "bringup.yaml"))
    
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    actions = []
    # vehicle_model argümanını default olarak ekle
    actions.append(DeclareLaunchArgument(
        "vehicle_model",
        default_value="rdw_vehicle",
        description="Default vehicle model name"
    ))
    actions.append(DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock for all included launch files that support it"
    ))
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # ========== PACKAGES ==========
    for pkg_name, pkg_cfg in packages_cfg.get("packages", {}).items():
        if not pkg_cfg.get("enabled", False):
            continue
        launch_file = pkg_cfg.get("launch_file")
        if not launch_file:
            continue
        try:
            pkg_share = get_package_share_directory(pkg_name)
            launch_path = os.path.join(pkg_share, "launch", launch_file)
            if os.path.exists(launch_path):
                pkg_args = dict(pkg_cfg.get("arguments", {}))
                pkg_args.setdefault("use_sim_time", use_sim_time)
                actions.append(IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(launch_path),
                    launch_arguments=pkg_args.items(),
                ))
        except Exception as e:
            print(f"[WARN] Could not launch {pkg_name}: {e}")
    
    return LaunchDescription(actions)
