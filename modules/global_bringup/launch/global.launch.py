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
    
    actions = []
    
    # ========== AUTOWARE ==========
    # Autoware'i başlat (eğer enabled ise)
    if _get_nested(autoware_cfg, "autoware.enabled", True):
        autoware_args = autoware_cfg.get("autoware", {})
        launch_args = {
            "map_path": autoware_args.get("map_path", ""),
            "vehicle_model": autoware_args.get("vehicle_model", ""),
            "sensor_model": autoware_args.get("sensor_model", ""),
            "pose_source": autoware_args.get("pose_source", "yabloc"),
            "use_sim_time": str(autoware_args.get("use_sim_time", False)).lower(),
        }
        
        # Try custom wrapper first
        autoware_include = os.path.join(includes_dir, "autoware.launch.py")
        if os.path.exists(autoware_include):
            actions.append(IncludeLaunchDescription(
                AnyLaunchDescriptionSource(autoware_include),
                launch_arguments=launch_args.items(),
            ))
        else:
            # Fallback to autoware_launch package
            try:
                aw_share = get_package_share_directory("autoware_launch")
                aw_launch = os.path.join(aw_share, "launch", "autoware.launch.xml")
                if os.path.exists(aw_launch):
                    actions.append(IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(aw_launch),
                        launch_arguments=launch_args.items(),
                    ))
            except Exception:
                pass
    
    # ========== PACKAGES ==========
    # TODO: Yeni paket eklemek için:
    # 1. bringup.yaml dosyasına paket konfigürasyonu ekleyin
    # 2. launch/includes/ altında {paket_adı}.launch.py oluşturun
    # 3. Aşağıdaki kod bloğunu kopyalayıp paket adını değiştirin:
    #
    # if packages_cfg.get("packages", {}).get("my_package", {}).get("enabled", False):
    #     pkg_cfg = packages_cfg["packages"]["my_package"]
    #     pkg_launch = os.path.join(includes_dir, pkg_cfg.get("launch_file", "my_package.launch.py"))
    #     if os.path.exists(pkg_launch):
    #         pkg_args = pkg_cfg.get("arguments", {})
    #         actions.append(IncludeLaunchDescription(
    #             AnyLaunchDescriptionSource(pkg_launch),
    #             launch_arguments=pkg_args.items(),
    #         ))
    
    return LaunchDescription(actions)