#!/usr/bin/env python3
import os
import sys
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def _load_yaml(path: str) -> dict:
    """Load YAML file; raise if missing or malformed."""
    if not os.path.exists(path):
        raise FileNotFoundError(f'Required configuration not found: {path}')
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    if data is None:
        raise ValueError(f'Configuration file is empty: {path}')
    return data


def generate_launch_description():
    # Package paths
    this_share = get_package_share_directory('global_bringup')
    cfg_dir = os.path.join(this_share, 'config')

    # Load configuration (fail-fast on missing/invalid YAML)
    packages_cfg = _load_yaml(os.path.join(cfg_dir, 'bringup.yaml'))

    actions = []

    actions.append(DeclareLaunchArgument(
        'vehicle_model',
        default_value='my_vehicle',
        description='Default vehicle model name',
    ))

    # ========== PACKAGES ==========
    packages = packages_cfg.get('packages', {})
    if not packages:
        raise RuntimeError(
            'bringup.yaml contains no packages. Cannot build launch graph.'
        )

    for pkg_name, pkg_cfg in packages.items():
        if not pkg_cfg.get('enabled', False):
            actions.append(LogInfo(msg=f'[bringup] {pkg_name}: disabled, skipping'))
            continue

        launch_file = pkg_cfg.get('launch_file')
        if not launch_file:
            raise RuntimeError(
                f"Package '{pkg_name}' is enabled but has no 'launch_file' specified in bringup.yaml."
            )

        # Fail-fast: package must be discoverable
        try:
            pkg_share = get_package_share_directory(pkg_name)
        except PackageNotFoundError:
            raise RuntimeError(
                f"Package '{pkg_name}' is enabled in bringup.yaml but not found in the workspace. "
                f"Build it with 'colcon build --packages-select {pkg_name}' or disable it in bringup.yaml."
            )

        # Fail-fast: launch file must exist
        launch_path = os.path.join(pkg_share, 'launch', launch_file)
        if not os.path.exists(launch_path):
            raise RuntimeError(
                f"Launch file not found: {launch_path}\n"
                f"Package '{pkg_name}' is built but its launch file '{launch_file}' is not installed. "
                f"Check that the package's CMakeLists.txt installs the launch/ directory."
            )

        pkg_args = pkg_cfg.get('arguments', {})
        actions.append(LogInfo(msg=f'[bringup] {pkg_name}: launching {launch_file}'))
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_path),
            launch_arguments=pkg_args.items(),
        ))

    return LaunchDescription(actions)
