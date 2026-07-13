#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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


def _as_bool(value: str) -> bool:
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    this_share = get_package_share_directory('global_bringup')
    cfg_dir = os.path.join(this_share, 'config')
    packages_cfg = _load_yaml(os.path.join(cfg_dir, 'bringup.yaml'))
    manual_mode = _as_bool(LaunchConfiguration('manual').perform(context))
    actions = []

    # ========== PACKAGES ==========
    packages = packages_cfg.get('packages', {})
    if not packages:
        raise RuntimeError(
            'bringup.yaml contains no packages. Cannot build launch graph.'
        )

    if manual_mode:
        actions.append(LogInfo(
            msg='[bringup] manual mode enabled: launching xbox_controller and skipping planning/control'
        ))

    for pkg_name, pkg_cfg in packages.items():
        enabled = pkg_cfg.get('enabled', False)

        if manual_mode and pkg_name in ('planning', 'control'):
            actions.append(LogInfo(msg=f'[bringup] {pkg_name}: disabled in manual mode'))
            continue

        if manual_mode and pkg_name == 'xbox_controller':
            enabled = True

        if not enabled:
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

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_model',
            default_value='my_vehicle',
            description='Default vehicle model name',
        ),
        DeclareLaunchArgument(
            'manual',
            default_value='false',
            description='Enable Xbox manual control and disable autonomous planning/control command publishers',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
