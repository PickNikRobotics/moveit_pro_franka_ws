import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Nav2 parameters, in layering order: the mock's base file, the sim's AMCL and laser layers,
# and this package's real-hardware layer.
PARAM_LAYERS = (
    ("mobile_fr3_duo_mock", "params/nav2_params.yaml"),
    ("mobile_fr3_duo_sim", "params/nav2_params.yaml"),
    ("mobile_fr3_duo_hw", "params/nav2_params.yaml"),
)


def merge_params(base, overlay):
    """Merge a Nav2 parameter overlay into a base: mappings merge, every other value replaces."""
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_params(merged[key], value)
        else:
            merged[key] = value
    return merged


def nav2(context):
    params = {}
    for package, path in PARAM_LAYERS:
        with open(os.path.join(get_package_share_directory(package), path)) as f:
            params = merge_params(params, yaml.safe_load(f))
    params_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="mobile_fr3_duo_hw_nav2_", suffix=".yaml", delete=False
    )
    with params_file:
        yaml.safe_dump(params, params_file)

    map_yaml = LaunchConfiguration("map").perform(context)
    if map_yaml:
        # Localization on a saved map: map_server and AMCL, which owns map -> odom.
        localization = [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[params_file.name, {"yaml_filename": map_yaml}],
                output="log",
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[params_file.name],
                output="log",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}],
                output="log",
            ),
        ]
    else:
        # No saved map: slam_toolbox builds one and owns map -> odom.
        localization = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("slam_toolbox"),
                            "launch",
                            "online_async_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "slam_params_file": os.path.join(
                        get_package_share_directory("mobile_fr3_duo_sim"),
                        "params",
                        "slam_toolbox.yaml",
                    ),
                    "use_sim_time": "false",
                }.items(),
            )
        ]

    return localization + [
        # Nav2's planner, controller, behavior trees and the rest. The params send its final
        # velocity command to the base controller.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ),
            launch_arguments={
                "params_file": params_file.name,
                "use_sim_time": "false",
                "autostart": "true",
            }.items(),
        ),
    ]


def generate_launch_description():
    # Frames: world -> map (fixed) -> odom (AMCL or slam_toolbox) -> planar joints -> base_link.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Map yaml to localize on with AMCL. Empty: build a map with slam_toolbox.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_world_to_map",
                arguments=["--frame-id", "world", "--child-frame-id", "map"],
                output="log",
            ),
            OpaqueFunction(function=nav2),
        ]
    )
