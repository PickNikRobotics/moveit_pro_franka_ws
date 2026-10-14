# Starts the franka_gripper, SICK and RealSense nodes as franka_ros2 3.6.0's launch files do (Copyright 2023
# Franka Robotics GmbH, Apache License, Version 2.0; LICENSES/franka_ros2-LICENSE). Modified by PickNik Inc., 2026.

import functools
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_studio_utils_py.system_config import SystemConfigParser

PACKAGE = "mobile_fr3_duo_hw"


@functools.cache
def urdf_params():
    """The xacro arguments in config.yaml, read once, so ros2_control and these drivers share them."""
    params = {}
    for param in (
        SystemConfigParser().get_hardware_config().robot_description.urdf_params
    ):
        params.update(param)
    return params


def urdf_param(name):
    if name not in urdf_params():
        raise RuntimeError(
            f"{PACKAGE}: hardware.robot_description.urdf_params in config.yaml has no {name}."
        )
    return str(urdf_params()[name])


def drivers_config():
    with open(
        os.path.join(get_package_share_directory(PACKAGE), "config", "drivers.yaml")
    ) as f:
        return yaml.safe_load(f)


def static_tf(parent, child, yaw=0.0):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent}_to_{child}",
        arguments=[
            "--yaw",
            str(yaw),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        output="log",
    )


def gripper(side):
    # Started directly, not through franka_gripper's launch file, for the joint names and the remap.
    # Its actions are under /<side>/franka_gripper/.
    return Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        name="franka_gripper",
        namespace=side,
        parameters=[
            {
                "robot_ip": urdf_param(f"{side}_robot_ip"),
                "joint_names": [
                    f"{side}_fr3v2_finger_joint1",
                    f"{side}_fr3v2_finger_joint2",
                ],
            },
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_gripper"),
                    "config",
                    "franka_gripper_node.yaml",
                ]
            ),
        ],
        remappings=[("~/joint_states", "/joint_states")],
        output="log",
    )


def lidars(config):
    """The two safety scanners, their scan frames, and one merged /scan for AMCL."""
    actions = []
    for scanner in config["scanners"]:
        name = scanner["name"]
        actions.append(
            Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name=f"{name}_driver",
                namespace=name,
                parameters=[
                    {
                        "frame_id": name,
                        "sensor_ip": scanner["sensor_ip"],
                        "host_ip": config["host_ip"],
                        "interface_ip": "0.0.0.0",
                        "host_udp_port": 0,
                        **config["parameters"],
                    }
                ],
                output="log",
            )
        )
        # The scan frame sits on the mounting point, turned as in franka_mobile_sensors' nanoScan3 macro.
        actions.append(
            static_tf(f"{name}_mounting_point", name, yaw=-math.radians(90.0))
        )
    actions.append(
        Node(
            package="mobile_fr3_duo_sim",
            executable="scan_merger.py",
            name="scan_merger",
            parameters=[
                {
                    "front_topic": "/lidar_front/scan",
                    "rear_topic": "/lidar_rear/scan",
                    "output_topic": "/scan",
                }
            ],
            output="log",
        )
    )
    return actions


def cameras(config):
    """The base RealSense cameras, each attached at its mounting point."""
    actions = []
    for device in config["devices"]:
        name = device["name"]
        selector = {
            k: v for k, v in device.items() if k in ("serial_no", "usb_port_id")
        }
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name=name,
                namespace=name,
                parameters=[{"camera_name": name, **selector, **config["parameters"]}],
                output="log",
            )
        )
        # camera_front -> front_mounting_point, and so on.
        mount = name.replace("camera_", "") + "_mounting_point"
        actions.append(static_tf(mount, f"{name}_link"))
    return actions


def generate_launch_description():
    config = drivers_config()
    use_lidars = LaunchConfiguration("use_lidars")
    use_cameras = LaunchConfiguration("use_cameras")
    use_navigation = LaunchConfiguration("use_navigation")

    # The spine has no ros2_control interface: franka_spine_server bridges its REST API.
    spine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_spine_server"), "launch", "spine.launch.py"]
            )
        ),
        launch_arguments={
            "spine_ip": str(config["spine"]["spine_ip"]),
            "namespace": "",
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(PACKAGE), "launch", "navigation.launch.py"]
            )
        ),
        condition=IfCondition(use_navigation),
    )

    # moveit_pro run passes no launch arguments: change these defaults to turn a group off.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_lidars",
                default_value="true",
                description="Start the two safety scanners.",
            ),
            DeclareLaunchArgument(
                "use_cameras",
                default_value="true",
                description="Start the base cameras.",
            ),
            DeclareLaunchArgument(
                "use_navigation",
                default_value="true",
                description="Start Nav2; it needs the lidars for localization.",
            ),
            gripper("left"),
            gripper("right"),
            spine,
            Node(
                package=PACKAGE,
                executable="spine_joint_state_publisher.py",
                name="spine_joint_state_publisher",
                output="log",
            ),
            Node(
                package=PACKAGE,
                executable="base_odometry_bridge.py",
                name="base_odometry_bridge",
                output="log",
            ),
            GroupAction(lidars(config["lidars"]), condition=IfCondition(use_lidars)),
            GroupAction(cameras(config["cameras"]), condition=IfCondition(use_cameras)),
            navigation,
        ]
    )
