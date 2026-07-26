# Copyright 2022 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_studio_utils_py.launch_common import empty_gen
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    hw_config = system_config_parser.get_hardware_config()

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_gripper"),
                                "launch",
                                "gripper.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_ip": "192.168.19.21",
                    # franka_ros2 v3.3.0 gripper.launch.py requires 'namespace' (no default) and
                    # derives joint_names from robot_type internally (fr3 -> fr3_finger_joint1/2).
                    # Use "fr3_gripper" so actions are /fr3_gripper/... (matches the gripper objectives).
                    "namespace": "fr3_gripper",
                }.items(),
            )
        ]
    )
