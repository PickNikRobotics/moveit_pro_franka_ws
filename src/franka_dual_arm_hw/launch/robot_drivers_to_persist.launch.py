#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

############################################################################
# Dual-arm FR3 gripper drivers (persisted for the Agent's lifetime).
#
# The single un-namespaced controller_manager (launched by the MoveIt Pro Agent)
# drives both ARMS via franka_hardware/FrankaHardwareInterface. The FR3 GRIPPERS,
# however, are NOT ros2_control controllers -- they are driven by the libfranka
# `franka_gripper_node` action server. This launch file brings up one gripper node
# per arm, namespaced "left"/"right", so their action servers live at
#   /left/franka_gripper/...   and   /right/franka_gripper/...
# (open_gripper.xml / close_gripper.xml target /right/franka_gripper/gripper_action).
#
# MoveIt Pro includes this file (config.yaml hardware.robot_driver_persist_launch_file)
# with NO launch arguments when hardware.simulated is False, so the per-arm robot IPs
# are declared here as defaults. KEEP THESE IN SYNC with config.yaml
# hardware.robot_description.urdf_params (left_robot_ip / right_robot_ip).
############################################################################

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def gripper_launch(namespace, robot_ip):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_gripper"),
                    "launch",
                    "gripper.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "namespace": namespace,
            "robot_type": "fr3",
            # Real hardware: connect to the physical gripper over the FCI.
            # (This persist launch is only loaded when hardware.simulated is False.)
            "use_fake_hardware": "false",
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            # Defaults mirror config.yaml urdf_params; keep them in sync.
            DeclareLaunchArgument("left_robot_ip", default_value="192.168.1.21"),
            DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.22"),
            gripper_launch("left", LaunchConfiguration("left_robot_ip")),
            gripper_launch("right", LaunchConfiguration("right_robot_ip")),
        ]
    )
