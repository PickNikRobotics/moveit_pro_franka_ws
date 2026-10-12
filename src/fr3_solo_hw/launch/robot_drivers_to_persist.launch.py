from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_studio_utils_py.system_config import SystemConfigParser


def robot_ip_from_config():
    """Return the robot_ip xacro argument from config.yaml, so the arm and the hand use one address."""
    hardware = SystemConfigParser().get_hardware_config()
    for param in hardware.robot_description.urdf_params:
        if "robot_ip" in param:
            return str(param["robot_ip"])
    raise RuntimeError(
        "fr3_solo_hw: hardware.robot_description.urdf_params in config.yaml has no robot_ip."
    )


def generate_launch_description():
    # The Franka Hand. ros2_control drives the arm, so this node is the only extra driver.
    # Its finger joint states go to /joint_states, so the planning scene sees the real fingers.
    gripper = Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        name="franka_gripper",
        parameters=[
            {
                "robot_ip": robot_ip_from_config(),
                "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
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
    return LaunchDescription([gripper])
