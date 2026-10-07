import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Turns the two MuJoCo 3D-lidar clouds into /scan_front and /scan_rear.
    # SICK nanoScan3 values measured from the real scanner's recorded scans:
    # 275 deg sweep, 0.10-40 m. The 6 deg trims drop the beams that graze the
    # chassis window; noise and rounding match the real sensor.
    lidar_flattener = Node(
        package="mobile_fr3_duo_sim",
        executable="lidar_flattener.py",
        name="lidar_flattener",
        parameters=[
            {
                "angle_min": 0.0,
                "angle_max": math.radians(275.0),
                "trim_low_deg": 6.0,
                "trim_high_deg": 6.0,
                "range_min": 0.10,
                "range_max": 40.0,
                "range_noise_stddev": 0.003,
                "range_quantum": 0.001,
            }
        ],
        output="log",
    )

    # Nav2 drives the base through this bridge: cmd_vel (base frame) becomes
    # world-frame velocities on the planar joints, for base_jgvc.
    base_twist_to_planar = Node(
        package="mobile_fr3_duo_sim",
        executable="base_twist_to_planar.py",
        name="base_twist_to_planar",
        output="log",
    )

    # AMCL and slam_toolbox each take one scan topic, so the two 275 deg scans
    # are merged into a 360 deg /scan in base_link.
    scan_merger = Node(
        package="mobile_fr3_duo_sim",
        executable="scan_merger.py",
        name="scan_merger",
        output="log",
    )

    # The plugin's twist is in the odom frame; Nav2 needs it in base_link.
    odom_twist_to_base = Node(
        package="mobile_fr3_duo_sim",
        executable="odom_twist_to_base.py",
        name="odom_twist_to_base",
        output="log",
    )

    # Scan frames: z up, X at beam 0 (the clockwise end of each 275 deg sweep),
    # at the URDF lidar mounting points. Static on base_link, as in hangar_sim.
    static_tf_lidar_front_ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_front_ros",
        arguments=[
            "--x",
            "0.3275",
            "--y",
            "0.2175",
            "--z",
            "0.19065",
            "--yaw",
            str(math.radians(-92.5)),
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_front_ROS",
        ],
        output="log",
    )
    static_tf_lidar_rear_ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_rear_ros",
        arguments=[
            "--x",
            "-0.3275",
            "--y",
            "-0.2175",
            "--z",
            "0.19065",
            "--yaw",
            str(math.radians(87.5)),
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_rear_ROS",
        ],
        output="log",
    )

    # Nav2 frames: mj_world -> map and odom -> world are identity; map -> odom
    # comes from AMCL. The robot's pose below world comes from
    # robot_state_publisher (planar joints), so odom -> base_link is exact.
    static_tf_mj_world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_mj_world_to_map",
        arguments=["--frame-id", "mj_world", "--child-frame-id", "map"],
        output="log",
    )
    # Nav2, on the map built with slam_toolbox. AMCL owns map -> odom, and the
    # chain ends at /cmd_vel, which base_twist_to_planar turns into planar joint
    # velocities. use_sim_time is false because the plugin stamps wall time.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
            )
        ),
        launch_arguments={
            "map": PathJoinSubstitution(
                [FindPackageShare("mobile_fr3_duo_sim"), "maps", "room.yaml"]
            ),
            "params_file": PathJoinSubstitution(
                [FindPackageShare("mobile_fr3_duo_sim"), "params", "nav2_params.yaml"]
            ),
            "use_sim_time": "false",
            "autostart": "true",
            # Capitalised: bringup_launch evaluates this as a Python expression.
            "slam": "False",
        }.items(),
    )

    static_tf_odom_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_to_world",
        arguments=["--frame-id", "odom", "--child-frame-id", "world"],
        output="log",
    )

    return LaunchDescription(
        [
            lidar_flattener,
            scan_merger,
            base_twist_to_planar,
            odom_twist_to_base,
            static_tf_lidar_front_ros,
            static_tf_lidar_rear_ros,
            static_tf_mj_world_to_map,
            nav2,
            static_tf_odom_to_world,
        ]
    )
