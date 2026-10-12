# fr3_solo_hw

MoveIt Pro configuration for one real Franka Research 3 (FR3) arm with a Franka Hand.

It inherits from `fr3_solo_sim` (which inherits from `fr3_solo_mock`): robot description, SRDF, kinematics, jogging and 3D sensors stay the same.
It replaces the ros2_control hardware with franka_hardware's `FrankaHardwareInterface`, adds the franka_gripper driver for the Franka Hand, and has its own controllers, joint limits, Objectives and waypoints.
The drivers come from the `franka_ros2` and `libfranka` submodules in `src/external_dependencies`.

## Before you run

- Connect the host to the FR3 control box and give the host a static address on the same network.
- In Franka Desk, unlock the joints and activate FCI.
- The arm must be reachable when the stack starts: the robot state broadcaster starts active, and if the arm is unreachable, its start times out and MoveIt Pro stops the stack.
- Set the control box address in `config/config.yaml`, `robot_ip` under `hardware.robot_description.urdf_params`. The arm and the Franka Hand driver both read it from there. The default is the address in franka_ros2's own single-arm bringup.
- A real-time (PREEMPT_RT) kernel on the host is recommended for the 1 kHz control loop. See the [libfranka](https://github.com/frankarobotics/libfranka) documentation.

The workspace `docker-compose.yaml` gives the drivers container the real-time thread priority and unlimited memory locking that the controller manager settings in `config/control/fr3_solo_hw.ros2_control.yaml` use.

## Build and run

```bash
git submodule update --init --recursive
moveit_pro build
moveit_pro run -c fr3_solo_hw
```

The first build compiles libfranka and the franka_ros2 driver packages.

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `fr3_solo_sim`, `robot_ip`, Behavior loaders, Objectives, controllers |
| `config/control/` | ros2_control controllers for the real arm |
| `config/moveit/joint_limits.yaml` | joint limits with lower accelerations for the real arm |
| `description/` | ros2_control hardware: franka_hardware `FrankaHardwareInterface` |
| `launch/robot_drivers_to_persist.launch.py` | the franka_gripper node for the Franka Hand |
| `launch/runtime.launch.xml` | the MoveIt Pro runtime services |
| `launch/moveit_pro.launch.xml` | the whole stack in one launch, for a host without the MoveIt Pro containers |
| `objectives/`, `waypoints/` | Objectives for the real robot and saved waypoints |

## Controllers

| Controller | Type | Command interface | At startup |
| --- | --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | none | active |
| `franka_robot_state_broadcaster` | `franka_robot_state_broadcaster/FrankaRobotStateBroadcaster` | none | active |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | effort | inactive |
| `joint_trajectory_admittance_controller` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | position | inactive |
| `velocity_force_controller` | `velocity_force_controller/VelocityForceController` | velocity | inactive |
| `joint_velocity_controller` | `joint_velocity_controller/JointVelocityController` | velocity | inactive |

Only the broadcasters are active at startup. An Objective activates the controller it needs.
The Objectives in this package and Teleoperate execute on `joint_trajectory_controller`. Pose jog uses `velocity_force_controller` and joint jog uses `joint_velocity_controller`.
The Franka Hand has no controller: the franka_gripper node drives it, with its actions under `/franka_gripper/`.

## Objectives

| Objective | What it does |
| --- | --- |
| Open Gripper, Close Gripper | Opens or closes the Franka Hand. Close Gripper fails if the fingers stop on an object. |
| Grasp Object | Grasps an object at a set width and force. |
| Move To Waypoint with Approval | Plans to a saved waypoint and executes it, optionally after the user approves the trajectory. Subtree: call it with a waypoint name. |
| MTC Cartesian Move to Pose, MTC Cartesian Move to Poses | Moves the tip link along a Cartesian path to one pose or through a list of poses. Subtrees: call them from an Objective that gives the pose. |
| Teleoperate | MoveIt Pro's Teleoperate, executing on `joint_trajectory_controller`. |
| Recover Arm | Clears a reflex or error on the arm, then restores its hardware component and broadcasters. |

After a reflex (for example a collision), run Recover Arm before the next motion.

## Known limitations

- Each switch between controllers that command the arm in different modes (for example from `joint_trajectory_controller` to `joint_velocity_controller` for jogging) stalls the 1 kHz control loop once, while the arm changes its control mode.
- MoveIt Pro's core motion Objectives default to `joint_trajectory_admittance_controller`, which sends position commands. Prefer this package's Objectives, which use `joint_trajectory_controller`.
- This configuration has no camera, the same as `fr3_solo_sim`.
- The configuration is built and checked without an arm connected. Motion on a real FR3 is not yet tested.
