# mobile_fr3_duo_hw

MoveIt Pro configuration for the real Franka Mobile FR3 Duo: two FR3 v2 arms with Franka Hands, the vertical spine and the TMR v0.2 swerve base.

It inherits from `mobile_fr3_duo_sim` (which inherits from `mobile_fr3_duo_mock`): robot description, SRDF, kinematics, joint limits and the Nav2 parameters stay the same.
It replaces the ros2_control hardware with franka_hardware: one `FrankaHardwareInterface` per arm and one for the base. It adds the drivers that are outside ros2_control, and has its own controllers, jog settings, Objectives and waypoints.
The drivers come from the `franka_ros2` and `libfranka` submodules in `src/external_dependencies`.
CI does not build or test this package, because it depends on libfranka (see `.github/workflows/ci.yaml`): run its tests locally.

## Before you run

- Connect the host to the robot network.
- In Franka Desk, unlock the joints and activate FCI on each arm.
- Both arms must be reachable when the stack starts: each arm's robot state broadcaster starts active, and if its arm is unreachable, its start times out and MoveIt Pro stops the whole stack. An unreachable base only leaves the base joints and controller unavailable.
- Set the addresses:
  - arms and base: `left_robot_ip`, `right_robot_ip` and `base_robot_ip` under `hardware.robot_description.urdf_params` in `config/config.yaml`. The Franka Hand drivers read the arm addresses from there too.
  - spine, lidars and cameras: `config/drivers.yaml`.
  - The defaults are the addresses in franka_ros2's own bringup and sensor files.
- A real-time (PREEMPT_RT) kernel on the host is recommended for the 1 kHz control loop. See the [libfranka](https://github.com/frankarobotics/libfranka) documentation.

The workspace `docker-compose.yaml` gives the drivers container the real-time thread priority and unlimited memory locking that the controller manager settings in `config/control/` use.

## Build and run

```bash
git submodule update --init --recursive
moveit_pro build
moveit_pro run -c mobile_fr3_duo_hw
```

## Hardware

| ros2_control component | Device | Joints | Commands |
| --- | --- | --- | --- |
| `left_FrankaHardwareInterface` | left FR3 | `left_fr3v2_joint1`..`7` | position, velocity, effort |
| `right_FrankaHardwareInterface` | right FR3 | `right_fr3v2_joint1`..`7` | position, velocity, effort |
| `TmrHardware` | TMR base | the four drive joints `tmrv0_2_joint_0`..`3` (state) | a Cartesian velocity of the base |

Each component runs asynchronously, on its own thread, as in franka_ros2's mobile FR3 Duo bringup.
The drivers outside ros2_control start from `launch/robot_drivers_to_persist.launch.py`:

- a `franka_gripper` node per hand, with actions under `/left/franka_gripper/` and `/right/franka_gripper/`;
- `franka_spine_server`, which bridges the spine's REST API to ROS;
- the two SICK nanoScan3 safety scanners (`sick_safetyscanners2`) and the four base RealSense D455 cameras (`realsense2_camera`), the sensors that `mobile_fr3_duo_sim` models;
- `script/base_odometry_bridge.py`, `script/spine_joint_state_publisher.py` and Nav2 (below).

Set `use_lidars`, `use_cameras` or `use_navigation` to `false` in that file to leave a group out. `moveit_pro run` passes no launch arguments, so change the default there.

## Joint states

MoveIt's robot state is complete only when every moving joint is on `/joint_states`. Each joint has exactly one publisher:

| Joints | Publisher |
| --- | --- |
| arm joints, base drive joints | `joint_state_broadcaster` (from the three hardware components) |
| `rocker_arm_joint` and the four caster joints | `joint_state_broadcaster` `extra_joints`, always zero: no device reports them |
| `planar_x`, `planar_y`, `planar_theta` | `script/base_odometry_bridge.py` |
| `franka_spine_vertical_joint` | `script/spine_joint_state_publisher.py` |
| finger joints | the two `franka_gripper` nodes |

The spine joint appears only after the spine server reads a valid height. Until then, MoveIt plans nothing: a guessed height would move both arms in the planning scene.

## Frames and odometry

`world` -> `map` (fixed) -> `odom` (slam_toolbox or AMCL) -> planar joints -> `base_link`, the same chain as `mobile_fr3_duo_mock`.

- `base_swerve_controller` publishes the base odometry on `/base_swerve_controller/odom`. It does not publish `odom` -> `base_link`: robot_state_publisher does, from the planar joints.
- `script/base_odometry_bridge.py` turns that odometry into the planar joints and into `/odom` for Nav2. The base pose of its first sample is the `odom` origin, so the joints start at zero and do not jump when the controller activates. Until then it publishes zeros.
- The controller computes its closed-loop twist along the odometry axes; the bridge turns it into `base_link` axes for Nav2 (parameter `twist_in_odom_axes`).
- The lidar scan frames and the camera frames hang from the base's mounting-point links, through static transforms in the drivers launch file.

## Navigation

`launch/navigation.launch.py` starts Nav2 with three layers of parameters: `mobile_fr3_duo_mock`'s, `mobile_fr3_duo_sim`'s (AMCL and the laser obstacle layers) and this package's `params/nav2_params.yaml`. The last layer makes Nav2's velocity commands stamped and sends the collision monitor's output straight to `/base_swerve_controller/cmd_vel`.

- With no map (the default), slam_toolbox builds a map of the site and publishes `map` -> `odom`. Save it with the slam_toolbox tools.
- With a map, set the `map` launch argument's default to its yaml: map_server and AMCL localize on it. AMCL starts at the map origin, so start the robot where the map was started.

The base controller is inactive at startup. Every navigation Objective first runs "Stow Arms for Navigation", which moves each arm to the `Stow` waypoint and then runs "Activate Mobile Base".

## Spine

The real spine has no ros2_control interface, so no controller moves it. `franka_spine_server` bridges its REST API: the `move_absolute` action, the `switch_on` and `fault_reset` services, and the `position` topic that the spine joint publisher reads.

- "Enable Spine" switches the spine on (`SwitchOnSpine`); if the spine refuses, it clears the fault (`ResetSpineFault`) and tries once more. Nothing switches the spine on at startup.
- "Move Spine Absolute" and "Move Spine to Pose Height" replace the mock's versions with the same ports. They check the height against the spine travel, then call `MoveSpineAbsolute`. The arms do not move.
- The spine Behaviors come from `franka_behaviors`. Planning groups that include the spine still cannot execute here: no trajectory controller drives the spine.

## Controllers

| Controller | Type | Joints or group | At startup |
| --- | --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all hardware joints | active |
| `left_franka_robot_state_broadcaster`, `right_franka_robot_state_broadcaster` | `franka_robot_state_broadcaster/FrankaRobotStateBroadcaster` | one arm | active |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController`, effort | both arms; a goal for one arm holds the other | inactive |
| `left_jtc`, `right_jtc` | `joint_trajectory_controller/JointTrajectoryController`, effort | one arm | inactive |
| `left_jtac`, `right_jtac` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController`, position | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_vfc`, `right_vfc` | `velocity_force_controller/VelocityForceController`, velocity | one arm, pose jog | inactive |
| `left_jvc`, `right_jvc` | `joint_velocity_controller/JointVelocityController`, velocity | one arm, joint jog | inactive |
| `base_swerve_controller` | `franka_mobile/SwerveDriveController` | the base | inactive |

Only the broadcasters are active at startup. An Objective activates the controller it needs.
Jogging works on the arms only: the base and the spine have no jog controller here.

## Objectives

| Objective | What it does |
| --- | --- |
| Open/Close Gripper, Open/Close Left Gripper, Open/Close Right Gripper | Opens or closes a Franka Hand (the right one by default). Close fails if the fingers stop on an object. |
| Grasp Object | Grasps an object at a set width and force with the right hand. |
| Move To Waypoint with Approval | Plans one arm to a saved waypoint and executes it, optionally after the user approves. Subtree. |
| MTC Cartesian Move to Pose, MTC Cartesian Move to Poses | Moves one arm's tip link along a Cartesian path. Subtrees. |
| Stow Arms for Navigation | Moves each arm to `Stow`, then activates the base. |
| Activate Mobile Base | Activates `base_swerve_controller`, so Nav2 can drive. |
| Move Base to Ready, Navigate to Clicked Point | Stow the arms, then drive with Nav2. |
| Teleoperate | MoveIt Pro's Teleoperate, executing on `joint_trajectory_controller`. |
| Recover Left Arm, Recover Right Arm, Recover Base | Clear a reflex or error on one device, then restore its hardware component and the broadcasters. After Recover Base, run Activate Mobile Base. |
| Enable Spine | Switches the spine on, clearing a fault first if needed. |
| Move Spine Absolute, Move Spine to Pose Height | Moves the spine to a height, or to the height of a target pose. Run Enable Spine first. |

## Known limitations

- A switch between controllers that command a device in different modes (for example activating the base, or a jog controller) changes the device's control mode with blocking calls to it. The pinned franka_hardware skips that device's reads and writes during the switch instead of waiting for it. The 1 kHz control loop still stalls once when the switch runs inside that loop, which is what MoveIt Pro's `SwitchController` asks for by default (`activate_asap: true`). Keep the arms still when the base activates.
- When a device reports a reflex or an error at activation (at startup, or in a Recover Objective), franka_hardware first tries one automatic error recovery. The device stays inactive only if that fails.
- MoveIt Pro's core motion Objectives default to `joint_trajectory_admittance_controller` and the `manipulator` group, which includes the spine. Neither works here: use this package's Objectives, or pass a spine-free group and `joint_trajectory_controller`.
- In teleoperation, dragging the base iMarker stows the arms and then fails: `mobile_fr3_duo_mock`'s "Request Teleoperation" switches to `base_jgvc`, which does not exist on the real robot. Drive the base with "Navigate to Clicked Point".
- The whole-body Objectives inherited from `mobile_fr3_duo_mock` ("Whole Body Move to Waypoint", "Whole Body Move to Pose", "Whole Body Reach Left" and "Whole Body Move to Joint State") do not run here: they switch to `body_jtc` and `base_pid`, which this package does not have.
- The spine moves only through its own Objectives (see Spine), not as part of a planned trajectory. The IMU that the sim models has no driver here.
- The configuration is built and checked without a robot. Nothing has run on a real Mobile FR3 Duo yet.

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `mobile_fr3_duo_sim`, device addresses, Behavior loaders, Objectives, controllers |
| `config/control/` | ros2_control controllers for the real robot |
| `config/moveit/` | arm-only jog settings |
| `config/drivers.yaml` | spine, lidar and camera addresses and settings |
| `description/` | ros2_control hardware: franka_hardware for both arms and the base |
| `launch/` | the drivers outside ros2_control, and Nav2 |
| `params/` | this package's Nav2 parameter layer |
| `script/`, `test/` | the base odometry bridge, the spine joint publisher, and their tests |
| `objectives/`, `waypoints/` | Objectives for the real robot; the mock's waypoints with `Ready` and `Stow` also for each arm |
