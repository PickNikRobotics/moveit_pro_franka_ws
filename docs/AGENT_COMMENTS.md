# AGENT_COMMENTS.md

The detail behind short code comments: why a setting is what it is, and where a value comes from.
Organized by file, then by line, so an entry is findable from a code location.
Keep entries short. When you change the code an entry describes, change the entry in the same commit.

## .gitmodules

### the franka_ros2 pin
PickNik's fork of franka_ros2, branch `fix/v3.6.0-activation-error-recovery`: upstream v3.6.0 plus two franka_hardware commits.
- `read()` and `write()` try the control lock and skip the cycle while a mode switch holds it, instead of waiting for it.
- `on_activate()` tries one automatic error recovery before it refuses to activate on a reflex or error.
Upstream v3.6.0 has neither. Only `franka_hardware_interface.cpp` and its test differ from v3.6.0, so the "v3.6.0" citations in this file hold for the pin.

## colcon-defaults.yaml

### L16 `packages-ignore`
fr3_solo_hw needs five franka_ros2 packages: franka_msgs, franka_hardware, franka_gripper, franka_robot_state_broadcaster, franka_semantic_components. mobile_fr3_duo_hw also needs franka_mobile (the base controller), franka_spine_msgs, franka_spine_server and franka_desk_api, which both of those use.
The ignore list is every other package in the franka_ros2 submodule. Ignored, not skipped: colcon then treats them as absent, so a dependency on `realtime_tools` resolves to the system package without a warning. Several need Gazebo, robot-specific hardware or packages with no Jazzy release.
`realtime_tools` in franka_ros2 is a vendored copy of the ROS package. Building it would shadow the system realtime_tools for the whole workspace; the system version has what franka_hardware uses.
When the franka_ros2 submodule moves to a new release, check for new packages and add the unused ones here.

## colcon.meta

### L1-L2 libfranka arguments
colcon reads `./colcon.meta` from the directory where it runs (the workspace root for `moveit_pro build` and for CI). A per-package `cmake-args` entry is appended to that package's arguments only.
This file is used instead of custom colcon mixins because CI copies only `colcon-defaults.yaml`, so a mixin defined in this workspace would not exist there.
libfranka's tests also need GMock in a form the MoveIt Pro image may not provide.

## Dockerfile

### L65 the skipped rosdep keys
Only packages in the colcon ignore list depend on these keys, and none of the keys resolves on Jazzy. Without the skip, `rosdep install` (no `-r`) fails the image build.
CI runs rosdep with `-r`, so it does not need the list. Add a key only when an image build shows it is needed.

### L87 `CCACHE_DEPEND`
In ccache's default mode, ccache runs a separate preprocessor pass. A GCC 13 crash in that mode on libfranka's model sources has been reported. Depend mode compiles the source directly and keeps the cache.

## docker-compose.yaml

### L17-L18 `ulimits` of the drivers service
The controller manager in `fr3_solo_hw` sets a real-time thread priority and locks its memory. Both need these limits: the process runs as a non-root user, so without `memlock` it cannot lock its memory and only logs a warning.
Compose merges this file with the MoveIt Pro compose file. The `rtprio` value repeats the MoveIt Pro value so both limits are in the merged service.

## src/fr3_solo_hw/config/config.yaml

### L8 based on fr3_solo_sim, not fr3_solo_mock
The workspace convention is mock, then sim, then hw. fr3_solo_sim changes only `urdf_params`: the `ros2_control_xacro` file, which fr3_solo_hw replaces, and the MuJoCo arguments `mujoco_model` and `mujoco_viewer`.
Those two arguments stay in the merged `urdf_params`, but no xacro file in the fr3_solo_hw description declares them, and xacro ignores an argument that nothing declares. Checked by expanding the description with the real hardware file.
The MuJoCo Behavior loader and the MuJoCo Objectives come from fr3_solo_mock, not from the sim, so basing on the mock would not remove them.

### L45-L52 the two Objective library keys
`objective_library_paths` is a map, and a child config can change an inherited entry but not remove it (`merge()` in `moveit_studio_utils_py/system_config.py`).
The mock's `mujoco_objectives` entry holds "Reset MuJoCo Sim", which needs `MujocoBehaviorsLoader`. This config removes that loader, so it points that key at its own folder; otherwise the Objective set fails to load.
The merged map keeps an inherited key at its old position, and only a new key goes to the end. MoveIt Pro copies the libraries from last to first and skips a file name it already has, so `fr3_solo_hw_objectives` must be a new key to win over the mock's `open_gripper.xml` and `close_gripper.xml` and core `teleoperate.xml`. Seen on MP10: with only the re-pointed `mujoco_objectives` key, "Open Gripper" ran the mock's file. The last library is also where the UI saves new Objectives.

### L61-L62 broadcasters only at startup
franka_hardware changes the arm's control mode when a controller that claims command interfaces activates. That switch stops the robot and starts the new mode with blocking calls to the control box. With `activate_asap: true`, the default of MoveIt Pro's `SwitchController`, ros2_control runs it inside the controller manager's update loop, so the loop misses cycles. The pinned franka_hardware's reads and writes skip cycles during a switch instead of waiting for it; that keeps the loop running only for a switch requested with `activate_asap: false`, which ros2_control runs outside the loop (`manage_switch()`, controller_manager 4.45).
During bring-up that can trip the communication reflex. With only state broadcasters at startup, bring-up switches no mode; each Objective activates the controller it needs.

## src/fr3_solo_hw/config/control/fr3_solo_hw.ros2_control.yaml

### L8 `thread_priority`
The real-time priority of the controller manager's thread. The hardware component is synchronous (see the description file), so this thread also reads and writes the robot. The drivers container allows this priority (docker-compose.yaml).

### L9-L12 `overruns`
ros2_control's default skips the cycles a long loop missed and re-phases. `manage: false` lets the loop catch up instead, as franka_ros2's own controller files set it. Warnings stay on: they are the only report of overruns.

### L13-L14 `lock_memory`
`ros2_control_node` locks the process memory only when this is true or the kernel is real-time. A failure to lock is only a warning, so check the drivers log for "Unable to lock the memory".

### L15-L17 `shutdown_on_initial_state_failure`
While the robot reports a reflex or an error, the pinned franka_hardware first tries one automatic error recovery at activation, then refuses to activate if the fault remains. The default turns that refusal into the end of the controller manager. False keeps the node up with the arm inactive, so "Recover Arm" can clear the fault.

### L35 `update_rate` of the joint state broadcaster
At the full loop rate the broadcaster floods `/joint_states`. The finger joints are not in this list: the franka_gripper node publishes them to `/joint_states`.

### L48 `allow_nonzero_velocity_at_trajectory_end`
Trajectories sampled from a plan can end on a tiny residual velocity. Without this, the trajectory controller rejects such a goal and logs the reason only in its own log.

### L63 the effort gains
Copied from franka_ros2 `franka_fr3_moveit_config/config/fr3_ros_controllers.yaml` (release v3.6.0). In effort mode the gains are used, so a gain under a wrong joint name changes how the arm moves.

### L73 the admittance controller
It sends position commands, the only command interface it supports in MoveIt Pro 10.1.0. franka_hardware does not rate-limit joint position commands by default, and the FR3 rejects a position command stream that is not smooth enough. Not tested on a real arm.
franka_hardware also exports an estimated wrench sensor (`fr3_tcp`), which is not wired into this controller.

### L82, L101 the jog controllers
Velocity commands: jog commands in the FR3's position mode can trip its continuity reflex. Only the joint velocity controller reads the JointJog messages, so joint jog must name it.
The joint accelerations are lower than in the mock for the real arm.

## src/fr3_solo_hw/config/moveit/joint_limits.yaml

### L3-L4 the accelerations
From franka_ros2 `franka_fr3_moveit_config/config/fr3_joint_limits.yaml` (release v3.6.0), Franka's own MoveIt limits for the real FR3. The mock's accelerations are higher.

## src/fr3_solo_hw/description/fr3_solo_hw.ros2_control.xacro

### L3 the `robot_ip` default
The default of the `robot_ip` launch argument in franka_ros2 `franka_bringup/launch/franka.launch.py` (release v3.6.0), the single-arm bringup. The value in `config/config.yaml` wins.

### L9 synchronous hardware
franka_ros2's single-arm bringup (`franka_bringup/urdf/franka_arm.urdf.xacro`) uses `is_async` false; only its dual-arm setups use an async component.
An async component would need franka_ros2's `manage_overruns` patch to ros2_control for its overrun setting; the synchronous one needs no patch.

### L10 the Franka Hand
`include_finger_joint` false: the hand is not a ros2_control joint here. Its joint names (`fr3_finger_joint1`, `fr3_finger_joint2`) come from the franka_gripper node.

## src/fr3_solo_hw/launch/robot_drivers_to_persist.launch.py

### L11 `robot_ip_from_config`
The arm reads `robot_ip` from `config/config.yaml` through `urdf_params`. The Franka Hand driver reads the same entry through MoveIt Pro's config parser, so one edit changes both. This file runs in the drivers process, where `MOVEIT_CONFIG_PACKAGE` and `USER_WS` are set.

### L23-L24 the franka_gripper node
Started directly rather than through franka_gripper's `gripper.launch.py`, which has no remap option. The node publishes finger joint states on `~/joint_states`; the remap puts them on `/joint_states`, so the planning scene shows the real finger opening.
If the node cannot reach the hand, it exits and the finger joints are missing from the robot state.

## src/fr3_solo_hw/objectives/open_gripper.xml

### L6 the position
franka_gripper's GripperCommand action moves the fingers to twice the position (a per-finger value to a total width). A width above the hand's maximum width is rejected, so Open Gripper stays below the maximum.

## src/fr3_solo_hw/objectives/teleoperate.xml

### L3 why this file exists
Core "Teleoperate" leaves "Request Teleoperation" at its defaults, which execute on the admittance controller with position commands. This file passes the effort trajectory controller instead. It wraps "Request Teleoperation" rather than copying it, so it does not go stale against MoveIt Pro's ports.

## src/fr3_solo_hw/objectives/recover_arm.xml

### L13 the recovery sequence
A reflex or error makes franka_hardware's `read()` fail. ros2_control then takes the hardware component out of active and deactivates every controller that uses its interfaces.
So recovery has three steps: clear the fault on the arm, make the component active again, and re-activate the broadcasters. Motion controllers stay inactive until an Objective activates them.

## src/franka_behaviors/include/franka_behaviors/error_recovery_action.hpp

### L42-L44 why recovery is only the first step
See the "Recover Arm" entry above. The franka_ros2 action server node is created in the hardware component's `on_init`, so the `error_recovery` action stays reachable while the component is inactive. The action has no goal or result fields.

## src/franka_behaviors/include/franka_behaviors/set_hardware_component_state.hpp

### L43-L45 lifecycle transitions
The controller manager works out the chain of transitions, so an unconfigured component can go to active in one call. A request for the state the component is already in reports failure.

## src/mobile_fr3_duo_hw/config/config.yaml

### L8 based on mobile_fr3_duo_sim, not mobile_fr3_duo_mock
The reasons of fr3_solo_hw's entry apply: the sim changes the `ros2_control_xacro` argument, the driver launch file and the controllers, which this config replaces, and adds `mujoco_model` and `mujoco_viewer`, which no hw xacro file declares, so xacro ignores them.
The hw Nav2 launch also layers the sim's AMCL and laser parameters.

### L30 `joint_group_name`
The mock's default `manipulator` includes the spine, which no controller moves on the real robot. The SRDF has no group of both arms without the spine; adding one would change the mock's SRDF.

### L51-L58 the two Objective library keys
As in fr3_solo_hw's config.yaml (see that entry).

### L59 `waypoints_file`
RetrieveWaypoint returns a waypoint only for a group listed in its `joint_group_names` (retrieve_waypoint.cpp in 10.1.0). The mock lists only `manipulator` for Ready and Stow, so this copy adds the two spine-free arm groups to both.

### L67-L68 broadcasters only at startup
As in fr3_solo_hw. The base controller is inactive too: activating it switches the base to Cartesian velocity mode, with the same stall. "Stow Arms for Navigation" activates it while the arms are still.

## src/mobile_fr3_duo_hw/config/control/mobile_fr3_duo_hw.ros2_control.yaml

### L3-L17 the controller manager settings
The settings and reasons of fr3_solo_hw's controllers file. With three devices, `shutdown_on_initial_state_failure: false` also keeps two working devices up when the third one is unreachable.

### L51 `extra_joints`
TmrHardware without the passive base exports only the four drive joints, but the URDF also has the rocker and caster joints. Without a value for them the robot state stays incomplete. They are published at zero, their position on level ground.

### L71-L73 the controller of both arms
Teleoperation and the per-arm Objectives send a trajectory for one arm. With `allow_partial_joints_goal`, the controller holds the joints the goal does not name at their current position.

### L96, L128, L153 the effort gains
From franka_ros2 `franka_fr3_moveit_config/config/fr3_ros_controllers.yaml` (v3.6.0), the same for both arms. Franka's mobile FR3 Duo MoveIt configuration has no effort trajectory controller.

### L163, L178, L201 the per-arm controllers
They use the mock's spine-free groups. The admittance controllers send position commands only (10.1.0), as in fr3_solo_hw. The jog controllers command velocity, with lower joint accelerations than the mock.

### L217 `base_swerve_controller`
- The name follows the workspace's `base_` prefix. The controller reads TwistStamped on `~/cmd_vel` and publishes `~/odom`. Its `cmd_vel_topic` parameter is declared but not used in v3.6.0 (`swerve_drive_controller.cpp`).
- `enable_odom_tf: false`: robot_state_publisher already publishes `odom` -> `base_link` from the planar joints. A second publisher would give `base_link` two parents.
- `odom_open_loop: false`: odometry from the base's own Cartesian pose state, not from integrated commands. The parameter file's default and the code's own default disagree, so it is set.
- The limits are franka_ros2 `franka_bringup/config/controllers.yaml` (v3.6.0), whose comments say its angular acceleration was tested against the base's reflex. Franka's mobile FR3 Duo MoveIt configuration has higher limits.

## src/mobile_fr3_duo_hw/description/mobile_fr3_duo_hw.ros2_control.xacro

### L3-L6 the address defaults
`robot_ips` in franka_ros2 `franka_bringup/config/mobile_fr3_duo.config.yaml` (v3.6.0), in the order base, left arm, right arm. The values in `config/config.yaml` win.

### L14 asynchronous components
franka_ros2's `mobile_fr3_duo.launch.py` makes all three components asynchronous. Each `read()` waits for its device's next state, so three synchronous devices would wait in turn inside one loop.
franka_ros2 patches ros2_control to read `manage_overruns` from the async tag. Unpatched ros2_control (hardware_interface 4.45 in the MoveIt Pro main image) reads the tag's other attributes and ignores that one, so the async threads use its default overrun handling. Not tested on a robot.

## src/mobile_fr3_duo_hw/launch/robot_drivers_to_persist.launch.py

### L65 `gripper()`
As in fr3_solo_hw, with a namespace per side so the two nodes and their actions are distinct.

### L120 the scan frame
franka_mobile_sensors' nanoScan3 macro (v3.6.0) puts the scan frame on the mounting point with a quarter turn about z. The macro itself is not used: franka_mobile_sensors is not built (one of its dependencies has no Jazzy release), and new URDF links with geometry would need SRDF collision entries.

### L125 the merged scan
AMCL and slam_toolbox take one scan topic, so mobile_fr3_duo_sim's scan merger combines the two scanners.

### L140 `cameras()`
The camera link sits on the mounting point, as in franka_mobile_sensors' `sensor_suite.xacro`. That macro also adds the D455's mount offset, which this static transform leaves out. Not checked on a robot.

### L171 the spine
`franka_spine_server` is franka_ros2's bridge to the spine's REST API. Its node name, `franka_spine_node`, sets the topic, service and action names that this package's spine joint publisher uses.

## src/mobile_fr3_duo_hw/launch/navigation.launch.py

### L19 `PARAM_LAYERS`
The sim's AMCL and laser layer applies unchanged on top of the mock's file; mappings merge and lists replace, as in the mock's launch file.
This package has its own launch file because the mock's starts `base_twist_to_planar` and a fixed `map` -> `odom`, which do not apply to the real base.

### L49-L91 a saved map or slam_toolbox
A site map is site data and stays out of the repository. Without one, slam_toolbox maps with the sim's `slam_toolbox.yaml` (mapping mode).

## src/mobile_fr3_duo_hw/params/nav2_params.yaml

### L22-L25 the command chain
In nav2_bringup's `navigation_launch.py` (1.3.12, checked in the MoveIt Pro main image), the controller server, the behavior server and the velocity smoother publish into the chain, and the collision monitor publishes Nav2's final velocity command. Pointing its output topic at the base controller avoids a relay node.
Nav2 Jazzy publishes Twist unless every node in the chain sets `enable_stamped_cmd_vel`; the behavior server's spin, back-up and other behavior plugins read it too. The docking server publishes outside the chain, so docking does not drive this base.

## src/mobile_fr3_duo_hw/script/base_odometry_bridge.py

### L89 `twist_in_odom_axes`
In closed-loop mode the swerve controller differentiates the base's Cartesian pose, so its twist is along the odometry axes (`swerve_drive_controller.cpp`, v3.6.0). Nav2 reads the twist in `base_link`. Set false if a later version reports body-frame velocities.

### L112-L117 the odom origin
The base's pose state is its own odometry since power-on, so the first sample can be far from zero. Taking it as the origin keeps the planar joints from jumping when the controller activates; AMCL would read a jump as a teleport.

## src/mobile_fr3_duo_hw/script/spine_joint_state_publisher.py

### L88-L90 nothing before the first valid sample
The spine height moves the base of both arms in the planning scene, so a default height could hide a real collision. The cost: without spine data, MoveIt plans nothing.

## src/mobile_fr3_duo_hw/config/drivers.yaml

### L9 `spine_ip`
franka_ros2 `franka_spine/index.rst` launches the spine server with the base's address.

### L16 `host_ip`
The scanners send their data to this address, so it must be the host's address on the robot network.
