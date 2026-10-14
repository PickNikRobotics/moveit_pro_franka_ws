# AGENT_COMMENTS.md

The detail behind short code comments: why a setting is what it is, and where a value comes from.
Organized by file, then by line, so an entry is findable from a code location.
Keep entries short. When you change the code an entry describes, change the entry in the same commit.

## colcon-defaults.yaml

### L16 `packages-ignore`
fr3_solo_hw needs five franka_ros2 packages: franka_msgs, franka_hardware, franka_gripper, franka_robot_state_broadcaster, franka_semantic_components.
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
franka_hardware changes the arm's control mode when a controller that claims command interfaces activates. That switch stops the robot and starts the new mode with blocking calls to the control box, inside the controller manager's update loop, so the loop misses cycles.
During bring-up that can trip the communication reflex. With only state broadcasters at startup, bring-up switches no mode; each Objective activates the controller it needs.

## src/fr3_solo_hw/config/control/fr3_solo_hw.ros2_control.yaml

### L8 `thread_priority`
The real-time priority of the controller manager's thread. The hardware component is synchronous (see the description file), so this thread also reads and writes the robot. The drivers container allows this priority (docker-compose.yaml).

### L9-L12 `overruns`
ros2_control's default skips the cycles a long loop missed and re-phases. `manage: false` lets the loop catch up instead, as franka_ros2's own controller files set it. Warnings stay on: they are the only report of overruns.

### L13-L14 `lock_memory`
`ros2_control_node` locks the process memory only when this is true or the kernel is real-time. A failure to lock is only a warning, so check the drivers log for "Unable to lock the memory".

### L15-L17 `shutdown_on_initial_state_failure`
franka_hardware refuses to activate while the robot reports a reflex or an error. The default turns that refusal into the end of the controller manager. False keeps the node up with the arm inactive, so "Recover Arm" can clear the fault.

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
