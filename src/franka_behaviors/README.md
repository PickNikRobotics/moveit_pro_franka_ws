# franka_behaviors

MoveIt Pro Behaviors for Franka robots. The loader is `franka_behaviors::FrankaBehaviorsLoader`.

## Built Behaviors

| Behavior | What it does |
| --- | --- |
| `CreateSpineState` | Creates a spine-only goal at an absolute spine height. A height outside the spine travel fails. |
| `GetSpineStateForPoseHeight` | Creates a spine-only goal that puts the arm mount at the height of a target pose, clamped to the travel. |

Neither moves the robot: the Objectives "Move Spine Absolute" and "Move Spine to Pose Height" in `fr3_duo_spine_mock` and `mobile_fr3_duo_mock` feed their output to "Move to Joint State" on the `spine` group.

The package builds on MoveIt Pro 10.1.0 and later. Tests: `colcon test --packages-select franka_behaviors`.

## Real-hardware Behaviors

The real-hardware Behaviors (`FrankaGraspAction`, `FrankaSetForceTorqueCollisionBehavior`, `FrankaSetFullCollisionBehavior`) need franka_ros2. They return with the `_hw` configurations, tracked in [issue #10](https://github.com/PickNikRobotics/moveit_pro_franka_ws/issues/10).
