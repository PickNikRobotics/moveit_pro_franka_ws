# franka_behaviors

MoveIt Pro Behaviors for Franka robots. The loader is `franka_behaviors::FrankaBehaviorsLoader`.

## Built Behaviors

| Behavior | What it does |
| --- | --- |
| `CreateSpineState` | Creates a spine-only goal at an absolute spine height. A height outside the spine travel fails. |
| `GetSpineStateForPoseHeight` | Creates a spine-only goal that puts the arm mount at the height of a target pose, clamped to the travel. |
| `FrankaErrorRecovery` | Clears a reflex or error on a Franka arm through the franka_ros2 `error_recovery` action. |
| `SetHardwareComponentState` | Moves a ros2_control hardware component to a lifecycle state, for example back to `active` after a driver error. |
| `FrankaGraspAction` | Grasps with the Franka Hand at a set width, speed and force through the franka_gripper `grasp` action. |

The spine Behaviors do not move the robot: the Objectives "Move Spine Absolute" and "Move Spine to Pose Height" in `fr3_duo_spine_mock` and `mobile_fr3_duo_mock` feed their output to "Move to Joint State" on the `spine` group.
The real-hardware Behaviors need franka_ros2 (`franka_msgs`, from `src/external_dependencies/franka_ros2`). The Objectives "Recover Arm" and "Grasp Object" in `fr3_solo_hw` use them.

The package builds on MoveIt Pro 10.1.0 and later. Tests: `colcon test --packages-select franka_behaviors`.

## Not included

The collision-threshold Behaviors (`FrankaSetForceTorqueCollisionBehavior`, `FrankaSetFullCollisionBehavior`) are not part of this package. No Objective uses them.
