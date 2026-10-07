# fr3_duo_spine_mock

MoveIt Pro configuration for the Franka FR3 Duo on a fixed base: a pedestal, the vertical spine, the Duo mount with the head, two FR3 v2 arms and two Franka Hands.
It is the Mobile FR3 Duo (`mobile_fr3_duo_mock`) without the TMR mobile base, so the two configurations share joint names, planning groups and controller names.

This configuration runs on mock hardware (`mock_components/GenericSystem`), with no physics simulation and no driver.
The pedestal is a plain box whose height is the spine mounting height in franka_description.

## Spine

The spine joint (`franka_spine_vertical_joint`, group `spine`) moves through:

- joint jog in the teleop, through `spine_jvc`;
- "Move Spine Absolute": an absolute height in meters, within the travel;
- "Move Spine to Pose Height": the height of a target pose, plus an optional bias, clamped to the travel.

Both Objectives run `spine_jtc` and leave the arms where they are. Their ports follow the real spine's `MoveAbsolute` action (position, velocity, acceleration, deceleration, timeout, action name), so callers stay the same on real hardware. Mock and sim ignore everything but the position. Real hardware has no ros2_control interface for the spine: it will need its own versions of these Objectives, with a Behavior that calls Franka's spine `MoveAbsolute` action. That is not part of this workspace yet.

The Behaviors `CreateSpineState` and `GetSpineStateForPoseHeight` come from `franka_spine_behaviors`.

## Build and run

```bash
git submodule update --init
moveit_pro build
moveit_pro run -c fr3_duo_spine_mock
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | hardware, MoveIt parameters, controllers, Objectives |
| `config/control/` | ros2_control controllers |
| `config/moveit/` | SRDF, joint limits, kinematics, jogging |
| `description/` | robot xacro and mock ros2_control hardware |
| `objectives/`, `waypoints/` | gripper and spine Objectives, and saved waypoints |

## Controllers

The whole upper body (`manipulator`: spine and both arms) keeps the standard controller name. Controllers for a part of the robot use short names: `jtac` (JointTrajectoryAdmittanceController), `jtc` (JointTrajectoryController), `vfc` (VelocityForceController), `jvc` (JointVelocityController). `left_`/`right_` is one arm, `spine_` is the spine alone, and the `_w_spine` suffix adds the spine to an arm.

| Controller | Type | Group | At startup |
| --- | --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all joints | active |
| `joint_trajectory_admittance_controller` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `manipulator` | active |
| `left_gripper_controller`, `right_gripper_controller` | `position_controllers/GripperActionController` | `left_gripper`, `right_gripper` | active |
| `left_jtac_w_spine`, `right_jtac_w_spine` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `left_manipulator`, `right_manipulator` | inactive |
| `left_jtac`, `right_jtac` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_vfc`, `right_vfc` | `velocity_force_controller/VelocityForceController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_jvc`, `right_jvc` | `joint_velocity_controller/JointVelocityController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `spine_jtc` | `joint_trajectory_controller/JointTrajectoryController` | `spine` | inactive |
| `spine_jvc` | `joint_velocity_controller/JointVelocityController` | `spine` | inactive |
