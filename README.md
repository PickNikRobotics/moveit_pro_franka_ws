# Franka Configurations for MoveIt Pro

[MoveIt Pro](https://docs.picknik.ai) configuration packages for [Franka Robotics](https://franka.de) robots: the Franka Research 3 (FR3) arm, the FR3 Duo and the Mobile FR3 Duo.

This is a MoveIt Pro **10.2+ / ROS 2 Jazzy** workspace.

## Packages

| Package | Robot | Hardware |
| --- | --- | --- |
| [`fr3_solo_mock`](src/fr3_solo_mock) | One FR3 arm with a Franka Hand | mock hardware |
| [`fr3_solo_sim`](src/fr3_solo_sim) | One FR3 arm with a Franka Hand | MuJoCo, robot only |
| [`fr3_duo_mock`](src/fr3_duo_mock) | FR3 Duo on a fixed pedestal and spine: two FR3 v2 arms, two Franka Hands | mock hardware |
| [`fr3_duo_sim`](src/fr3_duo_sim) | FR3 Duo on a fixed pedestal and spine | MuJoCo, robot only |
| [`mobile_fr3_duo_mock`](src/mobile_fr3_duo_mock) | Mobile FR3 Duo: TMR v0.2 swerve base, spine, two FR3 v2 arms, two Franka Hands | mock hardware |
| [`mobile_fr3_duo_sim`](src/mobile_fr3_duo_sim) | Mobile FR3 Duo | MuJoCo, with a room scene, lidars and Nav2 |
| [`franka_behaviors`](src/franka_behaviors) | Franka-specific Behaviors (grasp, collision thresholds) | excluded from the build |
| [`external_dependencies/franka_description`](https://github.com/frankarobotics/franka_description) | Franka robot descriptions | submodule, release 2.9.0 |

Each `_sim` package inherits from its `_mock` package (`based_on_package`) and changes only the hardware.
Real-hardware (`_hw`) configurations are not part of this workspace yet; they return in a later change.

## Getting started

```bash
git clone --recurse-submodules https://github.com/PickNikRobotics/moveit_pro_franka_ws.git
cd moveit_pro_franka_ws
moveit_pro configure -w "$(pwd)"
moveit_pro build
moveit_pro run -c fr3_solo_sim
```

Use `-c` with any configuration package from the table, for example `-c mobile_fr3_duo_sim`.
If you cloned without `--recurse-submodules`, run `git submodule update --init` first.

## Controller names

The whole-body group of each robot keeps the standard MoveIt Pro controller names (`joint_trajectory_admittance_controller`, `joint_trajectory_controller`, `velocity_force_controller`, `joint_velocity_controller`).
Controllers for part of a robot use short names: `jtac` (JointTrajectoryAdmittanceController), `jtc` (JointTrajectoryController), `vfc` (VelocityForceController), `jvc` (JointVelocityController) and `jgvc` (JointGroupVelocityController).
`left_` and `right_` mean one arm, the `_w_spine` suffix adds the spine, and `spine_` and `base_` name those groups. Each package README lists its controllers.

## Use from another workspace

The packages are laid out so that another workspace, such as [moveit_pro_example_ws](https://github.com/PickNikRobotics/moveit_pro_example_ws), can take them in as a vendored copy under its `src/external_dependencies/` and build its own configurations on them with `based_on_package`.
Such a copy leaves out this workspace's submodules, so the consuming workspace provides franka_description itself, at the same release.

## License

BSD 3-Clause, see [LICENSE](LICENSE). Third-party files and their licenses are listed in [NOTICE](NOTICE).
