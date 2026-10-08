# Franka Configurations for MoveIt Pro

[MoveIt Pro](https://docs.picknik.ai) configuration packages for [Franka Robotics](https://franka.de) robots: the Franka Research 3 (FR3) arm and the FR3 Duo.

This is a MoveIt Pro **10.1.0+ / ROS 2 Jazzy** workspace.

## Packages

| Package | Robot | Hardware |
| --- | --- | --- |
| [`fr3_solo_mock`](src/fr3_solo_mock) | One FR3 arm with a Franka Hand | mock hardware |
| [`fr3_solo_sim`](src/fr3_solo_sim) | One FR3 arm with a Franka Hand | MuJoCo, robot only |
| [`fr3_duo_mock`](src/fr3_duo_mock) | FR3 Duo without a spine: the Duo mount on a fixed base, two FR3 arms, two Franka Hands | mock hardware |
| [`fr3_duo_sim`](src/fr3_duo_sim) | FR3 Duo without a spine | MuJoCo, robot only |
| [`fr3_duo_spine_mock`](src/fr3_duo_spine_mock) | FR3 Duo on a fixed pedestal and spine: two FR3 v2 arms, two Franka Hands | mock hardware |
| [`fr3_duo_spine_sim`](src/fr3_duo_spine_sim) | FR3 Duo on a fixed pedestal and spine | MuJoCo, robot only |
| [`franka_behaviors`](src/franka_behaviors) | Franka Behaviors: the spine Behaviors (`CreateSpineState`, `GetSpineStateForPoseHeight`); real-hardware Behaviors kept in `hardware/` | built with the workspace, except `hardware/` |
| [`external_dependencies/franka_description`](https://github.com/frankarobotics/franka_description) | Franka robot descriptions | submodule, release 2.9.0 |

Each `_sim` package inherits from its `_mock` package (`based_on_package`) and sets the mock's `ros2_control_xacro` argument to its own MuJoCo hardware.
`fr3_solo_sim`, `fr3_duo_sim` and `fr3_duo_spine_sim` change only the hardware.
Real-hardware (`_hw`) configurations are not part of this workspace yet; [issue #10](https://github.com/PickNikRobotics/moveit_pro_franka_ws/issues/10) tracks them.

## Getting started

Meshes and images are stored in Git LFS, so install [git-lfs](https://git-lfs.com) before you clone.

```bash
git lfs install
git clone --recurse-submodules https://github.com/PickNikRobotics/moveit_pro_franka_ws.git
cd moveit_pro_franka_ws
moveit_pro configure -w "$(pwd)"
moveit_pro build
moveit_pro run -c fr3_solo_sim
```

Use `-c` with any configuration package from the table.
If you cloned without `--recurse-submodules`, run `git submodule update --init` first.
If you cloned without git-lfs, run `git lfs install` and `git lfs pull` first.

## Controller names

The whole-body group of each robot keeps the standard MoveIt Pro controller names (`joint_trajectory_admittance_controller`, `joint_trajectory_controller`, `velocity_force_controller`, `joint_velocity_controller`).
Controllers for part of a robot use short names: `jtac` (JointTrajectoryAdmittanceController), `jtc` (JointTrajectoryController), `vfc` (VelocityForceController), `jvc` (JointVelocityController) and `jgvc` (JointGroupVelocityController).
`left_` and `right_` mean one arm, the `_w_spine` suffix adds the spine, and `spine_` and `base_` name those groups. Each package README lists its controllers.

## Make your own configuration

To make your own configuration, copy a package and rename it.

## License

BSD 3-Clause, see [LICENSE](LICENSE). Third-party files and their licenses are listed in [NOTICE](NOTICE).
