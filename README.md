# Franka Hardware Packages for MoveIt Pro

Hardware-specific configuration packages for using [Franka Research 3 (FR3)](https://franka.de) robots with [MoveIt Pro](https://docs.picknik.ai).

This is a MoveIt Pro **9.x / ROS 2 Jazzy** workspace. (For the older MoveIt Pro 8.x /
Humble version, see history before the Jazzy migration.)

## Layout

Config packages live under `src/`; external driver sources are git submodules under
`src/external_dependencies/`:

- **src/franka_arm_hw** — Single-arm hardware configuration
- **src/franka_dual_arm_hw** — Dual-arm hardware configuration
- **src/franka_base_config** — Shared base configuration (config-only; `MOVEIT_PRO_IGNORE`)
- **src/franka_behaviors** — Franka-specific behaviors (e.g., gripper grasp/move actions)
- **src/external_dependencies/franka_ros2** — Franka ROS 2 driver stack (submodule, `jazzy`)
- **src/external_dependencies/libfranka** — libfranka (submodule, 0.20.5)
- **src/external_dependencies/franka_description** — FR3 URDF/xacro (submodule, 2.7.1)

On Jazzy the franka driver packages (`franka_hardware`, `franka_gripper`,
`franka_robot_state_broadcaster`, `franka_semantic_components`) and `libfranka` /
`franka_description` are built from source rather than provided by the base image.

## Build

The quickest way to set up the workspace from scratch is the `setup_jazzy.sh` script in
the repo root. It updates the git submodules, runs `moveit_pro configure`, regenerates the
`.env` file (ensuring `MOVEIT_ROS_DISTRO=jazzy`), and builds the user image + workspace:

```bash
./setup_jazzy.sh
moveit_pro run -c franka_arm_hw
```

To run the steps manually instead:

```bash
git submodule update --init --recursive   # fetch franka_ros2, libfranka, franka_description
moveit_pro build                           # builds the Jazzy user image + workspace
moveit_pro run -c franka_arm_hw
```

`.env` selects the base image (`MOVEIT_DOCKER_TAG=9.3.0`, `MOVEIT_ROS_DISTRO=jazzy`) and is
gitignored; regenerate with `moveit_pro configure` / `moveit_pro envfile`.

## Setup

See the [Franka FR3 Hardware Setup Guide](https://docs.picknik.ai/hardware_guides/franka_fr3_hardware_setup_guide) for full instructions.
