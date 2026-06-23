# MoveIt Pro 8.x (Humble) → 9.x (Jazzy) Migration

This document records every change made to migrate the `franka_hw` workspace from
MoveIt Pro 8.x / ROS 2 Humble to MoveIt Pro 9.x / ROS 2 Jazzy.

Reference: [9.0 user workspace migration guide](https://docs.picknik.ai/how_to/migration_guides/9.0_user_workspace_migration/).
The changes were validated against a known-good Jazzy 9.3.0 workspace (`meta_ws`) that
uses the same `franka_arm_hw` / `franka_base_config` packages and the same `franka_ros2`
submodule.

Branch: `migrate/moveit-pro-9-jazzy` (5 logical commits).

---

## What did *not* need changing

The package *naming* was already 9.x-style, so no rename pass was needed:

- Behaviors already use `moveit_pro::behaviors::` and depend on
  `moveit_pro_behavior_interface` / `moveit_pro_behavior` / `moveit_pro_package`.
- Objectives already use the 9.x port name `message_timeout_sec`.
- `behavior_loader_plugins` already use `moveit_pro::behaviors::*Loader`.
- `joint_jog.yaml` already existed for both arms, and the dual-arm control yaml already
  defined `joint_velocity_controller`.

`moveit_studio_agent`, `moveit_studio_plugins`, and `moveit_studio_sdk_msgs` are **not**
deprecated — those package names are unchanged in 9.x (only the behavior packages and the
`STUDIO_*` → `MOVEIT_*` environment variables were renamed).

---

## 1. Workspace restructure into `src/`

**Why:** MoveIt Pro 9 builds from `<workspace>/src` (the Dockerfile bind-mounts `./src`).
The packages previously sat at the repo root.

- Moved the four config packages under `src/`:
  - `franka_arm_hw`, `franka_dual_arm_hw`, `franka_base_config`, `franka_behaviors`
- Removed the `MOVEIT_PRO_IGNORE` markers from `franka_arm_hw` and `franka_dual_arm_hw`
  (they were tracked but deleted in the working tree — committing the deletion lets these
  packages build). `franka_base_config` keeps its marker (config-only, not built).

## 2. Submodules → Jazzy driver stack (`src/external_dependencies/`)

**Why:** On Humble the franka driver packages came from the MoveIt Pro base image. On Jazzy
they are built from source, and the Jazzy Dockerfile needs `patches/manage_overruns.patch`
+ `realtime_tools/` from a newer `franka_ros2`.

| Submodule | Old | New |
|-----------|-----|-----|
| `franka_ros2` | `PickNikRobotics/franka_ros2` `humble-fixes` (root) | `frankarobotics/franka_ros2` `jazzy` v3.3.0 (`d812aab`) under `src/external_dependencies/` |
| `libfranka` | *(none)* | `frankarobotics/libfranka` 0.20.5 (`3cc708c`) |
| `franka_description` | *(image-provided)* | `frankarobotics/franka_description` 2.7.1 (`2fd6aaa`) |

`libfranka` (needed by `franka_hardware`) and `franka_description` (the FR3 URDF does
`$(find franka_description)`) are not apt packages on Jazzy, hence vendored as submodules.
Commits pinned to the exact commits the reference workspace uses.

## 3. Single-arm config (`src/franka_arm_hw`)

- **`config/control/franka_ros2_control.yaml`** — added `joint_velocity_controller`:
  the `controller_manager` entry (`joint_velocity_controller/JointVelocityController`)
  and a param block (`planning_group_name: manipulator`, 7-joint limits). Required for
  MoveIt Pro 9 Joint Jog (the old Servo-based jog was removed); `joint_jog.yaml` already
  referenced this controller. `velocity_force_controller` and `joint_velocity_controller`
  use `command_interfaces: ["velocity"]`, matching the meta_ws reference (see §9).
- **`config/config.yaml`** — controller startup lists aligned to the meta_ws
  `franka_arm_hw` reference (see §9): `joint_trajectory_controller` active;
  `joint_trajectory_admittance_controller`, `velocity_force_controller`,
  `joint_velocity_controller` inactive.
- **`package.xml`** — added `exec_depend`s now built from source:
  `franka_hardware`, `franka_gripper`, `franka_robot_state_broadcaster`,
  `franka_semantic_components`.
- **`launch/robot_drivers_to_persist.launch.py`** — `franka_ros2` v3.3.0
  `gripper.launch.py` now derives `joint_names` from `robot_type` and requires a
  `namespace` argument. Replaced `joint_names=[...]` with `namespace: fr3_gripper`.

## 4. Dual-arm config (`src/franka_dual_arm_hw`)

### Pre-existing breakage (not Jazzy-specific — would have failed on 8.x too)

The package referenced names that do not exist in this workspace. Fixed all of them:

| Broken reference | Fixed to |
|------------------|----------|
| `based_on_package: "franka_dual_arm_config"` | *removed* — config is self-contained |
| `moveit_params.*.package: "dual_arm_sim"` (kinematics, sensors_3d, joint_limits, pose_jog) | `franka_dual_arm_hw` |
| commented-out SRDF (`franka_dual_arm_config`) | uncommented → `franka_dual_arm_hw/config/moveit/franka.srdf` |
| `custom_objectives.package_name: "franka_dual_arm_hw_config"` | `franka_dual_arm_hw` |
| commented-out `waypoints_file` (`dual_arm_sim`) | uncommented → `franka_dual_arm_hw` |
| `ros2_control.config.package: "franka_dual_arm_hw_config"` | `franka_dual_arm_hw` |
| `FindPackageShare("franka_dual_arm_config_hw")` in 3 launch files | `franka_dual_arm_hw` |
| `README.md` title `# franka_dual_arm_config` | `# franka_dual_arm_hw` |

### Control architecture reconciliation

This config uses `launch_control_node: False` and starts **two namespaced** (`left`/`right`)
`ros2_control_node`s via `robot_drivers_to_persist.launch.py` →
`left_franka.launch.py` / `right_franka.launch.py`.

- **`ros2_control.config.path`** → `config/control/left_franka_ros2_control.yaml`.
  The combined `franka_ros2_control.yaml` it pointed at does not exist (the hardware
  config splits control into `left_`/`right_` files). MoveIt Pro resolves this path
  *unconditionally* (`ros2_control_config.launch.py`), so it must point at a real file;
  the contents are unused here because `launch_control_node` is False.
- **`controllers_active_at_startup` / `controllers_inactive_at_startup`** → emptied.
  MoveIt Pro's startup spawners run *un-namespaced* against a default `controller_manager`
  that does not exist in this two-namespace setup (the old `left_/right_velocity_force_controller`
  names could never spawn). The per-arm controllers are spawned by the persist launch
  files inside their `left`/`right` namespaces instead — so for the dual arm the
  active/inactive split lives in the spawner arguments, not these lists (see §9).

### Jazzy migration (dual)

- **`package.xml`** — same source-built driver deps as the single arm.
- **`objectives/draw.xml`** — replaced the removed `ActivateControllers` behavior with
  `SwitchController` (see §6).

> The dual-arm `left`/`right` launches already used the v3.3.0 gripper API
> (`namespace` + `robot_ip`), so no gripper-launch change was needed there.

## 5. Dockerfile (Jazzy)

Adapted from the reference workspace Dockerfile:

- Base image distro defaults to `jazzy`
  (`picknikciuser/moveit-studio:${MOVEIT_DOCKER_TAG}-${MOVEIT_ROS_DISTRO:-jazzy}`).
- Handle a pre-existing UID/GID in the base image (`userdel`/`groupdel` guard).
- `rosdep --skip-keys` for the vendored `franka_ros2` packages' unresolvable deps
  (`sick_safetyscanners2`, `zed_*`, `robotiq_*`, …).
- Reinstall the OpenCV core runtime package (pristine restore).
- Build a `manage_overruns`-patched `hardware_interface` (ros2_control 4.44.0) +
  `realtime_tools` 3.10.1 into the overlay, from `franka_ros2`'s
  `patches/manage_overruns.patch` (guarded to hardware_interface 4.44.0).
- Copy the custom colcon mixin; set `CCACHE_DEPEND=1` (works around a GCC 13 ICE in
  libfranka under ccache preprocessor mode).

## 6. `SwitchController` — verified against MoveIt Pro source

The 9.0 guide says to replace `ActivateControllers` with "SwitchControllers", but the
source (`moveit_pro_behavior/behaviors/core/switch_controller.hpp`,
`register_core_behaviors.cpp`) shows:

- Registered behavior ID is **`SwitchController`** (singular).
- Ports **`activate_controllers`** / **`deactivate_controllers`**
  (`std::vector<std::string>`, `;`-separated), both default `""`.
- `automatic_deactivation` defaults to `true` (auto-deactivates conflicting controllers).

`draw.xml` now uses:

```xml
<Action ID="SwitchController" activate_controllers="{controller_name}" />
```

## 7. Workspace build files (new, at repo root)

- **`colcon-defaults.yaml`** — `symlink-install`, build mixins, and `packages-skip` for
  the franka_ros2 packages that are not Jazzy-ready / not needed
  (`franka_bringup`, `franka_example_controllers`, `franka_fr3_moveit_config`,
  `franka_gazebo_bringup`, `franka_mobile*`, `franka_selfcollision`,
  `franka_vision_and_manipulation_kit`, the `franka_ros2` metapackage, …). The packages
  actually built from `franka_ros2` are `franka_hardware`, `franka_gripper`,
  `franka_robot_state_broadcaster`, `franka_semantic_components`, `franka_msgs`.
- **`mixin/disable-libfranka-tests.mixin`** — `-DBUILD_TESTS=OFF` for libfranka (its tests
  need `GTest::gmock`, absent from the container's system GTest).
- **`docker-compose.yaml`** — RT ulimits (`rtprio: 99`, `memlock: -1`) and `/dev`
  passthrough for FR3 1 kHz control + RealSense.
- **`.gitignore`** — `build/ install/ log/ __pycache__/` and the machine-specific `.env`.

The `.env` (license key, host paths, `MOVEIT_DOCKER_TAG=9.3.0`, `MOVEIT_ROS_DISTRO=jazzy`)
is git-ignored; regenerate with `moveit_pro configure` / `moveit_pro envfile`.

## 8. `franka_behaviors` dropped from the build

**Why:** `franka_behaviors` provided a single custom behavior, `FrankaGraspAction`
(an `ActionClientBehaviorBase<franka_msgs::action::Grasp>` wrapper around the gripper
`grasp` action), used only by `franka_arm_hw/objectives/grasp_object.xml`. The package is
unique to this workspace — it does **not** exist in the reference `meta_ws`, so it was
never exercised by the migration validation.

On Jazzy it fails to configure: `find_package(moveit_pro_behavior)` transitively re-finds
`moveit_pro_mpc` → PCL → system VTK 9.1, whose `VTK::mpi` target references an
`MPI::MPI_C` imported target that nothing in the package's CMake defines, so the CMake
*generate* step aborts. (The base image *does* ship `libopenmpi-dev` + `FindMPI.cmake`,
so the alternative fix would be an early `find_package(MPI)` — but the behavior isn't
needed, so we drop the package instead.)

- **`objectives/grasp_object.xml`** — the `FrankaGraspAction` node was replaced with a
  `<SubTree ID="Close Gripper" />` call, mirroring how `meta_ws` objectives invoke the
  gripper objectives (e.g. `franka_arm_hw/objectives/request_teleoperation.xml`).
  `close_gripper.xml` already uses the **core** `MoveGripperAction` behavior against
  `/fr3_gripper/gripper`, so no custom behavior is required.
- **`colcon-defaults.yaml`** — `franka_behaviors` added to both the build and test
  `packages-skip` lists.
- The package source is left in `src/franka_behaviors/` (not built); the
  `FrankaBehaviorsLoader` was never registered in any config's `behavior_loader_plugins`,
  so nothing else references it.

> Note: `franka_behaviors/franka_grasp_action.hpp` also `#include`d
> `franka_msgs/action/move.hpp` but never used the `Move` action — only `Grasp` was
> registered. Moot now that the package is skipped.

## 9. Controller startup aligned with the meta_ws `franka_arm_hw` reference

**Why:** the arms should be driven by a **joint trajectory controller** only, with the
active/inactive-at-startup split matching the known-good meta_ws `franka_arm_hw`
config. Previously `franka_arm_hw` had the admittance controller active and the plain
`joint_trajectory_controller` inactive — the reverse of the reference.

### Single arm (`franka_arm_hw`)

`config/config.yaml` now matches meta_ws:

| | Controllers |
|---|---|
| `controllers_active_at_startup` | `joint_state_broadcaster`, `franka_robot_state_broadcaster`, `joint_trajectory_controller` |
| `controllers_inactive_at_startup` | `joint_trajectory_admittance_controller`, `velocity_force_controller`, `joint_velocity_controller` |

`joint_trajectory_controller` is the sole active motion controller (`effort` command
interface). The admittance / velocity-force / joint-velocity controllers stay **defined
but inactive**, available to switch to at runtime (`joint_velocity_controller` is what
Joint Jog activates). In `franka_ros2_control.yaml`, `velocity_force_controller` and
`joint_velocity_controller` were switched from `command_interfaces: ["position"]` to
`["velocity"]` to match the reference.

### Dual arm (`franka_dual_arm_hw`)

This config doesn't use MoveIt Pro's startup spawners (`launch_control_node: False`,
empty startup lists — see §4), so the active/inactive split lives in the per-arm spawner
arguments in `left_franka.launch.py` / `right_franka.launch.py`. Each namespace now spawns:

- `joint_state_broadcaster` — active
- `joint_trajectory_controller` — **active** (was `--inactive`)
- `velocity_force_controller` — `--inactive`
- `joint_velocity_controller` — `--inactive` (added; required for the dual `joint_jog.yaml`)

Their `command_interfaces` were likewise aligned to `["velocity"]`. `franka_robot_state_broadcaster`
remains commented out in the per-arm launches (its dual-arm wiring is unverified — see caveats).

---

## Build

```bash
git submodule update --init --recursive
moveit_pro build
moveit_pro run -c franka_arm_hw
```

## Open items / caveats

- **Dual-arm runtime wiring is not verified.** The static references are fixed and the
  config loads, but the two-namespace runtime behavior can only be validated on a real
  robot / sim. In particular `config/moveit/joint_jog.yaml` lists `joint_velocity_controller`
  for both arms, which MoveIt Pro Joint Jog would need to address per-namespace
  (`left/…`, `right/…`). The single-arm config is the clean, primary path.
- **`SwitchController` activation** in `draw.xml` relies on `automatic_deactivation`
  defaulting to true; confirm the controller switch behaves as intended once running.
