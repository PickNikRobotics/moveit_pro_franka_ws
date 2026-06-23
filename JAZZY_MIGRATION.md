# MoveIt Pro 8.x (Humble) → 9.x (Jazzy) Migration

This document records every change made to migrate the `franka_hw` workspace from
MoveIt Pro 8.x / ROS 2 Humble to MoveIt Pro 9.x / ROS 2 Jazzy.

Reference: [9.0 user workspace migration guide](https://docs.picknik.ai/how_to/migration_guides/9.0_user_workspace_migration/).
The changes were validated against a known-good Jazzy 9.3.0 workspace (`meta_ws`) that
uses the same `franka_arm_hw` / `franka_base_config` packages and the same `franka_ros2`
submodule.

Branch: `migrate/moveit-pro-9-jazzy` (logical commits — see `git log`).

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

## 10. Teleoperation objectives

Adapted from the meta_ws `franka_arm_hw` `teleoperate.xml` / `request_teleoperation.xml`
(the standard MoveIt Pro teleop subtree: gripper open/close, joint-slider interpolate,
interactive-marker move-to-pose, waypoint move-to-joint-state, Cartesian jog, joint jog).

### Single arm (`franka_arm_hw`)

Copied essentially verbatim — same robot as the reference (`manipulator`, `grasp_link`,
`joint_trajectory_controller` active, `jtc` pipeline). `Teleoperate` invokes
`Request Teleoperation`, which uses the existing `Close Gripper` / `Open Gripper`
objectives and the core `Move to Pose` / `Move to Joint State` / `Interpolate to Joint State`
subtrees.

### Dual arm (`franka_dual_arm_hw`) — right arm only

The dual config runs **two namespaced controller_managers** (`/left`, `/right`). The
teleop objective here is scoped to the **right arm** (`right_manipulator`,
`right_fr3_link8`, `/right/joint_trajectory_controller`):

- Jog modes (joint / Cartesian) point `SwitchController` at
  `/right/controller_manager/{list,switch}_controllers` via its action-name ports, so
  they drive the right arm. These are the supported teleop paths.
- **Known limitation:** the core `Move to Pose` / `Move to Joint State` /
  `Interpolate to Joint State` subtrees (teleop modes 3/4/5) run an internal
  `SwitchController` against the default, un-namespaced `/controller_manager`, with no
  port to override the namespace. That manager doesn't exist in this two-namespace setup,
  so those modes are not expected to work as-is.
- Added right-arm `close_gripper.xml` / `open_gripper.xml` (the dual config had none) so
  the gripper modes resolve. They drive `/right/franka_gripper/gripper_action` — the
  franka_gripper node runs in the `right` namespace (matches the meta_ws
  `/<ns>/franka_gripper/gripper_action` convention).

> Aside: the **single-arm** gripper objectives use `/fr3_gripper/gripper_action`
> (and `close_gripper.xml` even uses `/fr3_gripper/gripper`), omitting the
> `/franka_gripper/` node level that the v3.3.0 `gripper.launch.py` actually creates.
> Pre-existing; left as-is, flagged for follow-up.

---

## 11. Mock-hardware bring-up (fixes found running both configs locally)

Bringing both configs up against `mock_components/GenericSystem` (no real FR3 / FCI)
surfaced a set of latent migration bugs — each one blocked *real* hardware too, since the
URDF/SRDF/launch wiring is shared. All of the fixes below are kept on by default; the
mock-vs-real choice is a small, documented toggle (see "Enabling mock hardware" at the end).

### 11.1 `franka_description` 2.7.1 URDF macro signature (both arms)

`description/franka.urdf.xacro` (single **and** dual) called `xacro:franka_robot` with the
old signature — `arm_id="fr3"` plus `joint_limits`/`inertials`/`kinematics`/`dynamics`
loaded via `xacro.load_yaml`. The pinned `franka_description` 2.7.1 macro instead takes
`robot_type` and loads those tables internally, so the old call aborted with
`Invalid parameter "arm_id"` and the URDF never parsed (mock or real). Updated both files to
`robot_type="fr3"` and removed the four `load_yaml` params. The single-arm file also gained a
`use_fake_hardware` arg (default `false`) threaded into the macro, used by the optional mock
toggle.

### 11.2 Single-arm SRDF robot name + waypoint group

- `franka_base_config/config/moveit/franka.srdf`: `<robot name="franka">` → `<robot name="fr3">`.
  It must match the URDF robot name (`fr3`, as in meta_ws); otherwise MoveIt drops the
  semantic description (`"Semantic description is not specified for the same robot as the
  URDF"`) and **no planning groups load**.
- `franka_arm_hw/waypoints/waypoints.yaml`: the `Home` waypoint referenced group `arm`,
  which doesn't exist — changed to `manipulator` (the SRDF group, and the config's
  `joint_group_name`).

### 11.3 `franka_behaviors` excluded from behavior discovery (`COLCON_IGNORE`)

§8 dropped `franka_behaviors` from the *build*, but the agent still aborted on startup:

```
Failed to load library `franka_behaviors::FrankaBehaviorsLoader` ... does not exist
```

The objective server's behavior scanner (`moveit_studio_utils_py/system_config.py`) walks
`<ws>/src` and merges the `behavior_loader_plugins` from every package that has a
`behavior_plugin.yaml`, **skipping a directory only if it contains `COLCON_IGNORE`** — it does
*not* honor `MOVEIT_PRO_IGNORE`. So `src/franka_behaviors/behavior_plugin.yaml` was still
discovered, its loader requested, and — since the package isn't built — pluginlib failed and
crashed the agent. Fix: add `src/franka_behaviors/COLCON_IGNORE` (this also redundantly
drops it from the build). This blocked **both** configs.

### 11.4 Dual-arm controller bring-up on Jazzy

The dual config runs two namespaced `ros2_control_node`s (`/left`, `/right`) started by the
per-arm launches. Three Jazzy issues stopped the controllers from coming up:

- **Dead `franka_bringup` import** — `robot_drivers_to_persist.launch.py` called
  `get_package_share_directory("franka_bringup")` at module import to append a `utils` path
  to `sys.path` that nothing used. `franka_bringup` isn't installed, so the import threw and
  the whole drivers launch died. Removed the dead import (and the now-unused `os`/`sys`/
  `get_package_share_directory` imports).
- **`robot_description` not published per namespace** — Jazzy's `controller_manager` reads
  `robot_description` from a (latched) topic, not the node parameter the launches passed, so
  both managers hung on *"Waiting for data on 'robot_description' topic"*. Added a
  `robot_state_publisher` in each namespace (`left_franka.launch.py` / `right_franka.launch.py`)
  to publish `/<ns>/robot_description`, with `/tf` + `/tf_static` remapped to sinks so the
  agent's combined dual-arm RSP keeps sole TF authority.
- **Control YAML not namespaced** — `{left,right}_franka_ros2_control.yaml` used bare
  top-level keys (`controller_manager:`, `joint_trajectory_controller:`, …), which resolve to
  `/controller_manager` etc. and never reach the `/left`–`/right` nodes, so controllers loaded
  with *"The 'type' param was not defined"*. Prefixed every node key with its namespace
  (`/left/…`, `/right/…`).

After these, both arms load mock hardware and `joint_state_broadcaster` +
`joint_trajectory_controller` go active in each namespace; `/joint_states` aggregates at
50 Hz across all 18 joints.

### 11.5 Dual-arm teleop blackboard seeding

`objectives/request_teleoperation.xml` failed immediately with
`The input port 'activate_controllers' was set to '{controllers}', which was not found`.
`DoTeleoperateAction` *outputs* `controllers` / `planning_groups` / `tip_links` /
`skip_collision_checks` / `velocity_scale_factor`, but only once the web UI sends its first
feedback. With `initial_teleop_mode=1` (joint jog) the jog branch ticks before that feedback,
reading keys that don't exist yet. (The single-arm objective dodges this only because it
defaults to mode 3, which doesn't read `{controllers}`.) Fix: seed those keys with the
right-arm defaults via `SetBlackboard` before the teleop `Parallel`; `DoTeleoperateAction`
overwrites them on UI feedback. The modes-3/4/5 `/controller_manager` limitation in §10 is
unchanged.

### Enabling mock hardware

Both configs ship in **real-hardware** mode. To run mock locally:

- **Single arm** — in `franka_arm_hw/config/config.yaml`: set `simulated: True` and uncomment
  `- use_fake_hardware: "true"` under `robot_description.urdf_params`.
- **Dual arm** — in `franka_dual_arm_hw/launch/robot_drivers_to_persist.launch.py`: set
  `use_fake_hardware` to `"true"` for both arms.

Both are marked with `MOCK HARDWARE:` comments at the exact lines.

---

## Build

```bash
git submodule update --init --recursive
moveit_pro build
moveit_pro run -c franka_arm_hw
```

## Open items / caveats

- **Dual-arm now verified in mock** (see §11): both namespaced controller_managers load mock
  hardware, controllers activate, and `/joint_states` streams at 50 Hz. Still unverified:
  end-to-end **jogging** from the web UI (the `JointJog`/`PoseJog` nodes must address the
  controller per-namespace — `right/…`), and any behavior on a real robot / sim. The
  single-arm config remains the clean, primary path.
- **Dual-arm teleop modes 3/4/5** (Move to Pose / Joint State / Interpolate) still can't work
  in the two-namespace setup — their core subtrees hardcode the un-namespaced
  `/controller_manager` (§10, §11.5). Consolidating both arms onto a single
  `controller_manager` would remove this limitation.
- **`SwitchController` activation** in `draw.xml` relies on `automatic_deactivation`
  defaulting to true; confirm the controller switch behaves as intended once running.
