# Project agent memory

MoveIt Pro 10.1.0+ (ROS 2 Jazzy) config workspace for Franka robots. `README.md` lists the packages.

## Layout and inheritance

- Each robot has `src/<robot>_mock` (mock hardware) and `src/<robot>_sim` (MuJoCo). The sim config sets `based_on_package: <robot>_mock` and, through `urdf_params`, points the mock URDF's `ros2_control_xacro` argument at its own MuJoCo `ros2_control` file, which declares the MuJoCo arguments. A mock never refers to its sim. Change the description in the mock package only. `fr3_solo_sim`, `fr3_duo_sim` and `fr3_duo_spine_sim` override nothing else; `mobile_fr3_duo_sim` also replaces the controllers and layers AMCL and laser obstacle layers on the mock's Nav2 (`params_overlay` of the mock's `launch/navigation.launch.py`).
- Mobile FR3 Duo frames are `world` -> `map` (fixed) -> `odom` -> planar joints -> `base_link`, with the URDF rooted at `odom` (no option to root it elsewhere). In `mobile_fr3_duo_sim` the odometry bridge is the only publisher of the planar joints, and `joint_state_broadcaster` lists every MJCF joint except them: add any new MJCF joint to that list, or its state never reaches `/joint_states`.
- Mobile FR3 Duo navigation Objectives first run the subtree "Stow Arms for Navigation" (`mobile_fr3_duo_mock`). The Nav2 footprint in the mock's `params/nav2_params.yaml` covers the robot in that Stow pose, and the `inflation_radius` of both costmaps must reach the footprint's farthest point from `base_link` (a smaller radius makes MPPI check the full footprint everywhere and lets the planner route too close to walls): change the three together.
- `mobile_fr3_duo_mock/objectives/request_teleoperation.xml` replaces MoveIt Pro's core "Request Teleoperation" (same file name, leaf wins) so the base iMarker drives with Nav2. It is a copy of the 10.1.0 core file: when MoveIt Pro's core version changes, re-copy it and re-apply the `mobile_base` branch.
- There are no `_hw` packages yet (issue #10). `src/franka_behaviors` builds the spine Behaviors.

## Conventions

- Controller names: a robot's whole-body group keeps the standard names (`joint_trajectory_admittance_controller`, `joint_trajectory_controller`, `velocity_force_controller`, `joint_velocity_controller`), because MoveIt Pro defaults and UI lookups use them. Partial groups use `<prefix>_<acronym>`: `jtac`, `jtc`, `vfc`, `jvc`, `jgvc` (JointGroupVelocityController). `left_`/`right_` is one arm, `_w_spine` adds the spine, `spine_` and `base_` name those groups. Grippers are `gripper_controller` or `left_`/`right_gripper_controller`; broadcasters keep their standard names.
- Meshes, images and the Nav2 map are stored in Git LFS, by the rules in `.gitattributes`. Install git-lfs (`git lfs install`) before you clone: without it, a checkout holds only pointer files that MuJoCo and Nav2 cannot load, and a new mesh is committed as a plain file.
- Every MuJoCo keyframe `qpos` must have exactly `nq` values. Give `ctrl` too, matching `qpos` for position actuators, or a reset drives those joints to 0. Every Objective needs a `MetadataFields` block. Never put `--` inside an XML comment.

## Known issues

- Nav2 Jazzy (the 1.3.x in the MoveIt Pro image) can leave the base driving after a failed navigation. If the controller server acknowledges a new path-following goal later than the navigator's `default_server_timeout`, the navigator drops that goal without cancelling it and aborts the navigation. The controller keeps driving the base along the old path, with no Objective running, until the next navigation goal replaces it. Heavy CPU load makes it more likely. Upstream: Nav2 issue #6370, fixed on Nav2 main (#6373, #6445), not on Jazzy. The larger `default_server_timeout` in `mobile_fr3_duo_mock/params/nav2_params.yaml` makes it rarer but does not remove it; the remaining mitigation would be a node that cancels a `follow_path` goal still executing while no `navigate_to_pose` goal is active.

## Checking changes without a robot

CI (`.github/workflows/ci.yaml`) calls the shared `moveit_pro_ci` workspace integration test, which builds and runs `colcon test` in `picknikciuser/moveit-pro:10.1.0-jazzy`. The Format job runs the hooks in `.pre-commit-config.yaml`, with `.prettierrc.cjs` for XML and `.clang-format` for C++: run `pre-commit run -a` before you push.

Build and load every config inside a MoveIt Pro image. Mount the workspace and the franka_description checkout. Run `colcon build` with an out-of-tree `--build-base`/`--install-base`, then construct `moveit_studio_utils_py.system_config.SystemConfigParser()` with `MOVEIT_CONFIG_PACKAGE=<package>` and `USER_WS` set. This runs the real config merge, schema checks and URDF/SRDF xacro. Source `/opt/overlay_ws/install/setup.bash` and use `--entrypoint bash`.

## Maintaining this file

Keep this file for knowledge useful to almost every future agent session in this project.
Do not repeat what the codebase already shows; point to the authoritative file or command instead.
Prefer rewriting or pruning existing entries over appending new ones.
When updating this file, preserve this bar for all agents and keep entries concise.
