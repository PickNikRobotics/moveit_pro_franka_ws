# fr3_solo_sim

MoveIt Pro MuJoCo simulation configuration for one Franka Research 3 (FR3) arm with a Franka Hand.
The scene holds the robot only: a floor and a light, no environment.

It inherits everything from `fr3_solo_mock` (robot description, SRDF, MoveIt parameters, controllers, Objectives and waypoints) and changes only the ros2_control hardware to MuJoCo, through the `hardware_interface` xacro argument.

## Build and run

```bash
git submodule update --init
moveit_pro build
moveit_pro run -c fr3_solo_sim
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `fr3_solo_mock`, MuJoCo xacro arguments, MuJoCo Behaviors and Objectives |
| `description/` | MuJoCo ros2_control hardware macro |
| `mjcf/scene.xml` | scene: floor, light and the `default` keyframe (the "ready" arm pose) |
| `mjcf/fr3_solo.xml`, `mjcf/assets/` | FR3 and Franka Hand model and meshes |

## Third-party assets

The MuJoCo model and meshes in `mjcf/` are derived from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) `franka_fr3` model and its Franka Hand (Apache-2.0, see [`mjcf/LICENSE`](mjcf/LICENSE)), as modified for the MoveIt Pro example workspace. Here the arm base sits on the floor.
