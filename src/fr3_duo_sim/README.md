# fr3_duo_sim

MoveIt Pro MuJoCo simulation configuration for the Franka FR3 Duo without a spine: the FR3 Duo mount on a fixed base, two FR3 arms and two Franka Hands.
The scene holds the robot only: a floor and a light, no environment.

It inherits everything from `fr3_duo_mock` (robot description, SRDF, MoveIt parameters, controllers, Objectives and waypoints) and changes only the ros2_control hardware to MuJoCo, through the mock's `ros2_control_xacro` xacro argument.

## Build and run

```bash
git lfs pull
git submodule update --init
moveit_pro build
moveit_pro run -c fr3_duo_sim
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `fr3_duo_mock` and the MuJoCo xacro arguments |
| `description/` | MuJoCo ros2_control hardware macro |
| `mjcf/scene.xml` | scene: floor, light and the `default` keyframe (both arms in the "ready" pose) |
| `mjcf/fr3_duo.xml`, `mjcf/assets/` | robot model and meshes |

## Third-party assets

`mjcf/fr3_duo.xml` and the arm and hand meshes in `mjcf/assets/` are derived from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) Franka FR3 and Franka Hand models (Apache-2.0, see [`mjcf/LICENSE`](mjcf/LICENSE)).
The mount meshes `fr3_duo_mount.stl` and `fr3_duo_cover.stl` are copied from [franka_description](https://github.com/frankarobotics/franka_description) 2.9.0 (Apache-2.0, Copyright Franka Robotics GmbH).
