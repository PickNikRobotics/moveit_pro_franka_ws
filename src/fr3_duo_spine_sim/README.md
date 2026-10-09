# fr3_duo_spine_sim

MoveIt Pro MuJoCo simulation configuration for the Franka FR3 Duo on a fixed pedestal and spine.
The scene holds the robot only: a floor and a light, no environment.

It inherits everything from `fr3_duo_spine_mock` (robot description, SRDF, MoveIt parameters, controllers, Objectives and waypoints) and changes only the ros2_control hardware to MuJoCo, through the mock's `ros2_control_xacro` xacro argument.

## Build and run

```bash
git lfs pull
git submodule update --init
moveit_pro build
moveit_pro run -c fr3_duo_spine_sim
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `fr3_duo_spine_mock` and the MuJoCo xacro arguments |
| `description/` | MuJoCo ros2_control hardware macro |
| `mjcf/scene.xml` | scene: floor, light and the `default` keyframe (spine down, both arms in the "ready" pose) |
| `mjcf/fr3_duo.xml`, `mjcf/assets/` | robot model and meshes |

`mjcf/fr3_duo.xml` is the `mobile_fr3_duo_sim` robot model without the TMR mobile base: the spine stands on the same box pedestal as the `fr3_duo_spine_mock` URDF.

## Third-party assets

The finger meshes `finger_0.obj` and `finger_1.obj` in `mjcf/assets/` come from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Apache-2.0, see [`LICENSE-mujoco_menagerie`](mjcf/assets/LICENSE-mujoco_menagerie)).
The other meshes are copied from [franka_description](https://github.com/frankarobotics/franka_description) 2.9.0 (Apache-2.0, Copyright Franka Robotics GmbH).
