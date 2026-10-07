# mobile_fr3_duo_sim

MoveIt Pro MuJoCo simulation configuration for the Franka Mobile FR3 Duo, with front and rear lidars, an IMU and base cameras.

The scene is a 12 m by 12 m room with a narrow passage, obstacles, a table and two scene cameras. Nav2 runs on a map of that room (`maps/room.yaml`, made with slam_toolbox), and the "Navigate to Clicked Point" Objective drives the base there.

It inherits everything from `mobile_fr3_duo_mock` (robot description, SRDF, MoveIt parameters, Objectives and waypoints) and replaces only the hardware and the controllers.

## Build and run

```bash
git submodule update --init
moveit_pro build
moveit_pro run -c mobile_fr3_duo_sim
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `mobile_fr3_duo_mock`, MuJoCo hardware parameters, Nav2 Behaviors |
| `config/control/` | ros2_control controllers for the simulation |
| `description/` | MuJoCo ros2_control hardware macro |
| `mjcf/` | MuJoCo scene and robot model |
| `mjcf/assets/` | meshes and the conversion input URDF |
| `launch/`, `script/` | lidar flattening, scan merging, odometry and `cmd_vel` bridges, Nav2 bring-up |
| `maps/`, `params/` | room map, Nav2 and slam_toolbox parameters |
| `objectives/` | Navigate to Clicked Point |

## Third-party assets

The finger meshes `finger_0.obj` and `finger_1.obj` in `mjcf/assets/` come from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Apache-2.0, see [`LICENSE-mujoco_menagerie`](mjcf/assets/LICENSE-mujoco_menagerie)).
The other meshes are copied or converted from [franka_description](https://github.com/frankarobotics/franka_description) 2.9.0 (Apache-2.0, Copyright Franka Robotics GmbH).
