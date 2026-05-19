# base101_gazebo

Gazebo Sim integration for base101. Worlds, the launch that spawns the
robot + ros2_control, and the ROS↔Gazebo bridges for sensors and clock.

Gazebo is the default and most production-tested of the three simulators
this repo supports. See [`SIMULATORS.md`](../../SIMULATORS.md) for how the
gazebo / mujoco / isaac backends compare and how they share scaffolding.

## Layout

```
launch/
└── gazebo.launch.py          # main entry point — spawns robot, runs sim,
                              # bridges sensors, loads controllers

worlds/
├── empty.sdf                 # flat ground + 2 reference boxes
└── sticky_floor.sdf          # apartment scene (15x12 m), Fuel-sourced
                              # furniture, ODE physics tuned for skid steer

config/
├── gz_ros_bridge.yaml        # ros_gz_bridge topic map for /scan, camera_info
└── gazebo.rviz               # RViz preset for the in-sim TF + scan + camera
```

## Quick start

```bash
ros2 launch base101_gazebo gazebo.launch.py                              # simple variant, sticky_floor
ros2 launch base101_gazebo gazebo.launch.py variant:=pro world:=empty.sdf
ros2 launch base101_gazebo gazebo.launch.py rosboard:=false              # quieter, no web dashboard
```

`world:=` accepts either a basename under `worlds/` or an absolute path.

## What the launch does

1. Process the URDF via xacro with `simulator:=gazebo`. That pulls in
   `base101.gazebo.ros2control` (which selects the
   `gz_ros2_control/GazeboSimSystem` plugin) and `base101_<variant>.gazebo`
   (sensor + friction definitions per link).
2. Start `robot_state_publisher` with the processed URDF.
3. Launch Gazebo Sim with the chosen `.sdf` world.
4. Spawn the robot at `z=0.10` via `ros_gz_sim create`.
5. Spin up the ROS↔gz bridges:
   - `/clock` (so `use_sim_time:=true` actually works downstream)
   - `/scan` + `/base_camera/camera_info` (config in `gz_ros_bridge.yaml`)
   - `/base_camera/image_raw` (dedicated `ros_gz_image` bridge)
6. Once the robot model is spawned, `gz_ros2_control` instantiates the
   `controller_manager` inside the sim plugin. The launch then sequences
   `joint_state_broadcaster` followed by `diff_drive_controller` via
   `OnProcessExit` event handlers.
7. Run `twist_mux` (config from `base101_control`) with output remapped to
   `/diff_drive_controller/cmd_vel`.
8. Optionally start `rosboard` for the browser dashboard + teleop card.

## Gotchas

- **Physics engine.** `sticky_floor.sdf` pins ODE. Bullet-Featherstone
  ignores the anisotropic `<mu2>/<fdir1>` friction needed for 4-wheel
  skid steer; switching engines silently breaks turning.
- **Always pass `use_sim_time:=true`** to Nav2 / SLAM / RViz when running
  against this sim. Wall-clock mismatch causes silent TF extrapolation
  failures.
- **Fuel models.** `sticky_floor.sdf` references models from
  https://fuel.gazebosim.org. First launch downloads them — a minute or
  two on a slow connection. Subsequent launches use the local cache.
- **`GZ_SIM_RESOURCE_PATH`.** The launch sets this to include
  `base101_description` and `base101_control` shares so `package://`
  mesh URIs resolve. If you add a new package with meshes Gazebo needs,
  add it to the `resource_dirs` list at the top of `_setup()`.
- **Spawn z-offset.** The robot spawns at `z=0.10` to give wheels room
  to settle. Tweak `-z` in `spawn_robot` if you change wheel size.

## Launch args

| Arg | Default | Notes |
|---|---|---|
| `variant` | `simple` | `simple` or `pro`. |
| `world` | `sticky_floor.sdf` | Basename under `worlds/` or absolute path. |
| `rosboard` | `true` | Run the web dashboard + teleop card alongside the sim. |
| `rosboard_port` | `8888` | HTTP/WS port. |
