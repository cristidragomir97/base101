# base101_worlds

Sim-common **assets** for base101. No launch files, no nodes — just the
things a sim bringup reads.

```
worlds/empty.sdf          bare ground plane + physics/sensor systems
worlds/sticky_floor.sdf   default world: high-friction floor, gz IMU system
config/gz_ros_bridge.yaml ros_gz_bridge topic map (lidar, cameras, IMU, odom)
config/gazebo.rviz        RViz preset for looking at the sim
```

The launch that consumes all of this is
`base101_bringup_gazebo/launch/sim.launch.py`:

```bash
ros2 launch base101_bringup_gazebo sim.launch.py
ros2 launch base101_bringup_gazebo sim.launch.py world:=empty.sdf
ros2 launch base101_bringup_gazebo sim.launch.py arm:=true
```

`world:=` takes either a bare filename from `worlds/` or an absolute path.

Renamed from `base101_gazebo` in the 2026-08 bringup restructure — the old
name sat one letter away from `base101_bringup_gazebo` and the two were
constantly confused. See `docs/bringup-restructure.md`.
