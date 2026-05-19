# base101_mujoco

MuJoCo simulation for the base101 mobile base. Scenes, launch, and a
companion node that fills the gap where `mujoco_ros2_control` doesn't ship
a laser sensor.

See [`SIMULATORS.md`](../../SIMULATORS.md) at the repo root for how this
backend compares to gazebo/isaac and the full install + dependency guide.

## Layout

```
scenes/
├── base101_simple.xml        # MJCF for simple variant (DDSM210 wheels)
└── base101_pro.xml           # MJCF for pro variant (DDSM115 wheels)

base101_mujoco/
└── lidar_bridge.py           # parallel-mjData LaserScan publisher

launch/
└── mujoco.launch.py          # mujoco_ros2_control + lidar bridge + ROS scaffolding
```

## Quick start

```bash
ros2 launch base101_mujoco mujoco.launch.py                  # simple
ros2 launch base101_mujoco mujoco.launch.py variant:=pro
ros2 launch base101_mujoco mujoco.launch.py scene:=/path/to/custom.xml
```

`scene:=` empty (the default) picks `scenes/base101_<variant>.xml`. Pass
either a basename under `scenes/` or an absolute path.

## How it fits together

```
                  ┌───────────────────────────┐
                  │  mujoco_ros2_control      │
  cmd_vel  ──►  diff_drive_controller          │  ──► joint_states
                  │  MujocoSystem hw plugin   │      odom, /tf
                  │  MJCF physics + cameras   │  ──► /base_camera/image_raw
                  │   ┌───────────────────┐   │
                  │   │ scenes/base101_*  │   │
                  │   └───────────────────┘   │
                  └─────────────▲─────────────┘
                                │ same MJCF
                  ┌─────────────┴─────────────┐
                  │  lidar_bridge (this pkg)  │  ──► /scan
                  │  parallel mjData,         │
                  │  mj_multiRay vs world     │
                  └───────────────────────────┘
```

- `mujoco_ros2_control` owns the simulation loop. It loads the MJCF,
  hosts the controller_manager from `base101_control/config/controllers.<variant>.sim.yaml`,
  and binds each ros2_control joint to the MJCF `<joint>` with the same name.
- Cameras declared in the MJCF (`<camera name="base_camera"/>`) are
  picked up automatically by upstream's `mujoco_cameras.cpp` and
  published to `/base_camera/image_raw` + depth + camera_info.
- `lidar_bridge.py` opens a read-only second copy of the same MJCF,
  subscribes to `/tf` for `lidar_frame`'s world pose, and calls
  `mj_multiRay` against the static world geoms. Robot geoms are in
  group 1 and excluded by `geomgroup=[1,0,0,0,0,0]`, so the chassis
  doesn't shadow the scan.

## Why a separate lidar bridge

`mujoco_ros2_control` doesn't yet publish range / IMU sensors (its README
lists them as future work). Rather than fork the upstream plugin we run
a tiny sibling node — it only needs the static world geoms, not the
dynamic robot state, so a read-only mjData is enough.

The bridge gets the lidar's pose from TF (not from its own mjData), so
the actual physics simulation and the ray casting are decoupled. As long
as `robot_state_publisher` + `diff_drive_controller` are publishing
`odom → base_link → lidar_frame`, the bridge works.

## MJCF scenes — what's modelled

Each scene defines:

- A flat ground plane with a checker-pattern texture (group 0 — lidar sees it).
- A handful of reference obstacles (group 0 — lidar sees them).
- The chassis as a simplified box with the variant's mass + inertia
  (group 1 — lidar excludes it).
- Four hinge-jointed wheel cylinders named `wheel_front_left`,
  `wheel_front_right`, `wheel_rear_left`, `wheel_rear_right`. These
  names are the binding contract with ros2_control — change them in
  one place and you must change them in all of:
  `base101.mujoco.ros2control`, `controllers.<variant>.sim.yaml`, and
  this MJCF.
- A `<camera name="base_camera"/>` at the front of the chassis,
  pointing along +X.
- A visual lidar mast on top (cosmetic — actual ray casting is from
  the TF position of `lidar_frame`).

The scenes are kept deliberately simple — RViz still renders the full
URDF visuals from `base101_description`, so we don't need to mirror
every bumper / box in MJCF.

## Adding a custom scene

Drop a new `.xml` under `scenes/` (any MJCF that contains the four
joint names + a `base_camera` camera will work) and run:

```bash
ros2 launch base101_mujoco mujoco.launch.py scene:=my_scene.xml
```

To add static world obstacles the lidar should see, give them
`group="0"` (or use the `class="world"` default defined in the supplied
scenes). To add chassis decoration the lidar should ignore, use
`group="1"` / `class="chassis"`.

## Launch args

| Arg | Default | Notes |
|---|---|---|
| `variant` | `simple` | `simple` or `pro`. |
| `scene` | `` (empty) | MJCF basename under `scenes/` or absolute path. Empty → `base101_<variant>.xml`. |
| `rosboard` | `true` | Run the web dashboard alongside the sim. |
| `rosboard_port` | `8888` | HTTP/WS port for rosboard. |

## Gotchas

- **Build dep on the C++ mujoco library**, not just the python package.
  Set `MUJOCO_DIR=/opt/mujoco` (or wherever you extracted it) before
  `colcon build`.
- **`mujoco_ros2_control` doesn't publish sensor data** beyond cameras
  upstream. If you add an IMU or other sensor, expect to add another
  companion node here.
- **MJCF and URDF wheel positions must agree.** The MJCF positions
  drive physics; the URDF positions (in `base101_description`) drive
  TF and visuals. If they diverge, RViz and the actual sim disagree.
