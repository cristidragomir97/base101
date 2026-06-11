# base101

<p align="center">
  <img src="img/simple.webp" alt="simple variant" width="48%" />
  <img src="img/pro_mirrored.webp" alt="pro variant" width="48%" />
</p>

An open-source 4WD mobile robot platform designed to carry the mod101 arm and other payloads. Built from 60×20 aluminum extrusion, PLA-CF printed parts, and Waveshare DDSM hub motors.

Two configurations, same chassis, different motors.

## At a Glance

| | base101 | base101 PRO |
|---|---|---|
| Motors | 4× DDSM210 | 4× DDSM115 |
| Drive | 4WD skid steer | 4WD skid steer |
| Torque (per motor) | 0.85 N·m stall | 2.0 N·m stall |
| Load capacity | ~12kg total | ~30kg total |
| Suspension | PLA-CF printed | PLA-CF printed |
| Footprint | 280 × 400mm | 280 × 400mm |
| Lidar | RPLidar C1 | RPLidar C1 |
| Depth camera | RealSense D435 | RealSense D435 |
| BOM | ~$340 | ~$480 |

Both configurations share the same chassis, top plate, bumpers, electronics, and software. The upgrade from base to PRO is a motor and bracket swap.


## Chassis

**Frame:** 60×20 aluminum extrusion, black anodized. Two parallel rails connected at the corners by PLA-CF printed mounts. The thin profile keeps the center of gravity low — this is a MacBook Air, not a brick.

**Top plate:** 280×400mm aluminum, 3mm thick, CNC-machined grid of M3 tapped holes on 20mm spacing. Mount anything anywhere — the arm, the lidar, the compute, accessories. No adapters, no T-nuts, just bolt it down.

**Bumpers:** TPU printed corner pieces. Absorb collisions, protect furniture, and visually soften the rectangular chassis. Press-fit onto the extrusion corners, no fasteners needed.


## Drivetrain

**4WD skid steer.** Four hub motors, all driven. Turn-in-place capability. No mecanum wheels, no omni wheels — just rubber tires on smooth direct-drive motors. Quiet, clean, simple kinematics.


### DDSM210 (base101)

- 0.25 N·m rated / 0.85 N·m stall
- ~65mm diameter, 216g
- UART bus, 9-28V
- ~$25 each

Sized for a 5-8kg robot. Four wheels provide 97 N total tractive force at stall — enough to move 8kg up a 10% incline.

### DDSM115 (base101 PRO)

- 0.96 N·m rated / 2.0 N·m stall
- ~115mm diameter, 765g
- RS485 bus, 12-24V
- ~$60 each


## Sensors

### RPLidar C1

Mounted on the top plate. 360° DTOF scanning, 12m range, 5000 samples/sec. Handles SLAM, mapping, and obstacle detection. ROS2 driver available out of the box.

### Intel RealSense D435

Front-mounted between the extrusion rails. 87° wide FOV for spatial awareness and 3D perception. Provides point cloud data for obstacle avoidance and workspace mapping. The wide field of view captures the arm's entire workspace in front of the robot.


## Software

ROS 2 Jazzy workspace. The `diff_drive_controller` handles skid-steer kinematics for both DDSM210 and DDSM115 configurations — the only parameter change is wheel diameter and separation.

### Packages

| Package | Type | Purpose |
|---|---|---|
| `base101_description` | ament_python | Unified URDF (simple/pro + simulator selector via xacro args), meshes, RViz config. |
| `base101_control` | ament_cmake | `diff_drive_controller` + `twist_mux` config, hardware bringup launch. |
| `base101_gazebo` | ament_cmake | Gazebo Sim worlds, launch, ros↔gz bridge. |
| `base101_mujoco` | ament_python | MuJoCo scenes, `mujoco_ros2_control` launch, companion lidar ray-cast bridge. |
| `base101_isaac` | ament_python | NVIDIA Isaac Sim runner + launch. Imports the URDF, wires the OmniGraph ROS2 bridge. |
| `base101_nav` | ament_cmake | Nav2 + slam_toolbox + frontier exploration. Configs, launches, RViz preset. |
| `base101_mcp` | ament_python | Generic ROS2 ↔ MCP (Model Context Protocol) bridge. Lets Claude (or any MCP client) discover topics/services and read/publish messages over natural language. Requires `pip install "fastmcp>=2,<3"`. |
| `base101_teleop` | ament_python | Standalone single-page web teleop (base + every joint) on `:8700`. Fallback for the rosboard Joint sliders card. |
| `rosboard` | ament_python | Vendored web dashboard. Carries two publisher cards: **Teleop** (Twist) and **Joint sliders** (Float64MultiArray position commands for tower + arms). |

### Quickstart

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch base101_gazebo gazebo.launch.py        # simple variant, sticky_floor world
ros2 launch base101_gazebo gazebo.launch.py variant:=pro world:=empty.sdf
```

Web teleop is at `http://localhost:8888/` (rosboard) once the sim is up.

### Cross tower (optional)

An optional vertical tower (merged from the former `base101_cross_description`
CAD export) bolts onto the top plate: a column with a prismatic **lift**
carrying a crossbeam with two arm-mount brackets, and a **pan/tilt head**
with a camera on top. Enable it with `tower:=true` — it works on both
variants and in RViz, Gazebo, and the hardware overlay:

```bash
ros2 launch base101_gazebo gazebo.launch.py tower:=true
ros2 launch base101_description display.launch.py tower:=true
```

| Joint | Type | Range | Notes |
|---|---|---|---|
| `lift` | prismatic | ±0.26 m | Axis points **down**: `+0.26` = bottom of stroke, `-0.26` = top, `0` = mid-travel. Effort limit 400 N — sized for the fully loaded carriage (crossbeam + brackets + two arms). |
| `head_pan` | continuous | — | `+` = CCW/left (REP-103 yaw). Axis re-anchored on the pan motor shaft (the CAD export had it ~15 cm off). |
| `head_tilt` | continuous | — | `+y` axis. |

All three are position-controlled by **`tower_controller`**
(`position_controllers/JointGroupPositionController`, spawned automatically
when `tower:=true`):

```bash
ros2 topic pub /tower_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0]}"   # [lift, head_pan, head_tilt]
```

The head camera publishes `/head_camera/image_raw` (bridged like the base
camera). The tower URDF lives in `base101_description/urdf/base101_tower.xacro`
(+ `.gazebo`), meshes in `meshes/tower/`; the attachment point is variant-aware
because the simple and pro CAD exports use different top-plate frames. The
`left_arm_bracket_1` / `right_arm_bracket_1` links on the crossbeam are the
mount points for two mod101 arms — see the next section. Full merge/debug
notes: [`docs/worklogs/tower.md`](docs/worklogs/tower.md).

### Dual mod101 arms (optional, `arms:=true`)

Two [mod101](https://github.com/robocore-dev/mod101) arms mount on the
tower's crossbeam brackets. The mod101 repo stays standalone — its arm is a
prefix-parameterised xacro macro (`mod101_arm`, see
`mod101_description/urdf/mod101_macro.xacro`) that this repo instantiates
twice in `base101_description/urdf/base101_arms.xacro`, producing joints
`left_arm_1…6` and `right_arm_1…6`.

**Build** (mod101 first, then this workspace as an overlay — only needed
when you actually use `arms:=true`):

```bash
cd ~/Work/mod101  && colcon build --symlink-install && source install/setup.bash
cd ~/Work/base101
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

(The `Python3_EXECUTABLE` pin guards against a stray non-system python on
`PATH` breaking ament's package.xml parsing and `rosidl`'s `em` import. The
`mod101_description` exec_depend in `base101_description` is not a rosdep
key — pass `--skip-keys mod101_description` to `rosdep install` if you use
it.)

**Launch:**

```bash
ros2 launch base101_gazebo gazebo.launch.py tower:=true arms:=true            # jaws grippers
ros2 launch base101_gazebo gazebo.launch.py tower:=true arms:=true arm_tool:=parallel
ros2 launch base101_description display.launch.py tower:=true arms:=true     # rviz only
```

**Controllers** (all `position_controllers/JointGroupPositionController`,
commands are `std_msgs/Float64MultiArray` on `/<name>/commands`):

| Controller | Joints |
|---|---|
| `tower_controller` | `lift, head_pan, head_tilt` |
| `left_arm_controller` / `right_arm_controller` | `<side>_arm_1 … _5` |
| `left_gripper_controller` / `right_gripper_controller` | `<side>_arm_6` |
| `diff_drive_controller` | wheels (via `twist_mux`) |

Each arm's wrist camera is bridged as `/<side>_arm_wrist_camera/image_raw`.
Integration notes (mount-point measurement, the lift-effort and arm-yaw
fixes, the stale-`robot_state_publisher` gotcha):
[`docs/worklogs/dual_arm.md`](docs/worklogs/dual_arm.md).

### Teleop

Two ways to drive everything from a browser:

- **rosboard "Joint sliders" card** (recommended): open rosboard
  (`http://localhost:8888/`), pick **Joint sliders** in the System nav. One
  panel with a position slider for every controlled joint — lift, pan/tilt
  head, both arms, both grippers — initialised from the live robot pose,
  with live position readouts and a re-sync button. Groups whose hardware
  isn't loaded hide automatically; the card persists across reloads. Base
  driving stays on the separate **Teleop** card. (Backed by rosboard's
  `MSG_PUB` channel; `std_msgs/Float64MultiArray` is allowlisted and —
  unlike Twist — deliberately *not* zeroed by the publish watchdog, so
  position commands hold when the browser goes quiet.)
- **`base101_teleop`** — a zero-dependency standalone fallback:
  `ros2 run base101_teleop server` → `http://localhost:8700/`. Same sliders
  plus a hold-to-drive base pad. See
  [`src/base101_teleop/README.md`](src/base101_teleop/README.md).

### Simulators

The same robot URDF runs in three simulators. A `simulator` xacro arg
(`gazebo|mujoco|isaac|none`) selects the ros2_control hardware plugin and
any sim-specific URDF extensions. All three publish the same
`/cmd_vel_*`, `/odom`, `/scan`, `/joint_states`, and
`/base_camera/image_raw` topics, so Nav2/SLAM/exploration are oblivious to
which one is running.

```bash
ros2 launch base101_gazebo gazebo.launch.py variant:=simple  # default, recommended
ros2 launch base101_mujoco mujoco.launch.py variant:=simple
ros2 launch base101_isaac  isaac.launch.py  variant:=simple
```

| | Gazebo | MuJoCo | Isaac Sim |
|---|---|---|---|
| Physics | DART / Bullet-Featherstone | MuJoCo (Featherstone) | PhysX 5 |
| World format | SDF | MJCF | USD |
| ros2_control plugin | `gz_ros2_control/GazeboSimSystem` | `mujoco_ros2_control/MujocoSystem` | None — OmniGraph drives joints natively |
| Lidar | `<gpu_lidar>` + `ros_gz_bridge` | `base101_mujoco` companion node (parallel mjData + `mj_multiRay`) | RTX lidar + `ROS2RtxLidarHelper` |
| Camera | Native + `ros_gz_image` | `mujoco_cameras.cpp` (built into `mujoco_ros2_control`) | USD camera + `ROS2CameraHelper` |
| Maturity | Production | Joints + cameras production; lidar via companion node | Experimental — Isaac 6.x early-dev |

**Install + dependencies.** See [`SIMULATORS.md`](SIMULATORS.md) for the
full setup guide for each backend (apt packages, pip installs, build
flags, common gotchas). Per-package details and the rationale behind
each integration live in the package READMEs: [`base101_gazebo`](src/base101_gazebo/README.md),
[`base101_mujoco`](src/base101_mujoco/README.md), [`base101_isaac`](src/base101_isaac/README.md).

### Navigation, SLAM, and Exploration

`base101_nav` ships a full Nav2 stack tuned for the base101 differential drive: SmacPlanner2D for global planning, Regulated Pure Pursuit for local control, slam_toolbox for online mapping, and `explore_lite` (vendored via `base101.repos`) for autonomous frontier exploration. Maps go in `~/.base101/maps/`. An RViz preset with Map, costmaps, paths, and the Nav2 goal panel is at `base101_nav/config/nav.rviz`.

```bash
# Drive around manually and build a map
ros2 launch base101_nav mapping.launch.py use_sim_time:=true
# (save with: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/$USER/.base101/maps/home'}}")

# Navigate inside a saved map
ros2 launch base101_nav navigation.launch.py map:=$HOME/.base101/maps/home.yaml use_sim_time:=true

# SLAM + Nav2 at the same time, with optional autonomous exploration
ros2 launch base101_nav slam_nav.launch.py use_sim_time:=true explore:=true

# RViz with everything preconfigured
ros2 launch base101_nav rviz.launch.py use_sim_time:=true
```

Pass `use_sim_time:=true` on every launch when running against Gazebo (sim/wall-clock mismatch causes silent TF extrapolation failures otherwise). Outputs are remapped through `velocity_smoother → /cmd_vel_nav → twist_mux → /diff_drive_controller/cmd_vel`, so joystick/keyboard inputs still preempt nav at their existing higher priorities.

Runtime deps not in apt: `explore_lite` is pulled in via `vcs import src < base101.repos`. Everything else (`nav2_*`, `slam_toolbox`, `robot_localization`) is `ros-jazzy-*` packages.



## Project Status

- [x] Chassis design (Fusion 360)
- [x] Render and proportioning
- [x] Motor selection and analysis
- [x] Suspension design (PLA-CF adaptation)
- [x] URDF/xacro
- [x] Gazebo simulation
- [x] MuJoCo simulation
- [x] Isaac Sim simulation
- [x] ros2_control integration
- [x] Nav2 + SLAM + frontier exploration
- [x] Cross tower (prismatic lift + pan/tilt head, `tower:=true`)
- [x] Combined base101 + dual mod101 system launch (`arms:=true`)
- [x] Web teleop for all joints (rosboard Joint sliders + `base101_teleop`)
- [ ] E-stop handle mechanism
- [ ] CNC top plate manufacturing files (DXF)

## Related Projects

- **[mod101](https://github.com/robocore-dev/mod101)** — 5+1 DOF modular robot arm
- **[Axon](https://github.com/robocore-dev/axon)** — Multi-protocol controller board
- **[Forge](https://github.com/robocore-dev/forge)** — ROS2 deployment orchestration

## License

MIT
