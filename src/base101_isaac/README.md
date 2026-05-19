# base101_isaac

NVIDIA Isaac Sim 6.x integration for the base101. Imports the URDF,
attaches an RTX lidar + RGB camera, and wires an OmniGraph that bridges
sensors and `/cmd_vel` to the same ROS2 topics the other simulators use.

> Status: experimental. Isaac Sim 6.0 is an early-developer release.
> Expect API tweaks. See [`SIMULATORS.md`](../../SIMULATORS.md) at the
> repo root for the full install guide and known gotchas.

## Layout

```
scripts/
└── run_isaac.py            # standalone runner — bootstraps Kit via
                            # SimulationApp, owns the sim loop

launch/
└── isaac.launch.py         # ROS-side scaffolding + spawns run_isaac.py

scenes/                     # USD scenes (currently empty — runner spawns
                            # a default ground plane if scene:= is unset)

config/                     # reserved for future Isaac-specific config
```

## Quick start

```bash
ros2 launch base101_isaac isaac.launch.py                   # simple, viewport open
ros2 launch base101_isaac isaac.launch.py variant:=pro
ros2 launch base101_isaac isaac.launch.py headless:=true    # no viewport — useful for CI
ros2 launch base101_isaac isaac.launch.py scene:=/abs/path/to/world.usd
```

First launch downloads several hundred MB of extension caches on top of
the pip install — give it 5–10 minutes.

## How it fits together

```
                ┌─────────────────────────────────────┐
   ┌──────────► │  run_isaac.py  (Kit interpreter)    │
   │ subprocess │   ┌────────────────────────────┐    │
   │            │   │  isaacsim.asset.importer   │ ◄──┼── URDF (tmp)
   │            │   │  → ground plane            │    │
   │            │   │  → RTX lidar prim          │    │
   │            │   │  → USD camera prim         │    │
   │            │   │  → OmniGraph ROS2 bridge   │    │
   │            │   └────────────────────────────┘    │
   │            └─────────────────────────────────────┘
   │
   │  ┌─────────────────────────────────────┐
   ├──┤ robot_state_publisher (URDF→TF)     │
   │  └─────────────────────────────────────┘
   │
   │  ┌─────────────────────────────────────┐
   └──┤ twist_mux (joy/key/nav → cmd_vel)   │
      └─────────────────────────────────────┘
```

The runner is the only piece that runs inside Isaac's Kit interpreter.
Everything else (robot_state_publisher, twist_mux, rosboard) launches as
plain ROS2 nodes, so the topology is identical to gazebo/mujoco from the
outside.

Topic flow:

| Direction | Topic | Source |
|---|---|---|
| → robot | `/diff_drive_controller/cmd_vel` | `twist_mux` (from `/cmd_vel_joy`, `/cmd_vel_nav`, etc.) |
| ← robot | `/joint_states` | Isaac OmniGraph `ROS2PublishJointState` |
| ← robot | `/odom` | Isaac OmniGraph `IsaacComputeOdometry` + `ROS2PublishOdometry` |
| ← robot | `/tf` (odom → base_link) | Isaac (joint TFs come from robot_state_publisher) |
| ← robot | `/scan` | RTX lidar → `ROS2RtxLidarHelper` |
| ← robot | `/base_camera/image_raw` + `camera_info` | USD camera → `ROS2CameraHelper` |

## No ros2_control on the Isaac side

Unlike gazebo/mujoco, the Isaac backend does **not** load a
`controller_manager`. The `simulator:=isaac` xacro path produces a URDF
without a `<ros2_control>` block, and diff drive is handled inside the
OmniGraph:

```
ROS2SubscribeTwist  →  DifferentialController  →  IsaacArticulationController
```

`base101_control/config/controllers.*.sim.yaml` is **ignored** here. The
wheel radius / separation values are pulled from the `WHEEL_GEOMETRY`
dict at the top of `isaac.launch.py` and fed to the
`DifferentialController` node. Keep that dict in sync with the
controllers YAML if you tune diff-drive parameters.

## Launch args

| Arg | Default | Notes |
|---|---|---|
| `variant` | `simple` | `simple` or `pro`. |
| `scene` | `` (empty) | Absolute path to a USD scene. Empty → built-in ground plane + reference obstacles. |
| `headless` | `false` | Run Kit without a viewport window. Useful for CI / SSH sessions. |
| `rosboard` | `true` | Run rosboard web dashboard alongside the sim. |
| `rosboard_port` | `8888` | HTTP/WS port. |

## Editing `run_isaac.py`

The runner uses three layered Isaac APIs:

- `isaacsim.SimulationApp` — bootstraps Kit. **Must be the first import**;
  every other `isaacsim.*` / `omni.*` import has to come after the
  `SimulationApp(...)` call.
- `isaacsim.asset.importer.urdf` — programmatic URDF → USD import.
- `omni.graph.core` — building the OmniGraph that does the ROS2 bridging.

If the script crashes on import, the most common cause is the pip
install being incomplete: `isaacsim` (metapackage only) isn't enough —
you need `isaacsim[all]`.

## Gotchas

- **No controller_manager** means `ros2 control list_controllers` is
  empty under Isaac. That's expected, not a bug.
- **Conflicting joint TFs.** Isaac is configured to publish `odom →
  base_link` only; `robot_state_publisher` handles the rest from the
  URDF. If you change `tf_pub` in the OmniGraph to publish the full
  tree, drop `robot_state_publisher` from the launch or you'll have
  competing publishers.
- **OmniGraph node names drift across Isaac versions.** The runner has
  defensive `try/except` around the URDF importer and wheeled_robots
  imports to cover 5.x → 6.x renames. If your Isaac install is
  intermediate (e.g. nightly), you may need to update the
  `'isaacsim.<...>'` strings inside `_build_ros2_graph`.
- **RTX lidar requires an RTX GPU.** Without one, the lidar prim
  silently produces nothing. There's no software fallback — use the
  gazebo or mujoco backend on non-RTX machines.
