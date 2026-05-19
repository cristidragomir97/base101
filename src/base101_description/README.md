# base101_description

URDF, xacro, meshes, and RViz configuration for the base101 mobile base.

This package is the source of truth for the robot's kinematic tree. Every
other package — controllers, sim backends, navigation — pulls the same
URDF from here.

## Variants and simulators

A single top-level xacro (`urdf/base101.xacro`) drives two orthogonal
selectors:

| Arg | Values | What it picks |
|---|---|---|
| `variant` | `simple`, `pro` | Which hardware build (DDSM210 vs DDSM115 hubs). Loads `base101_<variant>.xacro` — different wheel meshes, motor masses, and track widths, same chassis. |
| `simulator` | `gazebo`, `mujoco`, `isaac`, `none` | Which downstream uses the URDF. Selects the ros2_control hardware plugin and any sim-specific URDF extensions. |

Example processings:

```bash
xacro urdf/base101.xacro variant:=simple simulator:=gazebo    # full Gazebo URDF
xacro urdf/base101.xacro variant:=pro    simulator:=mujoco    # ros2_control with MujocoSystem
xacro urdf/base101.xacro variant:=simple simulator:=isaac     # no ros2_control block (Isaac drives joints natively)
xacro urdf/base101.xacro variant:=simple simulator:=none      # bare URDF, no plugins (for rviz, real hardware)
```

The launches in `base101_gazebo`, `base101_mujoco`, `base101_isaac`, and
`base101_control` each pass the right combination automatically. You
rarely need to invoke xacro by hand outside of `display.launch.py`.

## Files

```
urdf/
├── base101.xacro                   # top-level: variant + simulator selectors
├── base101_simple.xacro            # simple-variant link/joint tree
├── base101_pro.xacro               # pro-variant link/joint tree
├── base101_simple.gazebo           # gazebo-only extensions for simple (sensors, friction)
├── base101_pro.gazebo              # ditto for pro
├── base101.gazebo.ros2control      # ros2_control block w/ gz_ros2_control plugin
├── base101.mujoco.ros2control      # ros2_control block w/ mujoco_ros2_control plugin
└── materials.xacro                 # shared material definitions

meshes/
├── simple/                         # STLs for the simple variant
└── pro/                            # STLs for the pro variant

config/
└── display.rviz                    # RViz config used by display.launch.py
```

There is no per-simulator URDF for Isaac — `simulator:=isaac` produces a
plain URDF, and Isaac's OmniGraph publishes joint states + odom + TF
directly (see `base101_isaac`).

## Quick start: visualise without a sim

```bash
ros2 launch base101_description display.launch.py variant:=pro
```

That brings up `robot_state_publisher` + `joint_state_publisher_gui` + RViz
on the bare URDF (`simulator:=none`), so you can sweep wheel joints and
inspect the kinematic tree without any physics.

## Things to remember when editing the URDF

- Joint names are the binding contract with every other package. The
  four wheels are exactly `wheel_front_left`, `wheel_front_right`,
  `wheel_rear_left`, `wheel_rear_right`. If you rename one, the
  `controllers.*.sim.yaml`, `base101.*.ros2control`, MJCF scenes, and
  Isaac OmniGraph all need the same change.
- `<gazebo>` extensions are gated by `simulator=gazebo` in
  `base101.xacro`, so adding them won't affect MuJoCo / Isaac builds.
  Don't put cross-sim configuration in there.
- Wheel separation / radius live in `base101_control/config/controllers.*.sim.yaml`
  for diff drive. The MJCF scenes (`base101_mujoco/scenes/`) and the
  Isaac launch (`WHEEL_GEOMETRY` dict in `isaac.launch.py`) duplicate
  those numbers — keep all three in sync.
