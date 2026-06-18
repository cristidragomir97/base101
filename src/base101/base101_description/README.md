# base101_description

The **shared chassis library** for the base101 robot: the common base —
wheels, motor supports, bumpers, lidar, base camera, the `top_plate_1` deck —
plus materials, sensors, and meshes.

This package is a *parts library, not a launchable robot*. It exposes
`chassis.xacro` (a link/joint fragment) that every variant includes; you never
load it directly. The variant packages assemble it into complete robots:

| Variant | Package | Adds |
|---|---|---|
| simple | `base101_simple_description` | nothing (bare chassis) |
| arm    | `base101_arm_description`    | hex deck + 1 mod101 arm |
| tower  | `base101_tower_description`  | lift column + pan/tilt head + optional 2 arms |

Each variant `xacro:include`s `chassis.xacro` + `materials.xacro`, picks the
sim wiring for its `simulator` arg, and adds its own links. See the top-level
[README](../../../README.md) for the package-structure graph.

## Files

```
urdf/
├── chassis.xacro                 # chassis link/joint tree (the shared fragment)
├── chassis.gazebo                # gazebo-only extensions: sensors (imu/lidar/camera),
│                                 #   materials, wheel friction — NO gz_ros2_control plugin
├── chassis.gazebo.ros2control    # wheel ros2_control system (gz_ros2_control/GazeboSimSystem)
├── chassis.mujoco.ros2control    # wheel ros2_control system (mujoco_ros2_control/MujocoSystem)
└── materials.xacro               # shared material definitions

meshes/                           # chassis STLs (flat — one variant's worth: simple == base)
config/
└── display.rviz                  # RViz preset reused by the variants' display launches
```

The variant top-level xacros own the per-simulator dispatch and the
`gz_ros2_control` plugin block (which points at that variant's
`controllers.sim.yaml`), so this package carries no controller-file reference.

## Things to remember when editing the chassis

- **Joint names are the binding contract** with every other package. The four
  wheels are exactly `front_left_wheel_joint`, `front_right_wheel_joint`,
  `back_left_wheel_joint`, `back_right_wheel_joint`. Renaming one ripples into
  every variant's `controllers.sim.yaml` and `chassis.*.ros2control`.
- **`base_link` is recentred on the wheel-contact centroid** (the raw CAD
  export was off-centre): the chassis is shifted so the wheels sit symmetrically
  about the origin, which diff_drive and the nav footprint assume.
- `<gazebo>` extensions in `chassis.gazebo` are only included on the gazebo
  branch of each variant xacro, so they don't affect the `none` (rviz / real
  hardware) build.
- Wheel separation / radius live in each variant's `controllers.sim.yaml` —
  keep them in sync with the URDF geometry.
