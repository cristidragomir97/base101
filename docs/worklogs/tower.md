# Cross tower worklog

> **Note (post-consolidation):** the tower now lives in its own package,
> `base101_dual_arm`. Paths below that say `base101_description/urdf/
> base101_tower.xacro` are now `base101_dual_arm/urdf/tower.xacro`
> (+ `tower.gazebo`, `tower.ros2control`), and the meshes are under
> `base101_dual_arm/meshes/tower/`. The geometry/debugging notes still apply.

Notes from merging `base101_cross_description` (a standalone Fusion CAD
export of the simple chassis + tower) into the base101 workspace, plus the
geometry-debugging passes that followed. Current state is whatever's in
`base101_dual_arm/urdf/tower.xacro` at HEAD.

## Origin

`base101_cross_description` was a fresh CAD export containing the **whole
robot**: a duplicate of the simple chassis plus the new tower assembly
(column, lift carriage, crossbeam, arm brackets, camera support, pan/tilt
head). Its `.ros2control` file was empty and its `.gazebo` file was
materials-only, so the only genuinely new content was 13 tower links/joints
and their meshes. The chassis duplicate was dropped; the tower was extracted
into `base101_description/urdf/base101_tower.xacro` with link/joint names and
origins kept verbatim from the export (meshes → `meshes/tower/`).

## Variant-aware attachment

The export shares its `core_top_plate_v1_1` frame with the **simple**
variant's export, so the tower's attachment joint (`Rigid 79`) transfers
verbatim for simple. The **pro** export's plate frame sits at the *opposite
corner* of the same physical plate (offset `(0.34, -0.24, 0)` — derived by
comparing the lidar-mount and bumper joint origins across exports), so
`base101_tower.xacro` picks the mount origin per variant:

- simple: `-0.17 0.12 0.003`
- pro: `0.17 -0.12 0.003`

Both put the column at the plate centre (verified numerically via FK).

## Integration points

- `tower` xacro arg in `base101.xacro` (default `false`), threaded through
  `gazebo.launch.py`, `display.launch.py`, `control_stack.launch.py`, and
  the hardware overlay.
- Tower joints (`lift`, `head_pan`, `head_tilt`) added to all three
  ros2_control files behind `<xacro:if value="$(arg tower)">`; the mujoco
  block needs matching joints in the scene XML before it's usable there.
- `tower_controller` (`position_controllers/JointGroupPositionController`,
  joints `[lift, head_pan, head_tilt]`) added to all controller YAMLs and
  spawned conditionally from the launches.
- Head camera sensor on `head_camera_1` in `base101_tower.gazebo` (same
  camera model as `base_camera`) + a `ros_gz_image` bridge in the launch.
- `self_collide` is **off** on the lift carriage chain — the mount/connector
  slide along (and overlap) the column, and resolving those contacts makes
  the prismatic joint fight the physics engine.

## Bugs found & fixed after the merge

### Lift limits and effort

The CAD export's limits (`0.03 … 0.52`, velocity 100 m/s) were wrong. Real
travel is ±0.26 m around mid-stroke; the axis points **down**, so `+0.26` is
the bottom. Velocity clamped to 0.5 m/s for sim stability. The effort limit
was raised 100 → 400 N after the dual-arm integration: the loaded carriage
(crossbeam 3.7 kg + brackets 2.6 kg + two arms & tools ≈ 4 kg ≈ 118 N of
gravity) exceeded 100 N, so in Gazebo the joint silently sagged to its
bottom stop and could not lift back ("commanded 0.1, reads 0.26" is the
symptom to remember).

### head_pan orbited instead of panning

The exporter anchored the `head_pan` joint at `head_pan_1`'s arbitrary link
frame — ~15 cm off the real pivot (design `(-0.0767, -0.1505, 1.26)` vs. the
hardware cluster at `y≈-0.054, z≈0.78-0.86`). Panning swung the whole head in
an arc; it only looked right at zero. The true shaft was measured from the
meshes: slicing `head_pan_camera_1.stl` (the pan motor) and `head_pan_1.stl`
(the bracket) shows the motor's top-face circle and the bracket's bottom hub
both centred at design `(-0.0219, -0.0549)`. The joint origin was moved
there, with `head_pan_1`'s visual/collision/inertial origins and the
`Rigid 87` child offset compensated so the zero pose is byte-identical
(verified by FK diff). Axis flipped to `+Z` so positive pan = CCW/left
(REP-103 yaw).

### Misnamed base-camera meshes

Both variant CAD exports shipped the *base* camera mesh under the name
`head_camera_1.stl` and never shipped `base_camera.stl` — which both
`base101_simple.xacro` and `base101_pro.xacro` reference. That was the
long-standing "Failed to load geometry for visual: …base_camera…" error in
Gazebo. Identity was confirmed by matching each STL's bounding-box centre
against the base_camera link's CoM, then the files were renamed to
`base_camera.stl` in `meshes/simple/` and `meshes/pro/`. (The tower's actual
head camera mesh lives at `meshes/tower/head_camera_1.stl` — no conflict.)

## Debugging techniques worth keeping

- Fusion-style exports place every link frame axis-aligned with the design
  frame; a link's design-frame position is simply `-(visual origin)`. That
  identity makes cross-export frame reconciliation and FK spot-checks cheap.
- Binary STL slicing (1 mm z-bands, per-band XY centroid + radius spread)
  reliably locates motor shafts, hubs, and bolt-pattern centres when the
  export's joint anchors can't be trusted.
- When a controller "fails to activate" with `command interface X/position
  is not available`, check whether the controller_manager logged
  `ResourceManager has already loaded a urdf` — a stale
  `robot_state_publisher` from a previous run published an old
  `/robot_description` first. Kill *everything* between sim runs.
