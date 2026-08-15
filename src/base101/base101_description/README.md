# base101_description

The **shared chassis library** for the base101 robot: the common base —
extrusion frame, wheels, mounts, bumpers, lidar, camera, compute tray, the
`top_plate_1` deck — plus materials, sensors, and meshes.

This package is a *parts library, not a launchable robot*. It exposes
`chassis.xacro` (a link/joint fragment) that every variant includes; you never
load it directly. The variant packages assemble it into complete robots:

| Variant | Package | Adds |
|---|---|---|
| simple | `base101_simple_description` | nothing (bare chassis) |
| arm    | `base101_arm_description`    | 1 mod101 arm on `top_plate_1` |
| ~~tower~~ | `attic/base101_tower/` | *parked* — see [attic](../../../attic/README.md) |

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
scripts/
├── import_cad_export.py          # fold a fresh Fusion export into meshes/
├── mass_properties.py            # integrate mass/COM/inertia off a mesh
├── collision_primitives.py       # swap <collision> meshes for boxes/cylinders
└── decimate_meshes.py            # quadric-error decimation of the visual STLs
```

Those four run in that order, and the order matters: `mass_properties` needs
watertight meshes to integrate a volume, and `decimate_meshes` deliberately
gives that up (see below).

The variant top-level xacros own the per-simulator dispatch and the
`gz_ros2_control` plugin block (which points at that variant's
`controllers.sim.yaml`), so this package carries no controller-file reference.

## Arguments

| arg | values | default | effect |
|---|---|---|---|
| `camera` | `realsense`, `oak_d` | `realsense` | which module hangs off the edge bracket |

The bracket takes either an Intel RealSense D435 or a Luxonis OAK-D on the same
face. Only the selected one is instantiated, but `camera_link` and
`camera_optical_frame` exist either way and the sim topics stay `/base_camera/*`,
so nothing downstream has to branch. The two differ in body depth (the D435
stands ~8 mm further proud) and in HFOV (87° vs 69°, wired into `chassis.gazebo`).

## Frames

| frame | where it is | who consumes it |
|---|---|---|
| `base_link` | wheel contact plane, centred on the chassis | odom, nav, diff_drive |
| `lidar_frame` | RPLidar C1 rotor axis, 96.5 mm off the floor | `/scan`, slam, nav |
| `camera_link` | front face of the fitted camera, +X out of the lens | `/base_camera/*` |
| `camera_optical_frame` | REP-103 twist of `camera_link` | point clouds |
| `imu_link` | on the link101 PCB, 99 mm up and 10 mm back | `/sensors/imu`, EKF |

`base_link` sits *on the ground*, not on the deck — a chassis-height origin
would shift every costmap and camera height downstream. The CAD origin is the
top of the extrusion box, 81.965 mm up, so `chassis.xacro` drops the root by
exactly that much and leaves every other position at its exported value.

## Re-importing a CAD export

The Fusion export is a flat pile of STLs in global assembly coordinates plus a
machine-generated URDF. `scripts/import_cad_export.py <export-dir>` handles the
mesh half:

- **The right-hand wheels come out misnamed.** Fusion's `wheel_back_right_1` is
  physically at the front. The two files are swapped on import.
- **The deck standoffs come out as `standoff2/3/4`**, which says nothing about
  which corner they are. They are renamed after their corner.
- **Each wheel is exported as one rigid body.** The DDSM210 is a hub motor: the
  stator boss is bolted into the printed mount and does not turn. The mesh
  splits cleanly (boss at r ≤ 12.5 mm, rim at r ≥ 35 mm), so the importer cuts
  each wheel at r = 20 mm into a fixed `wheel_hub_*` and a turning `wheel_*`,
  capping both halves so they stay watertight.

The URDF half is folded in by hand. Things to redo when the geometry moves:
the `base_link` drop to the contact plane, the wheel joint names and axes, the
five hand-placed frames in the table above, and any link the export left at
`mass="0"` (`scripts/mass_properties.py` prints a ready-made `<inertial>`).

## Collision geometry

**No link uses a mesh for `<collision>`, and `self_collide` is off everywhere.**
Both are deliberate and both are load-bearing for how fast the sim runs.

The CAD export gives every link the same mesh for visual and collision, which
put 216k triangles into the physics engine for a chassis that is, geometrically,
a box on four cylinders. Gazebo and MoveIt/FCL build a BVH per collision body
and re-test them every step, so the RealSense housing (54k), the lidar can
(51k), the link101 PCB (40k) and the Orin (26k) — four bodies sealed inside the
shell that nothing can ever touch — were 80% of the budget. `self_collide` then
multiplied it: 26 links joined by fixed joints, so ~325 pairs of full-resolution
meshes tested per step to discover that a rigid frame is still rigid.

Now: 21 boxes + 9 cylinders, **0 collision triangles**, no self-collision pairs.
Visual meshes are untouched — a GPU does not care about 216k triangles.

`scripts/collision_primitives.py` regenerates the primitives from the meshes
(bounds taken in each link's own frame); the shape per link is the `SHAPES`
table at the top of it. Re-run it after a CAD re-import:

```bash
python3 scripts/collision_primitives.py urdf/chassis.xacro --report   # dry run
python3 scripts/collision_primitives.py urdf/chassis.xacro --write
```

The wheel cylinders are the one shape worth checking by hand after a re-import
— the contact patch is what the drivetrain pushes against. The hubs get no
collision at all, and neither should anything else that lives inside the shell.

## Visual meshes

The export tessellates for manufacturing tolerance, not for rendering, so the
STLs arrive far denser than anything needs: 272k triangles, 14 MB.
`scripts/decimate_meshes.py` runs quadric-error decimation over them at a
0.5 mm surface tolerance, which brings that to 60k triangles and 3.0 MB with a
worst-case bounding-box drift of 0.1 mm.

```bash
python3 scripts/decimate_meshes.py meshes/ --tolerance 0.5 --report   # dry run
python3 scripts/decimate_meshes.py meshes/ --tolerance 0.5 --write
```

It allows topology change by default, which is the setting that actually does
the work: the classic link condition keeps a mesh watertight but forbids ever
closing a hole, so a 3.4 mm bolt hole in a 340 mm plate survives at any
tolerance. Decimated meshes are therefore **not watertight** — fine, because
by this point collision is primitives and nothing integrates over them. Pass
`--watertight` if you need closed meshes back (it reduces much less), and run
`mass_properties.py` *before* decimating, not after.

`base_link` is the stubborn one: it only reduces 34%, because it is genus 199
— the M3 tapped grid means nearly every vertex sits on a hole boundary. That
grid is the part's defining feature, so leave the tolerance where it is rather
than dissolving it.

## Things to remember when editing the chassis

- **Joint names are the binding contract** with every other package. The four
  wheels are exactly `front_left_wheel_joint`, `front_right_wheel_joint`,
  `back_left_wheel_joint`, `back_right_wheel_joint`. Renaming one ripples into
  every variant's `controllers.sim.yaml` and `chassis.*.ros2control`.
- **All four wheel joints use axis +Y**, including the right-hand pair (the raw
  export flips them), so a positive diff_drive command drives forward on both
  sides.
- **`top_plate_1` is the deck every add-on parents to.** It is now 180 × 240 mm
  and rides 45 mm of standoff (130 mm off the floor); the previous export had a
  340 × 240 mm plate sitting straight on the extrusion at 82 mm. That change
  made `base101_arm_description`'s own mount deck redundant — it was an
  identical 180 × 240 plate on identical 45 mm standoffs, stacked on this one —
  so the arm now bolts directly here. Moving or renaming `top_plate_1` ripples
  into that package.
- `<gazebo>` extensions in `chassis.gazebo` are only included on the gazebo
  branch of each variant xacro, so they don't affect the `none` (rviz / real
  hardware) build.
- Wheel separation / radius live in each variant's `controllers.sim.yaml` —
  keep them in sync with the URDF geometry. Measured off the current meshes:
  track 0.2899 m (rim mid-planes), rolling radius 0.036 m, wheelbase 0.2393 m.
- **The exported inertials are all steel.** Fusion had no materials assigned,
  so every body came out at ~7850 kg/m³ and the chassis totals 14 kg against a
  real ~5 kg. They are kept as exported (the sim contact params were tuned
  against numbers of this order) but they are not physical — worth a pass with
  real densities if the sim dynamics ever start to matter.
