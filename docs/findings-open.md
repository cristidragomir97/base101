# Open findings — things noticed but not fixed

Written 2026-08-22 during the bringup restructure (see
`bringup-restructure.md`). None of these are caused by that restructure;
they were either already there or are environment gaps. Ordered by how
likely they are to bite.

## 1. Nav goal aborts with START_OCCUPIED on a cold map — UNVERIFIED

A `NavigateToPose` to (0.8, 0.0) on a freshly launched sim came back:

```
GridBased plugin failed to plan from (-0.00, -0.00) to (0.80, 0.00): "Start occupied"
NavigateToPose -> ABORTED, error_code 205   # ComputePathToPose START_OCCUPIED
```

The planner believes the robot's own start cell is lethal. `/map` and the
global costmap agreed on extent (297x163 @ 0.05 m, origin -7.43/-2.22), so
the goal was well inside bounds — this is about cost at the start cell, not
geometry.

**Why it is unverified:** a stale `gz sim` server from an earlier run was
still alive during this test, and immediately afterwards teleop failed to
move the robot at all (`map->base_link` stayed at 0,0), which is the
duplicate-server signature. So the result may be an artifact of two servers
fighting rather than a real planning failure. **Re-run on a clean sim before
spending time on it.**

If it does reproduce, the leads are:

- The `nav_restructure.md` bring-up test drove the robot around *before*
  goaling, so it never planned from a cold, mostly-unknown map. A cold start
  may simply mark the unexplored start cell lethal.
- The related warning below (#2) is emitted at the same time and is about
  exactly this: inflation not configured for the planner's collision checker.
- `planner_server` also logs `Robot is out of bounds of the costmap` once
  during activation, before the static layer resizes to the SLAM map. Benign
  on its own, but worth confirming it clears.

## 2. SmacPlanner2D warns the inflation layer is inadequate

Logged as ERROR on every single nav bringup, before any goal:

```
Inflation layer either not found or inflation is not set sufficiently for
optimized non-circular collision checking capabilities. It is HIGHLY
recommended to set the inflation radius to be at MINIMUM half of the robot's
largest cross-section.
```

`costmap.yaml` does configure an `inflation_layer` — so this is about the
radius being too small relative to the chassis, not a missing plugin. The
message claims a substantial run-time performance cost, and it may well be
the same root cause as #1. Worth one pass with the real chassis
cross-section.

## 3. No hardware path for the arm

`base101.hardware.xacro` emits a `ros2_control` block for the four wheel
joints only, and `controllers.hw.yaml` has no arm section. There is nothing
for an arm controller to claim on the real robot.

`base101_bringup_hw` therefore **raises on `arm:=true`** rather than coming
up half-working. Making it real needs, in order: a mod101 `SystemInterface`
(or a second `ROS2ControlBridge` instance) in the hardware xacro, then the
arm block mirrored from `controllers.sim.yaml`, then dropping the guard in
`robot.launch.py`.

## 4. MoveIt is not installed on this machine

`ros2 launch base101_arm_moveit_config move_group.launch.py` fails with
`ModuleNotFoundError: No module named 'moveit_configs_utils'`, and
`dpkg -l | grep ros-jazzy-moveit` returns nothing. The package builds fine
(it is ament_cmake installing configs) but cannot run here, so
`sim.launch.py moveit:=true` is untested on this box. Everything MoveIt-side
in the restructure — the SRDF robot-name fix and the URDF path change below —
is therefore **verified only by inspection, never executed**.

Note the restructure did change two things MoveIt depends on:

- `srdf/base101_arm.srdf.xacro`'s `<robot name>` went `base101_arm` ->
  `base101`, to match the merged URDF. A mismatch here makes MoveIt reject
  the semantic description outright.
- `move_group.launch.py` now builds from
  `base101_description/urdf/base101.xacro` with `arm:=true`.

Both need a real run on a machine with MoveIt before they can be trusted.

## 5. `base101_control/README.md` documents the wrong hardware interface

It says the real-hardware plugin is `mock_components/GenericSystem`, "replace
before flying". `base101.hardware.xacro` actually declares
`base101_control_plugin/ROS2ControlBridge` — the real Axon 2 firmware bridge
over zenoh. The README is stale, describing a placeholder that has since been
replaced. (`bringup-restructure.md` repeated the claim; corrected there.)

The deleted `control_stack.launch.py` carried the same stale note in its
docstring, so that copy is gone with it.

## 6. The robocore engine still points at the stale flagship profile

`engine/profiles/base101_full.yaml` describes chassis + cross tower + **dual**
mod101 arms — a robot that can no longer be built (tower parked in `attic/`,
dual-arm overlay gone). None of its topics resolve: `/tower_controller`,
`/left_arm_controller`, `/right_arm_controller`, `/head_camera/*`, and the
`controllers.arms.yaml` it cites as the source of its joint names.

It has been marked `STALE — DO NOT USE` at the top rather than deleted,
because these still name it and need retargeting to `base101_arm.yaml` first:

- `bpe/docker-compose.localhost.yaml` (the agent's `--profile` argument)
- `bpe/simulation.yaml`
- `engine/README.md` (two places), `engine/missions/README.md`,
  `engine/examples/README.md`, `engine/docs/course/README.md` and
  `engine/docs/course/build_notebooks.py`

Those live in the engine repo, outside this workspace, so they were left
alone.

## 7. `base101_nav/config/laser_filter.yaml` is orphaned

Nothing launches a `laser_filters` node — the package's own README already
admits "(not currently launched)". Either wire it into the nav stack or
delete the file; right now it reads as configuration that is doing something
when it is not.

## 8. SLAM and Nav disagree on `bond_timeout`

`base101_slam` sets `bond_timeout: 0.0` deliberately — `nav_restructure.md`
records that the bond heartbeat misfires under sim time and looped
slam_toolbox through deactivate/reactivate forever. `base101_nav` still uses
`30.0` and has not shown the problem. Fine as it stands, but if the nav
servers ever start flapping the same way, that is the knob, and the reason is
written down in `nav_restructure.md` rather than at the call site.

## 9. Cosmetic: SDF warnings and rosboard topic warnings

- Gazebo logs `XML Element[gz_frame_id] ... not defined in SDF` for all four
  sensors on every launch. Harmless (gz copies the element through), but it
  is four lines of noise at every startup.
- With `arm:=false`, rosboard repeatedly warns `topic
  /arm_wrist_camera/image_raw not found`. The topic list it subscribes to is
  not conditioned on the configuration. Noise only.
