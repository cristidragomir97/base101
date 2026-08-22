# base101 package restructure — two bringup packages

Proposal, 2026-08-22. Replaces the `simple`/`arm` package split with one
model + one config set + two launch owners.

## Why

The `simple` vs `arm` distinction is one boolean, currently expressed as
six packages across three axes:

| axis | simple | arm |
|---|---|---|
| description | `base101_simple_description` | `base101_arm_description` |
| control | `base101_simple_control` | `base101_arm_control` |
| gazebo | `base101_simple_gazebo` | `base101_arm_gazebo` |

The two top-level xacros differ by **two lines** (one `<xacro:include>` of
the arm, one path in the gz_ros2_control `<parameters>` tag). The two
gazebo launches are ~220 lines each and ~90% identical — bridges, twist_mux,
rosboard, spawner event chains, all copy-pasted. Wiring nav in today meant
writing the same 30-line block twice, which is what prompted this.

## Target layout

Three layers, and only the launch layer is typed by a human.

### Model — what the robot is

```
base101_description/
  urdf/base101.xacro          <- THE robot. args: arm, arm_tool, camera, simulator
  urdf/chassis.xacro          (unchanged)
  urdf/chassis.{gazebo,gazebo.ros2control,mujoco.ros2control}
  urdf/arm.xacro              <- moved from base101_arm_description
  urdf/materials.xacro
  meshes/                     (unchanged)
base101_control_plugin/       (unchanged — C++ SystemInterface)
```

`base101.xacro` absorbs both variant xacros:

```xml
<xacro:arg name="arm" default="false"/>
<xacro:if value="$(arg arm)">
  <xacro:include filename="$(find base101_description)/urdf/arm.xacro"/>
</xacro:if>
```

**The one risk to verify first:** `arm.xacro` pulls in `mod101_description`,
so folding it into `base101_description` threatens armless builds that have
no mod101 underlay. xacro skips `<xacro:include>` inside a false
`<xacro:if>`, so `arm:=false` should never resolve `$(find mod101_*)` — but
this needs a real test (build in a container without mod101) before the
merge is committed. If it fails, the fallback is to keep
`base101_arm_description` as the single remaining `*_arm` package and have
`base101.xacro` include from it conditionally; everything else below is
unaffected.

### Config — how it's tuned

```
base101_control/
  config/controllers.sim.yaml   <- wheels + arm + trajectory variants, merged
  config/controllers.hw.yaml    <- same, for the real robot
  config/twist_mux.yaml
  urdf/base101.hardware.xacro   <- moved from base101_simple_control
base101_nav/     config/ + behavior_trees/ + launch/nav.launch.py
base101_slam/    config/ + maps/ + launch/slam.launch.py
base101_worlds/  worlds/ + config/gz_ros_bridge.yaml   <- renamed from base101_gazebo
base101_arm_moveit_config/                             <- unchanged
```

Merging the controller yamls is safe: the arm controllers are *declared*
in `controller_manager.ros__parameters` but only **spawned** by name, so an
armless robot simply never spawns them. `base101_worlds` gets renamed off
`base101_gazebo` because `base101_gazebo` vs `base101_bringup_gazebo`
side-by-side is a coin flip every time you read it.

### Launch — the two packages you actually type

```
base101_bringup_gazebo/launch/sim.launch.py
base101_bringup_hw/launch/robot.launch.py
```

Both own the **whole** graph and expose the same argument contract, so
muscle memory transfers between sim and robot:

| arg | default | meaning |
|---|---|---|
| `arm` | `false` | mount one mod101 arm |
| `arm_tool` | configurator's | mod101 end-effector |
| `arm_control` | `sliders` | `sliders` (position) or `moveit` (trajectory) |
| `nav` | `true` | Nav2 |
| `slam` | `true` | EKF + slam_toolbox |
| `rviz` | `false` | nav display config |
| `rosboard` | `true` | web dashboard |
| `moveit` | `false` | move_group (implies `arm_control:=moveit`) |
| `agent` | `true` | robocore agent (JSON-RPC bridge) |
| `profile` | auto | robocore profile YAML; auto-picked by `arm` |
| `world` | `sticky_floor.sdf` | **sim only** |
| `camera` | `realsense` | **sim only** (hw reads what's plugged in) |

### What stays composable

`base101_nav` and `base101_slam` **keep** their launch files. They are the
stack definitions; the bringup packages compose them via
`IncludeLaunchDescription`. This preserves the independence ruling from
`nav_restructure.md` — `ros2 launch base101_nav nav.launch.py` still works
standalone for debugging, and neither stack knows about the other. The rule
is narrower than "bringup owns all launch":

> **The bringup packages own every *top-level* launch — the ones a human
> types. Stack packages may own a stack launch, which only bringup includes.**

`base101_arm_moveit_config` is left alone; it is
moveit_setup_assistant-shaped and regenerating it is not worth the fight.

### Deleted

`base101_simple_description`, `base101_simple_control`,
`base101_simple_gazebo`, `base101_arm_control`, `base101_arm_gazebo`, and
`base101_arm_description` (the xacro test above passed — see Outcome). Their
`display.launch.py` files collapse into one
`base101_bringup_hw/launch/display.launch.py` with the same `arm` arg.

Net: 14 packages -> 9.

## Migration order

Each step leaves the tree buildable and the sim launchable.

1. **`base101_worlds` rename.** Mechanical, no behaviour change. Do it first
   so later steps write the final name.
2. **Merge the xacros** into `base101_description/urdf/base101.xacro`, with
   the armless-build test. Old variant xacros stay as two-line shims that
   include the new one, so nothing breaks yet.
3. **Merge the controller yamls** into `base101_control`.
4. **Write `base101_bringup_gazebo`** as the union of the two gazebo
   launches — one `arm` branch instead of two files. Verify against both
   `arm:=false` and `arm:=true`.
5. **Write `base101_bringup_hw`** from `control_stack.launch.py` + the
   autonomy includes.
6. **Delete the six packages** and the shims; update `README.md`,
   `docs/testing.md`, and the robocore profiles that name launch files.

Steps 1–3 are safe to land independently of whether 4–6 ever happen.

## Gaps this surfaces (not caused by it)

Moved to `findings-open.md`, which is the live list. In summary: no hardware
path for the arm, a cold-map `START_OCCUPIED` nav abort that still needs
confirming, an inflation-radius warning on every nav bringup, MoveIt not
installed on this machine, an orphaned `laser_filter.yaml`, and a
`bond_timeout` disagreement between the two stacks.

One item from the original draft was **wrong** and is corrected there: the
hardware control stack is *not* mock. `base101.hardware.xacro` declares
`base101_control_plugin/ROS2ControlBridge`, the real Axon 2 bridge. It was
`base101_control/README.md` that still described a
`mock_components/GenericSystem` placeholder, and this document repeated it.

## Outcome

Landed 2026-08-22, all six steps. Deviations from the plan above:

- **The armless-build risk did not materialise.** `arm:=false` processes
  with the mod101 underlay stripped from `AMENT_PREFIX_PATH` entirely (38
  links), and `arm:=true` without it fails cleanly pointing at `arm.xacro`.
  So `base101_arm_description` was deleted rather than kept as the fallback.
- **`base101_arm_moveit_config/launch/demo.launch.py` was deleted too.** It
  was a top-level launch (it included the gazebo launch and then move_group),
  which is exactly what the ownership rule says bringup owns. Its job is now
  `sim.launch.py arm:=true moveit:=true`. `move_group.launch.py` stays as the
  stack launch.
- **The SRDF's `<robot name>` had to change** from `base101_arm` to
  `base101` to match the merged URDF's robot name. Unverified — see
  `findings-open.md` #4.
- The two `display.launch.py` files collapsed into
  `base101_bringup_hw/launch/display.launch.py` as planned.
- **`robocore_agent` was added to both bringups** (`agent:=true` by default),
  which the original plan did not cover — it had been run by hand. Its
  profile is resolved from the engine repo rather than copied into this
  workspace, because docker-compose, `simulation.yaml` and the course
  notebooks all reference `engine/profiles/` and a copy would drift.

Verified on a clean sim: `sim.launch.py` with no arguments brings up Gazebo,
both controllers, `/clock` at ~300 Hz, and all five lifecycle nodes
(`planner_server`, `controller_server`, `bt_navigator`, `velocity_smoother`,
`slam_toolbox`) reach `active` with `map->odom` publishing. Full `colcon
build` clean; every remaining launch file parses except `move_group.launch.py`,
which needs MoveIt installed.
