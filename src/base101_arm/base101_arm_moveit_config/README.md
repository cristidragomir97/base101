# base101_arm_moveit_config

MoveIt2 semantics for the **composed** robot: the base101 chassis with one
mod101 arm on its deck.

## Why this exists separately from mod101_moveit_config

mod101 ships a complete MoveIt config for the arm on its own. It cannot be
reused as-is for two reasons, and only the second one really matters:

1. It anchors the robot at a `world` link that exists only in the standalone
   `mod101.xacro`.
2. **Its self-collision matrix contains only arm-vs-arm pairs.** The arm reaches
   ~525 mm from a turret 186 mm up, and every chassis feature is inside that
   envelope — Orin 105 mm away (top at z=126 mm), lidar 138 mm (z=107),
   RealSense 173 mm (z=81), bumpers 206 mm (z=85). Without arm-vs-chassis pairs
   the planner will route the gripper straight through the robot's own lidar.

What this package adds is exactly those two things. The arm's own semantics are
*not* restated: mod101 exposes them as the prefix-parameterised
`mod101_arm_srdf` macro and this package instantiates it with `prefix="arm_"`.

```
srdf/base101_arm.srdf.xacro   virtual_joint -> base_link, macro at prefix arm_
config/
├── kinematics.yaml           pick_ik, position-only (5-DOF arm)
├── joint_limits.yaml         velocity/accel for time parameterisation
├── moveit_controllers.yaml   the two FollowJointTrajectory controllers
├── ompl_planning.yaml        RRTConnect default
├── moveit.rviz               fixed frame base_link, group arm_arm
└── collisions/<tool>.srdf.xacro   GENERATED — chassis pairs only
launch/
└── move_group.launch.py      move_group alone; base101_bringup_gazebo composes
                              it via sim.launch.py moveit:=true
scripts/gen_collision_matrix.py
```

## Running it

```bash
source ~/robots/mod101/install/setup.bash      # underlay first
source install/setup.bash
ros2 launch base101_bringup_gazebo sim.launch.py arm:=true moveit:=true
```

or, against a sim you already have up:

```bash
ros2 launch base101_bringup_gazebo sim.launch.py arm:=true arm_control:=moveit
ros2 launch base101_arm_moveit_config move_group.launch.py
```

`arm_control:=moveit` is not optional (`moveit:=true` sets it for you).
`base101_control/config/controllers.sim.yaml` declares two controllers per
group: `arm_controller` / `gripper_controller` take
`Float64MultiArray` and are what the web slider UIs drive, and
`arm_trajectory_controller` / `gripper_trajectory_controller` offer
`FollowJointTrajectory`, which is the only thing MoveIt can execute. Each pair
claims the same joints, so exactly one may be active.

## Group names

They are `arm_arm` and `arm_gripper`, not `arm` and `gripper` — the macro names
its groups `${prefix}arm`, and the prefix is `arm_`. Every config file keys on
the prefixed names. Cosmetically poor; fixable by giving the mod101 macro an
explicit group-name parameter if it ever grates.

## The collision matrix is generated, and "never" is statistical

`config/collisions/*.srdf.xacro` holds only pairs with at least one chassis
link — mod101 supplies the arm-internal ones and the arm's internal geometry
doesn't change by being bolted down.

It is generated rather than written because it is a property of the *build*:
the configurator can change the rail lengths, which changes the arm's reach,
which changes which pairs can never touch.

**mod101's configurator does not regenerate these for you.** It regenerates its
own matrices and then tells you to run this — the arm deliberately doesn't know
where its consumers live. After any configurator change, run:

```bash
./scripts/sync_arm_change.sh                    # all tools
./scripts/sync_arm_change.sh --tool jaws
./scripts/sync_arm_change.sh --trials 4000000
```

That sources the mod101 underlay (`MOD101_WS`, default `~/robots/mod101`) then
this workspace, so the generator reads the build args the configurator last
saved. Arguments pass straight through to `gen_collision_matrix.py`, which you
can still call directly if the environment is already set up.

**Read the trials note in that script before lowering `--trials`.** "Never
colliding" is decided by random sampling, and disabling a pair that *can*
collide is the dangerous direction. On this robot the default of 10,000 that
mod101 uses was measurably wrong — it disabled `arm_jaws_moving` against both
back standoffs and the back-left wheel. The default here is 1,000,000, which is
past the knee but still sheds about one pair per doubling at 2M.

**mod101's own matrix is *not* converged at 10k** — an earlier version of this
note claimed it was. Diffing a 10k run against a 1M run of the same build (276
pairs, not 210) shows 10k wrongly disabling real pairs on every tool, including
`base_cover_1`↔`wrist_camera_v1_1` on all four — which would let the planner
drive the wrist camera through the base cover. Run mod101's generator with
`--trials 1000000` until its default is raised.

## Not done

- No `controllers.hw.yaml`; the arm is sim-only on real hardware.
- No sensor plugin for octomap, so the planning scene has no perceived
  obstacles — self-collision and the static chassis only. Whether that belongs
  here or in a picking layer, and what it would take, is worked through in
  [`docs/obstacle-awareness.md`](../../../docs/obstacle-awareness.md).
- The virtual joint is `fixed` to `base_link`, i.e. arm-only planning with the
  base treated as stationary. Planning base motion would mean a `planar` joint
  on `odom` and a controller that can execute it.
