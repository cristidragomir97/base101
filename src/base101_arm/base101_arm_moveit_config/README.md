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
├── move_group.launch.py      move_group alone (bring the robot up yourself)
└── demo.launch.py            gazebo + move_group, in order
scripts/gen_collision_matrix.py
```

## Running it

```bash
source ~/mod101/install/setup.bash      # underlay first
source install/setup.bash
ros2 launch base101_arm_moveit_config demo.launch.py
```

or, against a sim you already have up:

```bash
ros2 launch base101_arm_gazebo gazebo.launch.py arm_control:=moveit
ros2 launch base101_arm_moveit_config move_group.launch.py
```

`arm_control:=moveit` is not optional. `base101_arm_control` declares two
controllers per group: `arm_controller` / `gripper_controller` take
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
which changes which pairs can never touch. The mod101 configurator's Save
regenerates both repos' matrices automatically (see `configurator/server.py`,
`BASE101_WS`); to do it by hand:

```bash
python3 scripts/gen_collision_matrix.py            # all tools, ~26 s
python3 scripts/gen_collision_matrix.py --tool jaws
```

**Read the trials note in that script before lowering `--trials`.** "Never
colliding" is decided by random sampling, and disabling a pair that *can*
collide is the dangerous direction. On this robot the default of 10,000 that
mod101 uses was measurably wrong — it disabled `arm_jaws_moving` against both
back standoffs and the back-left wheel. The default here is 1,000,000, which is
past the knee but still sheds about one pair per doubling at 2M. mod101's own
matrix is unaffected: with 21 links and 210 pairs it is converged at 10k.

## Not done

- No `controllers.hw.yaml`; the arm is sim-only on real hardware.
- No sensor plugin for octomap, so the planning scene has no perceived
  obstacles — self-collision and the static chassis only. Whether that belongs
  here or in a picking layer, and what it would take, is worked through in
  [`docs/obstacle-awareness.md`](../../../docs/obstacle-awareness.md).
- The virtual joint is `fixed` to `base_link`, i.e. arm-only planning with the
  base treated as stationary. Planning base motion would mean a `planar` joint
  on `odom` and a controller that can execute it.
