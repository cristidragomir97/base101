# base101 Navigation Package

Pure Nav2 stack for the base101 robot: planner, controller, bt_navigator,
velocity smoother. Restructured for robocore autonomy (see
`docs/worklogs/nav_restructure.md`): one always-on launch, no modes, no
mode_manager, no behavior_server, no AMCL, no explore_lite.

SLAM and odometry fusion live in **base101_slam**. The two stacks are
deliberately independent — each has its own lifecycle manager and neither
package depends on the other. Nav2 consumes `/map` and the `map->odom`
TF from whatever publishes them (slam_toolbox here) and starts fine
before they exist.

## Quick Start

Normally you don't launch this yourself — the bringup packages compose it,
and it comes up with the sim by default:

```bash
ros2 launch base101_bringup_gazebo sim.launch.py       # sim + slam + nav2
ros2 launch base101_bringup_hw     robot.launch.py     # robot + slam + nav2
```

Launching the halves by hand is still supported, and is the point of keeping
them independent — it's how you restart Nav2 without disturbing the map:

```bash
colcon build --packages-select base101_nav base101_slam
source install/setup.bash

# The robot's body, without any autonomy
ros2 launch base101_bringup_gazebo sim.launch.py nav:=false slam:=false

# SLAM half (mapping by default; localization via service switch)
ros2 launch base101_slam slam.launch.py use_sim_time:=true

# Nav2 half
ros2 launch base101_nav nav.launch.py use_sim_time:=true
```

**Nav2 on its own does nothing visible.** It blocks in `Activating` until
something publishes the `map` frame, so `nav.launch.py` without
`slam.launch.py` (or another map source) looks like a launch that hangs.

## Design

- **Modes are service calls, not process restarts.** slam_toolbox starts
  in mapping mode; the robocore bridge switches to localization via
  `/slam_toolbox/deserialize_map` and saves maps via
  `/slam_toolbox/serialize_map` + `map_saver_cli`.
- **No recovery behaviors in ROS.** The BT is plan/follow/fail
  (`behavior_trees/nav_to_pose.xml`): Nav2 returns ABORTED on any
  failure, the bridge yields `NavStatus(phase="stuck")`, and the Python
  mission decides what happens next. The bridge clears both costmaps
  before each goal via `/global_costmap/clear_entirely_global_costmap`
  and `/local_costmap/clear_entirely_local_costmap`. No spinning — the
  tower and arms make it dangerous.
- **One costmap config for all modes** (`config/costmap.yaml`):
  slam_toolbox always publishes `/map`, so the static layer always has a
  map — growing while mapping, fixed when localized.
- **cmd_vel topology:** controller → `cmd_vel_raw` → velocity_smoother →
  `cmd_vel_nav` → twist_mux (priority 10, below agent teleop at 50 and
  joystick at 100).

## Configuration Files

| File | Purpose |
|------|---------|
| `planner.yaml` | SmacPlanner2D global path planning |
| `controller.yaml` | MPPI controller for trajectory following |
| `costmap.yaml` | Unified global/local costmaps (all modes) |
| `bt_navigator.yaml` | Behavior tree navigator config |
| `velocity_smoother.yaml` | Velocity command smoothing |
| `laser_filter.yaml` | Scan filter config (not currently launched) |

## Robot Parameters

The configuration is tuned for the base101 robot (differential drive, DDSM115 wheels):
- Wheel separation: 0.38 m
- Wheel radius: 0.05035 m (100.7 mm wheel)
- Max linear velocity: 0.8 m/s (controller cap; hw allows 1.5)
- Max angular velocity: 1.5 rad/s (controller cap; hw allows 2.5)
- Footprint: ~50cm x 48cm rectangle (refine against CAD)
