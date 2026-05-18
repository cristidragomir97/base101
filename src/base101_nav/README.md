# base101 Navigation Package

Nav2 configuration and mode management for the base101 robot.

## Quick Start

```bash
# Build the package
colcon build --packages-select base101_nav
source install/setup.bash

# Launch navigation with a map
ros2 launch base101_nav navigation.launch.py map:=/path/to/map.yaml

# Launch SLAM for mapping
ros2 launch base101_nav mapping.launch.py

# Launch mapfree navigation (no localization)
ros2 launch base101_nav mapfree.launch.py

# Launch the mode manager (orchestrates mode switching)
ros2 launch base101_nav mode_manager.launch.py
```

## Navigation Modes

### Navigation Mode
Full autonomous navigation using a pre-built map:
- Map server provides static occupancy grid
- AMCL localizes the robot in the map
- Nav2 plans and executes paths
- Recovery behaviors handle stuck situations

### Mapping Mode
SLAM for building new maps:
- SLAM Toolbox builds map in real-time
- Teleoperate the robot to explore
- Save maps via service call

### Mapfree Mode
Local navigation without a map:
- Identity transform from map to odom
- Rolling window costmaps
- Good for teleoperation with obstacle avoidance

## Configuration Files

| File | Purpose |
|------|---------|
| `planner.yaml` | SmacPlanner2D global path planning |
| `controller.yaml` | MPPI controller for trajectory following |
| `costmap.yaml` | Global/local costmaps for map-based navigation |
| `costmap_mapfree.yaml` | Costmaps for mapfree mode |
| `amcl.yaml` | AMCL localization parameters |
| `bt_navigator.yaml` | Behavior tree navigator config |
| `behavior.yaml` | Recovery behaviors (spin, backup, wait) |
| `velocity_smoother.yaml` | Velocity command smoothing |
| `slam_toolbox.yaml` | SLAM Toolbox configuration |

## Robot Parameters

The configuration is tuned for the base101 robot (differential drive, DDSM115 wheels):
- Wheel separation: 0.38 m
- Wheel radius: 0.05035 m (100.7 mm wheel)
- Max linear velocity: 0.8 m/s (controller cap; hw allows 1.5)
- Max angular velocity: 1.5 rad/s (controller cap; hw allows 2.5)
- Footprint: ~50cm x 48cm rectangle (refine against CAD)

## Map Storage

Maps are stored in `~/.base101/maps/` with the format:
- `mapname.yaml` - Map metadata
- `mapname.pgm` - Occupancy grid image

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/nav/change_mode` | Trigger | Cycle through modes (Phase 1) |
| `/nav/save_map` | Trigger | Save current SLAM map |
| `/nav/stop` | Trigger | Stop current mode |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/nav/mode` | String | Current mode |
| `/nav/maps` | String (JSON) | Available maps |
| `/nav/current_map` | String | Active map name |

## Phase 2 Roadmap

Phase 2 will migrate from subprocess spawning to lifecycle management:
- Faster mode switching (< 1 second vs 5+ seconds)
- Built-in health monitoring
- Graceful state transitions
- Custom service types for proper API

See `SPEC.md` for the full specification.
