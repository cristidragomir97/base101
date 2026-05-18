# Navigation worklog

Notes from porting Nav2 + slam_toolbox + frontier exploration into the
base101 workspace, plus the debugging passes that followed. Written
chronologically; the "current state" is whatever's in `src/base101_nav/`
and `base101.repos` at HEAD.

## Origin

The stack was ported from `LLMy/ros/src/llmy_nav` (the LLMy repo had two
candidates, `llmy_slam` and `llmy_nav`; `llmy_nav` is a strict superset
that already includes the slam_toolbox configs, so `llmy_slam` was
dropped). Rename happened in one pass:

- `cp -r llmy_nav → src/base101_nav`
- Python module dir, resource marker, `package.xml` `<name>`, and
  `CMakeLists.txt` `project()` all renamed.
- Bulk `sed` of `llmy_nav → base101_nav` and `~/.llmy/maps → ~/.base101/maps`
  across launch files, configs, README, and `mode_manager.py`.
- Dropped `llmy_description` / `llmy_camera` deps from `package.xml`.

## Adaptations vs LLMy

| Area | LLMy (origin) | base101 (after) |
|---|---|---|
| Drive type | omnidirectional_controller | diff_drive_controller |
| Wheel separation | 0.176 m | 0.38 m |
| Wheel radius | 0.025 m | 0.05035 m (DDSM115) |
| Footprint | 0.30 × 0.20 m | 0.50 × 0.48 m (`[[0.25, 0.24], ...]`) |
| Velocity caps (controller) | 0.5 / 1.0 m/s, rad/s | 1.2 / 2.0 (hw allows 1.5 / 2.5) |
| Odom topic | `/omnidirectional_controller/odom` | `/diff_drive_controller/odom` (was already correct in LLMy ekf configs) |
| EKF `base_link_frame` | `base_footprint` | `base_link` |
| EKF `publish_tf` | `true` | `false` (diff_drive_controller already owns `odom→base_link`; two publishers would conflict) |
| SLAM `base_frame` | `base_footprint` | `base_link` |
| BT `odom_topic` | `/odom` | `/diff_drive_controller/odom` |

The Nav→controller wiring (`velocity_smoother → /cmd_vel_nav →
twist_mux → /diff_drive_controller/cmd_vel`) needed no changes —
`/cmd_vel_nav` was already what base101's twist_mux expected.

## Runtime deps

apt-installable (`ros-jazzy-*`):
- `nav2_{map_server,amcl,planner,smac_planner,controller,bt_navigator,
  behaviors,velocity_smoother,lifecycle_manager,msgs}`
- `slam_toolbox`, `robot_localization`
- `nav2_regulated_pure_pursuit_controller` (replaced `nav2_mppi_controller` —
  see "Controller choice" below)

Not in apt — vendored via `base101.repos` at workspace root:
- `explore_lite` + `explore_lite_msgs` (`map_merge` is `COLCON_IGNORE`d).
- Source repo: `github.com/robo-friends/m-explore-ros2`, pinned to commit
  `86742bff6edd53fcce16f99dbc8c06ed5d7eed22` on `main`.

Bring-up for collaborators:

```bash
vcs import src < base101.repos
sudo apt install ros-jazzy-nav2-{map-server,amcl,planner,smac-planner,controller,bt-navigator,behaviors,velocity-smoother,lifecycle-manager,msgs,regulated-pure-pursuit-controller} ros-jazzy-slam-toolbox ros-jazzy-robot-localization
colcon build
```

## Launch files (what each one does)

Five launches, each in `.py` and `.yaml` flavors (identical behavior,
pick your preferred format).

- **`mapping.launch.py`** — pure SLAM. Just `slam_toolbox` (async) + its
  lifecycle manager. Drive around, save the map via service.
- **`navigation.launch.py map:=<map.yaml>`** — pure Nav2 with a
  pre-built map. `map_server` + `amcl` + planner + RPP controller +
  bt_navigator + behaviors + velocity_smoother + lifecycle manager.
- **`mapfree.launch.py`** — Nav2 without a map. Identity `map→odom`,
  rolling-window costmaps. Assisted teleop with obstacle avoidance.
- **`slam_nav.launch.py [explore:=true]`** — SLAM + Nav2 simultaneously.
  Now uses SLAM's `/map` as the global costmap's static layer (was
  rolling window in `costmap_mapfree.yaml`, see "Fix #1" below).
- **`mode_manager.launch.py`** — runs `mode_manager.py` which exposes
  `/nav/{change_mode,save_map,stop}` services and subprocess-spawns the
  above launches.
- **`rviz.launch.py`** — opens RViz with `config/nav.rviz` (copy of
  `nav2_bringup/rviz/nav2_default_view.rviz`, Map / costmaps / paths /
  Nav2 goal panel preloaded).

Always pass `use_sim_time:=true` when running against Gazebo, on every
launch including RViz — otherwise TF lookups silently fail (see "Bug
trail" below).

## Sensor frame fix (gpu_lidar)

Side-effect change in `base101_description/urdf/base101_*.gazebo`:
moved the `<sensor type="gpu_lidar">` block from
`<gazebo reference="lidar_1">` to `<gazebo reference="lidar_frame">`.
Before: rays were cast from the mesh origin while `/scan` was stamped
with `lidar_frame`, so the data was offset ~3 cm from where TF said it
came from. After: rays come from the laser plane, stamp matches. The
material/friction props stayed on `lidar_1` (where the mesh and
collision geometry live).

## Controller choice

Started on MPPI (LLMy's default) for ~5 minutes before deciding it was
overkill: a flat 2D diff drive in indoor environments doesn't need a
2000-sample × 56-step model predictive controller. Swapped in
**Regulated Pure Pursuit** (RPP):

- ~1/20th of MPPI's compute.
- ~6 main knobs vs MPPI's ~30 (8 critics, each with weight + threshold).
- Tradeoff: RPP follows the global path rigidly; doesn't reason about
  obstacles itself, relies on planner replanning + costmap inflation.

`controller.yaml` is now a fresh-written RPP config, not a tweaked
MPPI one. Tunables most likely to touch:
- `desired_linear_vel` (top speed)
- `lookahead_time` (bigger = smoother, cuts corners; smaller = tighter)
- `rotate_to_heading_min_angle` (when to turn-in-place vs drive-and-curve)

## Bug trail

### TF extrapolation: `Requested time 1779091262 but latest data is at 813`

Classic sim/wall-clock mismatch. `slam_toolbox` was on sim time (TF
stamped at sim-second ~813), planner_server + behavior_server +
explore_node were on wall time (looking for unix epoch ~1.78e9). The
TF buffer in wall-time nodes considered the sim-stamped frames ~1.7e9
seconds in the past and gave up. Hence every plan aborted and explore
reported "all frontiers traversed" — it never managed a single pose
lookup.

**Fix:** `use_sim_time:=true` on every launch when in Gazebo. Launch
files already propagate it; default stays `false` for portability to
hardware.

### TF extrapolation: `Requested time 9.092 but earliest data is at 13.1`

Different shape, both on sim time. explore_node's `TimerAction`
started it at sim-second ~9, before slam_toolbox had published the
first `map→odom` (around sim-second 13). The initial goal's stamp
predated the TF buffer.

**Fix:** bumped explore start delay 20 → 30 s in `slam_nav.launch.py`.
Cosmetic — explore_lite retries and recovers on its own — but keeps
startup logs clean.

### EKF removed from slam_nav

`ekf_filter_node` was in the original launch. With no IMU on base101,
EKF on wheel odom alone is a glorified passthrough that adds latency.
Removed the node and the 10-second `TimerAction` that was waiting for
it to publish `odom→base_link`. Configs (`ekf.yaml`, `ekf.sim.yaml`)
left in place with `publish_tf: false`; re-enable once an IMU is
wired in.

### `Ignoring the received message ... older than 0.5s`

`diff_drive_controller` was rejecting cmd_vel messages as stale.
Stamps were sim-time (good), but ~0.8 s behind current sim time. Root
cause was Gazebo RTF dropping under Nav2 load — sim time advances
slower than wall time, and the cmd_vel pipeline (planner → smoother →
twist_mux → diff_drive_controller) accumulates real-time latency that
shows up as sim-time staleness.

**Fix:** raised `cmd_vel_timeout: 0.5 → 2.0` in
`base101_control/config/controllers.{pro,simple}.sim.yaml`. Left HW
config strict at 0.5 s. If you need to debug RTF:
`gz topic -e -t /world/<world>/stats`.

### "Courageous leaps, then minutes of recomputing"

From `mapping_navigation.log`: 72 planner aborts, 30+ "Goal Coordinates
outside bounds", 20+ "RegulatedPurePursuitController detected collision
ahead", 23 spin/backup recoveries firing in cycles of ~20-30 s each.
Multiple root causes stacked:

1. **`slam_nav` used a rolling 10×10 m global costmap** (from
   `costmap_mapfree.yaml`) while explore picked frontiers up to 7+ m
   away from the SLAM `/map`. Goals outside the 5 m radius window =
   "outside bounds". → Switched `costmap_mapfree.yaml`'s global_costmap
   to a static layer subscribed to `/map`, frame=`map`,
   `rolling_window: false`. Local costmap stays rolling.

2. **Inflation 0.35 m on a 0.50×0.48 m footprint = thick personal
   bubble.** Spin and backup recoveries both failed with "Collision
   Ahead" because the inflated halo around the robot intersected
   itself when projected. → 0.35 → 0.25 in all three configs
   (`costmap.yaml`, `costmap_mapfree.yaml`, RPP's ObstaclesCritic
   mirror in `controller.yaml`).

3. **RPP forward collision check too aggressive.** At 1.2 m/s with
   `max_allowed_time_to_collision_up_to_carrot: 1.0`, the projected
   carrot was 1.2 m ahead — well into the inflation halo of any
   nearby obstacle. → 1.0 → 0.4 s. Also dropped `cost_scaling_dist`
   0.6 → 0.4.

4. **Explore picked unreachable frontiers.** `min_frontier_size: 0.3`
   caught noise pixels at map edges. → 0.3 → 0.6. Bumped
   `potential_scale: 3.0 → 5.0` (stronger distance penalty, prefer
   nearby).

### Follow-up: "no valid path found" everywhere after the StaticLayer swap

Round-2 symptom: the SLAM-static-layer fix above introduced a worse
failure — robot sat at spawn forever, planner replanned at 1 Hz with
the controller logging "Passing new path to controller" 12+ times
without any motion, then progress_checker killed the goal.

Root cause: `track_unknown_space: true` on the global_costmap meant
unknown cells from SLAM's `/map` were treated as not-passable. When
SLAM has just started, ~everything is unknown — robot sits on a tiny
island of known-free cells, frontiers (by definition) border unknown,
and the planner can't cross the unknown gap to reach them.

Fix: `track_unknown_space: false` on the global_costmap +
`trinary_costmap: true` on the static_layer (don't preserve the
unknown=gray middle value, collapse to free/lethal). Planner now
crosses unknown freely; obstacle_layer catches anything live lidar
sees.

Also bumped the BT's `RateController hz="1.0" → "0.5"` — replanning at
1 Hz spammed "Passing new path" and made the log unreadable. 0.5 Hz
is still plenty for indoor speeds.

### Inflation rewrite (the actually-important fix)

After all the above tuning the robot was still getting trapped easily.
Compared our config to two reference Nav2 setups (upstream
`nav2_bringup/params/nav2_params.yaml` on jazzy, TurtleBot4's
`turtlebot4_navigation/config/nav2.yaml`). Discovered we'd been using
a fundamentally wrong inflation pattern, copy-pasted from somewhere
without understanding what it meant:

|                              | Nav2 default | TurtleBot4 | Ours (was) |
|------------------------------|:------------:|:----------:|:----------:|
| `inflation_radius`           |    0.70 m    |   0.45 m   |   0.25 m   |
| `cost_scaling_factor`        |     3.0      |    4.0     |    10.0    |

The cost formula is `(cost_max - 1) * exp(-cost_scaling_factor *
(distance - inscribed_radius))`. With our small radius + steep
scaling, the cost map looked like a binary cliff: ~free outside 25 cm,
near-lethal inside. The planner had to thread perfect needles past
obstacles; RPP's forward collision check tripped on its own
inflation halo at any speed; recoveries failed because spin/backup
projection always intersected the halo.

The canonical pattern is the inverse: **large radius + gentle
scaling**, so cost rises smoothly from obstacles. Planner naturally
prefers centered corridors but can still squeeze through tight gaps
when needed, and RPP/recoveries see a soft cost gradient instead of a
wall.

Changes (applied to both `costmap.yaml` and `costmap_mapfree.yaml`):

- `inflation_radius`: 0.25 → **0.55** (a bit larger than TB4 since
  base101 is slightly bigger).
- `cost_scaling_factor`: 10.0 → **3.0** (Nav2 default).
- `controller.yaml`'s `inflation_cost_scaling_factor`: 10.0 → **3.0**
  (MUST match the inflation layer or RPP miscalculates proximity).
- `controller.yaml`'s `cost_scaling_dist`: 0.4 → **0.55** (match
  inflation radius).
- `controller.yaml`'s `max_allowed_time_to_collision_up_to_carrot`:
  0.4 → **1.0** (with gentle gradient, the carrot doesn't false-trip).

Also brought update frequencies in line with Nav2 defaults (we were
3-5× over):

- global_costmap `update_frequency`: 5.0 → **1.0 Hz**
- global_costmap `publish_frequency`: 2.0 → **1.0 Hz**
- local_costmap `update_frequency`: 10.0 → **5.0 Hz**
- local_costmap `publish_frequency`: 5.0 → **2.0 Hz**

This single change set is more impactful than every previous tuning
round combined. Lesson: when copying configs, copy from the *canonical*
reference (`nav2_bringup`), not from a derivative project — derivatives
often have undocumented bot-specific tweaks that don't generalize.

### BT and behavior plugins audit

Same "copy from canonical" pass on the behavior trees. Both
`nav_to_pose.xml` and `nav_through_poses.xml` were derivatives missing
the `<WouldAPlannerRecoveryHelp>` / `<WouldAControllerRecoveryHelp>`
guards introduced in jazzy. Those guards inspect the planner/controller
error code before triggering recovery — e.g. "invalid goal" errors no
longer waste ~10 s on spin/backup attempts that can't help.

Replaced both BTs with the canonical Nav2 jazzy versions
(`navigate_{to_pose,through_poses}_w_replanning_and_recovery.xml`) and
kept exactly one local tweak: `Wait wait_duration: 5.0 -> 2.0`. Reverted
several of our earlier "compensation" tweaks now that the costmap is
sane:

- Top-level `RecoveryNode number_of_retries`: 2 → **6** (canonical)
- `RateController hz`: 0.5 → **1.0** (canonical; nav_through_poses
  uses 0.333)
- `BackUp` 0.20 m → **0.30 m** (canonical)
- Added `error_code_id` on `<Spin>` and `<BackUp>` so the BT propagates
  *why* a recovery failed, not just that it did.
- `nav_through_poses` now includes `<RemovePassedGoals radius="0.7">`
  inside a `ReactiveSequence` (was missing, meaning passed waypoints
  weren't being pruned).

For the `behavior_server` config: registered all five Nav2-default
plugins (was `spin, backup, wait`; added `drive_on_heading,
assisted_teleop`). They don't run unless referenced from a BT, but
this avoids "plugin not registered" surprises if we want to add a
`drive_on_heading` nudge to the recovery round-robin later.

Also bumped `max_rotational_vel: 1.0 → 1.5` on behavior_server so
`<Spin>` actually rotates at the rate base101 hardware allows.

5. **BT retried for ~90 seconds before failing.** `RecoveryNode
   number_of_retries="6"` × ~15 s/cycle = ~90 s. → 6 → 2 (~30 s).
   Also shortened the `Wait wait_duration` 5.0 → 2.0 s and made
   backup faster: `backup_dist: 0.30 m @ 0.05 m/s` (6 s) →
   `0.20 m @ 0.15 m/s` (~1.3 s). Same edits applied to both
   `nav_to_pose.xml` and `nav_through_poses.xml`.

## Velocity tuning history

Three passes:

1. Initial: vx_max 0.8, wz_max 1.5, smoother accel 0.5 m/s² → too slow
   to react, robot felt mushy.
2. Sped up: vx_max 1.2, wz_max 2.0, accel 1.5 m/s², smoother 30 Hz →
   better but courageous leaps interspersed with stuck cycles.
3. Stuck cycles fixed via costmap/BT changes above; velocities kept
   at pass-2 values.

`velocity_smoother.yaml` and `controller.yaml` must agree on caps.
Hardware caps in `controllers.{pro,simple}.sim.yaml` are 1.5 m/s and
2.5 rad/s; controller is configured 0.3 below hw to leave headroom
for the smoother to actually reach commanded velocities without
clipping at the diff_drive_controller layer.

## Retro: we approached this wrong

The right bring-up order for a new Nav2 stack is:

1. **SLAM with teleop only.** Drive manually, build a map, save it.
   Validates: lidar → SLAM TF (`map → odom → base_link`),
   `/scan` topic, sim time wiring, robot can actually move under
   teleop. Nothing else runs. No Nav2, no costmap, no controller, no
   exploration.
2. **Nav2 with the saved static map, manual goals.** Map_server +
   AMCL + planner + controller. Tune costmap inflation, controller
   limits, goal tolerances against a known, frozen map. Validates: the
   Nav2 stack itself works on this robot's geometry, with a costmap
   you can inspect statically in RViz.
3. **Semi-known: SLAM + Nav2, manual goals.** Drive to clicked goals
   while SLAM is actively expanding the map. Validates: the
   StaticLayer keeps up with map growth, costmap inflation behaves
   sanely on a still-growing world, `track_unknown_space` decision is
   correct.
4. **Autonomous exploration.** Now add `explore_lite`. By this point
   everything underneath is known-good; if something breaks it's the
   explorer's choice of frontiers or the BT's recovery loops, not the
   stack itself.

What we actually did: jumped to step 4. Every problem we hit
(track_unknown_space, sim-time mismatch, inflation cliff, BT
over-recovery, MPPI compute overhead) was simultaneously broken, and
each one masked the others. A symptom in step 4 could have been
caused by anything in steps 1-3. We spent debugging effort
fault-isolating in a system with N unknown variables instead of N=1.

Concretely, if we'd done step 1 first, we'd have validated SLAM and
the `use_sim_time` plumbing in 15 minutes. Step 2 would have flushed
out the inflation pattern and controller choice without ever
involving SLAM. Step 3 would have exposed the StaticLayer +
track_unknown_space tension as the only new variable. By the time we
got to step 4, exploration would have been the only thing left to
tune.

Lesson for next time (and this is general, not Nav2-specific): when
bringing up a stack of N interacting components, run a bring-up
sequence where each step adds **exactly one** new component on top of
known-good ones. If a step misbehaves, the new thing is the cause.
Skipping ahead to "let's just run the whole stack" trades 15 minutes
of bring-up discipline for hours of fault isolation in a system where
everything is suspect.

## Things deliberately not done

- **`laser_filter`** is configured (LLMy used it to crop body returns)
  but not in any launch. base101's lidar mount is high enough that the
  full 360° is probably usable; revisit if `/scan` shows phantom
  returns at body-shaped angles.
- **EKF integration** — IMU wiring not in scope yet. Configs ready.
- **MoveIt or arm integration** — out of scope for nav.
- **Custom service types** for mode_manager — code has a "Phase 2"
  comment about migrating from Trigger services to typed ones. Not
  done; Trigger works fine for now.
