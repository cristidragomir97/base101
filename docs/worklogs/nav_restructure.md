# base101 nav restructure — preparation for robocore autonomy

Per `bpe/nav_setup.md` (Cristi, 2026-06-12), plus two rulings given
during implementation: section 12 wins (behavior_server removed
entirely), and SLAM/Nav2 must be independently launchable — "feel free
to even split it into a different package", so it is one.

## What changed

**New package `base101_slam`** — the localization half:
- `launch/slam.launch.py`: robot_localization EKF (config picked by
  `use_sim_time`: ekf.sim.yaml vs ekf.yaml) + slam_toolbox
  (async, mapping mode) + its own lifecycle manager
  (`lifecycle_manager_slam`, only slam_toolbox — EKF is not a lifecycle
  node). Args: `use_sim_time`, `autostart`, `slam_config` override.
- `config/slam_toolbox.yaml`: unified config, the tested
  online_mapping.yaml values (Ceres solver, 8 m laser range, 0.15 m /
  0.2 rad travel thresholds, tighter loop-closure gates). Mode switching
  is a SERVICE CALL (`/slam_toolbox/serialize_map`,
  `/slam_toolbox/deserialize_map`), never a process restart — no
  separate localization config anymore.
- `config/ekf.sim.yaml`, `config/ekf.yaml`, `config/lifelong_mapping.yaml`
  (marked "not currently used") and `maps/` moved here from base101_nav.

**`base101_nav` is now pure Nav2:**
- `launch/nav.launch.py`: planner_server (Smac2D), controller_server
  (MPPI, cmd_vel→cmd_vel_raw), bt_navigator, velocity_smoother
  (cmd_vel_raw→cmd_vel_nav), own lifecycle manager
  (`lifecycle_manager_nav`). 3 s-delayed lifecycle managers,
  bond_timeout 30 s, autostart arg.
- `behavior_trees/nav_to_pose.xml`: minimal — plan/follow/fail. No
  RecoveryNode, no retries, no spin (tower + dual arms), no wait. Nav2
  returns ABORTED; the bridge yields NavStatus(phase="stuck"); Python
  decides. The bridge clears both costmaps before each goal via the
  costmap clear services (no behavior_server to do it).
- `config/costmap.yaml`: unified for all modes. Global: rolling_window
  false, 3.0/2.0 Hz update/publish (the SLAM map GROWS in mapping mode),
  track_unknown_space true (frontier finding is a Python concern now),
  static_layer subscribe_to_updates + map_topic /map.
- package.xml/CMakeLists slimmed: ament_cmake only, exec_depends on the
  five nav2 packages actually launched (+ smac/mppi plugins). Dropped:
  nav2_amcl, nav2_behaviors, nav2_map_server (moved to base101_slam for
  map_saver_cli), slam_toolbox, robot_localization, explore_lite, rclpy.

**Independence (Cristi's ruling):** neither package references the
other. Separate lifecycle managers mean no bond coupling: Nav2 starts,
runs and dies without slam_toolbox and vice versa. The only remaining
coupling is the `/map` topic + `map->odom` TF — consumed from whatever
publishes them.

**Deleted** (sections 2, 6, 7, 9, 12): mode_manager.py + its two
launches, mapping/navigation/mapfree/slam_nav launches (8 files),
costmap_mapfree.yaml, online_mapping.yaml, slam_toolbox_localization.yaml,
amcl.yaml, explore.yaml, behavior.yaml. Also the stray
`config/{build,install,log}` dirs from an accidental in-config colcon
run. The mode_manager's jobs (mode switching, map discovery/saving,
status) move to the robocore bridge in Phase 5.

**Sim IMU (Cristi: add it):** `base_imu` gz sensor on base_link at
100 Hz → `/sensors/imu`, bridged (gz.msgs.IMU → sensor_msgs/Imu);
`gz-sim-imu-system` added to empty.sdf (sticky_floor already had it).
This makes ekf.sim.yaml's imu0 input real instead of a dead topic, and
gives robocore's imu API live sim coverage. The robocore profiles
(engine/profiles/base101*.yaml) gained `imu: {topic: /sensors/imu}`.

## Two launch bugs found and fixed during bring-up

1. **slam_toolbox bond loop under sim time.** With a nav2 lifecycle
   manager driving it, the manager reported "Server slam_toolbox
   connected with bond" then "no heartbeat for 30000 ms" ~200 ms later,
   looping deactivate/reactivate forever. The bond heartbeat misfires on
   the sim clock. Removing the manager doesn't work either:
   slam_toolbox is a LifecycleNode that does NOT self-activate — it sits
   in `unconfigured` until something drives configure->activate. Fix:
   keep the manager (it must autostart slam) but set `bond_timeout: 0.0`,
   which makes nav2 skip bond creation entirely. Manager still
   activates; no heartbeat watchdog to misfire. Crash supervision is the
   bridge's job anyway.
2. **`nav_through_poses.xml` referenced absent action servers.** It was
   the canonical nav2 recovery tree (Spin/Wait/BackUp). With no
   behavior_server, bt_navigator refused to ACTIVATE ("Action server
   spin not available") and aborted the whole nav bringup — not just
   nav_through_poses. Replaced with a minimal plan/follow/fail tree
   (RemovePassedGoals is BT-internal, kept for waypoint progress).

Process lesson worth remembering: colcon's `install(DIRECTORY)` did not
recopy edited launch/config files on an incremental build ("1 package
finished" but the installed file was unchanged), so several fixes looked
ineffective while the stack kept running the old launch. `rm -rf
build/<pkg> install/<pkg>` before rebuild forces it.

## Verification — all green against the live stack (2026-06-12)

- [x] gazebo.launch.py starts cleanly (with sim IMU + depth additions)
- [x] slam.launch.py + nav.launch.py start independently; all 5 lifecycle
      nodes reach `active`
- [x] slam_toolbox publishes /map and a live map->odom TF
- [x] NavigateToPose SUCCEEDED while mapping (drove -0.82 -> 0.38 m to a
      0.8 m goal)
- [x] Map save: serialize_map (result=0, .data+.posegraph) + map_saver_cli
      (.pgm+.yaml)
- [x] Map load: deserialize_map match_type=LOCALIZE_AT_POSE switched to
      localization; map->odom keeps publishing
- [x] NavigateToPose SUCCEEDED after the localization switch (drove to a
      (0,0.5) goal)
- [x] No behavior_server / AMCL / explore_lite / mode_manager processes
- [x] Costmap clear services callable (clean Empty response on both)
- [x] twist_mux priorities: navigation /cmd_vel_nav = 10 < agent = 50
- [x] /sensors/imu publishes (BEST_EFFORT — use that QoS to echo); EKF
      outputs /odometry/filtered fusing odom + imu

Not run (deferred): blocked-path -> ABORTED timing. The minimal BT has no
RecoveryNode so a planner/controller failure returns ABORTED by
construction; staging a live obstacle to time it belongs with the Phase 5
navigate() generator work, where the bridge maps ABORTED ->
NavStatus(phase="stuck").
