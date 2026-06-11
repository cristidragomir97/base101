# Dual mod101 arms worklog

Notes from mounting two mod101 arms on the cross tower's crossbeam brackets.
Current state is `base101_description/urdf/base101_arms.xacro` +
`base101_control/config/controllers.arms.yaml` at HEAD. The tower itself has
its own worklog: [`tower.md`](tower.md).

## Design constraints

- mod101 must stay a **standalone repo** — no base101 knowledge inside it,
  and its single-arm bringup must keep working unchanged.
- Two instances of the same arm in one URDF means every link, joint,
  ros2_control system, gazebo reference, and sensor topic needs a unique
  name.

## mod101 refactor (done in the mod101 repo)

`mod101_description/urdf/mod101.xacro` was a flat 776-line robot file with
joints named `1…5` and a hardcoded `world` anchor. It was split into:

- **`mod101_macro.xacro`** — `xacro:macro mod101_arm(prefix, parent, xyz,
  rpy, tool, use_sim)`. Every link/joint name gets `${prefix}`; a fixed
  `${prefix}base_mount` joint attaches `${prefix}base_link` to `parent`
  (empty `parent` = no mount joint). The per-instance ros2_control block,
  gazebo link extensions (incl. the wrist camera, topic
  `${prefix}wrist_camera/image_raw`), and the selected tool are all emitted
  inside the macro.
- **`mod101.xacro`** — thin standalone wrapper: same `use_sim`/`tool` args
  as before, the `world` anchor, the (once-per-robot) gz_ros2_control
  plugin block, and one `prefix=""` macro call.
- The four tool packages' `tool.urdf.xacro` / `tool.ros2control` /
  `tool.gazebo` became macros too (`mod101_tool_<name>[...](prefix,
  use_sim)`), with `$(arg use_sim)` replaced by the macro param.

The conversion was script-driven (regex prefixing of `name=` / `parent
link=` / `child link=` / `mimic joint=` attributes) and verified by
canonical XML diff: the standalone URDF is **semantically identical** before
vs. after, except the anchor joint rename `world_to_base → base_mount`
(nothing referenced the old name).

## Mount geometry

Measured from the meshes, not guessed: the brackets are 80×120×30 mm slabs;
the mod101 base plate is 80×100 mm and sits centred on each bracket's top
face. In the bracket frames that's `(0, +0.06, 0.024)` (left) /
`(0, -0.06, 0.024)` (right). Mount yaw **0** = arms reach toward the robot's
front; the first guess of yaw π pointed them backwards (the arm's ready
stance reaches opposite to its CAD-zero direction — empirically corrected).

## Integration points (this workspace)

- `arms` + `arm_tool` xacro args in `base101.xacro`;
  `base101_arms.xacro` instantiates `left_arm_` / `right_arm_`.
  `arms:=true` requires `tower:=true` (the brackets are tower links).
- Arms get sim ros2_control only for `simulator == gazebo`
  (`arms_use_sim` property); mujoco scenes don't model them yet.
- `controllers.arms.yaml` (left/right arm + gripper position controllers)
  is added to the gz_ros2_control plugin's `<parameters>` list behind
  `$(arg arms)` in both variant `.gazebo` files; spawners + per-side wrist
  camera image bridges are conditional in `gazebo.launch.py`.
- `GZ_SIM_RESOURCE_PATH` must include the mod101 underlay's share dirs —
  the **tool** meshes use `package://mod101_tool_<name>/...` URIs (the arm
  meshes are `file://$(find ...)` and resolve at xacro time). Forgetting
  this = invisible grippers + "Failed to load geometry" for jaws parts.
- Build: mod101 underlay first, then this workspace with
  `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` (a stray
  `~/.local/bin/python3.11` on PATH otherwise breaks `ament_package_xml`
  and rosidl's `import em`).

## Bugs & gotchas hit along the way

- **Lift sagged to its bottom stop** once the arms were added: carriage +
  arms ≈ 12 kg ≈ 118 N > the joint's 100 N effort limit. Raised to 400 N
  (see tower worklog).
- **Stale `robot_state_publisher` poisons ros2_control.** controller_manager
  takes the *first* `/robot_description` it hears and logs
  `ResourceManager has already loaded a urdf. Ignoring attempt to reload`
  for the real one. Symptom: spawners die with `command interface
  left_arm_1/position is not available` while the URDF is fine. Kill every
  node from previous sim runs before relaunching — and when scripting the
  cleanup, use patterns that can't match your own shell
  (`pgrep -f 'rosboard_nod[e]'`-style bracket trick), or the `pkill -f`
  kills the cleanup script itself.
- gz_ros2_control happily initialises **five** hardware systems from one
  URDF (base + 2× arm + 2× tool) under a single controller_manager — no
  special handling needed beyond unique system names (`${prefix}…`).

## Teleop

- `base101_teleop` package: stdlib-only HTTP server (`:8700`) + one
  embedded HTML page; sliders publish full group arrays to the controller
  topics, base pad publishes Twist to `/cmd_vel_key` at 10 Hz while held
  (twist_mux's 0.5 s timeout is the dead-man).
- rosboard **Joint sliders** card (`JointCommandViewer.js`): a real
  `/joint_states` subscriber viewer that publishes
  `std_msgs/Float64MultiArray` over the existing `MSG_PUB` websocket
  channel. Backend changes: type added to `publish_allowlist` +
  `_dict_to_ros` converter. The zero-on-silence publish watchdog only
  handles Twist types **by design** — zeroing a position-command topic
  would slam every joint to 0. `initSubscribe()` gained an optional
  `viewerName` override (persisted) so the System-nav entry can open
  `/joint_states` with the slider card instead of the default
  `JointStateViewer`.
