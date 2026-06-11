# robocore Phase 1 worklog — foundations

Scaffolding session for robocore: the pure-Python client SDK (`engine/` at
the monorepo root) and the agent (`src/robocore_agent`, ament_python).
Spec: `robocore-api-v0.4.md`; tracker: `robocore-implementation-plan.md`
(both at the monorepo root, one level above this repo).

## Spec version discrepancy (for Cristi)

The kickoff brief references `robocore-api-v0.5.md` with a fully RESOLVED
Open Questions section. Only **v0.4** exists on disk, with the questions
still open. Proceeded against v0.4 plus the three resolutions the brief
states explicitly (binding):

- Q3: `resume()` is **removed**.
- Q4: the live waypoint list is replaced by an explicit **Nav handle**.
- Q10: `robot.drive()` is **removed** (teleop is the only direct-velocity
  path).

None of the other open questions affect Phase 1. When v0.5 lands, diff it
against these assumptions.

## Workspace state found / fixed

- The base101 workspace had been **moved** (CMake caches pointed at
  `/home/cdr/Work/base101`, the repo now lives at
  `/home/cdr/Work/bpe/base101`), so every CMake package failed to build.
  Fixed with a clean `rm -rf build install log` + rebuild.
- `mujoco_ros2_control` fails to build (find_package error). Per Cristi
  mid-session: **MuJoCo and Isaac are out of scope for now — Gazebo only.**
  Added `COLCON_IGNORE` to `src/base101_mujoco`, `src/base101_isaac`,
  `src/mujoco_ros2_control`. Remove the markers to bring them back.
- Stale nodes from a session predating the move (an old
  `base101_dual_arm` workspace!) were still running — including a
  `robot_state_publisher`, exactly the "poisons the next ros2_control run"
  trap from `dual_arm.md`. Killed everything; the pytest harness now does
  this cleanup in teardown, every run.
- Verified `ros2 launch base101_gazebo gazebo.launch.py tower:=true
  arms:=true rosboard:=false`: all **7** controllers reach `active`
  (diff_drive, tower, left/right arm, left/right gripper,
  joint_state_broadcaster). That count is the harness's readiness gate.
- Build pinned with `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`
  per `dual_arm.md`.

## Decisions

- **Package/component naming:** the spec calls the ROS side
  `robocore_bridge`; the kickoff brief renames it "the agent". Package is
  `robocore_agent`, node name `robocore_agent`. The README cross-references
  the spec's name.
- **Shared models, one source of truth:** the agent imports
  `robocore.models` (pure pydantic) from the engine package. Dependency
  direction is agent → SDK, never the reverse; the client still imports no
  ROS anything. The agent needs `robocore` importable: `pip install -e
  engine/` for dev, and the pytest harness injects `PYTHONPATH` so tests
  work without the install.
- **Agent module split keeps rclpy quarantined in `main.py`.**
  `server.py` (transport), `handlers.py` (methods), `profile.py` (YAML)
  import no rclpy, so the wire protocol is unit-testable without a ROS
  environment — `tests/test_wire.py` runs the real server + handlers
  in-process in ~0.1 s.
- **Binary payload framing:** header notification `payload.header
  {payload_id, kind, meta}` followed by one binary WS frame = 8-byte
  big-endian payload id + raw bytes. To keep the channel tested before
  real image methods exist (Phase 4), the agent has a test-only wire
  method `debug.send_payload {size}` → random bytes + sha256. It is wire
  plumbing, not public SDK surface; remove it if it ever bothers anyone.
- **Stub profile capability derivation:** Phase 1 derives the capability
  list from which known top-level sections exist in the YAML
  (`profiles/stub.yaml` → mobility, cameras, lidar, status). Real schema
  validation is Phase 2.
- **Default port 7447** (spec's working value) and default agent socket
  `/tmp/robocore.sock` (the spec's `/run/robocore.sock` needs root or a
  systemd RuntimeDirectory; client default still tries `/run` first).
  Both marked OPEN-Q in code (spec Q9 names are unresolved).
- **`ros2 run` arg quirk:** the agent tolerates and ignores `--ros-args`
  tails via `parse_known_args` — ros2 run appends them.

## Harness notes (measured)

- Cold Gazebo start with tower + arms to all-controllers-active:
  ~40–90 s on this machine (measured). Harness timeout is 240 s.
- Readiness polling is `ros2 control list_controllers` in a sourced
  subshell; counting ` active` lines is crude but exact for this launch.
- Teardown: SIGINT to the launch's process group, 20 s grace, SIGKILL,
  then `pkill` for `gz sim`/`robot_state_publisher`/bridges that escape
  the group. Both layers are needed in practice.
- Sourcing only `base101/install/setup.bash` chains /opt/ros/jazzy and the
  mod101 underlay (colcon bakes the underlay prefix in), so the harness
  sources one file.

## What landed

- `engine/`: `robocore` package (client.py, transport.py, wire.py, uri.py,
  errors.py, models/protocol.py, version.py), `profiles/stub.yaml`,
  `scripts/gen_protocol.py` → `protocol.json`, tests (17 unit + 4 sim).
- `src/robocore_agent`: profile loader, asyncio WebSocket JSON-RPC server
  (unix + TCP from one loop), Phase 1 handlers, rclpy node skeleton.
- Phase 1 exit test: see tracker.

## Dead ends / gotchas

- websockets 12 needs a nominal `ws://localhost/` URI argument for
  `unix_connect`; without it the client's Host header logic trips.
- `pkill -f` patterns must not match the invoking shell — use the
  `[x]`-bracket trick (re-learned from dual_arm.md the easy way).
