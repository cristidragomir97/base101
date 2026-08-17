# Manual test guide

Every combination this robot ships, how to launch it, and what a pass looks
like. Written to be worked through top to bottom after a change that touches the
description, the controllers, MoveIt, or the mod101 integration.

Section 1 is offline and takes seconds — run it first, it catches most
breakage. Sections 2-6 need a sim. Section 7 is the independence check that
must pass before you ship anything that touched mod101.

## 0. Build

mod101 is an **underlay**: build and source it first, always. Full rules and the
reason for the `Python3_EXECUTABLE` pin are in the
[README's Build section](../README.md#build) — this is the same thing, in one
paste:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/robots/mod101 && colcon build --symlink-install && source install/setup.bash
cd ~/robots/base101
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Expect 8 packages in mod101 and 15 in base101. `base101_control_plugin`
produces stderr output during build; that is pre-existing and not a failure.

If you changed a mesh or deleted a file, `--symlink-install` will trip over the
stale symlink. Clear the one package:

```bash
rm -rf build/<pkg> install/<pkg> && colcon build --symlink-install
```

## 1. Offline checks (no sim, ~30 s)

This is the gate. Everything here is pure xacro expansion, so it catches
malformed XML, missing meshes, renamed joints, broken args, and SRDF/URDF
disagreement without waiting for Gazebo.

```bash
cd ~/robots/base101
source /opt/ros/jazzy/setup.bash
source ~/robots/mod101/install/setup.bash
source install/setup.bash

# --- simple variant: 2 cameras x 3 simulators
for cam in realsense oak_d; do for sim in none gazebo mujoco; do
  xacro src/base101/base101_simple_description/urdf/base101_simple.xacro \
        simulator:=$sim camera:=$cam >/dev/null \
    && printf "ok simple/%s/%s  " $sim $cam || printf "FAIL simple/%s/%s  " $sim $cam
done; done; echo

# --- arm variant: 4 tools x 2 simulators, plus arm:=false
for tool in jaws parallel none pincopen; do for sim in none gazebo; do
  xacro src/base101_arm/base101_arm_description/urdf/base101_arm.xacro \
        simulator:=$sim arm:=true arm_tool:=$tool >/dev/null \
    && printf "ok arm/%s/%s  " $sim $tool || printf "FAIL arm/%s/%s  " $sim $tool
done; done
xacro src/base101_arm/base101_arm_description/urdf/base101_arm.xacro \
      simulator:=none arm:=false >/dev/null && printf "ok arm:=false"; echo

# --- MoveIt semantics: 4 tools
for tool in jaws parallel none pincopen; do
  xacro src/base101_arm/base101_arm_moveit_config/srdf/base101_arm.srdf.xacro \
        arm_tool:=$tool >/dev/null \
    && printf "ok srdf/%s  " $tool || printf "FAIL srdf/%s  " $tool
done; echo

# --- every launch file resolves its args
for l in "base101_simple_gazebo gazebo.launch.py" \
         "base101_simple_description display.launch.py" \
         "base101_arm_gazebo gazebo.launch.py" \
         "base101_arm_description display.launch.py" \
         "base101_arm_moveit_config move_group.launch.py" \
         "base101_arm_moveit_config demo.launch.py"; do
  printf "%-52s " "$l"
  timeout 90 ros2 launch $l --show-args >/dev/null 2>&1 && echo ok || echo FAIL
done

# --- kinematic tree is sane
xacro src/base101_arm/base101_arm_description/urdf/base101_arm.xacro \
      simulator:=gazebo arm:=true > /tmp/base101_arm.urdf
check_urdf /tmp/base101_arm.urdf | head -3
```

**Pass:** every line `ok`, and `check_urdf` reports
`root Link: base_link has 16 child(ren)`.

### 1b. Controller/URDF agreement

The failure this catches is silent at expansion time and fatal at runtime: a
controller naming a joint that doesn't exist just fails to activate, and the arm
doesn't move with no obvious error.

```bash
python3 - <<'EOF'
import xml.etree.ElementTree as ET, yaml
r = ET.parse('/tmp/base101_arm.urdf').getroot()
joints = {j.get('name') for j in r.findall('joint')}
rcj = {j.get('name') for rc in r.findall('ros2_control') for j in rc.findall('joint')}
y = yaml.safe_load(open('src/base101_arm/base101_arm_control/config/controllers.sim.yaml'))
for n, c in y.items():
    if n == 'controller_manager': continue
    pr = c.get('ros__parameters', {})
    js = (pr.get('joints') or []) + pr.get('left_wheel_names', []) + pr.get('right_wheel_names', [])
    bad = [j for j in js if j not in joints or j not in rcj]
    print(f"  {n:30s} {'OK' if not bad else 'BAD ' + str(bad)}")
EOF
```

**Pass:** all six controllers `OK`.

## 2. Simple variant

```bash
ros2 launch base101_simple_gazebo gazebo.launch.py
```

| check | how | pass |
|---|---|---|
| model spawns | Gazebo window | chassis on wheels, black plastics, dark-grey extrusion, lidar at the front |
| controllers | `ros2 control list_controllers` | `joint_state_broadcaster`, `diff_drive_controller` both `active` |
| driving | rosboard `http://localhost:8888/` Teleop card, or `ros2 topic pub /cmd_vel_key geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` | robot drives forward, doesn't spin or crab |
| lidar | `ros2 topic hz /scan` | ~10 Hz |
| camera | `ros2 topic hz /base_camera/image_raw` and `/base_camera/depth_image` | ~30 Hz each |
| IMU | `ros2 topic hz /sensors/imu` | ~100 Hz |
| frames | `ros2 run tf2_tools view_frames` | `base_link` root; `lidar_frame`, `camera_link`, `camera_optical_frame`, `imu_link` present |

Camera toggle — the mesh and the simulated FOV change, the topics do not:

```bash
ros2 launch base101_simple_gazebo gazebo.launch.py camera:=oak_d
```

**Pass:** the front module is visibly shorter, `/base_camera/*` topics are
identical, and `camera_link` still exists.

RViz only, no sim:

```bash
ros2 launch base101_simple_description display.launch.py            # joint sliders GUI
ros2 launch base101_simple_description display.launch.py gui:=false camera:=oak_d
```

## 3. Arm variant — slider control

```bash
ros2 launch base101_arm_gazebo gazebo.launch.py                 # arm:=true is the default
```

| check | how | pass |
|---|---|---|
| arm mounted | Gazebo | arm base flat on the deck, centred, not floating |
| controllers | `ros2 control list_controllers` | `arm_controller` and `gripper_controller` `active`; the `*_trajectory_controller` pair NOT loaded |
| rosboard sliders | `http://localhost:8888/` → "Joint sliders" | an **arm** group of 5 and a **gripper** group of 1 render; tower groups do not |
| teleop sliders | `http://localhost:8700/` | same groups |
| motion | drag a slider | the corresponding joint moves in Gazebo |
| wrist camera | `ros2 topic hz /arm_wrist_camera/image_raw` | publishing |

Direct command, bypassing the UIs:

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 1.2, 1.5, 0.3, 0.0]}"     # the SRDF 'ready' pose
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [2.0]}"                          # jaws open
```

Every tool:

```bash
for t in jaws parallel pincopen none; do
  ros2 launch base101_arm_gazebo gazebo.launch.py arm_tool:=$t
done
```

**Pass:** each end-effector appears; `gripper_controller` is absent for
`none`; on `parallel` and `pincopen` the extra jaw joints mimic `arm_6`, so one
command moves the whole gripper.

## 4. Arm variant — MoveIt planning

```bash
ros2 launch base101_arm_moveit_config demo.launch.py
```

That is Gazebo with `arm_control:=moveit` plus `move_group`, in order. To drive
a sim you already have up, in two terminals:

```bash
ros2 launch base101_arm_gazebo gazebo.launch.py arm_control:=moveit
ros2 launch base101_arm_moveit_config move_group.launch.py
```

| check | how | pass |
|---|---|---|
| controllers swapped | `ros2 control list_controllers` | `arm_trajectory_controller`, `gripper_trajectory_controller` `active`; the position pair NOT loaded |
| actions offered | `ros2 action list \| grep follow_joint_trajectory` | two actions |
| move_group up | its console | `Successfully loaded planner 'OMPL'`, `Added FollowJointTrajectory controller for arm_trajectory_controller` ×2, then **`You can start planning now!`** |
| groups | RViz MotionPlanning → Planning Group | `arm_arm` and `arm_gripper` |
| named states | Goal State dropdown | `home`, `ready` on the arm; `open`, `closed` on the gripper |
| plan | `home` → `ready`, Plan | a trajectory is found in well under a second |
| execute | Plan & Execute | the arm follows it in Gazebo |

The only expected error at startup is
`No 3D sensor plugin(s) defined for octomap updates` — there is deliberately no
perception in the planning scene (see
[obstacle-awareness.md](obstacle-awareness.md)).

### 4b. The collision test that matters

This is the whole reason `base101_arm_moveit_config` exists, so test it
explicitly rather than assuming.

In RViz, drag the interactive marker to put the gripper **inside the lidar** —
low and forward, around `x = 0.14, y = 0.0, z = 0.10` in `base_link`. Then Plan.

**Pass:** the goal state shows in collision (the arm renders red) and planning
fails, or the planner routes around. **Fail:** it plans a straight line through
the lidar — which means the collision matrix over-disabled, and you should
regenerate with more trials (section 5c).

Also worth checking the inverse, that it is not *over*-constrained: `home` →
`ready` must still plan. If everything fails to plan, the matrix is
under-disabled and adjacent links are reporting permanent collision.

### 4c. Automated smoke test

mod101 ships `test/moveit_smoke.py` (FK → IK → impossible-IK → joint-space
plan). It hardcodes the unprefixed group and joint names, so against base101 it
needs `arm_arm` / `arm_joint_*` / `arm_wrist_flange`. Run the mod101 original
against mod101, and adapt it if you want the same gate here.

## 5. Configurator round-trip

The configurator is the source of truth for the arm's build: rail lengths,
servo mounts, and tool. base101 reads the same file, so a change must reach
both.

```bash
cd ~/robots/mod101
python3 configurator/server.py          # http://localhost:8000/
```

The configurator regenerates **mod101's** matrices only. base101's are this
workspace's responsibility — see 5b.

### 5a. It propagates

Set shoulder 0.21 m / elbow 0.19 m, pick a big shoulder motor, pick the parallel
tool, Save. Then:

```bash
grep default= ~/robots/mod101/src/mod101_description/urdf/mod101_config.xacro
```

**Pass:** `shoulder_ext_length="0.2100"`, `elbow_ext_length="0.1900"`,
`shoulder_mount="big"`, `tool="parallel"`.

Rebuild both workspaces, then confirm both robots changed together:

```bash
xacro ~/robots/mod101/src/mod101_description/urdf/mod101.xacro | grep -m1 'box size="0.21'
xacro src/base101_arm/base101_arm_description/urdf/base101_arm.xacro \
      simulator:=none arm:=true | grep -m1 'box size="0.21'
```

**Pass:** both match. **Fail:** base101 shows 0.082/0.098 — the macro's own
defaults — meaning `arm.xacro` stopped passing the build args through.

The launch defaults follow too:

```bash
ros2 launch base101_arm_gazebo gazebo.launch.py --show-args | grep -A2 "'arm_tool'"
```

**Pass:** `(default: 'parallel')`.

### 5b. It regenerates the collision matrices

Saving kicks a background regeneration of **mod101's** matrices. Watch the
configurator console for `[collisions] mod101: regenerated`, or poll:

```bash
curl -s localhost:8000/collisions
```

The same response carries a `downstream` note reminding you that consumers
regenerate themselves. Do that now, from this workspace:

```bash
./src/base101_arm/base101_arm_moveit_config/scripts/sync_arm_change.sh
```

**Pass:** it prints the mod101 underlay it resolved, then a per-tool pair count.
**Fail:** `mod101 underlay not built` — build mod101, or set `MOD101_WS`.

Then check the stamps:

```bash
grep -m1 shoulder_ext_length \
  ~/robots/mod101/src/mod101_moveit_config/config/collisions/jaws.srdf.xacro
grep -m1 shoulder_ext_length \
  src/base101_arm/base101_arm_moveit_config/config/collisions/jaws.srdf.xacro
```

**Pass:** both headers carry the values you just saved. A stale stamp means the
matrix no longer describes the arm you are planning for.

### 5c. Regenerating by hand

```bash
# base101 (chassis pairs) — all four tools, minutes at the 1M default
python3 src/base101_arm/base101_arm_moveit_config/scripts/gen_collision_matrix.py

# mod101 (arm-internal pairs)
python3 ~/robots/mod101/tools/gen_collision_matrix.py --trials 1000000
```

If you increased the arm's reach, re-run with more trials and diff — "never
colliding" is a sampling result, not a proof:

```bash
cp src/base101_arm/base101_arm_moveit_config/config/collisions/jaws.srdf.xacro /tmp/before
python3 src/base101_arm/base101_arm_moveit_config/scripts/gen_collision_matrix.py \
        --tool jaws --trials 4000000
diff <(grep -o 'link1="[^"]*" link2="[^"]*"' /tmp/before) \
     <(grep -o 'link1="[^"]*" link2="[^"]*"' \
       src/base101_arm/base101_arm_moveit_config/config/collisions/jaws.srdf.xacro)
```

Lines only in `/tmp/before` are pairs the lower trial count wrongly disabled.
Expect a handful; if it's dozens, the default needs raising.

## 6. Real hardware (simple variant only)

The arm is sim-only — `base101_arm_control` has no `controllers.hw.yaml`.

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch base101_simple_control control_stack.launch.py
```

See [HARDWARE.md](../HARDWARE.md). Note `controllers.hw.yaml` carries its own
`wheel_separation` (hardware-calibrated, deliberately not the URDF value).

## 7. mod101 independence

**Run this before shipping anything that touched mod101.** base101 is a
downstream consumer; mod101 must not have acquired a dependency on it.

```bash
env -i HOME=$HOME USER=$USER PATH=/usr/bin:/bin bash -lc '
source /opt/ros/jazzy/setup.bash
source ~/robots/mod101/install/setup.bash

echo "base101 entries on the path: $(echo $AMENT_PREFIX_PATH | tr : "\n" | grep -c base101)"

for t in jaws parallel none pincopen; do for m in small big; do
  xacro ~/robots/mod101/src/mod101_description/urdf/mod101.xacro \
        tool:=$t shoulder_mount:=$m elbow_mount:=$m >/dev/null \
    && printf "ok urdf/%s/%s " $t $m || printf "FAIL urdf/%s/%s " $t $m
done; done; echo

for t in jaws parallel none pincopen; do
  xacro ~/robots/mod101/src/mod101_moveit_config/srdf/mod101.srdf.xacro tool:=$t >/dev/null \
    && printf "ok srdf/%s " $t || printf "FAIL srdf/%s " $t
done; echo

for f in move_group mock demo; do
  timeout 60 ros2 launch mod101_moveit_config $f.launch.py --show-args >/dev/null 2>&1 \
    && printf "ok launch/%s " $f || printf "FAIL launch/%s " $f
done; echo
'
```

**Pass:** `base101 entries on the path: 0` and every check `ok`.

Also confirm the configurator works with no base101:

```bash
python3 -c "
import importlib.util
s = importlib.util.spec_from_file_location('c', '$HOME/robots/mod101/configurator/server.py')
m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
print('targets:', [l for l,_,_ in m._regen_targets()])
print('downstream note:', bool(m.DOWNSTREAM_NOTE))"
```

**Pass:** `targets: ['mod101']` and `downstream note: True`.

This is now an invariant rather than a configuration: the configurator has no
consumer-workspace setting at all, so there is nothing to unset. If a second
label ever appears in `targets`, the dependency has been inverted again.

And the standalone MoveIt bringup still plans:

```bash
ros2 launch mod101_moveit_config mock.launch.py rviz:=false
python3 $(ros2 pkg prefix --share mod101_moveit_config)/test/moveit_smoke.py
```

## 8. Reference values

Compare against these when something looks off. All measured at
140/140 mm rails, small mounts, jaws.

| quantity | value |
|---|---|
| ground clearance to `base_link` | 0 mm (`base_link` is on the wheel contact plane) |
| extrusion frame | z 22–82 mm |
| base deck (`base_link` mesh) | 340 × 240 × 3 mm, z 82–85 |
| payload deck (`top_plate_1`) | 180 × 240 × 3 mm, z 130–133 |
| arm base plate | 100 × 120 mm, centred, z 133–186 |
| turret axis | (2, 1, 186) mm, reach ~525 mm |
| wheel radius / track / wheelbase | 0.036 / 0.2899 / 0.2393 m |
| `lidar_frame` | (117.4, 0, 96.5) mm |
| `camera_link` (realsense) | (199.1, 0, 68.5) mm |
| `imu_link` | (−9.9, −0.1, 99.0) mm |
| composed robot links | 61 (arm variant, jaws) |
| collision triangles | 0 chassis, ~5.4k total (gripper meshes only) |
| visual triangles | ~92k (chassis + arm) |
| disable_collisions in the SRDF | ~1022 (167 arm-internal + 855 chassis, tool=jaws) |

## 9. What failure looks like

| symptom | likely cause |
|---|---|
| arm renders no sliders in either web UI | the UI group table doesn't match the controller's joint names |
| controller stays `inactive`, arm never moves | `controllers.sim.yaml` names a joint the URDF doesn't have — run check 1b |
| MoveIt plans but execution does nothing | sim came up with `arm_control:=sliders`; the trajectory controllers aren't running |
| every plan fails, even `home`→`ready` | collision matrix under-disabled; adjacent links report permanent collision |
| planner routes through the chassis | matrix over-disabled; regenerate with more trials (5c) |
| base101 arm is a different size from mod101's | `arm.xacro` stopped passing the four build args; it falls back to macro defaults silently |
| `xacro: unterminated`/`not well-formed` in a comment | a `--` inside an XML comment; not legal, and it bites often here |
| build fails on a missing symlink | stale `--symlink-install` artifact; `rm -rf build/<pkg> install/<pkg>` |
