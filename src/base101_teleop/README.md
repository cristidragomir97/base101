# base101_teleop

Single-page web teleop for the base101 dual-arm robot. One panel with
everything: a hold-to-drive base pad plus a position slider for every
controlled joint — lift, pan/tilt head, both mod101 arms, both grippers.

Zero dependencies beyond `rclpy`: one python file with the HTML embedded,
served by the stdlib HTTP server. This is the standalone fallback for the
rosboard **Joint sliders** card (which is the recommended UI — see the
workspace README).

## Run

```bash
ros2 run base101_teleop server                       # http://localhost:8700/
ros2 run base101_teleop server --ros-args -p port:=9000
```

## What it publishes

| UI | Topic | Type |
|---|---|---|
| Base pad | `/cmd_vel_key` (twist_mux, priority 90) | `geometry_msgs/Twist` |
| Tower sliders | `/tower_controller/commands` | `std_msgs/Float64MultiArray` `[lift, head_pan, head_tilt]` |
| Arm sliders | `/<side>_arm_controller/commands` | `Float64MultiArray` `[<side>_arm_1 … _5]` |
| Gripper slider | `/<side>_gripper_controller/commands` | `Float64MultiArray` `[<side>_arm_6]` |

Sliders always send the **full group array** (untouched joints keep their
slider value). The base pad republishes at 10 Hz while held and zeroes on
release; twist_mux's 0.5 s timeout acts as the dead-man stop.

## Behavior notes

- The page builds itself from a `/joint_states` snapshot (`GET /state`), so
  sections for hardware that isn't loaded (no tower, no arms) don't render,
  and sliders initialise to the robot's actual pose. "Sync sliders to
  robot" re-pulls at any time.
- Plain HTTP polling — no rosbridge, no websockets. `POST /cmd` carries the
  commands; see the docstring in `base101_teleop/server.py` for payload
  shapes.
