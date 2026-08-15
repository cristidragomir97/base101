# Running base101 on real hardware

base101's motors, IMU and lidar are driven by the **Axon 2 board (RP2354B)**
running the [`axon-firmware`](../axon-firmware). The firmware is a native ROS 2
node (Pico-ROS + zenoh-pico, compatible with `rmw_zenoh`): the host talks ROS
topics, not raw serial. On the host side, `ros2_control` runs the usual
`diff_drive_controller`, and `base101_control_plugin` bridges its per-wheel
command/state interfaces to the firmware's motor-manager topics.

```
diff_drive_controller ──▶ base101_control_plugin/ROS2ControlBridge
                              │  pub /motor_manager/base_cmd   (4× wheel vel, rad/s)
                              │  sub /motor_manager/joint_states
                              ▼      ── zenoh ──▶  Axon 2 firmware ──▶ 4× DDSM210
```

## Topic contract (firmware ⇄ host)

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/motor_manager/base_cmd` | `std_msgs/Float64MultiArray` | host→fw | 4 wheel velocities (rad/s), order **[FL, FR, BL, BR]** |
| `/motor_manager/joint_states` | `sensor_msgs/JointState` | fw→host | wheels in slots 0–3, 50 Hz |
| `/imu/data`, `/imu/mag`, `/imu/temperature` | `Imu` / `MagneticField` / `Temperature` | fw→host | BNO055, frame `imu_link`, 50 Hz |

**Joint names + order are a contract** with `axon-firmware/src/ros/axon_config.h`:

- names: `front_left_wheel_joint`, `front_right_wheel_joint`,
  `back_left_wheel_joint`, `back_right_wheel_joint` (used by both the URDF and
  the firmware's `joint_states`);
- `base_cmd` index order `[0]=FL [1]=FR [2]=BL [3]=BR` — this is the order the
  wheel `<joint>`s appear in `base101_control/urdf/base101.hardware.xacro`.

If you change either side, change both.

## Host one-time setup

1. **udev rules** — exposes the board as stable device names:
   ```
   cd ~/Work/axon-firmware && ./install.sh
   #  /dev/axon-zenoh  zenoh serial transport
   #  /dev/axon-lidar  RPLidar C1 UART passthrough
   #  /dev/axon-debug  firmware debug log
   ```
2. **zenoh router** — bridges the board's serial zenoh to the host's
   `rmw_zenoh` sessions (TCP 7447). Use the firmware's compose file:
   ```
   cd ~/Work/axon-firmware/docker && docker compose up -d   # uses zenoh-serial.json5
   ```
3. **rmw_zenoh** — every ROS 2 shell that should see the board:
   ```
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

## Bring up the base

```
source /opt/ros/jazzy/setup.bash
source ~/Work/base101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

ros2 launch base101_control control_stack.launch.py variant:=simple   # or pro
```

This starts `robot_state_publisher` (from `base101.hardware.xacro`, i.e.
`simulator:=none` + the Axon bridge), the `controller_manager` with
`controllers.hw.yaml`, the `joint_state_broadcaster` + `diff_drive_controller`,
and a `twist_mux` in front of `/diff_drive_controller/cmd_vel`.

Drive it:
```
ros2 topic pub /cmd_vel_key geometry_msgs/msg/Twist '{linear: {x: 0.1}}' -r 10
```

### Verify
- `ros2 control list_hardware_components` → `base101_hw_system` **active**.
- `ros2 control list_controllers` → `diff_drive_controller` + `joint_state_broadcaster` **active**.
- `ros2 topic echo /motor_manager/base_cmd` reacts to `cmd_vel`.
- `ros2 topic echo /joint_states` shows the four wheels turning; `/odom` is published.
- `ros2 topic echo /imu/data` streams from the BNO055.

### Wheel direction calibration
If forward/back or turning is inverted, flip the wheel `direction` rows in the
firmware's `axon_config.h` (all four for fwd/back; per-side for spin) and
reflash — the host side stays unchanged.

## Sensors

- **Lidar (RPLidar C1)** on the firmware passthrough port:
  ```
  ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/axon-lidar -p frame_id:=lidar_frame
  ```
- **IMU** needs no driver — the firmware publishes `/imu/data` directly.

## Navigation / SLAM

`base101_slam` and `base101_nav` run on top of the base unchanged. When you run
their EKF (`robot_localization`, fusing wheel odom + `/imu/data`), set
`diff_drive_controller.enable_odom_tf: false` in `controllers.hw.yaml` so the
EKF owns the `odom → base_link` transform (otherwise two nodes publish it).
Standalone driving (no nav) keeps `enable_odom_tf: true`.

## Tower / arms

The deck-mounted mod101 arm (`base101_arm_*`) is **sim-only** for now. The Axon
firmware does expose `/motor_manager/arm_cmd` (ST3215 servos) for a future arm
bring-up.

The cross tower is parked out of the build in [`attic/`](attic/README.md) — its
deck mount no longer matches the re-exported chassis, and no real driver was
ever wired for its lift / pan / tilt.
