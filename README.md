# base101

<p align="center">
  <img src="img/simple.webp" alt="simple variant" width="48%" />
  <img src="img/pro_mirrored.webp" alt="pro variant" width="48%" />
</p>

An open-source 4WD mobile robot platform designed to carry the mod101 arm and other payloads. Built from 60×20 aluminum extrusion, PLA-CF printed parts, and Waveshare DDSM hub motors.

Two configurations, same chassis, different motors.

## At a Glance

| | base101 | base101 PRO |
|---|---|---|
| Motors | 4× DDSM210 | 4× DDSM115 |
| Drive | 4WD skid steer | 4WD skid steer |
| Torque (per motor) | 0.85 N·m stall | 2.0 N·m stall |
| Load capacity | ~12kg total | ~30kg total |
| Suspension | PLA-CF printed | PLA-CF printed |
| Footprint | 280 × 400mm | 280 × 400mm |
| Lidar | RPLidar C1 | RPLidar C1 |
| Depth camera | RealSense D435 | RealSense D435 |
| BOM | ~$340 | ~$480 |

Both configurations share the same chassis, top plate, bumpers, electronics, and software. The upgrade from base to PRO is a motor and bracket swap.

## Design Philosophy

Inspired by the Roland System 100 — a modular synth from 1975 built as separate compatible units (101, 102, 103, 104). The base101 follows the same principle: a self-contained module that connects to the mod101 arm, Axon controller, and Bolt power hub to form a complete system.

Each module works alone. Together they're a mobile manipulator.

## Chassis

**Frame:** 60×20 aluminum extrusion, black anodized. Two parallel rails connected at the corners by PLA-CF printed mounts. The thin profile keeps the center of gravity low — this is a MacBook Air, not a brick.

**Top plate:** 280×400mm aluminum, 3mm thick, CNC-machined grid of M3 tapped holes on 20mm spacing. Mount anything anywhere — the arm, the lidar, the compute, accessories. No adapters, no T-nuts, just bolt it down.

**Bumpers:** TPU printed corner pieces. Absorb collisions, protect furniture, and visually soften the rectangular chassis. Press-fit onto the extrusion corners, no fasteners needed.

**Handle:** Rear-mounted carry handle, Mac Pro G5 inspired. Doubles as an emergency stop — a normally-closed microswitch inside the handle triggers when squeezed hard. Panic grip = instant motor cut. Mechanical, failsafe, no software in the loop.

## Suspension

PLA-CF printed suspension brackets, derived from Waveshare's UGV suspension design (STEP files available). The structural brackets are reprinted in PLA-CF at 6mm wall thickness. Only the compression springs are purchased — generic, bulk, cents each.

Cost per suspension unit: ~$1.50 (vs $20 retail for the metal version). Same functionality, custom-fit to the 60×20 frame.

## Drivetrain

**4WD skid steer.** Four hub motors, all driven. Turn-in-place capability. No mecanum wheels, no omni wheels — just rubber tires on smooth direct-drive motors. Quiet, clean, simple kinematics.

Why not omnidirectional? Omni wheels are noisy, collect debris, provide minimal benefit over turn-in-place skid steer, complicate the nav stack, and cost more. On a mobile manipulator, the arm does the fine positioning — the base just needs to get close and stop.

### DDSM210 (base101)

- 0.25 N·m rated / 0.85 N·m stall
- ~65mm diameter, 216g
- UART bus, 9-28V
- ~$25 each

Sized for a 5-8kg robot. Four wheels provide 97 N total tractive force at stall — enough to move 8kg up a 10% incline.

### DDSM115 (base101 PRO)

- 0.96 N·m rated / 2.0 N·m stall
- ~115mm diameter, 765g
- RS485 bus, 12-24V
- ~$60 each

Same motor as the DFRobot M0601 (same Shenzhen factory, different sticker). 4× the torque of the DDSM210 for applications requiring heavier payloads or outdoor use.

## Sensors

### RPLidar C1

Mounted on the top plate. 360° DTOF scanning, 12m range, 5000 samples/sec. Handles SLAM, mapping, and obstacle detection. ROS2 driver available out of the box.

### Intel RealSense D435

Front-mounted between the extrusion rails. 87° wide FOV for spatial awareness and 3D perception. Provides point cloud data for obstacle avoidance and workspace mapping. The wide field of view captures the arm's entire workspace in front of the robot.

The D435 was chosen over the D415 for its wider FOV. At the arm's working distance (<502mm), both cameras have equivalent depth accuracy. The D435's extra peripheral vision matters more than the D415's marginal precision advantage.

### Wrist Camera

Not on the base — it's on the mod101 arm as a quick-change end effector or last-link mount. A simple USB camera provides close-up RGB for grasp detection. Detections are projected into the RealSense coordinate frame via the arm's TF chain for exact 3D positioning.

## Electronics

| Component | Role |
|---|---|
| Axon | Motor control (UART/RS485), Feetech servo bus for mod101 arm |
| Bolt | Power distribution, 4S LiPo to all subsystems |
| Jetson Orin Nano | Compute: ROS2, Nav2, MoveIt, perception |
| JK BMS (4S) | Battery cell balancing and protection |

All electronics mount to the underside of the top plate or inside the extrusion cavity using the tapped M3 hole grid. No separate electronics tray.

## Power

4S LiPo RC battery pack, 16-20Ah, 25C+ discharge rating. XT60 connector. Provides 14.8V nominal with high burst current capability for motor stall events and fast arm movements.

LiPo chosen over LiFePO4 for higher voltage (14.8V vs 12.8V) and better peak current delivery. The JK BMS handles cell balancing, over-discharge protection, and over-current cutoff.

Estimated runtime: 2-4 hours depending on usage pattern (continuous driving vs intermittent manipulation).

## Motor Wiring

The DDSM210 uses UART, the DDSM115 uses RS485. Axon supports both protocols natively. Motor wiring goes directly from Bolt to each motor — no daisy-chaining power through the signal connectors.

## Software

ROS 2 Jazzy workspace. The `diff_drive_controller` handles skid-steer kinematics for both DDSM210 and DDSM115 configurations — the only parameter change is wheel diameter and separation.

### Packages

| Package | Type | Purpose |
|---|---|---|
| `base101_description` | ament_python | Unified URDF (simple/pro selectable via xacro arg), meshes, RViz config. |
| `base101_control` | ament_cmake | `diff_drive_controller` + `twist_mux` config, hardware bringup launch. |
| `base101_gazebo` | ament_cmake | Gazebo Sim worlds, launch, ros↔gz bridge. |
| `base101_teleop_web` | ament_python | Browser-based virtual joystick → `/cmd_vel_joy`. |
| `rosboard` | ament_python | Vendored web dashboard (publishes a Teleop card too). |
| `base101_{pro,simple}_description` | ament_python | Legacy pre-consolidation stubs — likely deletable. |

### Quickstart

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch base101_gazebo gazebo.launch.py        # simple variant, sticky_floor world
ros2 launch base101_gazebo gazebo.launch.py variant:=pro world:=empty.sdf
```

Web teleop is at `http://localhost:8888/` (rosboard) once the sim is up.

### Sim notes (gotchas learned the hard way)

- **Physics engine:** worlds must use DART, not `bullet-featherstone`. The
  latter ignores anisotropic friction (`mu2`, `fdir1`) generated from URDF
  `<gazebo reference>` tags, which breaks 4-wheel skid-steer turning.
- **Wheel friction:** the four `wheel_outside_*` links use `mu1=1.0, mu2=0.1,
  fdir1=1 0 0` so wheels grip forward but can scrub sideways during rotation.
- **Wheel coplanarity:** rear wheel joints (`Rigid 56`, `Rigid 61`,
  `wheel_rear_left`, `wheel_rear_right`) were patched to share the same Z
  height as the fronts. Original CAD export had a ~0.3 mm asymmetry that
  caused chassis pitch and front-biased rotation.
- **Lateral symmetry:** `box_right_1` / `box_left_1` Y offsets were shifted
  to put `base_link` on the wheel midline (original CAD had base_link 8 mm
  off-center, producing odom-drift arcs during rotation). `wheel_separation`
  in the controller config matches the new geometry (0.2886 m).
- **Hardware vs sim:** controller YAMLs are split — `controllers.<variant>.sim.yaml`
  for Gazebo, `controllers.hw.yaml` for the real robot.

### Topics

- `/cmd_vel_joy`, `/cmd_vel_key`, `/cmd_vel_nav` — twist_mux inputs (priority
  100 / 90 / 10).
- `/diff_drive_controller/cmd_vel` — twist_mux output, controller input.
- `/diff_drive_controller/odom` — wheel-encoder odometry.
- `/scan`, `/head_camera/image_raw`, `/head_camera/camera_info` — sensors.

## Combined System

Mount the mod101 arm on the top plate, connect the Feetech servo bus through Axon, and launch both arm and base stacks. The combined URDF includes the full kinematic chain from wheels to gripper tip.

| System | Reach | Arm Payload | Base Payload | BOM |
|---|---|---|---|---|
| base101 + mod101 | 502mm | 547g | ~7kg | ~$474 |
| base101 + mod101 PRO | 502mm | 1,105g | ~7kg | ~$551 |
| base101 PRO + mod101 PRO | 502mm | 1,105g | ~25kg | ~$691 |

## BOM

### base101 (~$340)

| Part | Qty | Price | Subtotal |
|---|---|---|---|
| DDSM210 hub motor | 4 | $25 | $100 |
| RPLidar C1 | 1 | $65 | $65 |
| RealSense D435 | 1 | $0* | $0* |
| Axon | 1 | $25 | $25 |
| Bolt | 1 | $15 | $15 |
| 4S 16Ah LiPo battery | 1 | $65 | $65 |
| JK BMS 4S | 1 | $18 | $18 |
| 60×20 extrusion (~1.2m) | 1 | $10 | $10 |
| Aluminum top plate (CNC, tapped) | 1 | $12 | $12 |
| PLA-CF parts (~200g) | 1 | $10 | $10 |
| TPU bumpers (~80g) | 1 | $4 | $4 |
| Compression springs (suspension) | 8 | $0.50 | $4 |
| Hardware (bolts, T-nuts, wiring) | lot | $12 | $12 |
| **Total** | | | **~$340** |

*\*D435 often already owned; add ~$150 if purchasing new.*

### base101 PRO upgrade (+$140)

Replace 4× DDSM210 ($100) with 4× DDSM115 ($240). Print larger suspension brackets. Everything else identical.

## Mechanical Decisions

**Why 60×20 extrusion?** 80×20 made the proportions look like a brick. 60×20 keeps the chassis thin and visually clean while providing enough internal height for flat-mount electronics. Form follows function — a thin, confident slab.

**Why skid steer?** Turn-in-place covers 99% of mobile manipulation needs. Mecanum wheels add noise, debris collection, cost, and nav complexity for the ability to strafe — which the arm compensates for with its 502mm reach.

**Why CNC top plate?** A grid of tapped M3 holes eliminates adapters, mounting brackets, and hardware-store tapping sessions. At $8-15 per plate from Shenzhen in moderate quantities, it's worth every cent for the user experience of "bolt things down and go."

**Why a carry handle?** Because robots misbehave. And because it doubles as an E-stop, which means every time someone grabs the robot out of instinct, the right thing happens. Safety through ergonomics, not through reading manuals.

## Project Status

- [x] Chassis design (Fusion 360)
- [x] Render and proportioning
- [x] Motor selection and analysis
- [x] Suspension design (PLA-CF adaptation)
- [x] URDF/xacro
- [x] Gazebo simulation
- [x] ros2_control integration
- [ ] Nav2 configuration
- [ ] E-stop handle mechanism
- [ ] CNC top plate manufacturing files (DXF)
- [ ] Combined base101 + mod101 system launch
- [ ] Elecrow kit production

## Related Projects

- **[mod101](https://github.com/robocore-dev/mod101)** — 5+1 DOF modular robot arm
- **[Axon](https://github.com/robocore-dev/axon)** — Multi-protocol controller board
- **[Bolt](https://github.com/robocore-dev/bolt)** — Power distribution hub
- **[Forge](https://github.com/robocore-dev/forge)** — ROS2 deployment orchestration

## License

MIT
