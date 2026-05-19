# Simulator setup

`base101` runs in three simulators today. They all consume the same URDF
(via the `simulator` xacro arg) and publish the same ROS2 topics, so
everything downstream (Nav2, slam_toolbox, joystick teleop, RViz) is
unaware of which one is underneath. Build only the simulator(s) you need.

| | Gazebo Sim | MuJoCo | Isaac Sim |
|---|---|---|---|
| Package | `base101_gazebo` | `base101_mujoco` | `base101_isaac` |
| World format | SDF | MJCF | USD |
| Physics | DART or Bullet-Featherstone | MuJoCo (Featherstone) | PhysX 5 |
| ros2_control plugin | `gz_ros2_control/GazeboSimSystem` | `mujoco_ros2_control/MujocoSystem` | None — Isaac drives joints natively from an OmniGraph |
| Lidar source | `<gpu_lidar>` sensor + `ros_gz_bridge` | `base101_mujoco.lidar_bridge` (parallel-mjData `mj_multiRay`) | RTX lidar + `ROS2RtxLidarHelper` |
| Camera source | Native + `ros_gz_image` | Built into `mujoco_ros2_control`'s `mujoco_cameras.cpp` | USD `Camera` + `ROS2CameraHelper` |
| Maturity | Production | Production for joints/cameras; lidar via companion node | Experimental — Isaac Sim 6.x is early-developer |

The rest of this document covers prerequisites, install steps, and known
gotchas for each backend. Once a sim is running, the launch commands are:

```bash
ros2 launch base101_gazebo gazebo.launch.py [variant:=simple|pro] [world:=…]
ros2 launch base101_mujoco mujoco.launch.py [variant:=simple|pro] [scene:=…]
ros2 launch base101_isaac  isaac.launch.py  [variant:=simple|pro] [scene:=…] [headless:=true]
```

Web teleop is available at `http://localhost:8888/` (rosboard) for all three.

---

## Common prerequisites

Same as the rest of the stack:

- Ubuntu 24.04 (or any distro that ships ROS 2 Jazzy)
- ROS 2 Jazzy
- `python3-vcstool` for fetching repos:
  ```bash
  sudo apt install python3-vcstool python3-colcon-common-extensions
  ```

Fetch the workspace's external repos once. This pulls
`m-explore-ros2` (frontier exploration) and `mujoco_ros2_control` (only
needed for the MuJoCo backend; safe to keep otherwise):

```bash
cd ~/your_workspace          # the directory containing src/
vcs import src < src/base101/base101.repos
```

---

## Gazebo Sim

The default backend. Already wired up in the main README's Quickstart.

### Install

```bash
sudo apt install \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-controller-manager \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-twist-mux \
    ros-jazzy-robot-state-publisher
```

### Build

```bash
colcon build --symlink-install --packages-up-to base101_gazebo
source install/setup.bash
```

### Launch

```bash
ros2 launch base101_gazebo gazebo.launch.py                              # simple, sticky_floor world
ros2 launch base101_gazebo gazebo.launch.py variant:=pro world:=empty.sdf
```

### Gotchas

- **Physics engine.** `sticky_floor.sdf` pins ODE because Bullet-Featherstone
  ignores the anisotropic `<mu2>/<fdir1>` friction needed for 4-wheel skid
  steer. `empty.sdf` uses DART, which also honours it.
- **Sim time.** Always pass `use_sim_time:=true` to Nav2 / slam_toolbox
  launches when running against Gazebo, or you'll see silent TF
  extrapolation failures.
- **Fuel models.** `sticky_floor.sdf` references models from
  https://fuel.gazebosim.org; first launch downloads them and may take a
  minute on slow connections.

---

## MuJoCo

MuJoCo is faster than Gazebo on commodity hardware and has cleaner
contact dynamics, which is useful for skid steer. We use the
[`sangteak601/mujoco_ros2_control`](https://github.com/sangteak601/mujoco_ros2_control)
plugin — a community fork of moveit's original — for ros2_control + camera
publishing. Lidar isn't supported upstream, so `base101_mujoco` ships a
companion node (`lidar_bridge`) that loads a parallel copy of the MJCF and
ray-casts against it via `mj_multiRay`.

### Install

System packages (most are already installed if Gazebo works):

```bash
sudo apt install \
    ros-jazzy-controller-manager \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-twist-mux \
    ros-jazzy-robot-state-publisher \
    libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```

MuJoCo C++ + Python:

```bash
pip install "mujoco>=3.2"

# C++ library — needed to build mujoco_ros2_control. Either grab a release
# tarball or build from source.
MUJOCO_VERSION=3.2.5
wget -O /tmp/mujoco.tar.gz \
    https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz
sudo mkdir -p /opt/mujoco
sudo tar -xzf /tmp/mujoco.tar.gz -C /opt/mujoco --strip-components=1
echo 'export MUJOCO_DIR=/opt/mujoco'  >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/opt/mujoco/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

If you skipped `vcs import` above, do it now so `mujoco_ros2_control` lands
under `src/`:

```bash
vcs import src < src/base101/base101.repos
```

### Build

`mujoco_ros2_control` is a ROS 2 ament_cmake package — colcon picks it up
once it's under `src/`:

```bash
colcon build --symlink-install --packages-up-to base101_mujoco
source install/setup.bash
```

If CMake can't find MuJoCo, pass it explicitly:

```bash
colcon build --packages-up-to base101_mujoco \
    --cmake-args -DMUJOCO_DIR=/opt/mujoco
```

### Launch

```bash
ros2 launch base101_mujoco mujoco.launch.py                  # simple
ros2 launch base101_mujoco mujoco.launch.py variant:=pro
ros2 launch base101_mujoco mujoco.launch.py scene:=/path/to/custom.xml
```

Scenes live in `src/base101_mujoco/scenes/`. Empty `scene:=` (default)
selects `base101_<variant>.xml`.

### How it fits together

- `mujoco_ros2_control` node owns the simulation loop. It loads
  `scenes/base101_<variant>.xml`, hosts the `controller_manager`, and
  binds each ros2_control joint to the matching MJCF `<joint>` by name.
- Cameras declared in the MJCF (`<camera name="base_camera"/>`) are picked
  up by `mujoco_cameras.cpp` and published to `/base_camera/image_raw`
  + depth + camera_info automatically.
- The companion `lidar_bridge` node loads a second copy of the same MJCF
  read-only. It subscribes to `/tf` for `lidar_frame`'s world pose, calls
  `mj_multiRay` against the static geoms (group 0 only — the robot is in
  group 1 and excluded), and publishes `sensor_msgs/LaserScan` on `/scan`.
- The diff-drive controller's wheel velocity commands are written
  directly to `mjData.qvel` by `MujocoSystem`. Wheel friction + chassis
  inertia in the MJCF translate that into real motion.

### Gotchas

- **`mujoco_ros2_control` does not (yet) publish IMU/LaserScan.** Cameras
  work upstream; everything else is on us. The README on their repo lists
  IMU + range sensors as future work.
- **Joint names are the binding contract.** The MJCF joints
  (`wheel_front_left`, `wheel_front_right`, `wheel_rear_left`,
  `wheel_rear_right`) and the URDF's `base101.mujoco.ros2control` joints
  must match exactly.
- **Lidar bridge needs TF.** The bridge waits for the
  `odom → lidar_frame` transform; this comes from `robot_state_publisher`
  (URDF kinematics) + `diff_drive_controller` (odom→base_link). If the
  controllers haven't loaded yet, you'll see "No TF" warnings — they go
  away as soon as the first odom message arrives.
- **`mj_multiRay` excludes the robot via `geomgroup`.** If you add new
  geoms to the MJCF and want them to be invisible to the lidar (e.g.
  bumpers), put them in group 1 along with the rest of the chassis.

---

## Isaac Sim

Isaac Sim is the heaviest of the three backends (requires an RTX GPU) but
gives you photorealistic rendering and PhysX 5 contact dynamics. The
`base101_isaac` package targets Isaac Sim **6.x** via pip; older
Omniverse-Launcher installs aren't supported.

> Status: experimental. Isaac Sim 6.0 is an early-developer release with
> incomplete documentation. The runner script in this repo targets the
> documented OmniGraph node names but may need API tweaks as 6.x
> stabilises. Confirm with `ros2 topic list` after launch that
> `/joint_states`, `/odom`, `/scan`, `/base_camera/image_raw`, and `/tf`
> are all publishing.

### Install

GPU prerequisites (Isaac Sim's docs are authoritative; this is a summary):

- NVIDIA GPU with at least 8 GB VRAM (RTX 30xx-class or newer recommended)
- NVIDIA driver ≥ 535
- CUDA 12.x runtime (bundled with the pip install)

Install the SDK into the same Python that runs ROS2 (system Python on
Jazzy, usually `python3.12` on Ubuntu 24.04):

```bash
pip install 'isaacsim[all]==6.0.0.*' --extra-index-url https://pypi.nvidia.com
```

`isaacsim[all]` pulls the URDF importer, sensor extensions, and ROS2
bridge extensions. A bare `pip install isaacsim` only gives you the
metapackage + kernel and most of the runner script will fail at import
time.

Sanity check:

```bash
python3 -c "from isaacsim import SimulationApp; print('SimulationApp OK')"
```

The first launch downloads a few hundred MB of cached extensions on top
of the pip install — expect 5-10 minutes of initial setup.

### Build

```bash
colcon build --symlink-install --packages-up-to base101_isaac
source install/setup.bash
```

### Launch

```bash
ros2 launch base101_isaac isaac.launch.py                  # simple
ros2 launch base101_isaac isaac.launch.py variant:=pro
ros2 launch base101_isaac isaac.launch.py headless:=true   # no viewport
```

### How it fits together

- The launch file processes the URDF via xacro with
  `simulator:=isaac` (no ros2_control block), writes it to `/tmp`, and
  spawns `scripts/run_isaac.py` as a subprocess.
- `run_isaac.py` is the only piece that runs inside the Isaac Sim Kit
  interpreter. It bootstraps Kit via `SimulationApp(...)`, imports the
  URDF using `isaacsim.asset.importer.urdf`, attaches an RTX lidar + USD
  camera to the robot, and wires an OmniGraph that bridges:
  - `/diff_drive_controller/cmd_vel` → `DifferentialController` →
    `IsaacArticulationController`
  - articulation state → `/joint_states`, `/odom`, `/tf`
  - RTX lidar → `/scan`
  - USD camera → `/base_camera/image_raw` + `/base_camera/camera_info`
- ROS-side scaffolding (`robot_state_publisher`, `twist_mux`, `rosboard`)
  is launched as normal Node actions, identical to the gazebo/mujoco
  launches.

### Gotchas

- **`SimulationApp` must be the first import.** The runner does this; if
  you adapt it, keep that ordering or Kit fails to start.
- **No `controller_manager` on the Isaac side.** Diff drive happens
  inside the OmniGraph. `diff_drive_controller` from
  `base101_control/config/controllers.*.sim.yaml` is **not** loaded for
  this backend — `twist_mux` writes directly to the topic the OmniGraph
  subscribes to. Nav2 sees the same `/cmd_vel_nav` → `twist_mux` flow.
- **TF coming from two sources.** Isaac publishes `odom → base_link` and
  joint TFs. `robot_state_publisher` (also launched) publishes the static
  TFs from the URDF. If both publish dynamic joint TFs you'll see
  warnings — the runner is configured to only publish `odom → base_link`
  in `/tf`, leaving the rest to `robot_state_publisher`.
- **OmniGraph node names changed between Isaac 5.x and 6.x.** The runner
  catches `ImportError` for the URDF importer and wheeled_robots modules
  and falls back to the older namespaces. If your install is somewhere
  between, expect breakage and pin the version with
  `pip install 'isaacsim[all]==6.0.0.0'`.

---

## Which one should I use?

- **Gazebo** — default, most production-tested. Use this unless you have
  a specific reason not to.
- **MuJoCo** — pick this if you need faster-than-realtime physics for RL
  experiments, or if Gazebo's wheel friction model is giving you trouble.
- **Isaac Sim** — pick this if you need photorealistic rendering for
  vision-based pipelines, or you're already in the Omniverse ecosystem.
  Expect more setup pain.
