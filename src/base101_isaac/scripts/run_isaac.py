#!/usr/bin/env python3
"""Standalone Isaac Sim runner for the base101 mobile base.

MUST be executed inside the Isaac Sim Python environment (the pip-installed
isaacsim package brings its own Kit interpreter that this script bootstraps
via SimulationApp). The base101_isaac launch file spawns it as a subprocess.

Pipeline once Kit is up:
  1. Spawn a ground plane (or the user-provided USD scene).
  2. Import the base101 URDF using isaacsim's URDF importer.
  3. Attach an RTX lidar + RGB camera to the robot.
  4. Wire an OmniGraph that publishes /scan, /base_camera/image_raw, /tf,
     and /joint_states, and subscribes to /diff_drive_controller/cmd_vel to
     drive the wheels through Isaac's DifferentialController.

CLI args mirror the launch file. Anything Isaac-specific (paths, render
quality, headless flag) is forwarded as documented below; everything that's
also a ROS2 parameter is read from the ROS2 environment.
"""

from __future__ import annotations

import argparse
import math
import sys


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True,
                        help='Absolute path to the processed URDF file.')
    parser.add_argument('--variant', default='simple',
                        choices=['simple', 'pro'])
    parser.add_argument('--scene', default='',
                        help='USD scene file to load. Empty → default flat '
                             'ground + a few reference obstacles.')
    parser.add_argument('--headless', action='store_true',
                        help='Run Kit without a viewport window.')
    parser.add_argument('--wheel-radius', type=float, required=True)
    parser.add_argument('--wheel-separation', type=float, required=True)
    parser.add_argument('--max-linear', type=float, default=1.0)
    parser.add_argument('--max-angular', type=float, default=2.0)
    parser.add_argument('--cmd-topic', default='/diff_drive_controller/cmd_vel',
                        help='Twist topic that drives the wheels.')
    parser.add_argument('--namespace', default='',
                        help='ROS2 namespace prefix for published topics.')
    # parse_known_args so we tolerate any launch-injected flags we don't care about.
    args, _ = parser.parse_known_args()
    return args


ARGS = _parse_args()


# ----- Bootstrap Kit BEFORE any other isaac/omni import. -------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({
    'headless': ARGS.headless,
    'renderer': 'RayTracedLighting',
})


# ----- Post-bootstrap imports ----------------------------------------------
import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni  # noqa: E402
import omni.graph.core as og  # noqa: E402
from omni.isaac.core import World  # noqa: E402
from omni.isaac.core.objects.ground_plane import GroundPlane  # noqa: E402
from omni.isaac.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

try:
    # Isaac Sim 5.x / 6.x importer namespace.
    from isaacsim.asset.importer.urdf import _urdf as urdf_importer
except ImportError:  # pragma: no cover
    # 4.x fallback.
    from omni.importer.urdf import _urdf as urdf_importer  # type: ignore

try:
    from omni.isaac.wheeled_robots.robots import WheeledRobot
    from omni.isaac.wheeled_robots.controllers.differential_controller import (
        DifferentialController,
    )
except ImportError:  # pragma: no cover
    WheeledRobot = None
    DifferentialController = None


WHEEL_JOINT_NAMES = [
    'wheel_front_left',
    'wheel_front_right',
    'wheel_rear_left',
    'wheel_rear_right',
]


def _import_urdf(urdf_path: str) -> str:
    """Import a URDF into the current stage and return the robot's prim path."""
    cfg = urdf_importer.ImportConfig()
    cfg.merge_fixed_joints = True
    cfg.convex_decomp = False
    cfg.fix_base = False
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    cfg.density = 0.0
    cfg.default_drive_type = urdf_importer.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
    cfg.default_drive_strength = 1e3
    cfg.default_position_drive_damping = 1e2
    cfg.self_collision = False
    cfg.create_physics_scene = True
    cfg.make_default_prim = True

    status, robot_repr = urdf_importer.parse_urdf(urdf_path, cfg)
    if not status:
        raise RuntimeError(f'URDF parse failed for {urdf_path}')

    _, prim_path = urdf_importer.import_robot(urdf_path, robot_repr, cfg)
    if not prim_path:
        raise RuntimeError(f'URDF import returned empty prim path for {urdf_path}')
    return prim_path


def _add_default_scene(stage) -> None:
    """Drop a ground plane, a sun light, and a few reference obstacles."""
    GroundPlane('/World/ground', z_position=0.0)

    sun = UsdLux.DistantLight.Define(stage, '/World/Lights/Sun')
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(0.5)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 0.0, 0.0))

    def _box(name: str, pos: tuple[float, float, float],
             size: tuple[float, float, float], color: tuple[float, float, float]) -> None:
        cube = UsdGeom.Cube.Define(stage, f'/World/obstacles/{name}')
        cube.AddTranslateOp().Set(Gf.Vec3f(*pos))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    _box('ref_front', (2.0, 0.0, 0.25), (0.20, 0.20, 0.25), (0.2, 0.5, 0.9))
    _box('ref_left',  (0.0, 1.5, 0.15), (0.15, 0.15, 0.15), (0.9, 0.4, 0.2))
    _box('ref_right', (0.6, -1.2, 0.20), (0.18, 0.18, 0.20), (0.7, 0.7, 0.2))


def _attach_lidar(stage, parent_path: str) -> str:
    """Attach an RTX lidar prim under the robot. Returns its prim path."""
    lidar_path = f'{parent_path}/lidar_frame/lidar_rtx'
    lidar_prim = UsdGeom.Xform.Define(stage, lidar_path)
    # The actual RtxLidar schema is added by the Isaac Sensor extension; we
    # tag the prim with a custom attribute the graph reads.
    lidar_prim.GetPrim().CreateAttribute('isaacsim:sensor:rtx_lidar',
                                         UsdGeom.Tokens.token).Set('rotary_2d')
    return lidar_path


def _attach_camera(stage, parent_path: str) -> str:
    """Attach an Isaac RGB camera under the robot. Returns its prim path."""
    cam_path = f'{parent_path}/base_camera/camera_sensor'
    cam = UsdGeom.Camera.Define(stage, cam_path)
    cam.CreateFocalLengthAttr(18.14)   # Matches ~58° HFOV @ 24mm aperture
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(15.7)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))
    return cam_path


def _build_ros2_graph(robot_prim_path: str, lidar_path: str, camera_path: str) -> None:
    """Wire an OmniGraph that bridges Isaac sensors/actuators to ROS2 topics.

    Topology:
      ROS2SubTwist(/cmd_vel)
          → DifferentialController(wheel_radius, wheel_base)
          → ArticulationController(wheel joints)

      ArticulationState → ROS2PublishJointState(/joint_states)
                        → ROS2PublishOdometry(/odom)
                        → ROS2PublishTransformTree(/tf)

      RtxLidar(lidar_path) → ROS2PublishLaserScan(/scan)
      Camera(camera_path)  → ROS2CameraHelper(/base_camera/image_raw, /camera_info)

    All node names are namespaced under /ROS2_Graph for easy inspection.
    """
    keys = og.Controller.Keys
    graph_path = '/ROS2_Graph'

    (graph_handle, *_) = og.Controller.edit(
        {'graph_path': graph_path, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('on_tick', 'omni.graph.action.OnPlaybackTick'),

                ('sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),

                # ROS2 sub: /cmd_vel → linear/angular
                ('sub_twist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),

                # Differential controller computes per-wheel angular velocity
                ('diff_ctrl', 'isaacsim.wheeled_robots.DifferentialController'),

                # Articulation controller pushes the velocity command to the
                # robot's wheel joints.
                ('articulation', 'isaacsim.core.nodes.IsaacArticulationController'),

                # Joint states publisher
                ('joint_state_pub', 'isaacsim.ros2.bridge.ROS2PublishJointState'),

                # Odometry + TF
                ('compute_odom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('odom_pub',     'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('tf_pub',       'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
                ('tf_raw_pub',   'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),

                # Lidar
                ('lidar_helper', 'isaacsim.ros2.bridge.ROS2RtxLidarHelper'),
                # Camera
                ('camera_helper', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
            ],
            keys.SET_VALUES: [
                ('sub_twist.inputs:topicName', ARGS.cmd_topic),

                ('diff_ctrl.inputs:wheelDistance', ARGS.wheel_separation),
                ('diff_ctrl.inputs:wheelRadius',   ARGS.wheel_radius),
                ('diff_ctrl.inputs:maxLinearSpeed',  ARGS.max_linear),
                ('diff_ctrl.inputs:maxAngularSpeed', ARGS.max_angular),

                ('articulation.inputs:jointNames', WHEEL_JOINT_NAMES),
                ('articulation.inputs:targetPrim', robot_prim_path),

                ('joint_state_pub.inputs:topicName', 'joint_states'),
                ('joint_state_pub.inputs:targetPrim', robot_prim_path),

                ('compute_odom.inputs:chassisPrim', robot_prim_path),
                ('odom_pub.inputs:topicName', 'odom'),
                ('tf_pub.inputs:topicName', 'tf'),
                ('tf_pub.inputs:targetPrims', robot_prim_path),

                ('lidar_helper.inputs:topicName', 'scan'),
                ('lidar_helper.inputs:frameId',   'lidar_frame'),
                ('lidar_helper.inputs:lidarPrim', lidar_path),
                ('lidar_helper.inputs:type',      'laser_scan'),

                ('camera_helper.inputs:topicName', 'base_camera/image_raw'),
                ('camera_helper.inputs:frameId',   'base_camera'),
                ('camera_helper.inputs:cameraPrim', camera_path),
                ('camera_helper.inputs:type',      'rgb'),
            ],
            keys.CONNECT: [
                ('on_tick.outputs:tick', 'sub_twist.inputs:execIn'),
                ('on_tick.outputs:tick', 'compute_odom.inputs:execIn'),
                ('on_tick.outputs:tick', 'joint_state_pub.inputs:execIn'),
                ('on_tick.outputs:tick', 'lidar_helper.inputs:execIn'),
                ('on_tick.outputs:tick', 'camera_helper.inputs:execIn'),

                ('sub_twist.outputs:execOut',     'diff_ctrl.inputs:execIn'),
                ('sub_twist.outputs:linearVelocity', 'diff_ctrl.inputs:linearVelocity'),
                ('sub_twist.outputs:angularVelocity', 'diff_ctrl.inputs:angularVelocity'),

                ('diff_ctrl.outputs:execOut', 'articulation.inputs:execIn'),
                ('diff_ctrl.outputs:velocityCommand', 'articulation.inputs:velocityCommand'),

                ('compute_odom.outputs:execOut',         'odom_pub.inputs:execIn'),
                ('compute_odom.outputs:linearVelocity',  'odom_pub.inputs:linearVelocity'),
                ('compute_odom.outputs:angularVelocity', 'odom_pub.inputs:angularVelocity'),
                ('compute_odom.outputs:position',        'odom_pub.inputs:position'),
                ('compute_odom.outputs:orientation',     'odom_pub.inputs:orientation'),

                ('sim_time.outputs:simulationTime', 'odom_pub.inputs:timeStamp'),
                ('sim_time.outputs:simulationTime', 'tf_pub.inputs:timeStamp'),
                ('sim_time.outputs:simulationTime', 'joint_state_pub.inputs:timeStamp'),
            ],
        },
    )
    carb.log_info(f'ROS2 graph built at {graph_path}')


def main() -> None:
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    if ARGS.scene:
        add_reference_to_stage(usd_path=ARGS.scene, prim_path='/World/scene')
    else:
        _add_default_scene(stage)

    robot_prim_path = _import_urdf(ARGS.urdf)
    carb.log_info(f'Imported URDF as {robot_prim_path}')

    lidar_path  = _attach_lidar(stage, robot_prim_path)
    camera_path = _attach_camera(stage, robot_prim_path)

    _build_ros2_graph(robot_prim_path, lidar_path, camera_path)

    world.reset()

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:  # noqa: BLE001
        carb.log_error(f'base101_isaac runner crashed: {e}')
        simulation_app.close()
        sys.exit(1)
