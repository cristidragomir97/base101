#!/usr/bin/env python3
"""LaserScan bridge for the base101 MuJoCo sim.

mujoco_ros2_control handles joints (and cameras) but has no laser/range
sensor output, so this node loads a parallel copy of the same MJCF, watches
TF for the lidar frame's world pose, and ray-casts against the static world
geoms via mj_multiRay. The robot's own geoms are excluded via geomgroup so
the chassis doesn't shadow the scan.

Parameters:
    mujoco_model_path : abs path to the MJCF scene
    lidar_frame_id    : TF frame the rays originate from (default: lidar_frame)
    world_frame_id    : TF frame the MJCF world is expressed in (default: odom).
                        The simulated world is static, so any frame whose
                        transform to map/odom is identity works.
    samples           : number of rays in a full sweep (default: 360)
    angle_min         : start of sweep in lidar frame (rad, default: -pi)
    angle_max         : end of sweep in lidar frame (rad, default: pi)
    range_min         : sensor minimum range (m, default: 0.12)
    range_max         : sensor maximum range (m, default: 12.0)
    update_rate       : publish rate (Hz, default: 10)
    output_topic      : output LaserScan topic (default: scan)
"""

from __future__ import annotations

import math

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def _quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """ROS quaternion (xyzw) → 3x3 rotation matrix."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ])


class LidarBridge(Node):

    def __init__(self) -> None:
        super().__init__('mujoco_lidar_bridge')

        self.declare_parameter('mujoco_model_path', '')
        self.declare_parameter('lidar_frame_id', 'lidar_frame')
        self.declare_parameter('world_frame_id', 'odom')
        self.declare_parameter('samples', 360)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max',  math.pi)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('output_topic', 'scan')

        model_path = self.get_parameter('mujoco_model_path').value
        if not model_path:
            raise RuntimeError(
                'mujoco_model_path is required so the lidar bridge can load the '
                'same MJCF scene as mujoco_ros2_control.'
            )

        self._lidar_frame  = self.get_parameter('lidar_frame_id').value
        self._world_frame  = self.get_parameter('world_frame_id').value
        self._samples      = int(self.get_parameter('samples').value)
        self._angle_min    = float(self.get_parameter('angle_min').value)
        self._angle_max    = float(self.get_parameter('angle_max').value)
        self._range_min    = float(self.get_parameter('range_min').value)
        self._range_max    = float(self.get_parameter('range_max').value)
        update_rate        = float(self.get_parameter('update_rate').value)
        output_topic       = self.get_parameter('output_topic').value

        self._angle_increment = (self._angle_max - self._angle_min) / float(self._samples)
        self._scan_time = 1.0 / update_rate

        # Local-frame unit ray directions, pre-computed once. CCW in the
        # lidar frame around +Z, starting at angle_min.
        angles = self._angle_min + np.arange(self._samples) * self._angle_increment
        self._rays_local = np.stack(
            [np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1
        ).astype(np.float64)  # (N, 3)

        self.get_logger().info(f'Loading MJCF: {model_path}')
        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        # Static world — just need an initial kinematic update.
        mujoco.mj_forward(self._model, self._data)

        # Include group 0 (world) only; exclude robot geoms in group 1+.
        self._geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        scan_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, output_topic, scan_qos)

        self._timer = self.create_timer(1.0 / update_rate, self._tick)
        self._warned_missing_tf = False
        self.get_logger().info(
            f'MuJoCo lidar bridge ready: {self._samples} rays @ {update_rate:.1f} Hz, '
            f'origin={self._lidar_frame}, world={self._world_frame}.'
        )

    def _tick(self) -> None:
        try:
            t: TransformStamped = self._tf_buffer.lookup_transform(
                self._world_frame, self._lidar_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            if not self._warned_missing_tf:
                self.get_logger().warn(
                    f'No TF {self._world_frame}->{self._lidar_frame} yet, '
                    'waiting for robot_state_publisher + odometry.',
                    throttle_duration_sec=5.0,
                )
                self._warned_missing_tf = True
            return

        pnt = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ], dtype=np.float64)
        R = _quat_to_rotation_matrix(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        )
        # World-frame ray directions: rays_world = R @ rays_local^T -> (3, N) -> (N, 3)
        rays_world = (R @ self._rays_local.T).T  # (N, 3)

        # mj_multiRay expects pnt (3,), vec (3*nray,), geomgroup (6,) uint8,
        # geomid (nray,) int32 out, dist (nray,) float64 out.
        nray = self._samples
        vec = np.ascontiguousarray(rays_world.reshape(-1))  # flattened (3N,)
        geomid = np.empty(nray, dtype=np.int32)
        dist = np.empty(nray, dtype=np.float64)

        mujoco.mj_multiRay(
            self._model,
            self._data,
            pnt,
            vec,
            self._geomgroup,
            1,            # flg_static — include static geoms
            -1,           # bodyexclude — no specific body filter
            geomid,
            dist,
            None,         # normal — not needed
            nray,
            self._range_max,
        )

        # mj_multiRay returns -1 (geomid) and a sentinel distance for misses.
        # Treat misses and out-of-range hits as +inf so Nav2/SLAM ignore them.
        ranges = np.where(
            (geomid >= 0) & (dist >= self._range_min) & (dist <= self._range_max),
            dist,
            float('inf'),
        ).astype(np.float32)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._lidar_frame
        msg.angle_min = self._angle_min
        msg.angle_max = self._angle_max - self._angle_increment
        msg.angle_increment = self._angle_increment
        msg.time_increment = 0.0
        msg.scan_time = self._scan_time
        msg.range_min = self._range_min
        msg.range_max = self._range_max
        msg.ranges = ranges.tolist()
        msg.intensities = []
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
