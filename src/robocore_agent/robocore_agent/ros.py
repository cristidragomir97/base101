"""The agent's ROS interface: publishers, subscriptions, TF.

The ONLY module (besides main.py) that imports rclpy. Everything robot-
specific (topic names, frame names) comes from the profile; there is no
robot knowledge here. Handlers talk to this object through the small
method surface below — blocking methods (TF lookups) must be called via
asyncio.to_thread so they never stall the RPC loop.

Thread model: the node spins on a background MultiThreadedExecutor
(main.py); subscription callbacks update state under a lock; the asyncio
thread reads snapshots and publishes commands (rclpy publishers are safe
to call from any thread).
"""

from __future__ import annotations

import math
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from typing import Any

import numpy as np
import yaml
from geometry_msgs.msg import Twist as TwistMsg
from geometry_msgs.msg import TwistStamped as TwistStampedMsg
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CameraInfo, Image, JointState
from sensor_msgs.msg import LaserScan as LaserScanMsg
from std_msgs.msg import String
import tf2_ros

# Odometry QoS: BEST_EFFORT + VOLATILE, keep-last depth 10. We always want
# the latest pose, so dropped samples don't matter; more importantly a
# BEST_EFFORT subscription is compatible with BOTH best-effort and reliable
# publishers, while a RELIABLE subscription silently receives NOTHING from a
# best-effort publisher (which is how diff_drive_controller publishes odom).
# A RELIABLE+VOLATILE sub looked fine in theory and got zero messages in
# practice — see docs/worklogs/robocore_phase3.md.
_ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Sensor streams (images, scans, joint states, battery): same rule as
# odometry — BEST_EFFORT receives from both best-effort and reliable
# publishers, RELIABLE silently receives nothing from best-effort ones.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# /robot_description is published once, latched (TRANSIENT_LOCAL); a
# volatile subscription joins too late and never sees it.
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

from .profile import Profile

# RGB/depth messages buffered per camera for stamp pairing (get_synced).
_CAMERA_BUFFER = 10

# Below these speeds the robot reads as "not moving". Guessed: sim odom
# noise at standstill is ~1e-4; revisit on hardware.
_MOVING_LINEAR = 0.01   # m/s
_MOVING_ANGULAR = 0.01  # rad/s


class TfLookupError(Exception):
    """A TF lookup failed (unknown frame or timeout waiting for data)."""


class _CameraState:
    """Per-camera subscriptions and message buffers."""

    def __init__(self) -> None:
        self.rgb: deque = deque(maxlen=_CAMERA_BUFFER)    # (stamp, Image)
        self.depth: deque = deque(maxlen=_CAMERA_BUFFER)  # (stamp, Image)
        self.info: CameraInfo | None = None
        self.subscribed = False


class RosInterface:
    """Profile-driven ROS plumbing for one robot."""

    def __init__(self, node: Node, profile: Profile) -> None:
        self._node = node
        self._spec = profile.spec
        self._frames = profile.spec.frames
        self._lock = threading.Lock()
        self._odom: Odometry | None = None
        self._distance = 0.0
        self._last_xy: tuple[float, float] | None = None

        # Sensing state (Phase 4). Camera subscriptions are created
        # lazily on first use (4 cameras x 30 fps of Python-side image
        # callbacks is real CPU; don't pay it for cameras nobody reads).
        self._sub_lock = threading.Lock()
        self._cameras: dict[str, _CameraState] = {}
        self._scan: LaserScanMsg | None = None
        self._joints: dict[str, tuple[float, float, float]] = {}
        self._battery: BatteryState | None = None
        self._description: str | None = None
        self._joint_limits: dict[str, tuple[float, float] | None] | None = None

        # Cheap state subscriptions are eager: joint states and battery
        # feed the watch sampler, which must see data before any client
        # asks for it. The robot description arrives once, latched.
        if self._spec.manipulation is not None or self._spec.joint_groups:
            node.create_subscription(
                JointState, self._spec.joint_states,
                self._on_joint_state, _SENSOR_QOS)
        status = self._spec.status
        if status is not None and status.battery:
            node.create_subscription(
                BatteryState, status.battery, self._on_battery, _SENSOR_QOS)
        node.create_subscription(
            String, "/robot_description", self._on_description, _LATCHED_QOS)

        self._cmd_vel_pub = None
        self._odom_topic = None
        self._cmd_vel_stamped = True
        mobility = profile.spec.mobility
        if mobility is not None and mobility.odom_topic:
            self._odom_topic = mobility.odom_topic
        if mobility is not None and mobility.cmd_vel:
            # On Jazzy, twist_mux (use_stamped:true) and diff_drive_controller
            # both subscribe TwistStamped, so the cmd_vel sink type is a
            # profile detail (mobility.cmd_vel_stamped). Publishing the wrong
            # type means the messages are silently never delivered.
            self._cmd_vel_stamped = mobility.cmd_vel_stamped
            msg_type = TwistStampedMsg if self._cmd_vel_stamped else TwistMsg
            self._cmd_vel_pub = node.create_publisher(
                msg_type, mobility.cmd_vel, 10)
        if mobility is not None and mobility.odom_topic:
            node.create_subscription(
                Odometry, mobility.odom_topic, self._on_odom, _ODOM_QOS)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, node, spin_thread=False)

    # -- frame aliases ---------------------------------------------------------

    def resolve_frame(self, name: str) -> str:
        """Resolve "@map"/"@odom"/"@base" to the profile's frame names;
        anything else passes through unchanged."""
        aliases = {"@map": self._frames.map, "@odom": self._frames.odom,
                   "@base": self._frames.base}
        return aliases.get(name, name)

    # -- velocity commands -------------------------------------------------------

    def publish_twist(self, linear: float, angular: float,
                      lateral: float = 0.0) -> None:
        """Send one velocity command (m/s, rad/s). Caller clamps."""
        if self._cmd_vel_pub is None:
            raise TfLookupError("profile declares no cmd_vel topic")
        if self._cmd_vel_stamped:
            msg = TwistStampedMsg()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = self._frames.base
            twist = msg.twist
        else:
            msg = TwistMsg()
            twist = msg
        twist.linear.x = float(linear)
        twist.linear.y = float(lateral)
        twist.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    def publish_zero(self) -> None:
        """Halt: zero velocity command."""
        self.publish_twist(0.0, 0.0, 0.0)

    # -- odometry state -----------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            if self._odom is None:
                self._node.get_logger().info(
                    f"first odometry received on {self._odom_topic!r}")
            self._odom = msg
            xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if self._last_xy is not None:
                self._distance += math.hypot(xy[0] - self._last_xy[0],
                                             xy[1] - self._last_xy[1])
            self._last_xy = xy

    def get_state(self) -> dict[str, Any]:
        """Pose / velocity / is_moving / distance_traveled, wire-shaped.

        Pose prefers the map frame (when TF can localize base in map),
        falls back to the odom frame; ``pose.frame`` says which you got.
        Raises TfLookupError if no odometry has arrived yet.
        """
        with self._lock:
            odom = self._odom
            distance = self._distance
        if odom is None:
            raise TfLookupError("no odometry received yet")

        pose = self._best_pose()
        twist = odom.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        moving = speed > _MOVING_LINEAR or abs(twist.angular.z) > _MOVING_ANGULAR
        return {
            "pose": pose,
            "velocity": {
                "linear": {"x": twist.linear.x, "y": twist.linear.y,
                           "z": twist.linear.z},
                "angular": {"x": twist.angular.x, "y": twist.angular.y,
                            "z": twist.angular.z},
            },
            "is_moving": moving,
            "distance_traveled": distance,
        }

    def _best_pose(self) -> dict[str, Any]:
        """base pose in map frame if available, else odom frame."""
        base = self._frames.base
        for frame in (self._frames.map, self._frames.odom):
            if self._tf_buffer.can_transform(frame, base, Time()):
                t = self._tf_buffer.lookup_transform(frame, base, Time())
                tr, rot = t.transform.translation, t.transform.rotation
                return {
                    "x": tr.x, "y": tr.y, "z": tr.z,
                    "q": {"x": rot.x, "y": rot.y, "z": rot.z, "w": rot.w},
                    "frame": frame,
                }
        raise TfLookupError(
            f"cannot locate {base!r} in {self._frames.map!r} or "
            f"{self._frames.odom!r} (TF not up yet?)"
        )

    # -- TF ---------------------------------------------------------------------

    def tf_lookup(self, parent: str, child: str,
                  timeout: float) -> dict[str, Any]:
        """Latest transform child->parent coordinates, wire-shaped.
        Blocking up to ``timeout``; call via asyncio.to_thread."""
        parent = self.resolve_frame(parent)
        child = self.resolve_frame(child)
        try:
            t = self._tf_buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=timeout))
        except tf2_ros.TransformException as exc:
            raise TfLookupError(
                f"no transform {parent!r} <- {child!r}: {exc}") from exc
        tr, rot = t.transform.translation, t.transform.rotation
        return {
            "translation": {"x": tr.x, "y": tr.y, "z": tr.z},
            "rotation": {"x": rot.x, "y": rot.y, "z": rot.z, "w": rot.w},
            "parent": parent,
            "child": child,
        }

    def tf_frames(self) -> list[str]:
        """All frame names currently in the TF buffer."""
        return self._frame_names()

    def _frame_names(self) -> list[str]:
        data = yaml.safe_load(self._tf_buffer.all_frames_as_yaml()) or {}
        names = set(data.keys())
        for info in data.values():
            names.add(info.get("parent", ""))
        names.discard("")
        return sorted(names)

    def tf_wait_for(self, frame: str, timeout: float) -> None:
        """Block until ``frame`` exists in TF. Raises TfLookupError on
        timeout. Call via asyncio.to_thread."""
        frame = self.resolve_frame(frame)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if frame in self._frame_names():
                return
            time.sleep(0.1)
        raise TfLookupError(f"frame {frame!r} did not appear in {timeout}s")

    # -- cameras (Phase 4) -------------------------------------------------------

    def ensure_camera(self, name: str) -> None:
        """Create the camera's subscriptions on first use. Thread-safe;
        raises TfLookupError for a camera the profile does not declare."""
        cfg = (self._spec.cameras or {}).get(name)
        if cfg is None:
            raise TfLookupError(f"profile declares no camera {name!r}")
        with self._sub_lock:
            state = self._cameras.setdefault(name, _CameraState())
            if state.subscribed:
                return
            state.subscribed = True
            if cfg.rgb:
                self._node.create_subscription(
                    Image, cfg.rgb,
                    lambda msg, s=state: self._on_image(s.rgb, msg),
                    _SENSOR_QOS)
            if cfg.depth:
                self._node.create_subscription(
                    Image, cfg.depth,
                    lambda msg, s=state: self._on_image(s.depth, msg),
                    _SENSOR_QOS)
            if cfg.info:
                self._node.create_subscription(
                    CameraInfo, cfg.info,
                    lambda msg, s=state: setattr(s, "info", msg),
                    _SENSOR_QOS)

    def _on_image(self, buffer: deque, msg: Image) -> None:
        with self._lock:
            buffer.append((_stamp(msg.header), msg))

    def camera_sample(
        self,
        name: str,
        need_rgb: bool,
        need_depth: bool,
        sync_tolerance: float = 0.05,
        wait: float = 2.0,
    ) -> dict[str, Any]:
        """Latest camera data, decoded to numpy.

        Returns {stamp, rgb, depth, intrinsics, frame_id}; rgb/depth are
        None when not requested or not configured. When both are
        requested, returns the pair with the closest stamps (within
        ``sync_tolerance``); when only rgb is requested but the camera
        has depth, the closest depth within tolerance rides along for
        the deprojection record (None if nothing close exists).

        Blocks up to ``wait`` for the first messages after a fresh
        subscription; call via asyncio.to_thread.
        """
        self.ensure_camera(name)
        cfg = (self._spec.cameras or {})[name]
        state = self._cameras[name]
        want_rgb = need_rgb and bool(cfg.rgb)
        want_depth = need_depth and bool(cfg.depth)
        if need_rgb and not cfg.rgb:
            raise TfLookupError(f"camera {name!r} has no rgb topic")
        if need_depth and not cfg.depth:
            raise TfLookupError(
                f"camera {name!r} has no depth topic; deprojection and "
                "clouds need a depth-capable camera")

        deadline = time.monotonic() + wait
        while True:
            with self._lock:
                rgb = list(state.rgb)
                depth = list(state.depth)
            ready = ((not want_rgb or rgb)
                     and (not want_depth or depth))
            if ready:
                break
            if time.monotonic() > deadline:
                missing = []
                if want_rgb and not rgb:
                    missing.append(cfg.rgb)
                if want_depth and not depth:
                    missing.append(cfg.depth)
                raise TfLookupError(
                    f"no data from camera {name!r} on {missing} within "
                    f"{wait}s (topic remapped? sim paused?)")
            time.sleep(0.02)

        if want_rgb and (want_depth or (cfg.depth and depth)):
            pair = _closest_pair(rgb, depth)
            if pair is None or abs(pair[0][0] - pair[1][0]) > sync_tolerance:
                if want_depth:
                    raise TfLookupError(
                        f"no rgb/depth pair within {sync_tolerance}s for "
                        f"camera {name!r}")
                pair = ((rgb[-1][0], rgb[-1][1]), None)
            rgb_entry, depth_entry = pair
        elif want_rgb:
            rgb_entry, depth_entry = rgb[-1], None
        else:
            rgb_entry, depth_entry = None, depth[-1]

        info = state.info
        intrinsics = None
        if info is not None:
            intrinsics = {
                "fx": info.k[0], "fy": info.k[4],
                "cx": info.k[2], "cy": info.k[5],
                "width": info.width, "height": info.height,
            }
        frame_id = cfg.frame or (
            info.header.frame_id if info is not None else None)
        if frame_id is None:
            anchor = rgb_entry or depth_entry
            frame_id = anchor[1].header.frame_id
        return {
            "stamp": (rgb_entry or depth_entry)[0],
            "rgb": _image_to_numpy(rgb_entry[1]) if rgb_entry else None,
            "rgb_stamp": rgb_entry[0] if rgb_entry else None,
            "depth": _image_to_numpy(depth_entry[1]) if depth_entry else None,
            "depth_stamp": depth_entry[0] if depth_entry else None,
            "intrinsics": intrinsics,
            "frame_id": frame_id,
        }

    def camera_meta(self, name: str, wait: float = 2.0) -> dict[str, Any]:
        """Intrinsics + frame id without pulling an image. Blocks up to
        ``wait`` for camera_info after a fresh subscription (None fields
        if the camera has no info topic)."""
        self.ensure_camera(name)
        cfg = (self._spec.cameras or {})[name]
        state = self._cameras[name]
        info = None
        if cfg.info:
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline:
                info = state.info
                if info is not None:
                    break
                time.sleep(0.02)
        intrinsics = None
        if info is not None:
            intrinsics = {
                "fx": info.k[0], "fy": info.k[4],
                "cx": info.k[2], "cy": info.k[5],
                "width": info.width, "height": info.height,
            }
        frame_id = cfg.frame or (
            info.header.frame_id if info is not None else None)
        return {"intrinsics": intrinsics, "frame_id": frame_id}

    def snapshot_transforms(self, camera_frame: str) -> dict[str, dict]:
        """Latest transforms target <- camera for map/odom/base, for the
        deprojection record. Frames TF cannot resolve right now are
        simply absent."""
        out: dict[str, dict] = {}
        for target in (self._frames.map, self._frames.odom,
                       self._frames.base):
            if target == camera_frame or target in out:
                continue
            if not self._tf_buffer.can_transform(
                    target, camera_frame, Time()):
                continue
            t = self._tf_buffer.lookup_transform(target, camera_frame, Time())
            tr, rot = t.transform.translation, t.transform.rotation
            out[target] = {
                "translation": {"x": tr.x, "y": tr.y, "z": tr.z},
                "rotation": {"x": rot.x, "y": rot.y, "z": rot.z, "w": rot.w},
                "parent": target,
                "child": camera_frame,
            }
        return out

    # -- lidar (Phase 4) -----------------------------------------------------------

    def ensure_lidar(self) -> None:
        if self._spec.lidar is None:
            raise TfLookupError("profile declares no lidar")
        with self._sub_lock:
            if getattr(self, "_lidar_subscribed", False):
                return
            self._lidar_subscribed = True
            self._node.create_subscription(
                LaserScanMsg, self._spec.lidar.scan,
                self._on_scan, _SENSOR_QOS)

    def _on_scan(self, msg: LaserScanMsg) -> None:
        with self._lock:
            self._scan = msg

    def lidar_sample(self, wait: float = 2.0) -> dict[str, Any]:
        """Latest scan as {stamp, frame, angle_*, ranges (np.float32)}.
        Blocks up to ``wait`` after a fresh subscription; call via
        asyncio.to_thread."""
        self.ensure_lidar()
        deadline = time.monotonic() + wait
        while True:
            with self._lock:
                msg = self._scan
            if msg is not None:
                break
            if time.monotonic() > deadline:
                raise TfLookupError(
                    f"no scan on {self._spec.lidar.scan!r} within {wait}s")
            time.sleep(0.02)
        return {
            "stamp": _stamp(msg.header),
            "frame": msg.header.frame_id,
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "ranges": np.asarray(msg.ranges, dtype=np.float32),
        }

    # -- joint states / battery (Phase 4) ----------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                self._joints[name] = (
                    msg.position[i] if i < len(msg.position) else 0.0,
                    msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    msg.effort[i] if i < len(msg.effort) else 0.0,
                )

    def joint_snapshot(self) -> dict[str, tuple[float, float, float]]:
        """{joint: (position, velocity, effort)} from the last message."""
        with self._lock:
            return dict(self._joints)

    def joint_limits(self) -> dict[str, tuple[float, float] | None]:
        """{joint: (lower, upper) | None} parsed from /robot_description.
        Empty until the latched description arrives."""
        with self._lock:
            description = self._description
            if self._joint_limits is not None:
                return self._joint_limits
        if description is None:
            return {}
        limits: dict[str, tuple[float, float] | None] = {}
        try:
            root = ET.fromstring(description)
        except ET.ParseError:
            return {}
        for joint in root.iter("joint"):
            name = joint.get("name")
            kind = joint.get("type")
            if name is None or kind in (None, "fixed"):
                continue
            limit = joint.find("limit")
            if kind == "continuous" or limit is None:
                limits[name] = None
                continue
            lower, upper = limit.get("lower"), limit.get("upper")
            if lower is None or upper is None:
                limits[name] = None
            else:
                limits[name] = (float(lower), float(upper))
        with self._lock:
            self._joint_limits = limits
        return limits

    def _on_description(self, msg: String) -> None:
        with self._lock:
            self._description = msg.data
            self._joint_limits = None  # reparse lazily

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._battery = msg

    def battery_state(self) -> tuple[float | None, bool | None]:
        """(level 0..100, is_charging), (None, None) before data.

        sensor_msgs/BatteryState.percentage is 0..1 per the message
        definition, but real drivers (and `ros2 topic pub` users)
        routinely publish 0..100; values above 1.5 are passed through
        as-is, smaller ones are scaled.
        """
        with self._lock:
            msg = self._battery
        if msg is None:
            return None, None
        level = float(msg.percentage)
        if math.isnan(level):
            return None, msg.power_supply_status == 1
        if level <= 1.5:
            level *= 100.0
        return level, msg.power_supply_status == 1


def _stamp(header: Any) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _closest_pair(
    rgb: list, depth: list
) -> tuple[tuple, tuple] | None:
    """The (rgb, depth) entry pair with minimal stamp distance."""
    if not rgb or not depth:
        return None
    best = None
    for r in rgb:
        for d in depth:
            delta = abs(r[0] - d[0])
            if best is None or delta < best[0]:
                best = (delta, r, d)
    return (best[1], best[2])


def _image_to_numpy(msg: Image) -> np.ndarray:
    """Decode a sensor_msgs/Image without cv_bridge.

    Supports what cameras actually publish here: rgb8/bgr8 color,
    32FC1 float meters and 16UC1 millimeter depth.
    """
    h, w, step = msg.height, msg.width, msg.step
    enc = msg.encoding.lower()
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        arr = data.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        if enc == "bgr8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)
    if enc == "32fc1":
        arr = data.reshape(h, step).view(np.float32)[:, :w]
        return np.ascontiguousarray(arr)
    if enc in ("16uc1", "mono16"):
        arr = data.reshape(h, step).view(np.uint16)[:, :w]
        return np.ascontiguousarray(arr.astype(np.float32) / 1000.0)
    raise TfLookupError(f"unsupported image encoding {msg.encoding!r}")
