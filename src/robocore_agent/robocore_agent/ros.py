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
from typing import Any

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

from .profile import Profile

# Below these speeds the robot reads as "not moving". Guessed: sim odom
# noise at standstill is ~1e-4; revisit on hardware.
_MOVING_LINEAR = 0.01   # m/s
_MOVING_ANGULAR = 0.01  # rad/s


class TfLookupError(Exception):
    """A TF lookup failed (unknown frame or timeout waiting for data)."""


class RosInterface:
    """Profile-driven ROS plumbing for one robot."""

    def __init__(self, node: Node, profile: Profile) -> None:
        self._node = node
        self._frames = profile.spec.frames
        self._lock = threading.Lock()
        self._odom: Odometry | None = None
        self._distance = 0.0
        self._last_xy: tuple[float, float] | None = None

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
