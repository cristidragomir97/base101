"""AgentContext: the wired-together pieces handlers and the server share.

Built once at startup by bootstrap.build_agent(). Keeping it a plain
dataclass (no behavior) avoids import cycles between server, safety,
tasks and handlers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from .audit import AuditLog
from .events import EventHub
from .profile import Profile
from .safety import AgentState, MotionLock, SafetyLayer
from .sensing import FrameCache
from .streams import StreamManager
from .tasks import TaskManager
from .watches import WatchRegistry

if TYPE_CHECKING:
    from .shm import ShmStore
    from .teleop import TeleopManager


@dataclass
class AgentContext:
    profile: Profile
    audit: AuditLog
    tasks: TaskManager
    safety: SafetyLayer
    state: AgentState
    lock: MotionLock
    watches: WatchRegistry
    events: EventHub
    frames: FrameCache
    streams: StreamManager
    # The RosInterface (ros.py), or a test fake, or None when running the
    # protocol stack without ROS (unit tests). Typed loosely on purpose:
    # importing ros.py here would drag rclpy into every unit test.
    ros: Any = None
    teleop: "TeleopManager | None" = None
    # The shared-memory payload store (None only in wire-stack unit
    # tests that never touch sensor methods).
    shm: "ShmStore | None" = None
    # Watchable path table built from the profile (state_paths.py).
    watch_paths: dict = None  # type: ignore[assignment]
