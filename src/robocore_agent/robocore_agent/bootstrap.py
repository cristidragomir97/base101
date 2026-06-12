"""Wires a loaded profile into a ready-to-start AgentServer.

Used by main.py and by the engine's tests (which run the same stack
in-process, minus rclpy).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .audit import AuditLog
from .context import AgentContext
from .events import EventHub
from .handlers import build_registry
from .profile import Profile
from .safety import AgentState, MotionLock, SafetyLayer
from .sensing import FrameCache
from .server import AgentServer
from .shm import ShmStore
from .state_paths import build_path_table
from .streams import StreamManager
from .tasks import TaskManager
from .teleop import TeleopManager
from .watches import WatchRegistry


def build_agent(
    profile: Profile,
    *,
    unix_path: str | None,
    host: str | None = None,
    port: int | None = None,
    audit_dir: str | Path | None = None,
    ros: Any = None,
) -> tuple[AgentServer, AgentContext]:
    """Assemble audit + safety + tasks + handlers + server for one profile.

    ``audit_dir`` overrides the profile's audit directory (tests use a tmp
    dir). ``ros`` is the RosInterface (main.py) or a fake (tests); None
    runs the protocol stack without a robot. Call ``await server.start()``
    on the returned server; call ``ctx.audit.close()`` after shutdown.
    """
    audit = AuditLog(profile.info.name, profile.spec.audit,
                     directory=audit_dir)
    state = AgentState()
    lock = MotionLock()
    teleop = None
    if "teleop" in profile.info.capabilities and ros is not None:
        teleop = TeleopManager(lock, audit, ros, profile)
    ctx = AgentContext(
        profile=profile,
        audit=audit,
        tasks=TaskManager(audit),
        safety=SafetyLayer(state, lock),
        state=state,
        lock=lock,
        watches=WatchRegistry(),
        events=EventHub(),
        frames=FrameCache(),
        streams=StreamManager(),
        ros=ros,
        teleop=teleop,
        shm=ShmStore(profile.info.name),
    )
    ctx.watch_paths = build_path_table(ctx) if ros is not None else {}
    server = AgentServer(
        registry=build_registry(ctx),
        ctx=ctx,
        unix_path=unix_path,
        host=host,
        port=port,
    )
    return server, ctx
