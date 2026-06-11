"""AgentContext: the wired-together pieces handlers and the server share.

Built once at startup by bootstrap.build_agent(). Keeping it a plain
dataclass (no behavior) avoids import cycles between server, safety,
tasks and handlers.
"""

from __future__ import annotations

from dataclasses import dataclass

from .audit import AuditLog
from .profile import Profile
from .safety import AgentState, MotionLock, SafetyLayer
from .tasks import TaskManager


@dataclass
class AgentContext:
    profile: Profile
    audit: AuditLog
    tasks: TaskManager
    safety: SafetyLayer
    state: AgentState
    lock: MotionLock
