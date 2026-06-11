"""Safety layer skeleton (spec section 21) and the motion lock.

Every motion-classed wire method passes ``SafetyLayer.check`` before its
handler runs — server-side, non-bypassable, regardless of which client
sent it. Phase 2 checks: e-stop state and the motion lock. Velocity
clamps, workspace bounds, geofence etc. land with their phases.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .server import RpcError

# Wire methods that move the robot. Grows each phase; a method missing from
# this set runs without safety checks, so additions are part of review.
# Note teleop.start is motion-classed too: it takes the motion lock, and an
# engaged e-stop must reject the whole session, not just each drive.
MOTION_METHODS = frozenset({
    "mobility.navigate_to",
    "teleop.start",
    "teleop.drive",
})


@dataclass
class AgentState:
    """Mutable agent-wide state the safety layer reads."""

    estop_engaged: bool = False


@dataclass
class MotionLock:
    """Motion is single-writer (spec section 14 resolution): one client id
    may hold the lock; teleop sessions (Phase 3) hold it exclusively.
    Read access is never locked."""

    owner: int | None = field(default=None)

    def acquire(self, client_id: int) -> bool:
        """Take the lock. True on success or if already held by this
        client; False if another client holds it."""
        if self.owner is None or self.owner == client_id:
            self.owner = client_id
            return True
        return False

    def release(self, client_id: int) -> bool:
        """Release if held by this client. True if released."""
        if self.owner == client_id:
            self.owner = None
            return True
        return False

    def held_by_other(self, client_id: int) -> bool:
        return self.owner is not None and self.owner != client_id


class SafetyLayer:
    """Validates motion commands before they reach ROS."""

    def __init__(self, state: AgentState, lock: MotionLock) -> None:
        self._state = state
        self._lock = lock

    def check(self, client_id: int, method: str) -> None:
        """Raise RpcError(SafetyViolation) if ``method`` must not run now.

        Non-motion methods always pass: reads are never safety-gated.
        """
        if method not in MOTION_METHODS:
            return
        if self._state.estop_engaged:
            raise RpcError(
                "SafetyViolation",
                "e-stop is engaged; all motion rejected",
                {"reason": "estop"},
            )
        if self._lock.held_by_other(client_id):
            raise RpcError(
                "SafetyViolation",
                f"motion lock held by client {self._lock.owner}",
                {"reason": "motion lock held"},
            )
