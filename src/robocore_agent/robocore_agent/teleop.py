"""Teleop sessions (spec section 14): streamed intent with a server-side
watchdog.

A session holds the motion lock exclusively. The client streams drive
commands at its own rate; if they stop arriving (WiFi drop, crashed
client, sleeping laptop) the watchdog zeroes the velocity within the
watchdog window. The client is never trusted to stop the robot.

No rclpy here: velocity goes out through the RosInterface handed in at
construction (tests inject a fake).
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any

from .audit import AuditLog
from .profile import Profile
from .safety import MotionLock
from .server import RpcError

log = logging.getLogger("robocore_agent.teleop")

# A watchdog below this is unenforceable (scheduler jitter); reject it.
_MIN_WATCHDOG = 0.05  # seconds


@dataclass
class _Session:
    client_id: int
    watchdog: float
    last_command: float = field(default_factory=time.monotonic)
    halted_by_watchdog: bool = False
    watchdog_task: asyncio.Task | None = None


class TeleopManager:
    """Owns all teleop sessions (one per client, one lock for all)."""

    def __init__(self, lock: MotionLock, audit: AuditLog, ros: Any,
                 profile: Profile) -> None:
        self._lock = lock
        self._audit = audit
        self._ros = ros
        self._mobility = profile.spec.mobility
        self._teleop = profile.spec.teleop
        self._sessions: dict[int, _Session] = {}

    # -- session lifecycle ---------------------------------------------------

    def start(self, client_id: int, requested_watchdog: float) -> float:
        """Acquire the motion lock and start the watchdog. Returns the
        granted watchdog period (server-clamped, spec: clients may request
        shorter than the profile max, never longer)."""
        if not self._lock.acquire(client_id):
            raise RpcError(
                "SafetyViolation",
                f"motion lock held by client {self._lock.owner}",
                {"reason": "motion lock held"},
            )
        if client_id in self._sessions:
            raise RpcError("RobocoreError", "teleop session already active")
        max_watchdog = self._teleop.max_watchdog if self._teleop else 1.0
        watchdog = min(float(requested_watchdog), max_watchdog)
        if watchdog < _MIN_WATCHDOG:
            self._lock.release(client_id)
            raise RpcError("RobocoreError",
                           f"watchdog must be >= {_MIN_WATCHDOG}s")
        session = _Session(client_id=client_id, watchdog=watchdog)
        session.watchdog_task = asyncio.ensure_future(self._watch(session))
        self._sessions[client_id] = session
        return watchdog

    def end(self, client_id: int) -> None:
        """Zero velocity, stop the watchdog, release the lock. Idempotent."""
        session = self._sessions.pop(client_id, None)
        if session is None:
            return
        if session.watchdog_task is not None:
            session.watchdog_task.cancel()
        self._publish_zero_quietly()
        self._lock.release(client_id)

    def on_disconnect(self, client_id: int) -> None:
        """Server hook: a vanished client must not leave the robot moving
        or locked."""
        if client_id in self._sessions:
            self.end(client_id)
            self._audit.record(
                "safety", client=client_id, call="teleop",
                outcome="session ended by disconnect, velocity zeroed",
            )

    # -- commands ----------------------------------------------------------------

    def drive(self, client_id: int, linear: float, angular: float,
              lateral: float = 0.0) -> None:
        """One velocity command, clamped by the profile's limits."""
        session = self._session(client_id)
        if lateral != 0.0:
            locomotion = self._mobility.locomotion if self._mobility else ""
            if locomotion != "omni":
                raise RpcError(
                    "CapabilityNotSupported",
                    f"lateral velocity needs omni locomotion, "
                    f"this robot is {locomotion or 'unknown'}",
                )
        linear = self._clamp(linear, self._max_linear())
        lateral = self._clamp(lateral, self._max_linear())
        angular = self._clamp(angular, self._max_angular())
        session.last_command = time.monotonic()
        session.halted_by_watchdog = False
        self._ros.publish_twist(linear, angular, lateral)

    def stop(self, client_id: int) -> None:
        """Immediate zero, session stays alive."""
        session = self._session(client_id)
        session.last_command = time.monotonic()
        self._ros.publish_zero()

    def zero_all(self) -> None:
        """Emergency hook (e-stop engaged): zero regardless of sessions."""
        self._publish_zero_quietly()

    # -- internals -----------------------------------------------------------------

    def _session(self, client_id: int) -> _Session:
        session = self._sessions.get(client_id)
        if session is None:
            raise RpcError(
                "SafetyViolation", "no active teleop session; call "
                "teleop.start first", {"reason": "no teleop session"},
            )
        return session

    def _max_linear(self) -> float | None:
        return self._mobility.max_linear if self._mobility else None

    def _max_angular(self) -> float | None:
        return self._mobility.max_angular if self._mobility else None

    @staticmethod
    def _clamp(value: float, limit: float | None) -> float:
        value = float(value)
        if limit is None:
            return value
        return max(-limit, min(limit, value))

    async def _watch(self, session: _Session) -> None:
        """Zero the velocity when commands stop arriving for > watchdog."""
        interval = session.watchdog / 4
        while True:
            await asyncio.sleep(interval)
            silent = time.monotonic() - session.last_command
            if silent > session.watchdog and not session.halted_by_watchdog:
                session.halted_by_watchdog = True
                self._publish_zero_quietly()
                self._audit.record(
                    "safety", client=session.client_id, call="teleop",
                    outcome="watchdog halt",
                    detail={"reason": "teleop watchdog",
                            "silent_for": round(silent, 3)},
                )
                log.warning("teleop watchdog halted robot (client %d silent "
                            "%.2fs)", session.client_id, silent)

    def _publish_zero_quietly(self) -> None:
        try:
            self._ros.publish_zero()
        except Exception:
            log.exception("failed to publish zero twist")
