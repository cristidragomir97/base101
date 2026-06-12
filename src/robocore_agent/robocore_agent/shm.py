"""Agent-side shared-memory payload store (the v1 frame transport).

Architecture ruling (Cristi, 2026-06-12): the agent writes raw sensor
arrays to files under /dev/shm and sends a ``PayloadRef`` descriptor over
the RPC channel; the local client mmaps the file as numpy. Zero copy on
the read side; the agent pays one memcpy (ROS message -> segment).

Lifetime: segments expire after their TTL or, oldest first, when the
store exceeds its byte budget. Expiring means unlink — a client that
already mapped the segment keeps valid memory (POSIX semantics), so
aggressive expiry is safe; a client that waits past the TTL to fetch
gets FrameExpired from its transport layer.

No ROS imports here: unit-testable anywhere with a /dev/shm.
"""

from __future__ import annotations

import logging
import os
import re
import time

import numpy as np

from robocore.models.sensing import PayloadRef

log = logging.getLogger("robocore_agent.shm")

SHM_DIR = "/dev/shm"

# Default byte budget across all live segments. At VGA RGB (~0.9 MiB per
# frame) this holds ~4.5 min of 1 Hz frames or ~37 s of a 5 fps stream
# alongside everything else; oldest segments evict first.
DEFAULT_BUDGET = 192 * 1024 * 1024

DEFAULT_TTL = 30.0  # seconds; per-call override for stream frames


def _sanitize(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_-]", "-", name)


class ShmStore:
    """Owns every shm segment one agent process creates."""

    def __init__(self, robot_name: str, budget: int = DEFAULT_BUDGET) -> None:
        self._prefix = f"rc-{_sanitize(robot_name)}-{os.getpid()}"
        self._seq = 0
        # name -> (deadline, size); insertion order == creation order.
        self._live: dict[str, tuple[float, int]] = {}
        self._bytes = 0
        self._budget = budget
        self._sweep_stale_siblings(robot_name)

    def put(self, array: np.ndarray, ttl: float = DEFAULT_TTL) -> PayloadRef:
        """Write one array into a fresh segment; returns its descriptor."""
        array = np.ascontiguousarray(array)
        self._seq += 1
        name = f"{self._prefix}-{self._seq}"
        fd = os.open(f"{SHM_DIR}/{name}",
                     os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        try:
            os.write(fd, array.data.cast("B"))
        finally:
            os.close(fd)
        self._live[name] = (time.monotonic() + ttl, array.nbytes)
        self._bytes += array.nbytes
        self._expire()
        return PayloadRef(
            transport="shm",
            name=name,
            dtype=array.dtype.name,
            shape=tuple(array.shape),
            size=array.nbytes,
        )

    def close(self) -> None:
        """Unlink every live segment (agent shutdown)."""
        for name in list(self._live):
            self._unlink(name)

    # -- internals -------------------------------------------------------------

    def _expire(self) -> None:
        now = time.monotonic()
        for name, (deadline, _size) in list(self._live.items()):
            if deadline <= now:
                self._unlink(name)
        while self._bytes > self._budget and self._live:
            self._unlink(next(iter(self._live)))  # oldest first

    def _unlink(self, name: str) -> None:
        _deadline, size = self._live.pop(name)
        self._bytes -= size
        try:
            os.unlink(f"{SHM_DIR}/{name}")
        except OSError:
            pass  # already gone; nothing to free

    def _sweep_stale_siblings(self, robot_name: str) -> None:
        """Unlink segments left by dead agents for the same robot.

        A SIGKILLed agent can't clean up after itself; its segments would
        sit in /dev/shm forever (same trap as stale fastrtps_* segments,
        see the Phase 3 worklog). Only our own naming pattern is touched.
        """
        pattern = re.compile(
            rf"^rc-{re.escape(_sanitize(robot_name))}-(\d+)-\d+$")
        try:
            entries = os.listdir(SHM_DIR)
        except OSError:
            return
        for entry in entries:
            match = pattern.match(entry)
            if match is None or int(match.group(1)) == os.getpid():
                continue
            if not os.path.exists(f"/proc/{match.group(1)}"):
                try:
                    os.unlink(f"{SHM_DIR}/{entry}")
                    log.info("removed stale shm segment %s", entry)
                except OSError:
                    pass
