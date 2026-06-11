"""The agent's audit log (spec section 18).

Every command, safety rejection, task transition, e-stop flip and
connection event passes through here. Two sinks:

- an in-memory ring buffer (what ``audit.query`` reads), and
- append-only JSONL on disk, one file pair per robot, rotated by size.

Rotation keeps two files (current + previous), each capped at half the
profile's ``retention_mb``, so disk usage stays under retention_mb total.

Subscribers (the server's audit.tail fan-out) are plain callables invoked
synchronously from record(); they must not block.
"""

from __future__ import annotations

import logging
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from robocore.models import AuditEvent
from robocore.models.profile import AuditConfig

log = logging.getLogger("robocore_agent.audit")

# Ring buffer depth. ~10k events ≈ a long session; disk holds the history.
_RING_SIZE = 10_000


def default_audit_dir(robot_name: str) -> Path:
    """~/.robocore/audit/<robot-name>/ (decided by Cristi 2026-06-11)."""
    return Path.home() / ".robocore" / "audit" / robot_name


class AuditLog:
    """Ring buffer + JSONL persistence + tail subscribers."""

    def __init__(self, robot_name: str, config: AuditConfig | None,
                 directory: str | Path | None = None) -> None:
        config = config or AuditConfig()
        self._ring: deque[AuditEvent] = deque(maxlen=_RING_SIZE)
        self.subscribers: list[Callable[[AuditEvent], None]] = []
        # Per-file cap: two files (current + previous) fit retention_mb.
        self._max_file_bytes = config.retention_mb * 1024 * 1024 // 2
        if directory is not None:
            self._dir = Path(directory).expanduser()
        elif config.dir is not None:
            self._dir = Path(config.dir).expanduser()
        else:
            self._dir = default_audit_dir(robot_name)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._path = self._dir / "audit.jsonl"
        self._file = open(self._path, "a", encoding="utf-8")

    # -- writing ----------------------------------------------------------------

    def record(
        self,
        kind: str,
        *,
        client: int | None = None,
        call: str | None = None,
        outcome: str | None = None,
        task_id: str | None = None,
        detail: dict[str, Any] | None = None,
    ) -> AuditEvent:
        """Append one event. Never raises (a broken disk must not take the
        robot down); disk errors are logged once per occurrence."""
        event = AuditEvent(
            stamp=datetime.now(timezone.utc).isoformat(),
            kind=kind,
            client=client,
            call=call,
            outcome=outcome,
            task_id=task_id,
            detail=detail or {},
        )
        self._ring.append(event)
        try:
            self._file.write(event.model_dump_json() + "\n")
            self._file.flush()
            if self._file.tell() > self._max_file_bytes:
                self._rotate()
        except OSError as exc:
            log.error("audit disk write failed: %s", exc)
        for subscriber in list(self.subscribers):
            subscriber(event)
        return event

    def _rotate(self) -> None:
        """current -> previous (overwriting), reopen current empty."""
        self._file.close()
        self._path.replace(self._dir / "audit.previous.jsonl")
        self._file = open(self._path, "a", encoding="utf-8")

    # -- reading ----------------------------------------------------------------

    def query(
        self,
        since: str | None = None,
        kind: str | None = None,
        task_id: str | None = None,
        limit: int = 500,
    ) -> list[AuditEvent]:
        """Filter the ring buffer (not disk history). Newest events last.

        ``since`` compares ISO 8601 strings — all stamps are UTC with the
        same format, so lexicographic order is chronological order.
        """
        out = [
            event for event in self._ring
            if (since is None or event.stamp >= since)
            and (kind is None or event.kind == kind)
            and (task_id is None or event.task_id == task_id)
        ]
        return out[-limit:]

    def close(self) -> None:
        try:
            self._file.close()
        except OSError:
            pass
