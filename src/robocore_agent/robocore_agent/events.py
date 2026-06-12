"""System event fan-out (spec section 19: ``robot.events()``).

Mirrors the audit-tail pattern: emitters call ``emit``; subscribers
(the server, which forwards to sessions that called events.subscribe)
are plain callables. v1 event kinds: "estop" (engaged/released) and
"watch" (a watch fired; data carries path/value/stop). Navigation
aborts join in Phase 5.
"""

from __future__ import annotations

import time
from typing import Any, Callable


class EventHub:
    def __init__(self) -> None:
        self.subscribers: list[Callable[[dict], None]] = []

    def emit(self, kind: str, data: dict[str, Any] | None = None) -> None:
        event = {"kind": kind, "stamp": time.time(), "data": data or {}}
        for subscriber in self.subscribers:
            subscriber(event)
