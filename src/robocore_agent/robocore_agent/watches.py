"""Bridge-side condition monitors (spec section 16).

A watch is a numeric condition over an observable dotted path
("battery.level", "velocity.linear.x", "arms.left.effort.elbow"). The
agent evaluates every active watch in its sampler loop, independent of
client speed or connectivity — that is the whole point: with
``stop=True`` the bridge halts all motion the instant the condition
fires, even if the owning client is hung or gone.

This module is the pure state machine (registry, conditions, debounce,
latching); path resolution against live robot state and the halt action
are injected by the caller. No rclpy, unit-testable with a fake clock.
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field
from typing import Callable

from .server import RpcError

# Trigger events kept per watch (spec: w.events). Oldest dropped beyond.
_MAX_EVENTS = 100

# How long a stop=True watch stays armed after its owner disconnects,
# when the client gave no explicit lifetime. OPEN-Q: the spec says
# "their declared lifetime" without a default; 600 s is the conservative
# stand-in until Cristi rules.
DEFAULT_STOP_LIFETIME = 600.0


@dataclass
class Watch:
    id: str
    owner: int                      # client id
    path: str
    above: float | None
    below: float | None
    outside: tuple[float, float] | None
    stop: bool
    debounce: float
    lifetime: float
    # state
    triggered: bool = False         # latched
    value: float | None = None
    triggered_at: float | None = None
    events: list[tuple[float, float]] = field(default_factory=list)
    # debounce bookkeeping: when the condition started holding, monotonic
    holding_since: float | None = None
    condition_active: bool = False  # condition (post-debounce) currently on
    orphan_deadline: float | None = None  # set when the owner disconnects

    def condition(self, value: float) -> bool:
        if self.above is not None:
            return value > self.above
        if self.below is not None:
            return value < self.below
        lo, hi = self.outside  # validated at start
        return value < lo or value > hi


class WatchRegistry:
    """All active watches on one agent."""

    _ids = itertools.count(1)

    def __init__(self) -> None:
        self._watches: dict[str, Watch] = {}

    def start(self, owner: int, path: str, above: float | None,
              below: float | None, outside: tuple[float, float] | None,
              stop: bool, debounce: float,
              lifetime: float | None) -> str:
        given = [c for c in (above, below, outside) if c is not None]
        if len(given) != 1:
            raise RpcError(
                "RobocoreError",
                "give exactly one of above= / below= / outside=(lo, hi)",
            )
        if outside is not None and outside[0] >= outside[1]:
            raise RpcError("RobocoreError", "outside needs lo < hi")
        if debounce < 0:
            raise RpcError("RobocoreError", "debounce must be >= 0")
        watch = Watch(
            id=f"w{next(self._ids)}",
            owner=owner,
            path=path,
            above=above,
            below=below,
            outside=tuple(outside) if outside is not None else None,
            stop=bool(stop),
            debounce=float(debounce),
            lifetime=float(lifetime) if lifetime is not None
            else DEFAULT_STOP_LIFETIME,
        )
        self._watches[watch.id] = watch
        return watch.id

    def poll(self, watch_id: str, owner: int) -> dict:
        watch = self._watches.get(watch_id)
        if watch is None or watch.owner != owner:
            raise RpcError("RobocoreError",
                           f"no watch {watch_id!r} owned by this client")
        return {
            "triggered": watch.triggered,
            "value": watch.value,
            "triggered_at": watch.triggered_at,
            "events": [list(e) for e in watch.events],
        }

    def stop(self, watch_id: str, owner: int) -> None:
        watch = self._watches.get(watch_id)
        if watch is None or watch.owner != owner:
            raise RpcError("RobocoreError",
                           f"no watch {watch_id!r} owned by this client")
        del self._watches[watch_id]

    def on_disconnect(self, owner: int) -> None:
        """Plain watches die with their owner; stop=True watches stay
        armed for their declared lifetime (spec section 16)."""
        now = time.monotonic()
        for watch in list(self._watches.values()):
            if watch.owner != owner:
                continue
            if watch.stop:
                watch.orphan_deadline = now + watch.lifetime
            else:
                del self._watches[watch.id]

    def evaluate(
        self,
        resolve: Callable[[str], float | None],
        on_fire: Callable[[Watch, float], None],
        now_mono: float | None = None,
        now_wall: float | None = None,
    ) -> None:
        """One sampler tick: update every watch against current state.

        ``resolve`` maps a path to its current value (None = no data yet:
        the watch keeps waiting). ``on_fire`` runs once per rising edge
        of a watch's (debounced) condition, BEFORE the tick returns —
        stop-watch halts must not wait for anything.
        """
        if now_mono is None:
            now_mono = time.monotonic()
        if now_wall is None:
            now_wall = time.time()
        for watch in list(self._watches.values()):
            if (watch.orphan_deadline is not None
                    and now_mono >= watch.orphan_deadline):
                del self._watches[watch.id]
                continue
            value = resolve(watch.path)
            if value is None:
                continue
            watch.value = value
            raw = watch.condition(value)
            if not raw:
                watch.holding_since = None
                watch.condition_active = False
                continue
            if watch.holding_since is None:
                watch.holding_since = now_mono
            held = now_mono - watch.holding_since
            if held < watch.debounce:
                continue
            if watch.condition_active:
                continue  # still the same episode; not a new edge
            watch.condition_active = True
            watch.triggered = True
            if watch.triggered_at is None:
                watch.triggered_at = now_wall
            if len(watch.events) >= _MAX_EVENTS:
                watch.events.pop(0)
            watch.events.append((now_wall, value))
            on_fire(watch, value)
