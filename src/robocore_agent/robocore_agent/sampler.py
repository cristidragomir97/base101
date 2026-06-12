"""The watch sampler: evaluates every active watch against live state.

Runs as one asyncio task started with the server. 20 Hz in v1 — the spec
says "source rate"; sampling decouples the safety path from subscription
callbacks and 50 ms is far inside any battery/effort reaction budget.
Documented deviation, revisit if a watch ever needs to catch a
single-message spike.

A ``stop=True`` watch firing halts all motion right here, before the
tick returns: teleop sessions end (velocity zeroed, motion lock
released), running tasks are cancelled. The hung/disconnected owner is
exactly the case this exists for (spec section 16, discipline 6).
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

from .watches import Watch

log = logging.getLogger("robocore_agent.sampler")

INTERVAL = 0.05  # 20 Hz


async def run_watch_sampler(ctx: Any, paths: dict, interval: float = INTERVAL) -> None:
    def resolve(path: str) -> float | None:
        getter = paths.get(path)
        return None if getter is None else getter()

    def on_fire(watch: Watch, value: float) -> None:
        ctx.events.emit("watch", {
            "watch_id": watch.id, "path": watch.path, "value": value,
            "stop": watch.stop,
        })
        if not watch.stop:
            return
        log.warning("stop-watch %s fired: %s = %s; halting all motion",
                    watch.id, watch.path, value)
        halted = []
        if ctx.teleop is not None:
            halted.append(f"{ctx.teleop.end_all()} teleop session(s)")
        cancelled = ctx.tasks.cancel_running()
        if cancelled:
            halted.append(f"{cancelled} task(s)")
        if ctx.ros is not None:
            try:
                ctx.ros.publish_zero()
            except Exception:
                log.exception("failed to publish zero twist on watch halt")
        ctx.audit.record(
            "safety", client=watch.owner, call="watch",
            outcome="watch halt",
            detail={"reason": "watch", "watch_id": watch.id,
                    "path": watch.path, "value": value,
                    "halted": halted},
        )

    while True:
        try:
            ctx.watches.evaluate(resolve, on_fire)
        except Exception:
            log.exception("watch sampler tick failed")
        await asyncio.sleep(interval)
