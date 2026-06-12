"""Server-pushed sensor streams (camera.stream / lidar.stream).

A stream is an agent-side ticker: every interval it samples a sensor and
pushes one ``stream.data`` notification whose payload arrays travel via
shared memory descriptors like every other large payload. The client
generator consumes the notifications; breaking the generator sends
stream.stop. A vanished client's streams are cancelled on disconnect.
"""

from __future__ import annotations

import asyncio
import itertools
import logging
import time
from typing import Any, Awaitable, Callable

from robocore import wire

from .server import RpcError, Session

log = logging.getLogger("robocore_agent.streams")

# A tick produces the notification params (without stream_id), or None
# to skip this tick (sensor has no data yet).
Tick = Callable[[], Awaitable[dict[str, Any] | None]]

MAX_STREAMS_PER_CLIENT = 8


class StreamManager:
    """Owns every running stream ticker."""

    _ids = itertools.count(1)

    def __init__(self) -> None:
        self._streams: dict[str, tuple[Session, asyncio.Task]] = {}

    def start(self, session: Session, interval: float, tick: Tick) -> str:
        mine = sum(1 for s, _t in self._streams.values() if s is session)
        if mine >= MAX_STREAMS_PER_CLIENT:
            raise RpcError(
                "RobocoreError",
                f"too many open streams (max {MAX_STREAMS_PER_CLIENT}); "
                "close some first",
            )
        stream_id = f"s{next(self._ids)}"
        task = asyncio.ensure_future(
            self._run(stream_id, session, interval, tick))
        self._streams[stream_id] = (session, task)
        return stream_id

    def stop(self, stream_id: str, session: Session) -> None:
        entry = self._streams.pop(stream_id, None)
        if entry is None or entry[0] is not session:
            if entry is not None:
                self._streams[stream_id] = entry
            raise RpcError("RobocoreError",
                           f"no stream {stream_id!r} owned by this client")
        entry[1].cancel()

    def on_disconnect(self, session: Session) -> None:
        for stream_id, (owner, task) in list(self._streams.items()):
            if owner is session:
                task.cancel()
                del self._streams[stream_id]

    def close(self) -> None:
        for _owner, task in self._streams.values():
            task.cancel()
        self._streams.clear()

    async def _run(self, stream_id: str, session: Session,
                   interval: float, tick: Tick) -> None:
        try:
            while True:
                started = time.monotonic()
                try:
                    params = await tick()
                except RpcError as exc:
                    await self._end(stream_id, session, error=str(exc))
                    return
                if params is not None:
                    params["stream_id"] = stream_id
                    await session.websocket.send(
                        wire.notification("stream.data", params))
                elapsed = time.monotonic() - started
                await asyncio.sleep(max(0.0, interval - elapsed))
        except asyncio.CancelledError:
            raise
        except Exception:
            # Send failure (client gone) or a tick bug: stop quietly,
            # the disconnect path / audit has the rest.
            log.debug("stream %s ended", stream_id, exc_info=True)
            self._streams.pop(stream_id, None)

    async def _end(self, stream_id: str, session: Session,
                   error: str | None) -> None:
        # Sent on the same notification method as the data so the client
        # generator sees data and termination in one ordered queue.
        self._streams.pop(stream_id, None)
        params: dict[str, Any] = {"stream_id": stream_id, "end": True}
        if error is not None:
            params["error"] = error
        try:
            await session.websocket.send(
                wire.notification("stream.data", params))
        except Exception:
            pass  # client gone
