"""Task lifecycle on the agent (spec sections 5 and 23).

A task is an asyncio coroutine with an id, an owning client session, and a
state machine (pending → running → succeeded | failed | cancelled). Every
state change is pushed to the owner as a ``task.update`` notification and
recorded in the audit log.

Task bodies signal user-visible failure by raising server.RpcError (the
error type name reaches the client); any other exception becomes a generic
RobocoreError-typed failure.
"""

from __future__ import annotations

import asyncio
import itertools
import logging
from typing import Any, Awaitable, Callable

from robocore import wire
from robocore.models import TaskError, TaskUpdate

from .audit import AuditLog
from .server import RpcError, Session

log = logging.getLogger("robocore_agent.tasks")


class TaskHandle:
    """Passed to a task body so it can report progress."""

    def __init__(self, record: "TaskRecord", manager: "TaskManager") -> None:
        self._record = record
        self._manager = manager

    async def report(self, progress: float | None = None,
                     message: str | None = None) -> None:
        """Push a "running" task.update with optional progress (0..1)."""
        await self._manager._push(self._record, "running",
                                  progress=progress, message=message)


# A task body: takes the handle, returns the result dict (or None).
TaskBody = Callable[[TaskHandle], Awaitable[dict[str, Any] | None]]


class TaskRecord:
    """One task's bookkeeping."""

    _ids = itertools.count(1)

    def __init__(self, kind: str, session: Session) -> None:
        self.id = f"t{next(self._ids)}"
        self.kind = kind
        self.session = session
        self.state = "pending"
        self.aio_task: asyncio.Task | None = None


class TaskManager:
    """Creates, tracks, cancels tasks; emits updates + audit records."""

    def __init__(self, audit: AuditLog) -> None:
        self._audit = audit
        self._tasks: dict[str, TaskRecord] = {}

    def start(self, kind: str, session: Session, body: TaskBody) -> str:
        """Create a task and schedule it. Returns the task id immediately."""
        record = TaskRecord(kind, session)
        self._tasks[record.id] = record
        record.aio_task = asyncio.ensure_future(self._run(record, body))
        self._audit.record(
            "command", client=session.client_id, call=kind,
            task_id=record.id, outcome="started",
        )
        return record.id

    async def cancel(self, task_id: str, session: Session) -> None:
        """Cancel a task by id. Raises RpcError for unknown/finished ids."""
        record = self._tasks.get(task_id)
        if record is None or record.aio_task is None or record.aio_task.done():
            raise RpcError("RobocoreError",
                           f"no running task with id {task_id!r}")
        record.aio_task.cancel()

    async def _run(self, record: TaskRecord, body: TaskBody) -> None:
        await self._push(record, "running")
        try:
            result = await body(TaskHandle(record, self))
        except asyncio.CancelledError:
            await self._finish(record, "cancelled")
            return
        except RpcError as exc:
            await self._finish(record, "failed", error=TaskError(
                type=exc.error_type, message=str(exc), data=exc.extra))
            return
        except Exception as exc:  # task body bug: fail the task, log it
            log.exception("task %s (%s) crashed", record.id, record.kind)
            await self._finish(record, "failed", error=TaskError(
                type="RobocoreError", message=f"internal error: {exc}"))
            return
        await self._finish(record, "succeeded", result=result or {})

    async def _finish(
        self,
        record: TaskRecord,
        state: str,
        result: dict[str, Any] | None = None,
        error: TaskError | None = None,
    ) -> None:
        self._tasks.pop(record.id, None)
        outcome = state if state != "failed" else f"failed: {error.message}"
        self._audit.record(
            "result", client=record.session.client_id, call=record.kind,
            task_id=record.id, outcome=outcome,
        )
        await self._push(record, state, result=result, error=error,
                         progress=1.0 if state == "succeeded" else None)

    async def _push(
        self,
        record: TaskRecord,
        state: str,
        progress: float | None = None,
        message: str | None = None,
        result: dict[str, Any] | None = None,
        error: TaskError | None = None,
    ) -> None:
        """Send a task.update to the owning client. Best effort: a gone
        client must not kill the task."""
        record.state = state
        update = TaskUpdate(task_id=record.id, state=state, progress=progress,
                            message=message, result=result, error=error)
        try:
            await record.session.websocket.send(
                wire.notification("task.update",
                                  update.model_dump(mode="json"))
            )
        except Exception:
            pass  # owner disconnected; task continues, audit has the trail

    async def cancel_all(self) -> None:
        """Shutdown helper: cancel everything still running."""
        for record in list(self._tasks.values()):
            if record.aio_task is not None and not record.aio_task.done():
                record.aio_task.cancel()
