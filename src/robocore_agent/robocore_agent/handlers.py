"""Wire method handlers.

Each handler is ``async def name(session, params) -> result``. Handlers
signal client-visible failures by raising server.RpcError with the name of
a robocore exception class. New methods register in build_registry and in
scripts/gen_protocol.py (engine/) so protocol.json stays truthful.

Capability-dependent methods are only registered when the profile declares
the capability, so a chassis-only robot answers method-not-found for arm
calls (the client SDK raises CapabilityNotSupported before the wire).
"""

from __future__ import annotations

import asyncio
import hashlib
import os
from typing import Any

from pydantic import ValidationError

from robocore.models import Hello, Welcome
from robocore.version import PROTOCOL_VERSION

from .context import AgentContext
from .server import Handler, RpcError, Session
from .tasks import TaskHandle

# debug.send_payload is test plumbing, not robot data; keep it small.
_MAX_DEBUG_PAYLOAD = 16 * 1024 * 1024


def build_registry(ctx: AgentContext) -> dict[str, Handler]:
    """Build the method table for one loaded profile."""

    profile = ctx.profile

    # -- handshake / diagnostics ----------------------------------------------

    async def hello(session: Session, params: dict[str, Any]) -> Any:
        try:
            request = Hello.model_validate(params)
        except ValidationError as exc:
            raise RpcError("ProtocolMismatch", f"bad hello: {exc}") from exc
        if request.protocol != PROTOCOL_VERSION:
            raise RpcError(
                "ProtocolMismatch",
                f"client speaks protocol {request.protocol}, "
                f"agent speaks {PROTOCOL_VERSION}",
            )
        session.handshaken = True
        return Welcome(
            protocol=PROTOCOL_VERSION,
            profile=profile.info,
            capabilities=profile.info.capabilities,
            instances=profile.instances,
        ).model_dump(mode="json")

    async def ping(session: Session, params: dict[str, Any]) -> Any:
        return {}

    # -- tasks -------------------------------------------------------------------

    async def task_cancel(session: Session, params: dict[str, Any]) -> Any:
        task_id = params.get("task_id")
        if not isinstance(task_id, str):
            raise RpcError("RobocoreError", "task_id must be a string")
        await ctx.tasks.cancel(task_id, session)
        return {}

    # -- audit -------------------------------------------------------------------

    async def audit_query(session: Session, params: dict[str, Any]) -> Any:
        events = ctx.audit.query(
            since=params.get("since"),
            kind=params.get("kind"),
            task_id=params.get("task_id"),
            limit=params.get("limit", 500),
        )
        return {"events": [event.model_dump(mode="json") for event in events]}

    async def audit_subscribe(session: Session, params: dict[str, Any]) -> Any:
        session.audit_subscribed = True
        return {}

    async def audit_unsubscribe(session: Session,
                                params: dict[str, Any]) -> Any:
        session.audit_subscribed = False
        return {}

    # -- mobility (Phase 2 stub) ---------------------------------------------------

    async def navigate_to(session: Session, params: dict[str, Any]) -> Any:
        # Phase 3/5 replace this body with real Nav2 execution. The safety
        # layer has already vetted the call (e-stop, motion lock) in
        # dispatch; what remains always fails, explicitly.
        async def body(handle: TaskHandle) -> dict[str, Any]:
            raise RpcError(
                "NavigationFailed",
                "mobility.navigate_to is not implemented until Phase 3",
                {"reason": "not_implemented"},
            )

        task_id = ctx.tasks.start("mobility.navigate_to", session, body)
        return {"task_id": task_id}

    # -- debug (test plumbing, not robot API) ----------------------------------------

    async def debug_send_payload(session: Session,
                                 params: dict[str, Any]) -> Any:
        # Exercises the binary payload channel until real image methods
        # exist (Phase 4). Sends `size` random bytes, returns the payload
        # id and sha256 so the client can verify integrity.
        size = params.get("size", 1024)
        if not isinstance(size, int) or not 0 <= size <= _MAX_DEBUG_PAYLOAD:
            raise RpcError(
                "RobocoreError",
                f"size must be an int in [0, {_MAX_DEBUG_PAYLOAD}]",
            )
        data = os.urandom(size)
        payload_id = await session.send_payload(
            kind="debug/random", meta={"size": size}, data=data
        )
        return {
            "payload_id": payload_id,
            "sha256": hashlib.sha256(data).hexdigest(),
        }

    async def debug_set_estop(session: Session,
                              params: dict[str, Any]) -> Any:
        # Simulated e-stop for profiles with no hardware e-stop topic.
        engaged = params.get("engaged")
        if not isinstance(engaged, bool):
            raise RpcError("RobocoreError", "engaged must be a bool")
        ctx.state.estop_engaged = engaged
        ctx.audit.record("estop", client=session.client_id,
                         outcome="engaged" if engaged else "released")
        return {"engaged": engaged}

    async def debug_run_task(session: Session,
                             params: dict[str, Any]) -> Any:
        # Exercises the task lifecycle until real long-running verbs exist:
        # ticks progress updates over `duration` seconds, cancellable,
        # optionally failing on purpose.
        duration = float(params.get("duration", 1.0))
        ticks = int(params.get("ticks", 10))
        fail = bool(params.get("fail", False))
        if not 0 < duration <= 600 or not 1 <= ticks <= 1000:
            raise RpcError("RobocoreError",
                           "need 0 < duration <= 600 and 1 <= ticks <= 1000")

        async def body(handle: TaskHandle) -> dict[str, Any]:
            for tick in range(ticks):
                await asyncio.sleep(duration / ticks)
                await handle.report(progress=(tick + 1) / ticks)
            if fail:
                raise RpcError("ExecutionFailed", "deliberate test failure")
            return {"ticks": ticks}

        task_id = ctx.tasks.start("debug.run_task", session, body)
        return {"task_id": task_id}

    registry: dict[str, Handler] = {
        "hello": hello,
        "ping": ping,
        "task.cancel": task_cancel,
        "audit.query": audit_query,
        "audit.subscribe": audit_subscribe,
        "audit.unsubscribe": audit_unsubscribe,
        "debug.send_payload": debug_send_payload,
        "debug.set_estop": debug_set_estop,
        "debug.run_task": debug_run_task,
    }
    if "mobility" in profile.info.capabilities:
        registry["mobility.navigate_to"] = navigate_to
    return registry
