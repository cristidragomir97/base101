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

from robocore.models import Hello, Point, Pose, Transform, Welcome
from robocore.version import PROTOCOL_VERSION

from .context import AgentContext
from .server import Handler, RpcError, Session
from .tasks import TaskHandle

# debug.send_payload is test plumbing, not robot data; keep it small.
_MAX_DEBUG_PAYLOAD = 16 * 1024 * 1024


def _require_ros(ctx: AgentContext) -> Any:
    """Handlers that touch the robot need the ROS interface."""
    if ctx.ros is None:
        raise RpcError("RobocoreError",
                       "agent is running without a ROS interface (test mode)")
    return ctx.ros


def _require_teleop(ctx: AgentContext) -> Any:
    """Teleop handlers need the manager (absent without ROS)."""
    if ctx.teleop is None:
        raise RpcError("RobocoreError",
                       "agent is running without a ROS interface (test mode)")
    return ctx.teleop


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

    # -- mobility state / TF (Phase 3) ----------------------------------------------

    async def mobility_get_state(session: Session,
                                 params: dict[str, Any]) -> Any:
        ros = _require_ros(ctx)
        try:
            return await asyncio.to_thread(ros.get_state)
        except Exception as exc:
            raise RpcError("RobocoreError", str(exc)) from exc

    async def tf_lookup(session: Session, params: dict[str, Any]) -> Any:
        ros = _require_ros(ctx)
        try:
            return await asyncio.to_thread(
                ros.tf_lookup,
                str(params["parent"]), str(params["child"]),
                float(params.get("timeout", 2.0)),
            )
        except KeyError as exc:
            raise RpcError("RobocoreError", f"missing param {exc}") from exc
        except Exception as exc:
            raise RpcError("TimeoutError", str(exc)) from exc

    async def tf_frames(session: Session, params: dict[str, Any]) -> Any:
        ros = _require_ros(ctx)
        return {"frames": await asyncio.to_thread(ros.tf_frames)}

    async def tf_wait_for(session: Session, params: dict[str, Any]) -> Any:
        ros = _require_ros(ctx)
        try:
            await asyncio.to_thread(
                ros.tf_wait_for, str(params["frame"]),
                float(params.get("timeout", 5.0)),
            )
        except Exception as exc:
            raise RpcError("TimeoutError", str(exc)) from exc
        return {}

    async def tf_transform(session: Session, params: dict[str, Any]) -> Any:
        # Re-express a Pose or Point in another frame: one TF lookup, then
        # the shared spatial math. kind selects the entity type.
        ros = _require_ros(ctx)
        kind = params.get("kind")
        if kind not in ("pose", "point"):
            raise RpcError("RobocoreError", "kind must be 'pose' or 'point'")
        model = Pose if kind == "pose" else Point
        try:
            entity = model.model_validate(params["entity"])
        except (KeyError, ValidationError) as exc:
            raise RpcError("RobocoreError", f"bad entity: {exc}") from exc
        try:
            raw = await asyncio.to_thread(
                ros.tf_lookup, str(params["to_frame"]), entity.frame,
                float(params.get("timeout", 2.0)),
            )
        except Exception as exc:
            raise RpcError("TimeoutError", str(exc)) from exc
        transform = Transform.model_validate(raw)
        if kind == "point":
            moved = transform.apply(entity)
        else:
            x, y, z = transform.rotation.rotate(entity.x, entity.y, entity.z)
            moved = Pose(
                x=x + transform.translation.x,
                y=y + transform.translation.y,
                z=z + transform.translation.z,
                q=transform.rotation.multiply(entity.q),
                frame=transform.parent,
            )
        return {"entity": moved.model_dump(mode="json")}

    # -- teleop (Phase 3) --------------------------------------------------------------

    async def teleop_start(session: Session, params: dict[str, Any]) -> Any:
        teleop = _require_teleop(ctx)
        granted = teleop.start(
            session.client_id, float(params.get("watchdog", 0.5)))
        return {"watchdog": granted}

    async def teleop_drive(session: Session, params: dict[str, Any]) -> Any:
        teleop = _require_teleop(ctx)
        teleop.drive(
            session.client_id,
            linear=float(params.get("linear", 0.0)),
            angular=float(params.get("angular", 0.0)),
            lateral=float(params.get("lateral", 0.0)),
        )
        return {}

    async def teleop_stop(session: Session, params: dict[str, Any]) -> Any:
        _require_teleop(ctx).stop(session.client_id)
        return {}

    async def teleop_end(session: Session, params: dict[str, Any]) -> Any:
        _require_teleop(ctx).end(session.client_id)
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
        if engaged and ctx.teleop is not None:
            ctx.teleop.zero_all()  # halt now, not at the next watchdog tick
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
        # TF is not capability-gated: every ROS robot has a TF tree.
        "tf.lookup": tf_lookup,
        "tf.frames": tf_frames,
        "tf.wait_for": tf_wait_for,
        "tf.transform": tf_transform,
    }
    if "mobility" in profile.info.capabilities:
        registry["mobility.navigate_to"] = navigate_to
        registry["mobility.get_state"] = mobility_get_state
    if "teleop" in profile.info.capabilities:
        registry["teleop.start"] = teleop_start
        registry["teleop.drive"] = teleop_drive
        registry["teleop.stop"] = teleop_stop
        registry["teleop.end"] = teleop_end
    return registry
