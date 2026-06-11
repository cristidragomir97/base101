"""The agent's WebSocket JSON-RPC server.

Serves the same protocol on a unix socket (local clients) and TCP (remote
clients) from one asyncio loop. Pure transport plus the two chokepoints
the spec demands of it (sections 18 and 21): every request is audited, and
motion-classed methods pass the safety layer before their handler runs.

Method behavior lives in handlers.py; this module never imports rclpy so
the whole wire stack stays unit-testable without a ROS environment.

Failure modes: a malformed frame gets a JSON-RPC parse/invalid-request
error back; an unknown method gets method-not-found; a handler exception
becomes an application error carrying the exception type name in
data["type"] (which the client maps back to a robocore exception).
"""

from __future__ import annotations

import asyncio
import itertools
import logging
from typing import TYPE_CHECKING, Any, Awaitable, Callable

import websockets

from robocore import wire
from robocore.aio.transport import MAX_MESSAGE_BYTES

if TYPE_CHECKING:
    from .context import AgentContext

log = logging.getLogger("robocore_agent.server")


class RpcError(Exception):
    """A handler-signaled error to send back to the client.

    ``error_type`` must name a robocore exception class (errors.py in the
    client SDK) so the client raises the right thing.
    """

    def __init__(self, error_type: str, message: str,
                 extra: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.error_type = error_type
        self.extra = extra or {}


class Session:
    """One connected client. Handlers receive this to send payloads and to
    check/flip per-connection state (handshake, audit subscription)."""

    _ids = itertools.count(1)

    def __init__(self, websocket: Any) -> None:
        self.websocket = websocket
        self.client_id = next(self._ids)
        self.handshaken = False
        self.audit_subscribed = False
        self._payload_ids = itertools.count(1)

    async def send_payload(self, kind: str, meta: dict[str, Any],
                           data: bytes) -> int:
        """Announce and send one binary payload; returns its payload id."""
        payload_id = next(self._payload_ids)
        await self.websocket.send(wire.payload_header(payload_id, kind, meta))
        await self.websocket.send(wire.encode_payload_frame(payload_id, data))
        return payload_id


# A handler takes (session, params) and returns the JSON-RPC result.
Handler = Callable[[Session, dict[str, Any]], Awaitable[Any]]


class AgentServer:
    """Owns the unix + TCP listeners and dispatches requests to handlers."""

    def __init__(
        self,
        registry: dict[str, Handler],
        ctx: "AgentContext",
        unix_path: str | None,
        host: str | None,
        port: int | None,
    ) -> None:
        if unix_path is None and port is None:
            raise ValueError("need at least one of unix_path / port")
        self._registry = registry
        self._ctx = ctx
        self._robot_name = ctx.profile.info.name
        self._unix_path = unix_path
        self._host = host or "0.0.0.0"
        self._port = port
        self._servers: list[Any] = []
        self._sessions: set[Session] = set()
        ctx.audit.subscribers.append(self._fan_out_audit_event)

    async def start(self) -> None:
        """Bind the listeners. Raises OSError if a bind fails."""
        # max_size must match the client transport's MAX_MESSAGE_BYTES;
        # websockets' 1 MiB default aborts connections on big payloads.
        if self._port is not None:
            self._servers.append(
                await websockets.serve(self._handle, self._host, self._port,
                                       max_size=MAX_MESSAGE_BYTES)
            )
            log.info("listening on ws://%s:%s", self._host, self._port)
        if self._unix_path is not None:
            self._servers.append(
                await websockets.unix_serve(self._handle, self._unix_path,
                                            max_size=MAX_MESSAGE_BYTES)
            )
            log.info("listening on unix://%s", self._unix_path)

    async def close(self) -> None:
        await self._ctx.tasks.cancel_all()
        for server in self._servers:
            server.close()
            await server.wait_closed()
        self._servers.clear()

    # -- per-connection ------------------------------------------------------

    async def _handle(self, websocket: Any) -> None:
        session = Session(websocket)
        self._sessions.add(session)
        self._ctx.audit.record("connection", client=session.client_id,
                               outcome="connected")
        try:
            async for message in websocket:
                if isinstance(message, bytes):
                    # Clients never send binary frames in protocol 1.
                    log.warning("client %d sent a binary frame; ignored",
                                session.client_id)
                    continue
                reply = await self._dispatch(session, message)
                if reply is not None:
                    await websocket.send(reply)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._sessions.discard(session)
            # A vanished client must not leave the robot moving or locked.
            if self._ctx.teleop is not None:
                self._ctx.teleop.on_disconnect(session.client_id)
            if self._ctx.profile.spec.safety.stop_on_disconnect:
                cancelled = self._ctx.tasks.cancel_for_session(session)
                if cancelled:
                    self._ctx.audit.record(
                        "safety", client=session.client_id,
                        outcome=f"stop_on_disconnect: cancelled {cancelled} "
                                "running task(s)",
                    )
            if self._ctx.lock.release(session.client_id):
                log.info("released motion lock of client %d",
                         session.client_id)
            self._ctx.audit.record("connection", client=session.client_id,
                                   outcome="disconnected")

    async def _dispatch(self, session: Session, text: str) -> str | None:
        try:
            msg = wire.parse_message(text)
        except ValueError as exc:
            return wire.error_response(None, wire.ERR_PARSE, str(exc))

        request_id = msg.get("id")
        method = msg.get("method")
        if method is None:
            if request_id is None:
                return None  # stray notification from client; ignore
            return wire.error_response(
                request_id, wire.ERR_INVALID_REQUEST, "missing method"
            )
        if request_id is None:
            return None  # client notifications are not part of protocol 1

        handler = self._registry.get(method)
        if handler is None:
            return wire.error_response(
                request_id, wire.ERR_METHOD_NOT_FOUND,
                f"unknown method {method!r}",
                self._error_data("RobocoreError", method, request_id),
            )
        if method != "hello" and not session.handshaken:
            return wire.error_response(
                request_id, wire.ERR_INVALID_REQUEST,
                "handshake required: call hello first",
                self._error_data("ProtocolMismatch", method, request_id),
            )

        params = msg.get("params") or {}
        audit = self._ctx.audit
        audit.record("command", client=session.client_id, call=method,
                     detail={"params": params})
        try:
            self._ctx.safety.check(session.client_id, method)
        except RpcError as exc:
            audit.record("safety", client=session.client_id, call=method,
                         outcome="rejected",
                         detail={"reason": exc.extra.get("reason"),
                                 "message": str(exc)})
            data = self._error_data(exc.error_type, method, request_id)
            data.update(exc.extra)
            return wire.error_response(
                request_id, wire.ERR_APPLICATION, str(exc), data
            )
        try:
            result = await handler(session, params)
        except RpcError as exc:
            audit.record("result", client=session.client_id, call=method,
                         outcome=f"error: {exc}")
            data = self._error_data(exc.error_type, method, request_id)
            data.update(exc.extra)
            return wire.error_response(
                request_id, wire.ERR_APPLICATION, str(exc), data
            )
        except Exception as exc:  # handler bug: report, don't kill the link
            log.exception("handler %s failed", method)
            audit.record("result", client=session.client_id, call=method,
                         outcome=f"error: internal: {exc}")
            return wire.error_response(
                request_id, wire.ERR_APPLICATION,
                f"internal error in {method}: {exc}",
                self._error_data("RobocoreError", method, request_id),
            )
        audit.record("result", client=session.client_id, call=method,
                     outcome="success")
        return wire.response(request_id, result)

    # -- audit tail fan-out ------------------------------------------------------

    def _fan_out_audit_event(self, event: Any) -> None:
        """AuditLog subscriber: push audit.event to subscribed sessions.

        Called synchronously from record() on the server's loop; the sends
        are scheduled, never awaited, so a slow client cannot stall
        dispatch.
        """
        subscribed = [s for s in self._sessions if s.audit_subscribed]
        if not subscribed:
            return
        text = wire.notification("audit.event", event.model_dump(mode="json"))
        for session in subscribed:
            asyncio.ensure_future(self._send_quietly(session, text))

    @staticmethod
    async def _send_quietly(session: Session, text: str) -> None:
        try:
            await session.websocket.send(text)
        except Exception:
            pass  # client gone; the disconnect path cleans up

    def _error_data(self, error_type: str, call: str,
                    request_id: int | str | None) -> dict[str, Any]:
        return {
            "type": error_type,
            "robot": self._robot_name,
            "call": call,
            "request_id": request_id,
        }
