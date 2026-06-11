"""The agent's WebSocket JSON-RPC server.

Serves the same protocol on a unix socket (local clients) and TCP (remote
clients) from one asyncio loop. Pure transport: method behavior lives in
handlers.py; this module never imports rclpy so it stays unit-testable
without a ROS environment.

Failure modes: a malformed frame gets a JSON-RPC parse/invalid-request
error back; an unknown method gets method-not-found; a handler exception
becomes an application error carrying the exception type name in
data["type"] (which the client maps back to a robocore exception).
"""

from __future__ import annotations

import asyncio
import itertools
import logging
from typing import Any, Awaitable, Callable

import websockets

from robocore import wire

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
    check/flip handshake state."""

    _ids = itertools.count(1)

    def __init__(self, websocket: Any) -> None:
        self.websocket = websocket
        self.client_id = next(self._ids)
        self.handshaken = False
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
        robot_name: str,
        unix_path: str | None,
        host: str | None,
        port: int | None,
    ) -> None:
        if unix_path is None and port is None:
            raise ValueError("need at least one of unix_path / port")
        self._registry = registry
        self._robot_name = robot_name
        self._unix_path = unix_path
        self._host = host or "0.0.0.0"
        self._port = port
        self._servers: list[Any] = []

    async def start(self) -> None:
        """Bind the listeners. Raises OSError if a bind fails."""
        if self._port is not None:
            self._servers.append(
                await websockets.serve(self._handle, self._host, self._port)
            )
            log.info("listening on ws://%s:%s", self._host, self._port)
        if self._unix_path is not None:
            self._servers.append(
                await websockets.unix_serve(self._handle, self._unix_path)
            )
            log.info("listening on unix://%s", self._unix_path)

    async def close(self) -> None:
        for server in self._servers:
            server.close()
            await server.wait_closed()
        self._servers.clear()

    # -- per-connection ------------------------------------------------------

    async def _handle(self, websocket: Any) -> None:
        session = Session(websocket)
        log.info("client %d connected", session.client_id)
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
            log.info("client %d disconnected", session.client_id)

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
        try:
            result = await handler(session, params)
        except RpcError as exc:
            data = self._error_data(exc.error_type, method, request_id)
            data.update(exc.extra)
            return wire.error_response(
                request_id, wire.ERR_APPLICATION, str(exc), data
            )
        except Exception as exc:  # handler bug: report, don't kill the link
            log.exception("handler %s failed", method)
            return wire.error_response(
                request_id, wire.ERR_APPLICATION,
                f"internal error in {method}: {exc}",
                self._error_data("RobocoreError", method, request_id),
            )
        return wire.response(request_id, result)

    def _error_data(self, error_type: str, call: str,
                    request_id: int | str | None) -> dict[str, Any]:
        return {
            "type": error_type,
            "robot": self._robot_name,
            "call": call,
            "request_id": request_id,
        }
