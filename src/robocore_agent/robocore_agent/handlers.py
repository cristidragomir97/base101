"""Wire method handlers (Phase 1: handshake, ping, binary-channel test).

Each handler is ``async def name(session, params) -> result``. Handlers
signal client-visible failures by raising server.RpcError with the name of
a robocore exception class. New methods register in build_registry and in
scripts/gen_protocol.py (engine/) so protocol.json stays truthful.
"""

from __future__ import annotations

import hashlib
import os
from typing import Any

from pydantic import ValidationError

from robocore.models import Hello, Welcome
from robocore.version import PROTOCOL_VERSION

from .profile import Profile
from .server import Handler, RpcError, Session

# debug.send_payload is test plumbing, not robot data; keep it small.
_MAX_DEBUG_PAYLOAD = 16 * 1024 * 1024


def build_registry(profile: Profile) -> dict[str, Handler]:
    """Build the method table for one loaded profile."""

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
        ).model_dump(mode="json")

    async def ping(session: Session, params: dict[str, Any]) -> Any:
        return {}

    async def debug_send_payload(session: Session,
                                 params: dict[str, Any]) -> Any:
        # Test-only: exercises the binary payload channel until real image
        # methods exist (Phase 4). Sends `size` random bytes, returns the
        # payload id and sha256 so the client can verify integrity.
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

    return {
        "hello": hello,
        "ping": ping,
        "debug.send_payload": debug_send_payload,
    }
