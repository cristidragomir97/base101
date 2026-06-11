"""Agent entry point: ``ros2 run robocore_agent agent --profile <yaml>``.

Starts a rclpy node (the ROS side; grows real subscriptions from Phase 3)
and the WebSocket JSON-RPC server. Exits cleanly on SIGINT/SIGTERM,
removing the unix socket file.

Failure modes: exits with code 1 and a one-line reason if the profile is
invalid or a listener cannot bind.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import os
import signal
import sys
import threading

import rclpy
from rclpy.node import Node

from .handlers import build_registry
from .profile import ProfileError, load_profile
from .server import AgentServer

log = logging.getLogger("robocore_agent")

# Production deployments should serve the client SDK's default,
# /run/robocore.sock (via systemd RuntimeDirectory). /tmp works unprivileged.
DEFAULT_SOCKET = "/tmp/robocore.sock"
DEFAULT_PORT = 7447


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="robocore_agent",
        description="robocore agent: robot-agnostic JSON-RPC bridge node",
    )
    parser.add_argument("--profile", required=True,
                        help="path to the robot profile YAML")
    parser.add_argument("--socket", default=DEFAULT_SOCKET,
                        help=f"unix socket path (default {DEFAULT_SOCKET}; "
                             "'none' disables)")
    parser.add_argument("--host", default="0.0.0.0",
                        help="TCP bind address (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"TCP port (default {DEFAULT_PORT}; 0 disables)")
    # ros2 run appends --ros-args; tolerate and ignore unknown trailing args.
    args, _unknown = parser.parse_known_args(argv)
    return args


async def _serve(server: AgentServer) -> None:
    """Run the server until SIGINT/SIGTERM."""
    await server.start()
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    log.info("agent ready")
    await stop.wait()
    log.info("shutting down")
    await server.close()


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] [%(name)s]: %(message)s",
    )
    args = _parse_args(argv if argv is not None else sys.argv[1:])

    try:
        profile = load_profile(args.profile)
    except ProfileError as exc:
        log.error("%s", exc)
        return 1
    log.info("profile %s (%s): capabilities %s",
             profile.info.name, profile.info.model,
             ", ".join(profile.info.capabilities) or "(none)")

    unix_path = None if args.socket == "none" else args.socket
    if unix_path is not None and os.path.exists(unix_path):
        os.unlink(unix_path)  # stale socket from an unclean previous exit
    port = None if args.port == 0 else args.port

    rclpy.init()
    node = Node("robocore_agent")
    # Phase 1: the node exists but observes nothing yet. Spinning it in a
    # daemon thread keeps the asyncio server as the main thread's loop.
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), name="rclpy-spin", daemon=True
    )
    spin_thread.start()

    server = AgentServer(
        registry=build_registry(profile),
        robot_name=profile.info.name,
        unix_path=unix_path,
        host=args.host,
        port=port,
    )
    exit_code = 0
    try:
        asyncio.run(_serve(server))
    except OSError as exc:
        log.error("cannot bind listener: %s", exc)
        exit_code = 1
    finally:
        node.destroy_node()
        with contextlib.suppress(Exception):
            rclpy.shutdown()
        if unix_path is not None:
            with contextlib.suppress(OSError):
                os.unlink(unix_path)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
