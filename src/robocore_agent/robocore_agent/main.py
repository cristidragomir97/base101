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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .bootstrap import build_agent
from .profile import ProfileError, load_profile
from .ros import RosInterface

log = logging.getLogger("robocore_agent")

# Production deployments should serve the client SDK's default,
# /run/robocore.sock (via systemd RuntimeDirectory). /tmp works unprivileged.
DEFAULT_SOCKET = "/tmp/robocore.sock"
# 10101 (Cristi, 2026-06-11): 7447 collides with zenoh's TCP default and
# the DDS UDP port range. Must match robocore.uri.DEFAULT_PORT.
DEFAULT_PORT = 10101


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
    parser.add_argument("--audit-dir", default=None,
                        help="override the audit log directory "
                             "(default: profile audit.dir or "
                             "~/.robocore/audit/<robot-name>)")
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

    # Pass the raw argv through so --ros-args (e.g. -p use_sim_time:=true,
    # required when running against Gazebo) reaches the node.
    rclpy.init(args=sys.argv[1:] if argv is None else argv)
    node = Node("robocore_agent")
    ros = RosInterface(node, profile)
    # ROS spins on a background MultiThreadedExecutor; the asyncio server
    # owns the main thread. Handlers reach ROS only through RosInterface,
    # whose blocking methods they call via asyncio.to_thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, name="ros-executor", daemon=True
    )
    spin_thread.start()

    server, ctx = build_agent(
        profile,
        unix_path=unix_path,
        host=args.host,
        port=port,
        audit_dir=args.audit_dir,
        ros=ros,
    )
    exit_code = 0
    try:
        asyncio.run(_serve(server))
    except OSError as exc:
        log.error("cannot bind listener: %s", exc)
        exit_code = 1
    finally:
        ctx.audit.close()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        with contextlib.suppress(Exception):
            rclpy.shutdown()
        if unix_path is not None:
            with contextlib.suppress(OSError):
                os.unlink(unix_path)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
