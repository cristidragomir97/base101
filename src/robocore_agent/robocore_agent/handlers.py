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

from robocore.models import (
    CameraIntrinsics,
    Hello,
    Point,
    Pose,
    Transform,
    Welcome,
)
from robocore.version import PROTOCOL_VERSION

from . import sensing
from .context import AgentContext
from .server import Handler, RpcError, Session
from .state_paths import validate_path
from .tasks import TaskHandle

# debug.send_payload is test plumbing, not robot data; keep it small.
_MAX_DEBUG_PAYLOAD = 16 * 1024 * 1024

# Server-side cap on lidar stream rates.
_MAX_LIDAR_HZ = 20.0


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

    # -- cameras (Phase 4) -------------------------------------------------------

    def _camera_cfg(params: dict[str, Any]) -> tuple[str, Any]:
        cameras = profile.spec.cameras or {}
        name = params.get("camera")
        if not isinstance(name, str) or name not in cameras:
            raise RpcError(
                "RobocoreError",
                f"unknown camera {name!r} (profile has {sorted(cameras)})",
            )
        return name, cameras[name]

    async def _capture(name: str, cfg: Any, need_rgb: bool,
                       need_depth: bool) -> tuple[Any, dict[str, Any]]:
        """Sample the camera, snapshot TF, store the deprojection record.

        Returns (record, sample). The record id is the frame handle the
        client gets; it carries depth + capture-time transforms so
        deproject stays paired with THIS frame.
        """
        ros = _require_ros(ctx)

        def grab() -> tuple[dict[str, Any], dict[str, dict]]:
            sample = ros.camera_sample(name, need_rgb, need_depth)
            return sample, ros.snapshot_transforms(sample["frame_id"])

        try:
            sample, raw_transforms = await asyncio.to_thread(grab)
        except RpcError:
            raise
        except Exception as exc:
            raise RpcError("RobocoreError", str(exc)) from exc
        record = sensing.FrameRecord(
            camera=name,
            stamp=sample["stamp"],
            tf_frame=sample["frame_id"],
            optical=cfg.optical,
            intrinsics=(None if sample["intrinsics"] is None
                        else CameraIntrinsics.model_validate(
                            sample["intrinsics"])),
            depth=sample["depth"],
            rgb=sample["rgb"],
            transforms={target: Transform.model_validate(raw)
                        for target, raw in raw_transforms.items()},
        )
        ctx.frames.store(record, ttl=cfg.frame_ttl)
        return record, sample

    def _shm_put(array: Any, ttl: float | None = None) -> dict[str, Any]:
        if ctx.shm is None:
            raise RpcError("RobocoreError",
                           "agent has no shared-memory store (test mode)")
        ref = ctx.shm.put(array) if ttl is None else ctx.shm.put(array, ttl)
        return ref.model_dump(mode="json")

    async def camera_info(session: Session, params: dict[str, Any]) -> Any:
        name, cfg = _camera_cfg(params)
        ros = _require_ros(ctx)
        try:
            meta = await asyncio.to_thread(ros.camera_meta, name)
        except Exception as exc:
            raise RpcError("RobocoreError", str(exc)) from exc
        return {
            "camera": name,
            "has_rgb": bool(cfg.rgb),
            "has_depth": bool(cfg.depth),
            "frame_ttl": cfg.frame_ttl,
            "max_stream_fps": cfg.max_stream_fps,
            "intrinsics": meta["intrinsics"],
            "frame": meta["frame_id"],
        }

    async def camera_get_frame(session: Session,
                               params: dict[str, Any]) -> Any:
        name, cfg = _camera_cfg(params)
        record, sample = await _capture(name, cfg, need_rgb=True,
                                        need_depth=False)
        return {
            "frame_id": record.id,
            "camera": name,
            "stamp": sample["rgb_stamp"],
            "payload": _shm_put(sample["rgb"]),
        }

    async def camera_get_depth(session: Session,
                               params: dict[str, Any]) -> Any:
        name, cfg = _camera_cfg(params)
        record, sample = await _capture(name, cfg, need_rgb=False,
                                        need_depth=True)
        return {
            "frame_id": record.id,
            "camera": name,
            "stamp": sample["depth_stamp"],
            "payload": _shm_put(sample["depth"]),
        }

    async def camera_get_synced(session: Session,
                                params: dict[str, Any]) -> Any:
        name, cfg = _camera_cfg(params)
        record, sample = await _capture(name, cfg, need_rgb=True,
                                        need_depth=True)
        return {
            "frame": {
                "frame_id": record.id,
                "camera": name,
                "stamp": sample["rgb_stamp"],
                "payload": _shm_put(sample["rgb"]),
            },
            "depth": {
                "frame_id": record.id,
                "camera": name,
                "stamp": sample["depth_stamp"],
                "payload": _shm_put(sample["depth"]),
            },
        }

    async def camera_get_cloud(session: Session,
                               params: dict[str, Any]) -> Any:
        name, cfg = _camera_cfg(params)
        voxel = params.get("voxel")
        if not isinstance(voxel, (int, float)) or voxel <= 0:
            raise RpcError(
                "RobocoreError",
                "voxel is mandatory and must be > 0 (meters) — the API's "
                "size budget; pick the coarsest voxel your task tolerates",
            )
        record, sample = await _capture(name, cfg, need_rgb=False,
                                        need_depth=True)
        if record.intrinsics is None:
            raise RpcError(
                "RobocoreError",
                f"camera {name!r} has no intrinsics (no camera_info topic "
                "in the profile); cannot build a cloud",
            )
        rgb = record.rgb
        if rgb is None and cfg.rgb:
            # Colors are best-effort: use the synced rgb if one exists.
            ros = _require_ros(ctx)
            try:
                extra = await asyncio.to_thread(
                    ros.camera_sample, name, True, True)
                rgb = extra["rgb"]
            except Exception:
                rgb = None
        points, colors = await asyncio.to_thread(
            sensing.make_cloud, sample["depth"], rgb, record.intrinsics,
            float(voxel), record.optical,
        )
        result = {
            "stamp": sample["depth_stamp"],
            "frame": record.tf_frame,
            "count": int(points.shape[0]),
            "points": _shm_put(points),
            "colors": None if colors is None else _shm_put(colors),
        }
        return result

    async def camera_deproject(session: Session,
                               params: dict[str, Any]) -> Any:
        ros = _require_ros(ctx)
        frame_id = params.get("frame_id")
        if not isinstance(frame_id, str):
            raise RpcError("RobocoreError", "frame_id must be a string")
        pixel = params.get("pixel")
        if (not isinstance(pixel, (list, tuple)) or len(pixel) != 2
                or not all(isinstance(c, (int, float)) for c in pixel)):
            raise RpcError("RobocoreError", "pixel must be [u, v]")
        record = ctx.frames.get(frame_id)
        in_frame = params.get("in_frame")
        if in_frame is not None:
            in_frame = ros.resolve_frame(str(in_frame))
        point = sensing.deproject(record, (pixel[0], pixel[1]), in_frame)
        return {"point": point}

    async def camera_stream_start(session: Session,
                                  params: dict[str, Any]) -> Any:
        name, cfg = _camera_cfg(params)
        ros = _require_ros(ctx)
        fps = float(params.get("fps", 5.0))
        fps = min(max(fps, 0.1), cfg.max_stream_fps)
        shm_ttl = max(2.0, 3.0 / fps)

        async def tick() -> dict[str, Any] | None:
            try:
                record, sample = await _capture(name, cfg, need_rgb=True,
                                                need_depth=False)
            except RpcError:
                return None  # sensor warming up / TF gap: skip this tick
            return {
                "kind": "camera.frame",
                "camera": name,
                "frame_id": record.id,
                "stamp": sample["rgb_stamp"],
                "payload": _shm_put(sample["rgb"], ttl=shm_ttl),
            }

        stream_id = ctx.streams.start(session, 1.0 / fps, tick)
        return {"stream_id": stream_id, "fps": fps}

    async def camera_stream_stop(session: Session,
                                 params: dict[str, Any]) -> Any:
        ctx.streams.stop(str(params.get("stream_id")), session)
        return {}

    # -- lidar (Phase 4) ---------------------------------------------------------

    async def _lidar_tick() -> dict[str, Any]:
        ros = _require_ros(ctx)
        try:
            sample = await asyncio.to_thread(ros.lidar_sample)
        except RpcError:
            raise
        except Exception as exc:
            raise RpcError("RobocoreError", str(exc)) from exc
        ranges = sample.pop("ranges")
        sample["ranges"] = _shm_put(ranges, ttl=10.0)
        return sample

    async def lidar_scan(session: Session, params: dict[str, Any]) -> Any:
        return await _lidar_tick()

    async def lidar_stream_start(session: Session,
                                 params: dict[str, Any]) -> Any:
        hz = float(params.get("hz", 5.0))
        hz = min(max(hz, 0.1), _MAX_LIDAR_HZ)

        async def tick() -> dict[str, Any] | None:
            try:
                sample = await _lidar_tick()
            except RpcError:
                return None
            sample["kind"] = "lidar.scan"
            return sample

        stream_id = ctx.streams.start(session, 1.0 / hz, tick)
        return {"stream_id": stream_id, "hz": hz}

    async def lidar_stream_stop(session: Session,
                                params: dict[str, Any]) -> Any:
        ctx.streams.stop(str(params.get("stream_id")), session)
        return {}

    # -- joint states (Phase 4, controller-level, pre-MoveIt) -----------------------

    async def arm_joint_states(session: Session,
                               params: dict[str, Any]) -> Any:
        ros = _require_ros(ctx)
        manipulation = profile.spec.manipulation
        arms = manipulation.arms if manipulation is not None else {}
        name = params.get("arm")
        if not isinstance(name, str) or name not in arms:
            raise RpcError(
                "RobocoreError",
                f"unknown arm {name!r} (profile has {sorted(arms)})",
            )
        joints = list(arms[name].joints)
        if not joints:
            raise RpcError(
                "RobocoreError",
                f"arm {name!r} declares no joints in the profile "
                "(manipulation.arms.<name>.joints)",
            )
        snapshot = await asyncio.to_thread(ros.joint_snapshot)
        limits = await asyncio.to_thread(ros.joint_limits)
        missing = [j for j in joints if j not in snapshot]
        if missing:
            raise RpcError(
                "RobocoreError",
                f"no joint state yet for {missing} (joint_state_broadcaster "
                "running? profile joint_states topic correct?)",
            )
        return {"joints": {
            j: {
                "position": snapshot[j][0],
                "velocity": snapshot[j][1],
                "effort": snapshot[j][2],
                "limits": limits.get(j),
            }
            for j in joints
        }}

    # -- watches (Phase 4) -------------------------------------------------------

    async def watch_start(session: Session, params: dict[str, Any]) -> Any:
        path = params.get("path")
        if not isinstance(path, str):
            raise RpcError("RobocoreError", "path must be a string")
        validate_path(ctx.watch_paths or {}, path)
        outside = params.get("outside")
        if outside is not None:
            if (not isinstance(outside, (list, tuple)) or len(outside) != 2):
                raise RpcError("RobocoreError", "outside must be [lo, hi]")
            outside = (float(outside[0]), float(outside[1]))
        above = params.get("above")
        below = params.get("below")
        watch_id = ctx.watches.start(
            owner=session.client_id,
            path=path,
            above=None if above is None else float(above),
            below=None if below is None else float(below),
            outside=outside,
            stop=bool(params.get("stop", False)),
            debounce=float(params.get("debounce", 0.0)),
            lifetime=params.get("lifetime"),
        )
        ctx.audit.record(
            "command", client=session.client_id, call="watch.start",
            outcome=watch_id,
            detail={"path": path, "above": above, "below": below,
                    "outside": list(outside) if outside else None,
                    "stop": bool(params.get("stop", False))},
        )
        return {"watch_id": watch_id}

    async def watch_poll(session: Session, params: dict[str, Any]) -> Any:
        return ctx.watches.poll(str(params.get("watch_id")),
                                session.client_id)

    async def watch_stop(session: Session, params: dict[str, Any]) -> Any:
        ctx.watches.stop(str(params.get("watch_id")), session.client_id)
        return {}

    # -- status / events (Phase 4) -------------------------------------------------

    async def status_get(session: Session, params: dict[str, Any]) -> Any:
        battery = None
        if (ctx.ros is not None and profile.spec.status is not None
                and profile.spec.status.battery):
            level, charging = ctx.ros.battery_state()
            battery = {"level": level, "is_charging": charging}
        return {"battery": battery, "estop": ctx.state.estop_engaged}

    async def events_subscribe(session: Session,
                               params: dict[str, Any]) -> Any:
        session.events_subscribed = True
        return {}

    async def events_unsubscribe(session: Session,
                                 params: dict[str, Any]) -> Any:
        session.events_subscribed = False
        return {}

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
        ctx.events.emit("estop", {"engaged": engaged})
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
        # Events are not capability-gated: estop/watch events apply to
        # every robot.
        "events.subscribe": events_subscribe,
        "events.unsubscribe": events_unsubscribe,
    }
    if "mobility" in profile.info.capabilities:
        registry["mobility.navigate_to"] = navigate_to
        registry["mobility.get_state"] = mobility_get_state
    if "teleop" in profile.info.capabilities:
        registry["teleop.start"] = teleop_start
        registry["teleop.drive"] = teleop_drive
        registry["teleop.stop"] = teleop_stop
        registry["teleop.end"] = teleop_end
    if "cameras" in profile.info.capabilities:
        registry["camera.info"] = camera_info
        registry["camera.get_frame"] = camera_get_frame
        registry["camera.get_depth"] = camera_get_depth
        registry["camera.get_synced"] = camera_get_synced
        registry["camera.get_cloud"] = camera_get_cloud
        registry["camera.deproject"] = camera_deproject
        registry["camera.stream_start"] = camera_stream_start
        registry["camera.stream_stop"] = camera_stream_stop
    if "lidar" in profile.info.capabilities:
        registry["lidar.scan"] = lidar_scan
        registry["lidar.stream_start"] = lidar_stream_start
        registry["lidar.stream_stop"] = lidar_stream_stop
    if "manipulation" in profile.info.capabilities:
        registry["arm.joint_states"] = arm_joint_states
    if "watches" in profile.info.capabilities:
        registry["watch.start"] = watch_start
        registry["watch.poll"] = watch_poll
        registry["watch.stop"] = watch_stop
    if "status" in profile.info.capabilities:
        registry["status.get"] = status_get
    return registry
