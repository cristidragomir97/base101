"""SLAM orchestration (spec section 11): the mode_manager's successor.

slam_toolbox stays up for the whole session (base101_slam launch);
everything here is service calls and bookkeeping, never process control.
Modes (Cristi's ruling, 2026-06-12): start() resumes mapping, stop()
pauses it (idle), load()/localize() switch to localization via
deserialize. Userland lifecycle control comes later; for now the nodes
are assumed up.

Map storage layout (one directory per map under the profile's
slam.map_dir, default ~/.robocore/maps):

    <map_dir>/<name>/<name>.posegraph + .data   serialized pose graph
    <map_dir>/<name>/grid.pgm + grid.yaml       2D occupancy export
    <map_dir>/<name>/map_info.yaml              MapInfo metadata
    <map_dir>/<name>/places.yaml                named places (places.py)

Localization quality is a HEURISTIC (slam_toolbox exposes no metric):
a 2 Hz monitor tracks the map->odom correction; smooth drift is healthy,
jumps mean scan-matching is fighting. quality = exp(-jumps); is_lost =
no TF at all or quality below 0.2. Honest enough for watches and
wait_for_localization; revisit if a real metric appears.
"""

from __future__ import annotations

import asyncio
import logging
import math
import shutil
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from .server import RpcError

log = logging.getLogger("robocore_agent.slam")

# Map name for places saved before the first save()/load(): they belong
# to the session's scratch map and are migrated into the named directory
# on save().
SCRATCH_MAP = "_unsaved"

QUALITY_LOST_THRESHOLD = 0.2

# Correction jumps (meters per sample at 2 Hz) that count as "fighting".
_JUMP_METERS = 0.05
_JUMP_DECAY = 0.85  # per sample; ~3 s to forgive one jump


class SlamManager:
    """Mode state machine + map files for one robot."""

    def __init__(self, ros: Any, map_dir: str | None) -> None:
        self._ros = ros
        self.map_dir = Path(map_dir or "~/.robocore/maps").expanduser()
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self.mode = "mapping"          # mapping | localization | idle
        self.active_map = SCRATCH_MAP
        # quality monitor state (read by watches via ctx.watch_paths)
        self.quality: float = 0.0
        self._last_correction: tuple[float, float, float] | None = None
        self._jump_level = 1.0  # 1.0 = no recent jumps -> quality 1.0

    # -- mode switching ---------------------------------------------------------

    def start(self) -> str:
        """Ensure mapping mode. From idle: unpause. From localization:
        reset the graph and map fresh (the loaded map stays on disk).

        slam_toolbox's Pause service is a TOGGLE; the bridge is the only
        mode owner (doctrine), so we track paused-ness here rather than
        trusting the response shape.
        """
        if self.mode == "idle":
            self._ros.slam_pause_toggle()   # paused -> running
        elif self.mode == "localization":
            self._ros.slam_reset(pause=False)
            self.active_map = SCRATCH_MAP
        self.mode = "mapping"
        return self.mode

    def stop(self) -> str:
        """Pause map building (idle). The map and TF stay published."""
        if self.mode == "mapping":
            self._ros.slam_pause_toggle()   # running -> paused
            self.mode = "idle"
        return self.mode

    # -- save / load ---------------------------------------------------------------

    def save(self, name: str) -> dict[str, Any]:
        """Serialize pose graph + export the 2D grid + write metadata.
        Places saved on the scratch map migrate to the named map."""
        self._check_name(name)
        target = self.map_dir / name
        target.mkdir(parents=True, exist_ok=True)
        self._ros.slam_serialize(str(target / name))
        grid_info = self._export_grid(target)
        info = {
            "name": name,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "resolution": grid_info.get("resolution", 0.05),
            "dimensions": grid_info.get("dimensions", [0.0, 0.0]),
            "backend": "slam_toolbox",
            "is_3d": False,
        }
        (target / "map_info.yaml").write_text(yaml.safe_dump(info))
        scratch_places = self.map_dir / SCRATCH_MAP / "places.yaml"
        if self.active_map == SCRATCH_MAP and scratch_places.exists():
            shutil.move(str(scratch_places), str(target / "places.yaml"))
        self.active_map = name
        return info

    def load(self, name: str) -> None:
        """Load a saved map and continue MAPPING on top of it."""
        posegraph = self._posegraph_path(name)
        self._ros.slam_deserialize(str(posegraph), match_type=1)
        self.active_map = name
        self.mode = "mapping"

    def localize(self, pose: dict[str, Any] | None = None,
                 name: str | None = None) -> None:
        """Switch to localization mode against a saved map.

        ``name`` defaults to the active map; ``pose`` (wire Pose dict)
        defaults to the robot's current pose estimate — good enough when
        the robot hasn't been kidnapped since mapping.
        """
        name = name or self.active_map
        if name == SCRATCH_MAP:
            raise RpcError(
                "RobocoreError",
                "no saved map to localize against — slam.save() first "
                "(or slam.load() a saved one)",
            )
        posegraph = self._posegraph_path(name)
        x, y, theta = self._initial_pose(pose)
        self._ros.slam_deserialize(str(posegraph), match_type=3,
                                   x=x, y=y, theta=theta)
        self.active_map = name
        self.mode = "localization"

    def relocalize(self) -> None:
        """Re-run localization at the current pose estimate (the
        is_lost recovery path)."""
        self.localize()

    def _initial_pose(self, pose: dict[str, Any] | None) -> tuple:
        if pose is not None:
            yaw = _yaw_of(pose)
            return float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), yaw
        try:
            state = self._ros.get_state()
            current = state["pose"]
            return current["x"], current["y"], _yaw_of(current)
        except Exception:
            return 0.0, 0.0, 0.0  # no odom yet: best effort from origin

    # -- map registry ---------------------------------------------------------------

    def saved_maps(self) -> list[str]:
        return sorted(
            p.parent.name for p in self.map_dir.glob("*/*.posegraph"))

    def map_info(self, name: str) -> dict[str, Any]:
        path = self.map_dir / name / "map_info.yaml"
        if not path.exists():
            raise RpcError("RobocoreError", f"no saved map {name!r}")
        return yaml.safe_load(path.read_text())

    def delete(self, name: str) -> None:
        self._check_name(name)
        target = self.map_dir / name
        if not target.is_dir():
            raise RpcError("RobocoreError", f"no saved map {name!r}")
        if name == self.active_map:
            raise RpcError("RobocoreError",
                           f"map {name!r} is active; switch maps first")
        shutil.rmtree(target)

    def _posegraph_path(self, name: str) -> Path:
        self._check_name(name)
        path = self.map_dir / name / f"{name}.posegraph"
        if not path.exists():
            raise RpcError(
                "RobocoreError",
                f"no saved map {name!r} (have: {self.saved_maps()})",
            )
        return path.with_suffix("")  # slam_toolbox wants the stem

    @staticmethod
    def _check_name(name: str) -> None:
        if (not name or name.startswith((".", "_"))
                or any(c in name for c in "/\\")):
            raise RpcError("RobocoreError",
                           f"bad map name {name!r} (no paths, no leading "
                           "./_; '_unsaved' is reserved)")

    def _export_grid(self, target: Path) -> dict[str, Any]:
        """Export the 2D grid with map_saver_cli (per nav_setup.md the
        bridge shells out — no map_server node in the stack)."""
        try:
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", str(target / "grid"),
                 "--ros-args", "-p", "use_sim_time:=true"],
                capture_output=True, timeout=30.0, check=True,
            )
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            log.warning("map_saver_cli failed (%s); pose graph saved, "
                        "grid export skipped", exc)
            return {}
        try:
            meta = yaml.safe_load((target / "grid.yaml").read_text())
            resolution = float(meta.get("resolution", 0.05))
            grid = self._ros.map_grid(wait=1.0)
            return {
                "resolution": resolution,
                "dimensions": [grid["width"] * resolution,
                               grid["height"] * resolution],
            }
        except Exception:
            return {}

    # -- localization quality (heuristic) ---------------------------------------------

    def sample_quality(self) -> None:
        """One monitor tick: update self.quality from map->odom motion."""
        correction = self._ros.map_odom_correction()
        if correction is None:
            self.quality = 0.0
            self._last_correction = None
            return
        if self._last_correction is not None:
            dx = correction[0] - self._last_correction[0]
            dy = correction[1] - self._last_correction[1]
            if math.hypot(dx, dy) > _JUMP_METERS:
                self._jump_level *= 0.3   # a jump: confidence drops hard
            else:
                self._jump_level = min(
                    1.0, self._jump_level / _JUMP_DECAY)  # recover slowly
        self._last_correction = correction
        self.quality = max(0.0, min(1.0, self._jump_level))

    @property
    def is_lost(self) -> bool:
        return self.quality < QUALITY_LOST_THRESHOLD

    def state(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "backend": "slam_toolbox",
            "active_map": (None if self.active_map == SCRATCH_MAP
                           else self.active_map),
            "localization_quality": self.quality,
            "is_lost": self.is_lost,
            "saved_maps": self.saved_maps(),
        }


async def run_quality_monitor(slam: SlamManager,
                              interval: float = 0.5) -> None:
    """2 Hz background sampler updating slam.quality (feeds the
    slam.localization_quality watch path and wait_for_localization)."""
    while True:
        try:
            await asyncio.to_thread(slam.sample_quality)
        except Exception:
            log.exception("slam quality sample failed")
        await asyncio.sleep(interval)


def _yaw_of(pose: dict[str, Any]) -> float:
    q = pose.get("q") or {}
    x, y = float(q.get("x", 0.0)), float(q.get("y", 0.0))
    z, w = float(q.get("z", 0.0)), float(q.get("w", 1.0))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
