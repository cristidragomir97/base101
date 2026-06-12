"""Named places (spec section 17): poses with names, scoped to a map.

The kitchen pose belongs with the map, not in someone's script: places
persist as human-editable YAML alongside the map files
(<map_dir>/<map>/places.yaml). The visible set follows the active map;
asking for a place from another map raises UnknownPlace, never silent
wrong-coordinates.

Pure file I/O keyed by the SlamManager's active map; no ROS.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .server import RpcError


class PlaceStore:
    def __init__(self, slam: Any) -> None:
        self._slam = slam  # active_map + map_dir live there

    def save(self, name: str, pose: dict[str, Any]) -> None:
        if not name or any(c in name for c in "/\\"):
            raise RpcError("RobocoreError", f"bad place name {name!r}")
        places = self._read()
        places[name] = pose
        self._write(places)

    def get(self, name: str) -> dict[str, Any]:
        places = self._read()
        if name not in places:
            raise RpcError(
                "UnknownPlace",
                f"no place {name!r} on map "
                f"{self._slam.active_map!r} (have: {sorted(places)})",
            )
        return places[name]

    def list(self) -> list[str]:
        return sorted(self._read())

    def delete(self, name: str) -> None:
        places = self._read()
        if name not in places:
            raise RpcError(
                "UnknownPlace",
                f"no place {name!r} on map {self._slam.active_map!r}",
            )
        del places[name]
        self._write(places)

    # -- storage -----------------------------------------------------------------

    def _path(self) -> Path:
        return self._slam.map_dir / self._slam.active_map / "places.yaml"

    def _read(self) -> dict[str, dict[str, Any]]:
        path = self._path()
        if not path.exists():
            return {}
        data = yaml.safe_load(path.read_text()) or {}
        return data if isinstance(data, dict) else {}

    def _write(self, places: dict[str, dict[str, Any]]) -> None:
        path = self._path()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(places, sort_keys=True))
