"""Server-side frame handles, deprojection and cloud math (spec section 10).

A ``FrameRecord`` is what makes deprojection honest: it snapshots the
depth image, intrinsics and TF at capture time, keyed by the frame id the
client received. ``deproject`` then always pairs a pixel with the depth
and transforms of THAT frame — the "detected in an old frame, deprojected
against new depth" bug class dies by construction. Records expire after
the profile's ``frame_ttl`` (default 30 s); using a stale id raises
FrameExpired.

Pure numpy + the shared spatial models; no rclpy, unit-testable anywhere.
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field

import numpy as np

from robocore.models import CameraIntrinsics, Transform

from .server import RpcError

# Depth bytes a FrameCache may pin before evicting old records (VGA
# float32 depth is ~1.2 MiB; 128 MiB pins ~100 depth-bearing handles).
DEPTH_BUDGET = 128 * 1024 * 1024
MAX_RECORDS = 256

# Fixed rotation taking ROS optical-frame coordinates (z forward, x
# right, y down) to the physical camera link (x forward, REP 103).
_OPTICAL_TO_LINK = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


@dataclass
class FrameRecord:
    """Everything captured with one frame that deprojection needs."""

    camera: str
    stamp: float
    tf_frame: str                       # TF frame of the camera data
    optical: bool                       # tf_frame follows optical convention
    intrinsics: CameraIntrinsics | None
    depth: np.ndarray | None            # (H, W) float32 meters
    rgb: np.ndarray | None              # (H, W, 3) uint8, for cloud colors
    # target frame name -> Transform(child=tf_frame, parent=target),
    # captured at frame time.
    transforms: dict[str, Transform] = field(default_factory=dict)
    deadline: float = 0.0
    id: str = ""


class FrameCache:
    """TTL'd registry of handed-out frame records."""

    _ids = itertools.count(1)

    def __init__(self, depth_budget: int = DEPTH_BUDGET,
                 max_records: int = MAX_RECORDS) -> None:
        self._records: dict[str, FrameRecord] = {}
        self._depth_budget = depth_budget
        self._max_records = max_records

    def store(self, record: FrameRecord, ttl: float) -> str:
        record.id = f"f{next(self._ids)}"
        record.deadline = time.monotonic() + ttl
        self._records[record.id] = record
        self._evict()
        return record.id

    def get(self, frame_id: str) -> FrameRecord:
        """Raises RpcError(FrameExpired) for unknown/expired ids."""
        self._evict()
        record = self._records.get(frame_id)
        if record is None:
            raise RpcError(
                "FrameExpired",
                f"frame {frame_id!r} is unknown or expired (handles live "
                "for the profile's frame_ttl after capture)",
            )
        return record

    def _evict(self) -> None:
        now = time.monotonic()
        for fid, rec in list(self._records.items()):
            if rec.deadline <= now:
                del self._records[fid]
        while len(self._records) > self._max_records:
            del self._records[next(iter(self._records))]
        while self._depth_bytes() > self._depth_budget and self._records:
            del self._records[next(iter(self._records))]

    def _depth_bytes(self) -> int:
        return sum(r.depth.nbytes for r in self._records.values()
                   if r.depth is not None)


def deproject(record: FrameRecord, pixel: tuple[float, float],
              in_frame: str | None) -> dict:
    """Pixel -> 3D point using the record's own depth/intrinsics/TF.

    Returns a wire-shaped Point dict. ``in_frame`` None means the
    camera's TF frame; otherwise it must be one of the frames snapshotted
    at capture (map/odom/base, resolved by the caller).
    """
    if record.depth is None:
        raise RpcError(
            "RobocoreError",
            f"camera {record.camera!r} has no depth stream; deprojection "
            "needs a depth-capable camera (profile cameras.<name>.depth)",
        )
    if record.intrinsics is None:
        raise RpcError(
            "RobocoreError",
            f"camera {record.camera!r} has no intrinsics (no camera_info "
            "topic in the profile); cannot deproject",
        )
    u, v = float(pixel[0]), float(pixel[1])
    height, width = record.depth.shape
    if not (0 <= u < width and 0 <= v < height):
        raise RpcError(
            "RobocoreError",
            f"pixel ({u:g}, {v:g}) outside the {width}x{height} image",
        )
    d = float(record.depth[int(v), int(u)])
    if not math.isfinite(d) or d <= 0.0:
        raise RpcError(
            "RobocoreError",
            f"no depth return at pixel ({u:g}, {v:g})",
        )
    k = record.intrinsics
    optical = np.array([(u - k.cx) * d / k.fx, (v - k.cy) * d / k.fy, d])
    point = optical if record.optical else _OPTICAL_TO_LINK @ optical

    target = in_frame or record.tf_frame
    if target != record.tf_frame:
        transform = record.transforms.get(target)
        if transform is None:
            known = sorted(record.transforms)
            raise RpcError(
                "RobocoreError",
                f"frame {target!r} was not snapshotted with this frame "
                f"(available: {record.tf_frame!r} + {known}); deprojection "
                "uses capture-time TF only",
            )
        x, y, z = transform.rotation.rotate(*point)
        point = np.array([x + transform.translation.x,
                          y + transform.translation.y,
                          z + transform.translation.z])
    return {"x": float(point[0]), "y": float(point[1]),
            "z": float(point[2]), "frame": target}


def make_cloud(depth: np.ndarray, rgb: np.ndarray | None,
               intrinsics: CameraIntrinsics, voxel: float,
               optical: bool) -> tuple[np.ndarray, np.ndarray | None]:
    """Full-image deprojection + voxel downsample.

    The mandatory ``voxel`` parameter is the API's size budget (ruling:
    keeps get_cloud honest before the TCP transport exists). Returns
    (points (N,3) float32, colors (N,3) uint8 | None) in the camera's TF
    frame convention (optical axes when ``optical``, REP-103 link axes
    otherwise). One representative point per voxel (first hit).
    """
    if voxel <= 0.0:
        raise RpcError("RobocoreError", "voxel must be > 0")
    height, width = depth.shape
    vs, us = np.mgrid[0:height, 0:width]
    d = depth.ravel()
    valid = np.isfinite(d) & (d > 0.0)
    d = d[valid]
    u = us.ravel()[valid].astype(np.float64)
    v = vs.ravel()[valid].astype(np.float64)
    points = np.column_stack([
        (u - intrinsics.cx) * d / intrinsics.fx,
        (v - intrinsics.cy) * d / intrinsics.fy,
        d,
    ])
    if not optical:
        points = points @ _OPTICAL_TO_LINK.T

    # One point per voxel: hash quantized coordinates, keep first index.
    quantized = np.floor(points / voxel).astype(np.int64) + (1 << 20)
    keys = (quantized[:, 0] << 42) | (quantized[:, 1] << 21) | quantized[:, 2]
    _unique, first = np.unique(keys, return_index=True)
    points = points[first].astype(np.float32)

    colors = None
    if rgb is not None and rgb.shape[:2] == depth.shape:
        colors = rgb.reshape(-1, 3)[valid][first].astype(np.uint8)
    return points, colors
