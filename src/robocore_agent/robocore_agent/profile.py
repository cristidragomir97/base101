"""Robot profile loading (Phase 1 stub).

A profile is a YAML file mapping capabilities to a robot's ROS interfaces
(spec section 22). Phase 1 only needs enough to produce a capability
report: the identity fields plus which capability sections are present.
Full schema validation and per-capability config land in Phase 2.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from robocore.models import ProfileInfo
from robocore.version import PROTOCOL_VERSION

# Capability registry, spec section 4. A profile section with one of these
# names declares that capability. Order here is the report order.
KNOWN_CAPABILITIES = (
    "mobility",
    "manipulation",
    "joint_groups",
    "cameras",
    "lidar",
    "slam",
    "exploration",
    "gps",
    "teleop",
    "control",
    "watches",
    "places",
    "audit",
    "status",
)

REQUIRED_KEYS = ("name", "model", "protocol", "frames")


class ProfileError(Exception):
    """The profile file is missing, malformed, or incompatible."""


@dataclass(frozen=True)
class Profile:
    """A loaded profile: handshake metadata plus the raw section dicts."""

    info: ProfileInfo
    frames: dict[str, str]
    raw: dict[str, Any]


def load_profile(path: str | Path) -> Profile:
    """Load and minimally validate a profile YAML.

    Raises ProfileError if the file is unreadable, lacks a required key
    (name, model, protocol, frames), or targets a different protocol
    version than this agent speaks.
    """
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ProfileError(f"cannot load profile {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ProfileError(f"profile {path} is not a YAML mapping")

    missing = [key for key in REQUIRED_KEYS if key not in data]
    if missing:
        raise ProfileError(f"profile {path} missing keys: {missing}")
    if data["protocol"] != PROTOCOL_VERSION:
        raise ProfileError(
            f"profile {path} targets protocol {data['protocol']}, "
            f"agent speaks {PROTOCOL_VERSION}"
        )

    capabilities = tuple(
        name for name in KNOWN_CAPABILITIES if data.get(name) is not None
    )
    info = ProfileInfo(
        name=str(data["name"]),
        version=str(data.get("version", "0")),
        model=str(data["model"]),
        capabilities=capabilities,
    )
    return Profile(info=info, frames=dict(data["frames"]), raw=data)
