"""Robot profile loading and validation.

A profile is a YAML file mapping capabilities to a robot's ROS interfaces
(spec section 22). The schema is the shared pydantic model
robocore.models.profile.RobotProfile — single source of truth; this module
only adds file IO and agent-protocol checks.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml
from pydantic import ValidationError

from robocore.models import ProfileInfo, RobotProfile
from robocore.version import PROTOCOL_VERSION


class ProfileError(Exception):
    """The profile file is missing, malformed, or incompatible."""


@dataclass(frozen=True)
class Profile:
    """A loaded profile: validated spec + handshake metadata."""

    spec: RobotProfile
    info: ProfileInfo

    @property
    def instances(self) -> dict[str, tuple[str, ...]]:
        return self.spec.instances


def load_profile(path: str | Path) -> Profile:
    """Load and validate a profile YAML.

    Raises ProfileError if the file is unreadable, fails schema validation
    (including unknown top-level sections — typos must not silently disable
    a capability), or targets a protocol version this agent does not speak.
    """
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ProfileError(f"cannot load profile {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ProfileError(f"profile {path} is not a YAML mapping")

    try:
        spec = RobotProfile.model_validate(data)
    except ValidationError as exc:
        raise ProfileError(f"profile {path} is invalid:\n{exc}") from exc
    if spec.protocol != PROTOCOL_VERSION:
        raise ProfileError(
            f"profile {path} targets protocol {spec.protocol}, "
            f"agent speaks {PROTOCOL_VERSION}"
        )

    info = ProfileInfo(
        name=spec.name,
        version=spec.version,
        model=spec.model,
        capabilities=spec.capabilities,
    )
    return Profile(spec=spec, info=info)
