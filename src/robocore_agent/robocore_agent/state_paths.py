"""Watchable dotted paths -> live values (spec section 16).

The grammar is fixed in v1 (numeric paths over what the agent already
observes); profile-declared extra paths are an OPEN-Q for Cristi. A path
is validated once at watch.start (unknown paths fail fast, listing the
grammar) and resolved on every sampler tick (None = no data yet).
"""

from __future__ import annotations

from typing import Any, Callable

from .server import RpcError

GRAMMAR = (
    "battery.level | pose.{x,y} | velocity.linear.{x,y} | "
    "velocity.angular.z | distance_traveled | "
    "arms.<arm>.effort.<joint> | arms.<arm>.joint_positions.<joint>"
)


def build_path_table(ctx: Any) -> dict[str, Callable[[], float | None]]:
    """All watchable paths for this profile, each mapped to a getter.

    Getters return None while the underlying data has not arrived;
    they must never raise (the sampler runs them at rate).
    """
    ros = ctx.ros
    spec = ctx.profile.spec
    table: dict[str, Callable[[], float | None]] = {}

    def state_field(picker: Callable[[dict], float]) -> Callable[[], float | None]:
        def get() -> float | None:
            try:
                return float(picker(ros.get_state()))
            except Exception:
                return None  # no odometry / TF yet
        return get

    if spec.mobility is not None:
        table["pose.x"] = state_field(lambda s: s["pose"]["x"])
        table["pose.y"] = state_field(lambda s: s["pose"]["y"])
        table["velocity.linear.x"] = state_field(
            lambda s: s["velocity"]["linear"]["x"])
        table["velocity.linear.y"] = state_field(
            lambda s: s["velocity"]["linear"]["y"])
        table["velocity.angular.z"] = state_field(
            lambda s: s["velocity"]["angular"]["z"])
        table["distance_traveled"] = state_field(
            lambda s: s["distance_traveled"])

    if spec.status is not None and spec.status.battery:
        def battery() -> float | None:
            return ros.battery_state()[0]
        table["battery.level"] = battery

    if spec.manipulation is not None:
        for arm_name, arm in spec.manipulation.arms.items():
            joints = list(arm.joints)
            if arm.gripper is not None and arm.gripper.joint:
                joints.append(arm.gripper.joint)
            for joint in joints:
                table[f"arms.{arm_name}.effort.{joint}"] = (
                    _joint_field(ros, joint, 2))
                table[f"arms.{arm_name}.joint_positions.{joint}"] = (
                    _joint_field(ros, joint, 0))
    return table


def _joint_field(ros: Any, joint: str, index: int) -> Callable[[], float | None]:
    def get() -> float | None:
        sample = ros.joint_snapshot().get(joint)
        return None if sample is None else float(sample[index])
    return get


def validate_path(table: dict[str, Callable], path: str) -> None:
    if path not in table:
        raise RpcError(
            "RobocoreError",
            f"path {path!r} is not watchable on this robot. "
            f"Grammar: {GRAMMAR}. "
            "battery.level needs status.battery in the profile; arm paths "
            "need manipulation.arms.<name>.joints.",
        )
