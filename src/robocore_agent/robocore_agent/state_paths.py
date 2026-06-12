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
    "battery.{level,voltage,current} | pose.{x,y} | "
    "velocity.linear.{x,y} | velocity.angular.z | distance_traveled | "
    "arms.<arm>.effort.<joint> | arms.<arm>.joint_positions.<joint> | "
    "arms.<arm>.wrench.{force,torque}.{x,y,z} | range.<name>.range | "
    "imu.angular_velocity.{x,y,z} | imu.linear_acceleration.{x,y,z} | "
    "environment.{temperature,pressure,illuminance}"
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
        for field in ("level", "voltage", "current"):
            table[f"battery.{field}"] = _dict_field(
                ros.battery_state, (field,))

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

    for name in (spec.range_sensors or {}):
        table[f"range.{name}.range"] = _dict_field(
            lambda n=name: ros.range_reading(n), ("range",))

    if spec.imu is not None:
        for vector in ("angular_velocity", "linear_acceleration"):
            for axis in "xyz":
                table[f"imu.{vector}.{axis}"] = _dict_field(
                    ros.imu_reading, (vector, axis))

    for arm_name in (spec.force_torque or {}):
        for vector in ("force", "torque"):
            for axis in "xyz":
                table[f"arms.{arm_name}.wrench.{vector}.{axis}"] = (
                    _dict_field(lambda n=arm_name: ros.wrench_reading(n),
                                (vector, axis)))

    if spec.environment is not None:
        for kind in ("temperature", "pressure", "illuminance"):
            if getattr(spec.environment, kind):
                table[f"environment.{kind}"] = _dict_field(
                    lambda k=kind: ros.environment_reading(k), (kind,))
    return table


def _dict_field(reader: Callable[[], dict | None],
                path: tuple[str, ...]) -> Callable[[], float | None]:
    """Getter digging ``path`` out of a wire-dict reader; None-safe."""
    def get() -> float | None:
        value: Any = reader()
        for key in path:
            if value is None:
                return None
            value = value.get(key)
        return None if value is None else float(value)
    return get


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
