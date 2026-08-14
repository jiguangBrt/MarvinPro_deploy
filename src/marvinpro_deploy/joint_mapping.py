"""Pure-Python Marvin Pro state layout and gripper command handling."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .config import GRIPPER_CLOSED_RAW, GRIPPER_OPEN_RAW, JOINT_NAMES


class JointMapError(ValueError):
    pass


@dataclass(frozen=True)
class JointMap:
    indices: tuple[int, ...]

    @classmethod
    def from_names(cls, names: list[str]) -> "JointMap":
        if len(names) != len(set(names)):
            raise JointMapError(f"duplicate names in /joint_states: {names}")
        index = {name: i for i, name in enumerate(names)}
        missing = [name for name in JOINT_NAMES if name not in index]
        if missing:
            raise JointMapError(f"missing joints {missing}; received {names}")
        return cls(tuple(index[name] for name in JOINT_NAMES))

    def canonical_positions(self, positions: list[float]) -> tuple[float, ...]:
        try:
            result = tuple(float(positions[i]) for i in self.indices)
        except (IndexError, TypeError, ValueError) as exc:
            raise JointMapError(f"invalid JointState.position: {exc}") from exc
        if len(result) != 14 or not all(math.isfinite(value) for value in result):
            raise JointMapError("joint positions must contain 14 finite values")
        return result


def normalize_gripper(
    raw: float,
    open_raw: float = GRIPPER_OPEN_RAW,
    closed_raw: float = GRIPPER_CLOSED_RAW,
) -> float:
    raw = float(raw)
    span = float(closed_raw) - float(open_raw)
    if not math.isfinite(raw) or not math.isfinite(span) or span <= 0:
        raise ValueError("gripper feedback/calibration must be finite with closed_raw > open_raw")
    return max(0.0, min(1.0, (raw - open_raw) / span))


def gripper_command_proxy(value: float) -> float:
    """Validate and clamp a 0=open, 1=closed command used as state proxy."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("gripper command proxy must be finite")
    return max(0.0, min(1.0, value))


def build_state16(
    joints: tuple[float, ...],
    gripper_proxy_left: float,
    gripper_proxy_right: float,
) -> tuple[float, ...]:
    if len(joints) != 14 or not all(math.isfinite(value) for value in joints):
        raise ValueError("canonical joint state must have 14 finite values")
    return (
        *joints[:7],
        gripper_command_proxy(gripper_proxy_left),
        *joints[7:],
        gripper_command_proxy(gripper_proxy_right),
    )
