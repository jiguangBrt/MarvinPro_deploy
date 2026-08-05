"""Safety checks shared by the rollout client and the robot bridge."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .config import JOINT_LOWER, JOINT_UPPER

ARM_ACTION_INDICES = tuple(range(7)) + tuple(range(8, 15))


class SafetyError(ValueError):
    pass


@dataclass(frozen=True)
class FilteredAction:
    action: tuple[float, ...]
    clipped_indices: tuple[int, ...]


def _finite_vector(values, length: int, label: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise SafetyError(f"{label} is not numeric: {exc}") from exc
    if len(result) != length:
        raise SafetyError(f"{label} must have {length} values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise SafetyError(f"{label} contains NaN or Inf")
    return result


def action_arms(action: tuple[float, ...]) -> tuple[float, ...]:
    return action[:7] + action[8:15]


def validate_action(
    action,
    current_joints,
    *,
    max_joint_step_rad: float,
    joint_limit_margin_rad: float = 0.02,
) -> tuple[float, ...]:
    values = _finite_vector(action, 16, "action")
    joints = _finite_vector(current_joints, 14, "current_joints")
    if max_joint_step_rad <= 0:
        raise SafetyError("max_joint_step_rad must be positive")
    if not (0.0 <= values[7] <= 1.0 and 0.0 <= values[15] <= 1.0):
        raise SafetyError(f"gripper targets must be in [0, 1], got {values[7]:.4f}, {values[15]:.4f}")

    targets = action_arms(values)
    for index, (target, current, lower, upper) in enumerate(zip(targets, joints, JOINT_LOWER, JOINT_UPPER)):
        safe_lower = lower + joint_limit_margin_rad
        safe_upper = upper - joint_limit_margin_rad
        if not safe_lower <= target <= safe_upper:
            raise SafetyError(
                f"joint {index} target {target:.5f} outside [{safe_lower:.5f}, {safe_upper:.5f}]"
            )
        delta = abs(target - current)
        if delta > max_joint_step_rad + 1e-9:
            raise SafetyError(
                f"joint {index} step {delta:.5f}rad exceeds {max_joint_step_rad:.5f}rad"
            )
    return values


def filter_action(
    action,
    current_joints,
    *,
    max_joint_step_rad: float,
    joint_limit_margin_rad: float = 0.02,
) -> FilteredAction:
    """Clamp policy output to hard limits and a per-command envelope around feedback."""
    values = list(_finite_vector(action, 16, "action"))
    joints = _finite_vector(current_joints, 14, "current_joints")
    if max_joint_step_rad <= 0:
        raise SafetyError("max_joint_step_rad must be positive")

    clipped: list[int] = []
    for joint_index, action_index in enumerate(ARM_ACTION_INDICES):
        lower = max(JOINT_LOWER[joint_index] + joint_limit_margin_rad, joints[joint_index] - max_joint_step_rad)
        upper = min(JOINT_UPPER[joint_index] - joint_limit_margin_rad, joints[joint_index] + max_joint_step_rad)
        if lower > upper:
            raise SafetyError(f"current joint {joint_index} is outside the configured safe range")
        original = values[action_index]
        values[action_index] = max(lower, min(upper, original))
        if values[action_index] != original:
            clipped.append(action_index)

    for action_index in (7, 15):
        original = values[action_index]
        values[action_index] = max(0.0, min(1.0, original))
        if values[action_index] != original:
            clipped.append(action_index)
    return FilteredAction(tuple(values), tuple(clipped))
