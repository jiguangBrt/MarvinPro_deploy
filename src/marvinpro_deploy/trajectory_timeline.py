"""Versioned model-knot timelines for tracking-aware execution."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _finite_knots(knots) -> tuple[tuple[float, ...], ...]:
    values = tuple(tuple(float(value) for value in knot) for knot in knots)
    if not values:
        raise ValueError("trajectory requires at least one knot")
    width = len(values[0])
    if width != 16 or any(len(knot) != width for knot in values):
        raise ValueError("trajectory knots must have shape [horizon, 16]")
    if not all(math.isfinite(value) for knot in values for value in knot):
        raise ValueError("trajectory knots must be finite")
    return values


@dataclass(frozen=True)
class TrajectoryTimeline:
    knots: tuple[tuple[float, ...], ...]
    knot_hz: float
    checkpoint_horizon: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "knots", _finite_knots(self.knots))
        if not math.isfinite(self.knot_hz) or self.knot_hz <= 0:
            raise ValueError("knot_hz must be finite and positive")
        if not 1 <= self.checkpoint_horizon <= len(self.knots):
            raise ValueError("checkpoint_horizon must be within the trajectory horizon")

    @property
    def horizon(self) -> int:
        return len(self.knots)

    @property
    def checkpoint_phase(self) -> float:
        return float(self.checkpoint_horizon - 1)

    @property
    def final_phase(self) -> float:
        return float(self.horizon - 1)

    def value(self, phase: float) -> tuple[float, ...]:
        phase = max(0.0, min(float(phase), self.final_phase))
        left_index = int(math.floor(phase))
        if left_index >= self.horizon - 1:
            return self.knots[-1]
        fraction = phase - left_index
        left = self.knots[left_index]
        right = self.knots[left_index + 1]
        return tuple(a + (b - a) * fraction for a, b in zip(left, right))

    def remaining_after_checkpoint(self) -> tuple[tuple[float, ...], ...]:
        return self.knots[self.checkpoint_horizon :]

    def replacement(self, actions, *, actual_delay_steps: int, anchor) -> tuple["TrajectoryTimeline", float]:
        replacement = list(_finite_knots(actions))
        if len(replacement) != self.horizon:
            raise ValueError("replacement horizon does not match current trajectory")
        if not 1 <= actual_delay_steps <= self.checkpoint_horizon:
            raise ValueError("actual delay must be between 1 and the execution horizon")
        phase = actual_delay_steps - 1
        replacement[phase] = tuple(float(value) for value in anchor)
        return (
            TrajectoryTimeline(tuple(replacement), self.knot_hz, self.checkpoint_horizon),
            float(phase),
        )
