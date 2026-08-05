"""Deterministic low-speed motion profiles used by hardware diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
import math


def minimum_jerk_blend(start: float, end: float, phase: float) -> float:
    phase = max(0.0, min(1.0, float(phase)))
    scale = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    return start + (end - start) * scale


@dataclass(frozen=True)
class MinimumJerkSweep:
    """Zero-velocity/acceleration sweep: 0 -> +A -> -A -> 0."""

    amplitude_rad: float
    segment_seconds: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.amplitude_rad) or self.amplitude_rad <= 0:
            raise ValueError("amplitude_rad must be finite and positive")
        if not math.isfinite(self.segment_seconds) or self.segment_seconds <= 0:
            raise ValueError("segment_seconds must be finite and positive")

    @property
    def duration(self) -> float:
        return 4.0 * self.segment_seconds

    @property
    def max_velocity(self) -> float:
        return 1.875 * self.amplitude_rad / self.segment_seconds

    @property
    def max_acceleration(self) -> float:
        return (10.0 / math.sqrt(3.0)) * self.amplitude_rad / self.segment_seconds**2

    def offset(self, elapsed_seconds: float) -> float:
        elapsed = max(0.0, min(self.duration, float(elapsed_seconds)))
        segment = self.segment_seconds
        if elapsed <= segment:
            return minimum_jerk_blend(0.0, self.amplitude_rad, elapsed / segment)
        if elapsed <= 3.0 * segment:
            return minimum_jerk_blend(
                self.amplitude_rad,
                -self.amplitude_rad,
                (elapsed - segment) / (2.0 * segment),
            )
        return minimum_jerk_blend(
            -self.amplitude_rad,
            0.0,
            (elapsed - 3.0 * segment) / segment,
        )


@dataclass(frozen=True)
class FrozenLinearPlan:
    """Piecewise-linear playback of immutable, equally spaced vector knots."""

    knots: tuple[tuple[float, ...], ...]
    knot_hz: float

    def __post_init__(self) -> None:
        if len(self.knots) < 2:
            raise ValueError("a frozen plan requires at least two knots")
        width = len(self.knots[0])
        if width == 0 or any(len(knot) != width for knot in self.knots):
            raise ValueError("all frozen plan knots must have the same nonzero width")
        if not all(math.isfinite(value) for knot in self.knots for value in knot):
            raise ValueError("frozen plan knots must be finite")
        if not math.isfinite(self.knot_hz) or self.knot_hz <= 0:
            raise ValueError("knot_hz must be finite and positive")

    @property
    def duration(self) -> float:
        return (len(self.knots) - 1) / self.knot_hz

    def value(self, elapsed_seconds: float) -> tuple[float, ...]:
        scaled = max(0.0, float(elapsed_seconds)) * self.knot_hz
        left_index = min(int(math.floor(scaled)), len(self.knots) - 1)
        if left_index == len(self.knots) - 1:
            return self.knots[-1]
        phase = scaled - left_index
        left = self.knots[left_index]
        right = self.knots[left_index + 1]
        return tuple(a + (b - a) * phase for a, b in zip(left, right))
