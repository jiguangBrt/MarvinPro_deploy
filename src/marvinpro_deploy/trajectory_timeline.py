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


def _finite_vector(values, *, width: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != width:
        raise ValueError(f"{label} must have {width} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must be finite")
    return result


@dataclass(frozen=True)
class QuinticBlend:
    """A local q/v/a-continuous trajectory segment parameterized by knot phase."""

    start_phase: float
    end_phase: float
    coefficients: tuple[tuple[float, float, float, float, float, float], ...]

    @classmethod
    def create(
        cls,
        *,
        start_phase: float,
        end_phase: float,
        start_position,
        start_velocity,
        start_acceleration,
        end_position,
        end_velocity,
        end_acceleration,
    ) -> "QuinticBlend":
        q0 = _finite_vector(start_position, width=16, label="blend start position")
        q1 = _finite_vector(end_position, width=16, label="blend end position")
        v0 = _finite_vector(start_velocity, width=16, label="blend start velocity")
        v1 = _finite_vector(end_velocity, width=16, label="blend end velocity")
        a0 = _finite_vector(start_acceleration, width=16, label="blend start acceleration")
        a1 = _finite_vector(end_acceleration, width=16, label="blend end acceleration")
        duration = float(end_phase) - float(start_phase)
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("blend phase duration must be finite and positive")

        coefficients = []
        for start, end, start_v, end_v, start_a, end_a in zip(q0, q1, v0, v1, a0, a1):
            delta = end - start
            m0 = start_v * duration
            m1 = end_v * duration
            n0 = start_a * duration**2
            n1 = end_a * duration**2
            coefficients.append(
                (
                    start,
                    m0,
                    0.5 * n0,
                    10.0 * delta - 6.0 * m0 - 4.0 * m1 - 1.5 * n0 + 0.5 * n1,
                    -15.0 * delta + 8.0 * m0 + 7.0 * m1 + 1.5 * n0 - n1,
                    6.0 * delta - 3.0 * m0 - 3.0 * m1 - 0.5 * n0 + 0.5 * n1,
                )
            )
        return cls(float(start_phase), float(end_phase), tuple(coefficients))

    @property
    def duration_phases(self) -> float:
        return self.end_phase - self.start_phase

    def kinematics(
        self, phase: float
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        duration = self.duration_phases
        u = max(0.0, min(1.0, (float(phase) - self.start_phase) / duration))
        positions = []
        velocities = []
        accelerations = []
        jerks = []
        for c0, c1, c2, c3, c4, c5 in self.coefficients:
            positions.append(c0 + c1 * u + c2 * u**2 + c3 * u**3 + c4 * u**4 + c5 * u**5)
            velocities.append((c1 + 2.0 * c2 * u + 3.0 * c3 * u**2 + 4.0 * c4 * u**3 + 5.0 * c5 * u**4) / duration)
            accelerations.append((2.0 * c2 + 6.0 * c3 * u + 12.0 * c4 * u**2 + 20.0 * c5 * u**3) / duration**2)
            jerks.append((6.0 * c3 + 24.0 * c4 * u + 60.0 * c5 * u**2) / duration**3)
        return tuple(positions), tuple(velocities), tuple(accelerations), tuple(jerks)


@dataclass(frozen=True)
class TrajectoryTimeline:
    knots: tuple[tuple[float, ...], ...]
    knot_hz: float
    checkpoint_horizon: int
    blend: QuinticBlend | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "knots", _finite_knots(self.knots))
        if not math.isfinite(self.knot_hz) or self.knot_hz <= 0:
            raise ValueError("knot_hz must be finite and positive")
        if not 1 <= self.checkpoint_horizon <= len(self.knots):
            raise ValueError("checkpoint_horizon must be within the trajectory horizon")
        if self.blend is not None:
            if self.blend.start_phase < 0 or self.blend.end_phase > self.final_phase:
                raise ValueError("blend must be within the trajectory horizon")
            if self.blend.end_phase > self.checkpoint_phase + 1e-9:
                raise ValueError("blend must finish before the next checkpoint")

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
        if self.blend is not None and self.blend.start_phase <= phase <= self.blend.end_phase:
            return self.blend.kinematics(phase)[0]
        left_index = int(math.floor(phase))
        if left_index >= self.horizon - 1:
            return self.knots[-1]
        fraction = phase - left_index
        left = self.knots[left_index]
        right = self.knots[left_index + 1]
        return tuple(a + (b - a) * fraction for a, b in zip(left, right))

    def remaining_after_checkpoint(self) -> tuple[tuple[float, ...], ...]:
        return self.knots[self.checkpoint_horizon :]

    def phase_kinematics(
        self, phase: float, *, side: str = "right"
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        phase = max(0.0, min(float(phase), self.final_phase))
        if self.blend is not None and self.blend.start_phase <= phase <= self.blend.end_phase:
            return self.blend.kinematics(phase)
        if side not in ("left", "right"):
            raise ValueError("kinematics side must be left or right")
        rounded = round(phase)
        if abs(phase - rounded) <= 1e-9:
            index = int(rounded)
            if side == "left" and index > 0:
                left_index = index - 1
            else:
                left_index = min(index, self.horizon - 2)
        else:
            left_index = min(int(math.floor(phase)), self.horizon - 2)
        velocity = tuple(
            right - left for left, right in zip(self.knots[left_index], self.knots[left_index + 1])
        )
        zeros = (0.0,) * 16
        return self.value(phase), velocity, zeros, zeros

    def replacement(
        self,
        actions,
        *,
        actual_delay_steps: int,
        anchor,
        blend_knots: int | None = None,
        start_velocity=None,
        start_acceleration=None,
    ) -> tuple["TrajectoryTimeline", float]:
        replacement = list(_finite_knots(actions))
        if len(replacement) != self.horizon:
            raise ValueError("replacement horizon does not match current trajectory")
        if not 1 <= actual_delay_steps <= self.checkpoint_horizon:
            raise ValueError("actual delay must be between 1 and the execution horizon")
        phase = actual_delay_steps - 1
        anchor = _finite_vector(anchor, width=16, label="replacement anchor")
        replacement[phase] = anchor
        timeline = TrajectoryTimeline(tuple(replacement), self.knot_hz, self.checkpoint_horizon)
        if blend_knots is not None:
            if blend_knots not in (2, 3):
                raise ValueError("RTC blend must span 2 or 3 knots")
            end_phase = float(phase + blend_knots)
            if end_phase > timeline.checkpoint_phase or end_phase >= timeline.final_phase:
                raise ValueError("RTC blend does not fit before the next checkpoint")
            if start_velocity is None or start_acceleration is None:
                raise ValueError("RTC blend requires start velocity and acceleration")
            end_position, end_velocity, end_acceleration, _ = timeline.phase_kinematics(
                end_phase, side="right"
            )
            blend = QuinticBlend.create(
                start_phase=float(phase),
                end_phase=end_phase,
                start_position=anchor,
                start_velocity=start_velocity,
                start_acceleration=start_acceleration,
                end_position=end_position,
                end_velocity=end_velocity,
                end_acceleration=end_acceleration,
            )
            timeline = TrajectoryTimeline(
                tuple(replacement), self.knot_hz, self.checkpoint_horizon, blend=blend
            )
        return (
            timeline,
            float(phase),
        )

    def with_c2_handoff(self, anchor, *, blend_knots: int) -> "TrajectoryTimeline":
        """Start this timeline from a stationary measured anchor with a C2 blend."""
        if blend_knots not in (2, 3):
            raise ValueError("trajectory handoff blend must span 2 or 3 knots")
        anchor = _finite_vector(anchor, width=16, label="trajectory handoff anchor")
        knots = list(self.knots)
        knots[0] = anchor
        timeline = TrajectoryTimeline(tuple(knots), self.knot_hz, self.checkpoint_horizon)
        end_phase = float(blend_knots)
        if end_phase > timeline.checkpoint_phase or end_phase >= timeline.final_phase:
            raise ValueError("trajectory handoff blend does not fit before the checkpoint")
        end_position, end_velocity, end_acceleration, _ = timeline.phase_kinematics(
            end_phase, side="right"
        )
        zeros = (0.0,) * 16
        blend = QuinticBlend.create(
            start_phase=0.0,
            end_phase=end_phase,
            start_position=anchor,
            start_velocity=zeros,
            start_acceleration=zeros,
            end_position=end_position,
            end_velocity=end_velocity,
            end_acceleration=end_acceleration,
        )
        return TrajectoryTimeline(
            tuple(knots), self.knot_hz, self.checkpoint_horizon, blend=blend
        )
