"""Tracking-aware phase governor independent of ROS and transport code."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GovernorDecision:
    phase_rate: float
    hard_frozen: bool
    frozen_reason: str | None


class TrackingGovernor:
    def __init__(self, *, run_error_rad: float, resume_error_rad: float, stop_error_rad: float) -> None:
        if not 0 < run_error_rad < resume_error_rad < stop_error_rad:
            raise ValueError("tracking thresholds must satisfy 0 < run < resume < stop")
        self.run_error_rad = float(run_error_rad)
        self.resume_error_rad = float(resume_error_rad)
        self.stop_error_rad = float(stop_error_rad)
        self._hard_frozen = False
        self._reason: str | None = None

    def reset(self) -> None:
        self._hard_frozen = False
        self._reason = None

    def update(
        self,
        tracking_error_rad: float,
        *,
        state_stale: bool = False,
        timer_overrun: bool = False,
        arm_clipped: bool = False,
    ) -> GovernorDecision:
        error = float(tracking_error_rad)
        if not math.isfinite(error) or error < 0:
            state_stale = True

        fault_reason = None
        if state_stale:
            fault_reason = "joint state stale"
        elif timer_overrun:
            fault_reason = "trajectory timer overrun"
        elif arm_clipped:
            fault_reason = "arm safety clipping"
        elif error >= self.stop_error_rad:
            fault_reason = "tracking error hard stop"

        if fault_reason is not None:
            self._hard_frozen = True
            self._reason = fault_reason
        elif self._hard_frozen:
            if error <= self.resume_error_rad:
                self._hard_frozen = False
                self._reason = None
            else:
                return GovernorDecision(0.0, True, self._reason)

        if self._hard_frozen:
            return GovernorDecision(0.0, True, self._reason)
        if error <= self.run_error_rad:
            return GovernorDecision(1.0, False, None)
        rate = (self.stop_error_rad - error) / (self.stop_error_rad - self.run_error_rad)
        return GovernorDecision(max(0.0, min(1.0, rate)), False, None)
