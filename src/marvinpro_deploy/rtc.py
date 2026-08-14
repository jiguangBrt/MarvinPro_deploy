"""Client-side RTC request validation and conservative delay estimation."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time

import numpy as np


RTC_REQUEST_TYPE = "rtc_v1"
RTC_HORIZON = 10
RTC_EXECUTION_HORIZON = 6
RTC_MAX_DELAY = 4


class RtcError(RuntimeError):
    pass


@dataclass(frozen=True)
class DelaySampleDecision:
    latency_seconds: float
    accepted: bool
    reason: str | None
    epoch: int
    stable_samples: int


class DelayEstimator:
    def __init__(
        self,
        *,
        max_samples: int = 20,
        guard_seconds: float = 0.05,
        stable_quantile: float = 0.95,
    ) -> None:
        if max_samples < 1:
            raise ValueError("max_samples must be positive")
        if guard_seconds < 0:
            raise ValueError("guard_seconds cannot be negative")
        if not 0 < stable_quantile <= 1:
            raise ValueError("stable_quantile must be in (0, 1]")
        self._latencies: deque[float] = deque(maxlen=max_samples)
        self.guard_seconds = float(guard_seconds)
        self.stable_quantile = float(stable_quantile)
        self._epoch = 0
        self._rejected_samples = 0

    @property
    def epoch(self) -> int:
        return self._epoch

    @property
    def stable_samples(self) -> int:
        return len(self._latencies)

    @property
    def rejected_samples(self) -> int:
        return self._rejected_samples

    def record_seconds(
        self,
        latency_seconds: float,
        *,
        knot_hz: float,
        eligible: bool = True,
        rejection_reason: str | None = None,
    ) -> DelaySampleDecision:
        latency = float(latency_seconds)
        rate = float(knot_hz)
        reason = rejection_reason
        if not math.isfinite(latency) or latency < 0:
            reason = reason or "invalid_latency"
        elif not math.isfinite(rate) or rate <= 0:
            reason = reason or "invalid_knot_rate"
        elif math.ceil((latency + self.guard_seconds) * rate) > RTC_MAX_DELAY:
            reason = "exceeds_rtc_horizon"
        elif not eligible:
            reason = reason or "sample_marked_unstable"

        accepted = reason is None
        if accepted:
            self._latencies.append(latency)
        else:
            self._rejected_samples += 1
        return DelaySampleDecision(
            latency_seconds=latency,
            accepted=accepted,
            reason=reason,
            epoch=self._epoch,
            stable_samples=len(self._latencies),
        )

    def reset_epoch(self) -> int:
        self._latencies.clear()
        self._rejected_samples = 0
        self._epoch += 1
        return self._epoch

    def _stable_latency_seconds(self) -> float:
        ordered = sorted(self._latencies)
        rank = max(1, math.ceil(self.stable_quantile * len(ordered)))
        return ordered[rank - 1]

    def predicted_steps(self, knot_hz: float) -> int:
        if not self._latencies:
            raise RtcError("no stable policy latency is available for RTC delay prediction")
        prediction = math.ceil((self._stable_latency_seconds() + self.guard_seconds) * float(knot_hz))
        if not 1 <= prediction <= RTC_MAX_DELAY:
            raise RtcError(f"predicted RTC delay {prediction} is outside 1..{RTC_MAX_DELAY}")
        return prediction


def build_rtc_request(
    *,
    request_id: str,
    plan_id: str,
    timeline_version: int,
    checkpoint_id: int,
    observation: dict,
    old_remaining_actions_absolute,
    predicted_delay_steps: int,
    warmup: bool = False,
) -> dict:
    prefix = np.asarray(old_remaining_actions_absolute, dtype=np.float32)
    expected_prefix = RTC_HORIZON - RTC_EXECUTION_HORIZON
    if prefix.shape != (expected_prefix, 16) or not np.isfinite(prefix).all():
        raise RtcError(f"RTC old prefix must have shape ({expected_prefix}, 16) and be finite, got {prefix.shape}")
    if not 1 <= int(predicted_delay_steps) <= RTC_MAX_DELAY:
        raise RtcError("RTC predicted delay must be within 1..4")
    return {
        "request_type": RTC_REQUEST_TYPE,
        "request_id": request_id,
        "plan_id": plan_id,
        "timeline_version": int(timeline_version),
        "checkpoint_id": int(checkpoint_id),
        "observation": observation,
        "old_remaining_actions_absolute": prefix,
        "d_pred": int(predicted_delay_steps),
        "s": RTC_EXECUTION_HORIZON,
        "schedule": "exp",
        "beta": 5.0,
        "warmup": bool(warmup),
        "client_request_monotonic": time.monotonic(),
    }


def parse_rtc_response(result: dict, *, request: dict) -> tuple[np.ndarray, dict]:
    if not isinstance(result, dict):
        raise RtcError(f"RTC server returned {type(result).__name__}, expected dict")
    if not result.get("ok", False):
        code = result.get("error_code", "unknown_error")
        raise RtcError(f"RTC server rejected request ({code}): {result.get('error', 'no detail')}")
    for key in ("request_id", "plan_id", "timeline_version", "checkpoint_id"):
        if result.get(key) != request.get(key):
            raise RtcError(f"RTC response {key} mismatch: {result.get(key)!r} != {request.get(key)!r}")
    actions = np.asarray(result.get("actions"), dtype=np.float64)
    if actions.shape != (RTC_HORIZON, 16) or not np.isfinite(actions).all():
        raise RtcError(f"RTC actions must have shape (10, 16) and be finite, got {actions.shape}")
    timing = {
        "wall_ms": (time.monotonic() - float(request["client_request_monotonic"])) * 1000.0,
        "client_timing": result.get("client_timing", {}),
        "policy_timing": result.get("policy_timing", {}),
        "server_timing": result.get("server_timing", {}),
        "rtc_timing": result.get("rtc_timing", {}),
    }
    return actions, timing
