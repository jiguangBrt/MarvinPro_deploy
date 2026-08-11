"""Client-side RTC request validation and conservative delay estimation."""

from __future__ import annotations

from collections import deque
import math
import time

import numpy as np


RTC_REQUEST_TYPE = "rtc_v1"
RTC_HORIZON = 10
RTC_EXECUTION_HORIZON = 6
RTC_MAX_DELAY = 4


class RtcError(RuntimeError):
    pass


class DelayEstimator:
    def __init__(self, *, max_samples: int = 20, guard_seconds: float = 0.05) -> None:
        self._latencies: deque[float] = deque(maxlen=max_samples)
        self.guard_seconds = float(guard_seconds)

    def record_seconds(self, latency_seconds: float) -> None:
        latency = float(latency_seconds)
        if math.isfinite(latency) and latency >= 0:
            self._latencies.append(latency)

    def predicted_steps(self, knot_hz: float) -> int:
        if not self._latencies:
            raise RtcError("no stable policy latency is available for RTC delay prediction")
        prediction = math.ceil((max(self._latencies) + self.guard_seconds) * float(knot_hz))
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
        "policy_timing": result.get("policy_timing", {}),
        "server_timing": result.get("server_timing", {}),
        "rtc_timing": result.get("rtc_timing", {}),
    }
    return actions, timing
