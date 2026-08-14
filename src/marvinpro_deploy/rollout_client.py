"""Marvin Pro rollout client for an OpenPI WebSocket policy server."""

from __future__ import annotations

import argparse
import csv
from collections import deque
from dataclasses import dataclass
import logging
import math
from pathlib import Path
import queue
import shlex
import socket
import sys
import threading
import time
import uuid

import numpy as np
from openpi_client import image_tools
from openpi_client import websocket_client_policy

from .config import (
    CONTROL_HZ,
    DEFAULT_BRIDGE_HOST,
    DEFAULT_BRIDGE_PORT,
    DEFAULT_POLICY_HOST,
    DEFAULT_POLICY_PORT,
    DEFAULT_PROMPT,
    JOINT_NAMES,
)
from .image_processing import ImageError, decode_and_split
from .joint_mapping import build_state16
from .motion_profile import FrozenLinearPlan
from .rtc import (
    DelayEstimator,
    RTC_EXECUTION_HORIZON,
    RTC_HORIZON,
    RtcError,
    build_rtc_request,
    parse_rtc_response,
)
from .protocol import (
    ActionCommand,
    BridgeHello,
    HoldPositionCommand,
    LoadTrajectoryCommand,
    ProtocolError,
    RobotObservation,
    RobotStateUpdate,
    ResumeTrajectoryCommand,
    StageRtcChunkCommand,
    StopCommand,
    TrajectoryEvent,
    TrajectoryHeartbeat,
    recv_message,
    require_current_version,
    send_message,
)
from .safety import SafetyError, action_arms, filter_action

LOGGER = logging.getLogger("marvinpro_rollout")

_ACTIVE_STATE_LOG_INTERVAL_S = 0.10
_HOLD_STATE_LOG_INTERVAL_S = 1.0
_ACTION_NAMES = JOINT_NAMES[:7] + ("Gripper_L",) + JOINT_NAMES[7:] + ("Gripper_R",)
_TRACKING_PLAYBACK_TIME_SCALE = 3.0


class JointTelemetryRecorder:
    """Write arm feedback, gripper command proxies, and outgoing commands."""

    def __init__(self, path: Path) -> None:
        path = path.expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file = path.open("w", encoding="utf-8", newline="", buffering=1)
        self._writer = csv.writer(self._file)
        self._lock = threading.Lock()
        self._rows: queue.Queue[list[object] | None] = queue.Queue()
        self._closed = False
        self._writer.writerow(self._columns())
        self._writer_thread = threading.Thread(target=self._write_loop, name="joint-telemetry", daemon=True)
        self._writer_thread.start()

    @staticmethod
    def _columns() -> list[str]:
        columns = [
            "record_type",
            "recorded_monotonic",
            "sampled_monotonic",
            "state_seq",
            "observation_seq",
            "command_id",
            "trajectory_mode",
            "timeline_version",
            "phase",
            "phase_rate",
            "tracking_error_rad",
            "servo_error_rad",
            "arm_clipped",
            "frozen_reason",
            "active_request_id",
        ]
        columns.extend(f"measured_{name}" for name in JOINT_NAMES)
        columns.extend(
            (
                "gripper_command_proxy_L",
                "gripper_command_proxy_R",
                "measured_gripper_position_raw_L",
                "measured_gripper_position_raw_R",
                "measured_gripper_position_L",
                "measured_gripper_position_R",
                "measured_gripper_velocity_L",
                "measured_gripper_velocity_R",
                "measured_gripper_torque_L",
                "measured_gripper_torque_R",
                "gripper_position_error_L",
                "gripper_position_error_R",
                "measured_gripper_mos_temperature_L",
                "measured_gripper_mos_temperature_R",
                "measured_gripper_motor_temperature_L",
                "measured_gripper_motor_temperature_R",
            )
        )
        columns.extend(f"raw_reference_{name}" for name in _ACTION_NAMES)
        columns.extend(f"bridge_command_{name}" for name in _ACTION_NAMES)
        columns.extend(f"client_reference_{name}" for name in _ACTION_NAMES)
        columns.extend(f"client_command_{name}" for name in _ACTION_NAMES)
        return columns

    @staticmethod
    def _values(values: tuple[float, ...] | None, size: int) -> list[float | str]:
        if values is None:
            return [""] * size
        if len(values) != size:
            raise ValueError(f"telemetry vector has length {len(values)}, expected {size}")
        return list(values)

    @staticmethod
    def _gripper_values(message, target: tuple[float, ...] | None) -> list[float | str]:
        del target
        return [
            message.gripper_raw_left,
            message.gripper_raw_right,
            "", "", "", "",
            "", "", "", "",
            "", "", "", "",
            "", "",
        ]

    def record_state(self, message: RobotStateUpdate, received_monotonic: float) -> None:
        row = [
            "bridge_state",
            received_monotonic,
            message.sampled_monotonic,
            message.state_seq,
            "",
            message.last_command_id if message.last_command_id is not None else "",
            message.trajectory_mode,
            message.timeline_version,
            "" if message.phase is None else message.phase,
            message.phase_rate,
            "" if message.tracking_error_rad is None else message.tracking_error_rad,
            "" if message.servo_error_rad is None else message.servo_error_rad,
            int(message.arm_clipped),
            message.frozen_reason or "",
            message.active_request_id or "",
        ]
        row.extend(message.joints)
        row.extend(self._gripper_values(message, message.sent_target))
        row.extend(self._values(message.raw_reference, 16))
        row.extend(self._values(message.sent_target, 16))
        row.extend([""] * 16)
        self._write(row)

    def record_client_command(
        self,
        *,
        recorded_monotonic: float,
        observation: RobotObservation,
        command_id: int,
        requested_action: tuple[float, ...],
        sent_action: tuple[float, ...],
        was_hold: bool,
    ) -> None:
        row = [
            "client_hold" if was_hold else "client_command",
            recorded_monotonic,
            observation.captured_monotonic,
            "",
            observation.seq,
            command_id,
            "legacy_client",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
        ]
        row.extend(observation.joints)
        row.extend(self._gripper_values(observation, sent_action))
        row.extend([""] * 16)
        row.extend([""] * 16)
        row.extend(self._values(requested_action, 16))
        row.extend(self._values(sent_action, 16))
        self._write(row)

    def _write(self, row: list[object]) -> None:
        with self._lock:
            if self._closed:
                return
            self._rows.put_nowait(row)

    def _write_loop(self) -> None:
        while True:
            row = self._rows.get()
            try:
                if row is None:
                    return
                self._writer.writerow(row)
                self._file.flush()
            finally:
                self._rows.task_done()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._rows.put_nowait(None)
        self._rows.join()
        self._writer_thread.join(timeout=1.0)
        self._file.close()


class RolloutError(RuntimeError):
    pass


def _state_log_interval_s(trajectory_mode: str) -> float:
    if trajectory_mode == "hold":
        return _HOLD_STATE_LOG_INTERVAL_S
    return _ACTIVE_STATE_LOG_INTERVAL_S


class RobotConnection:
    def __init__(
        self,
        host: str,
        port: int,
        connect_timeout_s: float = 5.0,
        telemetry: JointTelemetryRecorder | None = None,
    ) -> None:
        self._socket = socket.create_connection((host, port), timeout=connect_timeout_s)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        hello = recv_message(self._socket)
        if not isinstance(hello, BridgeHello):
            self._socket.close()
            raise RolloutError(f"expected BridgeHello, got {type(hello).__name__}")
        require_current_version(hello)
        self._socket.settimeout(None)
        self.hello = hello

        self._send_lock = threading.Lock()
        self._condition = threading.Condition()
        self._latest_observation: RobotObservation | None = None
        self._latest_observation_received = 0.0
        self._latest_state: RobotStateUpdate | None = None
        self._latest_state_received = 0.0
        self._telemetry = telemetry
        self._events: deque[TrajectoryEvent] = deque()
        self._last_event_seq = 0
        self._last_state_log_monotonic = 0.0
        self._last_state_log_signature: tuple[object, ...] | None = None
        self._error: BaseException | None = None
        self._closed = False
        self._receiver = threading.Thread(target=self._receive_loop, name="robot-observations", daemon=True)
        self._receiver.start()

    def _receive_loop(self) -> None:
        try:
            while True:
                message = recv_message(self._socket)
                if message is None:
                    raise ConnectionError("robot bridge closed the connection")
                require_current_version(message)
                with self._condition:
                    received = time.monotonic()
                    if isinstance(message, RobotObservation):
                        if self._latest_observation is None or message.seq > self._latest_observation.seq:
                            self._latest_observation = message
                            self._latest_observation_received = received
                    elif isinstance(message, RobotStateUpdate):
                        if self._latest_state is None or message.state_seq > self._latest_state.state_seq:
                            self._latest_state = message
                            self._latest_state_received = received
                            if self._telemetry is not None:
                                self._telemetry.record_state(message, received)
                            signature = (
                                message.motion_gate_open,
                                message.trajectory_mode,
                                message.session_id,
                                message.plan_id,
                                message.timeline_version,
                                message.arm_clipped,
                                message.frozen_reason,
                                message.active_request_id,
                            )
                            periodic = (
                                message.trajectory_mode != "legacy"
                                and received - self._last_state_log_monotonic
                                >= _state_log_interval_s(message.trajectory_mode)
                            )
                            if signature != self._last_state_log_signature or periodic:
                                reference_delta = _reference_delta(message.raw_reference, message.sent_target)
                                LOGGER.debug(
                                    "bridge_state seq=%d source=%.6f gate=%s mode=%s session=%s plan=%s "
                                    "version=%d phase=%s phase_rate=%.6f tracking_error=%s servo_error=%s "
                                    "reference_delta=%s arm_clipped=%s frozen=%s request=%s "
                                    "joints=%s raw_reference=%s sent_target=%s status=%r",
                                    message.state_seq,
                                    message.sampled_monotonic,
                                    message.motion_gate_open,
                                    message.trajectory_mode,
                                    message.session_id,
                                    message.plan_id,
                                    message.timeline_version,
                                    message.phase,
                                    message.phase_rate,
                                    message.tracking_error_rad,
                                    message.servo_error_rad,
                                    reference_delta,
                                    message.arm_clipped,
                                    message.frozen_reason,
                                    message.active_request_id,
                                    message.joints,
                                    message.raw_reference,
                                    message.sent_target,
                                    message.last_command_status,
                                )
                                self._last_state_log_monotonic = received
                                self._last_state_log_signature = signature
                    elif isinstance(message, TrajectoryEvent):
                        if message.event_seq > self._last_event_seq:
                            self._events.append(message)
                            self._last_event_seq = message.event_seq
                            LOGGER.info(
                                "bridge_event type=%s event_seq=%d session=%s plan=%s version=%d "
                                "phase=%.6f checkpoint=%s request=%s d_pred=%s d_actual=%s "
                                "tracking_error=%s servo_error=%s settle_s=%s joint_source=%s "
                                "reference_delta=%s arm_clipped=%s frozen=%s continuous_checkpoint=%s "
                                "raw_reference=%s "
                                "sent_target=%s boundary_old_velocity=%s boundary_new_velocity=%s "
                                "boundary_velocity_jump_rad=%s boundary_acceleration_jump_rad=%s detail=%r",
                                message.event_type,
                                message.event_seq,
                                message.session_id,
                                message.plan_id,
                                message.timeline_version,
                                message.phase,
                                message.checkpoint_id,
                                message.request_id,
                                message.predicted_delay_steps,
                                message.actual_delay_steps,
                                message.tracking_error_rad,
                                message.servo_error_rad,
                                message.settle_duration_s,
                                message.joint_source_monotonic,
                                _reference_delta(message.raw_reference, message.sent_target),
                                message.arm_clipped,
                                message.frozen_reason,
                                getattr(message, "continuous_checkpoint", False),
                                message.raw_reference,
                                message.sent_target,
                                getattr(message, "boundary_old_velocity", ()),
                                getattr(message, "boundary_new_velocity", ()),
                                getattr(message, "boundary_velocity_jump_rad", None),
                                getattr(message, "boundary_acceleration_jump_rad", None),
                                message.detail,
                            )
                    else:
                        raise ProtocolError(f"unexpected bridge message {type(message).__name__}")
                    self._condition.notify_all()
        except BaseException as exc:
            with self._condition:
                if not self._closed:
                    self._error = exc
                self._condition.notify_all()

    def latest(self, max_local_age_s: float | None = None) -> RobotObservation:
        with self._condition:
            if self._error is not None:
                raise RolloutError(f"robot bridge receive failed: {self._error}")
            if self._latest_observation is None:
                raise RolloutError("no robot observation received")
            if (
                max_local_age_s is not None
                and time.monotonic() - self._latest_observation_received > max_local_age_s
            ):
                raise RolloutError("latest robot observation is stale on the rollout client")
            return self._latest_observation

    def latest_state(self, max_local_age_s: float | None = None) -> RobotStateUpdate:
        with self._condition:
            if self._error is not None:
                raise RolloutError(f"robot bridge receive failed: {self._error}")
            if self._latest_state is None:
                raise RolloutError("no robot state update received")
            if max_local_age_s is not None and time.monotonic() - self._latest_state_received > max_local_age_s:
                raise RolloutError("latest robot state is stale on the rollout client")
            return self._latest_state

    def wait_for_state(
        self,
        *,
        timeout_s: float,
        newer_than: int | None = None,
        require_motion_gate: bool = False,
    ) -> RobotStateUpdate:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise RolloutError(f"robot bridge receive failed: {self._error}")
                state = self._latest_state
                is_new = state is not None and (newer_than is None or state.state_seq > newer_than)
                gate_ok = state is not None and (not require_motion_gate or state.motion_gate_open)
                if is_new and gate_ok:
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RolloutError("timed out waiting for robot state")
                self._condition.wait(timeout=min(remaining, 0.25))

    def wait_for_event(
        self,
        *,
        timeout_s: float,
        event_types: tuple[str, ...] | None = None,
        newer_than: int | None = None,
    ) -> TrajectoryEvent:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise RolloutError(f"robot bridge receive failed: {self._error}")
                for event in tuple(self._events):
                    if newer_than is not None and event.event_seq <= newer_than:
                        self._events.popleft()
                        continue
                    if event_types is None or event.event_type in event_types:
                        self._events.remove(event)
                        return event
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    expected = "any" if event_types is None else ",".join(event_types)
                    raise RolloutError(f"timed out waiting for trajectory event ({expected})")
                self._condition.wait(timeout=min(remaining, 0.25))

    def wait_for_observation(
        self,
        *,
        timeout_s: float,
        newer_than: int | None = None,
        require_motion_gate: bool = False,
    ) -> RobotObservation:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise RolloutError(f"robot bridge receive failed: {self._error}")
                observation = self._latest_observation
                is_new = observation is not None and (newer_than is None or observation.seq > newer_than)
                gate_ok = observation is not None and (not require_motion_gate or observation.motion_gate_open)
                if is_new and gate_ok:
                    return observation
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    detail = ""
                    if observation is not None:
                        detail = f"; latest gate={observation.motion_gate_open}: {observation.gate_reason}"
                    raise RolloutError(f"timed out waiting for robot observation{detail}")
                self._condition.wait(timeout=min(remaining, 0.5))

    def send_action(self, command: ActionCommand) -> None:
        self.send(command)

    def send(self, message) -> None:
        with self._condition:
            if self._error is not None:
                raise RolloutError(f"robot bridge receive failed: {self._error}")
        send_message(self._socket, message, self._send_lock)

    def close(self, reason: str) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
        try:
            send_message(self._socket, StopCommand(reason=reason), self._send_lock)
        except OSError:
            pass
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._socket.close()
        self._receiver.join(timeout=1.0)
        if self._telemetry is not None:
            self._telemetry.close()


@dataclass(frozen=True)
class PlanStep:
    action: tuple[float, ...]
    observation_seq: int


@dataclass(frozen=True)
class PlanReplacement:
    discarded_steps: int
    old_next_action: tuple[float, ...] | None


@dataclass(frozen=True)
class PlanAppend:
    queued_steps: int
    added_steps: int
    anchor_action: tuple[float, ...]
    final_action: tuple[float, ...]


@dataclass(frozen=True)
class PublisherSnapshot:
    sent: int
    plan_steps_sent: int
    underruns: int
    clipped: int
    arm_clipped: int
    gripper_clipped: int
    last_action: tuple[float, ...] | None
    last_was_hold: bool
    latched_plan_action: tuple[float, ...] | None


@dataclass(frozen=True)
class TrackingResult:
    observation: RobotObservation
    elapsed_s: float
    max_error_rad: float
    final_error_rad: float
    worst_joint: str


def _reference_delta(reference, target) -> float | None:
    if reference is None or target is None:
        return None
    reference_arms = np.asarray(action_arms(tuple(float(value) for value in reference)))
    target_arms = np.asarray(action_arms(tuple(float(value) for value in target)))
    return float(np.max(np.abs(reference_arms - target_arms)))


class ActionPlan:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._steps: deque[PlanStep] = deque()

    def replace(self, actions: np.ndarray, observation_seq: int, execute_steps: int) -> PlanReplacement:
        steps = [
            PlanStep(tuple(float(value) for value in row), observation_seq)
            for row in np.asarray(actions)[:execute_steps]
        ]
        with self._condition:
            replacement = PlanReplacement(
                discarded_steps=len(self._steps),
                old_next_action=self._steps[0].action if self._steps else None,
            )
            self._steps = deque(steps)
            self._condition.notify_all()
            return replacement

    def pop(self) -> PlanStep | None:
        with self._condition:
            if not self._steps:
                return None
            step = self._steps.popleft()
            self._condition.notify_all()
            return step

    def append_interpolated(
        self,
        actions: np.ndarray,
        observation_seq: int,
        execute_steps: int,
        *,
        fallback_anchor: tuple[float, ...],
        model_hz: float,
        playback_time_scale: float,
        command_hz: float,
    ) -> PlanAppend:
        action_knots = tuple(tuple(float(value) for value in row) for row in np.asarray(actions)[:execute_steps])
        effective_knot_hz = model_hz / playback_time_scale
        with self._condition:
            queued_steps = len(self._steps)
            anchor = self._steps[-1].action if self._steps else fallback_anchor
            trajectory = FrozenLinearPlan((anchor, *action_knots), effective_knot_hz)
            sample_count = math.ceil(trajectory.duration * command_hz)
            steps = [
                PlanStep(
                    trajectory.value(min(index / command_hz, trajectory.duration)),
                    observation_seq,
                )
                for index in range(1, sample_count + 1)
            ]
            self._steps.extend(steps)
            self._condition.notify_all()
            return PlanAppend(
                queued_steps=queued_steps,
                added_steps=len(steps),
                anchor_action=anchor,
                final_action=action_knots[-1],
            )

    def clear(self) -> None:
        with self._condition:
            self._steps.clear()
            self._condition.notify_all()

    def remaining(self) -> int:
        with self._condition:
            return len(self._steps)

    def wait_until_at_most(self, count: int, stop: threading.Event, timeout_s: float = 0.25) -> None:
        with self._condition:
            while len(self._steps) > count and not stop.is_set():
                self._condition.wait(timeout=timeout_s)


class ActionPublisher:
    def __init__(
        self,
        connection: RobotConnection,
        plan: ActionPlan,
        stop: threading.Event,
        *,
        execute: bool,
        control_hz: float,
        max_joint_step_rad: float,
        max_observation_age_s: float,
        joint_limit_margin_rad: float,
        warn_on_plan_empty: bool,
        refresh_observation_seq: bool,
        hold_last_plan_action: bool,
        telemetry: JointTelemetryRecorder | None = None,
    ) -> None:
        self.connection = connection
        self.plan = plan
        self.stop = stop
        self.execute = execute
        self.period_s = 1.0 / control_hz
        self.max_joint_step_rad = max_joint_step_rad
        self.max_observation_age_s = max_observation_age_s
        self.joint_limit_margin_rad = joint_limit_margin_rad
        self.refresh_observation_seq = refresh_observation_seq
        self.hold_last_plan_action = hold_last_plan_action
        self.telemetry = telemetry
        self.error: BaseException | None = None
        self.sent = 0
        self.plan_steps_sent = 0
        self.underruns = 0
        self.clipped = 0
        self.arm_clipped = 0
        self.gripper_clipped = 0
        self._state_lock = threading.Lock()
        self._last_action: tuple[float, ...] | None = None
        self._last_was_hold = False
        self._latched_plan_action: tuple[float, ...] | None = None
        self._allow_plan_latching = True
        self._warn_on_plan_empty = warn_on_plan_empty
        self._has_published_plan_action = False
        self._tick_count = 0
        self._late_tick_count = 0
        self._skipped_tick_count = 0
        self._max_tick_gap_s = 0.0
        self._thread = threading.Thread(target=self._run, name="action-publisher", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout=timeout)

    def snapshot(self) -> PublisherSnapshot:
        with self._state_lock:
            return PublisherSnapshot(
                sent=self.sent,
                plan_steps_sent=self.plan_steps_sent,
                underruns=self.underruns,
                clipped=self.clipped,
                arm_clipped=self.arm_clipped,
                gripper_clipped=self.gripper_clipped,
                last_action=self._last_action,
                last_was_hold=self._last_was_hold,
                latched_plan_action=self._latched_plan_action,
            )

    def suppress_plan_empty_warnings(self) -> None:
        with self._state_lock:
            self._warn_on_plan_empty = False

    def hold_fixed_pose(self, action) -> None:
        """Latch one pose for empty-plan publication until the client disconnects."""
        values = tuple(float(value) for value in action)
        if len(values) != 16 or not all(math_isfinite(value) for value in values):
            raise RolloutError("fixed hold action must contain 16 finite values")
        with self._state_lock:
            self.hold_last_plan_action = True
            self._latched_plan_action = values
            # A plan step already popped by the publisher must not overwrite the shutdown latch.
            self._allow_plan_latching = False

    def timing_snapshot(self) -> tuple[int, int, int, float]:
        with self._state_lock:
            return (
                self._tick_count,
                self._late_tick_count,
                self._skipped_tick_count,
                self._max_tick_gap_s,
            )

    def _run(self) -> None:
        command_id = 0
        next_tick = time.monotonic()
        previous_tick = None
        last_underrun_log = 0.0
        last_arm_clip_log = 0.0
        last_gripper_clip_log = 0.0
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if now < next_tick:
                    self.stop.wait(next_tick - now)
                    continue
                if previous_tick is not None:
                    tick_gap = now - previous_tick
                    with self._state_lock:
                        self._max_tick_gap_s = max(self._max_tick_gap_s, tick_gap)
                        self._late_tick_count += tick_gap > 1.5 * self.period_s
                        self._skipped_tick_count += max(0, int(tick_gap / self.period_s) - 1)
                previous_tick = now
                with self._state_lock:
                    self._tick_count += 1
                while next_tick <= now:
                    next_tick += self.period_s

                step = self.plan.pop()
                was_hold = step is None
                if step is None:
                    with self._state_lock:
                        self.underruns += 1
                        warn_on_plan_empty = self._warn_on_plan_empty and self._has_published_plan_action
                    if warn_on_plan_empty and now - last_underrun_log >= 2.0:
                        LOGGER.warning("action plan empty; commanding measured-pose hold")
                        last_underrun_log = now

                observation = self.connection.latest(self.max_observation_age_s)
                if self.execute and not observation.motion_gate_open:
                    raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
                if step is None:
                    if not self.execute:
                        continue
                    with self._state_lock:
                        latched_action = self._latched_plan_action if self.hold_last_plan_action else None
                    step = PlanStep(
                        action=(
                            latched_action
                            if latched_action is not None
                            else build_state16(
                                observation.joints,
                                observation.gripper_raw_left,
                                observation.gripper_raw_right,
                            )
                        ),
                        observation_seq=observation.seq,
                    )
                filtered = filter_action(
                    step.action,
                    observation.joints,
                    max_joint_step_rad=self.max_joint_step_rad,
                    joint_limit_margin_rad=self.joint_limit_margin_rad,
                )
                arm_was_clipped = any(index not in (7, 15) for index in filtered.clipped_indices)
                gripper_was_clipped = any(index in (7, 15) for index in filtered.clipped_indices)
                if arm_was_clipped and now - last_arm_clip_log >= 1.0:
                    clip_details = tuple(
                        f"{index}:{step.action[index]:.5f}->{filtered.action[index]:.5f}"
                        for index in filtered.clipped_indices
                        if index not in (7, 15)
                    )
                    LOGGER.warning(
                        "safety filter clipped arm dimensions %s raw_to_sent=%s",
                        tuple(index for index in filtered.clipped_indices if index not in (7, 15)),
                        clip_details,
                    )
                    last_arm_clip_log = now
                elif gripper_was_clipped and now - last_gripper_clip_log >= 2.0:
                    clip_details = tuple(
                        f"{index}:{step.action[index]:.5f}->{filtered.action[index]:.5f}"
                        for index in filtered.clipped_indices
                        if index in (7, 15)
                    )
                    LOGGER.debug(
                        "safety filter is repeatedly clamping gripper dimensions %s raw_to_sent=%s",
                        tuple(index for index in filtered.clipped_indices if index in (7, 15)),
                        clip_details,
                    )
                    last_gripper_clip_log = now
                command_id += 1
                if self.execute:
                    self.connection.send_action(
                        ActionCommand(
                            command_id=command_id,
                            observation_seq=(observation.seq if self.refresh_observation_seq else step.observation_seq),
                            action=filtered.action,
                            execute=True,
                        )
                    )
                    if self.telemetry is not None:
                        self.telemetry.record_client_command(
                            recorded_monotonic=now,
                            observation=observation,
                            command_id=command_id,
                            requested_action=step.action,
                            sent_action=filtered.action,
                            was_hold=was_hold,
                        )
                with self._state_lock:
                    self.sent += 1
                    self.plan_steps_sent += not was_hold
                    self.clipped += bool(filtered.clipped_indices)
                    self.arm_clipped += arm_was_clipped
                    self.gripper_clipped += gripper_was_clipped
                    self._last_action = filtered.action
                    self._last_was_hold = was_hold
                    if not was_hold and self._allow_plan_latching:
                        self._latched_plan_action = step.action
                    self._has_published_plan_action = self._has_published_plan_action or not was_hold
        except BaseException as exc:
            self.error = exc
            self.stop.set()


def validate_observation(observation: RobotObservation, max_source_age_s: float) -> None:
    if len(observation.joints) != 14 or not all(math_isfinite(value) for value in observation.joints):
        raise RolloutError("robot observation has invalid joint state")
    if not observation.image:
        raise RolloutError("robot observation has no camera image")
    ages = {"joint state": observation.age_state_s}
    for label, age in ages.items():
        if age is None or age > max_source_age_s:
            raise RolloutError(f"{label} is stale: age={age}")


def math_isfinite(value) -> bool:
    try:
        return bool(np.isfinite(float(value)))
    except (TypeError, ValueError):
        return False


def _arm_delta(action, reference) -> float:
    action_values = np.asarray(action_arms(tuple(float(value) for value in action)))
    reference_values = np.asarray(action_arms(tuple(float(value) for value in reference)))
    return float(np.max(np.abs(action_values - reference_values)))


def _candidate_deltas(actions: np.ndarray, reference, count: int = 5) -> str:
    if reference is None:
        return "n/a"
    values = [_arm_delta(row, reference) for row in np.asarray(actions)[:count]]
    return "[" + ",".join(f"{value:.5f}" for value in values) + "]"


def build_policy_observation(observation: RobotObservation, prompt: str) -> dict:
    state = np.asarray(
        build_state16(
            observation.joints,
            observation.gripper_raw_left,
            observation.gripper_raw_right,
        ),
        dtype=np.float32,
    )
    images_640 = decode_and_split(observation.image)
    images_224 = {
        name: image_tools.convert_to_uint8(image_tools.resize_with_pad(image, 224, 224))
        for name, image in images_640.items()
    }
    return {"state": state, "images": images_224, "prompt": prompt}


def infer_actions(policy, observation: RobotObservation, prompt: str) -> tuple[np.ndarray, dict]:
    started = time.monotonic()
    try:
        policy_observation = build_policy_observation(observation, prompt)
    except (ImageError, SafetyError, ValueError) as exc:
        raise RolloutError(f"cannot build policy observation: {exc}") from exc
    result = policy.infer(policy_observation)
    wall_ms = (time.monotonic() - started) * 1000.0
    actions = np.asarray(result.get("actions"))
    if actions.ndim != 2 or actions.shape[1] != 16 or actions.shape[0] < 1:
        raise RolloutError(f"policy actions must have shape [horizon, 16], got {actions.shape}")
    if not np.isfinite(actions).all():
        raise RolloutError("policy returned NaN or Inf")
    timing = {
        "wall_ms": wall_ms,
        "policy_timing": result.get("policy_timing", {}),
        "server_timing": result.get("server_timing", {}),
    }
    return actions, timing


def _wait_for_ready(connection: RobotConnection, timeout_s: float) -> RobotObservation:
    print("\nROLLOUT READY")
    print("  Now change Apex Input Mode to Custom.")
    print("  Waiting for input_mode=3, robot_state=(3, 3), arm_state=(3, 3)...", flush=True)
    observation = connection.wait_for_observation(timeout_s=timeout_s, require_motion_gate=True)
    print("  Motion gate is ready.")
    LOGGER.warning(
        "motion_gate_ready observation_seq=%d input_mode=%s robot_state=%s arm_state=%s",
        observation.seq,
        observation.input_mode,
        observation.robot_state,
        observation.arm_state,
    )
    return observation


def _confirm_execution(args: argparse.Namespace, observation: RobotObservation) -> None:
    LOGGER.warning(
        "real_robot_execution_requested observation_seq=%d input_mode=%s robot_state=%s "
        "arm_state=%s duration_s=%.3f schedule=%s",
        observation.seq,
        observation.input_mode,
        observation.robot_state,
        observation.arm_state,
        args.episode_seconds,
        args.rollout_schedule,
    )
    print("\nREAL ROBOT EXECUTION REQUESTED")
    print(f"  prompt: {args.prompt}")
    print(f"  input_mode: {observation.input_mode}")
    print(f"  robot_state: {observation.robot_state}")
    print(f"  arm_state: {observation.arm_state}")
    print(f"  duration: {args.episode_seconds:.1f}s")
    if args.playback_mode == "interpolated":
        effective_knot_hz = args.model_hz / args.playback_time_scale
        chunk_seconds = args.execute_steps / effective_knot_hz
        print("  action playback: continuous piecewise-linear")
        print(f"  policy knot rate: {args.model_hz:.1f}Hz")
        print(f"  playback time scale: {args.playback_time_scale:.2f}x")
        print(f"  effective knot rate: {effective_knot_hz:.2f}Hz")
        print(f"  command rate: {args.control_hz:.1f}Hz")
        print(f"  selected chunk: {args.execute_steps} knots over {chunk_seconds:.3f}s")
        if args.rollout_schedule == "synchronized":
            print("  rollout schedule: execute -> track -> hold -> observe -> infer")
            print(f"  tracking tolerance: {args.tracking_tolerance_rad:.5f}rad")
            print(f"  tracking settle time: {args.tracking_settle_seconds:.2f}s")
            print(f"  post-track hold: {args.post_track_hold_seconds:.2f}s")
            print(f"  tracking timeout: {args.tracking_timeout:.1f}s")
        elif args.rollout_schedule == "prefetch":
            print(f"  next-chunk inference lead: {args.chunk_prefetch_seconds:.2f}s")
        else:
            print(f"  rollout schedule: bridge-owned {args.rollout_schedule}")
            if args.rollout_schedule == "rtc" and args.rtc_continuous:
                print(f"  RTC checkpoint: continuous at action {RTC_EXECUTION_HORIZON} (no settle hold)")
            else:
                print(
                    f"  physical checkpoint: {RTC_EXECUTION_HORIZON} actions"
                    if args.rollout_schedule == "rtc"
                    else "  full-chunk checkpoint"
                )
            if args.rollout_schedule == "rtc" and args.max_rtc_merges is not None:
                print(f"  maximum RTC merges: {args.max_rtc_merges}")
    print("Keep the emergency stop reachable. Switch Input Mode to None before stopping the bridge.")
    if args.yes:
        LOGGER.warning("real_robot_execution_confirmed method=--yes")
        return
    answer = input('Type exactly "E" to start motion: ')
    if answer != "E":
        raise RolloutError("execution confirmation was not given")
    LOGGER.warning("real_robot_execution_confirmed method=typed_E")


def _confirm_and_refresh_execution_observation(
    args: argparse.Namespace,
    connection: RobotConnection,
    observation: RobotObservation,
) -> RobotObservation:
    _confirm_execution(args, observation)
    latest_after_confirmation = connection.latest(args.max_observation_age)
    fresh = connection.wait_for_observation(
        timeout_s=args.observation_timeout,
        newer_than=latest_after_confirmation.seq,
        require_motion_gate=True,
    )
    validate_observation(fresh, args.max_source_age)
    LOGGER.warning(
        "post_confirmation_observation baseline_seq=%d fresh_seq=%d captured=%.6f "
        "joint_age_ms=%.3f gripper_proxy=(%.3f,%.3f) source=%s",
        latest_after_confirmation.seq,
        fresh.seq,
        fresh.captured_monotonic,
        float(fresh.age_state_s) * 1000.0,
        fresh.gripper_raw_left,
        fresh.gripper_raw_right,
        getattr(fresh, "extra", {}).get("gripper_state_source", "unknown"),
    )
    return fresh


def _wait_for_none_after_rollout(
    connection: RobotConnection,
    publisher: ActionPublisher,
    plan: ActionPlan,
    timeout_s: float,
) -> None:
    publisher.suppress_plan_empty_warnings()
    observation = connection.latest()
    fixed_hold_action = build_state16(
        observation.joints,
        observation.gripper_raw_left,
        observation.gripper_raw_right,
    )
    publisher.hold_fixed_pose(fixed_hold_action)
    plan.clear()
    print("\nROLLOUT COMPLETE")
    print("  Motion commands are finished; one measured pose has been latched and is being held.")
    print("  Now change Apex Input Mode to None.")
    print("  Waiting for input_mode=0...", flush=True)
    deadline = time.monotonic() + timeout_s
    last_seq = -1
    while True:
        observation = connection.latest()
        if observation.input_mode != 3:
            print(f"  Input Mode is {observation.input_mode}; rollout client can disconnect safely.")
            LOGGER.warning("rollout_exit_mode_confirmed input_mode=%s", observation.input_mode)
            return
        if publisher.error is not None:
            raise RolloutError(f"action publisher failed while waiting for Input Mode None: {publisher.error}")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RolloutError(
                f"Input Mode stayed Custom for {timeout_s:.1f}s after rollout; disconnecting via watchdog"
            )
        try:
            observation = connection.wait_for_observation(timeout_s=min(1.0, remaining), newer_than=last_seq)
            last_seq = observation.seq
        except RolloutError as exc:
            if "timed out waiting" not in str(exc):
                raise


def _wait_for_plan_threshold(
    plan: ActionPlan,
    publisher: ActionPublisher,
    stop: threading.Event,
    *,
    threshold: int,
    episode_deadline: float,
) -> None:
    while plan.remaining() > threshold and not stop.is_set() and time.monotonic() < episode_deadline:
        plan.wait_until_at_most(threshold, stop, timeout_s=0.05)
        if publisher.error is not None:
            raise RolloutError(f"action publisher failed: {publisher.error}")


def _check_runtime_observation(
    observation: RobotObservation,
    *,
    max_source_age_s: float,
) -> None:
    validate_observation(observation, max_source_age_s)
    if not observation.motion_gate_open:
        raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
    status = observation.last_command_status
    if status.startswith("rejected") or "failed" in status:
        raise RolloutError(f"bridge {status}")


def _wait_for_plan_dispatch(
    connection: RobotConnection,
    publisher: ActionPublisher,
    stop: threading.Event,
    *,
    target_plan_steps_sent: int,
    timeout_s: float,
    max_source_age_s: float,
) -> PublisherSnapshot:
    deadline = time.monotonic() + timeout_s
    while not stop.is_set():
        if publisher.error is not None:
            raise RolloutError(f"action publisher failed: {publisher.error}")
        snapshot = publisher.snapshot()
        if snapshot.plan_steps_sent >= target_plan_steps_sent:
            return snapshot
        observation = connection.latest()
        _check_runtime_observation(observation, max_source_age_s=max_source_age_s)
        if time.monotonic() >= deadline:
            raise RolloutError(
                "timed out waiting for the complete policy chunk to be dispatched "
                f"({snapshot.plan_steps_sent}/{target_plan_steps_sent} plan ticks)"
            )
        stop.wait(0.01)
    raise RolloutError("rollout stopped while dispatching a policy chunk")


def _tracking_error(target_action: tuple[float, ...], observation: RobotObservation) -> tuple[float, int]:
    errors = np.abs(
        np.asarray(action_arms(target_action), dtype=np.float64) - np.asarray(observation.joints, dtype=np.float64)
    )
    worst_index = int(np.argmax(errors))
    return float(errors[worst_index]), worst_index


def _wait_for_target_tracking(
    connection: RobotConnection,
    publisher: ActionPublisher,
    stop: threading.Event,
    *,
    target_action: tuple[float, ...],
    tolerance_rad: float,
    settle_seconds: float,
    timeout_s: float,
    max_source_age_s: float,
) -> TrackingResult:
    started = time.monotonic()
    deadline = started + timeout_s
    stable_since: float | None = None
    max_error = 0.0
    final_error = math.inf
    worst_index = 0
    observation = connection.latest()
    last_seq = observation.seq

    while not stop.is_set():
        if publisher.error is not None:
            raise RolloutError(f"action publisher failed while tracking: {publisher.error}")
        _check_runtime_observation(observation, max_source_age_s=max_source_age_s)
        final_error, current_worst_index = _tracking_error(target_action, observation)
        max_error = max(max_error, final_error)
        worst_index = current_worst_index
        now = time.monotonic()
        if final_error <= tolerance_rad:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= settle_seconds:
                return TrackingResult(
                    observation=observation,
                    elapsed_s=now - started,
                    max_error_rad=max_error,
                    final_error_rad=final_error,
                    worst_joint=JOINT_NAMES[worst_index],
                )
        else:
            stable_since = None
        if now >= deadline:
            raise RolloutError(
                f"tracking timeout after {timeout_s:.1f}s: max error {final_error:.5f}rad "
                f"at {JOINT_NAMES[worst_index]} (limit {tolerance_rad:.5f}rad)"
            )
        try:
            observation = connection.wait_for_observation(timeout_s=min(0.5, deadline - now), newer_than=last_seq)
            last_seq = observation.seq
        except RolloutError as exc:
            if "timed out waiting" not in str(exc):
                raise
    raise RolloutError("rollout stopped while waiting for target tracking")


def _hold_target_and_reobserve(
    connection: RobotConnection,
    publisher: ActionPublisher,
    stop: threading.Event,
    *,
    target_action: tuple[float, ...],
    tolerance_rad: float,
    hold_seconds: float,
    timeout_s: float,
    max_source_age_s: float,
) -> tuple[RobotObservation, float]:
    started = time.monotonic()
    deadline = started + timeout_s
    hold_started: float | None = None
    max_error = 0.0
    observation = connection.latest()
    last_seq = observation.seq

    while not stop.is_set():
        if publisher.error is not None:
            raise RolloutError(f"action publisher failed while holding: {publisher.error}")
        _check_runtime_observation(observation, max_source_age_s=max_source_age_s)
        error, _ = _tracking_error(target_action, observation)
        max_error = max(max_error, error)
        now = time.monotonic()
        if error <= tolerance_rad:
            if hold_started is None:
                hold_started = now
            if now - hold_started >= hold_seconds:
                fresh = connection.wait_for_observation(
                    timeout_s=min(1.0, max(0.01, deadline - now)),
                    newer_than=last_seq,
                    require_motion_gate=True,
                )
                _check_runtime_observation(fresh, max_source_age_s=max_source_age_s)
                return fresh, max_error
        else:
            hold_started = None
        if now >= deadline:
            raise RolloutError(
                f"hold did not remain inside {tolerance_rad:.5f}rad for "
                f"{hold_seconds:.2f}s before the {timeout_s:.1f}s timeout"
            )
        try:
            observation = connection.wait_for_observation(timeout_s=min(0.5, deadline - now), newer_than=last_seq)
            last_seq = observation.seq
        except RolloutError as exc:
            if "timed out waiting" not in str(exc):
                raise
    raise RolloutError("rollout stopped while holding the tracked target")


def _run_synchronized_schedule(
    args: argparse.Namespace,
    connection: RobotConnection,
    policy,
    plan: ActionPlan,
    publisher: ActionPublisher,
    stop: threading.Event,
    episode_deadline: float,
) -> int:
    inference_count = 0
    next_observation = connection.latest(args.max_observation_age)

    while not stop.is_set() and time.monotonic() < episode_deadline:
        observation = next_observation
        _check_runtime_observation(observation, max_source_age_s=args.max_source_age)
        if plan.remaining() != 0:
            raise RolloutError("synchronized scheduler found a non-empty action queue before inference")

        chunk_number = inference_count + 1
        print(f"\nChunk {chunk_number}: fresh observation seq={observation.seq}; inferring...", flush=True)
        actions, timing = infer_actions(policy, observation, args.prompt)
        inference_count += 1
        arrival_observation = connection.latest(args.max_observation_age)
        _check_runtime_observation(arrival_observation, max_source_age_s=args.max_source_age)
        before_append = publisher.snapshot()
        feedback_action = build_state16(
            arrival_observation.joints,
            arrival_observation.gripper_raw_left,
            arrival_observation.gripper_raw_right,
        )
        fallback_anchor = before_append.latched_plan_action or feedback_action
        appended = plan.append_interpolated(
            actions,
            observation.seq,
            args.execute_steps,
            fallback_anchor=fallback_anchor,
            model_hz=args.model_hz,
            playback_time_scale=args.playback_time_scale,
            command_hz=args.control_hz,
        )
        if appended.queued_steps != 0:
            raise RolloutError(f"synchronized scheduler appended behind {appended.queued_steps} queued ticks")

        boundary_delta = _arm_delta(actions[0], appended.anchor_action)
        dispatch_target = before_append.plan_steps_sent + appended.added_steps
        dispatch_timeout = appended.added_steps / args.control_hz + 2.0
        print(
            f"  Inference complete in {timing['wall_ms']:.1f}ms; "
            f"executing {appended.added_steps} targets at {args.control_hz:.1f}Hz."
        )
        print(f"  First-knot delta from held target: {boundary_delta:.5f}rad.")
        dispatched = _wait_for_plan_dispatch(
            connection,
            publisher,
            stop,
            target_plan_steps_sent=dispatch_target,
            timeout_s=dispatch_timeout,
            max_source_age_s=args.max_source_age,
        )
        print("  Chunk dispatched; holding its final target and waiting for arm tracking...", flush=True)
        tracking = _wait_for_target_tracking(
            connection,
            publisher,
            stop,
            target_action=appended.final_action,
            tolerance_rad=args.tracking_tolerance_rad,
            settle_seconds=args.tracking_settle_seconds,
            timeout_s=args.tracking_timeout,
            max_source_age_s=args.max_source_age,
        )
        print(
            f"  Target reached in {tracking.elapsed_s:.2f}s: "
            f"final error={tracking.final_error_rad:.5f}rad "
            f"({tracking.worst_joint}); holding for {args.post_track_hold_seconds:.2f}s."
        )
        next_observation, hold_max_error = _hold_target_and_reobserve(
            connection,
            publisher,
            stop,
            target_action=appended.final_action,
            tolerance_rad=args.tracking_tolerance_rad,
            hold_seconds=args.post_track_hold_seconds,
            timeout_s=args.tracking_timeout,
            max_source_age_s=args.max_source_age,
        )
        after_hold = publisher.snapshot()
        chunk_arm_clipped = after_hold.arm_clipped - before_append.arm_clipped
        print(
            f"  Hold stable; fresh observation seq={next_observation.seq} captured. "
            f"Arm-clipped ticks in this chunk: {chunk_arm_clipped}."
        )
        LOGGER.debug(
            "sync_chunk_diag chunk=%d source_seq=%d arrival_seq=%d reobserve_seq=%d "
            "wall_ms=%.1f added=%d boundary_delta=%.5f dispatch_plan_ticks=%d "
            "track_elapsed=%.3f track_peak_error=%.5f track_final=%.5f "
            "hold_max_error=%.5f arm_clipped_chunk=%d arm_clipped_total=%d",
            chunk_number,
            observation.seq,
            arrival_observation.seq,
            next_observation.seq,
            timing["wall_ms"],
            appended.added_steps,
            boundary_delta,
            dispatched.plan_steps_sent,
            tracking.elapsed_s,
            tracking.max_error_rad,
            tracking.final_error_rad,
            hold_max_error,
            chunk_arm_clipped,
            after_hold.arm_clipped,
        )

    return inference_count


class _CommandIds:
    def __init__(self) -> None:
        self._value = 0

    def next(self) -> int:
        self._value += 1
        return self._value


class _TrajectoryHeartbeat:
    def __init__(self, connection: RobotConnection, session_id: str, stop: threading.Event) -> None:
        self.connection = connection
        self.session_id = session_id
        self.stop = stop
        self.error: BaseException | None = None
        self._version_lock = threading.Lock()
        self._timeline_version: int | None = None
        self._thread = threading.Thread(target=self._run, name="trajectory-heartbeat", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def join(self) -> None:
        self._thread.join(timeout=1.0)

    def update_version(self, timeline_version: int) -> None:
        with self._version_lock:
            self._timeline_version = int(timeline_version)

    def _run(self) -> None:
        try:
            while not self.stop.wait(0.10):
                with self._version_lock:
                    timeline_version = self._timeline_version
                if timeline_version is not None:
                    self.connection.send(TrajectoryHeartbeat(self.session_id, timeline_version))
        except BaseException as exc:
            self.error = exc
            self.stop.set()


def _actions_tuple(actions: np.ndarray) -> tuple[tuple[float, ...], ...]:
    values = np.asarray(actions, dtype=np.float64)
    if values.shape != (RTC_HORIZON, 16) or not np.isfinite(values).all():
        raise RolloutError(f"trajectory actions must have shape ({RTC_HORIZON}, 16), got {values.shape}")
    values = values.copy()
    raw_left = values[:, 7].copy()
    raw_right = values[:, 15].copy()
    values[:, 7] = np.clip(raw_left, 0.0, 1.0)
    values[:, 15] = np.clip(raw_right, 0.0, 1.0)
    left_clipped = int(np.count_nonzero(values[:, 7] != raw_left))
    right_clipped = int(np.count_nonzero(values[:, 15] != raw_right))
    if left_clipped or right_clipped:
        LOGGER.info(
            "trajectory_gripper_projection left_clipped=%d right_clipped=%d "
            "raw_left_range=[%.6f,%.6f] raw_right_range=[%.6f,%.6f]",
            left_clipped,
            right_clipped,
            float(np.min(raw_left)),
            float(np.max(raw_left)),
            float(np.min(raw_right)),
            float(np.max(raw_right)),
        )
    return tuple(tuple(float(value) for value in row) for row in values)


def _load_bridge_trajectory(
    connection: RobotConnection,
    command_ids: _CommandIds,
    *,
    session_id: str,
    plan_id: str,
    expected_timeline_version: int,
    observation_seq: int,
    actions: np.ndarray,
    knot_hz: float,
    checkpoint_horizon: int,
    timeout_s: float,
    continuous_checkpoint: bool = False,
) -> TrajectoryEvent:
    connection.send(
        LoadTrajectoryCommand(
            command_id=command_ids.next(),
            observation_seq=observation_seq,
            session_id=session_id,
            plan_id=plan_id,
            expected_timeline_version=expected_timeline_version,
            knots=_actions_tuple(actions),
            knot_hz=knot_hz,
            checkpoint_horizon=checkpoint_horizon,
            execute=True,
            continuous_checkpoint=continuous_checkpoint,
        )
    )
    event = connection.wait_for_event(
        timeout_s=timeout_s,
        event_types=("trajectory_loaded", "trajectory_command_rejected", "trajectory_stopped"),
    )
    if event.event_type != "trajectory_loaded" or event.plan_id != plan_id:
        raise RolloutError(f"bridge rejected trajectory {plan_id}: {event.detail}")
    return event


def _is_observation_lag_rejection(exc: BaseException) -> bool:
    """Identify a stale-source rejection that can be retried after re-observing."""
    return "action observation lag is" in str(exc)


def _wait_checkpoint_observation(
    connection: RobotConnection,
    event: TrajectoryEvent,
    *,
    timeout_s: float,
    max_source_age_s: float,
    max_state_image_skew_s: float,
) -> RobotObservation:
    if event.stable_monotonic is None:
        raise RolloutError("checkpoint event has no stable timestamp")
    deadline = time.monotonic() + timeout_s
    newer_than = event.observation_seq_at_stable
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RolloutError("timed out waiting for a fresh checkpoint image")
        observation = connection.wait_for_observation(
            timeout_s=remaining,
            newer_than=newer_than,
            require_motion_gate=True,
        )
        newer_than = observation.seq
        if observation.captured_monotonic <= event.stable_monotonic:
            continue
        sampled = observation.extra.get("state_sampled_monotonic")
        if sampled is None or abs(observation.captured_monotonic - float(sampled)) > max_state_image_skew_s:
            continue
        validate_observation(observation, max_source_age_s)
        LOGGER.info(
            "checkpoint_observation event_seq=%d checkpoint=%s image_seq=%d "
            "stable=%.6f captured=%.6f after_stable_ms=%.3f state_sampled=%.6f "
            "state_image_skew_ms=%.3f joint_age_ms=%.3f gripper_proxy=(%.3f,%.3f)",
            event.event_seq,
            event.checkpoint_id,
            observation.seq,
            event.stable_monotonic,
            observation.captured_monotonic,
            (observation.captured_monotonic - event.stable_monotonic) * 1000.0,
            float(sampled),
            abs(observation.captured_monotonic - float(sampled)) * 1000.0,
            float(observation.age_state_s) * 1000.0,
            observation.gripper_raw_left,
            observation.gripper_raw_right,
        )
        return observation


def _wait_bridge_tracking(
    connection: RobotConnection,
    *,
    tolerance_rad: float,
    settle_seconds: float,
    timeout_s: float,
) -> tuple[RobotStateUpdate, float]:
    deadline = time.monotonic() + timeout_s
    stable_source_time: float | None = None
    last_seq: int | None = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RolloutError("timed out waiting for bridge hold tracking")
        state = connection.wait_for_state(timeout_s=remaining, newer_than=last_seq, require_motion_gate=True)
        last_seq = state.state_seq
        if state.raw_reference is None:
            stable_source_time = None
            continue
        error = _arm_delta(state.raw_reference, build_state16(
            state.joints, state.gripper_raw_left, state.gripper_raw_right
        ))
        if error <= tolerance_rad:
            if stable_source_time is None:
                stable_source_time = state.sampled_monotonic
            if state.sampled_monotonic - stable_source_time >= settle_seconds:
                return state, stable_source_time
        else:
            stable_source_time = None


def _hold_bridge_position(
    connection: RobotConnection,
    command_ids: _CommandIds,
    *,
    session_id: str,
    reason: str,
    timeout_s: float,
) -> TrajectoryEvent:
    state = connection.latest_state(max_local_age_s=0.50)
    try:
        refreshed = connection.wait_for_state(
            timeout_s=min(0.50, timeout_s),
            newer_than=state.state_seq,
        )
        if refreshed.session_id == session_id:
            state = refreshed
    except RolloutError as exc:
        if "timed out waiting" not in str(exc):
            raise
    action = state.raw_reference or state.sent_target or build_state16(
        state.joints, state.gripper_raw_left, state.gripper_raw_right
    )
    connection.send(
        HoldPositionCommand(
            command_id=command_ids.next(),
            session_id=session_id,
            expected_timeline_version=state.timeline_version,
            action=tuple(action),
            execute=True,
            reason=reason,
        )
    )
    event = connection.wait_for_event(
        timeout_s=timeout_s,
        event_types=("holding", "trajectory_command_rejected", "trajectory_stopped"),
    )
    if event.event_type != "holding":
        raise RolloutError(f"bridge could not enter fixed hold: {event.detail}")
    return event


def _fresh_observation_after_source_time(
    connection: RobotConnection,
    source_time: float,
    *,
    timeout_s: float,
    max_source_age_s: float,
    max_state_image_skew_s: float,
) -> RobotObservation:
    synthetic = TrajectoryEvent(
        event_seq=-1,
        event_type="local_tracking_ready",
        emitted_monotonic=source_time,
        session_id="",
        plan_id="",
        timeline_version=0,
        phase=0.0,
        stable_monotonic=source_time,
        observation_seq_at_stable=connection.latest().seq,
    )
    return _wait_checkpoint_observation(
        connection,
        synthetic,
        timeout_s=timeout_s,
        max_source_age_s=max_source_age_s,
        max_state_image_skew_s=max_state_image_skew_s,
    )


def _run_bridge_synchronized(
    args: argparse.Namespace,
    connection: RobotConnection,
    policy,
    command_ids: _CommandIds,
    heartbeat: _TrajectoryHeartbeat,
    *,
    session_id: str,
    initial_observation: RobotObservation,
    initial_actions: np.ndarray | None,
    episode_deadline: float,
) -> int:
    inference_count = 0
    observation = initial_observation
    actions = initial_actions
    effective_knot_hz = args.model_hz / args.playback_time_scale
    while time.monotonic() < episode_deadline:
        if heartbeat.error is not None:
            raise RolloutError(f"trajectory heartbeat failed: {heartbeat.error}")
        if actions is None:
            actions, timing = infer_actions(policy, observation, args.prompt)
            inference_count += 1
            LOGGER.info("synchronized fallback inference wall_ms=%.1f", timing["wall_ms"])
        state = connection.latest_state(max_local_age_s=0.50)
        plan_id = f"sync-{uuid.uuid4().hex}"
        try:
            loaded = _load_bridge_trajectory(
                connection,
                command_ids,
                session_id=session_id,
                plan_id=plan_id,
                expected_timeline_version=state.timeline_version,
                observation_seq=observation.seq,
                actions=actions,
                knot_hz=effective_knot_hz,
                checkpoint_horizon=RTC_HORIZON,
                timeout_s=args.tracking_timeout,
            )
        except RolloutError as exc:
            if not _is_observation_lag_rejection(exc):
                raise
            latest = connection.latest(max_local_age_s=0.50)
            LOGGER.warning(
                "discarding synchronized fallback inference because source observation is stale: "
                "source_seq=%d latest_seq=%d lag=%d; re-observing and retrying",
                observation.seq,
                latest.seq,
                latest.seq - observation.seq,
            )
            # The action was conditioned on an observation that the bridge rejected.
            # Keep the bridge's fixed hold and infer again from a fresh image.
            observation = latest
            actions = None
            continue
        heartbeat.update_version(loaded.timeline_version)
        event = connection.wait_for_event(
            timeout_s=args.tracking_timeout + RTC_HORIZON / effective_knot_hz,
            event_types=("checkpoint_ready", "trajectory_stopped", "trajectory_command_rejected"),
        )
        if event.event_type != "checkpoint_ready":
            raise RolloutError(f"synchronized bridge trajectory failed: {event.detail}")
        if time.monotonic() >= episode_deadline:
            break
        observation = _wait_checkpoint_observation(
            connection,
            event,
            timeout_s=args.tracking_timeout,
            max_source_age_s=args.max_source_age,
            max_state_image_skew_s=args.max_state_image_skew,
        )
        actions = None
    return inference_count


def _run_trajectory_schedule(
    args: argparse.Namespace,
    connection: RobotConnection,
    policy,
    observation: RobotObservation,
    warmup_latencies_ms: list[float],
) -> int:
    session_id = uuid.uuid4().hex
    command_ids = _CommandIds()
    heartbeat_stop = threading.Event()
    heartbeat = _TrajectoryHeartbeat(connection, session_id, heartbeat_stop)
    estimator = DelayEstimator()
    for latency_ms in warmup_latencies_ms:
        estimator.record_seconds(latency_ms / 1000.0)
    effective_knot_hz = args.model_hz / args.playback_time_scale
    inference_count = 0
    fallback = args.rollout_schedule == "tracking"

    actions, timing = infer_actions(policy, observation, args.prompt)
    inference_count += 1
    estimator.record_seconds(timing["wall_ms"] / 1000.0)
    LOGGER.info(
        "trajectory_initial_inference observation_seq=%d wall_ms=%.1f policy_timing=%s server_timing=%s",
        observation.seq,
        timing["wall_ms"],
        timing["policy_timing"],
        timing["server_timing"],
    )
    first_plan_id = f"plan-{uuid.uuid4().hex}"
    loaded = _load_bridge_trajectory(
        connection,
        command_ids,
        session_id=session_id,
        plan_id=first_plan_id,
        expected_timeline_version=connection.latest_state(max_local_age_s=0.50).timeline_version,
        observation_seq=observation.seq,
        actions=actions,
        knot_hz=effective_knot_hz,
        checkpoint_horizon=RTC_HORIZON if fallback else RTC_EXECUTION_HORIZON,
        timeout_s=args.tracking_timeout,
        continuous_checkpoint=args.rtc_continuous,
    )
    LOGGER.info(
        "bridge trajectory session=%s plan=%s loaded_version=%d event_seq=%d phase=%.6f",
        session_id,
        first_plan_id,
        loaded.timeline_version,
        loaded.event_seq,
        loaded.phase,
    )
    heartbeat.update_version(loaded.timeline_version)
    heartbeat.start()
    episode_deadline = time.monotonic() + args.episode_seconds
    rtc_merge_count = 0

    try:
        if fallback:
            first_checkpoint = connection.wait_for_event(
                timeout_s=args.tracking_timeout + RTC_HORIZON / effective_knot_hz,
                event_types=("checkpoint_ready", "trajectory_stopped", "trajectory_command_rejected"),
            )
            if first_checkpoint.event_type != "checkpoint_ready":
                raise RolloutError(f"tracking trajectory failed: {first_checkpoint.detail}")
            observation = _wait_checkpoint_observation(
                connection,
                first_checkpoint,
                timeout_s=args.tracking_timeout,
                max_source_age_s=args.max_source_age,
                max_state_image_skew_s=args.max_state_image_skew,
            )
            inference_count += _run_bridge_synchronized(
                args,
                connection,
                policy,
                command_ids,
                heartbeat,
                session_id=session_id,
                initial_observation=observation,
                initial_actions=None,
                episode_deadline=episode_deadline,
            )

        while (
            time.monotonic() < episode_deadline
            or (args.max_rtc_merges is not None and rtc_merge_count >= args.max_rtc_merges)
        ):
            if heartbeat.error is not None:
                raise RolloutError(f"trajectory heartbeat failed: {heartbeat.error}")
            checkpoint = connection.wait_for_event(
                timeout_s=args.tracking_timeout + RTC_EXECUTION_HORIZON / effective_knot_hz,
                event_types=("checkpoint_ready", "trajectory_stopped", "trajectory_command_rejected"),
            )
            if checkpoint.event_type != "checkpoint_ready":
                raise RolloutError(f"RTC checkpoint failed: {checkpoint.detail}")
            if args.max_rtc_merges is not None and rtc_merge_count >= args.max_rtc_merges:
                if args.rtc_continuous:
                    LOGGER.info(
                        "RTC merge limit reached count=%d limit=%d; continuous checkpoint received, holding now",
                        rtc_merge_count,
                        args.max_rtc_merges,
                    )
                    print(
                        f"\nRTC merge limit reached ({rtc_merge_count}); "
                        "current RTC segment reached a continuous checkpoint; holding the target.",
                        flush=True,
                    )
                else:
                    LOGGER.info(
                        "RTC merge limit reached count=%d limit=%d; stable checkpoint received, holding now",
                        rtc_merge_count,
                        args.max_rtc_merges,
                    )
                    print(
                        f"\nRTC merge limit reached ({rtc_merge_count}); "
                        "current RTC segment reached a stable checkpoint; holding the target.",
                        flush=True,
                    )
                break
            observation = _wait_checkpoint_observation(
                connection,
                checkpoint,
                timeout_s=args.tracking_timeout,
                max_source_age_s=args.max_source_age,
                max_state_image_skew_s=args.max_state_image_skew,
            )
            try:
                predicted_delay = estimator.predicted_steps(effective_knot_hz)
                policy_observation = build_policy_observation(observation, args.prompt)
                request_id = uuid.uuid4().hex
                request = build_rtc_request(
                    request_id=request_id,
                    plan_id=checkpoint.plan_id,
                    timeline_version=checkpoint.timeline_version,
                    checkpoint_id=checkpoint.checkpoint_id or 0,
                    observation=policy_observation,
                    old_remaining_actions_absolute=checkpoint.old_remaining_actions_absolute,
                    predicted_delay_steps=predicted_delay,
                )
                result_box: dict[str, object] = {}

                def infer_rtc() -> None:
                    try:
                        result_box["result"] = policy.infer(request)
                    except BaseException as exc:
                        result_box["error"] = exc

                worker = threading.Thread(target=infer_rtc, name=f"rtc-{request_id[:8]}", daemon=True)
                worker.start()
                connection.send(
                    ResumeTrajectoryCommand(
                        command_id=command_ids.next(),
                        session_id=session_id,
                        plan_id=checkpoint.plan_id,
                        timeline_version=checkpoint.timeline_version,
                        checkpoint_id=checkpoint.checkpoint_id or 0,
                        request_id=request_id,
                        predicted_delay_steps=predicted_delay,
                    )
                )
                while worker.is_alive():
                    worker.join(timeout=0.05)
                    if heartbeat.error is not None:
                        raise RolloutError(f"trajectory heartbeat failed: {heartbeat.error}")
                if "error" in result_box:
                    raise RtcError(f"RTC inference transport failed: {result_box['error']}")
                rtc_actions, rtc_timing = parse_rtc_response(result_box["result"], request=request)
                inference_count += 1
                estimator.record_seconds(rtc_timing["wall_ms"] / 1000.0)
                LOGGER.info(
                    "RTC request %s wall_ms=%.1f d_pred=%d",
                    request_id,
                    rtc_timing["wall_ms"],
                    predicted_delay,
                )
                if args.rtc_shadow:
                    shadow_event = connection.wait_for_event(
                        timeout_s=args.tracking_timeout,
                        event_types=(
                            "rtc_waiting_at_deadline",
                            "rtc_invalid",
                            "trajectory_stopped",
                        ),
                    )
                    if shadow_event.event_type != "rtc_waiting_at_deadline":
                        raise RtcError(
                            f"RTC shadow invalidated before merge boundary: "
                            f"{shadow_event.detail or shadow_event.event_type}"
                        )
                    LOGGER.info(
                        "RTC shadow request=%s d_actual=%s d_pred=%d result discarded",
                        request_id,
                        shadow_event.actual_delay_steps,
                        predicted_delay,
                    )
                    raise RtcError(
                        f"RTC shadow result discarded at d_actual={shadow_event.actual_delay_steps}"
                    )
                replacement_plan_id = f"plan-{uuid.uuid4().hex}"
                connection.send(
                    StageRtcChunkCommand(
                        command_id=command_ids.next(),
                        session_id=session_id,
                        base_plan_id=checkpoint.plan_id,
                        replacement_plan_id=replacement_plan_id,
                        timeline_version=checkpoint.timeline_version,
                        checkpoint_id=checkpoint.checkpoint_id or 0,
                        request_id=request_id,
                        predicted_delay_steps=predicted_delay,
                        execution_horizon=RTC_EXECUTION_HORIZON,
                        actions=_actions_tuple(rtc_actions),
                    )
                )
                merged = connection.wait_for_event(
                    timeout_s=args.tracking_timeout,
                    event_types=(
                        "rtc_merged",
                        "rtc_invalid",
                        "trajectory_command_rejected",
                        "trajectory_stopped",
                    ),
                )
                if merged.event_type != "rtc_merged" or merged.request_id != request_id:
                    raise RtcError(f"bridge rejected RTC merge: {merged.detail or merged.event_type}")
                LOGGER.info(
                    "RTC merged request=%s d_actual=%s version=%d",
                    request_id,
                    merged.actual_delay_steps,
                    merged.timeline_version,
                )
                heartbeat.update_version(merged.timeline_version)
                rtc_merge_count += 1
                LOGGER.info("RTC merge count=%d limit=%s", rtc_merge_count, args.max_rtc_merges)
            except Exception as exc:
                LOGGER.warning("RTC disabled for the rest of this episode: %s", exc)
                holding = _hold_bridge_position(
                    connection,
                    command_ids,
                    session_id=session_id,
                    reason=f"RTC fallback: {exc}",
                    timeout_s=args.tracking_timeout,
                )
                heartbeat.update_version(holding.timeline_version)
                _, stable_source_time = _wait_bridge_tracking(
                    connection,
                    tolerance_rad=args.tracking_tolerance_rad,
                    settle_seconds=args.tracking_settle_seconds,
                    timeout_s=args.tracking_timeout,
                )
                observation = _fresh_observation_after_source_time(
                    connection,
                    stable_source_time,
                    timeout_s=args.tracking_timeout,
                    max_source_age_s=args.max_source_age,
                    max_state_image_skew_s=args.max_state_image_skew,
                )
                inference_count += _run_bridge_synchronized(
                    args,
                    connection,
                    policy,
                    command_ids,
                    heartbeat,
                    session_id=session_id,
                    initial_observation=observation,
                    initial_actions=None,
                    episode_deadline=episode_deadline,
                )
                break

        holding = _hold_bridge_position(
            connection,
            command_ids,
            session_id=session_id,
            reason="trajectory rollout complete",
            timeout_s=args.tracking_timeout,
        )
        heartbeat.update_version(holding.timeline_version)
        print("\nROLLOUT COMPLETE")
        print("  Motion is complete; the bridge is holding one fixed target.")
        print("  The client is waiting for the required manual safety handoff.")
        print("  Change Apex Input Mode to None now; the client will then exit.", flush=True)
        deadline = time.monotonic() + args.exit_mode_timeout
        while time.monotonic() < deadline:
            latest_observation = connection.latest()
            if latest_observation.input_mode != 3:
                LOGGER.warning("trajectory_exit_mode_confirmed input_mode=%s", latest_observation.input_mode)
                return inference_count
            time.sleep(0.05)
        raise RolloutError("Input Mode stayed Custom after trajectory rollout")
    finally:
        heartbeat_stop.set()
        heartbeat.join()


def run(args: argparse.Namespace) -> int:
    connection: RobotConnection | None = None
    telemetry: JointTelemetryRecorder | None = None
    stop = threading.Event()
    publisher: ActionPublisher | None = None
    reason = "rollout completed"
    try:
        telemetry_path = getattr(args, "telemetry_file", None)
        if telemetry_path:
            telemetry = JointTelemetryRecorder(Path(telemetry_path))
        print(f"Connecting to robot bridge at {args.robot_host}:{args.robot_port}...")
        connection = RobotConnection(
            args.robot_host,
            args.robot_port,
            args.connect_timeout,
            telemetry=telemetry,
        )
        print(
            "  Bridge connected: "
            f"motion_allowed={connection.hello.motion_allowed}, "
            f"publish_hz={connection.hello.publish_hz:.1f}"
        )
        LOGGER.info(
            "bridge_hello version=%d motion_allowed=%s publish_hz=%.1f max_joint_step_rad=%.5f limits=%d/%d",
            connection.hello.version,
            connection.hello.motion_allowed,
            connection.hello.publish_hz,
            connection.hello.max_joint_step_rad,
            len(connection.hello.joint_lower),
            len(connection.hello.joint_upper),
        )
        observation = connection.wait_for_observation(timeout_s=args.observation_timeout)
        validate_observation(observation, args.max_source_age)
        LOGGER.info(
            "robot_observation_ready seq=%d captured=%.6f input_mode=%s robot_state=%s arm_state=%s "
            "gate=%s reason=%r joint_age_ms=%.3f gripper_proxy=(%.3f,%.3f) source=%s",
            observation.seq,
            observation.captured_monotonic,
            observation.input_mode,
            observation.robot_state,
            observation.arm_state,
            observation.motion_gate_open,
            observation.gate_reason,
            float(observation.age_state_s) * 1000.0,
            observation.gripper_raw_left,
            observation.gripper_raw_right,
            getattr(observation, "extra", {}).get("gripper_state_source", "unknown"),
        )

        if args.execute and not connection.hello.motion_allowed:
            raise RolloutError("bridge motion is disabled; restart robot bridge with --allow-motion")
        if (
            args.execute
            and args.playback_mode == "interpolated"
            and connection.hello.publish_hz < 0.9 * args.control_hz
        ):
            raise RolloutError(
                f"bridge publishes at {connection.hello.publish_hz:.1f}Hz; "
                f"{args.control_hz:.1f}Hz interpolation requires at least "
                f"{0.9 * args.control_hz:.1f}Hz. Restart the bridge with --publish-hz 100"
            )

        print(f"\nConnecting to OpenPI policy at ws://{args.policy_host}:{args.policy_port}...")
        root_logger = logging.getLogger()
        previous_root_level = root_logger.level
        if args.log_level != "DEBUG":
            root_logger.setLevel(logging.WARNING)
        try:
            policy = websocket_client_policy.WebsocketClientPolicy(args.policy_host, args.policy_port)
        finally:
            root_logger.setLevel(previous_root_level)
        LOGGER.info("policy_metadata=%s", policy.get_server_metadata())

        warmup_latencies_ms: list[float] = []
        if args.warmup_inferences:
            print("Warming up policy; warmup actions will not be executed...")
        for index in range(args.warmup_inferences):
            observation = connection.latest(args.max_observation_age)
            validate_observation(observation, args.max_source_age)
            _, timing = infer_actions(policy, observation, args.prompt)
            if args.rollout_schedule != "rtc":
                warmup_latencies_ms.append(float(timing["wall_ms"]))
            LOGGER.debug("discarded warmup inference %d: %s", index + 1, timing)
            print(
                f"  Warmup {index + 1}/{args.warmup_inferences} complete ({timing['wall_ms']:.1f}ms); output discarded."
            )

        if args.rollout_schedule == "rtc":
            metadata = policy.get_server_metadata()
            rtc_metadata = metadata.get("rtc", {}) if isinstance(metadata, dict) else {}
            if rtc_metadata.get("protocol") != "rtc_v1":
                raise RolloutError("policy server does not advertise rtc_v1 support")
            if rtc_metadata.get("execution_horizon") != RTC_EXECUTION_HORIZON:
                raise RolloutError(
                    "policy server RTC execution horizon mismatch: "
                    f"expected {RTC_EXECUTION_HORIZON}, got {rtc_metadata.get('execution_horizon')!r}"
                )
            observation = connection.latest(args.max_observation_age)
            policy_observation = build_policy_observation(observation, args.prompt)
            current = build_state16(
                observation.joints,
                observation.gripper_raw_left,
                observation.gripper_raw_right,
            )
            print("Warming up RTC sampler; outputs will not be executed...")
            for rtc_warmup_index in range(2):
                warmup_request = build_rtc_request(
                    request_id=f"warmup-{uuid.uuid4().hex}",
                    plan_id="warmup",
                    timeline_version=0,
                    checkpoint_id=0,
                    observation=policy_observation,
                    old_remaining_actions_absolute=np.asarray(
                        [current] * (RTC_HORIZON - RTC_EXECUTION_HORIZON)
                    ),
                    predicted_delay_steps=4,
                    warmup=True,
                )
                warmup_result = policy.infer(warmup_request)
                _, rtc_warmup_timing = parse_rtc_response(warmup_result, request=warmup_request)
                if rtc_warmup_index == 1:
                    warmup_latencies_ms.append(float(rtc_warmup_timing["wall_ms"]))
                print(
                    f"  RTC warmup {rtc_warmup_index + 1}/2 complete "
                    f"({rtc_warmup_timing['wall_ms']:.1f}ms); output discarded."
                )

        if args.execute:
            observation = _wait_for_ready(connection, args.ready_timeout)
            observation = _confirm_and_refresh_execution_observation(args, connection, observation)
        else:
            print("\nDRY RUN: policy inference and safety filtering only; no actions will be sent.")

        if args.rollout_schedule in ("tracking", "rtc"):
            inference_count = _run_trajectory_schedule(
                args,
                connection,
                policy,
                observation,
                warmup_latencies_ms,
            )
            print(f"\nTrajectory rollout finished: {inference_count} inferences.")
            LOGGER.info("trajectory_rollout_finished inferences=%d", inference_count)
            return 0

        plan = ActionPlan()
        publisher = ActionPublisher(
            connection,
            plan,
            stop,
            execute=args.execute,
            control_hz=args.control_hz,
            max_joint_step_rad=args.max_joint_step_rad,
            max_observation_age_s=args.max_observation_age,
            joint_limit_margin_rad=args.joint_limit_margin_rad,
            warn_on_plan_empty=(
                args.rollout_schedule == "prefetch"
                and (args.prefetch_steps > 0 if args.playback_mode == "discrete" else args.chunk_prefetch_seconds > 0)
            ),
            refresh_observation_seq=args.playback_mode == "interpolated",
            hold_last_plan_action=args.rollout_schedule == "synchronized",
            telemetry=telemetry,
        )
        publisher.start()
        episode_started = time.monotonic()
        episode_deadline = episode_started + args.episode_seconds
        inference_count = 0
        last_status_command = None
        previous_diagnostic_underruns = publisher.snapshot().underruns
        interpolated_prefetch_steps = math.ceil(args.chunk_prefetch_seconds * args.control_hz)

        if args.rollout_schedule == "synchronized":
            inference_count = _run_synchronized_schedule(
                args,
                connection,
                policy,
                plan,
                publisher,
                stop,
                episode_deadline,
            )
        else:
            while not stop.is_set() and time.monotonic() < episode_deadline:
                observation = connection.latest(args.max_observation_age)
                validate_observation(observation, args.max_source_age)
                if args.execute and not observation.motion_gate_open:
                    raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
                if (
                    args.execute
                    and observation.last_command_id is not None
                    and observation.last_command_id != last_status_command
                ):
                    last_status_command = observation.last_command_id
                    if (
                        observation.last_command_status.startswith("rejected")
                        or "failed" in observation.last_command_status
                    ):
                        raise RolloutError(f"bridge {observation.last_command_status}")

                actions, timing = infer_actions(policy, observation, args.prompt)
                inference_count += 1
                arrival_observation = connection.latest(args.max_observation_age)
                publisher_snapshot = publisher.snapshot()
                feedback_action = build_state16(
                    arrival_observation.joints,
                    arrival_observation.gripper_raw_left,
                    arrival_observation.gripper_raw_right,
                )
                underruns_since_last = publisher_snapshot.underruns - previous_diagnostic_underruns
                previous_diagnostic_underruns = publisher_snapshot.underruns
                LOGGER.debug(
                    "inference=%d seq=%d shape=%s range=[%.5f, %.5f] wall=%.1fms policy=%s",
                    inference_count,
                    observation.seq,
                    tuple(actions.shape),
                    float(actions.min()),
                    float(actions.max()),
                    timing["wall_ms"],
                    timing["policy_timing"],
                )
                if args.playback_mode == "interpolated":
                    fallback_anchor = (
                        feedback_action
                        if publisher_snapshot.last_action is None or publisher_snapshot.last_was_hold
                        else publisher_snapshot.last_action
                    )
                    appended = plan.append_interpolated(
                        actions,
                        observation.seq,
                        args.execute_steps,
                        fallback_anchor=fallback_anchor,
                        model_hz=args.model_hz,
                        playback_time_scale=args.playback_time_scale,
                        command_hz=args.control_hz,
                    )
                    LOGGER.debug(
                        "chunk_append_diag inference=%d source_seq=%d arrival_seq=%d frame_lag=%d "
                        "wall_ms=%.1f queued_before=%d added=%d underruns_since_last=%d "
                        "last_was_hold=%s anchor_to_last=%s new_to_anchor=%s new_to_feedback=%s",
                        inference_count,
                        observation.seq,
                        arrival_observation.seq,
                        arrival_observation.seq - observation.seq,
                        timing["wall_ms"],
                        appended.queued_steps,
                        appended.added_steps,
                        underruns_since_last,
                        publisher_snapshot.last_was_hold,
                        (
                            "n/a"
                            if publisher_snapshot.last_action is None
                            else f"{_arm_delta(appended.anchor_action, publisher_snapshot.last_action):.5f}"
                        ),
                        _candidate_deltas(actions, appended.anchor_action),
                        _candidate_deltas(actions, feedback_action),
                    )
                    _wait_for_plan_threshold(
                        plan,
                        publisher,
                        stop,
                        threshold=interpolated_prefetch_steps,
                        episode_deadline=episode_deadline,
                    )
                else:
                    replacement = plan.replace(actions, observation.seq, args.execute_steps)
                    old_next_to_last = (
                        "n/a"
                        if replacement.old_next_action is None or publisher_snapshot.last_action is None
                        else f"{_arm_delta(replacement.old_next_action, publisher_snapshot.last_action):.5f}"
                    )
                    wall_steps = float(timing["wall_ms"]) * args.control_hz / 1000.0
                    LOGGER.debug(
                        "replan_diag inference=%d source_seq=%d arrival_seq=%d frame_lag=%d "
                        "wall_steps=%.2f discarded=%d underruns_since_last=%d last_was_hold=%s "
                        "old_next_to_last=%s new_to_last=%s new_to_feedback=%s",
                        inference_count,
                        observation.seq,
                        arrival_observation.seq,
                        arrival_observation.seq - observation.seq,
                        wall_steps,
                        replacement.discarded_steps,
                        underruns_since_last,
                        publisher_snapshot.last_was_hold,
                        old_next_to_last,
                        _candidate_deltas(actions, publisher_snapshot.last_action),
                        _candidate_deltas(actions, feedback_action),
                    )
                    _wait_for_plan_threshold(
                        plan,
                        publisher,
                        stop,
                        threshold=args.prefetch_steps,
                        episode_deadline=episode_deadline,
                    )

        if publisher.error is not None:
            raise RolloutError(f"action publisher failed: {publisher.error}")
        episode_snapshot = publisher.snapshot()
        if args.execute:
            _wait_for_none_after_rollout(
                connection,
                publisher,
                plan,
                args.exit_mode_timeout,
            )
        final_snapshot = publisher.snapshot()
        tick_count, late_tick_count, skipped_tick_count, max_tick_gap_s = publisher.timing_snapshot()
        LOGGER.debug(
            "publisher_timing ticks=%d late_ticks=%d skipped_ticks=%d max_gap_ms=%.3f",
            tick_count,
            late_tick_count,
            skipped_tick_count,
            max_tick_gap_s * 1000.0,
        )
        LOGGER.debug(
            "rollout done: inferences=%d episode_action_ticks=%d episode_underruns=%d "
            "episode_clipped_ticks=%d episode_arm_clipped_ticks=%d "
            "episode_gripper_clipped_ticks=%d shutdown_action_ticks=%d shutdown_hold_ticks=%d",
            inference_count,
            episode_snapshot.sent,
            episode_snapshot.underruns,
            episode_snapshot.clipped,
            episode_snapshot.arm_clipped,
            episode_snapshot.gripper_clipped,
            final_snapshot.sent - episode_snapshot.sent,
            final_snapshot.underruns - episode_snapshot.underruns,
        )
        print(
            "\nRollout finished: "
            f"{inference_count} inferences, {episode_snapshot.sent} action ticks, "
            f"{episode_snapshot.arm_clipped} arm-clipped ticks."
        )
        return 0
    except KeyboardInterrupt:
        reason = "operator interrupted rollout"
        LOGGER.warning(reason)
        return 130
    except (ConnectionError, OSError, ProtocolError, RolloutError, SafetyError) as exc:
        reason = f"rollout aborted: {exc}"
        LOGGER.error(reason)
        return 1
    except Exception as exc:
        reason = f"rollout aborted by unexpected error: {exc}"
        LOGGER.exception(reason)
        return 1
    finally:
        stop.set()
        if publisher is not None:
            publisher.plan.clear()
            publisher.join(timeout=1.0)
        if connection is not None:
            connection.close(reason)
        if telemetry is not None:
            telemetry.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default=DEFAULT_BRIDGE_HOST)
    parser.add_argument("--robot-port", type=int, default=DEFAULT_BRIDGE_PORT)
    parser.add_argument("--policy-host", default=DEFAULT_POLICY_HOST)
    parser.add_argument("--policy-port", type=int, default=DEFAULT_POLICY_PORT)
    parser.add_argument("--prompt", default=DEFAULT_PROMPT)
    parser.add_argument("--execute", action="store_true", help="send actions to the bridge; default is dry-run")
    parser.add_argument("--yes", action="store_true", help="skip the typed E confirmation")
    parser.add_argument("--episode-seconds", type=float, default=60.0)
    parser.add_argument(
        "--rollout-schedule",
        choices=("prefetch", "synchronized", "tracking", "rtc"),
        default="prefetch",
        help="prefetch chunks while moving, or infer only after the previous target is tracked and held",
    )
    parser.add_argument(
        "--playback-mode",
        choices=("discrete", "interpolated"),
        default="discrete",
        help="legacy knot-at-a-time playback or continuous full-segment interpolation",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=CONTROL_HZ,
        help="action publisher rate; use 100 with interpolated playback",
    )
    parser.add_argument(
        "--model-hz",
        type=float,
        default=CONTROL_HZ,
        help="time semantics of policy action knots",
    )
    parser.add_argument(
        "--playback-time-scale",
        type=float,
        default=1.0,
        help="stretch interpolated policy time by this factor; values below 1 are rejected",
    )
    parser.add_argument("--execute-steps", type=int, default=5)
    parser.add_argument(
        "--prefetch-steps",
        type=int,
        default=3,
        help="legacy discrete mode: replan when this many queued knots remain",
    )
    parser.add_argument(
        "--chunk-prefetch-seconds",
        type=float,
        default=0.30,
        help="interpolated mode: infer the next segment with this much queued motion left",
    )
    parser.add_argument(
        "--tracking-tolerance-rad",
        type=float,
        default=0.01,
        help="synchronized mode: maximum error on every arm joint before a target is reached",
    )
    parser.add_argument(
        "--tracking-settle-seconds",
        type=float,
        default=0.20,
        help="synchronized mode: time all arm joints must remain inside the tracking tolerance",
    )
    parser.add_argument(
        "--post-track-hold-seconds",
        type=float,
        default=0.20,
        help="synchronized mode: additional stable hold before capturing the next observation",
    )
    parser.add_argument(
        "--tracking-timeout",
        type=float,
        default=5.0,
        help="synchronized mode: maximum time for either tracking or stable hold",
    )
    parser.add_argument("--warmup-inferences", type=int, default=1)
    parser.add_argument("--rtc-shadow", action="store_true", help="run one RTC request, discard it, then use sync fallback")
    parser.add_argument(
        "--rtc-continuous",
        action="store_true",
        help=f"rtc mode: observe at action {RTC_EXECUTION_HORIZON} without pausing for checkpoint settle",
    )
    parser.add_argument(
        "--max-rtc-merges",
        type=int,
        help=(
            "rtc mode: after this many successful replacement merges, stop at the "
            "next checkpoint and hold the current target"
        ),
    )
    parser.add_argument("--max-state-image-skew", type=float, default=0.05)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.16)
    parser.add_argument("--joint-limit-margin-rad", type=float, default=0.02)
    parser.add_argument("--max-source-age", type=float, default=0.20)
    parser.add_argument("--max-observation-age", type=float, default=0.35)
    parser.add_argument("--observation-timeout", type=float, default=10.0)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument(
        "--exit-mode-timeout",
        type=float,
        default=30.0,
        help="seconds to hold after the episode while waiting for Input Mode None",
    )
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
        help="structured log-file detail level; also used for the console when no log file is set",
    )
    parser.add_argument(
        "--console-log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        help="terminal log level; defaults to WARNING with --log-file, otherwise --log-level",
    )
    parser.add_argument("--log-file", help="write structured rollout diagnostics to this local file")
    parser.add_argument(
        "--telemetry-file",
        help=(
            "write full-rate measured joints and interpolated commands as CSV; "
            "defaults to <log-file stem>.telemetry.csv"
        ),
    )
    args = parser.parse_args(argv)
    if (
        args.episode_seconds <= 0
        or args.control_hz <= 0
        or args.model_hz <= 0
        or args.playback_time_scale <= 0
        or args.tracking_tolerance_rad <= 0
        or args.tracking_settle_seconds < 0
        or args.post_track_hold_seconds < 0
        or args.tracking_timeout <= 0
        or args.exit_mode_timeout <= 0
    ):
        parser.error("episode duration and control rate must be positive")
    if not 1 <= args.execute_steps <= 10:
        parser.error("--execute-steps must be in [1, 10] for this checkpoint")
    if args.playback_mode == "discrete" and not 0 <= args.prefetch_steps < args.execute_steps:
        parser.error("--prefetch-steps must be >=0 and smaller than --execute-steps")
    if args.chunk_prefetch_seconds < 0:
        parser.error("--chunk-prefetch-seconds cannot be negative")
    if args.playback_time_scale < 1.0:
        parser.error("--playback-time-scale must be at least 1.0")
    if args.playback_mode == "discrete" and args.playback_time_scale != 1.0:
        parser.error("--playback-time-scale is only valid with --playback-mode interpolated")
    if args.playback_mode == "interpolated":
        effective_knot_hz = args.model_hz / args.playback_time_scale
        chunk_seconds = args.execute_steps / effective_knot_hz
        if args.control_hz < effective_knot_hz:
            parser.error("--control-hz must be at least the effective policy knot rate")
        if args.rollout_schedule == "prefetch" and args.chunk_prefetch_seconds >= chunk_seconds:
            parser.error("--chunk-prefetch-seconds must be shorter than the selected chunk duration")
    if args.rollout_schedule == "synchronized":
        if args.playback_mode != "interpolated":
            parser.error("--rollout-schedule synchronized requires --playback-mode interpolated")
        if not args.execute:
            parser.error("--rollout-schedule synchronized requires --execute")
    if args.rollout_schedule in ("tracking", "rtc"):
        if not args.execute:
            parser.error(f"--rollout-schedule {args.rollout_schedule} requires --execute")
        if args.playback_mode != "interpolated":
            parser.error(f"--rollout-schedule {args.rollout_schedule} requires --playback-mode interpolated")
        if (
            args.control_hz != 100.0
            or args.model_hz != 15.0
            or args.playback_time_scale != _TRACKING_PLAYBACK_TIME_SCALE
        ):
            parser.error(
                "tracking/rtc requires --control-hz 100 --model-hz 15 "
                "--playback-time-scale 3 (fixed 5 Hz knot rate)"
            )
        if args.execute_steps != RTC_HORIZON:
            parser.error("tracking/rtc requires --execute-steps 10")
    if args.rtc_shadow and args.rollout_schedule != "rtc":
        parser.error("--rtc-shadow requires --rollout-schedule rtc")
    if args.rtc_continuous and args.rollout_schedule != "rtc":
        parser.error("--rtc-continuous requires --rollout-schedule rtc")
    if args.max_rtc_merges is not None:
        if args.max_rtc_merges < 1:
            parser.error("--max-rtc-merges must be positive")
        if args.rollout_schedule != "rtc":
            parser.error("--max-rtc-merges requires --rollout-schedule rtc")
    if args.max_state_image_skew <= 0:
        parser.error("--max-state-image-skew must be positive")
    if args.warmup_inferences < 0:
        parser.error("--warmup-inferences cannot be negative")
    if args.yes and not args.execute:
        parser.error("--yes is only meaningful with --execute")
    return args


def _configure_logging(args: argparse.Namespace, argv: list[str] | None) -> None:
    if args.telemetry_file:
        args.telemetry_file = str(Path(args.telemetry_file).expanduser().resolve())
    elif args.log_file:
        log_path = Path(args.log_file).expanduser().resolve()
        args.telemetry_file = str(log_path.with_name(f"{log_path.stem}.telemetry.csv"))
    console_level_name = args.console_log_level or ("WARNING" if args.log_file else args.log_level)
    console_level = getattr(logging, console_level_name)
    file_level = getattr(logging, args.log_level)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(console_level)
    handlers: list[logging.Handler] = [console_handler]
    if args.log_file:
        log_path = Path(args.log_file).expanduser().resolve()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(log_path, encoding="utf-8")
        file_handler.setLevel(file_level)
        handlers.append(file_handler)
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s [%(threadName)s]: %(message)s",
        handlers=handlers,
        force=True,
    )
    LOGGER.setLevel(min(console_level, file_level if args.log_file else console_level))
    effective_argv = sys.argv[1:] if argv is None else argv
    LOGGER.info(
        "rollout_start argv=%s source=%s console_log_level=%s file_log_level=%s config=%s",
        shlex.join(["python", "-m", "marvinpro_deploy.rollout_client", *effective_argv]),
        Path(__file__).resolve(),
        console_level_name,
        args.log_level if args.log_file else None,
        vars(args),
    )
    if args.log_file:
        print(f"Structured rollout log: {Path(args.log_file).expanduser().resolve()}")
    if args.telemetry_file:
        print(f"Joint telemetry log: {args.telemetry_file}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    _configure_logging(args, argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
