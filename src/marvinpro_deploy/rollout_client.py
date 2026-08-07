"""Marvin Pro rollout client for an OpenPI WebSocket policy server."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import logging
import math
import socket
import threading
import time

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
from .protocol import (
    ActionCommand,
    BridgeHello,
    ProtocolError,
    RobotObservation,
    StopCommand,
    recv_message,
    require_current_version,
    send_message,
)
from .safety import SafetyError, action_arms, filter_action

LOGGER = logging.getLogger("marvinpro_rollout")


class RolloutError(RuntimeError):
    pass


class RobotConnection:
    def __init__(self, host: str, port: int, connect_timeout_s: float = 5.0) -> None:
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
        self._latest: RobotObservation | None = None
        self._latest_received = 0.0
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
                if not isinstance(message, RobotObservation):
                    raise ProtocolError(f"unexpected bridge message {type(message).__name__}")
                require_current_version(message)
                with self._condition:
                    self._latest = message
                    self._latest_received = time.monotonic()
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
            if self._latest is None:
                raise RolloutError("no robot observation received")
            if max_local_age_s is not None and time.monotonic() - self._latest_received > max_local_age_s:
                raise RolloutError("latest robot observation is stale on the rollout client")
            return self._latest

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
                observation = self._latest
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
        with self._condition:
            if self._error is not None:
                raise RolloutError(f"robot bridge receive failed: {self._error}")
        send_message(self._socket, command, self._send_lock)

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

    def _run(self) -> None:
        command_id = 0
        next_tick = time.monotonic()
        last_underrun_log = 0.0
        last_arm_clip_log = 0.0
        last_gripper_clip_log = 0.0
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if now < next_tick:
                    self.stop.wait(next_tick - now)
                    continue
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
                    LOGGER.warning("safety filter clipped action dimensions %s", filtered.clipped_indices)
                    last_arm_clip_log = now
                elif gripper_was_clipped and now - last_gripper_clip_log >= 2.0:
                    LOGGER.debug(
                        "safety filter is repeatedly clamping gripper dimensions %s",
                        filtered.clipped_indices,
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
    ages = {
        "joint state": observation.age_state_s,
        "left gripper": observation.age_gripper_left_s,
        "right gripper": observation.age_gripper_right_s,
    }
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
    return observation


def _confirm_execution(args: argparse.Namespace, observation: RobotObservation) -> None:
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
        else:
            print(f"  next-chunk inference lead: {args.chunk_prefetch_seconds:.2f}s")
    print("Keep the emergency stop reachable. Switch Input Mode to None before stopping the bridge.")
    if args.yes:
        return
    answer = input('Type exactly "EXECUTE" to start motion: ')
    if answer != "EXECUTE":
        raise RolloutError("execution confirmation was not given")


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


def run(args: argparse.Namespace) -> int:
    connection: RobotConnection | None = None
    stop = threading.Event()
    publisher: ActionPublisher | None = None
    reason = "rollout completed"
    try:
        print(f"Connecting to robot bridge at {args.robot_host}:{args.robot_port}...")
        connection = RobotConnection(args.robot_host, args.robot_port, args.connect_timeout)
        print(
            "  Bridge connected: "
            f"motion_allowed={connection.hello.motion_allowed}, "
            f"publish_hz={connection.hello.publish_hz:.1f}"
        )
        observation = connection.wait_for_observation(timeout_s=args.observation_timeout)
        validate_observation(observation, args.max_source_age)
        LOGGER.debug(
            "robot observation ready: seq=%d input_mode=%s robot_state=%s arm_state=%s gate=%s (%s)",
            observation.seq,
            observation.input_mode,
            observation.robot_state,
            observation.arm_state,
            observation.motion_gate_open,
            observation.gate_reason,
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
        LOGGER.debug("policy metadata: %s", policy.get_server_metadata())

        if args.warmup_inferences:
            print("Warming up policy; warmup actions will not be executed...")
        for index in range(args.warmup_inferences):
            observation = connection.latest(args.max_observation_age)
            validate_observation(observation, args.max_source_age)
            _, timing = infer_actions(policy, observation, args.prompt)
            LOGGER.debug("discarded warmup inference %d: %s", index + 1, timing)
            print(
                f"  Warmup {index + 1}/{args.warmup_inferences} complete ({timing['wall_ms']:.1f}ms); output discarded."
            )

        if args.execute:
            observation = _wait_for_ready(connection, args.ready_timeout)
            _confirm_execution(args, observation)
        else:
            print("\nDRY RUN: policy inference and safety filtering only; no actions will be sent.")

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


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default=DEFAULT_BRIDGE_HOST)
    parser.add_argument("--robot-port", type=int, default=DEFAULT_BRIDGE_PORT)
    parser.add_argument("--policy-host", default=DEFAULT_POLICY_HOST)
    parser.add_argument("--policy-port", type=int, default=DEFAULT_POLICY_PORT)
    parser.add_argument("--prompt", default=DEFAULT_PROMPT)
    parser.add_argument("--execute", action="store_true", help="send actions to the bridge; default is dry-run")
    parser.add_argument("--yes", action="store_true", help="skip the typed EXECUTE confirmation")
    parser.add_argument("--episode-seconds", type=float, default=60.0)
    parser.add_argument(
        "--rollout-schedule",
        choices=("prefetch", "synchronized"),
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
    parser.add_argument("--max-joint-step-rad", type=float, default=0.08)
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
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
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
    if args.warmup_inferences < 0:
        parser.error("--warmup-inferences cannot be negative")
    if args.yes and not args.execute:
        parser.error("--yes is only meaningful with --execute")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    LOGGER.setLevel(getattr(logging, args.log_level))
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
