"""Marvin Pro rollout client for an OpenPI WebSocket policy server."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import logging
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
)
from .image_processing import ImageError, decode_and_split
from .joint_mapping import build_state16
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
from .safety import SafetyError, filter_action

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
                is_new = observation is not None and (
                    newer_than is None or observation.seq > newer_than
                )
                gate_ok = observation is not None and (
                    not require_motion_gate or observation.motion_gate_open
                )
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


class ActionPlan:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._steps: deque[PlanStep] = deque()

    def replace(self, actions: np.ndarray, observation_seq: int, execute_steps: int) -> None:
        steps = [
            PlanStep(tuple(float(value) for value in row), observation_seq)
            for row in np.asarray(actions)[:execute_steps]
        ]
        with self._condition:
            self._steps = deque(steps)
            self._condition.notify_all()

    def pop(self) -> PlanStep | None:
        with self._condition:
            if not self._steps:
                return None
            step = self._steps.popleft()
            self._condition.notify_all()
            return step

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
    ) -> None:
        self.connection = connection
        self.plan = plan
        self.stop = stop
        self.execute = execute
        self.period_s = 1.0 / control_hz
        self.max_joint_step_rad = max_joint_step_rad
        self.max_observation_age_s = max_observation_age_s
        self.joint_limit_margin_rad = joint_limit_margin_rad
        self.error: BaseException | None = None
        self.sent = 0
        self.underruns = 0
        self.clipped = 0
        self._thread = threading.Thread(target=self._run, name="action-publisher", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        command_id = 0
        next_tick = time.monotonic()
        last_underrun_log = 0.0
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if now < next_tick:
                    self.stop.wait(next_tick - now)
                    continue
                while next_tick <= now:
                    next_tick += self.period_s

                step = self.plan.pop()
                if step is None:
                    self.underruns += 1
                    if now - last_underrun_log >= 2.0:
                        LOGGER.warning("action plan empty; commanding measured-pose hold")
                        last_underrun_log = now

                observation = self.connection.latest(self.max_observation_age_s)
                if self.execute and not observation.motion_gate_open:
                    raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
                if step is None:
                    if not self.execute:
                        continue
                    step = PlanStep(
                        action=build_state16(
                            observation.joints,
                            observation.gripper_raw_left,
                            observation.gripper_raw_right,
                        ),
                        observation_seq=observation.seq,
                    )
                filtered = filter_action(
                    step.action,
                    observation.joints,
                    max_joint_step_rad=self.max_joint_step_rad,
                    joint_limit_margin_rad=self.joint_limit_margin_rad,
                )
                if filtered.clipped_indices:
                    self.clipped += 1
                    LOGGER.warning(
                        "safety filter clipped action dimensions %s", filtered.clipped_indices
                    )
                command_id += 1
                if self.execute:
                    self.connection.send_action(
                        ActionCommand(
                            command_id=command_id,
                            observation_seq=step.observation_seq,
                            action=filtered.action,
                            execute=True,
                        )
                    )
                self.sent += 1
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
    LOGGER.info("waiting for bridge motion gate (Custom mode and both state arrays [3, 3])")
    return connection.wait_for_observation(timeout_s=timeout_s, require_motion_gate=True)


def _confirm_execution(args: argparse.Namespace, observation: RobotObservation) -> None:
    print("\nREAL ROBOT EXECUTION REQUESTED")
    print(f"  prompt: {args.prompt}")
    print(f"  input_mode: {observation.input_mode}")
    print(f"  robot_state: {observation.robot_state}")
    print(f"  arm_state: {observation.arm_state}")
    print(f"  duration: {args.episode_seconds:.1f}s")
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
    plan.clear()
    LOGGER.warning(
        "episode actions finished; holding measured pose. Switch Apex Input Mode to None now"
    )
    deadline = time.monotonic() + timeout_s
    last_seq = -1
    while True:
        observation = connection.latest()
        if observation.input_mode != 3:
            LOGGER.info("input_mode=%s; safe to disconnect rollout client", observation.input_mode)
            return
        if publisher.error is not None:
            raise RolloutError(f"action publisher failed while waiting for Input Mode None: {publisher.error}")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RolloutError(
                f"Input Mode stayed Custom for {timeout_s:.1f}s after rollout; disconnecting via watchdog"
            )
        try:
            observation = connection.wait_for_observation(
                timeout_s=min(1.0, remaining), newer_than=last_seq
            )
            last_seq = observation.seq
        except RolloutError as exc:
            if "timed out waiting" not in str(exc):
                raise


def run(args: argparse.Namespace) -> int:
    connection: RobotConnection | None = None
    stop = threading.Event()
    publisher: ActionPublisher | None = None
    reason = "rollout completed"
    try:
        LOGGER.info("connecting to robot bridge at %s:%d", args.robot_host, args.robot_port)
        connection = RobotConnection(args.robot_host, args.robot_port, args.connect_timeout)
        LOGGER.info(
            "bridge: motion_allowed=%s publish_hz=%.1f max_step=%.3frad",
            connection.hello.motion_allowed,
            connection.hello.publish_hz,
            connection.hello.max_joint_step_rad,
        )
        observation = connection.wait_for_observation(timeout_s=args.observation_timeout)
        validate_observation(observation, args.max_source_age)
        LOGGER.info(
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

        LOGGER.info("connecting to OpenPI policy at ws://%s:%d", args.policy_host, args.policy_port)
        policy = websocket_client_policy.WebsocketClientPolicy(args.policy_host, args.policy_port)
        LOGGER.info("policy metadata: %s", policy.get_server_metadata())

        for index in range(args.warmup_inferences):
            observation = connection.latest(args.max_observation_age)
            validate_observation(observation, args.max_source_age)
            _, timing = infer_actions(policy, observation, args.prompt)
            LOGGER.info("discarded warmup inference %d: %s", index + 1, timing)

        if args.execute:
            observation = _wait_for_ready(connection, args.ready_timeout)
            _confirm_execution(args, observation)
        else:
            LOGGER.info("DRY RUN: policy inference and safety filtering only; no actions will be sent")

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
        )
        publisher.start()
        episode_started = time.monotonic()
        inference_count = 0
        last_status_command = None

        while not stop.is_set() and time.monotonic() - episode_started < args.episode_seconds:
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
                if observation.last_command_status.startswith("rejected") or "failed" in observation.last_command_status:
                    raise RolloutError(f"bridge {observation.last_command_status}")

            actions, timing = infer_actions(policy, observation, args.prompt)
            inference_count += 1
            plan.replace(actions, observation.seq, args.execute_steps)
            LOGGER.info(
                "inference=%d seq=%d shape=%s range=[%.5f, %.5f] wall=%.1fms policy=%s",
                inference_count,
                observation.seq,
                tuple(actions.shape),
                float(actions.min()),
                float(actions.max()),
                timing["wall_ms"],
                timing["policy_timing"],
            )
            while plan.remaining() > args.prefetch_steps and not stop.is_set():
                plan.wait_until_at_most(args.prefetch_steps, stop)
                if publisher.error is not None:
                    raise RolloutError(f"action publisher failed: {publisher.error}")

        if publisher.error is not None:
            raise RolloutError(f"action publisher failed: {publisher.error}")
        if args.execute:
            _wait_for_none_after_rollout(
                connection,
                publisher,
                plan,
                args.exit_mode_timeout,
            )
        LOGGER.info(
            "rollout done: inferences=%d action_ticks=%d underruns=%d clipped_ticks=%d",
            inference_count,
            publisher.sent,
            publisher.underruns,
            publisher.clipped,
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
    parser.add_argument("--control-hz", type=float, default=CONTROL_HZ)
    parser.add_argument("--execute-steps", type=int, default=5)
    parser.add_argument("--prefetch-steps", type=int, default=3)
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
    if args.episode_seconds <= 0 or args.control_hz <= 0 or args.exit_mode_timeout <= 0:
        parser.error("episode duration and control rate must be positive")
    if not 1 <= args.execute_steps <= 10:
        parser.error("--execute-steps must be in [1, 10] for this checkpoint")
    if not 0 <= args.prefetch_steps < args.execute_steps:
        parser.error("--prefetch-steps must be >=0 and smaller than --execute-steps")
    if args.warmup_inferences < 0:
        parser.error("--warmup-inferences cannot be negative")
    if args.yes and not args.execute:
        parser.error("--yes is only meaningful with --execute")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
