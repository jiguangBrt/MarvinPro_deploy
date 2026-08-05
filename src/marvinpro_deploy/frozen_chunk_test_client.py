"""A/B playback of one saved OpenPI action chunk without replanning.

Capture mode calls the policy once while Apex Input Mode is None and saves a
bounded JSON plan. Replay the same plan first as discrete 15 Hz targets, then
from the same anchor pose with 100 Hz linear interpolation.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import time

import numpy as np
from openpi_client import websocket_client_policy

from .config import (
    DEFAULT_BRIDGE_HOST,
    DEFAULT_BRIDGE_PORT,
    DEFAULT_POLICY_HOST,
    DEFAULT_POLICY_PORT,
    DEFAULT_PROMPT,
    JOINT_NAMES,
)
from .joint_mapping import build_state16
from .motion_profile import FrozenLinearPlan, minimum_jerk_blend
from .protocol import ActionCommand, ProtocolError
from .rollout_client import RobotConnection, RolloutError, infer_actions, validate_observation
from .safety import SafetyError, action_arms, filter_action, validate_action

LOGGER = logging.getLogger("marvinpro_frozen_chunk_test")
PLAN_VERSION = 2


@dataclass(frozen=True)
class ChunkStats:
    max_velocity: float
    max_acceleration: float


@dataclass(frozen=True)
class ChunkSample:
    captured_monotonic: float
    observed_joints: tuple[float, ...]
    commanded_joints: tuple[float, ...]


def _check_command_feedback(observation) -> None:
    status = observation.last_command_status
    if status.startswith("rejected") or "failed" in status:
        raise RolloutError(f"bridge {status}")


def _chunk_stats(knots: tuple[tuple[float, ...], ...], model_hz: float) -> ChunkStats:
    arm_knots = np.asarray([action_arms(knot) for knot in knots], dtype=np.float64)
    velocities = np.diff(arm_knots, axis=0) * model_hz
    max_velocity = float(np.max(np.abs(velocities)))
    if len(velocities) >= 2:
        accelerations = np.diff(velocities, axis=0) * model_hz
        max_acceleration = float(np.max(np.abs(accelerations)))
    else:
        max_acceleration = 0.0
    return ChunkStats(max_velocity, max_acceleration)


def _prepare_knots(
    actions: np.ndarray,
    anchor_action: tuple[float, ...],
    anchor_joints: tuple[float, ...],
    args: argparse.Namespace,
) -> tuple[tuple[tuple[float, ...], ...], tuple[tuple[float, ...], ...], int]:
    raw: list[tuple[float, ...]] = []
    prepared: list[tuple[float, ...]] = []
    clipped_values = 0
    for row in np.asarray(actions, dtype=np.float64)[: args.chunk_steps]:
        candidate = [float(value) for value in row]
        candidate[7] = anchor_action[7]
        candidate[15] = anchor_action[15]
        raw.append(tuple(candidate))
        filtered = filter_action(
            candidate,
            anchor_joints,
            max_joint_step_rad=args.max_model_offset_rad,
            joint_limit_margin_rad=args.joint_limit_margin_rad,
        )
        clipped_values += sum(index not in (7, 15) for index in filtered.clipped_indices)
        prepared.append(filtered.action)
    return (anchor_action, *raw), (anchor_action, *prepared), clipped_values


def _write_plan(
    path: Path,
    *,
    raw_knots: tuple[tuple[float, ...], ...],
    playback_knots: tuple[tuple[float, ...], ...],
    model_hz: float,
    prompt: str,
    clipped_values: int,
    overwrite: bool,
) -> None:
    payload = {
        "version": PLAN_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "prompt": prompt,
        "model_hz": model_hz,
        "clipped_values": clipped_values,
        "raw_knots": [list(knot) for knot in raw_knots],
        "playback_knots": [list(knot) for knot in playback_knots],
    }
    mode = "w" if overwrite else "x"
    try:
        with path.open(mode, encoding="utf-8") as output:
            json.dump(payload, output, indent=2, ensure_ascii=False)
            output.write("\n")
    except FileExistsError as exc:
        raise RolloutError(f"plan file already exists: {path}; choose another path") from exc
    except OSError as exc:
        raise RolloutError(f"cannot write plan file {path}: {exc}") from exc


def _read_plan(
    path: Path,
) -> tuple[
    tuple[tuple[float, ...], ...],
    tuple[tuple[float, ...], ...],
    float,
    int,
    str,
]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RolloutError(f"cannot read plan file {path}: {exc}") from exc
    if payload.get("version") != PLAN_VERSION:
        raise RolloutError(f"unsupported frozen plan version: {payload.get('version')}")
    try:
        model_hz = float(payload["model_hz"])
        clipped_values = int(payload.get("clipped_values", 0))
        prompt = str(payload.get("prompt", ""))
        raw_knots = tuple(tuple(float(value) for value in knot) for knot in payload["raw_knots"])
        playback_knots = tuple(
            tuple(float(value) for value in knot) for knot in payload["playback_knots"]
        )
        FrozenLinearPlan(raw_knots, model_hz)
        FrozenLinearPlan(playback_knots, model_hz)
    except (KeyError, TypeError, ValueError) as exc:
        raise RolloutError(f"invalid frozen plan {path}: {exc}") from exc
    if any(len(knot) != 16 for knot in (*raw_knots, *playback_knots)):
        raise RolloutError("frozen plan actions must have 16 values")
    if len(raw_knots) != len(playback_knots):
        raise RolloutError("raw and playback frozen plans have different lengths")
    return raw_knots, playback_knots, model_hz, clipped_values, prompt


def _replace_grippers(
    knots: tuple[tuple[float, ...], ...], current_action: tuple[float, ...]
) -> tuple[tuple[float, ...], ...]:
    result = []
    for knot in knots:
        values = list(knot)
        values[7] = current_action[7]
        values[15] = current_action[15]
        result.append(tuple(values))
    return tuple(result)


def _confirm(
    args: argparse.Namespace,
    observation,
    *,
    raw_stats: ChunkStats,
    playback_stats: ChunkStats,
    clipped_values: int,
    plan_duration: float,
    model_hz: float,
    prompt: str,
    anchor_drift: float,
    bridge_publish_hz: float,
    inference_wall_ms: float | None,
) -> None:
    print("\nFROZEN POLICY CHUNK A/B TEST")
    print(f"  playback mode: {args.playback_mode}")
    print(f"  plan file: {args.capture_plan or args.load_plan}")
    print(f"  prompt: {prompt}")
    print(f"  input_mode: {observation.input_mode}")
    print(f"  robot_state: {observation.robot_state}")
    print(f"  arm_state: {observation.arm_state}")
    print(f"  bridge publish rate: {bridge_publish_hz:.1f}Hz")
    print(f"  model knot rate: {model_hz:.1f}Hz")
    playback_hz = model_hz if args.playback_mode == "discrete" else args.command_hz
    print(f"  playback target update rate: {playback_hz:.1f}Hz")
    print(f"  interpolation/return command rate: {args.command_hz:.1f}Hz")
    print(f"  playback duration: {plan_duration:.3f}s")
    if inference_wall_ms is not None:
        print(f"  policy inference wall time: {inference_wall_ms:.1f}ms")
    print(f"  max pose drift from saved anchor: {anchor_drift:.6f}rad")
    print(f"  raw policy max velocity: {raw_stats.max_velocity:.5f}rad/s")
    print(f"  raw policy max acceleration: {raw_stats.max_acceleration:.5f}rad/s^2")
    print(f"  bounded playback max velocity: {playback_stats.max_velocity:.5f}rad/s")
    print(f"  bounded playback max acceleration: {playback_stats.max_acceleration:.5f}rad/s^2")
    print(f"  arm knot values clipped during capture: {clipped_values}")
    print("  grippers stay fixed; no policy replanning occurs.")
    print(f"  after playback the robot automatically returns to the anchor in {args.return_seconds:.1f}s.")
    print("  keep the emergency stop reachable; switch Input Mode to None when prompted.")
    if args.yes:
        return
    expected = "DISCRETE" if args.playback_mode == "discrete" else "INTERPOLATE"
    if input(f'Type exactly "{expected}" to begin: ') != expected:
        raise RolloutError("frozen chunk confirmation was not given")


def _ready_observation(connection: RobotConnection, args: argparse.Namespace):
    observation = connection.latest(args.max_observation_age)
    if observation.input_mode != 3:
        raise RolloutError("Input Mode left Custom before playback and return completed")
    if not observation.motion_gate_open:
        raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
    validate_observation(observation, args.max_source_age)
    _check_command_feedback(observation)
    return observation


def _send_target(
    connection: RobotConnection,
    args: argparse.Namespace,
    command_id: int,
    target: tuple[float, ...],
):
    observation = _ready_observation(connection, args)
    connection.send_action(ActionCommand(command_id, observation.seq, target, execute=True))
    return observation


def _record_sample(
    samples: list[ChunkSample],
    observation,
    target: tuple[float, ...],
    last_seq: int,
) -> int:
    if observation.seq == last_seq:
        return last_seq
    samples.append(
        ChunkSample(
            captured_monotonic=observation.captured_monotonic,
            observed_joints=observation.joints,
            commanded_joints=action_arms(target),
        )
    )
    return observation.seq


def _settle_at_anchor(
    connection: RobotConnection,
    args: argparse.Namespace,
    command_id: int,
    anchor: tuple[float, ...],
) -> int:
    period = 1.0 / args.command_hz
    deadline = time.monotonic() + args.settle_seconds
    next_tick = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(next_tick - now)
        command_id += 1
        _send_target(connection, args, command_id, anchor)
        next_tick += period
        if next_tick < time.monotonic():
            next_tick = time.monotonic() + period
    return command_id


def _play_discrete(
    connection: RobotConnection,
    args: argparse.Namespace,
    knots: tuple[tuple[float, ...], ...],
    model_hz: float,
    command_id: int,
    samples: list[ChunkSample],
) -> int:
    period = 1.0 / model_hz
    started = time.monotonic()
    last_seq = -1
    for index, target in enumerate(knots[1:]):
        deadline = started + index * period
        now = time.monotonic()
        if now < deadline:
            time.sleep(deadline - now)
        command_id += 1
        observation = _send_target(connection, args, command_id, target)
        last_seq = _record_sample(samples, observation, target, last_seq)
    final_deadline = started + (len(knots) - 1) * period
    now = time.monotonic()
    if now < final_deadline:
        time.sleep(final_deadline - now)
    observation = _ready_observation(connection, args)
    _record_sample(samples, observation, knots[-1], last_seq)
    return command_id


def _play_interpolated(
    connection: RobotConnection,
    args: argparse.Namespace,
    plan: FrozenLinearPlan,
    command_id: int,
    samples: list[ChunkSample],
) -> int:
    period = 1.0 / args.command_hz
    started = time.monotonic()
    next_tick = started
    last_seq = -1
    while True:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(next_tick - now)
        now = time.monotonic()
        elapsed = min(now - started, plan.duration)
        target = plan.value(elapsed)
        command_id += 1
        observation = _send_target(connection, args, command_id, target)
        last_seq = _record_sample(samples, observation, target, last_seq)
        if elapsed >= plan.duration:
            break
        next_tick += period
        if next_tick < time.monotonic():
            next_tick = time.monotonic() + period
    return command_id


def _return_to_anchor(
    connection: RobotConnection,
    args: argparse.Namespace,
    command_id: int,
    start: tuple[float, ...],
    anchor: tuple[float, ...],
) -> int:
    period = 1.0 / args.command_hz
    started = time.monotonic()
    next_tick = started
    while True:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(next_tick - now)
        now = time.monotonic()
        elapsed = min(now - started, args.return_seconds)
        phase = elapsed / args.return_seconds
        target = tuple(minimum_jerk_blend(a, b, phase) for a, b in zip(start, anchor))
        command_id += 1
        _send_target(connection, args, command_id, target)
        if elapsed >= args.return_seconds:
            break
        next_tick += period
        if next_tick < time.monotonic():
            next_tick = time.monotonic() + period
    return command_id


def _print_report(
    mode: str,
    samples: list[ChunkSample],
    anchor: tuple[float, ...],
    final_joints: tuple[float, ...] | None,
) -> None:
    if len(samples) < 2:
        print("No usable frozen chunk samples were collected.")
        return
    times = np.asarray([sample.captured_monotonic for sample in samples], dtype=np.float64)
    observed = np.asarray([sample.observed_joints for sample in samples], dtype=np.float64)
    commanded = np.asarray([sample.commanded_joints for sample in samples], dtype=np.float64)
    dt = np.diff(times)
    valid = dt > 1e-6
    increments = np.abs(np.diff(observed, axis=0)[valid]).reshape(-1)
    velocities = np.diff(observed, axis=0)[valid] / dt[valid, None]
    velocity_times = (times[:-1][valid] + times[1:][valid]) * 0.5
    if len(velocities) >= 2:
        acceleration_dt = np.diff(velocity_times)
        acceleration_valid = acceleration_dt > 1e-6
        accelerations = (
            np.diff(velocities, axis=0)[acceleration_valid]
            / acceleration_dt[acceleration_valid, None]
        ).reshape(-1)
    else:
        accelerations = np.asarray([], dtype=np.float64)
    apparent_error = observed - commanded

    def percentile(values: np.ndarray, quantile: float) -> float:
        return 0.0 if values.size == 0 else float(np.percentile(np.abs(values), quantile))

    worst_flat = int(np.argmax(np.abs(apparent_error)))
    worst_joint = JOINT_NAMES[worst_flat % len(JOINT_NAMES)]
    print(f"\nFrozen policy chunk report ({mode})")
    print(f"  unique observation samples: {len(samples)}")
    print(f"  observed step |dq|: p95={percentile(increments, 95):.6f}, "
          f"max={percentile(increments, 100):.6f} rad")
    print(f"  observed |velocity|: p95={percentile(velocities, 95):.6f}, "
          f"max={percentile(velocities, 100):.6f} rad/s")
    print(f"  observed |acceleration|: p95={percentile(accelerations, 95):.6f}, "
          f"max={percentile(accelerations, 100):.6f} rad/s^2")
    print(f"  apparent tracking error: rms={float(np.sqrt(np.mean(apparent_error**2))):.6f}, "
          f"max={float(np.max(np.abs(apparent_error))):.6f} rad ({worst_joint})")
    if final_joints is not None:
        return_error = np.asarray(final_joints) - np.asarray(action_arms(anchor))
        print(f"  return-to-anchor error: max={float(np.max(np.abs(return_error))):.6f} rad")
    print("  note: tracking error includes camera/transport observation latency")


def run(args: argparse.Namespace) -> int:
    connection: RobotConnection | None = None
    samples: list[ChunkSample] = []
    anchor: tuple[float, ...] | None = None
    final_joints: tuple[float, ...] | None = None
    reason = "frozen chunk test completed"
    inference_wall_ms: float | None = None
    try:
        LOGGER.info("connecting to robot bridge at %s:%d", args.robot_host, args.robot_port)
        connection = RobotConnection(args.robot_host, args.robot_port, args.connect_timeout)
        hello = connection.hello
        LOGGER.info(
            "bridge: motion_allowed=%s publish_hz=%.1f max_step=%.3frad",
            hello.motion_allowed,
            hello.publish_hz,
            hello.max_joint_step_rad,
        )
        if not hello.motion_allowed:
            raise RolloutError("bridge motion is disabled; restart it with --allow-motion")
        if hello.publish_hz < args.min_bridge_hz:
            raise RolloutError(
                f"bridge publishes at {hello.publish_hz:.1f}Hz; this test requires at least "
                f"{args.min_bridge_hz:.1f}Hz"
            )

        observation = connection.wait_for_observation(timeout_s=args.observation_timeout)
        validate_observation(observation, args.max_source_age)
        if observation.input_mode == 3:
            raise RolloutError("start this test with Apex Input Mode None, not Custom")
        current_action = build_state16(
            observation.joints,
            observation.gripper_raw_left,
            observation.gripper_raw_right,
        )

        if args.capture_plan:
            LOGGER.info("connecting to OpenPI policy at ws://%s:%d", args.policy_host, args.policy_port)
            policy = websocket_client_policy.WebsocketClientPolicy(args.policy_host, args.policy_port)
            LOGGER.info("policy metadata: %s", policy.get_server_metadata())
            for index in range(args.warmup_inferences):
                observation = connection.latest(args.max_observation_age)
                validate_observation(observation, args.max_source_age)
                _, warmup_timing = infer_actions(policy, observation, args.prompt)
                LOGGER.info("discarded warmup inference %d: %s", index + 1, warmup_timing)
            anchor_observation = connection.latest(args.max_observation_age)
            validate_observation(anchor_observation, args.max_source_age)
            if anchor_observation.input_mode == 3:
                raise RolloutError("Input Mode changed to Custom before plan capture")
            actions, timing = infer_actions(policy, anchor_observation, args.prompt)
            inference_wall_ms = float(timing["wall_ms"])
            anchor = build_state16(
                anchor_observation.joints,
                anchor_observation.gripper_raw_left,
                anchor_observation.gripper_raw_right,
            )
            raw_knots, knots, clipped_values = _prepare_knots(
                actions, anchor, anchor_observation.joints, args
            )
            raw_stats = _chunk_stats(raw_knots, args.model_hz)
            playback_stats = _chunk_stats(knots, args.model_hz)
            model_hz = args.model_hz
            prompt = args.prompt
            plan_path = Path(args.capture_plan)
            _write_plan(
                plan_path,
                raw_knots=raw_knots,
                playback_knots=knots,
                model_hz=model_hz,
                prompt=prompt,
                clipped_values=clipped_values,
                overwrite=args.overwrite_plan,
            )
            LOGGER.info("saved frozen plan to %s", plan_path)
        else:
            plan_path = Path(args.load_plan)
            raw_knots, knots, model_hz, clipped_values, prompt = _read_plan(plan_path)
            raw_knots = _replace_grippers(raw_knots, current_action)
            anchor = knots[0]
            knots = _replace_grippers(knots, current_action)
            anchor = knots[0]
            raw_stats = _chunk_stats(raw_knots, model_hz)
            playback_stats = _chunk_stats(knots, model_hz)
            anchor_joints = action_arms(anchor)
            for knot in knots:
                validate_action(
                    knot,
                    anchor_joints,
                    max_joint_step_rad=0.08,
                    joint_limit_margin_rad=args.joint_limit_margin_rad,
                )

        LOGGER.info(
            "frozen dynamics: raw max_velocity=%.5frad/s max_acceleration=%.5frad/s^2; "
            "bounded max_velocity=%.5frad/s max_acceleration=%.5frad/s^2; clipped=%d/%d",
            raw_stats.max_velocity,
            raw_stats.max_acceleration,
            playback_stats.max_velocity,
            playback_stats.max_acceleration,
            clipped_values,
            (len(knots) - 1) * 14,
        )
        if playback_stats.max_velocity > args.safety_max_velocity_rad_s + 1e-9:
            raise RolloutError(
                f"bounded chunk max velocity {playback_stats.max_velocity:.5f}rad/s exceeds "
                f"diagnostic safety cap "
                f"{args.safety_max_velocity_rad_s:.5f}rad/s; plan was not executed"
            )
        if playback_stats.max_acceleration > args.safety_max_acceleration_rad_s2 + 1e-9:
            raise RolloutError(
                f"bounded chunk max acceleration {playback_stats.max_acceleration:.5f}rad/s^2 "
                f"exceeds diagnostic safety cap {args.safety_max_acceleration_rad_s2:.5f}rad/s^2; "
                "plan was not executed"
            )

        plan = FrozenLinearPlan(knots, model_hz)
        current = connection.latest(args.max_observation_age)
        pre_gate_drift = float(
            np.max(np.abs(np.asarray(current.joints) - np.asarray(action_arms(anchor))))
        )
        if pre_gate_drift > args.max_anchor_drift_rad:
            raise RolloutError(
                f"current pose differs from saved anchor by {pre_gate_drift:.5f}rad "
                f"(limit {args.max_anchor_drift_rad:.5f}rad)"
            )

        LOGGER.warning("plan ready; switch Apex Input Mode to Custom now")
        ready = connection.wait_for_observation(timeout_s=args.ready_timeout, require_motion_gate=True)
        validate_observation(ready, args.max_source_age)
        anchor_drift = float(
            np.max(np.abs(np.asarray(ready.joints) - np.asarray(action_arms(anchor))))
        )
        if anchor_drift > args.max_anchor_drift_rad:
            raise RolloutError(
                f"pose drifted {anchor_drift:.5f}rad from saved anchor "
                f"(limit {args.max_anchor_drift_rad:.5f}rad)"
            )
        _confirm(
            args,
            ready,
            raw_stats=raw_stats,
            playback_stats=playback_stats,
            clipped_values=clipped_values,
            plan_duration=plan.duration,
            model_hz=model_hz,
            prompt=prompt,
            anchor_drift=anchor_drift,
            bridge_publish_hz=hello.publish_hz,
            inference_wall_ms=inference_wall_ms,
        )

        command_id = _settle_at_anchor(connection, args, 0, anchor)
        if args.playback_mode == "discrete":
            command_id = _play_discrete(
                connection, args, knots, model_hz, command_id, samples
            )
        else:
            command_id = _play_interpolated(connection, args, plan, command_id, samples)

        LOGGER.info("playback complete; returning to saved anchor")
        command_id = _return_to_anchor(
            connection,
            args,
            command_id,
            knots[-1],
            anchor,
        )
        LOGGER.warning(
            "return-to-anchor complete; continuing to hold the anchor. "
            "Switch Apex Input Mode to None now"
        )
        deadline = time.monotonic() + args.exit_mode_timeout
        while True:
            observation = connection.latest(args.max_observation_age)
            if observation.input_mode != 3:
                final_joints = observation.joints
                break
            if time.monotonic() >= deadline:
                raise RolloutError(
                    f"Input Mode stayed Custom for {args.exit_mode_timeout:.1f}s after the test"
                )
            if not observation.motion_gate_open:
                raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
            validate_observation(observation, args.max_source_age)
            _check_command_feedback(observation)
            command_id += 1
            connection.send_action(ActionCommand(command_id, observation.seq, anchor, execute=True))
            time.sleep(1.0 / args.command_hz)

        LOGGER.info("Input Mode is no longer Custom; disconnecting frozen chunk client")
        _print_report(args.playback_mode, samples, anchor, final_joints)
        return 0
    except KeyboardInterrupt:
        reason = "operator interrupted frozen chunk test"
        LOGGER.warning(
            "%s; use Apex Input Mode None or the emergency stop if the robot is not stable",
            reason,
        )
        if anchor is not None:
            _print_report(args.playback_mode, samples, anchor, final_joints)
        return 130
    except (ConnectionError, OSError, ProtocolError, RolloutError, SafetyError, ValueError) as exc:
        reason = f"frozen chunk test aborted: {exc}"
        LOGGER.error(reason)
        if anchor is not None:
            _print_report(args.playback_mode, samples, anchor, final_joints)
        return 1
    except Exception as exc:
        reason = f"frozen chunk test aborted by unexpected error: {exc}"
        LOGGER.exception(reason)
        if anchor is not None:
            _print_report(args.playback_mode, samples, anchor, final_joints)
        return 1
    finally:
        if connection is not None:
            connection.close(reason)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default=DEFAULT_BRIDGE_HOST)
    parser.add_argument("--robot-port", type=int, default=DEFAULT_BRIDGE_PORT)
    parser.add_argument("--policy-host", default=DEFAULT_POLICY_HOST)
    parser.add_argument("--policy-port", type=int, default=DEFAULT_POLICY_PORT)
    parser.add_argument("--prompt", default=DEFAULT_PROMPT)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--capture-plan", help="capture policy output and save this JSON plan")
    source.add_argument("--load-plan", help="load the exact JSON plan captured by the first run")
    parser.add_argument("--overwrite-plan", action="store_true")
    parser.add_argument("--playback-mode", choices=("discrete", "interpolated"), required=True)
    parser.add_argument("--chunk-steps", type=int, default=10)
    parser.add_argument("--model-hz", type=float, default=15.0)
    parser.add_argument("--command-hz", type=float, default=100.0)
    parser.add_argument("--min-bridge-hz", type=float, default=90.0)
    parser.add_argument("--max-model-offset-rad", type=float, default=0.03)
    parser.add_argument("--safety-max-velocity-rad-s", type=float, default=0.45)
    parser.add_argument("--safety-max-acceleration-rad-s2", type=float, default=2.0)
    parser.add_argument("--max-anchor-drift-rad", type=float, default=0.01)
    parser.add_argument("--joint-limit-margin-rad", type=float, default=0.02)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--return-seconds", type=float, default=2.0)
    parser.add_argument("--warmup-inferences", type=int, default=1)
    parser.add_argument("--observation-timeout", type=float, default=10.0)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument("--exit-mode-timeout", type=float, default=30.0)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--max-source-age", type=float, default=0.20)
    parser.add_argument("--max-observation-age", type=float, default=0.35)
    parser.add_argument("--yes", action="store_true", help="skip the typed confirmation")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    args = parser.parse_args(argv)
    if not 1 <= args.chunk_steps <= 10:
        parser.error("chunk-steps must be in [1, 10]")
    positive = (
        args.model_hz,
        args.command_hz,
        args.min_bridge_hz,
        args.max_model_offset_rad,
        args.safety_max_velocity_rad_s,
        args.safety_max_acceleration_rad_s2,
        args.max_anchor_drift_rad,
        args.settle_seconds,
        args.return_seconds,
        args.exit_mode_timeout,
    )
    if any(value <= 0 for value in positive):
        parser.error("rates, limits, durations, and timeouts must be positive")
    if args.max_model_offset_rad > 0.08:
        parser.error("max-model-offset-rad must not exceed 0.08")
    if args.command_hz < args.min_bridge_hz:
        parser.error("command-hz must be at least min-bridge-hz")
    if args.warmup_inferences < 0:
        parser.error("warmup-inferences cannot be negative")
    if args.overwrite_plan and not args.capture_plan:
        parser.error("overwrite-plan is only valid with capture-plan")
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
