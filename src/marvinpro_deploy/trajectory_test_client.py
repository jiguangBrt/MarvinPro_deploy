"""Deterministic minimum-jerk motion diagnostic for Marvin Pro.

No policy is loaded. One selected joint follows a small, bounded trajectory
around its captured position while all other targets remain constant.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
import time

import numpy as np

from .config import DEFAULT_BRIDGE_HOST, DEFAULT_BRIDGE_PORT, JOINT_LOWER, JOINT_NAMES, JOINT_UPPER
from .joint_mapping import build_state16
from .motion_profile import MinimumJerkSweep
from .protocol import ActionCommand, ProtocolError
from .rollout_client import RobotConnection, RolloutError, validate_observation

LOGGER = logging.getLogger("marvinpro_trajectory_test")
HOME_VELOCITY_LIMIT_RAD_S = 0.2
HOME_ACCELERATION_LIMIT_RAD_S2 = 0.5
JOINT_LIMIT_MARGIN_RAD = 0.02


@dataclass(frozen=True)
class MotionSample:
    captured_monotonic: float
    observed_position: float
    commanded_position: float


def _check_command_feedback(observation) -> None:
    status = observation.last_command_status
    if status.startswith("rejected") or "failed" in status:
        raise RolloutError(f"bridge {status}")


def _action_with_joint_position(
    base_action: tuple[float, ...],
    joint_index: int,
    position: float,
) -> tuple[float, ...]:
    action_index = joint_index if joint_index < 7 else joint_index + 1
    action = list(base_action)
    action[action_index] = float(position)
    return tuple(action)


def _validate_sweep_range(joint_index: int, center: float, amplitude: float) -> None:
    safe_lower = JOINT_LOWER[joint_index] + JOINT_LIMIT_MARGIN_RAD
    safe_upper = JOINT_UPPER[joint_index] - JOINT_LIMIT_MARGIN_RAD
    if center - amplitude < safe_lower or center + amplitude > safe_upper:
        raise RolloutError(
            f"{JOINT_NAMES[joint_index]} sweep [{center - amplitude:.5f}, {center + amplitude:.5f}] "
            f"exceeds safe range [{safe_lower:.5f}, {safe_upper:.5f}]"
        )


def _confirm(
    args: argparse.Namespace,
    observation,
    bridge_publish_hz: float,
    center: float,
    profile: MinimumJerkSweep,
) -> None:
    print("\nDETERMINISTIC MINIMUM-JERK TEST")
    print(f"  input_mode: {observation.input_mode}")
    print(f"  robot_state: {observation.robot_state}")
    print(f"  arm_state: {observation.arm_state}")
    print(f"  bridge publish rate: {bridge_publish_hz:.1f}Hz")
    print(f"  client command rate: {args.command_hz:.1f}Hz")
    print(f"  moving joint: {args.joint}")
    print(f"  captured center: {center:.5f}rad")
    print(f"  sweep: 0 -> +{profile.amplitude_rad:.5f} -> -{profile.amplitude_rad:.5f} -> 0 rad")
    print(f"  trajectory duration: {profile.duration:.1f}s")
    print(f"  theoretical max velocity: {profile.max_velocity:.5f}rad/s")
    print(f"  theoretical max acceleration: {profile.max_acceleration:.5f}rad/s^2")
    print("  keep the emergency stop reachable; switch Input Mode to None to stop safely.")
    if args.yes:
        return
    if input('Type exactly "MOVE" to begin: ') != "MOVE":
        raise RolloutError("trajectory test confirmation was not given")


def _ready_observation(connection: RobotConnection, args: argparse.Namespace):
    observation = connection.latest(args.max_observation_age)
    if observation.input_mode != 3:
        raise RolloutError("Input Mode left Custom before the trajectory completed")
    if not observation.motion_gate_open:
        raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
    validate_observation(observation, args.max_source_age)
    _check_command_feedback(observation)
    return observation


def _print_report(
    samples: list[MotionSample],
    center: float,
    final_position: float | None,
) -> None:
    if len(samples) < 2:
        print("No usable trajectory samples were collected.")
        return

    times = np.asarray([sample.captured_monotonic for sample in samples], dtype=np.float64)
    observed = np.asarray([sample.observed_position for sample in samples], dtype=np.float64)
    commanded = np.asarray([sample.commanded_position for sample in samples], dtype=np.float64)
    dt = np.diff(times)
    valid = dt > 1e-6
    increments = np.abs(np.diff(observed)[valid])
    velocities = np.diff(observed)[valid] / dt[valid]
    velocity_times = (times[:-1][valid] + times[1:][valid]) * 0.5
    if len(velocities) >= 2:
        acceleration_dt = np.diff(velocity_times)
        acceleration_valid = acceleration_dt > 1e-6
        accelerations = np.diff(velocities)[acceleration_valid] / acceleration_dt[acceleration_valid]
    else:
        accelerations = np.asarray([], dtype=np.float64)
    apparent_error = observed - commanded

    def percentile(values: np.ndarray, quantile: float) -> float:
        return 0.0 if values.size == 0 else float(np.percentile(np.abs(values), quantile))

    print("\nDeterministic trajectory report")
    print(f"  unique observation samples: {len(samples)}")
    print(f"  observed range about center: [{observed.min() - center:.6f}, {observed.max() - center:.6f}] rad")
    print(f"  observed step |dq|: p50={percentile(increments, 50):.6f}, "
          f"p95={percentile(increments, 95):.6f}, max={percentile(increments, 100):.6f} rad")
    print(f"  observed |velocity|: p95={percentile(velocities, 95):.6f}, "
          f"max={percentile(velocities, 100):.6f} rad/s")
    print(f"  observed |acceleration|: p95={percentile(accelerations, 95):.6f}, "
          f"max={percentile(accelerations, 100):.6f} rad/s^2")
    print(f"  apparent tracking error: rms={float(np.sqrt(np.mean(apparent_error**2))):.6f}, "
          f"max={float(np.max(np.abs(apparent_error))):.6f} rad")
    if final_position is not None:
        print(f"  final return error: {final_position - center:.6f} rad")
    print("  note: tracking error includes camera/transport observation latency")


def run(args: argparse.Namespace) -> int:
    connection: RobotConnection | None = None
    samples: list[MotionSample] = []
    center: float | None = None
    final_position: float | None = None
    reason = "trajectory test completed"
    profile = MinimumJerkSweep(args.amplitude_rad, args.segment_seconds)
    joint_index = JOINT_NAMES.index(args.joint)

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
        if args.amplitude_rad > hello.max_joint_step_rad:
            raise RolloutError(
                f"amplitude {args.amplitude_rad:.5f}rad exceeds bridge step envelope "
                f"{hello.max_joint_step_rad:.5f}rad"
            )

        LOGGER.info("waiting for Custom input and joint impedance state [3, 3]")
        observation = connection.wait_for_observation(
            timeout_s=args.ready_timeout,
            require_motion_gate=True,
        )
        validate_observation(observation, args.max_source_age)
        base_action = build_state16(
            observation.joints,
            observation.gripper_raw_left,
            observation.gripper_raw_right,
        )
        center = observation.joints[joint_index]
        _validate_sweep_range(joint_index, center, args.amplitude_rad)
        _confirm(args, observation, hello.publish_hz, center, profile)

        command_id = 0
        period = 1.0 / args.command_hz

        settle_deadline = time.monotonic() + args.settle_seconds
        next_tick = time.monotonic()
        while time.monotonic() < settle_deadline:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            observation = _ready_observation(connection, args)
            command_id += 1
            connection.send_action(
                ActionCommand(
                    command_id=command_id,
                    observation_seq=observation.seq,
                    action=base_action,
                    execute=True,
                )
            )
            next_tick += period
            if next_tick < time.monotonic():
                next_tick = time.monotonic() + period

        started = time.monotonic()
        next_tick = started
        last_sample_seq = -1
        while True:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            now = time.monotonic()
            elapsed = min(now - started, profile.duration)
            commanded_position = center + profile.offset(elapsed)
            target = _action_with_joint_position(base_action, joint_index, commanded_position)
            observation = _ready_observation(connection, args)
            command_id += 1
            connection.send_action(
                ActionCommand(
                    command_id=command_id,
                    observation_seq=observation.seq,
                    action=target,
                    execute=True,
                )
            )
            if observation.seq != last_sample_seq:
                samples.append(
                    MotionSample(
                        captured_monotonic=observation.captured_monotonic,
                        observed_position=observation.joints[joint_index],
                        commanded_position=commanded_position,
                    )
                )
                last_sample_seq = observation.seq
            if elapsed >= profile.duration:
                break
            next_tick += period
            if next_tick < time.monotonic():
                next_tick = time.monotonic() + period

        LOGGER.warning(
            "trajectory finished and returned to its start target; continuing to hold. "
            "Switch Apex Input Mode to None now"
        )
        deadline = time.monotonic() + args.exit_mode_timeout
        while True:
            observation = connection.latest(args.max_observation_age)
            if observation.input_mode != 3:
                final_position = observation.joints[joint_index]
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
            connection.send_action(
                ActionCommand(
                    command_id=command_id,
                    observation_seq=observation.seq,
                    action=base_action,
                    execute=True,
                )
            )
            time.sleep(period)

        LOGGER.info("Input Mode is no longer Custom; disconnecting trajectory client")
        _print_report(samples, center, final_position)
        return 0
    except KeyboardInterrupt:
        reason = "operator interrupted trajectory test"
        LOGGER.warning(
            "%s; use Apex Input Mode None or the emergency stop if the robot is not stable",
            reason,
        )
        if center is not None:
            _print_report(samples, center, final_position)
        return 130
    except (ConnectionError, OSError, ProtocolError, RolloutError, ValueError) as exc:
        reason = f"trajectory test aborted: {exc}"
        LOGGER.error(reason)
        if center is not None:
            _print_report(samples, center, final_position)
        return 1
    finally:
        if connection is not None:
            connection.close(reason)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default=DEFAULT_BRIDGE_HOST)
    parser.add_argument("--robot-port", type=int, default=DEFAULT_BRIDGE_PORT)
    parser.add_argument("--joint", choices=JOINT_NAMES, default="Joint7_L")
    parser.add_argument("--amplitude-rad", type=float, default=0.04)
    parser.add_argument("--segment-seconds", type=float, default=2.0)
    parser.add_argument("--command-hz", type=float, default=15.0)
    parser.add_argument("--min-bridge-hz", type=float, default=90.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument("--exit-mode-timeout", type=float, default=30.0)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--max-source-age", type=float, default=0.20)
    parser.add_argument("--max-observation-age", type=float, default=0.35)
    parser.add_argument("--yes", action="store_true", help="skip the typed MOVE confirmation")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    args = parser.parse_args(argv)
    if args.amplitude_rad <= 0 or args.amplitude_rad > 0.05:
        parser.error("amplitude-rad must be in (0, 0.05]")
    if args.segment_seconds <= 0 or args.command_hz <= 0 or args.min_bridge_hz <= 0:
        parser.error("durations and rates must be positive")
    if args.settle_seconds <= 0 or args.exit_mode_timeout <= 0:
        parser.error("settle-seconds and exit-mode-timeout must be positive")
    profile = MinimumJerkSweep(args.amplitude_rad, args.segment_seconds)
    if profile.max_velocity > HOME_VELOCITY_LIMIT_RAD_S:
        parser.error(f"profile velocity {profile.max_velocity:.5f}rad/s exceeds Home limit")
    if profile.max_acceleration > HOME_ACCELERATION_LIMIT_RAD_S2:
        parser.error(f"profile acceleration {profile.max_acceleration:.5f}rad/s^2 exceeds Home limit")
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
