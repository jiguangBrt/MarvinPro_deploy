"""Latched-pose hold diagnostic for the Marvin Pro Custom control path.

No policy is loaded. The client captures one measured pose, then repeatedly
sends that exact 16-dimensional absolute target until Input Mode leaves Custom.
"""

from __future__ import annotations

import argparse
import logging
import time

import numpy as np

from .config import CONTROL_HZ, DEFAULT_BRIDGE_HOST, DEFAULT_BRIDGE_PORT, JOINT_NAMES
from .joint_mapping import build_state16
from .protocol import ActionCommand, ProtocolError
from .rollout_client import RobotConnection, RolloutError, validate_observation

LOGGER = logging.getLogger("marvinpro_hold_test")


def _check_command_feedback(observation) -> None:
    status = observation.last_command_status
    if status.startswith("rejected") or "failed" in status:
        raise RolloutError(f"bridge {status}")


def _print_report(samples: list[tuple[float, ...]], target: tuple[float, ...]) -> None:
    if not samples:
        print("No joint samples were collected.")
        return
    joints = np.asarray(samples, dtype=np.float64)
    target_joints = np.asarray(target[:7] + target[8:15], dtype=np.float64)
    peak_to_peak = np.ptp(joints, axis=0)
    std = np.std(joints, axis=0)
    max_error = np.max(np.abs(joints - target_joints), axis=0)

    print("\nLatched-pose hold report")
    print(f"  samples: {len(samples)}")
    print(f"  max joint peak-to-peak: {peak_to_peak.max():.6f} rad")
    print(f"  max joint std:          {std.max():.6f} rad")
    print(f"  max tracking error:     {max_error.max():.6f} rad")
    print("  per joint: name peak_to_peak std max_error (rad)")
    for name, ptp, sigma, error in zip(JOINT_NAMES, peak_to_peak, std, max_error):
        print(f"    {name:8s} {ptp:11.6f} {sigma:11.6f} {error:11.6f}")


def _confirm(args: argparse.Namespace, observation, target: tuple[float, ...]) -> None:
    print("\nLATCHED-POSE HOLD TEST")
    print(f"  input_mode: {observation.input_mode}")
    print(f"  robot_state: {observation.robot_state}")
    print(f"  arm_state: {observation.arm_state}")
    print(f"  test duration: {args.duration:.1f}s")
    print(f"  client refresh rate: {args.send_hz:.1f}Hz")
    print("  arm target is captured once and will not follow later feedback.")
    print("  keep the emergency stop reachable; switch Input Mode to None to stop safely.")
    print("  latched arm target:", np.asarray(target[:7] + target[8:15]).round(5).tolist())
    if args.yes:
        return
    if input('Type exactly "HOLD" to begin: ') != "HOLD":
        raise RolloutError("hold test confirmation was not given")


def run(args: argparse.Namespace) -> int:
    connection: RobotConnection | None = None
    samples: list[tuple[float, ...]] = []
    target: tuple[float, ...] | None = None
    reason = "hold test completed"
    try:
        LOGGER.info("connecting to robot bridge at %s:%d", args.robot_host, args.robot_port)
        connection = RobotConnection(args.robot_host, args.robot_port, args.connect_timeout)
        LOGGER.info(
            "bridge: motion_allowed=%s publish_hz=%.1f max_step=%.3frad",
            connection.hello.motion_allowed,
            connection.hello.publish_hz,
            connection.hello.max_joint_step_rad,
        )
        if not connection.hello.motion_allowed:
            raise RolloutError("bridge motion is disabled; restart it with --allow-motion")

        LOGGER.info("waiting for Custom input and joint impedance state [3, 3]")
        observation = connection.wait_for_observation(
            timeout_s=args.ready_timeout,
            require_motion_gate=True,
        )
        validate_observation(observation, args.max_source_age)
        target = build_state16(
            observation.joints,
            observation.gripper_raw_left,
            observation.gripper_raw_right,
        )
        _confirm(args, observation, target)

        command_id = 0
        period = 1.0 / args.send_hz
        started = time.monotonic()
        next_tick = started
        switched_to_none = False

        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            next_tick += period
            observation = connection.latest(args.max_observation_age)
            if observation.input_mode != 3:
                LOGGER.warning("Input Mode left Custom during the hold test; stopping")
                switched_to_none = True
                break
            if not observation.motion_gate_open:
                raise RolloutError(f"robot motion gate closed: {observation.gate_reason}")
            validate_observation(observation, args.max_source_age)
            _check_command_feedback(observation)
            command_id += 1
            connection.send_action(
                ActionCommand(
                    command_id=command_id,
                    observation_seq=observation.seq,
                    action=target,
                    execute=True,
                )
            )
            samples.append(observation.joints)

        if not switched_to_none:
            LOGGER.warning(
                "timed hold phase finished; continuing the same latched target. "
                "Switch Apex Input Mode to None now"
            )
            deadline = time.monotonic() + args.exit_mode_timeout
            while True:
                observation = connection.latest(args.max_observation_age)
                if observation.input_mode != 3:
                    switched_to_none = True
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
                        action=target,
                        execute=True,
                    )
                )
                samples.append(observation.joints)
                time.sleep(period)

        LOGGER.info("Input Mode is no longer Custom; disconnecting hold client")
        _print_report(samples, target)
        return 0
    except KeyboardInterrupt:
        reason = "operator interrupted hold test"
        LOGGER.warning(
            "%s; use Apex Input Mode None or the emergency stop if the robot is not stable",
            reason,
        )
        if target is not None:
            _print_report(samples, target)
        return 130
    except (ConnectionError, OSError, ProtocolError, RolloutError, ValueError) as exc:
        reason = f"hold test aborted: {exc}"
        LOGGER.error(reason)
        if target is not None:
            _print_report(samples, target)
        return 1
    finally:
        if connection is not None:
            connection.close(reason)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default=DEFAULT_BRIDGE_HOST)
    parser.add_argument("--robot-port", type=int, default=DEFAULT_BRIDGE_PORT)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--send-hz", type=float, default=CONTROL_HZ)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument("--exit-mode-timeout", type=float, default=30.0)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--max-source-age", type=float, default=0.20)
    parser.add_argument("--max-observation-age", type=float, default=0.35)
    parser.add_argument("--yes", action="store_true", help="skip the typed HOLD confirmation")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.send_hz <= 0 or args.exit_mode_timeout <= 0:
        parser.error("duration, rates, and timeouts must be positive")
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
