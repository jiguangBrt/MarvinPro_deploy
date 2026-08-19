"""Read-only recorder for the controller's gripper feedback topics."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
from pathlib import Path
import time

from .config import (
    TOPIC_GRIPPER_CMD_L,
    TOPIC_GRIPPER_CMD_R,
    TOPIC_GRIPPER_FEEDBACK_L,
    TOPIC_GRIPPER_FEEDBACK_R,
)


FIELD_NAMES = ("q_rad", "dq_rad_s", "tau", "T_mos", "T_motor")
MAX_DISTINCT_SAMPLES = 8192
DISCOVERY_TIMEOUT_S = 10.0


def _format_values(values: tuple[float | None, ...] | None) -> str:
    if values is None:
        return "none"
    return "[" + ", ".join("none" if value is None else f"{value:.6g}" for value in values) + "]"


@dataclass
class FeedbackStats:
    """Small bounded summary that makes a frozen stream obvious."""

    side: str
    count: int = 0
    invalid_count: int = 0
    changed_count: int = 0
    first_monotonic: float | None = None
    last_monotonic: float | None = None
    first_values: tuple[float | None, ...] | None = None
    last_values: tuple[float | None, ...] | None = None
    minimum: list[float | None] = field(default_factory=lambda: [None] * len(FIELD_NAMES))
    maximum: list[float | None] = field(default_factory=lambda: [None] * len(FIELD_NAMES))
    max_step: list[float] = field(default_factory=lambda: [0.0] * len(FIELD_NAMES))
    _distinct: set[tuple[float | None, ...]] = field(default_factory=set, repr=False)
    distinct_capped: bool = False
    _previous_values: tuple[float | None, ...] | None = field(default=None, init=False, repr=False)

    def record(self, raw_values, monotonic: float) -> tuple[float | None, ...] | None:
        """Record one ROS array and return its normalized five-column form."""
        self.count += 1
        try:
            raw = tuple(float(value) for value in raw_values)
        except (TypeError, ValueError):
            self.invalid_count += 1
            return None
        if len(raw) < 3 or not all(math.isfinite(value) for value in raw):
            self.invalid_count += 1
            return None

        values = tuple(raw[index] if index < len(raw) else None for index in range(len(FIELD_NAMES)))
        if self.first_values is None:
            self.first_values = values
            self.first_monotonic = monotonic
        elif values != self.last_values:
            self.changed_count += 1
        self.last_values = values
        self.last_monotonic = monotonic
        if not self.distinct_capped:
            self._distinct.add(values)
            if len(self._distinct) >= MAX_DISTINCT_SAMPLES:
                self.distinct_capped = True

        for index, value in enumerate(values):
            if value is None:
                continue
            if self.minimum[index] is None or value < self.minimum[index]:
                self.minimum[index] = value
            if self.maximum[index] is None or value > self.maximum[index]:
                self.maximum[index] = value
            previous = self._previous_values[index] if self._previous_values is not None else None
            if previous is not None:
                self.max_step[index] = max(self.max_step[index], abs(value - previous))
        self._previous_values = values
        return values

    @property
    def distinct_count(self) -> int:
        return len(self._distinct)

    def summary(self, elapsed: float) -> str:
        rate = self.count / max(elapsed, 1e-9)
        span = [
            None if lo is None or hi is None else hi - lo
            for lo, hi in zip(self.minimum, self.maximum)
        ]
        distinct_label = f">={self.distinct_count}" if self.distinct_capped else f"={self.distinct_count}"
        return (
            f"{self.side}: msgs={self.count}, valid={self.count - self.invalid_count}, "
            f"invalid={self.invalid_count}, rate={rate:.1f}Hz, changed={self.changed_count}, "
            f"distinct{distinct_label}\n"
            f"  first={_format_values(self.first_values)}\n"
            f"  last={_format_values(self.last_values)}\n"
            f"  span={_format_values(tuple(span))}\n"
            f"  max_step={_format_values(tuple(self.max_step))}"
        )


def _duration(value: str) -> float:
    duration = float(value)
    if not 0.0 <= duration <= 3600.0:
        raise argparse.ArgumentTypeError("duration must be 0 (until Ctrl+C) or between 1 and 3600 seconds")
    return duration


def _summary_period(value: str) -> float:
    period = float(value)
    if not 1.0 <= period <= 600.0:
        raise argparse.ArgumentTypeError("summary period must be between 1 and 600 seconds")
    return period


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record gripper feedback without publishing any command."
    )
    parser.add_argument(
        "--duration",
        type=_duration,
        default=0.0,
        metavar="SECONDS",
        help="record length; 0 means keep recording until Ctrl+C (default: 0)",
    )
    parser.add_argument("--output", type=Path, required=True, help="CSV output path")
    parser.add_argument("--summary-period", type=_summary_period, default=5.0, metavar="SECONDS")
    return parser


def _run(duration: float, output: Path, summary_period: float) -> int:
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32, Float32MultiArray
    except ImportError as exc:  # pragma: no cover - ROS is available only on the controller
        print(f"ROS 2 is unavailable in this Python environment: {exc}")
        return 2

    output.parent.mkdir(parents=True, exist_ok=True)
    stats = {"left": FeedbackStats("left"), "right": FeedbackStats("right")}
    topics = {"left": TOPIC_GRIPPER_FEEDBACK_L, "right": TOPIC_GRIPPER_FEEDBACK_R}
    feedback_qos = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    command_qos = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    rclpy.init(args=None)
    node = rclpy.create_node("marvinpro_gripper_feedback_recorder")
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "wall_time_utc",
                "monotonic_s",
                "record_type",
                "side",
                "command_value",
                "raw_len",
                *FIELD_NAMES,
            ]
        )
        command_counts = {"left": 0, "right": 0}
        last_commands: dict[str, float | None] = {"left": None, "right": None}

        def callback(side: str):
            def receive(message) -> None:
                now = time.monotonic()
                values = stats[side].record(message.data, now)
                raw = tuple(message.data) if message.data is not None else ()
                row_values = list(values) if values is not None else [None] * len(FIELD_NAMES)
                writer.writerow(
                    [
                        datetime.now(timezone.utc).isoformat(),
                        f"{now:.6f}",
                        "feedback",
                        side,
                        None,
                        len(raw),
                        *row_values,
                    ]
                )
                if stats[side].count % 100 == 0:
                    stream.flush()

            return receive

        def command_callback(side: str):
            def receive(message) -> None:
                now = time.monotonic()
                value = float(message.data)
                command_counts[side] += 1
                last_commands[side] = value
                writer.writerow(
                    [
                        datetime.now(timezone.utc).isoformat(),
                        f"{now:.6f}",
                        "command",
                        side,
                        value,
                        None,
                        *([None] * len(FIELD_NAMES)),
                    ]
                )
                stream.flush()

            return receive

        for side, topic in topics.items():
            node.create_subscription(Float32MultiArray, topic, callback(side), feedback_qos)
        command_topics = {"left": TOPIC_GRIPPER_CMD_L, "right": TOPIC_GRIPPER_CMD_R}
        for side, topic in command_topics.items():
            node.create_subscription(Float32, topic, command_callback(side), command_qos)

        print("Waiting for both gripper feedback topics...", flush=True)
        print(f"  left:  {TOPIC_GRIPPER_FEEDBACK_L}", flush=True)
        print(f"  right: {TOPIC_GRIPPER_FEEDBACK_R}", flush=True)
        print(f"  command markers: {TOPIC_GRIPPER_CMD_L}, {TOPIC_GRIPPER_CMD_R}", flush=True)
        print(f"  csv:   {output}", flush=True)
        discovery_started = time.monotonic()
        while (
            rclpy.ok()
            and (stats["left"].count == 0 or stats["right"].count == 0)
            and time.monotonic() - discovery_started < DISCOVERY_TIMEOUT_S
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        missing = [side for side in ("left", "right") if stats[side].count == 0]
        if missing:
            print(f"ERROR: no feedback received for: {', '.join(missing)}", flush=True)
            node.destroy_node()
            rclpy.try_shutdown()
            return 2

        started = time.monotonic()
        last_summary = started
        duration_text = "until Ctrl+C" if duration == 0.0 else f"for {duration:.1f}s"
        print(f"READY: recording read-only gripper feedback {duration_text}", flush=True)
        try:
            while rclpy.ok() and (duration == 0.0 or time.monotonic() - started < duration):
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now - last_summary >= summary_period:
                    elapsed = now - started
                    print(stats["left"].summary(elapsed), flush=True)
                    print(stats["right"].summary(elapsed), flush=True)
                    last_summary = now
        except KeyboardInterrupt:
            print("Interrupted; writing final feedback summary.", flush=True)
        finally:
            stream.flush()
            elapsed = max(0.0, time.monotonic() - started)
            print("Final feedback summary:", flush=True)
            print(stats["left"].summary(elapsed), flush=True)
            print(stats["right"].summary(elapsed), flush=True)
            print(
                "commands: "
                f"left={command_counts['left']} last={last_commands['left']}, "
                f"right={command_counts['right']} last={last_commands['right']}",
                flush=True,
            )
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return _run(args.duration, args.output, args.summary_period)


if __name__ == "__main__":
    raise SystemExit(main())
