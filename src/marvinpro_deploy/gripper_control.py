"""Direct, short-lived control of the Marvin Pro grippers."""

from __future__ import annotations

import argparse
import sys
import time

from .config import (
    GRIPPER_CLOSED_RAW,
    GRIPPER_OPEN_RAW,
    TOPIC_GRIPPER_CMD_L,
    TOPIC_GRIPPER_CMD_R,
    TOPIC_GRIPPER_FEEDBACK_L,
    TOPIC_GRIPPER_FEEDBACK_R,
)


COMMAND_VALUE = {"0": 0.0, "1": 1.0}
COMMAND_NAME = {"0": "fully open", "1": "fully closed"}
DEFAULT_DURATION_S = 1.0
PUBLISH_HZ = 20.0
DISCOVERY_TIMEOUT_S = 3.0


def command_topics(side: str) -> tuple[tuple[str, str], ...]:
    topics = {
        "left": (("left", TOPIC_GRIPPER_CMD_L),),
        "right": (("right", TOPIC_GRIPPER_CMD_R),),
        "both": (("left", TOPIC_GRIPPER_CMD_L), ("right", TOPIC_GRIPPER_CMD_R)),
    }
    try:
        return topics[side]
    except KeyError as exc:
        raise ValueError(f"unsupported gripper side: {side}") from exc


def positive_duration(value: str) -> float:
    duration = float(value)
    if not 0.1 <= duration <= 10.0:
        raise argparse.ArgumentTypeError("duration must be between 0.1 and 10 seconds")
    return duration


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send 0 (fully open) or 1 (fully closed) directly to the Marvin Pro grippers."
    )
    parser.add_argument("command", choices=tuple(COMMAND_VALUE), help="0=open, 1=closed")
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="gripper to control (default: both)",
    )
    parser.add_argument(
        "--duration",
        type=positive_duration,
        default=DEFAULT_DURATION_S,
        metavar="SECONDS",
        help=f"command publication duration (default: {DEFAULT_DURATION_S:g})",
    )
    return parser


def _run(command: str, side: str, duration_s: float) -> int:
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32, Float32MultiArray
    except ImportError:  # pragma: no cover - ROS is available only on the controller
        print(
            "ROS 2 is unavailable. Run this in the controller Apex environment or use "
            "scripts/control_gripper_on_controller.sh.",
            file=sys.stderr,
        )
        return 2

    command_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    feedback_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    rclpy.init(args=None)
    node = rclpy.create_node("marvinpro_direct_gripper_control")
    feedback: dict[str, float] = {}
    subscriptions = []

    def save_feedback(name: str):
        def callback(message) -> None:
            if message.data:
                feedback[name] = float(message.data[0])

        return callback

    selected_topics = command_topics(side)
    publishers = [(name, topic, node.create_publisher(Float32, topic, command_qos)) for name, topic in selected_topics]
    feedback_topics = {
        "left": TOPIC_GRIPPER_FEEDBACK_L,
        "right": TOPIC_GRIPPER_FEEDBACK_R,
    }
    for name, _topic in selected_topics:
        subscriptions.append(
            node.create_subscription(
                Float32MultiArray,
                feedback_topics[name],
                save_feedback(name),
                feedback_qos,
            )
        )

    try:
        discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(node.count_subscribers(topic) > 0 for _name, topic, _publisher in publishers):
                break

        missing = [topic for _name, topic, _publisher in publishers if node.count_subscribers(topic) == 0]
        if missing:
            print(
                "No gripper driver subscriber found for: " + ", ".join(missing),
                file=sys.stderr,
            )
            return 2

        graph_nodes = {name for name, _namespace in node.get_node_names_and_namespaces()}
        if "marvinpro_rollout_bridge" in graph_nodes:
            print(
                "marvinpro_rollout_bridge is running. Stop the rollout bridge before direct gripper control.",
                file=sys.stderr,
            )
            return 2

        value = COMMAND_VALUE[command]
        message = Float32(data=value)
        interval_s = 1.0 / PUBLISH_HZ
        publish_deadline = time.monotonic() + duration_s
        next_publish = time.monotonic()
        publish_count = 0
        while time.monotonic() < publish_deadline:
            for _name, _topic, publisher in publishers:
                publisher.publish(message)
            publish_count += 1
            next_publish += interval_s
            while time.monotonic() < min(next_publish, publish_deadline):
                rclpy.spin_once(
                    node,
                    timeout_sec=min(next_publish, publish_deadline) - time.monotonic(),
                )

        names = ", ".join(name for name, _topic, _publisher in publishers)
        print(
            f"Command sent: {names} -> {command} ({COMMAND_NAME[command]}), {publish_count} times at {PUBLISH_HZ:g} Hz."
        )
        if feedback:
            readings = ", ".join(f"{name}={feedback[name]:.3f}" for name, _topic in selected_topics if name in feedback)
            print(
                f"Latest raw feedback: {readings} "
                f"(calibration: open={GRIPPER_OPEN_RAW:g}, closed~={GRIPPER_CLOSED_RAW:g})."
            )
        else:
            print("No gripper feedback was received during the command.", file=sys.stderr)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return _run(args.command, args.side, args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
