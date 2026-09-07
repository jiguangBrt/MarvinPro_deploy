"""Return both arms to the recorded stack_two_cones home pose.

This reuses the official Apex go-home pipeline instead of the rollout bridge:
the ``/tj/control/movej`` service on ``planner_joint_node`` plans a trapezoidal
profile with the official per-joint limits (``home_vel_limit`` 0.2 rad/s,
``home_acc_limit`` 0.5 rad/s^2, 500 Hz planning) and publishes
``control/joint_cmd_plan_{A,B}``, which ``joint_cmd_mux`` forwards to the arms
only while the input mode is 2 (planner). The script therefore switches the
input mode to planner for the duration of the move and restores the previous
mode afterwards — the same sequence as the Apex Home button, but with the
recorded task start pose instead of the factory ``home_joints``.

Unlike the official go_home (which leaves grippers alone), this tool also
opens both grippers once the arms have settled, because the recorded task
start pose includes open grippers. Pass ``--keep-grippers`` to skip that step.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from . import gripper_control
from .config import (
    JOINT_NAMES,
    TOPIC_ARM_STATE,
    TOPIC_INPUT_MODE,
    TOPIC_JOINT_STATES,
    TOPIC_ROBOT_STATE,
)
from .joint_mapping import JointMap

# Task start pose for "Stack two red cones ...", recorded via teleoperation and
# read from /tj/joint_states on 2026-09-07 (arm stationary, |dq| < 1e-3 rad/s).
# Cross-checked against the first frames of the 109-episode stack_two_cones
# teleop dataset (2026-09-05/06). Order: L1..L7, R1..R7, radians — the same
# order as JOINT_NAMES and the MoveJ joint_values field.
STACK_TWO_CONES_HOME = (
    2.0767117, -0.8038981, -1.4687421, -2.2494760, -0.4545104, 0.4259046, -0.2547748,
    -2.1477914, -0.7910550, 1.5346968, -2.3803766, 0.6356401, 0.5740360, 0.0108498,
)

SERVICE_SET_INPUT = "/tj/control/set_input"
SERVICE_MOVEJ = "/tj/control/movej"
# joint_cmd_mux input modes: 0=idle 1=teleop 2=planner 3=user 4=replay.
PLANNER_INPUT_MODE = 2
# Official planner limits from marvin_teleop robot_param_m6_696.yaml.
HOME_VEL_LIMIT = 0.2
HOME_ACC_LIMIT = 0.5
# joint_cmd_mux ramps for 3.0 s on every input-mode switch.
MODE_RAMP_S = 3.5
SETTLE_POS_RAD = 0.01
SETTLE_VEL_RAD_S = 0.02
SETTLE_HOLD_S = 0.2
DISCOVERY_TIMEOUT_S = 3.0
SERVICE_TIMEOUT_S = 5.0
MAX_MOVE_TIMEOUT_S = 60.0


def trapezoid_time(distance_rad: float, vel_limit: float = HOME_VEL_LIMIT, acc_limit: float = HOME_ACC_LIMIT) -> float:
    """Minimum-time trapezoidal/triangular profile duration for one joint."""
    distance_rad = abs(float(distance_rad))
    if not math.isfinite(distance_rad) or vel_limit <= 0 or acc_limit <= 0:
        raise ValueError("distance must be finite and limits positive")
    if distance_rad <= vel_limit * vel_limit / acc_limit:
        return 2.0 * math.sqrt(distance_rad / acc_limit)
    return distance_rad / vel_limit + vel_limit / acc_limit


def move_timeout_s(current: tuple[float, ...], target: tuple[float, ...] = STACK_TWO_CONES_HOME) -> float:
    """Convergence deadline: slowest joint's profile plus ramp and margin."""
    slowest = max(trapezoid_time(c - t) for c, t in zip(current, target))
    return min(MAX_MOVE_TIMEOUT_S, slowest + MODE_RAMP_S + 10.0)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move both arms to the recorded stack_two_cones home pose via the official movej planner."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="read-only: report the current pose and distance to home without moving",
    )
    parser.add_argument(
        "--keep-grippers",
        action="store_true",
        help="do not open the grippers after the arms reach home",
    )
    return parser


def _run(check_only: bool) -> int:
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from marvin_msgs.srv import Int, MoveJ
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Int16MultiArray, Int32
    except ImportError:  # pragma: no cover - ROS is available only on the controller
        print(
            "ROS 2 is unavailable. Run this on the controller via scripts/go_home_on_controller.sh.",
            file=sys.stderr,
        )
        return 2

    qos_sensor = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    rclpy.init(args=None)
    node = rclpy.create_node("marvinpro_go_home")
    state: dict[str, object] = {
        "joint_map": None,
        "position": None,
        "velocity": None,
        "robot_state": None,
        "input_mode": None,
    }

    def on_joint_state(message) -> None:
        if state["joint_map"] is None:
            state["joint_map"] = JointMap.from_names(list(message.name))
        state["position"] = state["joint_map"].canonical_positions(list(message.position))
        if len(message.velocity) == len(message.position):
            state["velocity"] = tuple(abs(float(message.velocity[i])) for i in state["joint_map"].indices)

    node.create_subscription(JointState, TOPIC_JOINT_STATES, on_joint_state, qos_sensor)
    node.create_subscription(
        Int16MultiArray,
        TOPIC_ROBOT_STATE,
        lambda msg: state.__setitem__("robot_state", tuple(int(v) for v in msg.data)),
        qos_sensor,
    )
    node.create_subscription(
        Int32,
        TOPIC_INPUT_MODE,
        lambda msg: state.__setitem__("input_mode", int(msg.data)),
        qos_latched,
    )

    previous_mode: int | None = None
    mode_switched = False
    try:
        # Wait for all three inputs, not just joint_states: returning to the main
        # flow as soon as the first joint_states message arrives races the slower
        # robot_state/input_mode subscriptions and misreports them as missing.
        deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
        while time.monotonic() < deadline and (
            state["position"] is None or state["robot_state"] is None or state["input_mode"] is None
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if state["position"] is None:
            print(f"No message on {TOPIC_JOINT_STATES}. Is the robot service running?", file=sys.stderr)
            return 2

        current = state["position"]
        assert current is not None
        errors = [c - t for c, t in zip(current, STACK_TWO_CONES_HOME)]
        worst = max(range(14), key=lambda i: abs(errors[i]))
        print(
            f"Current pose is {abs(errors[worst]):.4f} rad away from home "
            f"(worst joint {JOINT_NAMES[worst]}).",
            flush=True,
        )
        if check_only:
            print("Home target (L1..L7, R1..R7, rad):")
            print("  " + ", ".join(f"{v:.4f}" for v in STACK_TWO_CONES_HOME))
            print("Per-joint distance to home (rad):")
            print("  " + ", ".join(f"{v:+.4f}" for v in errors))
            return 0

        graph_nodes = {name for name, _namespace in node.get_node_names_and_namespaces()}
        if "marvinpro_rollout_bridge" in graph_nodes:
            print(
                "marvinpro_rollout_bridge is running. Stop the rollout bridge before homing.",
                file=sys.stderr,
            )
            return 2

        if max(abs(e) for e in errors) <= SETTLE_POS_RAD:
            print("Arms already at the stack_two_cones home pose; skipping arm motion.")
            return 0

        robot_state = state["robot_state"]
        if robot_state is None:
            print(
                f"No message on {TOPIC_ROBOT_STATE}. Is the Apex robot service running?",
                file=sys.stderr,
            )
            return 2
        if robot_state == (0, 0):
            print(
                "robot_state is (0, 0); the arms are not ready. "
                "Finish Robot Ready in Apex first, then re-run this script.",
                file=sys.stderr,
            )
            return 2

        set_input = node.create_client(Int, SERVICE_SET_INPUT)
        movej = node.create_client(MoveJ, SERVICE_MOVEJ)
        for client, name in ((set_input, SERVICE_SET_INPUT), (movej, SERVICE_MOVEJ)):
            if not client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT_S):
                print(f"Service unavailable: {name}", file=sys.stderr)
                return 2

        previous_mode = state["input_mode"] if state["input_mode"] is not None else 0
        if previous_mode != PLANNER_INPUT_MODE:
            request = Int.Request()
            request.data = PLANNER_INPUT_MODE
            future = set_input.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_TIMEOUT_S)
            if not future.done() or not future.result().success:
                detail = future.result().message if future.done() else "service call timed out"
                print(f"Failed to switch input mode to planner (2): {detail}", file=sys.stderr)
                return 2
            mode_switched = True
            print(f"Input mode {previous_mode} -> planner (2); waiting {MODE_RAMP_S:g}s mux ramp.")
            ramp_deadline = time.monotonic() + MODE_RAMP_S
            while time.monotonic() < ramp_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

        movej_request = MoveJ.Request()
        movej_request.joint_values = list(STACK_TWO_CONES_HOME)
        future = movej.call_async(movej_request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_TIMEOUT_S)
        if not future.done() or not future.result().success:
            print("movej request was rejected by planner_joint_node.", file=sys.stderr)
            return 3
        print("movej accepted; waiting for the arms to settle...")

        timeout_s = move_timeout_s(current)
        move_deadline = time.monotonic() + timeout_s
        settled_since: float | None = None
        while time.monotonic() < move_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            position = state["position"]
            velocity = state["velocity"]
            if position is None:
                continue
            max_error = max(abs(c - t) for c, t in zip(position, STACK_TWO_CONES_HOME))
            max_velocity = max(velocity) if velocity is not None else 0.0
            if max_error <= SETTLE_POS_RAD and max_velocity <= SETTLE_VEL_RAD_S:
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= SETTLE_HOLD_S:
                    break
            else:
                settled_since = None
        else:
            position = state["position"]
            max_error = max(abs(c - t) for c, t in zip(position, STACK_TWO_CONES_HOME)) if position else float("nan")
            print(
                f"Timed out after {timeout_s:.1f}s waiting to settle (max error {max_error:.4f} rad). "
                f"robot_state={state['robot_state']}. Check Apex for faults before retrying.",
                file=sys.stderr,
            )
            return 3

        final = state["position"]
        max_error = max(abs(c - t) for c, t in zip(final, STACK_TWO_CONES_HOME))
        print(f"Arrived at stack_two_cones home pose (max joint error {max_error:.4f} rad).")
        return 0
    finally:
        if mode_switched:
            try:
                request = Int.Request()
                request.data = int(previous_mode)
                future = set_input.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_TIMEOUT_S)
                print(f"Input mode restored to {previous_mode}.")
            except Exception as exc:  # pragma: no cover - controller only
                print(f"WARNING: failed to restore input mode to {previous_mode}: {exc}", file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    result = _run(args.check)
    if result != 0 or args.check or args.keep_grippers:
        return result
    # The task start pose includes open grippers (the stack_two_cones teleop
    # episodes all start with normalized gripper ~0). Anything still held is
    # released at the home pose, after the arm motion has finished.
    print("Opening both grippers for the task start pose...")
    return gripper_control._run("0", "both", gripper_control.DEFAULT_DURATION_S)


if __name__ == "__main__":
    raise SystemExit(main())
