"""Bidirectional ROS 2 bridge for the Marvin Pro controller.

This module intentionally imports only rclpy, ROS messages, and the Python
standard library. Run it in the Apex environment on 6.6.7.100.
"""

from __future__ import annotations

import argparse
import math
import socket
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray, Int32

try:
    from marvin_msgs.msg import JointcmdArm
except ImportError as exc:  # pragma: no cover - only available on the controller
    raise SystemExit(
        "marvin_msgs is unavailable. Run after: source /etc/apex/apex_ros_env.sh"
    ) from exc

from .config import (
    CONTROL_HZ,
    CUSTOM_INPUT_MODE,
    DEFAULT_BRIDGE_PORT,
    JOINT_LOWER,
    JOINT_UPPER,
    READY_STATE,
    TOPIC_ARM_STATE,
    TOPIC_GRIPPER_CMD_L,
    TOPIC_GRIPPER_CMD_R,
    TOPIC_GRIPPER_FEEDBACK_L,
    TOPIC_GRIPPER_FEEDBACK_R,
    TOPIC_INPUT_MODE,
    TOPIC_JOINT_STATES,
    TOPIC_QUAD_IMAGE,
    TOPIC_ROBOT_STATE,
    TOPIC_USER_CMD_L,
    TOPIC_USER_CMD_R,
)
from .joint_mapping import JointMap, JointMapError
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
from .safety import SafetyError, validate_action


def _now() -> float:
    return time.monotonic()


def _age(now: float, stamp: float | None) -> float | None:
    return None if stamp is None else max(0.0, now - stamp)


QOS_SENSOR = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
QOS_RELIABLE = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
QOS_INPUT_MODE = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)
QOS_COMMAND = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class MarvinBridgeNode(Node):
    def __init__(
        self,
        *,
        allow_motion: bool,
        publish_hz: float,
        command_timeout_s: float,
        max_joint_step_rad: float,
        max_state_age_s: float,
        max_status_age_s: float,
        max_observation_lag: int,
        joint_limit_margin_rad: float,
    ) -> None:
        super().__init__("marvinpro_rollout_bridge")
        self.allow_motion = allow_motion
        self.publish_hz = publish_hz
        self.command_timeout_s = command_timeout_s
        self.max_joint_step_rad = max_joint_step_rad
        self.max_state_age_s = max_state_age_s
        self.max_status_age_s = max_status_age_s
        self.max_observation_lag = max_observation_lag
        self.joint_limit_margin_rad = joint_limit_margin_rad

        self._lock = threading.RLock()
        self._observation_ready = threading.Condition(self._lock)
        self._joint_map: JointMap | None = None
        self._joints: tuple[float, ...] | None = None
        self._joints_t: float | None = None
        self._gripper_l: float | None = None
        self._gripper_l_t: float | None = None
        self._gripper_r: float | None = None
        self._gripper_r_t: float | None = None
        self._input_mode: int | None = None
        self._input_mode_t: float | None = None
        self._robot_state: tuple[int, ...] | None = None
        self._robot_state_t: float | None = None
        self._arm_state: tuple[int, ...] | None = None
        self._arm_state_t: float | None = None
        self._latest_observation: RobotObservation | None = None
        self._seq = 0

        self._client_connected = False
        self._target: tuple[float, ...] | None = None
        self._target_t: float | None = None
        self._target_command_id: int | None = None
        self._last_command_id: int | None = None
        self._last_command_status = "no command"

        self.create_subscription(JointState, TOPIC_JOINT_STATES, self._on_joint_state, QOS_SENSOR)
        self.create_subscription(
            Float32MultiArray, TOPIC_GRIPPER_FEEDBACK_L, self._on_gripper_l, QOS_SENSOR
        )
        self.create_subscription(
            Float32MultiArray, TOPIC_GRIPPER_FEEDBACK_R, self._on_gripper_r, QOS_SENSOR
        )
        self.create_subscription(CompressedImage, TOPIC_QUAD_IMAGE, self._on_image, QOS_SENSOR)
        self.create_subscription(Int32, TOPIC_INPUT_MODE, self._on_input_mode, QOS_INPUT_MODE)
        self.create_subscription(Int16MultiArray, TOPIC_ROBOT_STATE, self._on_robot_state, QOS_SENSOR)
        self.create_subscription(Int16MultiArray, TOPIC_ARM_STATE, self._on_arm_state, QOS_SENSOR)

        self._pub_left = self.create_publisher(JointcmdArm, TOPIC_USER_CMD_L, QOS_COMMAND)
        self._pub_right = self.create_publisher(JointcmdArm, TOPIC_USER_CMD_R, QOS_COMMAND)
        self._pub_gripper_l = self.create_publisher(Float32, TOPIC_GRIPPER_CMD_L, QOS_RELIABLE)
        self._pub_gripper_r = self.create_publisher(Float32, TOPIC_GRIPPER_CMD_R, QOS_RELIABLE)
        self.create_timer(1.0 / publish_hz, self._publish_target)

        mode = "MOTION ENABLED" if allow_motion else "dry-run (motion disabled)"
        self.get_logger().info(f"bridge initialized at {publish_hz:.1f}Hz, {mode}")

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            if self._joint_map is None:
                try:
                    self._joint_map = JointMap.from_names(list(msg.name))
                    self.get_logger().info("canonical /joint_states mapping established")
                except JointMapError as exc:
                    self.get_logger().error(str(exc))
                    return
            try:
                self._joints = self._joint_map.canonical_positions(list(msg.position))
                self._joints_t = _now()
            except JointMapError as exc:
                self.get_logger().error(str(exc))

    def _on_gripper_l(self, msg: Float32MultiArray) -> None:
        if not msg.data:
            return
        value = float(msg.data[0])
        if not math.isfinite(value):
            return
        with self._lock:
            self._gripper_l = value
            self._gripper_l_t = _now()

    def _on_gripper_r(self, msg: Float32MultiArray) -> None:
        if not msg.data:
            return
        value = float(msg.data[0])
        if not math.isfinite(value):
            return
        with self._lock:
            self._gripper_r = value
            self._gripper_r_t = _now()

    def _on_input_mode(self, msg: Int32) -> None:
        with self._lock:
            self._input_mode = int(msg.data)
            self._input_mode_t = _now()

    def _on_robot_state(self, msg: Int16MultiArray) -> None:
        with self._lock:
            self._robot_state = tuple(int(value) for value in msg.data)
            self._robot_state_t = _now()

    def _on_arm_state(self, msg: Int16MultiArray) -> None:
        with self._lock:
            self._arm_state = tuple(int(value) for value in msg.data)
            self._arm_state_t = _now()

    def _readiness_gate_locked(self, now: float) -> tuple[bool, str]:
        if not self.allow_motion:
            return False, "bridge was started without --allow-motion"
        if not self._client_connected:
            return False, "no rollout client connected"
        if self._joints is None or _age(now, self._joints_t) is None:
            return False, "no joint state"
        if _age(now, self._joints_t) > self.max_state_age_s:
            return False, "joint state is stale"
        if self._gripper_l is None or self._gripper_r is None:
            return False, "no gripper feedback"
        if _age(now, self._gripper_l_t) > self.max_state_age_s:
            return False, "left gripper feedback is stale"
        if _age(now, self._gripper_r_t) > self.max_state_age_s:
            return False, "right gripper feedback is stale"
        if self._input_mode != CUSTOM_INPUT_MODE:
            return False, f"input_mode={self._input_mode}, expected {CUSTOM_INPUT_MODE} (Custom)"
        if self._robot_state != READY_STATE:
            return False, f"robot_state={self._robot_state}, expected {READY_STATE}"
        if self._arm_state != READY_STATE:
            return False, f"arm_state={self._arm_state}, expected {READY_STATE}"
        if _age(now, self._robot_state_t) is None or _age(now, self._robot_state_t) > self.max_status_age_s:
            return False, "robot_state is stale"
        if _age(now, self._arm_state_t) is None or _age(now, self._arm_state_t) > self.max_status_age_s:
            return False, "arm_state is stale"
        return True, "ready"

    def _on_image(self, msg: CompressedImage) -> None:
        now = _now()
        with self._observation_ready:
            if self._joints is None or self._gripper_l is None or self._gripper_r is None:
                return
            ready, reason = self._readiness_gate_locked(now)
            self._seq += 1
            self._latest_observation = RobotObservation(
                seq=self._seq,
                captured_monotonic=now,
                image=bytes(msg.data),
                image_format=str(msg.format),
                joints=self._joints,
                gripper_raw_left=self._gripper_l,
                gripper_raw_right=self._gripper_r,
                input_mode=self._input_mode,
                robot_state=self._robot_state,
                arm_state=self._arm_state,
                age_state_s=_age(now, self._joints_t),
                age_gripper_left_s=_age(now, self._gripper_l_t),
                age_gripper_right_s=_age(now, self._gripper_r_t),
                age_input_mode_s=_age(now, self._input_mode_t),
                age_robot_state_s=_age(now, self._robot_state_t),
                age_arm_state_s=_age(now, self._arm_state_t),
                motion_gate_open=ready,
                gate_reason=reason,
                last_command_id=self._last_command_id,
                last_command_status=self._last_command_status,
            )
            self._observation_ready.notify_all()

    def client_connected(self) -> None:
        with self._lock:
            self._client_connected = True
            self._clear_target_locked("client connected; waiting for action")

    def client_disconnected(self) -> None:
        with self._lock:
            self._client_connected = False
            self._clear_target_locked("client disconnected; command publication stopped")

    def _clear_target_locked(self, status: str) -> None:
        self._target = None
        self._target_t = None
        self._target_command_id = None
        self._last_command_status = status

    def accept_command(self, message) -> None:
        now = _now()
        with self._lock:
            if isinstance(message, StopCommand):
                try:
                    require_current_version(message)
                except ProtocolError as exc:
                    self._clear_target_locked(f"rejected stop: {exc}")
                    return
                self._clear_target_locked(f"stopped: {message.reason}")
                return
            if not isinstance(message, ActionCommand):
                self._clear_target_locked(f"rejected unexpected message {type(message).__name__}")
                return

            self._last_command_id = message.command_id
            try:
                require_current_version(message)
                if not message.execute:
                    raise SafetyError("execute flag is false")
                ready, reason = self._readiness_gate_locked(now)
                if not ready:
                    raise SafetyError(reason)
                if self._latest_observation is None:
                    raise SafetyError("no camera observation")
                lag = self._latest_observation.seq - int(message.observation_seq)
                if lag < 0 or lag > self.max_observation_lag:
                    raise SafetyError(
                        f"action observation lag is {lag} frames (limit {self.max_observation_lag})"
                    )
                assert self._joints is not None
                target = validate_action(
                    message.action,
                    self._joints,
                    max_joint_step_rad=self.max_joint_step_rad,
                    joint_limit_margin_rad=self.joint_limit_margin_rad,
                )
            except (ProtocolError, SafetyError, TypeError, ValueError) as exc:
                self._clear_target_locked(f"rejected command {message.command_id}: {exc}")
                return

            self._target = target
            self._target_t = now
            self._target_command_id = message.command_id
            self._last_command_status = f"accepted command {message.command_id}"

    def _publish_target(self) -> None:
        now = _now()
        with self._lock:
            ready, reason = self._readiness_gate_locked(now)
            if not ready:
                if self._target is not None:
                    self._clear_target_locked(f"publication blocked: {reason}")
                return
            if self._target is None or self._target_t is None:
                return
            if now - self._target_t > self.command_timeout_s:
                self._clear_target_locked("command timed out; publication stopped")
                return
            assert self._joints is not None
            try:
                target = validate_action(
                    self._target,
                    self._joints,
                    max_joint_step_rad=self.max_joint_step_rad,
                    joint_limit_margin_rad=self.joint_limit_margin_rad,
                )
            except SafetyError as exc:
                self._clear_target_locked(f"publication safety check failed: {exc}")
                return
            command_id = self._target_command_id

        left = JointcmdArm()
        left.header.stamp = self.get_clock().now().to_msg()
        left.positions = list(target[:7])
        right = JointcmdArm()
        right.header.stamp = left.header.stamp
        right.positions = list(target[8:15])
        gripper_l = Float32(data=float(target[7]))
        gripper_r = Float32(data=float(target[15]))
        self._pub_left.publish(left)
        self._pub_right.publish(right)
        self._pub_gripper_l.publish(gripper_l)
        self._pub_gripper_r.publish(gripper_r)
        with self._lock:
            self._last_command_status = f"published command {command_id}"

    def wait_for_observation(
        self, last_seq: int, stop: threading.Event, timeout_s: float = 1.0
    ) -> RobotObservation | None:
        deadline = _now() + timeout_s
        with self._observation_ready:
            while not stop.is_set():
                observation = self._latest_observation
                if observation is not None and observation.seq > last_seq:
                    return observation
                remaining = deadline - _now()
                if remaining <= 0:
                    return None
                self._observation_ready.wait(timeout=remaining)
        return None

    def latest_observation_seq(self) -> int:
        with self._lock:
            return -1 if self._latest_observation is None else self._latest_observation.seq


class DoctorNode(Node):
    def __init__(self) -> None:
        super().__init__("marvinpro_rollout_doctor")
        self.counts: dict[str, int] = {}
        self.latest: dict[str, object] = {}

        def subscribe(msg_type, topic, value_fn, qos=QOS_SENSOR):
            self.counts[topic] = 0

            def callback(msg):
                self.counts[topic] += 1
                self.latest[topic] = value_fn(msg)

            self.create_subscription(msg_type, topic, callback, qos)

        subscribe(JointState, TOPIC_JOINT_STATES, lambda msg: (list(msg.name), list(msg.position)))
        subscribe(Float32MultiArray, TOPIC_GRIPPER_FEEDBACK_L, lambda msg: list(msg.data))
        subscribe(Float32MultiArray, TOPIC_GRIPPER_FEEDBACK_R, lambda msg: list(msg.data))
        subscribe(CompressedImage, TOPIC_QUAD_IMAGE, lambda msg: (msg.format, len(msg.data)))
        subscribe(Int32, TOPIC_INPUT_MODE, lambda msg: int(msg.data), QOS_INPUT_MODE)
        subscribe(Int16MultiArray, TOPIC_ROBOT_STATE, lambda msg: tuple(int(v) for v in msg.data))
        subscribe(Int16MultiArray, TOPIC_ARM_STATE, lambda msg: tuple(int(v) for v in msg.data))


def doctor(duration_s: float) -> int:
    rclpy.init()
    node = DoctorNode()
    started = _now()
    try:
        while _now() - started < duration_s:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = max(1e-6, _now() - started)
    print(f"Marvin Pro rollout doctor ({elapsed:.1f}s, read-only)")
    failed = False
    for topic, count in node.counts.items():
        value = node.latest.get(topic)
        print(f"  {topic}: {count} msgs, {count / elapsed:.1f}Hz, latest={value}")
        if count == 0:
            failed = True
    joint_value = node.latest.get(TOPIC_JOINT_STATES)
    if joint_value is not None:
        try:
            JointMap.from_names(joint_value[0])
            print("  joint mapping: OK")
        except JointMapError as exc:
            failed = True
            print(f"  joint mapping: FAILED: {exc}")
    print("  expected before execution: input_mode=3, robot_state=(3, 3), arm_state=(3, 3)")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


def _serve_client(conn: socket.socket, address, node: MarvinBridgeNode) -> None:
    stop = threading.Event()
    node.client_connected()
    # Skip the cached frame captured before client_connected() opened that part
    # of the readiness gate. The first frame sent to a client reflects the
    # current connection state.
    last_seq = node.latest_observation_seq()
    print(f"[bridge] rollout client connected from {address[0]}:{address[1]}", flush=True)
    try:
        send_message(
            conn,
            BridgeHello(
                motion_allowed=node.allow_motion,
                publish_hz=node.publish_hz,
                max_joint_step_rad=node.max_joint_step_rad,
                joint_lower=JOINT_LOWER,
                joint_upper=JOINT_UPPER,
            ),
        )

        def receive_commands() -> None:
            try:
                while not stop.is_set():
                    message = recv_message(conn)
                    if message is None:
                        break
                    node.accept_command(message)
            except (ConnectionError, OSError, ProtocolError) as exc:
                print(f"[bridge] command receiver stopped: {exc}", flush=True)
            finally:
                stop.set()

        receiver = threading.Thread(target=receive_commands, daemon=True)
        receiver.start()
        while not stop.is_set():
            observation = node.wait_for_observation(last_seq, stop)
            if observation is None:
                continue
            send_message(conn, observation)
            last_seq = observation.seq
    except (BrokenPipeError, ConnectionResetError, OSError, ProtocolError) as exc:
        print(f"[bridge] observation sender stopped: {exc}", flush=True)
    finally:
        stop.set()
        node.client_disconnected()
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        conn.close()
        print("[bridge] rollout client disconnected; publishing is stopped", flush=True)


def serve(args: argparse.Namespace) -> int:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((args.bind_host, args.port))
        server.listen(1)
        server.settimeout(1.0)
    except OSError as exc:
        server.close()
        print(f"[bridge] cannot listen on {args.bind_host}:{args.port}: {exc}", flush=True)
        return 2

    rclpy.init()
    node = MarvinBridgeNode(
        allow_motion=args.allow_motion,
        publish_hz=args.publish_hz,
        command_timeout_s=args.command_timeout,
        max_joint_step_rad=args.max_joint_step_rad,
        max_state_age_s=args.max_state_age,
        max_status_age_s=args.max_status_age,
        max_observation_lag=args.max_observation_lag,
        joint_limit_margin_rad=args.joint_limit_margin_rad,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(
        f"[bridge] listening on {args.bind_host}:{args.port}; "
        f"motion_allowed={args.allow_motion} (Ctrl+C to stop)",
        flush=True,
    )
    try:
        while rclpy.ok():
            try:
                conn, address = server.accept()
            except socket.timeout:
                continue
            _serve_client(conn, address, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client_disconnected()
        server.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--doctor", action="store_true", help="read-only ROS topic preflight, then exit")
    parser.add_argument("--duration", type=float, default=5.0, help="doctor duration in seconds")
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_BRIDGE_PORT)
    parser.add_argument("--allow-motion", action="store_true", help="permit publishing after all other gates pass")
    parser.add_argument("--publish-hz", type=float, default=CONTROL_HZ)
    parser.add_argument("--command-timeout", type=float, default=0.25)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.12)
    parser.add_argument("--joint-limit-margin-rad", type=float, default=0.02)
    parser.add_argument("--max-state-age", type=float, default=0.20)
    parser.add_argument("--max-status-age", type=float, default=0.50)
    parser.add_argument("--max-observation-lag", type=int, default=8)
    args = parser.parse_args(argv)
    if args.publish_hz <= 0 or args.command_timeout <= 0 or args.duration <= 0:
        parser.error("rates, timeouts, and duration must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.doctor:
        return doctor(args.duration)
    return serve(args)


if __name__ == "__main__":
    raise SystemExit(main())
