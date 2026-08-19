"""Bidirectional ROS 2 bridge for the Marvin Pro controller.

This module intentionally imports only rclpy, ROS messages, and the Python
standard library. Run it in the Apex environment on 6.6.7.100.
"""

from __future__ import annotations

import argparse
from collections import deque
import math
import socket
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
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
    JOINT_NAMES,
    JOINT_UPPER,
    JOINT_VELOCITY,
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
from .joint_mapping import JointMap, JointMapError, build_state16, normalize_gripper
from .protocol import (
    ActionCommand,
    BridgeHello,
    HoldPositionCommand,
    LatchMeasuredHoldCommand,
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
from .rtc import RTC_EXECUTION_HORIZON, RTC_HORIZON
from .safety import SafetyError, action_arms, filter_action, validate_action
from .tracking import TrackingGovernor
from .trajectory_timeline import TrajectoryTimeline


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

TRAJECTORY_CLIENT_ENVELOPE_RAD = 0.16


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
        trajectory_state_timeout_s: float,
        trajectory_timer_timeout_s: float,
        trajectory_heartbeat_timeout_s: float,
        tracking_run_error_rad: float,
        tracking_resume_error_rad: float,
        tracking_stop_error_rad: float,
        tracking_tolerance_rad: float,
        tracking_settle_seconds: float,
        rtc_blend_max_velocity_rad_s: float,
        rtc_blend_max_acceleration_rad_s2: float,
        rtc_blend_max_jerk_rad_s3: float,
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
        self.trajectory_state_timeout_s = trajectory_state_timeout_s
        self.trajectory_timer_timeout_s = trajectory_timer_timeout_s
        self.trajectory_heartbeat_timeout_s = trajectory_heartbeat_timeout_s
        self.tracking_tolerance_rad = tracking_tolerance_rad
        self.tracking_settle_seconds = tracking_settle_seconds
        self.rtc_blend_max_velocity_rad_s = rtc_blend_max_velocity_rad_s
        self.rtc_blend_max_acceleration_rad_s2 = rtc_blend_max_acceleration_rad_s2
        self.rtc_blend_max_jerk_rad_s3 = rtc_blend_max_jerk_rad_s3
        self._governor = TrackingGovernor(
            run_error_rad=tracking_run_error_rad,
            resume_error_rad=tracking_resume_error_rad,
            stop_error_rad=tracking_stop_error_rad,
        )

        self._lock = threading.RLock()
        self._observation_ready = threading.Condition(self._lock)
        self._outbound_ready = threading.Condition(self._lock)
        self._joint_map: JointMap | None = None
        self._joints: tuple[float, ...] | None = None
        self._joints_t: float | None = None
        # Policy state uses normalized measured q from the DM feedback topics.
        self._gripper_l: float | None = None
        self._gripper_r: float | None = None
        self._gripper_command_l: float | None = None
        self._gripper_command_r: float | None = None
        self._gripper_feedback_l_position: float | None = None
        self._gripper_feedback_r_position: float | None = None
        self._gripper_l_t: float | None = None
        self._gripper_r_t: float | None = None
        self._gripper_l_velocity: float | None = None
        self._gripper_r_velocity: float | None = None
        self._gripper_l_torque: float | None = None
        self._gripper_r_torque: float | None = None
        self._gripper_l_mos_temperature: float | None = None
        self._gripper_r_mos_temperature: float | None = None
        self._gripper_l_motor_temperature: float | None = None
        self._gripper_r_motor_temperature: float | None = None
        self._input_mode: int | None = None
        self._input_mode_t: float | None = None
        self._robot_state: tuple[int, ...] | None = None
        self._robot_state_t: float | None = None
        self._arm_state: tuple[int, ...] | None = None
        self._arm_state_t: float | None = None
        self._latest_observation: RobotObservation | None = None
        self._seq = 0
        self._latest_state: RobotStateUpdate | None = None
        self._state_seq = 0
        self._events: deque[TrajectoryEvent] = deque()
        self._event_seq = 0

        self._client_connected = False
        self._target: tuple[float, ...] | None = None
        self._target_t: float | None = None
        self._target_command_id: int | None = None
        self._last_command_id: int | None = None
        self._last_command_status = "no command"

        self._trajectory_session_id: str | None = None
        self._trajectory_plan_id: str | None = None
        self._timeline: TrajectoryTimeline | None = None
        self._timeline_version = 0
        self._phase: float | None = None
        self._phase_rate = 0.0
        self._handoff_anchor: tuple[float, ...] | None = None
        self._handoff_phase: float | None = None
        self._checkpoint_consumed = False
        self._trajectory_paused = False
        self._pause_kind: str | None = None
        self._checkpoint_id = 0
        self._checkpoint_stable_since: float | None = None
        self._checkpoint_emitted = False
        self._continuous_checkpoint = False
        self._heartbeat_t: float | None = None
        self._last_trajectory_tick: float | None = None
        self._raw_reference: tuple[float, ...] | None = None
        self._sent_target: tuple[float, ...] | None = None
        self._tracking_error_rad: float | None = None
        self._servo_error_rad: float | None = None
        self._arm_clipped = False
        self._frozen_reason: str | None = None
        self._active_request_id: str | None = None
        self._active_predicted_delay: int | None = None
        self._late_result_policy = "discard"
        self._actual_delay_steps = 0
        self._inference_invalid = False
        self._pending_rtc: StageRtcChunkCommand | None = None
        self._trajectory_hold_action: tuple[float, ...] | None = None
        self._trajectory_loaded_monotonic: float | None = None
        self._trajectory_deadline_monotonic: float | None = None

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
        self._publish_tick_count = 0
        self._publish_late_tick_count = 0
        self._publish_max_gap_s = 0.0
        self._publish_first_tick_s: float | None = None
        self._publish_last_tick_s: float | None = None
        self.create_timer(1.0 / publish_hz, self._publish_target)

        mode = "MOTION ENABLED" if allow_motion else "dry-run (motion disabled)"
        self.get_logger().info(f"bridge initialized at {publish_hz:.1f}Hz, {mode}")
        self.get_logger().info("gripper policy state uses normalized measured DM feedback")

    def _on_joint_state(self, msg: JointState) -> None:
        now = _now()
        with self._lock:
            if self._joint_map is None:
                try:
                    self._joint_map = JointMap.from_names(list(msg.name))
                    self.get_logger().info("canonical /tj/joint_states mapping established")
                except JointMapError as exc:
                    self.get_logger().error(str(exc))
                    return
            try:
                self._joints = self._joint_map.canonical_positions(list(msg.position))
                self._joints_t = now
                self._refresh_state_locked(now)
            except JointMapError as exc:
                self.get_logger().error(str(exc))

    @staticmethod
    def _gripper_feedback_values(msg: Float32MultiArray) -> tuple[float, ...] | None:
        if len(msg.data) < 3:
            return None
        try:
            values = tuple(float(value) for value in msg.data[:5])
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return values

    def _on_gripper_l(self, msg: Float32MultiArray) -> None:
        values = self._gripper_feedback_values(msg)
        if values is None:
            return
        now = _now()
        with self._lock:
            self._gripper_feedback_l_position, self._gripper_l_velocity, self._gripper_l_torque = values[:3]
            self._gripper_l = normalize_gripper(self._gripper_feedback_l_position)
            self._gripper_l_mos_temperature = values[3] if len(values) > 3 else None
            self._gripper_l_motor_temperature = values[4] if len(values) > 4 else None
            self._gripper_l_t = now

    def _on_gripper_r(self, msg: Float32MultiArray) -> None:
        values = self._gripper_feedback_values(msg)
        if values is None:
            return
        now = _now()
        with self._lock:
            self._gripper_feedback_r_position, self._gripper_r_velocity, self._gripper_r_torque = values[:3]
            self._gripper_r = normalize_gripper(self._gripper_feedback_r_position)
            self._gripper_r_mos_temperature = values[3] if len(values) > 3 else None
            self._gripper_r_motor_temperature = values[4] if len(values) > 4 else None
            self._gripper_r_t = now

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
        if self._gripper_l is None or _age(now, self._gripper_l_t) is None:
            return False, "no left gripper feedback"
        if self._gripper_r is None or _age(now, self._gripper_r_t) is None:
            return False, "no right gripper feedback"
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

    def _trajectory_mode_locked(self) -> str:
        if self._trajectory_session_id is None:
            return "legacy"
        if self._trajectory_hold_action is not None:
            return "hold"
        return "trajectory"

    def _refresh_state_locked(self, now: float) -> None:
        if self._joints is None or self._gripper_l is None or self._gripper_r is None:
            return
        assert self._joints_t is not None
        ready, reason = self._readiness_gate_locked(now)
        self._state_seq += 1
        self._latest_state = RobotStateUpdate(
            state_seq=self._state_seq,
            # Timer-driven phase/status updates must not masquerade as new robot
            # feedback. This timestamp advances only in the joint callback.
            sampled_monotonic=self._joints_t,
            joints=self._joints,
            gripper_raw_left=self._gripper_l,
            gripper_raw_right=self._gripper_r,
            motion_gate_open=ready,
            gate_reason=reason,
            last_command_id=self._last_command_id,
            last_command_status=self._last_command_status,
            trajectory_mode=self._trajectory_mode_locked(),
            session_id=self._trajectory_session_id,
            plan_id=self._trajectory_plan_id,
            timeline_version=self._timeline_version,
            phase=self._phase,
            phase_rate=self._phase_rate,
            raw_reference=self._raw_reference,
            sent_target=self._sent_target,
            tracking_error_rad=self._tracking_error_rad,
            servo_error_rad=self._servo_error_rad,
            arm_clipped=self._arm_clipped,
            frozen_reason=self._frozen_reason,
            active_request_id=self._active_request_id,
            gripper_velocity_left=self._gripper_l_velocity,
            gripper_velocity_right=self._gripper_r_velocity,
            gripper_torque_left=self._gripper_l_torque,
            gripper_torque_right=self._gripper_r_torque,
            gripper_mos_temperature_left=self._gripper_l_mos_temperature,
            gripper_mos_temperature_right=self._gripper_r_mos_temperature,
            gripper_motor_temperature_left=self._gripper_l_motor_temperature,
            gripper_motor_temperature_right=self._gripper_r_motor_temperature,
            gripper_position_raw_left=self._gripper_feedback_l_position,
            gripper_position_raw_right=self._gripper_feedback_r_position,
        )
        self._outbound_ready.notify_all()

    def _emit_event_locked(
        self,
        event_type: str,
        now: float,
        *,
        session_id: str | None = None,
        plan_id: str | None = None,
        timeline_version: int | None = None,
        checkpoint_id: int | None = None,
        stable_monotonic: float | None = None,
        observation_seq_at_stable: int | None = None,
        old_remaining_actions_absolute: tuple[tuple[float, ...], ...] = (),
        request_id: str | None = None,
        predicted_delay_steps: int | None = None,
        actual_delay_steps: int | None = None,
        boundary_old_velocity: tuple[float, ...] = (),
        boundary_new_velocity: tuple[float, ...] = (),
        boundary_velocity_jump_rad: float | None = None,
        boundary_acceleration_jump_rad: float | None = None,
        blend_duration_knots: int | None = None,
        blend_max_velocity_rad_s: float | None = None,
        blend_max_acceleration_rad_s2: float | None = None,
        blend_max_jerk_rad_s3: float | None = None,
        reason_code: str | None = None,
        deadline_monotonic: float | None = None,
        elapsed_s: float | None = None,
        worst_joint: str | None = None,
        final_error_rad: float | None = None,
        detail: str = "",
    ) -> None:
        resolved_session_id = session_id or self._trajectory_session_id
        resolved_plan_id = plan_id or self._trajectory_plan_id
        if resolved_session_id is None or resolved_plan_id is None:
            return
        resolved_timeline_version = self._timeline_version if timeline_version is None else timeline_version
        settle_duration_s = None
        if stable_monotonic is not None and self._checkpoint_stable_since is not None:
            settle_duration_s = max(0.0, stable_monotonic - self._checkpoint_stable_since)
        self._event_seq += 1
        event = TrajectoryEvent(
            event_seq=self._event_seq,
            event_type=event_type,
            emitted_monotonic=now,
            session_id=resolved_session_id,
            plan_id=resolved_plan_id,
            timeline_version=resolved_timeline_version,
            phase=0.0 if self._phase is None else self._phase,
            checkpoint_id=checkpoint_id,
            stable_monotonic=stable_monotonic,
            observation_seq_at_stable=observation_seq_at_stable,
            old_remaining_actions_absolute=old_remaining_actions_absolute,
            request_id=request_id,
            predicted_delay_steps=predicted_delay_steps,
            actual_delay_steps=actual_delay_steps,
            detail=detail,
            tracking_error_rad=self._tracking_error_rad,
            servo_error_rad=self._servo_error_rad,
            joint_source_monotonic=self._joints_t,
            settle_duration_s=settle_duration_s,
            raw_reference=self._raw_reference,
            sent_target=self._sent_target,
            arm_clipped=self._arm_clipped,
            frozen_reason=self._frozen_reason,
            boundary_old_velocity=boundary_old_velocity,
            boundary_new_velocity=boundary_new_velocity,
            boundary_velocity_jump_rad=boundary_velocity_jump_rad,
            boundary_acceleration_jump_rad=boundary_acceleration_jump_rad,
            continuous_checkpoint=getattr(self, "_continuous_checkpoint", False),
            blend_duration_knots=blend_duration_knots,
            blend_max_velocity_rad_s=blend_max_velocity_rad_s,
            blend_max_acceleration_rad_s2=blend_max_acceleration_rad_s2,
            blend_max_jerk_rad_s3=blend_max_jerk_rad_s3,
            reason_code=reason_code,
            deadline_monotonic=deadline_monotonic,
            elapsed_s=elapsed_s,
            worst_joint=worst_joint,
            final_error_rad=final_error_rad,
        )
        self._events.append(event)
        self.get_logger().info(
            f"trajectory_event type={event.event_type} event_seq={event.event_seq} "
            f"session={event.session_id} plan={event.plan_id} version={event.timeline_version} "
            f"phase={event.phase:.6f} checkpoint={event.checkpoint_id} request={event.request_id} "
            f"d_pred={event.predicted_delay_steps} d_actual={event.actual_delay_steps} "
            f"tracking_error={event.tracking_error_rad} servo_error={event.servo_error_rad} "
            f"settle_s={event.settle_duration_s} joint_source={event.joint_source_monotonic} "
            f"arm_clipped={event.arm_clipped} frozen={event.frozen_reason} "
            f"continuous_checkpoint={event.continuous_checkpoint} "
            f"boundary_old_velocity={event.boundary_old_velocity} "
            f"boundary_new_velocity={event.boundary_new_velocity} "
            f"boundary_velocity_jump_rad={event.boundary_velocity_jump_rad} "
            f"boundary_acceleration_jump_rad={event.boundary_acceleration_jump_rad} "
            f"blend_duration_knots={event.blend_duration_knots} "
            f"blend_max_velocity_rad_s={event.blend_max_velocity_rad_s} "
            f"blend_max_acceleration_rad_s2={event.blend_max_acceleration_rad_s2} "
            f"blend_max_jerk_rad_s3={event.blend_max_jerk_rad_s3} "
            f"reason_code={event.reason_code} deadline={event.deadline_monotonic} "
            f"elapsed_s={event.elapsed_s} worst_joint={event.worst_joint} "
            f"final_error_rad={event.final_error_rad} "
            f"raw_reference={event.raw_reference} sent_target={event.sent_target} detail={event.detail!r}"
        )
        if len(self._events) > 128:
            self._clear_trajectory_locked("trajectory event queue overflow", now, emit_event=False)
        self._outbound_ready.notify_all()

    def _clear_trajectory_locked(self, status: str, now: float, *, emit_event: bool = True) -> None:
        if emit_event:
            self._emit_event_locked("trajectory_stopped", now, detail=status)
        self._trajectory_session_id = None
        self._trajectory_plan_id = None
        self._timeline = None
        self._phase = None
        self._phase_rate = 0.0
        self._handoff_anchor = None
        self._handoff_phase = None
        self._checkpoint_consumed = False
        self._trajectory_paused = False
        self._pause_kind = None
        self._checkpoint_stable_since = None
        self._checkpoint_emitted = False
        self._continuous_checkpoint = False
        self._heartbeat_t = None
        self._raw_reference = None
        self._sent_target = None
        self._tracking_error_rad = None
        self._servo_error_rad = None
        self._arm_clipped = False
        self._frozen_reason = None
        self._active_request_id = None
        self._active_predicted_delay = None
        self._late_result_policy = "discard"
        self._actual_delay_steps = 0
        self._inference_invalid = False
        self._pending_rtc = None
        self._trajectory_hold_action = None
        self._trajectory_loaded_monotonic = None
        self._trajectory_deadline_monotonic = None
        self._governor.reset()
        self._clear_target_locked(status)

    def next_outbound(
        self,
        *,
        last_state_seq: int,
        last_observation_seq: int,
        prefer_observation: bool,
        stop: threading.Event,
        timeout_s: float = 1.0,
    ) -> object | None:
        deadline = _now() + timeout_s
        with self._outbound_ready:
            while not stop.is_set():
                if self._events:
                    return self._events.popleft()
                observation_ready = (
                    self._latest_observation is not None and self._latest_observation.seq > last_observation_seq
                )
                state_ready = self._latest_state is not None and self._latest_state.state_seq > last_state_seq
                if prefer_observation and observation_ready:
                    return self._latest_observation
                if state_ready:
                    return self._latest_state
                if observation_ready:
                    return self._latest_observation
                remaining = deadline - _now()
                if remaining <= 0:
                    return None
                self._outbound_ready.wait(timeout=remaining)
        return None

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
                gripper_velocity_left=self._gripper_l_velocity,
                gripper_velocity_right=self._gripper_r_velocity,
                gripper_torque_left=self._gripper_l_torque,
                gripper_torque_right=self._gripper_r_torque,
                gripper_mos_temperature_left=self._gripper_l_mos_temperature,
                gripper_mos_temperature_right=self._gripper_r_mos_temperature,
                gripper_motor_temperature_left=self._gripper_l_motor_temperature,
                gripper_motor_temperature_right=self._gripper_r_motor_temperature,
                gripper_position_raw_left=self._gripper_feedback_l_position,
                gripper_position_raw_right=self._gripper_feedback_r_position,
                extra={
                    "state_seq": self._state_seq,
                    "state_sampled_monotonic": self._joints_t,
                    "gripper_state_source": "measured_feedback",
                    "gripper_command_left": self._gripper_command_l,
                    "gripper_command_right": self._gripper_command_r,
                },
            )
            self._observation_ready.notify_all()
            self._outbound_ready.notify_all()

    def client_connected(self) -> None:
        with self._lock:
            self._events.clear()
            self._client_connected = True
            self._clear_target_locked("client connected; waiting for action")
            self._refresh_state_locked(_now())

    def client_disconnected(self) -> None:
        with self._lock:
            self._client_connected = False
            now = _now()
            if self._trajectory_session_id is not None:
                self._clear_trajectory_locked(
                    "client disconnected; command publication stopped", now, emit_event=False
                )
            else:
                self._clear_target_locked("client disconnected; command publication stopped")
            self._refresh_state_locked(now)

    def _clear_target_locked(self, status: str) -> None:
        self._target = None
        self._target_t = None
        self._target_command_id = None
        self._last_command_status = status

    def _check_observation_lag_locked(self, observation_seq: int) -> None:
        if self._latest_observation is None:
            raise SafetyError("no camera observation")
        lag = self._latest_observation.seq - int(observation_seq)
        if lag < 0 or lag > self.max_observation_lag:
            raise SafetyError(f"action observation lag is {lag} frames (limit {self.max_observation_lag})")

    def _validate_trajectory_knots_locked(self, timeline: TrajectoryTimeline) -> None:
        for knot in timeline.knots:
            validate_action(
                knot,
                action_arms(knot),
                max_joint_step_rad=self.max_joint_step_rad,
                joint_limit_margin_rad=self.joint_limit_margin_rad,
            )

    def _validate_rtc_blend_locked(
        self, timeline: TrajectoryTimeline
    ) -> tuple[float, float, float]:
        blend = timeline.blend
        if blend is None:
            raise SafetyError("RTC replacement is missing its C1/C2 blend")
        duration_s = blend.duration_phases / timeline.knot_hz
        sample_count = max(1, int(math.ceil(duration_s * self.publish_hz)))
        max_velocity = 0.0
        max_acceleration = 0.0
        max_jerk = 0.0
        for sample_index in range(sample_count + 1):
            fraction = sample_index / sample_count
            phase = blend.start_phase + blend.duration_phases * fraction
            position, velocity_phase, acceleration_phase, jerk_phase = blend.kinematics(phase)
            validate_action(
                position,
                action_arms(position),
                max_joint_step_rad=self.max_joint_step_rad,
                joint_limit_margin_rad=self.joint_limit_margin_rad,
            )
            arm_velocity = tuple(abs(value) * timeline.knot_hz for value in action_arms(velocity_phase))
            arm_acceleration = tuple(
                abs(value) * timeline.knot_hz**2 for value in action_arms(acceleration_phase)
            )
            arm_jerk = tuple(abs(value) * timeline.knot_hz**3 for value in action_arms(jerk_phase))
            velocity = max(arm_velocity)
            acceleration = max(arm_acceleration)
            jerk = max(arm_jerk)
            max_velocity = max(max_velocity, velocity)
            max_acceleration = max(max_acceleration, acceleration)
            max_jerk = max(max_jerk, jerk)
            for joint_index, (value, urdf_limit) in enumerate(zip(arm_velocity, JOINT_VELOCITY)):
                limit = min(urdf_limit, self.rtc_blend_max_velocity_rad_s)
                if value > limit + 1e-9:
                    raise SafetyError(
                        f"RTC blend joint {joint_index} velocity {value:.5f}rad/s exceeds "
                        f"{limit:.5f}rad/s at 100Hz sample {sample_index}/{sample_count}"
                    )
            if acceleration > self.rtc_blend_max_acceleration_rad_s2 + 1e-9:
                raise SafetyError(
                    f"RTC blend acceleration {acceleration:.5f}rad/s^2 exceeds "
                    f"{self.rtc_blend_max_acceleration_rad_s2:.5f}rad/s^2 at 100Hz sample "
                    f"{sample_index}/{sample_count}"
                )
            if jerk > self.rtc_blend_max_jerk_rad_s3 + 1e-9:
                raise SafetyError(
                    f"RTC blend jerk {jerk:.5f}rad/s^3 exceeds "
                    f"{self.rtc_blend_max_jerk_rad_s3:.5f}rad/s^3 at 100Hz sample "
                    f"{sample_index}/{sample_count}"
                )
        return max_velocity, max_acceleration, max_jerk

    def _measured_hold_action_locked(self) -> tuple[float, ...]:
        assert self._joints is not None and self._gripper_l is not None and self._gripper_r is not None
        previous = self._sent_target or self._trajectory_hold_action
        gripper_l = self._gripper_l if previous is None else previous[7]
        gripper_r = self._gripper_r if previous is None else previous[15]
        return build_state16(self._joints, gripper_l, gripper_r)

    def _tracking_diag_locked(
        self, reference: tuple[float, ...] | None = None
    ) -> tuple[float | None, str | None]:
        if self._joints is None or (reference or self._raw_reference) is None:
            return None, None
        target = reference or self._raw_reference
        assert target is not None
        errors = tuple(abs(expected - actual) for expected, actual in zip(action_arms(target), self._joints))
        worst_index = max(range(len(errors)), key=errors.__getitem__)
        return errors[worst_index], JOINT_NAMES[worst_index]

    def _enter_hold_locked(
        self,
        action: tuple[float, ...],
        now: float,
        *,
        command_id: int,
        event_type: str,
        reason: str,
        reason_code: str,
        deadline_monotonic: float | None = None,
        elapsed_s: float | None = None,
        worst_joint: str | None = None,
        final_error_rad: float | None = None,
    ) -> None:
        self._timeline = None
        self._trajectory_hold_action = action
        self._timeline_version += 1
        self._phase = None
        self._phase_rate = 0.0
        self._handoff_anchor = None
        self._handoff_phase = None
        self._checkpoint_consumed = False
        self._trajectory_paused = True
        self._pause_kind = "hold"
        self._checkpoint_stable_since = None
        self._checkpoint_emitted = False
        self._continuous_checkpoint = False
        self._heartbeat_t = now
        self._raw_reference = action
        self._sent_target = action
        self._tracking_error_rad = 0.0
        self._servo_error_rad = 0.0
        self._arm_clipped = False
        self._frozen_reason = "hold"
        self._active_request_id = None
        self._active_predicted_delay = None
        self._late_result_policy = "discard"
        self._actual_delay_steps = 0
        self._inference_invalid = False
        self._pending_rtc = None
        self._trajectory_loaded_monotonic = None
        self._trajectory_deadline_monotonic = None
        self._governor.reset()
        self._last_command_id = command_id
        self._last_command_status = f"holding fixed position: {reason}"
        self._emit_event_locked(
            event_type,
            now,
            reason_code=reason_code,
            deadline_monotonic=deadline_monotonic,
            elapsed_s=elapsed_s,
            worst_joint=worst_joint,
            final_error_rad=final_error_rad,
            detail=reason,
        )

    def _accept_load_trajectory_locked(self, message: LoadTrajectoryCommand, now: float) -> None:
        require_current_version(message)
        if not message.execute:
            raise SafetyError("execute flag is false")
        if abs(self.publish_hz - 100.0) > 1e-6:
            raise SafetyError("trajectory mode requires bridge --publish-hz 100")
        ready, reason = self._readiness_gate_locked(now)
        if not ready:
            raise SafetyError(reason)
        self._check_observation_lag_locked(message.observation_seq)
        if self._trajectory_session_id is not None and message.session_id != self._trajectory_session_id:
            raise SafetyError("another trajectory session is active")
        if message.expected_timeline_version != self._timeline_version:
            raise SafetyError(
                f"timeline version mismatch: expected {message.expected_timeline_version}, current {self._timeline_version}"
            )
        if message.chunk_timeout_s is not None and message.chunk_timeout_s <= 0:
            raise SafetyError("chunk timeout must be positive")
        base_timeline = TrajectoryTimeline(message.knots, message.knot_hz, message.checkpoint_horizon)
        self._validate_trajectory_knots_locked(base_timeline)
        if base_timeline.horizon != RTC_HORIZON:
            raise SafetyError(f"trajectory mode currently requires horizon {RTC_HORIZON}")
        continuous_checkpoint = bool(getattr(message, "continuous_checkpoint", False))
        if continuous_checkpoint and base_timeline.checkpoint_horizon != RTC_EXECUTION_HORIZON:
            raise SafetyError(
                f"continuous checkpoints require execution horizon {RTC_EXECUTION_HORIZON}"
            )
        assert self._joints is not None
        measured_anchor = self._measured_hold_action_locked()
        timeline = base_timeline
        blend_metrics = None
        if message.c2_handoff:
            blend_errors = []
            for blend_knots in (3, 2):
                try:
                    candidate = base_timeline.with_c2_handoff(
                        measured_anchor, blend_knots=blend_knots
                    )
                    self._validate_trajectory_knots_locked(candidate)
                    candidate_metrics = self._validate_rtc_blend_locked(candidate)
                except (SafetyError, ValueError) as exc:
                    blend_errors.append(f"{blend_knots}-knot: {exc}")
                    continue
                timeline = candidate
                blend_metrics = (blend_knots, *candidate_metrics)
                break
            if blend_metrics is None:
                raise SafetyError(
                    "trajectory C2 handoff is infeasible (" + "; ".join(blend_errors) + ")"
                )
        self._clear_target_locked("trajectory ownership enabled")
        self._trajectory_session_id = message.session_id
        self._trajectory_plan_id = message.plan_id
        self._timeline = timeline
        self._timeline_version += 1
        self._phase = 0.0
        self._phase_rate = 0.0
        self._handoff_anchor = None if message.c2_handoff else measured_anchor
        self._handoff_phase = None if message.c2_handoff else 0.0
        self._checkpoint_consumed = False
        self._trajectory_paused = False
        self._pause_kind = None
        self._checkpoint_stable_since = None
        self._checkpoint_emitted = False
        self._continuous_checkpoint = continuous_checkpoint
        self._heartbeat_t = now
        self._last_trajectory_tick = now
        self._raw_reference = self._handoff_anchor
        self._sent_target = self._handoff_anchor
        self._tracking_error_rad = 0.0
        self._servo_error_rad = 0.0
        self._arm_clipped = False
        self._frozen_reason = None
        self._active_request_id = None
        self._active_predicted_delay = None
        self._late_result_policy = "discard"
        self._actual_delay_steps = 0
        self._inference_invalid = False
        self._pending_rtc = None
        self._trajectory_hold_action = None
        self._trajectory_loaded_monotonic = now
        self._trajectory_deadline_monotonic = (
            None if message.chunk_timeout_s is None else now + message.chunk_timeout_s
        )
        self._governor.reset()
        self._last_command_id = message.command_id
        self._last_command_status = f"loaded trajectory {message.plan_id} version {self._timeline_version}"
        blend_duration_knots = None
        blend_max_velocity = None
        blend_max_acceleration = None
        blend_max_jerk = None
        if blend_metrics is not None:
            (
                blend_duration_knots,
                blend_max_velocity,
                blend_max_acceleration,
                blend_max_jerk,
            ) = blend_metrics
        self._emit_event_locked(
            "trajectory_loaded",
            now,
            deadline_monotonic=self._trajectory_deadline_monotonic,
            blend_duration_knots=blend_duration_knots,
            blend_max_velocity_rad_s=blend_max_velocity,
            blend_max_acceleration_rad_s2=blend_max_acceleration,
            blend_max_jerk_rad_s3=blend_max_jerk,
            detail=self._last_command_status,
        )

    def _accept_resume_locked(self, message: ResumeTrajectoryCommand, now: float) -> None:
        require_current_version(message)
        if (
            message.session_id != self._trajectory_session_id
            or message.plan_id != self._trajectory_plan_id
            or message.timeline_version != self._timeline_version
            or message.checkpoint_id != self._checkpoint_id
        ):
            raise SafetyError("resume command does not match the active checkpoint")
        settled_checkpoint = (
            not self._continuous_checkpoint
            and self._trajectory_paused
            and self._pause_kind == "checkpoint"
            and self._checkpoint_emitted
        )
        continuous_checkpoint = (
            self._continuous_checkpoint
            and not self._trajectory_paused
            and self._checkpoint_consumed
            and self._checkpoint_emitted
            and self._active_request_id is None
        )
        if not settled_checkpoint and not continuous_checkpoint:
            raise SafetyError("trajectory is not at an available checkpoint")
        if continuous_checkpoint and (self._frozen_reason is not None or self._arm_clipped):
            raise SafetyError("continuous checkpoint became unsafe before RTC inference started")
        assert self._timeline is not None
        max_delay = min(self._timeline.checkpoint_horizon, self._timeline.horizon - self._timeline.checkpoint_horizon)
        if not 1 <= message.predicted_delay_steps <= max_delay:
            raise SafetyError(f"predicted delay must be within 1..{max_delay}")
        if message.late_result_policy not in ("discard", "wait"):
            raise SafetyError("late result policy must be discard or wait")
        elapsed_steps = 0
        if continuous_checkpoint:
            assert self._phase is not None
            elapsed_steps = max(
                0,
                int(math.floor(self._phase + 1e-9)) - int(self._timeline.checkpoint_phase),
            )
            if elapsed_steps >= message.predicted_delay_steps:
                raise SafetyError(
                    "continuous checkpoint resume reached the predicted delay boundary "
                    f"({elapsed_steps} >= {message.predicted_delay_steps})"
                )
        else:
            self._trajectory_paused = False
            self._pause_kind = None
            self._checkpoint_consumed = True
        self._active_request_id = message.request_id
        self._active_predicted_delay = message.predicted_delay_steps
        self._late_result_policy = message.late_result_policy
        self._actual_delay_steps = elapsed_steps
        self._inference_invalid = False
        self._pending_rtc = None
        self._checkpoint_stable_since = None
        self._last_command_id = message.command_id
        self._last_command_status = f"resumed trajectory for RTC request {message.request_id}"
        self._emit_event_locked(
            "rtc_resumed",
            now,
            checkpoint_id=message.checkpoint_id,
            request_id=message.request_id,
            predicted_delay_steps=message.predicted_delay_steps,
            actual_delay_steps=elapsed_steps,
        )

    def _apply_rtc_replacement_locked(self, message: StageRtcChunkCommand, now: float) -> None:
        assert self._timeline is not None and self._raw_reference is not None
        if self._inference_invalid:
            raise SafetyError("RTC inference epoch was invalidated by tracking or safety")
        if self._actual_delay_steps > message.predicted_delay_steps:
            raise SafetyError(
                f"actual delay {self._actual_delay_steps} exceeds prediction {message.predicted_delay_steps}"
            )
        old_timeline = self._timeline
        old_phase = self._phase
        anchor = self._raw_reference
        if old_phase is None or abs(old_phase - round(old_phase)) > 1e-6:
            raise SafetyError("RTC replacement must occur at an integer knot boundary")
        anchored_timeline, phase = old_timeline.replacement(
            message.actions,
            actual_delay_steps=self._actual_delay_steps,
            anchor=anchor,
        )
        self._validate_trajectory_knots_locked(anchored_timeline)
        _, start_velocity, start_acceleration, _ = old_timeline.phase_kinematics(
            old_phase, side="left"
        )
        if self._trajectory_paused and self._pause_kind == "rtc_deadline":
            start_velocity = (0.0,) * 16
            start_acceleration = (0.0,) * 16

        timeline = None
        blend_metrics = None
        blend_errors = []
        for blend_knots in (3, 2):
            try:
                candidate, candidate_phase = old_timeline.replacement(
                    message.actions,
                    actual_delay_steps=self._actual_delay_steps,
                    anchor=anchor,
                    blend_knots=blend_knots,
                    start_velocity=start_velocity,
                    start_acceleration=start_acceleration,
                )
                self._validate_trajectory_knots_locked(candidate)
                candidate_metrics = self._validate_rtc_blend_locked(candidate)
            except (SafetyError, ValueError) as exc:
                blend_errors.append(f"{blend_knots}-knot: {exc}")
                continue
            timeline = candidate
            phase = candidate_phase
            blend_metrics = (blend_knots, *candidate_metrics)
            break
        if timeline is None or blend_metrics is None:
            raise SafetyError("RTC C1/C2 blend is infeasible (" + "; ".join(blend_errors) + ")")

        boundary_old_velocity: tuple[float, ...] = ()
        boundary_new_velocity: tuple[float, ...] = ()
        boundary_velocity_jump_rad = None
        boundary_acceleration_jump_rad = None
        old_index = round(old_phase)
        if old_index >= 2 and phase + 2.0 <= anchored_timeline.final_phase:
            old_anchor = action_arms(anchor)
            old_previous = action_arms(old_timeline.value(old_index - 1))
            old_before_previous = action_arms(old_timeline.value(old_index - 2))
            new_next = action_arms(anchored_timeline.value(phase + 1.0))
            new_after_next = action_arms(anchored_timeline.value(phase + 2.0))
            boundary_old_velocity = tuple(
                current - previous for current, previous in zip(old_anchor, old_previous)
            )
            boundary_new_velocity = tuple(
                current - previous for current, previous in zip(new_next, old_anchor)
            )
            old_acceleration = tuple(
                current - 2.0 * previous + before
                for current, previous, before in zip(old_anchor, old_previous, old_before_previous)
            )
            new_acceleration = tuple(
                after - 2.0 * current + previous
                for after, current, previous in zip(new_after_next, new_next, old_anchor)
            )
            boundary_velocity_jump_rad = max(
                abs(new - old) for old, new in zip(boundary_old_velocity, boundary_new_velocity)
            )
            boundary_acceleration_jump_rad = max(
                abs(new - old) for old, new in zip(old_acceleration, new_acceleration)
            )
        blend_duration_knots, blend_max_velocity, blend_max_acceleration, blend_max_jerk = blend_metrics
        old_plan_id = self._trajectory_plan_id
        self._timeline = timeline
        self._trajectory_plan_id = message.replacement_plan_id
        self._timeline_version += 1
        self._phase = phase
        self._handoff_anchor = None
        self._handoff_phase = None
        self._checkpoint_consumed = False
        self._trajectory_paused = not self._continuous_checkpoint and phase >= timeline.checkpoint_phase
        self._pause_kind = "checkpoint" if self._trajectory_paused else None
        self._checkpoint_stable_since = None
        self._checkpoint_emitted = False
        self._active_request_id = None
        self._active_predicted_delay = None
        self._late_result_policy = "discard"
        actual_delay = self._actual_delay_steps
        self._actual_delay_steps = 0
        self._pending_rtc = None
        self._governor.reset()
        self._last_command_status = f"merged RTC plan {message.replacement_plan_id} after {actual_delay} steps"
        self._emit_event_locked(
            "rtc_merged",
            now,
            checkpoint_id=message.checkpoint_id,
            request_id=message.request_id,
            predicted_delay_steps=message.predicted_delay_steps,
            actual_delay_steps=actual_delay,
            boundary_old_velocity=boundary_old_velocity,
            boundary_new_velocity=boundary_new_velocity,
            boundary_velocity_jump_rad=boundary_velocity_jump_rad,
            boundary_acceleration_jump_rad=boundary_acceleration_jump_rad,
            blend_duration_knots=blend_duration_knots,
            blend_max_velocity_rad_s=blend_max_velocity,
            blend_max_acceleration_rad_s2=blend_max_acceleration,
            blend_max_jerk_rad_s3=blend_max_jerk,
            detail=(
                f"replaced {old_plan_id} with {message.replacement_plan_id} using "
                f"{blend_duration_knots}-knot quintic C2 blend"
            ),
        )

    def _accept_stage_rtc_locked(self, message: StageRtcChunkCommand, now: float) -> None:
        require_current_version(message)
        if (
            message.session_id != self._trajectory_session_id
            or message.base_plan_id != self._trajectory_plan_id
            or message.timeline_version != self._timeline_version
            or message.checkpoint_id != self._checkpoint_id
            or message.request_id != self._active_request_id
        ):
            raise SafetyError("RTC result IDs do not match the active inference epoch")
        if message.predicted_delay_steps != self._active_predicted_delay:
            raise SafetyError("RTC result predicted delay changed in flight")
        if self._inference_invalid:
            raise SafetyError("RTC inference epoch was invalidated by tracking or safety")
        assert self._timeline is not None
        if message.execution_horizon != self._timeline.checkpoint_horizon:
            raise SafetyError("RTC execution horizon does not match the timeline checkpoint")
        candidate = TrajectoryTimeline(message.actions, self._timeline.knot_hz, message.execution_horizon)
        self._validate_trajectory_knots_locked(candidate)
        self._last_command_id = message.command_id
        if self._trajectory_paused and self._pause_kind == "rtc_deadline":
            try:
                self._apply_rtc_replacement_locked(message, now)
            except (SafetyError, ValueError) as exc:
                self._invalidate_active_rtc_locked(now, str(exc))
                raise
            return
        if self._pending_rtc is not None:
            raise SafetyError("an RTC replacement is already staged")
        self._pending_rtc = message
        self._last_command_status = f"staged RTC result {message.request_id}"

    def _accept_hold_locked(self, message: HoldPositionCommand, now: float) -> None:
        require_current_version(message)
        if not message.execute:
            raise SafetyError("execute flag is false")
        if message.session_id != self._trajectory_session_id:
            raise SafetyError("hold command does not match trajectory session")
        if message.expected_timeline_version != self._timeline_version:
            raise SafetyError("hold command timeline version mismatch")
        assert self._joints is not None
        target = validate_action(
            message.action,
            self._joints,
            max_joint_step_rad=self.max_joint_step_rad,
            joint_limit_margin_rad=self.joint_limit_margin_rad,
        )
        self._enter_hold_locked(
            target,
            now,
            command_id=message.command_id,
            event_type="holding",
            reason=message.reason,
            reason_code="commanded_hold",
        )

    def _accept_latch_measured_hold_locked(
        self, message: LatchMeasuredHoldCommand, now: float
    ) -> None:
        require_current_version(message)
        if not message.execute:
            raise SafetyError("execute flag is false")
        if message.session_id != self._trajectory_session_id:
            raise SafetyError("measured hold command does not match trajectory session")
        if message.expected_timeline_version != self._timeline_version:
            raise SafetyError("measured hold command timeline version mismatch")
        ready, reason = self._readiness_gate_locked(now)
        if not ready:
            raise SafetyError(reason)
        old_reference = self._raw_reference
        target = self._measured_hold_action_locked()
        old_delta = 0.0 if old_reference is None else self._arm_error(old_reference, self._joints)
        self._enter_hold_locked(
            target,
            now,
            command_id=message.command_id,
            event_type="measured_holding",
            reason=f"{message.reason}; old_reference_delta={old_delta:.6f}rad",
            reason_code=message.reason_code,
            final_error_rad=old_delta,
        )

    def accept_command(self, message) -> None:
        now = _now()
        with self._lock:
            if isinstance(message, StopCommand):
                try:
                    require_current_version(message)
                except ProtocolError as exc:
                    self._clear_target_locked(f"rejected stop: {exc}")
                    return
                if self._trajectory_session_id is not None:
                    self._clear_trajectory_locked(f"stopped: {message.reason}", now)
                else:
                    self._clear_target_locked(f"stopped: {message.reason}")
                return

            if isinstance(message, TrajectoryHeartbeat):
                try:
                    require_current_version(message)
                    if message.session_id != self._trajectory_session_id:
                        raise SafetyError("heartbeat session mismatch")
                    if message.timeline_version != self._timeline_version:
                        raise SafetyError("heartbeat timeline version mismatch")
                    self._heartbeat_t = now
                except (ProtocolError, SafetyError) as exc:
                    self._last_command_status = f"rejected trajectory heartbeat: {exc}"
                return

            try:
                if isinstance(message, LoadTrajectoryCommand):
                    self._accept_load_trajectory_locked(message, now)
                elif isinstance(message, ResumeTrajectoryCommand):
                    self._accept_resume_locked(message, now)
                elif isinstance(message, StageRtcChunkCommand):
                    self._accept_stage_rtc_locked(message, now)
                elif isinstance(message, HoldPositionCommand):
                    self._accept_hold_locked(message, now)
                elif isinstance(message, LatchMeasuredHoldCommand):
                    self._accept_latch_measured_hold_locked(message, now)
                elif isinstance(message, ActionCommand):
                    if self._trajectory_session_id is not None:
                        raise SafetyError("legacy action rejected while trajectory session owns the bridge")
                    self._accept_legacy_action_locked(message, now)
                else:
                    raise ProtocolError(f"unexpected message {type(message).__name__}")
            except (ProtocolError, SafetyError, TypeError, ValueError) as exc:
                command_id = getattr(message, "command_id", "unknown")
                self._last_command_status = f"rejected command {command_id}: {exc}"
                if isinstance(message, ActionCommand) and self._trajectory_session_id is None:
                    self._clear_target_locked(self._last_command_status)
                rtc_failure_already_emitted = (
                    isinstance(message, StageRtcChunkCommand)
                    and self._inference_invalid
                    and bool(self._events)
                    and self._events[-1].event_type == "rtc_invalid"
                    and self._events[-1].request_id == message.request_id
                )
                if not rtc_failure_already_emitted and isinstance(
                    message,
                    (
                        LoadTrajectoryCommand,
                        ResumeTrajectoryCommand,
                        StageRtcChunkCommand,
                        HoldPositionCommand,
                        LatchMeasuredHoldCommand,
                    ),
                ):
                    rejected_plan_id = getattr(
                        message,
                        "plan_id",
                        getattr(message, "base_plan_id", self._trajectory_plan_id),
                    )
                    self._emit_event_locked(
                        "trajectory_command_rejected",
                        now,
                        session_id=getattr(message, "session_id", self._trajectory_session_id),
                        plan_id=rejected_plan_id,
                        timeline_version=getattr(
                            message,
                            "timeline_version",
                            getattr(message, "expected_timeline_version", self._timeline_version),
                        ),
                        checkpoint_id=getattr(message, "checkpoint_id", None),
                        request_id=getattr(message, "request_id", None),
                        reason_code="command_rejected",
                        detail=str(exc),
                    )
            self._refresh_state_locked(now)

    def _accept_legacy_action_locked(self, message: ActionCommand, now: float) -> None:
        self._last_command_id = message.command_id
        require_current_version(message)
        if not message.execute:
            raise SafetyError("execute flag is false")
        ready, reason = self._readiness_gate_locked(now)
        if not ready:
            raise SafetyError(reason)
        self._check_observation_lag_locked(message.observation_seq)
        assert self._joints is not None
        target = validate_action(
            message.action,
            self._joints,
            max_joint_step_rad=self.max_joint_step_rad,
            joint_limit_margin_rad=self.joint_limit_margin_rad,
        )
        self._target = target
        self._target_t = now
        self._target_command_id = message.command_id
        self._last_command_status = f"accepted command {message.command_id}"

    @staticmethod
    def _arm_error(action: tuple[float, ...], joints: tuple[float, ...]) -> float:
        return max(abs(target - measured) for target, measured in zip(action_arms(action), joints))

    def _trajectory_reference_locked(self) -> tuple[float, ...] | None:
        if self._trajectory_hold_action is not None:
            return self._trajectory_hold_action
        if self._timeline is None or self._phase is None:
            return None
        if self._handoff_phase is not None and self._handoff_anchor is not None:
            destination = self._timeline.knots[0]
            blend = max(0.0, min(1.0, self._handoff_phase))
            return tuple(a + (b - a) * blend for a, b in zip(self._handoff_anchor, destination))
        return self._timeline.value(self._phase)

    def _invalidate_active_rtc_locked(self, now: float, reason: str) -> None:
        if self._active_request_id is None or self._inference_invalid:
            return
        self._inference_invalid = True
        self._trajectory_paused = True
        self._pause_kind = "rtc_invalid"
        self._pending_rtc = None
        self._checkpoint_stable_since = None
        self._checkpoint_emitted = False
        self._emit_event_locked(
            "rtc_invalid",
            now,
            checkpoint_id=self._checkpoint_id,
            request_id=self._active_request_id,
            predicted_delay_steps=self._active_predicted_delay,
            actual_delay_steps=self._actual_delay_steps,
            reason_code=self._rtc_failure_reason_code(reason),
            detail=reason,
        )

    @staticmethod
    def _rtc_failure_reason_code(reason: str) -> str:
        detail = reason.lower()
        if "missed predicted delay" in detail or "late" in detail:
            return "rtc_late"
        if "blend" in detail and ("infeasible" in detail or "exceeds" in detail):
            return "c2_blend_infeasible"
        if "clipp" in detail:
            return "arm_clipping"
        if "stale" in detail:
            return "stale_feedback"
        if "tracking governor" in detail or "froze" in detail or "freeze" in detail:
            return "tracking_hard_freeze"
        return "rtc_invalid"

    def _update_pause_settle_locked(
        self,
        now: float,
        *,
        state_stale: bool,
        timer_overrun: bool,
    ) -> None:
        if (
            not self._trajectory_paused
            or self._raw_reference is None
            or self._joints is None
            or self._joints_t is None
            or state_stale
            or timer_overrun
        ):
            self._checkpoint_stable_since = None
            return
        error = self._arm_error(self._raw_reference, self._joints)
        if error > self.tracking_tolerance_rad:
            self._checkpoint_stable_since = None
            return
        if self._checkpoint_stable_since is None:
            self._checkpoint_stable_since = self._joints_t
            return
        if (
            self._joints_t < self._checkpoint_stable_since
            or self._joints_t - self._checkpoint_stable_since < self.tracking_settle_seconds
        ):
            if self._joints_t < self._checkpoint_stable_since:
                self._checkpoint_stable_since = None
            return
        if self._checkpoint_emitted:
            return
        self._checkpoint_emitted = True
        stable_monotonic = self._joints_t
        if self._pause_kind == "checkpoint":
            assert self._timeline is not None
            deadline = self._trajectory_deadline_monotonic
            elapsed = (
                None
                if self._trajectory_loaded_monotonic is None
                else max(0.0, now - self._trajectory_loaded_monotonic)
            )
            final_error, worst_joint = self._tracking_diag_locked()
            self._emit_event_locked(
                "checkpoint_ready",
                now,
                checkpoint_id=self._checkpoint_id,
                stable_monotonic=stable_monotonic,
                observation_seq_at_stable=self._seq,
                old_remaining_actions_absolute=self._timeline.remaining_after_checkpoint(),
                reason_code="chunk_clean",
                deadline_monotonic=deadline,
                elapsed_s=elapsed,
                worst_joint=worst_joint,
                final_error_rad=final_error,
            )
            self._trajectory_deadline_monotonic = None
        elif self._pause_kind == "rtc_invalid":
            self._emit_event_locked(
                "fallback_ready",
                now,
                checkpoint_id=self._checkpoint_id,
                stable_monotonic=stable_monotonic,
                observation_seq_at_stable=self._seq,
                request_id=self._active_request_id,
                predicted_delay_steps=self._active_predicted_delay,
                actual_delay_steps=self._actual_delay_steps,
                detail="RTC inference invalid; synchronized fallback may observe",
            )

    def _advance_trajectory_locked(self, now: float, dt: float, phase_rate: float) -> None:
        if self._timeline is None or self._phase is None or self._trajectory_paused:
            return
        if self._handoff_phase is not None:
            self._handoff_phase = min(1.0, self._handoff_phase + self._timeline.knot_hz * phase_rate * dt)
            if self._handoff_phase >= 1.0:
                self._handoff_phase = None
                self._handoff_anchor = None
                self._phase = 0.0
            return

        if (
            self._continuous_checkpoint
            and not self._checkpoint_consumed
            and self._phase >= self._timeline.checkpoint_phase
        ):
            self._checkpoint_id += 1
            self._checkpoint_consumed = True
            self._checkpoint_emitted = True
            self._checkpoint_stable_since = None
            self._emit_event_locked(
                "checkpoint_ready",
                now,
                checkpoint_id=self._checkpoint_id,
                stable_monotonic=self._joints_t,
                observation_seq_at_stable=self._seq,
                old_remaining_actions_absolute=self._timeline.remaining_after_checkpoint(),
                reason_code="rtc_checkpoint",
                detail="continuous RTC checkpoint; trajectory remained active",
            )

        old_phase = self._phase
        phase_limit = (
            self._timeline.final_phase
            if self._checkpoint_consumed or self._continuous_checkpoint
            else self._timeline.checkpoint_phase
        )
        new_phase = min(phase_limit, old_phase + self._timeline.knot_hz * phase_rate * dt)
        old_index = int(math.floor(old_phase + 1e-9))
        new_index = int(math.floor(new_phase + 1e-9))
        crossed = max(0, new_index - old_index)
        if crossed and self._active_request_id is not None:
            reaches_deadline = (
                self._active_predicted_delay is not None
                and self._actual_delay_steps + crossed >= self._active_predicted_delay
            )
            if self._pending_rtc is not None or reaches_deadline:
                new_phase = float(old_index + 1)
                crossed = 1
        self._phase = new_phase

        if crossed and self._active_request_id is not None:
            self._actual_delay_steps += crossed
            self._raw_reference = self._timeline.value(new_phase)
            if self._pending_rtc is not None:
                pending = self._pending_rtc
                try:
                    self._apply_rtc_replacement_locked(pending, now)
                except (SafetyError, ValueError) as exc:
                    self._invalidate_active_rtc_locked(now, str(exc))
                return
            if (
                self._active_predicted_delay is not None
                and self._actual_delay_steps >= self._active_predicted_delay
            ):
                self._trajectory_paused = True
                self._pause_kind = "rtc_deadline"
                self._phase_rate = 0.0
                if self._late_result_policy == "wait":
                    self._frozen_reason = "waiting for RTC result at predicted delay boundary"
                    self._emit_event_locked(
                        "rtc_waiting_at_deadline",
                        now,
                        checkpoint_id=self._checkpoint_id,
                        request_id=self._active_request_id,
                        predicted_delay_steps=self._active_predicted_delay,
                        actual_delay_steps=self._actual_delay_steps,
                    )
                else:
                    self._frozen_reason = "RTC result missed predicted delay boundary"
                    self._invalidate_active_rtc_locked(now, self._frozen_reason)
                return

        if not self._checkpoint_consumed and new_phase >= self._timeline.checkpoint_phase:
            self._phase = self._timeline.checkpoint_phase
            self._checkpoint_stable_since = None
            self._checkpoint_emitted = False
            if not self._continuous_checkpoint:
                self._trajectory_paused = True
                self._pause_kind = "checkpoint"
                self._phase_rate = 0.0
                self._checkpoint_id += 1

    def _trajectory_target_locked(self, now: float) -> tuple[tuple[float, ...], int | None] | None:
        if self._heartbeat_t is None or now - self._heartbeat_t > self.trajectory_heartbeat_timeout_s:
            self._clear_trajectory_locked("trajectory heartbeat timed out; publication stopped", now)
            return None
        assert self._joints is not None
        if (
            self._trajectory_deadline_monotonic is not None
            and now >= self._trajectory_deadline_monotonic
            and self._timeline is not None
            and not self._checkpoint_emitted
        ):
            if self._arm_clipped or self._frozen_reason not in (None, "checkpoint"):
                deadline = self._trajectory_deadline_monotonic
                loaded = self._trajectory_loaded_monotonic
                final_error, worst_joint = self._tracking_diag_locked()
                measured = self._measured_hold_action_locked()
                reason_code = "arm_clipping" if self._arm_clipped else "tracking_hard_freeze"
                self._enter_hold_locked(
                    measured,
                    now,
                    command_id=self._last_command_id or -1,
                    event_type="fatal_holding",
                    reason=(
                        "timed chunk reached its deadline after a fatal safety fault: "
                        f"arm_clipped={self._arm_clipped} frozen={self._frozen_reason!r}"
                    ),
                    reason_code=reason_code,
                    deadline_monotonic=deadline,
                    elapsed_s=None if loaded is None else max(0.0, now - loaded),
                    worst_joint=worst_joint,
                    final_error_rad=final_error,
                )
                return measured, self._last_command_id
            deadline = self._trajectory_deadline_monotonic
            loaded = self._trajectory_loaded_monotonic
            final_error, worst_joint = self._tracking_diag_locked()
            measured = self._measured_hold_action_locked()
            elapsed = None if loaded is None else max(0.0, now - loaded)
            self._enter_hold_locked(
                measured,
                now,
                command_id=self._last_command_id or -1,
                event_type="chunk_timed_out",
                reason=(
                    "synchronized chunk missed its controller deadline; "
                    f"phase={self._phase} final_error={final_error} worst_joint={worst_joint}"
                ),
                reason_code="tracking_timeout",
                deadline_monotonic=deadline,
                elapsed_s=elapsed,
                worst_joint=worst_joint,
                final_error_rad=final_error,
            )
            return measured, self._last_command_id
        raw = self._trajectory_reference_locked()
        if raw is None:
            return None
        state_age = _age(now, self._joints_t)
        dt = 0.0 if self._last_trajectory_tick is None else now - self._last_trajectory_tick
        self._last_trajectory_tick = now
        state_stale = state_age is None or state_age > self.trajectory_state_timeout_s
        timer_overrun = dt > self.trajectory_timer_timeout_s
        try:
            filtered = filter_action(
                raw,
                self._joints,
                max_joint_step_rad=TRAJECTORY_CLIENT_ENVELOPE_RAD,
                joint_limit_margin_rad=self.joint_limit_margin_rad,
            )
            arm_clipped = any(index not in (7, 15) for index in filtered.clipped_indices)
            target = validate_action(
                filtered.action,
                self._joints,
                max_joint_step_rad=self.max_joint_step_rad,
                joint_limit_margin_rad=self.joint_limit_margin_rad,
            )
        except SafetyError as exc:
            self._clear_trajectory_locked(f"trajectory safety check failed: {exc}", now)
            return None

        tracking_error = self._arm_error(raw, self._joints)
        servo_error = self._arm_error(target, self._joints)
        decision = self._governor.update(
            tracking_error,
            state_stale=state_stale,
            timer_overrun=timer_overrun,
            arm_clipped=arm_clipped,
        )
        self._raw_reference = raw
        self._sent_target = target
        self._tracking_error_rad = tracking_error
        self._servo_error_rad = servo_error
        self._arm_clipped = arm_clipped
        self._phase_rate = 0.0 if self._trajectory_paused else decision.phase_rate
        self._frozen_reason = self._pause_kind if self._trajectory_paused else decision.frozen_reason
        if decision.hard_frozen:
            self._invalidate_active_rtc_locked(now, decision.frozen_reason or "tracking governor froze")
            if self._trajectory_deadline_monotonic is not None:
                deadline = self._trajectory_deadline_monotonic
                loaded = self._trajectory_loaded_monotonic
                final_error, worst_joint = self._tracking_diag_locked(raw)
                measured = self._measured_hold_action_locked()
                if arm_clipped:
                    reason_code = "arm_clipping"
                elif state_stale:
                    reason_code = "stale_feedback"
                elif timer_overrun:
                    reason_code = "timer_overrun"
                else:
                    reason_code = "tracking_hard_freeze"
                self._enter_hold_locked(
                    measured,
                    now,
                    command_id=self._last_command_id or -1,
                    event_type="fatal_holding",
                    reason=decision.frozen_reason or "tracking governor froze",
                    reason_code=reason_code,
                    deadline_monotonic=deadline,
                    elapsed_s=None if loaded is None else max(0.0, now - loaded),
                    worst_joint=worst_joint,
                    final_error_rad=final_error,
                )
                return measured, self._last_command_id
        self._advance_trajectory_locked(now, dt, self._phase_rate)
        self._update_pause_settle_locked(
            now,
            state_stale=state_stale,
            timer_overrun=timer_overrun,
        )
        return target, self._last_command_id

    def _publish_target(self) -> None:
        now = _now()
        with self._lock:
            if self._publish_last_tick_s is not None:
                gap_s = now - self._publish_last_tick_s
                self._publish_max_gap_s = max(self._publish_max_gap_s, gap_s)
                self._publish_late_tick_count += gap_s > 1.5 / self.publish_hz
            else:
                self._publish_first_tick_s = now
            self._publish_last_tick_s = now
            self._publish_tick_count += 1
            ready, reason = self._readiness_gate_locked(now)
            if not ready:
                if self._trajectory_session_id is not None:
                    self._clear_trajectory_locked(f"publication blocked: {reason}", now)
                elif self._target is not None:
                    self._clear_target_locked(f"publication blocked: {reason}")
                self._refresh_state_locked(now)
                return
            if self._trajectory_session_id is not None:
                trajectory_target = self._trajectory_target_locked(now)
                self._refresh_state_locked(now)
                if trajectory_target is None:
                    return
                target, command_id = trajectory_target
            else:
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
            self._gripper_command_l = float(target[7])
            self._gripper_command_r = float(target[15])
            self._last_command_status = f"published command {command_id}"

    def publish_timing_summary(self) -> None:
        with self._lock:
            elapsed_s = (
                0.0
                if self._publish_first_tick_s is None or self._publish_last_tick_s is None
                else self._publish_last_tick_s - self._publish_first_tick_s
            )
            average_hz = (
                0.0
                if elapsed_s <= 0.0 or self._publish_tick_count < 2
                else (self._publish_tick_count - 1) / elapsed_s
            )
            self.get_logger().info(
                f"publish_timing ticks={self._publish_tick_count} average_hz={average_hz:.2f} "
                f"late_ticks={self._publish_late_tick_count} "
                f"max_gap_ms={self._publish_max_gap_s * 1000.0:.3f}"
            )

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
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
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
        gripper_feedback = topic in (TOPIC_GRIPPER_FEEDBACK_L, TOPIC_GRIPPER_FEEDBACK_R)
        label = " ([q, dq, tau, T_mos, T_motor])" if gripper_feedback else ""
        print(f"  {topic}{label}: {count} msgs, {count / elapsed:.1f}Hz, latest={value}")
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
    rclpy.try_shutdown()
    return 1 if failed else 0


def _serve_client(conn: socket.socket, address, node: MarvinBridgeNode) -> None:
    stop = threading.Event()
    node.client_connected()
    # Skip the cached frame captured before client_connected() opened that part
    # of the readiness gate. The first frame sent to a client reflects the
    # current connection state.
    last_observation_seq = node.latest_observation_seq()
    last_state_seq = 0
    prefer_observation = False
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
            message = node.next_outbound(
                last_state_seq=last_state_seq,
                last_observation_seq=last_observation_seq,
                prefer_observation=prefer_observation,
                stop=stop,
            )
            if message is None:
                continue
            send_message(conn, message)
            if isinstance(message, RobotStateUpdate):
                last_state_seq = message.state_seq
                prefer_observation = True
            elif isinstance(message, RobotObservation):
                last_observation_seq = message.seq
                prefer_observation = False
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


def _shutdown_ros_runtime(executor, spin_thread: threading.Thread, node: MarvinBridgeNode) -> None:
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    publish_timing_summary = getattr(node, "publish_timing_summary", None)
    if publish_timing_summary is not None:
        publish_timing_summary()
    node.destroy_node()
    rclpy.try_shutdown()


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

    # The default rclpy SIGINT handler shuts the context down underneath the
    # spin thread. Keep signal handling in this function so teardown has one
    # owner and Ctrl+C cannot race a second rclpy.shutdown().
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MarvinBridgeNode(
        allow_motion=args.allow_motion,
        publish_hz=args.publish_hz,
        command_timeout_s=args.command_timeout,
        max_joint_step_rad=args.max_joint_step_rad,
        max_state_age_s=args.max_state_age,
        max_status_age_s=args.max_status_age,
        max_observation_lag=args.max_observation_lag,
        joint_limit_margin_rad=args.joint_limit_margin_rad,
        trajectory_state_timeout_s=args.trajectory_state_timeout,
        trajectory_timer_timeout_s=args.trajectory_timer_timeout,
        trajectory_heartbeat_timeout_s=args.trajectory_heartbeat_timeout,
        tracking_run_error_rad=args.tracking_run_error_rad,
        tracking_resume_error_rad=args.tracking_resume_error_rad,
        tracking_stop_error_rad=args.tracking_stop_error_rad,
        tracking_tolerance_rad=args.tracking_tolerance_rad,
        tracking_settle_seconds=args.tracking_settle_seconds,
        rtc_blend_max_velocity_rad_s=args.rtc_blend_max_velocity_rad_s,
        rtc_blend_max_acceleration_rad_s2=args.rtc_blend_max_acceleration_rad_s2,
        rtc_blend_max_jerk_rad_s3=args.rtc_blend_max_jerk_rad_s3,
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
        _shutdown_ros_runtime(executor, spin_thread, node)
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
    parser.add_argument("--max-joint-step-rad", type=float, default=0.16)
    parser.add_argument("--joint-limit-margin-rad", type=float, default=0.02)
    parser.add_argument("--max-state-age", type=float, default=0.20)
    parser.add_argument("--max-status-age", type=float, default=0.50)
    parser.add_argument("--max-observation-lag", type=int, default=8)
    parser.add_argument("--trajectory-state-timeout", type=float, default=0.05)
    parser.add_argument("--trajectory-timer-timeout", type=float, default=0.05)
    parser.add_argument("--trajectory-heartbeat-timeout", type=float, default=0.25)
    parser.add_argument("--tracking-run-error-rad", type=float, default=0.02)
    parser.add_argument("--tracking-resume-error-rad", type=float, default=0.12)
    parser.add_argument("--tracking-stop-error-rad", type=float, default=0.16)
    parser.add_argument("--tracking-tolerance-rad", type=float, default=0.01)
    parser.add_argument("--tracking-settle-seconds", type=float, default=0.20)
    parser.add_argument("--rtc-blend-max-velocity-rad-s", type=float, default=0.45)
    parser.add_argument("--rtc-blend-max-acceleration-rad-s2", type=float, default=2.0)
    parser.add_argument("--rtc-blend-max-jerk-rad-s3", type=float, default=20.0)
    args = parser.parse_args(argv)
    if args.publish_hz <= 0 or args.command_timeout <= 0 or args.duration <= 0:
        parser.error("rates, timeouts, and duration must be positive")
    if not (
        0 < args.tracking_run_error_rad
        < args.tracking_resume_error_rad
        < args.tracking_stop_error_rad
        <= TRAJECTORY_CLIENT_ENVELOPE_RAD
    ):
        parser.error(
            "tracking thresholds must satisfy 0 < run < resume < stop "
            f"<= {TRAJECTORY_CLIENT_ENVELOPE_RAD:.2f}"
        )
    if args.max_joint_step_rad < TRAJECTORY_CLIENT_ENVELOPE_RAD:
        parser.error(
            "--max-joint-step-rad must be at least the trajectory clipping envelope "
            f"({TRAJECTORY_CLIENT_ENVELOPE_RAD:.2f} rad)"
        )
    if min(
        args.trajectory_state_timeout,
        args.trajectory_timer_timeout,
        args.trajectory_heartbeat_timeout,
        args.tracking_tolerance_rad,
        args.tracking_settle_seconds,
        args.rtc_blend_max_velocity_rad_s,
        args.rtc_blend_max_acceleration_rad_s2,
        args.rtc_blend_max_jerk_rad_s3,
    ) <= 0:
        parser.error("trajectory timeouts and tracking parameters must be positive")
    if args.rtc_blend_max_velocity_rad_s > min(JOINT_VELOCITY):
        parser.error(
            "--rtc-blend-max-velocity-rad-s cannot exceed the active URDF joint velocity limit "
            f"({min(JOINT_VELOCITY):.4f} rad/s)"
        )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.doctor:
        return doctor(args.duration)
    return serve(args)


if __name__ == "__main__":
    raise SystemExit(main())
