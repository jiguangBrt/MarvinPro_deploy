from collections import deque
from contextlib import redirect_stderr
from dataclasses import replace
import importlib
import io
import sys
import threading
import types


class _Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _Node:
    def __init__(self, *args, **kwargs):
        del args, kwargs

    def create_subscription(self, *args, **kwargs):
        del args, kwargs

    def create_publisher(self, *args, **kwargs):
        del args, kwargs
        return types.SimpleNamespace(publish=lambda message: None)

    def create_timer(self, *args, **kwargs):
        del args, kwargs

    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda message: None,
            error=lambda message: None,
        )


class _QoSProfile:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _Notify:
    def __init__(self):
        self.calls = 0

    def notify_all(self):
        self.calls += 1


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = object
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Node
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=0, TRANSIENT_LOCAL=1)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=0)
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1)
    rclpy_signals = types.ModuleType("rclpy.signals")
    rclpy_signals.SignalHandlerOptions = types.SimpleNamespace(NO=0)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CompressedImage = _Message
    sensor_msgs_msg.JointState = _Message
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    for name in ("Float32", "Float32MultiArray", "Int16MultiArray", "Int32"):
        setattr(std_msgs_msg, name, _Message)
    marvin_msgs = types.ModuleType("marvin_msgs")
    marvin_msgs_msg = types.ModuleType("marvin_msgs.msg")
    marvin_msgs_msg.JointcmdArm = _Message

    modules = {
        "rclpy": rclpy,
        "rclpy.executors": rclpy_executors,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "rclpy.signals": rclpy_signals,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "marvin_msgs": marvin_msgs,
        "marvin_msgs.msg": marvin_msgs_msg,
    }
    sys.modules.update(modules)


_install_ros_stubs()
robot_bridge = importlib.import_module("marvinpro_deploy.robot_bridge")


def test_bridge_cli_uses_current_governor_and_safety_envelope_defaults():
    args = robot_bridge.parse_args([])

    assert args.max_joint_step_rad == 0.16
    assert args.tracking_run_error_rad == 0.02
    assert args.tracking_resume_error_rad == 0.12
    assert args.tracking_stop_error_rad == 0.16
    assert args.rtc_blend_max_velocity_rad_s == 0.45
    assert args.rtc_blend_max_acceleration_rad_s2 == 2.0
    assert args.rtc_blend_max_jerk_rad_s3 == 40.0


def test_bridge_cli_rejects_validation_envelope_below_trajectory_clipping_envelope():
    with redirect_stderr(io.StringIO()):
        try:
            robot_bridge.parse_args(["--max-joint-step-rad", "0.159"])
        except SystemExit as exc:
            assert exc.code == 2
        else:
            raise AssertionError("bridge accepted a validation envelope below the clipping envelope")


def test_gripper_feedback_parser_preserves_q_velocity_torque_and_temperatures():
    values = robot_bridge.MarvinBridgeNode._gripper_feedback_values(
        _Message(data=[0.4, -0.02, 0.31, 32.0, 30.0, 999.0])
    )

    assert values == (0.4, -0.02, 0.31, 32.0, 30.0)
    assert robot_bridge.MarvinBridgeNode._gripper_feedback_values(
        _Message(data=[0.4, -0.02])
    ) is None
    assert robot_bridge.MarvinBridgeNode._gripper_feedback_values(
        _Message(data=[0.4, "invalid", 0.31])
    ) is None


def test_gripper_feedback_updates_normalized_measured_state(monkeypatch):
    node = object.__new__(robot_bridge.MarvinBridgeNode)
    node._lock = threading.RLock()
    node._gripper_r = 0.75
    node._gripper_feedback_r_position = None
    node._gripper_r_velocity = None
    node._gripper_r_torque = None
    node._gripper_r_mos_temperature = None
    node._gripper_r_motor_temperature = None
    node._gripper_r_t = None
    monkeypatch.setattr(robot_bridge, "_now", lambda: 12.0)

    node._on_gripper_r(_Message(data=[0.625, -0.007, 0.1, 31.0, 30.0]))

    assert node._gripper_r == 0.5
    assert node._gripper_feedback_r_position == 0.625
    assert node._gripper_r_velocity == -0.007
    assert node._gripper_r_torque == 0.1
    assert node._gripper_r_mos_temperature == 31.0
    assert node._gripper_r_motor_temperature == 30.0
    assert node._gripper_r_t == 12.0
    assert robot_bridge.MarvinBridgeNode._gripper_feedback_values(
        _Message(data=[0.4, float("nan"), 0.31])
    ) is None


def _bare_node():
    node = object.__new__(robot_bridge.MarvinBridgeNode)
    node._outbound_ready = _Notify()
    node._event_seq = 0
    node._events = deque()
    node._trajectory_session_id = None
    node._trajectory_plan_id = None
    node._trajectory_hold_action = None
    node._trajectory_loaded_monotonic = None
    node._trajectory_deadline_monotonic = None
    node._timeline_version = 7
    node._phase = None
    node._joints_t = None
    node._raw_reference = None
    node._sent_target = None
    node._tracking_error_rad = None
    node._servo_error_rad = None
    node._arm_clipped = False
    node._frozen_reason = None
    node._late_result_policy = "discard"
    node.publish_hz = 100.0
    node.max_joint_step_rad = 0.16
    node.joint_limit_margin_rad = 0.02
    node.rtc_blend_max_velocity_rad_s = 0.45
    node.rtc_blend_max_acceleration_rad_s2 = 2.0
    node.rtc_blend_max_jerk_rad_s3 = 20.0
    node._gripper_l_velocity = None
    node._gripper_r_velocity = None
    node._gripper_feedback_l_position = None
    node._gripper_feedback_r_position = None
    node._gripper_l_torque = None
    node._gripper_r_torque = None
    node._gripper_l_mos_temperature = None
    node._gripper_r_mos_temperature = None
    node._gripper_l_motor_temperature = None
    node._gripper_r_motor_temperature = None
    return node


def _ready_trajectory_node(clock):
    node = robot_bridge.MarvinBridgeNode(
        allow_motion=True,
        publish_hz=100.0,
        command_timeout_s=0.25,
        max_joint_step_rad=0.16,
        max_state_age_s=0.20,
        max_status_age_s=0.50,
        max_observation_lag=8,
        joint_limit_margin_rad=0.02,
        trajectory_state_timeout_s=0.05,
        trajectory_timer_timeout_s=0.05,
        trajectory_heartbeat_timeout_s=0.25,
        tracking_run_error_rad=0.02,
        tracking_resume_error_rad=0.12,
        tracking_stop_error_rad=0.16,
        tracking_tolerance_rad=0.01,
        tracking_settle_seconds=0.20,
        rtc_blend_max_velocity_rad_s=0.45,
        rtc_blend_max_acceleration_rad_s2=2.0,
        rtc_blend_max_jerk_rad_s3=20.0,
    )
    node._client_connected = True
    node._joints = (0.0,) * 14
    node._joints_t = clock[0]
    node._gripper_l = node._gripper_r = 0.0
    node._gripper_l_t = node._gripper_r_t = clock[0]
    node._input_mode = 3
    node._input_mode_t = clock[0]
    node._robot_state = node._arm_state = (3, 3)
    node._robot_state_t = node._arm_state_t = clock[0]
    node._latest_observation = robot_bridge.RobotObservation(
        seq=1,
        captured_monotonic=clock[0],
        image=b"jpeg",
        image_format="jpeg",
        joints=node._joints,
        gripper_raw_left=0.0,
        gripper_raw_right=0.0,
        input_mode=3,
        robot_state=(3, 3),
        arm_state=(3, 3),
        age_state_s=0.0,
        age_gripper_left_s=0.0,
        age_gripper_right_s=0.0,
        age_input_mode_s=0.0,
        age_robot_state_s=0.0,
        age_arm_state_s=0.0,
        motion_gate_open=True,
        gate_reason="ready",
    )
    return node


def _drive_fake_bridge_until(node, clock, event_type, *, newer_than=0, max_ticks=700):
    for _ in range(max_ticks):
        clock[0] += 0.01
        raw = node._trajectory_reference_locked()
        if raw is not None:
            node._joints = robot_bridge.action_arms(raw)
        node._joints_t = clock[0]
        node._gripper_l_t = node._gripper_r_t = clock[0]
        node._robot_state_t = node._arm_state_t = clock[0]
        node._heartbeat_t = clock[0]
        with node._lock:
            node._trajectory_target_locked(clock[0])
        matches = [
            event
            for event in node._events
            if event.event_type == event_type and event.event_seq > newer_than
        ]
        if matches:
            return matches[-1]
    raise AssertionError(f"fake bridge did not emit {event_type} after {max_ticks} ticks")


def test_shutdown_ros_runtime_has_one_ordered_owner(monkeypatch):
    calls = []
    executor = types.SimpleNamespace(shutdown=lambda: calls.append("executor"))
    spin_thread = types.SimpleNamespace(join=lambda timeout: calls.append(("join", timeout)))
    node = types.SimpleNamespace(destroy_node=lambda: calls.append("node"))
    monkeypatch.setattr(robot_bridge.rclpy, "try_shutdown", lambda: calls.append("context"), raising=False)

    robot_bridge._shutdown_ros_runtime(executor, spin_thread, node)

    assert calls == ["executor", ("join", 2.0), "node", "context"]


def test_timer_state_update_preserves_joint_source_timestamp():
    node = _bare_node()
    node._joints = (0.0,) * 14
    node._joints_t = 12.5
    node._gripper_l = 1.0
    node._gripper_r = 2.0
    node._state_seq = 0
    node._latest_state = None
    node._readiness_gate_locked = lambda now: (True, "ready")
    node._last_command_id = None
    node._last_command_status = "idle"
    node._phase_rate = 0.0
    node._raw_reference = None
    node._sent_target = None
    node._tracking_error_rad = None
    node._servo_error_rad = None
    node._arm_clipped = False
    node._frozen_reason = None
    node._active_request_id = None

    node._refresh_state_locked(99.0)

    assert node._latest_state.sampled_monotonic == 12.5
    assert node._latest_state.state_seq == 1


def test_motion_gate_requires_fresh_gripper_feedback():
    node = _bare_node()
    node.allow_motion = True
    node._client_connected = True
    node._joints = (0.0,) * 14
    node._joints_t = 10.0
    node.max_state_age_s = 0.20
    node._gripper_l = 0.4
    node._gripper_r = 0.7
    node._gripper_l_t = node._gripper_r_t = 10.0
    node._input_mode = 3
    node._robot_state = node._arm_state = (3, 3)
    node._robot_state_t = node._arm_state_t = 10.0
    node.max_status_age_s = 0.50

    ready, reason = node._readiness_gate_locked(10.1)

    assert ready
    assert reason == "ready"

    node._gripper_r_t = 9.0
    ready, reason = node._readiness_gate_locked(10.1)
    assert not ready
    assert reason == "right gripper feedback is stale"


def test_rejection_event_can_precede_trajectory_session():
    node = _bare_node()

    node._emit_event_locked(
        "trajectory_command_rejected",
        20.0,
        session_id="requested-session",
        plan_id="requested-plan",
        timeline_version=3,
        detail="bad shape",
    )

    assert len(node._events) == 1
    event = node._events[0]
    assert event.session_id == "requested-session"
    assert event.plan_id == "requested-plan"
    assert event.timeline_version == 3
    assert event.detail == "bad shape"


def _action(value):
    return (float(value),) * 16


def _arm_action(value):
    return (float(value),) * 7 + (0.0,) + (float(value),) * 7 + (0.0,)


def _active_rtc_deadline_node(late_result_policy):
    node = _bare_node()
    node._trajectory_session_id = "session"
    node._trajectory_plan_id = "plan"
    node._timeline_version = 1
    node._timeline = robot_bridge.TrajectoryTimeline(
        tuple(_action(index * 0.01) for index in range(10)),
        5.0,
        6,
    )
    node._phase = 6.0
    node._phase_rate = 1.0
    node._handoff_phase = None
    node._handoff_anchor = None
    node._checkpoint_consumed = True
    node._trajectory_paused = False
    node._pause_kind = None
    node._checkpoint_stable_since = None
    node._checkpoint_emitted = True
    node._checkpoint_id = 1
    node._continuous_checkpoint = True
    node._raw_reference = _action(0.06)
    node._sent_target = _action(0.06)
    node._active_request_id = "request"
    node._active_predicted_delay = 1
    node._late_result_policy = late_result_policy
    node._actual_delay_steps = 0
    node._inference_invalid = False
    node._pending_rtc = None
    return node


def test_discard_policy_invalidates_rtc_at_physical_delay_boundary():
    node = _active_rtc_deadline_node("discard")

    node._advance_trajectory_locked(1.0, 0.20, 1.0)

    assert node._actual_delay_steps == 1
    assert node._trajectory_paused
    assert node._pause_kind == "rtc_invalid"
    assert node._inference_invalid
    assert node._events[-1].event_type == "rtc_invalid"
    assert "missed predicted delay" in node._events[-1].detail

    try:
        node._accept_stage_rtc_locked(
            robot_bridge.StageRtcChunkCommand(
                command_id=2,
                session_id="session",
                base_plan_id="plan",
                replacement_plan_id="replacement",
                timeline_version=1,
                checkpoint_id=1,
                request_id="request",
                predicted_delay_steps=1,
                execution_horizon=6,
                actions=tuple(_action(0.07 + index * 0.001) for index in range(10)),
            ),
            1.1,
        )
    except robot_bridge.SafetyError as exc:
        assert "invalidated" in str(exc)
    else:
        raise AssertionError("discard policy accepted a result after the physical deadline")


def test_wait_policy_keeps_deadline_epoch_mergeable_for_comparison():
    node = _active_rtc_deadline_node("wait")

    node._advance_trajectory_locked(1.0, 0.20, 1.0)

    assert node._actual_delay_steps == 1
    assert node._trajectory_paused
    assert node._pause_kind == "rtc_deadline"
    assert not node._inference_invalid
    assert node._events[-1].event_type == "rtc_waiting_at_deadline"

    node._validate_trajectory_knots_locked = lambda timeline: None
    node._governor = types.SimpleNamespace(reset=lambda: None)
    node._accept_stage_rtc_locked(
        robot_bridge.StageRtcChunkCommand(
            command_id=2,
            session_id="session",
            base_plan_id="plan",
            replacement_plan_id="replacement",
            timeline_version=1,
            checkpoint_id=1,
            request_id="request",
            predicted_delay_steps=1,
            execution_horizon=6,
            actions=tuple(_action(0.07 + index * 0.001) for index in range(10)),
        ),
        1.1,
    )

    assert node._events[-1].event_type == "rtc_merged"
    assert node._events[-1].actual_delay_steps == 1
    assert node._trajectory_plan_id == "replacement"


def test_infeasible_deadline_blend_invalidates_rtc_without_replacing_timeline():
    node = _active_rtc_deadline_node("wait")
    original_timeline = node._timeline
    node._advance_trajectory_locked(1.0, 0.20, 1.0)
    node._validate_trajectory_knots_locked = lambda timeline: None
    node._governor = types.SimpleNamespace(reset=lambda: None)

    message = robot_bridge.StageRtcChunkCommand(
        command_id=2,
        session_id="session",
        base_plan_id="plan",
        replacement_plan_id="unsafe-replacement",
        timeline_version=1,
        checkpoint_id=1,
        request_id="request",
        predicted_delay_steps=1,
        execution_horizon=6,
        actions=tuple(_action(0.5 + index * 0.01) for index in range(10)),
    )
    try:
        node._accept_stage_rtc_locked(message, 1.1)
    except robot_bridge.SafetyError as exc:
        assert "blend is infeasible" in str(exc)
    else:
        raise AssertionError("unsafe RTC blend was accepted")

    event_types = [event.event_type for event in node._events]
    assert "rtc_invalid" in event_types
    assert event_types[-1] == "rtc_invalid"
    assert "blend is infeasible" in node._events[-1].detail
    assert node._inference_invalid
    assert node._trajectory_plan_id == "plan"
    assert node._timeline is original_timeline


def test_checkpoint_requires_fresh_feedback_at_a3_for_full_settle_window():
    node = _bare_node()
    node._trajectory_session_id = "session"
    node._trajectory_plan_id = "plan"
    node._trajectory_paused = True
    node._pause_kind = "checkpoint"
    node._raw_reference = _action(0.03)
    node._joints = (0.02,) * 14
    node._joints_t = 1.0
    node._checkpoint_stable_since = None
    node._checkpoint_emitted = False
    node._checkpoint_id = 1
    node._seq = 8
    node.tracking_tolerance_rad = 0.01
    node.tracking_settle_seconds = 0.20
    node._timeline = robot_bridge.TrajectoryTimeline(
        tuple(_action(index * 0.01) for index in range(10)),
        7.5,
        4,
    )
    node._active_request_id = None
    node._active_predicted_delay = None
    node._actual_delay_steps = 0

    node._update_pause_settle_locked(1.0, state_stale=False, timer_overrun=False)
    node._update_pause_settle_locked(1.3, state_stale=True, timer_overrun=False)
    assert not node._events
    assert node._checkpoint_stable_since is None

    node._joints = (0.03,) * 14
    for source_time in (2.0, 2.1, 2.19):
        node._joints_t = source_time
        node._update_pause_settle_locked(source_time, state_stale=False, timer_overrun=False)
    assert not node._events

    node._joints_t = 2.21
    node._tracking_error_rad = 0.0
    node._servo_error_rad = 0.0
    node._sent_target = _action(0.03)
    node._update_pause_settle_locked(2.21, state_stale=False, timer_overrun=False)
    assert len(node._events) == 1
    assert node._events[0].event_type == "checkpoint_ready"
    assert node._events[0].stable_monotonic == 2.21
    assert node._events[0].joint_source_monotonic == 2.21
    assert abs(node._events[0].settle_duration_s - 0.21) < 1e-9
    assert node._events[0].tracking_error_rad == 0.0
    assert node._events[0].servo_error_rad == 0.0
    assert node._events[0].raw_reference == node._events[0].sent_target


def test_continuous_checkpoint_does_not_pause_and_accounts_for_elapsed_knots():
    node = _bare_node()
    node._trajectory_session_id = "session"
    node._trajectory_plan_id = "plan"
    node._timeline_version = 1
    node._timeline = robot_bridge.TrajectoryTimeline(
        tuple(_action(index * 0.01) for index in range(10)),
        7.5,
        6,
    )
    node._phase = 5.0
    node._phase_rate = 1.0
    node._handoff_phase = None
    node._handoff_anchor = None
    node._checkpoint_consumed = False
    node._trajectory_paused = False
    node._pause_kind = None
    node._checkpoint_stable_since = None
    node._checkpoint_emitted = False
    node._checkpoint_id = 0
    node._continuous_checkpoint = True
    node._joints = (0.05,) * 14
    node._joints_t = 5.0
    node._seq = 12
    node._raw_reference = _action(0.05)
    node._sent_target = _action(0.05)
    node._tracking_error_rad = 0.0
    node._servo_error_rad = 0.0
    node._active_request_id = None
    node._active_predicted_delay = None
    node._actual_delay_steps = 0
    node._inference_invalid = False
    node._pending_rtc = None
    node._last_command_id = 1
    node._last_command_status = "loaded"

    node._advance_trajectory_locked(5.0, 0.01, 1.0)

    checkpoint = node._events[0]
    assert checkpoint.event_type == "checkpoint_ready"
    assert checkpoint.continuous_checkpoint
    assert checkpoint.settle_duration_s is None
    assert checkpoint.stable_monotonic == 5.0
    assert checkpoint.old_remaining_actions_absolute == node._timeline.knots[6:]
    assert node._phase > 5.0
    assert node._checkpoint_consumed
    assert not node._trajectory_paused

    node._phase = 6.25
    node._accept_resume_locked(
        robot_bridge.ResumeTrajectoryCommand(
            command_id=2,
            session_id="session",
            plan_id="plan",
            timeline_version=1,
            checkpoint_id=1,
            request_id="request",
            predicted_delay_steps=3,
        ),
        5.2,
    )

    assert node._active_request_id == "request"
    assert node._actual_delay_steps == 1
    assert node._events[-1].event_type == "rtc_resumed"
    assert node._events[-1].actual_delay_steps == 1

    node._validate_trajectory_knots_locked = lambda timeline: None
    node._governor = types.SimpleNamespace(reset=lambda: None)
    rtc_knots = tuple(_action(0.066 + index * 0.004) for index in range(10))
    node._accept_stage_rtc_locked(
        robot_bridge.StageRtcChunkCommand(
            command_id=3,
            session_id="session",
            base_plan_id="plan",
            replacement_plan_id="replacement",
            timeline_version=1,
            checkpoint_id=1,
            request_id="request",
            predicted_delay_steps=3,
            execution_horizon=6,
            actions=rtc_knots,
        ),
        5.21,
    )
    node._advance_trajectory_locked(5.3, 0.10, 1.0)

    merged = node._events[-1]
    assert merged.event_type == "rtc_merged"
    assert merged.actual_delay_steps == 2
    assert node._trajectory_plan_id == "replacement"
    assert node._phase == 1.0
    assert not node._trajectory_paused


def test_outbound_events_are_reliable_and_image_gets_fair_turn_after_state():
    node = _bare_node()
    node._outbound_ready = threading.Condition()
    node._latest_state = robot_bridge.RobotStateUpdate(
        4,
        1.0,
        (0.0,) * 14,
        0.0,
        0.0,
        True,
        "ready",
    )
    node._latest_observation = robot_bridge.RobotObservation(
        3,
        1.0,
        b"jpeg",
        "jpeg",
        (0.0,) * 14,
        0.0,
        0.0,
        3,
        (3, 3),
        (3, 3),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        True,
        "ready",
    )
    event = robot_bridge.TrajectoryEvent(1, "checkpoint_ready", 1.0, "s", "p", 1, 3.0)
    node._events.append(event)
    stop = threading.Event()

    assert node.next_outbound(
        last_state_seq=0,
        last_observation_seq=0,
        prefer_observation=False,
        stop=stop,
    ) is event
    assert node.next_outbound(
        last_state_seq=0,
        last_observation_seq=0,
        prefer_observation=False,
        stop=stop,
    ) is node._latest_state
    assert node.next_outbound(
        last_state_seq=4,
        last_observation_seq=0,
        prefer_observation=True,
        stop=stop,
    ) is node._latest_observation


def test_measured_hold_atomically_latches_arms_and_preserves_last_gripper_command(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    knots = tuple(_arm_action(index * 0.001) for index in range(robot_bridge.RTC_HORIZON))
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            1, 1, "session", "plan", 0, knots, 5.0, robot_bridge.RTC_HORIZON, True
        )
    )
    node._joints = (0.03,) * 14
    node._joints_t = clock[0]
    node._sent_target = (0.1,) * 7 + (0.25,) + (0.1,) * 7 + (0.75,)
    node._raw_reference = node._sent_target
    node._pending_rtc = object()

    node.accept_command(
        robot_bridge.LatchMeasuredHoldCommand(2, "session", 1, True, "stuck", "tracking_timeout")
    )

    expected = (0.03,) * 7 + (0.25,) + (0.03,) * 7 + (0.75,)
    assert node._trajectory_hold_action == expected
    assert node._raw_reference == expected
    assert node._sent_target == expected
    assert node._timeline is None
    assert node._pending_rtc is None
    assert node._timeline_version == 2
    assert node._events[-1].event_type == "measured_holding"
    assert node._events[-1].reason_code == "tracking_timeout"
    assert abs(node._events[-1].final_error_rad - 0.07) < 1e-12


def test_timed_chunk_deadline_latches_measured_hold_with_diagnostics(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    knots = tuple(_arm_action(index * 0.001) for index in range(robot_bridge.RTC_HORIZON))
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            command_id=1,
            observation_seq=1,
            session_id="session",
            plan_id="plan",
            expected_timeline_version=0,
            knots=knots,
            knot_hz=5.0,
            checkpoint_horizon=robot_bridge.RTC_HORIZON,
            execute=True,
            chunk_timeout_s=5.0,
        )
    )
    node._phase = 19.0
    node._raw_reference = knots[-1]
    node._sent_target = knots[-1]
    node._joints = (0.005,) * 14
    clock[0] = 105.01
    node._joints_t = node._gripper_l_t = node._gripper_r_t = clock[0]
    node._robot_state_t = node._arm_state_t = node._heartbeat_t = clock[0]

    with node._lock:
        target, _ = node._trajectory_target_locked(clock[0])

    assert robot_bridge.action_arms(target) == node._joints
    assert node._trajectory_hold_action == target
    timeout = node._events[-1]
    assert timeout.event_type == "chunk_timed_out"
    assert timeout.reason_code == "tracking_timeout"
    assert timeout.deadline_monotonic == 105.0
    assert abs(timeout.elapsed_s - 5.01) < 1e-12
    assert timeout.worst_joint == robot_bridge.JOINT_NAMES[0]
    assert abs(timeout.final_error_rad - 0.014) < 1e-12


def test_timed_chunk_clean_at_4_point_9_seconds_beats_deadline(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    knots = tuple(_arm_action(0.0) for _ in range(robot_bridge.RTC_HORIZON))
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            1,
            1,
            "session",
            "plan",
            0,
            knots,
            5.0,
            robot_bridge.RTC_HORIZON,
            True,
            chunk_timeout_s=5.0,
        )
    )
    node._phase = 19.0
    node._raw_reference = knots[-1]
    node._sent_target = knots[-1]
    node._trajectory_paused = True
    node._pause_kind = "checkpoint"
    node._checkpoint_stable_since = 104.69
    clock[0] = 104.9
    node._last_trajectory_tick = 104.89
    node._joints_t = node._gripper_l_t = node._gripper_r_t = clock[0]
    node._robot_state_t = node._arm_state_t = node._heartbeat_t = clock[0]

    with node._lock:
        node._trajectory_target_locked(clock[0])

    event = node._events[-1]
    assert event.event_type == "checkpoint_ready"
    assert event.reason_code == "chunk_clean"
    assert abs(event.elapsed_s - 4.9) < 1e-12
    assert all(item.event_type != "chunk_timed_out" for item in node._events)


def test_timed_chunk_fatal_freeze_holds_measured_pose_without_replan_event(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    knots = tuple(_arm_action(index * 0.001) for index in range(robot_bridge.RTC_HORIZON))
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            1,
            1,
            "session",
            "plan",
            0,
            knots,
            5.0,
            robot_bridge.RTC_HORIZON,
            True,
            chunk_timeout_s=5.0,
        )
    )
    node._phase = 10.0
    node._raw_reference = knots[10]
    node._sent_target = knots[10]
    node._frozen_reason = "tracking governor hard freeze"
    clock[0] = 105.01
    node._joints_t = node._gripper_l_t = node._gripper_r_t = clock[0]
    node._robot_state_t = node._arm_state_t = node._heartbeat_t = clock[0]

    with node._lock:
        target, _ = node._trajectory_target_locked(clock[0])

    assert robot_bridge.action_arms(target) == node._joints
    assert node._events[-1].event_type == "fatal_holding"
    assert node._events[-1].reason_code == "tracking_hard_freeze"
    assert all(event.event_type != "chunk_timed_out" for event in node._events)


def test_c2_handoff_rejection_is_atomic_and_keeps_existing_hold(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    initial = tuple(_arm_action(0.0) for _ in range(robot_bridge.RTC_HORIZON))
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            1, 1, "session", "plan", 0, initial, 5.0, robot_bridge.RTC_HORIZON, True
        )
    )
    node.accept_command(robot_bridge.LatchMeasuredHoldCommand(2, "session", 1, True))
    held = node._trajectory_hold_action
    unsafe = tuple(_arm_action(0.2) for _ in range(robot_bridge.RTC_HORIZON))

    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            3,
            1,
            "session",
            "unsafe",
            2,
            unsafe,
            5.0,
            robot_bridge.RTC_HORIZON,
            True,
            c2_handoff=True,
        )
    )

    assert node._timeline_version == 2
    assert node._trajectory_plan_id == "plan"
    assert node._timeline is None
    assert node._trajectory_hold_action == held
    assert node._events[-1].event_type == "trajectory_command_rejected"
    assert node._events[-1].reason_code == "c2_blend_infeasible"
    assert "C2 handoff is infeasible" in node._events[-1].detail


def test_c2_handoff_falls_back_from_three_knots_to_two(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    attempts = []

    def validate_blend(timeline):
        attempts.append(timeline.blend.end_phase)
        if timeline.blend.end_phase == 3.0:
            raise robot_bridge.SafetyError("synthetic 3-knot rejection")
        return 0.2, 0.8, 4.0

    monkeypatch.setattr(node, "_validate_rtc_blend_locked", validate_blend)
    knots = tuple(_arm_action(index * 0.001) for index in range(robot_bridge.RTC_HORIZON))

    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            1,
            1,
            "session",
            "plan",
            0,
            knots,
            5.0,
            robot_bridge.RTC_HORIZON,
            True,
            c2_handoff=True,
        )
    )

    loaded = node._events[-1]
    assert attempts == [3.0, 2.0]
    assert loaded.event_type == "trajectory_loaded"
    assert loaded.blend_duration_knots == 2
    assert node._timeline.blend.end_phase == 2.0


def test_c2_handoff_accepts_zero_boundary_grippers_without_overshoot(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    knots = [list(_arm_action(0.0)) for _ in range(robot_bridge.RTC_HORIZON)]
    knots[4][7] = 0.02
    knots[4][15] = 0.01

    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            1,
            1,
            "session",
            "plan",
            0,
            tuple(tuple(knot) for knot in knots),
            5.0,
            robot_bridge.RTC_HORIZON,
            True,
            c2_handoff=True,
        )
    )

    loaded = node._events[-1]
    assert loaded.event_type == "trajectory_loaded"
    assert loaded.blend_duration_knots == 3
    for sample in range(61):
        action = node._timeline.value(3.0 * sample / 60.0)
        assert 0.0 <= action[7] <= 1.0
        assert 0.0 <= action[15] <= 1.0


def test_fake_bridge_checkpoint_resume_and_atomic_rtc_merge(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = robot_bridge.MarvinBridgeNode(
        allow_motion=True,
        publish_hz=100.0,
        command_timeout_s=0.25,
        max_joint_step_rad=0.16,
        max_state_age_s=0.20,
        max_status_age_s=0.50,
        max_observation_lag=8,
        joint_limit_margin_rad=0.02,
        trajectory_state_timeout_s=0.05,
        trajectory_timer_timeout_s=0.05,
        trajectory_heartbeat_timeout_s=0.25,
        tracking_run_error_rad=0.02,
        tracking_resume_error_rad=0.12,
        tracking_stop_error_rad=0.16,
        tracking_tolerance_rad=0.01,
        tracking_settle_seconds=0.20,
        rtc_blend_max_velocity_rad_s=0.45,
        rtc_blend_max_acceleration_rad_s2=2.0,
        rtc_blend_max_jerk_rad_s3=20.0,
    )
    node._client_connected = True
    node._joints = (0.0,) * 14
    node._joints_t = clock[0]
    node._gripper_l = node._gripper_r = 0.0
    node._gripper_l_t = node._gripper_r_t = clock[0]
    node._input_mode = 3
    node._input_mode_t = clock[0]
    node._robot_state = node._arm_state = (3, 3)
    node._robot_state_t = node._arm_state_t = clock[0]
    node._latest_observation = robot_bridge.RobotObservation(
        seq=1,
        captured_monotonic=clock[0],
        image=b"jpeg",
        image_format="jpeg",
        joints=node._joints,
        gripper_raw_left=0.0,
        gripper_raw_right=0.0,
        input_mode=3,
        robot_state=(3, 3),
        arm_state=(3, 3),
        age_state_s=0.0,
        age_gripper_left_s=0.0,
        age_gripper_right_s=0.0,
        age_input_mode_s=0.0,
        age_robot_state_s=0.0,
        age_arm_state_s=0.0,
        motion_gate_open=True,
        gate_reason="ready",
    )
    old_knots = tuple(
        _arm_action(index * 0.005) for index in range(robot_bridge.RTC_HORIZON)
    )
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            command_id=1,
            observation_seq=1,
            session_id="session",
            plan_id="old-plan",
            expected_timeline_version=0,
            knots=old_knots,
            knot_hz=5.0,
            checkpoint_horizon=robot_bridge.RTC_EXECUTION_HORIZON,
            execute=True,
        )
    )

    for _ in range(300):
        raw = node._trajectory_reference_locked()
        node._joints = robot_bridge.action_arms(raw)
        node._joints_t = clock[0]
        node._gripper_l_t = node._gripper_r_t = clock[0]
        node._robot_state_t = node._arm_state_t = clock[0]
        node._heartbeat_t = clock[0]
        with node._lock:
            node._trajectory_target_locked(clock[0])
        if any(event.event_type == "checkpoint_ready" for event in node._events):
            break
        clock[0] += 0.01

    checkpoint = next(event for event in node._events if event.event_type == "checkpoint_ready")
    assert checkpoint.phase == 9.0
    assert checkpoint.checkpoint_id == 1
    assert checkpoint.old_remaining_actions_absolute == old_knots[10:]
    assert clock[0] - 100.0 > 0.20

    node.accept_command(
        robot_bridge.ResumeTrajectoryCommand(
            command_id=2,
            session_id="session",
            plan_id="old-plan",
            timeline_version=1,
            checkpoint_id=1,
            request_id="request",
            predicted_delay_steps=2,
        )
    )
    rtc_knots = tuple(
        _arm_action(0.050 + index * 0.004) for index in range(robot_bridge.RTC_HORIZON)
    )
    node.accept_command(
        robot_bridge.StageRtcChunkCommand(
            command_id=3,
            session_id="session",
            base_plan_id="old-plan",
            replacement_plan_id="new-plan",
            timeline_version=1,
            checkpoint_id=1,
            request_id="request",
            predicted_delay_steps=2,
            execution_horizon=robot_bridge.RTC_EXECUTION_HORIZON,
            actions=rtc_knots,
        )
    )

    for _ in range(100):
        clock[0] += 0.01
        raw = node._trajectory_reference_locked()
        node._joints = robot_bridge.action_arms(raw)
        node._joints_t = clock[0]
        node._gripper_l_t = node._gripper_r_t = clock[0]
        node._robot_state_t = node._arm_state_t = clock[0]
        node._heartbeat_t = clock[0]
        with node._lock:
            node._trajectory_target_locked(clock[0])
        if any(event.event_type == "rtc_merged" for event in node._events):
            break

    merged = next(event for event in node._events if event.event_type == "rtc_merged")
    assert merged.actual_delay_steps == 1
    assert merged.timeline_version == 2
    assert node._trajectory_plan_id == "new-plan"
    assert node._timeline_version == 2
    assert node._phase == 0.0
    assert node._timeline.knots[0] == old_knots[10]
    assert all(abs(value - 0.005) < 1e-12 for value in merged.boundary_old_velocity)
    assert all(abs(value - 0.004) < 1e-12 for value in merged.boundary_new_velocity)
    assert abs(merged.boundary_velocity_jump_rad - 0.001) < 1e-12
    assert merged.boundary_acceleration_jump_rad < 1e-12
    assert merged.blend_duration_knots == 3
    assert merged.blend_max_velocity_rad_s <= 0.45
    assert merged.blend_max_acceleration_rad_s2 <= 2.0
    assert merged.blend_max_jerk_rad_s3 <= 20.0


def test_fake_bridge_c2_reject_hold_sync_bootstrap_and_merge(monkeypatch):
    clock = [200.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = _ready_trajectory_node(clock)
    old_knots = tuple(
        _arm_action(index * 0.003) for index in range(robot_bridge.RTC_HORIZON)
    )
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            1,
            1,
            "recovery-session",
            "old-plan",
            0,
            old_knots,
            5.0,
            robot_bridge.RTC_EXECUTION_HORIZON,
            True,
        )
    )
    checkpoint = _drive_fake_bridge_until(node, clock, "checkpoint_ready")
    assert checkpoint.plan_id == "old-plan"
    assert checkpoint.timeline_version == 1
    assert checkpoint.checkpoint_id == 1

    node.accept_command(
        robot_bridge.ResumeTrajectoryCommand(
            2,
            "recovery-session",
            "old-plan",
            1,
            1,
            "failed-request",
            2,
        )
    )
    unsafe = tuple(_arm_action(0.2) for _ in range(robot_bridge.RTC_HORIZON))
    node.accept_command(
        robot_bridge.StageRtcChunkCommand(
            3,
            "recovery-session",
            "old-plan",
            "unsafe-plan",
            1,
            1,
            "failed-request",
            2,
            robot_bridge.RTC_EXECUTION_HORIZON,
            unsafe,
        )
    )
    _drive_fake_bridge_until(node, clock, "rtc_invalid")
    invalid = [event for event in node._events if event.event_type == "rtc_invalid"][-1]
    assert invalid.request_id == "failed-request"
    assert invalid.reason_code == "c2_blend_infeasible"
    assert node._trajectory_plan_id == "old-plan"
    assert node._timeline_version == 1

    node.accept_command(robot_bridge.LatchMeasuredHoldCommand(4, "recovery-session", 1, True))
    holding = node._events[-1]
    assert holding.event_type == "measured_holding"
    assert holding.timeline_version == 2
    assert node._active_request_id is None
    assert node._pending_rtc is None
    held_action = node._trajectory_hold_action

    node.accept_command(
        robot_bridge.StageRtcChunkCommand(
            5,
            "recovery-session",
            "old-plan",
            "late-plan",
            1,
            1,
            "failed-request",
            2,
            robot_bridge.RTC_EXECUTION_HORIZON,
            unsafe,
        )
    )
    late_rejection = node._events[-1]
    assert late_rejection.event_type == "trajectory_command_rejected"
    assert node._timeline_version == 2
    assert node._trajectory_hold_action == held_action

    sync_knots = tuple(held_action for _ in range(robot_bridge.RTC_HORIZON))
    node._latest_observation = replace(node._latest_observation, seq=2)
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            6,
            2,
            "recovery-session",
            "sync-plan",
            2,
            sync_knots,
            5.0,
            robot_bridge.RTC_HORIZON,
            True,
            chunk_timeout_s=5.0,
            c2_handoff=True,
        )
    )
    sync_event_seq = node._event_seq
    sync_checkpoint = _drive_fake_bridge_until(
        node,
        clock,
        "checkpoint_ready",
        newer_than=sync_event_seq,
    )
    assert sync_checkpoint.plan_id == "sync-plan"
    assert sync_checkpoint.timeline_version == 3
    assert sync_checkpoint.reason_code == "chunk_clean"
    assert sync_checkpoint.deadline_monotonic is not None

    bootstrap_knots = tuple(
        _arm_action(robot_bridge.action_arms(held_action)[0] + index * 0.002)
        for index in range(robot_bridge.RTC_HORIZON)
    )
    node._latest_observation = replace(node._latest_observation, seq=3)
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            7,
            3,
            "recovery-session",
            "bootstrap-plan",
            3,
            bootstrap_knots,
            5.0,
            robot_bridge.RTC_EXECUTION_HORIZON,
            True,
            c2_handoff=True,
        )
    )
    bootstrap_event_seq = node._event_seq
    bootstrap_checkpoint = _drive_fake_bridge_until(
        node,
        clock,
        "checkpoint_ready",
        newer_than=bootstrap_event_seq,
    )
    assert bootstrap_checkpoint.plan_id == "bootstrap-plan"
    assert bootstrap_checkpoint.timeline_version == 4
    assert bootstrap_checkpoint.checkpoint_id == 3

    node.accept_command(
        robot_bridge.ResumeTrajectoryCommand(
            8,
            "recovery-session",
            "bootstrap-plan",
            4,
            3,
            "recovered-request",
            2,
        )
    )
    replacement = tuple(
        _arm_action(bootstrap_knots[10][0] + index * 0.002)
        for index in range(robot_bridge.RTC_HORIZON)
    )
    node.accept_command(
        robot_bridge.StageRtcChunkCommand(
            9,
            "recovery-session",
            "bootstrap-plan",
            "recovered-plan",
            4,
            3,
            "recovered-request",
            2,
            robot_bridge.RTC_EXECUTION_HORIZON,
            replacement,
        )
    )
    merged = _drive_fake_bridge_until(node, clock, "rtc_merged")
    assert merged.request_id == "recovered-request"
    assert merged.plan_id == "recovered-plan"
    assert merged.timeline_version == 5
    assert merged.actual_delay_steps == 1
    assert node._trajectory_session_id == "recovery-session"
    assert node._trajectory_plan_id == "recovered-plan"
