from collections import deque
import importlib
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


def test_gripper_feedback_parser_preserves_q_velocity_torque_and_temperatures():
    values = robot_bridge.MarvinBridgeNode._gripper_feedback_values(
        _Message(data=[0.4, -0.02, 0.31, 32.0, 30.0, 999.0])
    )

    assert values == (0.4, -0.02, 0.31, 32.0, 30.0)
    assert robot_bridge.MarvinBridgeNode._gripper_feedback_values(
        _Message(data=[0.4, -0.02])
    ) is None
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
    node._timeline_version = 7
    node._phase = None
    node._joints_t = None
    node._raw_reference = None
    node._sent_target = None
    node._tracking_error_rad = None
    node._servo_error_rad = None
    node._arm_clipped = False
    node._frozen_reason = None
    node._gripper_l_velocity = None
    node._gripper_r_velocity = None
    node._gripper_l_torque = None
    node._gripper_r_torque = None
    node._gripper_l_mos_temperature = None
    node._gripper_r_mos_temperature = None
    node._gripper_l_motor_temperature = None
    node._gripper_r_motor_temperature = None
    return node


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


def test_motion_gate_requires_fresh_measured_gripper_feedback():
    node = _bare_node()
    node.allow_motion = True
    node._client_connected = True
    node._joints = (0.0,) * 14
    node._joints_t = 10.0
    node.max_state_age_s = 0.20
    node._gripper_l = 0.4
    node._gripper_r = 0.7
    node._gripper_l_t = node._gripper_r_t = 9.0
    node._input_mode = 3
    node._robot_state = node._arm_state = (3, 3)
    node._robot_state_t = node._arm_state_t = 10.0
    node.max_status_age_s = 0.50

    ready, reason = node._readiness_gate_locked(10.1)

    assert not ready
    assert reason == "left gripper feedback is stale"


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
    rtc_knots = tuple(_action(0.02 + index * 0.004) for index in range(10))
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


def test_fake_bridge_checkpoint_resume_and_atomic_rtc_merge(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(robot_bridge, "_now", lambda: clock[0])
    node = robot_bridge.MarvinBridgeNode(
        allow_motion=True,
        publish_hz=100.0,
        command_timeout_s=0.25,
        max_joint_step_rad=0.12,
        max_state_age_s=0.20,
        max_status_age_s=0.50,
        max_observation_lag=8,
        joint_limit_margin_rad=0.02,
        trajectory_state_timeout_s=0.05,
        trajectory_timer_timeout_s=0.05,
        trajectory_heartbeat_timeout_s=0.25,
        tracking_run_error_rad=0.01,
        tracking_resume_error_rad=0.03,
        tracking_stop_error_rad=0.04,
        tracking_tolerance_rad=0.01,
        tracking_settle_seconds=0.20,
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
    old_knots = tuple(_arm_action(index * 0.005) for index in range(10))
    node.accept_command(
        robot_bridge.LoadTrajectoryCommand(
            command_id=1,
            observation_seq=1,
            session_id="session",
            plan_id="old-plan",
            expected_timeline_version=0,
            knots=old_knots,
            knot_hz=7.5,
            checkpoint_horizon=6,
            execute=True,
        )
    )

    for _ in range(200):
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
    assert checkpoint.phase == 5.0
    assert checkpoint.checkpoint_id == 1
    assert checkpoint.old_remaining_actions_absolute == old_knots[6:]
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
    rtc_knots = tuple(_arm_action(0.030 + index * 0.004) for index in range(10))
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
            execution_horizon=6,
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
    assert node._timeline.knots[0] == old_knots[6]
    assert all(abs(value - 0.005) < 1e-12 for value in merged.boundary_old_velocity)
    assert all(abs(value - 0.004) < 1e-12 for value in merged.boundary_new_velocity)
    assert abs(merged.boundary_velocity_jump_rad - 0.001) < 1e-12
    assert merged.boundary_acceleration_jump_rad < 1e-12
