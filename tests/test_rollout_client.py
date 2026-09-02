from contextlib import redirect_stderr
from contextlib import redirect_stdout
import io
import csv
import json
import logging
from pathlib import Path
import socket
import tempfile
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from marvinpro_deploy.protocol import BridgeHello, RobotStateUpdate, TrajectoryEvent, send_message
from marvinpro_deploy.rtc import RTC_HORIZON, RtcError
from marvinpro_deploy.safety import SafetyError
from marvinpro_deploy.rollout_client import (
    BridgeCommandRejected,
    _CommandIds,
    _RecordingNotifier,
    _TrajectoryHeartbeat,
    _actions_tuple,
    _confirm_execution,
    _confirm_and_refresh_execution_observation,
    _classify_rtc_failure,
    _configure_logging,
    _is_observation_lag_rejection,
    _latch_measured_hold_with_retry,
    _run_bridge_synchronized,
    _run_trajectory_schedule,
    _state_log_interval_s,
    _validate_rtc_policy_metadata,
    ActionPlan,
    ActionPublisher,
    JointTelemetryRecorder,
    RobotConnection,
    RolloutError,
    TimedSynchronizedResult,
    parse_args,
    run,
    validate_observation,
)


def vector(value: float) -> tuple[float, ...]:
    return (value,) * 16


class InterpolatedActionPlanTest(unittest.TestCase):
    def test_interpolates_from_anchor_at_command_rate(self):
        plan = ActionPlan()
        appended = plan.append_interpolated(
            np.asarray((vector(2.0), vector(4.0))),
            observation_seq=12,
            execute_steps=2,
            fallback_anchor=vector(0.0),
            model_hz=2.0,
            playback_time_scale=2.0,
            command_hz=4.0,
        )

        self.assertEqual(appended.queued_steps, 0)
        self.assertEqual(appended.added_steps, 8)
        self.assertEqual(appended.anchor_action, vector(0.0))
        self.assertEqual(appended.final_action, vector(4.0))
        steps = [plan.pop() for _ in range(8)]
        self.assertEqual([step.action[0] for step in steps], [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0])
        self.assertEqual({step.observation_seq for step in steps}, {12})
        self.assertIsNone(plan.pop())

    def test_trajectory_actions_project_only_grippers_to_policy_domain(self):
        actions = np.arange(RTC_HORIZON * 16, dtype=np.float64).reshape(RTC_HORIZON, 16) / 100.0
        actions[:, 7] = np.linspace(-0.01, 1.01, RTC_HORIZON)
        actions[:, 15] = np.linspace(1.02, -0.02, RTC_HORIZON)
        original_arms = np.concatenate((actions[:, :7], actions[:, 8:15]), axis=1).copy()

        prepared = np.asarray(_actions_tuple(actions))

        np.testing.assert_array_equal(
            np.concatenate((prepared[:, :7], prepared[:, 8:15]), axis=1),
            original_arms,
        )
        self.assertTrue(np.all((0.0 <= prepared[:, 7]) & (prepared[:, 7] <= 1.0)))
        self.assertTrue(np.all((0.0 <= prepared[:, 15]) & (prepared[:, 15] <= 1.0)))
        self.assertEqual(prepared[0, 7], 0.0)
        self.assertEqual(prepared[0, 15], 1.0)

    def test_appends_after_existing_tail_without_replacement(self):
        plan = ActionPlan()
        plan.append_interpolated(
            np.asarray((vector(1.0),)),
            observation_seq=1,
            execute_steps=1,
            fallback_anchor=vector(0.0),
            model_hz=1.0,
            playback_time_scale=1.0,
            command_hz=2.0,
        )
        appended = plan.append_interpolated(
            np.asarray((vector(2.0),)),
            observation_seq=2,
            execute_steps=1,
            fallback_anchor=vector(-10.0),
            model_hz=1.0,
            playback_time_scale=1.0,
            command_hz=2.0,
        )

        self.assertEqual(appended.queued_steps, 2)
        self.assertEqual(appended.anchor_action, vector(1.0))
        steps = [plan.pop() for _ in range(4)]
        self.assertEqual([step.action[0] for step in steps], [0.5, 1.0, 1.5, 2.0])
        self.assertEqual([step.observation_seq for step in steps], [1, 1, 2, 2])


class FakeRobotConnection:
    def __init__(self):
        self.commands = []
        self.observation = SimpleNamespace(
            seq=1,
            joints=(0.0,) * 14,
            gripper_raw_left=0.0,
            gripper_raw_right=0.0,
            motion_gate_open=True,
            gate_reason="ready",
        )

    def latest(self, max_local_age_s=None):
        return self.observation

    def send_action(self, command):
        self.commands.append(command)


class ActionPublisherTest(unittest.TestCase):
    def test_shutdown_hold_latches_once_and_does_not_follow_feedback(self):
        connection = FakeRobotConnection()
        plan = ActionPlan()
        plan.replace(np.asarray((vector(0.02),)), observation_seq=1, execute_steps=1)
        stop = threading.Event()
        publisher = ActionPublisher(
            connection,
            plan,
            stop,
            execute=True,
            control_hz=200.0,
            max_joint_step_rad=0.08,
            max_observation_age_s=0.35,
            joint_limit_margin_rad=0.02,
            warn_on_plan_empty=False,
            refresh_observation_seq=True,
            hold_last_plan_action=True,
        )
        publisher.start()
        try:
            deadline = time.monotonic() + 0.5
            while len(connection.commands) < 3 and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertGreaterEqual(len(connection.commands), 3)
            self.assertEqual(connection.commands[0].action, vector(0.02))
            self.assertEqual(connection.commands[1].action, vector(0.02))
            snapshot = publisher.snapshot()
            self.assertEqual(snapshot.plan_steps_sent, 1)
            self.assertEqual(snapshot.latched_plan_action, vector(0.02))

            publisher.hold_fixed_pose(vector(0.0))
            connection.observation.joints = (0.01,) * 14
            connection.observation.seq += 1
            deadline = time.monotonic() + 0.5
            while connection.commands[-1].action != vector(0.0) and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertEqual(connection.commands[-1].action, vector(0.0))
            self.assertEqual(publisher.snapshot().latched_plan_action, vector(0.0))
        finally:
            stop.set()
            publisher.join(timeout=1.0)


class RobotConnectionTest(unittest.TestCase):
    def test_late_event_cannot_reappear_after_newer_event_was_consumed(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        accepted = []
        ready = threading.Event()

        def accept_client():
            bridge_socket, _ = listener.accept()
            accepted.append(bridge_socket)
            send_message(bridge_socket, BridgeHello())
            ready.set()

        accept_thread = threading.Thread(target=accept_client, daemon=True)
        accept_thread.start()
        connection = RobotConnection("127.0.0.1", listener.getsockname()[1])
        self.assertTrue(ready.wait(1.0))
        bridge_socket = accepted[0]
        try:
            first = TrajectoryEvent(10, "checkpoint_ready", 1.0, "s", "p", 1, 3.0)
            send_message(bridge_socket, first)
            self.assertEqual(connection.wait_for_event(timeout_s=1.0).event_seq, 10)

            send_message(
                bridge_socket,
                TrajectoryEvent(9, "checkpoint_ready", 1.1, "s", "p", 1, 3.0),
            )
            send_message(
                bridge_socket,
                TrajectoryEvent(11, "checkpoint_ready", 1.2, "s", "p", 1, 3.0),
            )
            self.assertEqual(connection.wait_for_event(timeout_s=1.0).event_seq, 11)
        finally:
            connection.close("test complete")
            bridge_socket.close()
            listener.close()
            accept_thread.join(timeout=1.0)

    def test_rtc_failure_classification_prefers_structured_bridge_reason(self):
        event = TrajectoryEvent(
            1,
            "rtc_invalid",
            1.0,
            "session",
            "plan",
            2,
            3.0,
            reason_code="c2_blend_infeasible",
        )

        failure = _classify_rtc_failure(RtcError("unstructured detail"), event)

        self.assertEqual(failure.reason_code, "c2_blend_infeasible")
        self.assertTrue(failure.recoverable)

    def test_bridge_command_rejection_preserves_structured_c2_reason(self):
        event = TrajectoryEvent(
            1,
            "trajectory_command_rejected",
            1.0,
            "session",
            "plan",
            2,
            0.0,
            reason_code="c2_blend_infeasible",
            detail="trajectory C2 handoff is infeasible",
        )

        failure = _classify_rtc_failure(
            BridgeCommandRejected("bridge rejected trajectory", event)
        )

        self.assertEqual(failure.reason_code, "c2_blend_infeasible")
        self.assertTrue(failure.recoverable)

    def test_rtc_failure_classification_keeps_safety_faults_fatal(self):
        clipping = _classify_rtc_failure(RtcError("bridge arm clipping detected"))
        mismatch = _classify_rtc_failure(RtcError("response request ID mismatch"))

        self.assertEqual(clipping.reason_code, "arm_clipping")
        self.assertFalse(clipping.recoverable)
        self.assertEqual(mismatch.reason_code, "transaction_mismatch")
        self.assertFalse(mismatch.recoverable)

    def test_shadow_discard_uses_fallback_without_reentering_rtc(self):
        failure = _classify_rtc_failure(RtcError("RTC shadow result discarded at d_actual=2"))

        self.assertEqual(failure.reason_code, "rtc_shadow")
        self.assertTrue(failure.recoverable)

    def test_rtc_metadata_validation_checks_the_complete_contract(self):
        metadata = {
            "rtc": {
                "protocol": "rtc_v1",
                "action_horizon": 20,
                "native_action_dim": 16,
                "model_action_dim": 32,
                "execution_horizon": 10,
                "max_predicted_delay": 4,
                "prefix_attention_schedule": "exp",
            }
        }
        _validate_rtc_policy_metadata(metadata)

        metadata["rtc"]["prefix_attention_schedule"] = "linear"
        with self.assertRaisesRegex(RolloutError, "metadata mismatch"):
            _validate_rtc_policy_metadata(metadata)


class TimedSynchronizedRunnerTest(unittest.TestCase):
    @staticmethod
    def _args():
        return SimpleNamespace(
            model_hz=15.0,
            playback_time_scale=3.0,
            sync_chunk_timeout_grace=1.0,
            tracking_timeout=1.0,
            max_stuck_replans=2,
            tracking_tolerance_rad=0.01,
            tracking_settle_seconds=0.20,
            max_source_age=0.20,
            max_state_image_skew=0.05,
            prompt="test",
        )

    def test_two_consecutive_healthy_timeouts_exhaust_replans(self):
        observation = SimpleNamespace(seq=1)
        state = SimpleNamespace(
            timeline_version=2,
            motion_gate_open=True,
            gate_reason="ready",
            arm_clipped=False,
            frozen_reason="hold",
        )
        events = [
            TrajectoryEvent(2, "chunk_timed_out", 5.0, "s", "p1", 2, 10.0),
            TrajectoryEvent(4, "chunk_timed_out", 10.0, "s", "p2", 4, 11.0),
        ]
        connection = SimpleNamespace(
            latest_state=lambda max_local_age_s=None: state,
            wait_for_event=lambda **kwargs: events.pop(0),
        )
        heartbeat = SimpleNamespace(error=None, update_version=lambda version: None)
        actions = np.zeros((RTC_HORIZON, 16))
        loaded_versions = iter((1, 3))

        with (
            patch(
                "marvinpro_deploy.rollout_client._load_bridge_trajectory",
                side_effect=lambda *args, **kwargs: TrajectoryEvent(
                    1, "trajectory_loaded", 0.0, "s", "p", next(loaded_versions), 0.0
                ),
            ),
            patch(
                "marvinpro_deploy.rollout_client._wait_bridge_tracking",
                return_value=(state, 1.0),
            ),
            patch(
                "marvinpro_deploy.rollout_client._fresh_observation_after_source_time",
                return_value=SimpleNamespace(seq=2),
            ),
            patch(
                "marvinpro_deploy.rollout_client.infer_actions",
                return_value=(actions, {"wall_ms": 100.0}),
            ),
        ):
            result = _run_bridge_synchronized(
                self._args(),
                connection,
                object(),
                SimpleNamespace(),
                heartbeat,
                session_id="s",
                initial_observation=observation,
                initial_actions=actions,
                episode_deadline=time.monotonic() + 30.0,
            )

        self.assertTrue(result.exhausted)
        self.assertEqual(result.stuck_replans, 2)
        self.assertEqual(result.inference_count, 1)

    def test_clean_chunk_resets_stuck_counter_before_rtc_bootstrap(self):
        observation = SimpleNamespace(seq=1)
        fresh = SimpleNamespace(seq=3)
        state = SimpleNamespace(
            timeline_version=2,
            motion_gate_open=True,
            gate_reason="ready",
            arm_clipped=False,
            frozen_reason="hold",
        )
        events = [
            TrajectoryEvent(2, "chunk_timed_out", 5.0, "s", "p1", 2, 10.0),
            TrajectoryEvent(4, "checkpoint_ready", 9.0, "s", "p2", 3, 19.0),
        ]
        connection = SimpleNamespace(
            latest_state=lambda max_local_age_s=None: state,
            wait_for_event=lambda **kwargs: events.pop(0),
        )
        heartbeat = SimpleNamespace(error=None, update_version=lambda version: None)
        actions = np.zeros((RTC_HORIZON, 16))

        with (
            patch(
                "marvinpro_deploy.rollout_client._load_bridge_trajectory",
                side_effect=(
                    TrajectoryEvent(1, "trajectory_loaded", 0.0, "s", "p1", 1, 0.0),
                    TrajectoryEvent(3, "trajectory_loaded", 6.0, "s", "p2", 3, 0.0),
                ),
            ),
            patch(
                "marvinpro_deploy.rollout_client._wait_bridge_tracking",
                return_value=(state, 1.0),
            ),
            patch(
                "marvinpro_deploy.rollout_client._fresh_observation_after_source_time",
                return_value=SimpleNamespace(seq=2),
            ),
            patch(
                "marvinpro_deploy.rollout_client._wait_checkpoint_observation",
                return_value=fresh,
            ),
            patch(
                "marvinpro_deploy.rollout_client.infer_actions",
                return_value=(actions, {"wall_ms": 100.0}),
            ),
        ):
            result = _run_bridge_synchronized(
                self._args(),
                connection,
                object(),
                SimpleNamespace(),
                heartbeat,
                session_id="s",
                initial_observation=observation,
                initial_actions=actions,
                episode_deadline=time.monotonic() + 30.0,
                required_clean_chunks=1,
            )

        self.assertFalse(result.exhausted)
        self.assertEqual(result.clean_chunks, 1)
        self.assertEqual(result.stuck_replans, 0)
        self.assertIs(result.observation, fresh)

    def test_exhausted_rtc_recovery_reobserves_after_c2_handoff_rejection(self):
        observation = SimpleNamespace(seq=10)
        fresh = SimpleNamespace(seq=20)
        state = SimpleNamespace(
            timeline_version=8,
            motion_gate_open=True,
            gate_reason="ready",
            arm_clipped=False,
            frozen_reason="hold",
        )
        rejected = TrajectoryEvent(
            4,
            "trajectory_command_rejected",
            1.0,
            "s",
            "rejected",
            8,
            0.0,
            reason_code="c2_blend_infeasible",
            detail="trajectory C2 handoff is infeasible",
        )
        checkpoint = TrajectoryEvent(6, "checkpoint_ready", 6.0, "s", "clean", 9, 19.0)
        connection = SimpleNamespace(
            latest_state=lambda max_local_age_s=None: state,
            wait_for_event=lambda **kwargs: checkpoint,
        )
        heartbeat = SimpleNamespace(error=None, update_version=lambda version: None)
        actions = np.zeros((RTC_HORIZON, 16))

        with (
            patch(
                "marvinpro_deploy.rollout_client._load_bridge_trajectory",
                side_effect=(
                    BridgeCommandRejected("bridge rejected trajectory", rejected),
                    TrajectoryEvent(5, "trajectory_loaded", 2.0, "s", "clean", 9, 0.0),
                ),
            ),
            patch(
                "marvinpro_deploy.rollout_client._wait_bridge_tracking",
                return_value=(state, 1.0),
            ) as wait_tracking,
            patch(
                "marvinpro_deploy.rollout_client._fresh_observation_after_source_time",
                return_value=fresh,
            ) as reobserve,
            patch(
                "marvinpro_deploy.rollout_client._wait_checkpoint_observation",
                return_value=SimpleNamespace(seq=30),
            ),
            patch(
                "marvinpro_deploy.rollout_client.infer_actions",
                return_value=(actions, {"wall_ms": 100.0}),
            ),
        ):
            result = _run_bridge_synchronized(
                self._args(),
                connection,
                object(),
                SimpleNamespace(),
                heartbeat,
                session_id="s",
                initial_observation=observation,
                initial_actions=actions,
                episode_deadline=time.monotonic() + 30.0,
                required_clean_chunks=1,
                retry_c2_handoff_rejections=True,
            )

        wait_tracking.assert_called_once()
        reobserve.assert_called_once()
        self.assertEqual(result.clean_chunks, 1)
        self.assertEqual(result.stuck_replans, 0)
        self.assertEqual(result.inference_count, 1)

    def test_exhausted_rtc_recovery_stops_after_two_c2_handoff_rejections(self):
        observation = SimpleNamespace(seq=10)
        fresh = SimpleNamespace(seq=20)
        state = SimpleNamespace(
            timeline_version=8,
            motion_gate_open=True,
            gate_reason="ready",
            arm_clipped=False,
            frozen_reason="hold",
        )
        rejection = TrajectoryEvent(
            4,
            "trajectory_command_rejected",
            1.0,
            "s",
            "rejected",
            8,
            0.0,
            reason_code="c2_blend_infeasible",
            detail="trajectory C2 handoff is infeasible",
        )
        connection = SimpleNamespace(latest_state=lambda max_local_age_s=None: state)
        heartbeat = SimpleNamespace(error=None, update_version=lambda version: None)
        actions = np.zeros((RTC_HORIZON, 16))

        with (
            patch(
                "marvinpro_deploy.rollout_client._load_bridge_trajectory",
                side_effect=(
                    BridgeCommandRejected("first rejection", rejection),
                    BridgeCommandRejected("second rejection", rejection),
                ),
            ),
            patch(
                "marvinpro_deploy.rollout_client._wait_bridge_tracking",
                return_value=(state, 1.0),
            ),
            patch(
                "marvinpro_deploy.rollout_client._fresh_observation_after_source_time",
                return_value=fresh,
            ),
            patch(
                "marvinpro_deploy.rollout_client.infer_actions",
                return_value=(actions, {"wall_ms": 100.0}),
            ),
        ):
            result = _run_bridge_synchronized(
                self._args(),
                connection,
                object(),
                SimpleNamespace(),
                heartbeat,
                session_id="s",
                initial_observation=observation,
                initial_actions=actions,
                episode_deadline=time.monotonic() + 30.0,
                retry_c2_handoff_rejections=True,
            )

        self.assertTrue(result.exhausted)
        self.assertEqual(result.stuck_replans, 2)
        self.assertEqual(result.inference_count, 1)

    def test_third_recovery_failure_switches_to_synchronized_fallback(self):
        observation = SimpleNamespace(seq=10)
        state = SimpleNamespace(
            timeline_version=8,
            motion_gate_open=True,
            gate_reason="ready",
            arm_clipped=False,
            frozen_reason="hold",
        )
        checkpoint = TrajectoryEvent(
            1,
            "checkpoint_ready",
            1.0,
            "s",
            "initial",
            1,
            10.0,
            checkpoint_id=1,
        )
        loaded = TrajectoryEvent(2, "trajectory_loaded", 0.0, "s", "initial", 1, 0.0)
        rejection = TrajectoryEvent(
            3,
            "trajectory_command_rejected",
            1.0,
            "s",
            "rejected",
            8,
            0.0,
            reason_code="c2_blend_infeasible",
            detail="trajectory C2 handoff is infeasible",
        )
        actions = np.zeros((RTC_HORIZON, 16))
        sync_result = TimedSynchronizedResult(
            inference_count=0,
            observation=SimpleNamespace(seq=30),
            clean_chunks=1,
            stuck_replans=0,
            exhausted=False,
        )
        connection = SimpleNamespace(
            latest_state=lambda max_local_age_s=None: state,
            latest=lambda: SimpleNamespace(input_mode=0),
            wait_for_event=lambda **kwargs: checkpoint,
            poll_event=lambda **kwargs: None,
            send=lambda command: None,
        )
        policy = SimpleNamespace(
            infer=lambda request: (_ for _ in ()).throw(ConnectionError("transport timeout")),
            close=lambda: None,
        )

        class FakeHeartbeat:
            error = None

            def start(self):
                return None

            def join(self):
                return None

            def update_version(self, version):
                return None

        with (
            patch("marvinpro_deploy.rollout_client._TrajectoryHeartbeat", return_value=FakeHeartbeat()),
            patch(
                "marvinpro_deploy.rollout_client.infer_actions",
                return_value=(
                    actions,
                    {
                        "wall_ms": 100.0,
                        "observation_preparation_ms": 0.0,
                        "client_timing": {},
                        "policy_timing": {},
                        "server_timing": {},
                    },
                ),
            ),
            patch("marvinpro_deploy.rollout_client.build_policy_observation", return_value={}),
            patch("marvinpro_deploy.rollout_client.build_rtc_request", return_value={}),
            patch("marvinpro_deploy.rollout_client._load_bridge_trajectory", return_value=loaded),
            patch(
                "marvinpro_deploy.rollout_client._wait_checkpoint_observation",
                return_value=observation,
            ),
            patch(
                "marvinpro_deploy.rollout_client._latch_measured_bridge_position",
                return_value=loaded,
            ) as latch_hold,
            patch(
                "marvinpro_deploy.rollout_client._wait_bridge_tracking",
                return_value=(state, 1.0),
            ),
            patch(
                "marvinpro_deploy.rollout_client._fresh_observation_after_source_time",
                return_value=observation,
            ),
            patch(
                "marvinpro_deploy.rollout_client._reconnect_policy",
                return_value=(1, {"rtc": {}}),
            ) as reconnect,
            patch(
                "marvinpro_deploy.rollout_client._run_bridge_synchronized",
                side_effect=(
                    BridgeCommandRejected("first", rejection),
                    BridgeCommandRejected("second", rejection),
                    BridgeCommandRejected("third", rejection),
                    sync_result,
                ),
            ) as run_sync,
            patch("marvinpro_deploy.rollout_client._hold_bridge_position", return_value=loaded),
            patch("marvinpro_deploy.rollout_client._wait_for_none_after_trajectory"),
        ):
            result = _run_trajectory_schedule(
                parse_args(
                    [
                        "--robot-host",
                        "127.0.0.1",
                        "--policy-host",
                        "127.0.0.1",
                    "--episode-seconds",
                        "300",
                        "--execute",
                        "--rollout-schedule",
                        "rtc",
                        "--playback-mode",
                        "interpolated",
                        "--control-hz",
                        "100",
                        "--model-hz",
                        "15",
                        "--playback-time-scale",
                        "3",
                        "--execute-steps",
                        "20",
                    ]
                ),
                connection,
                policy,
                observation,
                [],
            )

        self.assertEqual(result, 1)
        self.assertEqual(run_sync.call_count, 4)
        self.assertEqual(reconnect.call_count, 4)
        self.assertGreaterEqual(latch_hold.call_count, 4)
        fourth_kwargs = run_sync.call_args_list[3].kwargs
        self.assertFalse(fourth_kwargs["required_clean_chunks"] is not None)
        self.assertTrue(fourth_kwargs["retry_c2_handoff_rejections"])


    def test_execution_confirmation_forces_a_new_observation(self):
        ready_observation = SimpleNamespace(seq=10)
        latest_after_confirmation = SimpleNamespace(seq=72)
        fresh_observation = SimpleNamespace(
            seq=73,
            captured_monotonic=100.0,
            joints=(0.0,) * 14,
            image=b"jpeg",
            age_state_s=0.001,
            age_gripper_left_s=0.002,
            age_gripper_right_s=0.003,
            gripper_raw_left=0.4,
            gripper_raw_right=0.6,
            gripper_torque_left=0.1,
            gripper_torque_right=0.2,
            extra={"gripper_state_source": "measured_feedback"},
        )

        class FakeConnection:
            def __init__(self):
                self.wait_kwargs = None

            def latest(self, max_local_age_s=None):
                self.latest_max_age = max_local_age_s
                return latest_after_confirmation

            def wait_for_observation(self, **kwargs):
                self.wait_kwargs = kwargs
                return fresh_observation

        connection = FakeConnection()
        args = SimpleNamespace(
            max_observation_age=0.35,
            observation_timeout=10.0,
            max_source_age=0.20,
        )
        with patch("marvinpro_deploy.rollout_client._confirm_execution") as confirm:
            result = _confirm_and_refresh_execution_observation(args, connection, ready_observation)

        confirm.assert_called_once_with(args, ready_observation)
        self.assertIs(result, fresh_observation)
        self.assertEqual(connection.latest_max_age, 0.35)
        self.assertEqual(
            connection.wait_kwargs,
            {"timeout_s": 10.0, "newer_than": 72, "require_motion_gate": True},
        )

    def test_heartbeat_does_not_depend_on_outbound_state_telemetry(self):
        sent = []
        connection = SimpleNamespace(send=sent.append)
        stop = threading.Event()
        heartbeat = _TrajectoryHeartbeat(connection, "session", stop)
        heartbeat.update_version(7)
        heartbeat.start()
        try:
            deadline = time.monotonic() + 0.5
            while not sent and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(sent)
            self.assertEqual(sent[0].session_id, "session")
            self.assertEqual(sent[0].timeline_version, 7)
        finally:
            stop.set()
            heartbeat.join()


class JointTelemetryRecorderTest(unittest.TestCase):
    def test_records_measured_gripper_feedback_and_command(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "rollout.telemetry.csv"
            recorder = JointTelemetryRecorder(path)
            state = RobotStateUpdate(
                5,
                12.5,
                (0.1,) * 14,
                0.2,
                0.3,
                True,
                "ready",
                trajectory_mode="trajectory",
                timeline_version=3,
                phase=2.5,
                phase_rate=1.0,
                raw_reference=vector(1.0),
                sent_target=vector(2.0),
                gripper_velocity_left=0.01,
                gripper_velocity_right=0.02,
                gripper_torque_left=0.11,
                gripper_torque_right=0.12,
                gripper_mos_temperature_left=31.0,
                gripper_mos_temperature_right=32.0,
                gripper_motor_temperature_left=29.0,
                gripper_motor_temperature_right=30.0,
                gripper_position_raw_left=0.25,
                gripper_position_raw_right=0.375,
            )
            recorder.record_state(state, 20.0)
            observation = SimpleNamespace(
                seq=8,
                captured_monotonic=13.0,
                joints=(0.4,) * 14,
                gripper_raw_left=0.5,
                gripper_raw_right=0.6,
            )
            recorder.record_client_command(
                recorded_monotonic=21.0,
                observation=observation,
                command_id=9,
                requested_action=vector(3.0),
                sent_action=vector(2.9),
                was_hold=False,
            )
            recorder.close()

            with path.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["record_type"] for row in rows], ["bridge_state", "client_command"])
            self.assertEqual(float(rows[0]["measured_Joint1_L"]), 0.1)
            self.assertEqual(float(rows[0]["gripper_command_L"]), 2.0)
            self.assertEqual(float(rows[0]["measured_gripper_position_raw_L"]), 0.25)
            self.assertEqual(float(rows[0]["measured_gripper_position_L"]), 0.2)
            self.assertEqual(float(rows[0]["measured_gripper_velocity_L"]), 0.01)
            self.assertEqual(float(rows[0]["measured_gripper_torque_L"]), 0.11)
            self.assertEqual(float(rows[0]["gripper_position_error_L"]), 1.8)
            self.assertEqual(float(rows[0]["measured_gripper_mos_temperature_L"]), 31.0)
            self.assertEqual(float(rows[0]["measured_gripper_motor_temperature_L"]), 29.0)
            self.assertEqual(float(rows[0]["bridge_command_Joint1_L"]), 2.0)
            self.assertEqual(float(rows[1]["client_reference_Joint1_L"]), 3.0)
            self.assertEqual(float(rows[1]["client_command_Joint1_L"]), 2.9)


class RolloutArgumentTest(unittest.TestCase):
    def test_observation_validation_requires_fresh_normalized_gripper_feedback(self):
        observation = SimpleNamespace(
            joints=(0.0,) * 14,
            image=b"image",
            gripper_raw_left=0.2,
            gripper_raw_right=0.8,
            age_state_s=0.01,
            age_gripper_left_s=0.01,
            age_gripper_right_s=0.01,
        )
        validate_observation(observation, 0.05)

        observation.age_gripper_right_s = 0.06
        with self.assertRaisesRegex(RolloutError, "right gripper feedback is stale"):
            validate_observation(observation, 0.05)

        observation.age_gripper_right_s = 0.01
        observation.gripper_raw_right = 1.1
        with self.assertRaisesRegex(RolloutError, "invalid normalized gripper feedback"):
            validate_observation(observation, 0.05)

    def test_observation_lag_rejection_is_retryable_but_other_rejections_are_not(self):
        self.assertTrue(_is_observation_lag_rejection(RolloutError("bridge rejected trajectory: action observation lag is 17 frames (limit 8)")))
        self.assertFalse(_is_observation_lag_rejection(RolloutError("bridge rejected trajectory: robot_state=(3, 12)")))

    def test_observation_lag_rejection_uses_structured_reason_code(self):
        event = TrajectoryEvent(
            1,
            "trajectory_command_rejected",
            1.0,
            "session",
            "plan",
            2,
            0.0,
            reason_code="observation_lag",
            detail="action observation lag is 13 frames (limit 8)",
        )

        failure = _classify_rtc_failure(BridgeCommandRejected("bridge rejected trajectory", event))

        self.assertEqual(failure.reason_code, "observation_lag")
        self.assertTrue(failure.recoverable)

    def test_latch_measured_hold_retries_once_with_freshly_sampled_version(self):
        class FakeConnection:
            def __init__(self):
                self.attempt = 0
                self.commands = []

            def latest_state(self, max_local_age_s):
                return SimpleNamespace(timeline_version=41 if self.attempt == 0 else 42)

            def send(self, message):
                self.commands.append(message)

            def wait_for_event(self, *, timeout_s, event_types):
                self.attempt += 1
                if self.attempt == 1:
                    return TrajectoryEvent(
                        1,
                        "trajectory_command_rejected",
                        1.0,
                        "s",
                        "p",
                        42,
                        0.0,
                        reason_code="command_rejected",
                        detail="measured hold command timeline version mismatch",
                    )
                return TrajectoryEvent(
                    2, "measured_holding", 1.1, "s", "p", 42, 0.0, reason_code="rtc_late"
                )

        connection = FakeConnection()
        holding = _latch_measured_hold_with_retry(
            connection,
            _CommandIds(),
            session_id="s",
            reason="RTC failure",
            reason_code="rtc_late",
            timeout_s=1.0,
        )

        self.assertEqual(holding.event_type, "measured_holding")
        self.assertEqual(holding.timeline_version, 42)
        self.assertEqual(
            [command.expected_timeline_version for command in connection.commands],
            [41, 42],
        )

    def test_latch_measured_hold_retry_exhaustion_raises(self):
        class FakeConnection:
            def __init__(self):
                self.attempt = 0

            def latest_state(self, max_local_age_s):
                return SimpleNamespace(timeline_version=41 + self.attempt)

            def send(self, message):
                pass

            def wait_for_event(self, *, timeout_s, event_types):
                self.attempt += 1
                return TrajectoryEvent(
                    self.attempt,
                    "trajectory_command_rejected",
                    1.0,
                    "s",
                    "p",
                    41 + self.attempt,
                    0.0,
                    reason_code="command_rejected",
                    detail="measured hold command timeline version mismatch",
                )

        with self.assertRaises(RolloutError):
            _latch_measured_hold_with_retry(
                FakeConnection(),
                _CommandIds(),
                session_id="s",
                reason="RTC failure",
                reason_code="rtc_late",
                timeout_s=1.0,
            )

    def test_execution_confirmation_accepts_only_single_uppercase_e(self):
        args = parse_args(["--execute"])
        observation = SimpleNamespace(
            seq=1,
            input_mode=3,
            robot_state=(3, 3),
            arm_state=(3, 3),
        )
        with patch("builtins.input", return_value="E"), redirect_stdout(io.StringIO()):
            _confirm_execution(args, observation)
        with (
            patch("builtins.input", return_value="EXECUTE"),
            redirect_stdout(io.StringIO()),
            self.assertRaises(RolloutError),
        ):
            _confirm_execution(args, observation)

    def test_interpolated_two_times_configuration(self):
        args = parse_args(
            [
                "--playback-mode",
                "interpolated",
                "--control-hz",
                "100",
                "--model-hz",
                "15",
                "--playback-time-scale",
                "2",
                "--execute-steps",
                str(RTC_HORIZON),
                "--chunk-prefetch-seconds",
                "0.30",
            ]
        )

        self.assertEqual(args.playback_mode, "interpolated")
        self.assertEqual(args.control_hz, 100.0)
        self.assertEqual(args.model_hz, 15.0)
        self.assertEqual(args.playback_time_scale, 2.0)

    def test_log_file_is_preserved_in_configuration(self):
        args = parse_args(
            [
                "--log-level",
                "DEBUG",
                "--console-log-level",
                "WARNING",
                "--log-file",
                "/tmp/marvinpro-rollout.log",
            ]
        )

        self.assertEqual(args.log_level, "DEBUG")
        self.assertEqual(args.console_log_level, "WARNING")
        self.assertEqual(args.log_file, "/tmp/marvinpro-rollout.log")

    def test_telemetry_file_is_preserved_in_configuration(self):
        args = parse_args(
            [
                "--log-file",
                "/tmp/marvinpro-rollout.log",
                "--telemetry-file",
                "/tmp/marvinpro-rollout.telemetry.csv",
            ]
        )
        self.assertEqual(args.telemetry_file, "/tmp/marvinpro-rollout.telemetry.csv")

    def test_hold_state_logging_is_throttled_more_than_active_motion(self):
        self.assertEqual(_state_log_interval_s("trajectory"), 0.10)
        self.assertEqual(_state_log_interval_s("hold"), 1.0)

    def test_log_file_keeps_debug_while_console_defaults_to_warning(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "rollout.log"
            args = parse_args(["--log-level", "DEBUG", "--log-file", str(log_path)])
            console = io.StringIO()
            with redirect_stderr(console), redirect_stdout(io.StringIO()):
                _configure_logging(args, [])
                self.assertEqual(args.telemetry_file, str(log_path.with_name("rollout.telemetry.csv")))
                logging.getLogger("marvinpro_rollout").debug("file-detail")
                logging.getLogger("marvinpro_rollout").warning("console-warning")
                for handler in logging.getLogger().handlers:
                    handler.flush()

            self.assertNotIn("file-detail", console.getvalue())
            self.assertIn("console-warning", console.getvalue())
            contents = log_path.read_text(encoding="utf-8")
            self.assertIn("file-detail", contents)
            self.assertIn("console-warning", contents)

    def test_synchronized_schedule_configuration(self):
        args = parse_args(
            [
                "--execute",
                "--rollout-schedule",
                "synchronized",
                "--playback-mode",
                "interpolated",
                "--control-hz",
                "100",
                "--model-hz",
                "15",
                "--playback-time-scale",
                "3",
                "--execute-steps",
                str(RTC_HORIZON),
            ]
        )

        self.assertEqual(args.rollout_schedule, "synchronized")
        self.assertEqual(args.tracking_tolerance_rad, 0.01)
        self.assertEqual(args.tracking_settle_seconds, 0.20)
        self.assertEqual(args.post_track_hold_seconds, 0.20)
        self.assertEqual(args.tracking_timeout, 5.0)
        self.assertEqual(args.sync_chunk_timeout_grace, 1.0)
        self.assertEqual(args.max_stuck_replans, 2)
        self.assertEqual(args.max_rtc_recoveries, 3)
        self.assertEqual(args.policy_connect_timeout, 5.0)
        self.assertEqual(args.policy_request_timeout, 2.0)

    def test_synchronized_schedule_requires_execution_and_interpolation(self):
        invalid_argv = (
            ["--rollout-schedule", "synchronized", "--playback-mode", "interpolated"],
            ["--execute", "--rollout-schedule", "synchronized"],
        )
        for argv in invalid_argv:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(argv)

    def test_rtc_schedule_requires_fixed_five_hz_configuration(self):
        args = parse_args(
            [
                "--execute",
                "--rollout-schedule",
                "rtc",
                "--playback-mode",
                "interpolated",
                "--control-hz",
                "100",
                "--model-hz",
                "15",
                "--playback-time-scale",
                "3",
                "--execute-steps",
                str(RTC_HORIZON),
                "--max-rtc-merges",
                "2",
                "--rtc-continuous",
            ]
        )
        self.assertEqual(args.rollout_schedule, "rtc")
        self.assertFalse(args.rtc_shadow)
        self.assertTrue(args.rtc_continuous)
        self.assertEqual(args.max_rtc_merges, 2)
        self.assertEqual(args.playback_time_scale, 3.0)
        self.assertEqual(args.max_joint_step_rad, 0.16)
        self.assertEqual(args.rtc_late_result_policy, "discard")

    def test_rtc_schedule_allows_native_fifteen_hz_playback_scale(self):
        args = parse_args(
            [
                "--execute",
                "--rollout-schedule",
                "rtc",
                "--playback-mode",
                "interpolated",
                "--control-hz",
                "100",
                "--model-hz",
                "15",
                "--playback-time-scale",
                "1",
                "--execute-steps",
                str(RTC_HORIZON),
                "--rtc-continuous",
            ]
        )

        self.assertEqual(args.rollout_schedule, "rtc")
        self.assertEqual(args.playback_time_scale, 1.0)

    def test_rtc_schedule_allows_wait_late_result_comparison_policy(self):
        args = parse_args(
            [
                "--execute",
                "--rollout-schedule",
                "rtc",
                "--playback-mode",
                "interpolated",
                "--control-hz",
                "100",
                "--model-hz",
                "15",
                "--playback-time-scale",
                "3",
                "--execute-steps",
                str(RTC_HORIZON),
                "--rtc-late-result-policy",
                "wait",
            ]
        )

        self.assertEqual(args.rtc_late_result_policy, "wait")

    def test_rtc_schedule_rejects_previous_seven_point_five_hz_rate(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(
                [
                    "--execute",
                    "--rollout-schedule",
                    "rtc",
                    "--playback-mode",
                    "interpolated",
                    "--control-hz",
                    "100",
                    "--model-hz",
                    "15",
                    "--playback-time-scale",
                    "2",
                    "--execute-steps",
                    "10",
                ]
            )

    def test_rtc_schedule_rejects_unvalidated_playback_scale(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(
                [
                    "--execute",
                    "--rollout-schedule",
                    "rtc",
                    "--playback-mode",
                    "interpolated",
                    "--control-hz",
                    "100",
                    "--model-hz",
                    "15",
                    "--playback-time-scale",
                    "4",
                    "--execute-steps",
                    "10",
                ]
            )

    def test_continuous_checkpoint_requires_rtc_schedule(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(["--rtc-continuous"])

    def test_rejects_speedup(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(
                    [
                        "--playback-mode",
                        "interpolated",
                        "--playback-time-scale",
                        "0.5",
                    ]
                )


class FakeCollector:
    """Minimal TCP server capturing one JSON line per connection."""

    def __init__(self, port: int = 0):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", port))
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.events = []
        self._lock = threading.Lock()
        self._closed = False
        self._thread = threading.Thread(target=self._serve, name="fake-collector", daemon=True)
        self._thread.start()

    def _serve(self):
        while not self._closed:
            try:
                conn, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with conn:
                conn.settimeout(1.0)
                data = b""
                try:
                    while not data.endswith(b"\n"):
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        data += chunk
                except OSError:
                    pass
            line = data.strip()
            if line:
                with self._lock:
                    self.events.append(json.loads(line.decode("utf-8")))

    def snapshot(self):
        with self._lock:
            return list(self.events)

    def wait_events(self, count: int, timeout_s: float = 2.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            events = self.snapshot()
            if len(events) >= count:
                return events
            time.sleep(0.01)
        return self.snapshot()

    def close(self):
        self._closed = True
        self.listener.close()
        self._thread.join(timeout=1.0)


def unused_tcp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]
    finally:
        sock.close()


class RecordingNotifierTest(unittest.TestCase):
    def test_notify_sends_one_json_line_per_event(self):
        collector = FakeCollector()
        try:
            notifier = _RecordingNotifier("127.0.0.1", collector.port)
            self.assertTrue(notifier.notify({"event": "ping", "value": 3}))
            self.assertEqual(collector.wait_events(1), [{"event": "ping", "value": 3}])
        finally:
            collector.close()

    def test_unreachable_collector_is_a_warning_not_an_exception(self):
        notifier = _RecordingNotifier("127.0.0.1", unused_tcp_port())
        with self.assertLogs("marvinpro_rollout", level="WARNING") as captured:
            self.assertFalse(notifier.probe())
            self.assertFalse(notifier.notify({"event": "ping"}))
        self.assertTrue(any("record_notify_unreachable" in line for line in captured.output))
        self.assertTrue(any("record_notify_failed" in line for line in captured.output))

    def test_episode_end_is_sent_at_most_once(self):
        collector = FakeCollector()
        try:
            notifier = _RecordingNotifier("127.0.0.1", collector.port)
            self.assertTrue(notifier.episode_start(task="stack cones", run_dir=None))
            notifier.episode_end("completed", "clean_completion")
            notifier.episode_end("operator_stopped", "operator interrupted rollout")
            events = collector.wait_events(2)
            time.sleep(0.1)
            self.assertEqual(events, collector.snapshot())
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[0]["task"], "stack cones")
            self.assertIsNone(events[0]["run_dir"])
            self.assertEqual(events[1]["status"], "completed")
            self.assertEqual(events[1]["reason"], "clean_completion")
        finally:
            collector.close()

    def test_episode_end_is_skipped_when_start_was_never_delivered(self):
        port = unused_tcp_port()
        notifier = _RecordingNotifier("127.0.0.1", port)
        with self.assertLogs("marvinpro_rollout", level="WARNING"):
            self.assertFalse(notifier.episode_start(task="stack cones", run_dir=None))
        collector = FakeCollector(port=port)
        try:
            notifier.episode_end("completed", "clean_completion")
            time.sleep(0.1)
            self.assertEqual(collector.snapshot(), [])
        finally:
            collector.close()

    def test_record_notify_configuration_defaults_to_disabled(self):
        args = parse_args([])
        self.assertIsNone(args.record_notify_host)
        self.assertEqual(args.record_notify_port, 7931)
        self.assertFalse(args.record_notify_without_execute)
        args = parse_args(["--record-notify-host", "127.0.0.1", "--record-notify-port", "7932"])
        self.assertEqual(args.record_notify_host, "127.0.0.1")
        self.assertEqual(args.record_notify_port, 7932)

    def test_record_notify_without_execute_requires_host(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--record-notify-without-execute"])
        args = parse_args(["--record-notify-host", "127.0.0.1", "--record-notify-without-execute"])
        self.assertTrue(args.record_notify_without_execute)


class RecordingNotifierRunTest(unittest.TestCase):
    """Run the dry-run/execution shell with a fake bridge, policy, and collector."""

    @staticmethod
    def _observation(input_mode: int = 0):
        return SimpleNamespace(
            seq=1,
            captured_monotonic=time.monotonic(),
            joints=(0.0,) * 14,
            image=b"jpeg",
            gripper_raw_left=0.0,
            gripper_raw_right=0.0,
            age_state_s=0.001,
            age_gripper_left_s=0.001,
            age_gripper_right_s=0.001,
            input_mode=input_mode,
            robot_state=(3, 3),
            arm_state=(3, 3),
            motion_gate_open=True,
            gate_reason="ready",
            last_command_id=None,
            last_command_status="",
            extra={"gripper_state_source": "measured_feedback"},
        )

    @staticmethod
    def _fake_bridge(observation):
        return SimpleNamespace(
            hello=BridgeHello(motion_allowed=True, publish_hz=100.0),
            wait_for_observation=lambda **kwargs: observation,
            latest=lambda max_local_age_s=None: observation,
            send_action=lambda command: None,
            close=lambda reason: None,
        )

    def _run_rollout(self, args, infer):
        observation = self._observation()
        bridge = self._fake_bridge(observation)
        policy = SimpleNamespace(get_server_metadata=lambda: {}, close=lambda: None)
        stdout = io.StringIO()
        with (
            patch("marvinpro_deploy.rollout_client.RobotConnection", return_value=bridge),
            patch(
                "marvinpro_deploy.rollout_client.websocket_client_policy.WebsocketClientPolicy",
                return_value=policy,
            ),
            patch("marvinpro_deploy.rollout_client.infer_actions", side_effect=infer),
            redirect_stdout(stdout),
        ):
            code = run(args)
        return code, stdout.getvalue()

    @staticmethod
    def _infer_ok(policy, observation, prompt):
        return np.zeros((5, 16)), {"wall_ms": 1.0, "policy_timing": {}}

    def test_dry_run_sends_start_then_completed(self):
        collector = FakeCollector()
        try:
            with tempfile.TemporaryDirectory() as temp_dir:
                log_file = str(Path(temp_dir) / "client.log")
                args = parse_args(
                    [
                        "--episode-seconds",
                        "0.4",
                        "--prompt",
                        "collect the cones",
                        "--log-file",
                        log_file,
                        "--record-notify-host",
                        "127.0.0.1",
                        "--record-notify-port",
                        str(collector.port),
                        "--record-notify-without-execute",
                    ]
                )
                code, _ = self._run_rollout(args, self._infer_ok)
            self.assertEqual(code, 0)
            events = collector.wait_events(2)
            time.sleep(0.1)
            self.assertEqual(events, collector.snapshot())
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            start, end = events
            self.assertEqual(start["task"], "collect the cones")
            self.assertEqual(start["run_dir"], str(Path(log_file).expanduser().resolve().parent))
            self.assertIsInstance(start["ts"], float)
            self.assertEqual(end["status"], "completed")
            self.assertEqual(end["reason"], "clean_completion")
            self.assertGreaterEqual(end["ts"], start["ts"])
        finally:
            collector.close()

    def test_dry_run_ctrl_c_sends_operator_stopped(self):
        def infer(policy, observation, prompt):
            raise KeyboardInterrupt

        collector = FakeCollector()
        try:
            args = parse_args(
                [
                    "--episode-seconds",
                    "30",
                    "--warmup-inferences",
                    "0",
                    "--record-notify-host",
                    "127.0.0.1",
                    "--record-notify-port",
                    str(collector.port),
                    "--record-notify-without-execute",
                ]
            )
            with self.assertLogs("marvinpro_rollout", level="WARNING"):
                code, _ = self._run_rollout(args, infer)
            self.assertEqual(code, 130)
            events = collector.wait_events(2)
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[1]["status"], "operator_stopped")
            self.assertIn("operator interrupted", events[1]["reason"])
        finally:
            collector.close()

    def test_dry_run_abort_sends_aborted_with_error_string(self):
        def infer(policy, observation, prompt):
            raise RolloutError("robot motion gate closed: input_mode changed")

        collector = FakeCollector()
        try:
            args = parse_args(
                [
                    "--episode-seconds",
                    "30",
                    "--warmup-inferences",
                    "0",
                    "--record-notify-host",
                    "127.0.0.1",
                    "--record-notify-port",
                    str(collector.port),
                    "--record-notify-without-execute",
                ]
            )
            with self.assertLogs("marvinpro_rollout", level="WARNING"):
                code, _ = self._run_rollout(args, infer)
            self.assertEqual(code, 1)
            events = collector.wait_events(2)
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[1]["status"], "aborted")
            self.assertEqual(events[1]["reason"], "robot motion gate closed: input_mode changed")
        finally:
            collector.close()

    def test_disabled_notifier_sends_nothing_and_rollout_proceeds(self):
        collector = FakeCollector()
        try:
            args = parse_args(["--episode-seconds", "0.3"])
            code, _ = self._run_rollout(args, self._infer_ok)
            self.assertEqual(code, 0)
            time.sleep(0.1)
            self.assertEqual(collector.snapshot(), [])
        finally:
            collector.close()

    def test_dry_run_does_not_notify_by_default(self):
        collector = FakeCollector()
        try:
            args = parse_args(
                [
                    "--episode-seconds",
                    "0.3",
                    "--record-notify-host",
                    "127.0.0.1",
                    "--record-notify-port",
                    str(collector.port),
                ]
            )
            with self.assertNoLogs("marvinpro_rollout", level="WARNING"):
                code, stdout = self._run_rollout(args, self._infer_ok)
            self.assertEqual(code, 0)
            time.sleep(0.1)
            self.assertEqual(collector.snapshot(), [])
            self.assertNotIn("THIS EPISODE WILL NOT BE RECORDED", stdout)
        finally:
            collector.close()

    def test_absent_collector_only_warns_and_rollout_proceeds(self):
        args = parse_args(
            [
                "--episode-seconds",
                "0.3",
                "--record-notify-host",
                "127.0.0.1",
                "--record-notify-port",
                str(unused_tcp_port()),
                "--record-notify-without-execute",
            ]
        )
        with self.assertLogs("marvinpro_rollout", level="WARNING") as captured:
            code, stdout = self._run_rollout(args, self._infer_ok)
        self.assertEqual(code, 0)
        self.assertTrue(any("record_notify_failed" in line for line in captured.output))
        self.assertFalse(any("ERROR" in line for line in captured.output))
        self.assertIn("THIS EPISODE WILL NOT BE RECORDED", stdout)

    def test_execute_sends_start_after_typed_e_confirmation(self):
        collector = FakeCollector()
        prompts = []

        def fake_input(prompt=""):
            prompts.append(prompt)
            # The pre-prompt probe must not produce collector events; the start
            # notification is only sent after the operator types E.
            self.assertEqual(collector.snapshot(), [])
            return "E"

        try:
            args = parse_args(
                [
                    "--execute",
                    "--episode-seconds",
                    "0.4",
                    "--prompt",
                    "collect the cones",
                    "--record-notify-host",
                    "127.0.0.1",
                    "--record-notify-port",
                    str(collector.port),
                ]
            )
            observation = self._observation(input_mode=0)
            bridge = self._fake_bridge(observation)
            policy = SimpleNamespace(get_server_metadata=lambda: {}, close=lambda: None)
            with (
                patch("marvinpro_deploy.rollout_client.RobotConnection", return_value=bridge),
                patch(
                    "marvinpro_deploy.rollout_client.websocket_client_policy.WebsocketClientPolicy",
                    return_value=policy,
                ),
                patch("marvinpro_deploy.rollout_client.infer_actions", side_effect=self._infer_ok),
                patch("builtins.input", side_effect=fake_input),
                redirect_stdout(io.StringIO()),
            ):
                code = run(args)
            self.assertEqual(code, 0)
            self.assertTrue(prompts)
            events = collector.wait_events(2)
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[0]["task"], "collect the cones")
            self.assertIsNone(events[0]["run_dir"])
            self.assertEqual(events[1]["status"], "completed")
        finally:
            collector.close()

    def test_execute_warns_before_confirmation_prompt_when_collector_absent(self):
        args = parse_args(
            [
                "--execute",
                "--episode-seconds",
                "0.3",
                "--record-notify-host",
                "127.0.0.1",
                "--record-notify-port",
                str(unused_tcp_port()),
            ]
        )
        observation = self._observation(input_mode=0)
        bridge = self._fake_bridge(observation)
        policy = SimpleNamespace(get_server_metadata=lambda: {}, close=lambda: None)
        stdout = io.StringIO()
        with (
            patch("marvinpro_deploy.rollout_client.RobotConnection", return_value=bridge),
            patch(
                "marvinpro_deploy.rollout_client.websocket_client_policy.WebsocketClientPolicy",
                return_value=policy,
            ),
            patch("marvinpro_deploy.rollout_client.infer_actions", side_effect=self._infer_ok),
            patch("builtins.input", return_value="E"),
            self.assertLogs("marvinpro_rollout", level="WARNING"),
            redirect_stdout(stdout),
        ):
            code = run(args)
        self.assertEqual(code, 0)
        output = stdout.getvalue()
        self.assertIn("THIS EPISODE WILL NOT BE RECORDED", output)
        self.assertLess(
            output.index("THIS EPISODE WILL NOT BE RECORDED"),
            output.index("REAL ROBOT EXECUTION REQUESTED"),
        )


class RecordingNotifierTrajectoryTest(unittest.TestCase):
    """episode_end send points inside _run_trajectory_schedule."""

    class _FakeHeartbeat:
        error = None

        def start(self):
            return None

        def join(self):
            return None

        def update_version(self, version):
            return None

    @staticmethod
    def _timing():
        return {
            "wall_ms": 100.0,
            "observation_preparation_ms": 0.0,
            "client_timing": {},
            "policy_timing": {},
            "server_timing": {},
        }

    @staticmethod
    def _sync_args():
        return parse_args(
            [
                "--execute",
                "--rollout-schedule",
                "synchronized",
                "--playback-mode",
                "interpolated",
                "--control-hz",
                "100",
                "--model-hz",
                "15",
                "--playback-time-scale",
                "3",
                "--execute-steps",
                str(RTC_HORIZON),
            ]
        )

    @staticmethod
    def _rtc_args(*extra):
        return parse_args(
            [
                "--robot-host",
                "127.0.0.1",
                "--policy-host",
                "127.0.0.1",
                "--execute",
                "--rollout-schedule",
                "rtc",
                "--playback-mode",
                "interpolated",
                "--control-hz",
                "100",
                "--model-hz",
                "15",
                "--playback-time-scale",
                "3",
                "--execute-steps",
                str(RTC_HORIZON),
                *extra,
            ]
        )

    @staticmethod
    def _rtc_connection(checkpoint):
        return SimpleNamespace(
            latest_state=lambda max_local_age_s=None: SimpleNamespace(timeline_version=1),
            latest=lambda: SimpleNamespace(input_mode=0),
            wait_for_event=lambda **kwargs: checkpoint,
            poll_event=lambda **kwargs: None,
            send=lambda command: None,
        )

    def _run_fallback(self, notifier, sync_result):
        actions = np.zeros((RTC_HORIZON, 16))
        loaded = TrajectoryEvent(2, "trajectory_loaded", 0.0, "s", "p", 1, 0.0)
        with (
            patch("marvinpro_deploy.rollout_client._TrajectoryHeartbeat", return_value=self._FakeHeartbeat()),
            patch("marvinpro_deploy.rollout_client.infer_actions", return_value=(actions, self._timing())),
            patch("marvinpro_deploy.rollout_client._run_bridge_synchronized", return_value=sync_result),
            patch("marvinpro_deploy.rollout_client._hold_bridge_position", return_value=loaded),
            patch("marvinpro_deploy.rollout_client._wait_for_none_after_trajectory"),
            redirect_stdout(io.StringIO()),
        ):
            return _run_trajectory_schedule(
                self._sync_args(),
                SimpleNamespace(),
                SimpleNamespace(),
                SimpleNamespace(seq=1),
                [],
                notifier=notifier,
            )

    def test_fallback_path_sends_completed_once(self):
        collector = FakeCollector()
        try:
            notifier = _RecordingNotifier("127.0.0.1", collector.port)
            self.assertTrue(notifier.episode_start(task="stack cones", run_dir="/tmp/run"))
            sync_result = TimedSynchronizedResult(
                inference_count=2,
                observation=SimpleNamespace(seq=2),
                clean_chunks=3,
                stuck_replans=0,
                exhausted=False,
            )
            result = self._run_fallback(notifier, sync_result)
            self.assertEqual(result, 3)
            events = collector.wait_events(2)
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[1]["status"], "completed")
            self.assertEqual(events[1]["reason"], "clean_completion")
            notifier.episode_end("operator_stopped", "operator interrupted rollout")
            time.sleep(0.1)
            self.assertEqual(len(collector.snapshot()), 2)
        finally:
            collector.close()

    def test_fallback_path_sends_aborted_on_stuck_exhausted(self):
        collector = FakeCollector()
        try:
            notifier = _RecordingNotifier("127.0.0.1", collector.port)
            self.assertTrue(notifier.episode_start(task="stack cones", run_dir=None))
            sync_result = TimedSynchronizedResult(
                inference_count=0,
                observation=SimpleNamespace(seq=2),
                clean_chunks=1,
                stuck_replans=2,
                exhausted=True,
            )
            self._run_fallback(notifier, sync_result)
            events = collector.wait_events(2)
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[1]["status"], "aborted")
            self.assertEqual(events[1]["reason"], "stuck_exhausted")
        finally:
            collector.close()

    def test_trajectory_path_sends_nothing_when_start_was_not_delivered(self):
        collector = FakeCollector()
        try:
            notifier = _RecordingNotifier("127.0.0.1", collector.port)
            sync_result = TimedSynchronizedResult(
                inference_count=0,
                observation=SimpleNamespace(seq=2),
                clean_chunks=1,
                stuck_replans=0,
                exhausted=False,
            )
            self._run_fallback(notifier, sync_result)
            time.sleep(0.1)
            self.assertEqual(collector.snapshot(), [])
        finally:
            collector.close()

    def test_rtc_path_sends_completed_on_clean_final_status(self):
        collector = FakeCollector()
        try:
            notifier = _RecordingNotifier("127.0.0.1", collector.port)
            self.assertTrue(notifier.episode_start(task="stack cones", run_dir=None))
            actions = np.zeros((RTC_HORIZON, 16))
            loaded = TrajectoryEvent(2, "trajectory_loaded", 0.0, "s", "p", 1, 0.0)

            def expired_load(*args, **kwargs):
                # Push past the tiny episode deadline so the RTC loop is skipped
                # and rtc_final_status stays clean_completion.
                time.sleep(0.06)
                return loaded

            with (
                patch("marvinpro_deploy.rollout_client._TrajectoryHeartbeat", return_value=self._FakeHeartbeat()),
                patch("marvinpro_deploy.rollout_client.infer_actions", return_value=(actions, self._timing())),
                patch("marvinpro_deploy.rollout_client._load_bridge_trajectory", side_effect=expired_load),
                patch("marvinpro_deploy.rollout_client._hold_bridge_position", return_value=loaded),
                redirect_stdout(io.StringIO()),
            ):
                result = _run_trajectory_schedule(
                    self._rtc_args("--episode-seconds", "0.05"),
                    self._rtc_connection(None),
                    SimpleNamespace(close=lambda: None),
                    SimpleNamespace(seq=1),
                    [],
                    notifier=notifier,
                )
            self.assertEqual(result, 1)
            events = collector.wait_events(2)
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[1]["status"], "completed")
            self.assertEqual(events[1]["reason"], "clean_completion")
        finally:
            collector.close()

    def test_rtc_path_sends_aborted_on_fatal_safety_hold(self):
        collector = FakeCollector()
        try:
            notifier = _RecordingNotifier("127.0.0.1", collector.port)
            self.assertTrue(notifier.episode_start(task="stack cones", run_dir=None))
            actions = np.zeros((RTC_HORIZON, 16))
            loaded = TrajectoryEvent(2, "trajectory_loaded", 0.0, "s", "initial", 1, 0.0)
            checkpoint = TrajectoryEvent(1, "checkpoint_ready", 1.0, "s", "initial", 1, 10.0, checkpoint_id=1)
            policy = SimpleNamespace(
                infer=lambda request: (_ for _ in ()).throw(ConnectionError("transport timeout")),
                close=lambda: None,
            )
            with (
                patch("marvinpro_deploy.rollout_client._TrajectoryHeartbeat", return_value=self._FakeHeartbeat()),
                patch("marvinpro_deploy.rollout_client.infer_actions", return_value=(actions, self._timing())),
                patch("marvinpro_deploy.rollout_client.build_policy_observation", return_value={}),
                patch("marvinpro_deploy.rollout_client.build_rtc_request", return_value={}),
                patch("marvinpro_deploy.rollout_client._load_bridge_trajectory", return_value=loaded),
                patch(
                    "marvinpro_deploy.rollout_client._wait_checkpoint_observation",
                    return_value=SimpleNamespace(seq=10),
                ),
                patch(
                    "marvinpro_deploy.rollout_client._latch_measured_bridge_position",
                    side_effect=SafetyError("measured hold failed"),
                ),
                patch("marvinpro_deploy.rollout_client._hold_bridge_position", return_value=loaded),
                redirect_stdout(io.StringIO()),
            ):
                result = _run_trajectory_schedule(
                    self._rtc_args("--episode-seconds", "300"),
                    self._rtc_connection(checkpoint),
                    policy,
                    SimpleNamespace(seq=1),
                    [],
                    notifier=notifier,
                )
            self.assertEqual(result, 1)
            events = collector.wait_events(2)
            self.assertEqual([event["cmd"] for event in events], ["episode_start", "episode_end"])
            self.assertEqual(events[1]["status"], "aborted")
            self.assertEqual(events[1]["reason"], "fatal_safety_hold")
        finally:
            collector.close()


if __name__ == "__main__":
    unittest.main()
