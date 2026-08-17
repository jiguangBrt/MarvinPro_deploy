from contextlib import redirect_stderr
from contextlib import redirect_stdout
import io
import csv
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
from marvinpro_deploy.rtc import RTC_HORIZON
from marvinpro_deploy.rollout_client import (
    _TrajectoryHeartbeat,
    _actions_tuple,
    _confirm_execution,
    _confirm_and_refresh_execution_observation,
    _configure_logging,
    _is_observation_lag_rejection,
    _state_log_interval_s,
    ActionPlan,
    ActionPublisher,
    JointTelemetryRecorder,
    RobotConnection,
    RolloutError,
    parse_args,
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


    def test_execution_confirmation_forces_a_new_observation(self):
        ready_observation = SimpleNamespace(seq=10)
        latest_after_confirmation = SimpleNamespace(seq=72)
        fresh_observation = SimpleNamespace(
            seq=73,
            captured_monotonic=100.0,
            joints=(0.0,) * 14,
            image=b"jpeg",
            age_state_s=0.001,
            age_gripper_left_s=None,
            age_gripper_right_s=None,
            gripper_raw_left=0.4,
            gripper_raw_right=0.6,
            gripper_torque_left=0.1,
            gripper_torque_right=0.2,
            extra={"gripper_state_source": "command_proxy"},
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
    def test_records_arm_and_gripper_proxy_without_false_measured_feedback(self):
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
            self.assertEqual(float(rows[0]["gripper_command_proxy_L"]), 0.2)
            self.assertEqual(rows[0]["measured_gripper_position_raw_L"], "")
            self.assertEqual(rows[0]["measured_gripper_position_L"], "")
            self.assertEqual(rows[0]["measured_gripper_torque_L"], "")
            self.assertEqual(float(rows[0]["bridge_command_Joint1_L"]), 2.0)
            self.assertEqual(float(rows[1]["client_reference_Joint1_L"]), 3.0)
            self.assertEqual(float(rows[1]["client_command_Joint1_L"]), 2.9)


class RolloutArgumentTest(unittest.TestCase):
    def test_observation_lag_rejection_is_retryable_but_other_rejections_are_not(self):
        self.assertTrue(_is_observation_lag_rejection(RolloutError("bridge rejected trajectory: action observation lag is 17 frames (limit 8)")))
        self.assertFalse(_is_observation_lag_rejection(RolloutError("bridge rejected trajectory: robot_state=(3, 12)")))

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
                "2",
                "--execute-steps",
                str(RTC_HORIZON),
            ]
        )

        self.assertEqual(args.rollout_schedule, "synchronized")
        self.assertEqual(args.tracking_tolerance_rad, 0.01)
        self.assertEqual(args.tracking_settle_seconds, 0.20)
        self.assertEqual(args.post_track_hold_seconds, 0.20)
        self.assertEqual(args.tracking_timeout, 5.0)

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


if __name__ == "__main__":
    unittest.main()
