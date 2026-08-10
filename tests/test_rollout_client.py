from contextlib import redirect_stderr
import io
import socket
from types import SimpleNamespace
import threading
import time
import unittest

import numpy as np

from marvinpro_deploy.protocol import BridgeHello, TrajectoryEvent, send_message
from marvinpro_deploy.rollout_client import (
    _TrajectoryHeartbeat,
    ActionPlan,
    ActionPublisher,
    RobotConnection,
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


class RolloutArgumentTest(unittest.TestCase):
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
                "10",
                "--chunk-prefetch-seconds",
                "0.30",
            ]
        )

        self.assertEqual(args.playback_mode, "interpolated")
        self.assertEqual(args.control_hz, 100.0)
        self.assertEqual(args.model_hz, 15.0)
        self.assertEqual(args.playback_time_scale, 2.0)

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
                "10",
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

    def test_rtc_schedule_requires_fixed_first_version_configuration(self):
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
                "2",
                "--execute-steps",
                "10",
            ]
        )
        self.assertEqual(args.rollout_schedule, "rtc")
        self.assertFalse(args.rtc_shadow)

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
