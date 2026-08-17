import socket
import struct
from types import SimpleNamespace
import unittest

from marvinpro_deploy.protocol import (
    ActionCommand,
    BridgeHello,
    LoadTrajectoryCommand,
    ProtocolError,
    RobotStateUpdate,
    ResumeTrajectoryCommand,
    TrajectoryEvent,
    recv_message,
    require_current_version,
    send_message,
)
from marvinpro_deploy.rtc import RTC_EXECUTION_HORIZON, RTC_HORIZON


class ProtocolTest(unittest.TestCase):
    def test_bridge_hello_uses_current_safety_envelope(self):
        self.assertEqual(BridgeHello().max_joint_step_rad, 0.16)

    def test_resume_command_defaults_to_discarding_late_results(self):
        command = ResumeTrajectoryCommand(1, "session", "plan", 2, 3, "request", 4)
        self.assertEqual(command.late_result_policy, "discard")

    def test_socket_round_trip(self):
        left, right = socket.socketpair()
        try:
            expected = ActionCommand(3, 7, tuple([0.0] * 16), False)
            send_message(left, expected)
            self.assertEqual(recv_message(right), expected)
        finally:
            left.close()
            right.close()

    def test_rejects_oversized_frame_header(self):
        left, right = socket.socketpair()
        try:
            left.sendall(struct.pack(">I", 0xFFFFFFFF))
            with self.assertRaises(ProtocolError):
                recv_message(right)
        finally:
            left.close()
            right.close()

    def test_state_and_event_round_trip(self):
        messages = (
            RobotStateUpdate(1, 3.5, (0.0,) * 14, 0.0, 0.0, True, "ready"),
            TrajectoryEvent(
                2,
                "checkpoint_ready",
                4.0,
                "session",
                "plan",
                3,
                3.0,
                boundary_old_velocity=(0.01,) * 14,
                boundary_new_velocity=(0.02,) * 14,
                boundary_velocity_jump_rad=0.01,
                boundary_acceleration_jump_rad=0.03,
                continuous_checkpoint=True,
                blend_duration_knots=3,
                blend_max_velocity_rad_s=0.2,
                blend_max_acceleration_rad_s2=0.8,
                blend_max_jerk_rad_s3=4.0,
            ),
            LoadTrajectoryCommand(
                command_id=3,
                observation_seq=8,
                session_id="session",
                plan_id="plan",
                expected_timeline_version=1,
                knots=((0.0,) * 16,) * RTC_HORIZON,
                knot_hz=5.0,
                checkpoint_horizon=RTC_EXECUTION_HORIZON,
                execute=True,
                continuous_checkpoint=True,
            ),
        )
        left, right = socket.socketpair()
        try:
            for expected in messages:
                send_message(left, expected)
                self.assertEqual(recv_message(right), expected)
        finally:
            left.close()
            right.close()

    def test_rejects_protocol_version_mismatch(self):
        with self.assertRaisesRegex(ProtocolError, "version mismatch"):
            require_current_version(SimpleNamespace(version=1))


if __name__ == "__main__":
    unittest.main()
