import socket
import struct
from types import SimpleNamespace
import unittest

from marvinpro_deploy.protocol import (
    ActionCommand,
    LoadTrajectoryCommand,
    ProtocolError,
    RobotStateUpdate,
    TrajectoryEvent,
    recv_message,
    require_current_version,
    send_message,
)


class ProtocolTest(unittest.TestCase):
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
            ),
            LoadTrajectoryCommand(
                command_id=3,
                observation_seq=8,
                session_id="session",
                plan_id="plan",
                expected_timeline_version=1,
                knots=((0.0,) * 16,) * 10,
                knot_hz=7.5,
                checkpoint_horizon=4,
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
