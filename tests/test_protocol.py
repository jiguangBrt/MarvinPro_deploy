import socket
import struct
from types import SimpleNamespace
import unittest

from marvinpro_deploy.protocol import (
    ActionCommand,
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
            TrajectoryEvent(2, "checkpoint_ready", 4.0, "session", "plan", 3, 3.0),
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
