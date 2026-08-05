import socket
import struct
import unittest

from marvinpro_deploy.protocol import ActionCommand, ProtocolError, recv_message, send_message


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


if __name__ == "__main__":
    unittest.main()
