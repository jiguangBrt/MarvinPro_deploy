import argparse
from contextlib import redirect_stderr
from io import StringIO
import unittest

from marvinpro_deploy.config import TOPIC_GRIPPER_CMD_L, TOPIC_GRIPPER_CMD_R
from marvinpro_deploy.gripper_control import build_parser, command_topics, positive_duration


class GripperControlTest(unittest.TestCase):
    def test_command_and_default_side(self):
        args = build_parser().parse_args(["0"])
        self.assertEqual(args.command, "0")
        self.assertEqual(args.side, "both")

    def test_side_selects_expected_topics(self):
        self.assertEqual(command_topics("left"), (("left", TOPIC_GRIPPER_CMD_L),))
        self.assertEqual(command_topics("right"), (("right", TOPIC_GRIPPER_CMD_R),))
        self.assertEqual(
            command_topics("both"),
            (("left", TOPIC_GRIPPER_CMD_L), ("right", TOPIC_GRIPPER_CMD_R)),
        )

    def test_duration_is_bounded(self):
        self.assertEqual(positive_duration("1.5"), 1.5)
        for value in ("0", "10.1"):
            with self.assertRaises(argparse.ArgumentTypeError):
                positive_duration(value)

    def test_rejects_non_binary_command(self):
        with redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit):
                build_parser().parse_args(["0.5"])


if __name__ == "__main__":
    unittest.main()
