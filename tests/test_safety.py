import unittest

from marvinpro_deploy.safety import SafetyError, filter_action, validate_action


def action16(arm_value=0.0, gripper=0.5):
    return [arm_value] * 7 + [gripper] + [arm_value] * 7 + [gripper]


class SafetyTest(unittest.TestCase):
    def test_accepts_finite_bounded_action(self):
        result = validate_action(action16(0.05), [0.0] * 14, max_joint_step_rad=0.08)
        self.assertEqual(len(result), 16)

    def test_rejects_large_step(self):
        with self.assertRaisesRegex(SafetyError, "exceeds"):
            validate_action(action16(0.09), [0.0] * 14, max_joint_step_rad=0.08)

    def test_rejects_nonfinite_and_bad_gripper(self):
        bad = action16()
        bad[2] = float("nan")
        with self.assertRaisesRegex(SafetyError, "NaN"):
            validate_action(bad, [0.0] * 14, max_joint_step_rad=0.08)
        bad = action16()
        bad[7] = 1.1
        with self.assertRaisesRegex(SafetyError, "gripper"):
            validate_action(bad, [0.0] * 14, max_joint_step_rad=0.08)

    def test_clamps_only_numerical_gripper_boundary_error(self):
        action = action16()
        action[7] = -3.469446951953614e-18
        action[15] = 1.0 + 2.220446049250313e-16

        result = validate_action(action, [0.0] * 14, max_joint_step_rad=0.08)

        self.assertEqual(result[7], 0.0)
        self.assertEqual(result[15], 1.0)

        action[7] = -1e-6
        with self.assertRaisesRegex(SafetyError, "gripper"):
            validate_action(action, [0.0] * 14, max_joint_step_rad=0.08)

    def test_filter_clamps_step_and_gripper(self):
        filtered = filter_action(action16(1.0, 2.0), [0.0] * 14, max_joint_step_rad=0.08)
        self.assertEqual(filtered.action[0], 0.08)
        self.assertEqual(filtered.action[7], 1.0)
        self.assertEqual(filtered.action[15], 1.0)
        self.assertEqual(len(filtered.clipped_indices), 16)

    def test_hardware_joint_limit_is_enforced(self):
        action = action16()
        action[5] = 1.04
        current = [0.0] * 14
        current[5] = 1.0
        with self.assertRaisesRegex(SafetyError, "outside"):
            validate_action(action, current, max_joint_step_rad=0.08, joint_limit_margin_rad=0.02)


if __name__ == "__main__":
    unittest.main()
