import unittest

from marvinpro_deploy.joint_mapping import (
    JointMap,
    JointMapError,
    build_state16,
    clamp_gripper_position,
    normalize_gripper,
)


class JointMappingTest(unittest.TestCase):
    def test_maps_reordered_joint_state(self):
        canonical = [f"Joint{i}_L" for i in range(1, 8)] + [f"Joint{i}_R" for i in range(1, 8)]
        names = list(reversed(canonical))
        positions = list(range(14))
        result = JointMap.from_names(names).canonical_positions(positions)
        self.assertEqual(result, tuple(reversed(range(14))))

    def test_rejects_missing_joint(self):
        with self.assertRaises(JointMapError):
            JointMap.from_names([f"Joint{i}_L" for i in range(1, 8)])

    def test_normalizes_measured_gripper_feedback_to_policy_domain(self):
        self.assertEqual(normalize_gripper(-0.1), 0.0)
        self.assertAlmostEqual(normalize_gripper(0.625), 0.5)
        self.assertEqual(normalize_gripper(1.4), 1.0)

    def test_builds_training_state_order(self):
        state = build_state16(tuple(float(i) for i in range(14)), 0.5, 1.0)
        self.assertEqual(state[:8], (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.5))
        self.assertEqual(state[8:], (7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 1.0))

    def test_clamps_normalized_gripper_position_to_policy_domain(self):
        self.assertEqual(clamp_gripper_position(-0.1), 0.0)
        self.assertEqual(clamp_gripper_position(0.5), 0.5)
        self.assertEqual(clamp_gripper_position(1.4), 1.0)
        state = build_state16((0.0,) * 14, 1.0, 0.25)
        self.assertEqual(state[7], 1.0)
        self.assertEqual(state[15], 0.25)


if __name__ == "__main__":
    unittest.main()
