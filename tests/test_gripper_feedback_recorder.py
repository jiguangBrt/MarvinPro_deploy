import unittest

from marvinpro_deploy.gripper_feedback_recorder import FeedbackStats


class FeedbackStatsTest(unittest.TestCase):
    def test_repeated_feedback_is_reported_as_frozen(self):
        stats = FeedbackStats("right")
        stats.record([-0.037, -0.007, 0.1, 31.0, 30.0], 1.0)
        stats.record([-0.037, -0.007, 0.1, 31.0, 30.0], 1.01)
        stats.record([-0.02, 0.01, 0.2, 31.0, 30.0], 1.02)

        self.assertEqual(stats.count, 3)
        self.assertEqual(stats.invalid_count, 0)
        self.assertEqual(stats.changed_count, 1)
        self.assertEqual(stats.distinct_count, 2)
        self.assertEqual(stats.first_values, (-0.037, -0.007, 0.1, 31.0, 30.0))
        self.assertEqual(stats.last_values, (-0.02, 0.01, 0.2, 31.0, 30.0))
        self.assertAlmostEqual(stats.max_step[0], 0.017)
        self.assertAlmostEqual(stats.maximum[2], 0.2)

    def test_short_or_nonfinite_feedback_is_invalid(self):
        stats = FeedbackStats("left")
        self.assertIsNone(stats.record([0.1, 0.2], 1.0))
        self.assertIsNone(stats.record([0.1, float("nan"), 0.2], 1.1))
        self.assertEqual(stats.count, 2)
        self.assertEqual(stats.invalid_count, 2)


if __name__ == "__main__":
    unittest.main()
