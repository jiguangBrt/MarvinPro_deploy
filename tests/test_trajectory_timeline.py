import unittest

from marvinpro_deploy.trajectory_timeline import TrajectoryTimeline


def knot(value):
    return (float(value),) * 16


class TrajectoryTimelineTest(unittest.TestCase):
    def test_phase_and_checkpoint_indexing(self):
        timeline = TrajectoryTimeline(tuple(knot(i) for i in range(10)), 7.5, 4)
        self.assertEqual(timeline.checkpoint_phase, 3.0)
        self.assertEqual(timeline.value(0.0), knot(0))
        self.assertEqual(timeline.value(3.0), knot(3))
        self.assertEqual(timeline.value(3.25), knot(3.25))
        self.assertEqual(timeline.remaining_after_checkpoint(), tuple(knot(i) for i in range(4, 10)))

    def test_replacement_preserves_reached_anchor(self):
        timeline = TrajectoryTimeline(tuple(knot(i) for i in range(10)), 7.5, 4)
        replacement, phase = timeline.replacement(
            tuple(knot(100 + i) for i in range(10)),
            actual_delay_steps=2,
            anchor=knot(5),
        )
        self.assertEqual(phase, 1.0)
        self.assertEqual(replacement.knots[1], knot(5))
        self.assertEqual(replacement.knots[2], knot(102))

    def test_all_supported_actual_delays_anchor_the_previous_rtc_knot(self):
        timeline = TrajectoryTimeline(tuple(knot(i) for i in range(10)), 7.5, 4)
        for actual_delay in range(1, 5):
            replacement, phase = timeline.replacement(
                tuple(knot(100 + i) for i in range(10)),
                actual_delay_steps=actual_delay,
                anchor=knot(50 + actual_delay),
            )
            self.assertEqual(phase, float(actual_delay - 1))
            self.assertEqual(replacement.knots[actual_delay - 1], knot(50 + actual_delay))
            if actual_delay < 4:
                self.assertEqual(replacement.knots[actual_delay], knot(100 + actual_delay))

    def test_replacement_rejects_unsupported_actual_delays(self):
        timeline = TrajectoryTimeline(tuple(knot(i) for i in range(10)), 7.5, 4)
        for actual_delay in (0, 5):
            with self.assertRaises(ValueError):
                timeline.replacement(
                    tuple(knot(100 + i) for i in range(10)),
                    actual_delay_steps=actual_delay,
                    anchor=knot(0),
                )


if __name__ == "__main__":
    unittest.main()
