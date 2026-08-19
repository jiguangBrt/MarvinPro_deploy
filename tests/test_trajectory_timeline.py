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

    def test_quintic_replacement_is_c2_at_both_ends(self):
        timeline = TrajectoryTimeline(tuple(knot(index * 0.01) for index in range(20)), 5.0, 10)
        actions = tuple(knot(0.2 + index * 0.02) for index in range(20))
        replacement, phase = timeline.replacement(
            actions,
            actual_delay_steps=2,
            anchor=knot(0.1),
            blend_knots=3,
            start_velocity=knot(0.01),
            start_acceleration=knot(0.0),
        )

        self.assertEqual(phase, 1.0)
        self.assertEqual(replacement.value(phase), knot(0.1))
        start = replacement.phase_kinematics(phase)
        end = replacement.phase_kinematics(phase + 3.0)
        for actual, expected in zip(start[1], knot(0.01)):
            self.assertAlmostEqual(actual, expected)
        for actual in start[2]:
            self.assertAlmostEqual(actual, 0.0)
        for actual, expected in zip(end[0], actions[4]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(end[1], knot(0.02)):
            self.assertAlmostEqual(actual, expected)
        for actual in end[2]:
            self.assertAlmostEqual(actual, 0.0)

    def test_quintic_replacement_must_finish_before_checkpoint(self):
        timeline = TrajectoryTimeline(tuple(knot(i) for i in range(10)), 5.0, 6)

        with self.assertRaisesRegex(ValueError, "does not fit"):
            timeline.replacement(
                tuple(knot(100 + i) for i in range(10)),
                actual_delay_steps=4,
                anchor=knot(5),
                blend_knots=3,
                start_velocity=knot(1),
                start_acceleration=knot(0),
            )

    def test_c2_handoff_starts_stationary_at_measured_anchor(self):
        timeline = TrajectoryTimeline(tuple(knot(0.2 + index * 0.01) for index in range(20)), 5.0, 20)

        handoff = timeline.with_c2_handoff(knot(0.1), blend_knots=3)

        start = handoff.phase_kinematics(0.0)
        end = handoff.phase_kinematics(3.0)
        self.assertEqual(start[0], knot(0.1))
        self.assertEqual(start[1], knot(0.0))
        self.assertEqual(start[2], knot(0.0))
        for actual, expected in zip(end[0], timeline.knots[3]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(end[1], knot(0.01)):
            self.assertAlmostEqual(actual, expected)

    def test_c2_handoff_rejects_blend_past_checkpoint(self):
        timeline = TrajectoryTimeline(tuple(knot(index) for index in range(20)), 5.0, 3)

        with self.assertRaisesRegex(ValueError, "does not fit"):
            timeline.with_c2_handoff(knot(0), blend_knots=3)


if __name__ == "__main__":
    unittest.main()
