import unittest

from marvinpro_deploy.tracking import TrackingGovernor
from marvinpro_deploy.trajectory_timeline import TrajectoryTimeline


class TrackingGovernorTest(unittest.TestCase):
    def setUp(self):
        self.governor = TrackingGovernor(
            run_error_rad=0.02,
            resume_error_rad=0.12,
            stop_error_rad=0.16,
        )

    def test_scales_phase_rate_and_latches_hard_freeze(self):
        self.assertEqual(self.governor.update(0.02).phase_rate, 1.0)
        decision = self.governor.update(0.09)
        self.assertAlmostEqual(decision.phase_rate, 0.5)

        frozen = self.governor.update(0.16)
        self.assertTrue(frozen.hard_frozen)
        self.assertEqual(frozen.phase_rate, 0.0)
        self.assertTrue(self.governor.update(0.120001).hard_frozen)

        resumed = self.governor.update(0.12)
        self.assertFalse(resumed.hard_frozen)
        self.assertAlmostEqual(resumed.phase_rate, 2.0 / 7.0)

    def test_stale_state_timer_overrun_and_clipping_freeze(self):
        self.assertTrue(self.governor.update(0.0, state_stale=True).hard_frozen)
        self.governor.reset()
        self.assertTrue(self.governor.update(0.0, timer_overrun=True).hard_frozen)
        self.governor.reset()
        self.assertTrue(self.governor.update(0.0, arm_clipped=True).hard_frozen)

    def test_slow_first_order_robot_throttles_plan_phase(self):
        knots = tuple((0.03 * index,) * 16 for index in range(10))
        timeline = TrajectoryTimeline(knots, 5.0, 4)
        phase = 0.0
        measured = 0.0
        max_error = 0.0

        for _ in range(2000):
            reference = timeline.value(phase)[0]
            error = abs(reference - measured)
            max_error = max(max_error, error)
            decision = self.governor.update(error)
            phase = min(timeline.checkpoint_phase, phase + timeline.knot_hz * decision.phase_rate * 0.01)
            reference = timeline.value(phase)[0]
            measured += max(-0.0006, min(0.0006, reference - measured))
            if phase == timeline.checkpoint_phase and abs(reference - measured) <= 0.01:
                break

        self.assertEqual(phase, timeline.checkpoint_phase)
        self.assertLessEqual(max_error, 0.1625)
        self.assertGreater(measured, 0.08)


if __name__ == "__main__":
    unittest.main()
