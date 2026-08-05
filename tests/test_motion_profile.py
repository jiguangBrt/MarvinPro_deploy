import unittest

from marvinpro_deploy.motion_profile import FrozenLinearPlan, MinimumJerkSweep


class MinimumJerkSweepTest(unittest.TestCase):
    def test_key_positions_and_clamping(self):
        profile = MinimumJerkSweep(amplitude_rad=0.04, segment_seconds=2.0)
        self.assertAlmostEqual(profile.offset(-1.0), 0.0)
        self.assertAlmostEqual(profile.offset(0.0), 0.0)
        self.assertAlmostEqual(profile.offset(2.0), 0.04)
        self.assertAlmostEqual(profile.offset(6.0), -0.04)
        self.assertAlmostEqual(profile.offset(8.0), 0.0)
        self.assertAlmostEqual(profile.offset(9.0), 0.0)

    def test_profile_stays_inside_amplitude(self):
        profile = MinimumJerkSweep(amplitude_rad=0.04, segment_seconds=2.0)
        offsets = [profile.offset(index * profile.duration / 1000.0) for index in range(1001)]
        self.assertLessEqual(max(offsets), profile.amplitude_rad)
        self.assertGreaterEqual(min(offsets), -profile.amplitude_rad)

    def test_default_limits_are_below_home_limits(self):
        profile = MinimumJerkSweep(amplitude_rad=0.04, segment_seconds=2.0)
        self.assertAlmostEqual(profile.duration, 8.0)
        self.assertAlmostEqual(profile.max_velocity, 0.0375)
        self.assertLess(profile.max_acceleration, 0.06)

    def test_requires_positive_finite_parameters(self):
        for amplitude, seconds in ((0.0, 2.0), (float("nan"), 2.0), (0.04, 0.0)):
            with self.subTest(amplitude=amplitude, seconds=seconds):
                with self.assertRaises(ValueError):
                    MinimumJerkSweep(amplitude, seconds)


class FrozenLinearPlanTest(unittest.TestCase):
    def test_interpolates_and_clamps_time(self):
        plan = FrozenLinearPlan(((0.0, 10.0), (2.0, 14.0), (4.0, 18.0)), knot_hz=2.0)
        self.assertAlmostEqual(plan.duration, 1.0)
        self.assertEqual(plan.value(-1.0), (0.0, 10.0))
        self.assertEqual(plan.value(0.25), (1.0, 12.0))
        self.assertEqual(plan.value(0.75), (3.0, 16.0))
        self.assertEqual(plan.value(2.0), (4.0, 18.0))

    def test_rejects_invalid_knots_and_rate(self):
        with self.assertRaises(ValueError):
            FrozenLinearPlan(((0.0,),), knot_hz=1.0)
        with self.assertRaises(ValueError):
            FrozenLinearPlan(((0.0,), (1.0, 2.0)), knot_hz=1.0)
        with self.assertRaises(ValueError):
            FrozenLinearPlan(((0.0,), (1.0,)), knot_hz=0.0)


if __name__ == "__main__":
    unittest.main()
