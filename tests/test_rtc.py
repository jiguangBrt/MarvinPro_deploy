import unittest

import numpy as np

from marvinpro_deploy.rtc import (
    DelayEstimator,
    RTC_EXECUTION_HORIZON,
    RTC_HORIZON,
    RtcError,
    build_rtc_request,
    parse_rtc_response,
)


class RtcClientTest(unittest.TestCase):
    def test_delay_estimator_uses_stable_distribution_and_isolates_horizon_fault(self):
        estimator = DelayEstimator()
        stable = estimator.record_seconds(0.289, knot_hz=5.0)
        self.assertTrue(stable.accepted)
        self.assertEqual(estimator.predicted_steps(5.0), 2)

        outlier = estimator.record_seconds(1.56, knot_hz=5.0)
        self.assertFalse(outlier.accepted)
        self.assertEqual(outlier.reason, "exceeds_rtc_horizon")
        self.assertEqual(estimator.predicted_steps(5.0), 2)
        self.assertEqual(estimator.stable_samples, 1)
        self.assertEqual(estimator.rejected_samples, 1)

    def test_delay_estimator_excludes_missed_deadline_and_resets_epoch(self):
        estimator = DelayEstimator()
        estimator.record_seconds(0.20, knot_hz=5.0)
        missed = estimator.record_seconds(
            0.30,
            knot_hz=5.0,
            eligible=False,
            rejection_reason="bridge_rtc_invalid",
        )
        self.assertFalse(missed.accepted)
        self.assertEqual(estimator.predicted_steps(5.0), 2)

        self.assertEqual(estimator.reset_epoch(), 1)
        self.assertEqual(estimator.stable_samples, 0)
        self.assertEqual(estimator.rejected_samples, 0)
        with self.assertRaises(RtcError):
            estimator.predicted_steps(5.0)

    def test_horizon_fault_takes_priority_over_deadline_rejection_reason(self):
        estimator = DelayEstimator()

        sample = estimator.record_seconds(
            1.56,
            knot_hz=5.0,
            eligible=False,
            rejection_reason="bridge_rtc_invalid",
        )

        self.assertFalse(sample.accepted)
        self.assertEqual(sample.reason, "exceeds_rtc_horizon")

    def test_delay_estimator_uses_p95_instead_of_maximum_stable_sample(self):
        estimator = DelayEstimator(max_samples=20)
        for _ in range(19):
            estimator.record_seconds(0.20, knot_hz=5.0)
        high_but_feasible = estimator.record_seconds(0.70, knot_hz=5.0)

        self.assertTrue(high_but_feasible.accepted)
        self.assertEqual(estimator.predicted_steps(5.0), 2)

    def test_request_response_ids_and_shape(self):
        request = build_rtc_request(
            request_id="request",
            plan_id="plan",
            timeline_version=3,
            checkpoint_id=4,
            observation={"state": np.zeros(16)},
            old_remaining_actions_absolute=np.zeros((RTC_HORIZON - RTC_EXECUTION_HORIZON, 16)),
            predicted_delay_steps=2,
        )
        self.assertEqual(request["d_pred"], 2)
        self.assertEqual(request["s"], RTC_EXECUTION_HORIZON)
        self.assertEqual(request["schedule"], "exp")
        self.assertEqual(request["beta"], 5.0)
        self.assertNotIn("predicted_delay_steps", request)
        response = {
            "ok": True,
            "request_id": "request",
            "plan_id": "plan",
            "timeline_version": 3,
            "checkpoint_id": 4,
            "actions": np.zeros((RTC_HORIZON, 16)),
            "client_timing": {"request_serialization_ms": 1.0},
        }
        actions, timing = parse_rtc_response(response, request=request)
        self.assertEqual(actions.shape, (RTC_HORIZON, 16))
        self.assertEqual(timing["client_timing"]["request_serialization_ms"], 1.0)

        response["timeline_version"] = 2
        with self.assertRaises(RtcError):
            parse_rtc_response(response, request=request)


if __name__ == "__main__":
    unittest.main()
