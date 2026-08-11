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
    def test_delay_estimator_uses_guard_and_rejects_infeasible_delay(self):
        estimator = DelayEstimator()
        estimator.record_seconds(0.289)
        self.assertEqual(estimator.predicted_steps(7.5), 3)
        estimator.record_seconds(0.60)
        with self.assertRaises(RtcError):
            estimator.predicted_steps(7.5)

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
        self.assertEqual(request["s"], 6)
        self.assertEqual(request["schedule"], "exp")
        self.assertEqual(request["beta"], 5.0)
        self.assertNotIn("predicted_delay_steps", request)
        response = {
            "ok": True,
            "request_id": "request",
            "plan_id": "plan",
            "timeline_version": 3,
            "checkpoint_id": 4,
            "actions": np.zeros((10, 16)),
        }
        actions, _ = parse_rtc_response(response, request=request)
        self.assertEqual(actions.shape, (10, 16))

        response["timeline_version"] = 2
        with self.assertRaises(RtcError):
            parse_rtc_response(response, request=request)


if __name__ == "__main__":
    unittest.main()
