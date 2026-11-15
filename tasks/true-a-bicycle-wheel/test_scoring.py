import unittest
from scoring import score, CHECKS


def record():
    return {"schema": "wheel-judging-v2", "checks": dict.fromkeys(CHECKS, True),
            "lateral_runout_mm": .5, "radial_runout_mm": .3, "dish_offset_mm": .1,
            "spoke_tensions_n": [600.]*32}


class ScoringTests(unittest.TestCase):
    def test_thresholds(self):
        r = record()
        self.assertEqual(score(r)["score"], 1.)
        for runout, expected in ((.5001,.5), (1.5,.5), (1.5001,0.)):
            r["lateral_runout_mm"] = runout
            self.assertEqual(score(r)["score"], expected)

    def test_straight_but_slack_does_not_pass(self):
        r = record()
        r["lateral_runout_mm"] = 0.
        r["spoke_tensions_n"][8] = 0.
        self.assertEqual(score(r)["score"], 0.)

    def test_braced_wheel_and_damaged_wheel_do_not_pass(self):
        r = record()
        r["checks"]["hands_and_tools_clear"] = False
        self.assertEqual(score(r)["score"], 0.)
        r = record()
        r["checks"]["wheel_and_stand_intact"] = False
        self.assertEqual(score(r)["score"], 0.)

    def test_missing_review_and_nonfinite_values_fail_closed(self):
        r = record()
        del r["checks"]["spoke_tension_check_complete"]
        self.assertFalse(score(r)["record_valid"])
        r = record()
        r["lateral_runout_mm"] = float("nan")
        self.assertFalse(score(r)["record_valid"])

    def test_draft_cannot_claim_submission_certification(self):
        self.assertFalse(score(record())["submission_certified"])

    def test_over_tension_and_missing_spoke_measurement_fail(self):
        r = record()
        r["spoke_tensions_n"][10] = 1000.1
        self.assertEqual(score(r)["score"], 0.)
        r = record()
        r["spoke_tensions_n"].pop()
        self.assertFalse(score(r)["record_valid"])


if __name__ == "__main__":
    unittest.main()
