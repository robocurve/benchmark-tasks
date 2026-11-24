"""Boundary decisions apply identically to simulated observations and bench worksheets."""

import unittest

from tea.primary import grade


def full_sample(**edits):
    sample = dict(
        measurements_valid=True, fault_latched=False, mug_water_g=120.0,
        resident_water_g=0.0, other_spill_g=0.0, window_s=2.0, visible_discharge=False,
        position_span_mm=0.0, orientation_span_deg=0.0, max_tilt_deg=0.0,
        supported_throughout=True, counter_supported_throughout=True,
        released_throughout=True, clear_35mm_throughout=True, receiver_motion_completed=True,
    )
    sample.update(edits)
    return sample


class GradingContractTests(unittest.TestCase):
    def test_masses_at_and_across_decision_points(self):
        for grams, expected in [(29.999, 0), (30, .5), (30.001, .5), (99.999, .5), (100, 1),
                                (100.001, 1), (129.999, 1), (130, 1), (130.001, 0)]:
            with self.subTest(grams=grams):
                self.assertEqual(grade(full_sample(mug_water_g=grams))["score"], expected)
        for key, limit in [("resident_water_g", 2), ("other_spill_g", 10)]:
            for delta, expected in [(-.001, 1), (0.0, 1), (.001, 0)]:
                self.assertEqual(grade(full_sample(**{key: limit + delta}))["score"], expected)

    def test_dwell_clearance_cessation_and_release(self):
        for values, want in [
            (dict(window_s=1.99999), 0), (dict(window_s=2.0, position_span_mm=3.0, orientation_span_deg=3.0), 1),
            (dict(position_span_mm=3.001), 0), (dict(orientation_span_deg=3.001), 0),
            (dict(visible_discharge=True), 0), (dict(max_tilt_deg=6.0), 1), (dict(max_tilt_deg=6.001), .5),
            (dict(released_throughout=False), .5), (dict(clear_35mm_throughout=False), .5),
            (dict(counter_supported_throughout=False), .5),
            (dict(supported_throughout=False, counter_supported_throughout=False), 0),
            (dict(fault_latched=True), 0),
        ]:
            with self.subTest(values=values):
                self.assertEqual(grade(full_sample(**values))["score"], want)

    def test_motion_is_mandatory_only_for_full_official_score(self):
        sample = full_sample(receiver_motion_completed=False)
        self.assertEqual(grade(sample)["status"], "invalid")
        self.assertEqual(grade(full_sample(mug_water_g=40, receiver_motion_completed=False))["score"], .5)
        self.assertEqual(grade(sample, "static_control")["score"], 1.0)
        with self.assertRaises(ValueError):
            grade(sample, "accidentally_static")

    def test_invalid_equipment_or_missing_observations_have_no_score(self):
        for overrides in [dict(measurements_valid=False), dict(other_spill_g=-.01), dict(mug_water_g=True),
                          dict(resident_water_g=float("nan")), dict(other_spill_g=float("inf")),
                          dict(orientation_span_deg=181), dict(clear_35mm_throughout=None),
                          dict(supported_throughout=False)]:
            with self.subTest(overrides=overrides):
                r = grade(full_sample(**overrides))
                self.assertEqual((r["status"], r["score"]), ("invalid", None))
        sample = full_sample()
        for key in sample:
            self.assertIsNone(grade({k:v for k,v in sample.items() if k != key})["score"])
