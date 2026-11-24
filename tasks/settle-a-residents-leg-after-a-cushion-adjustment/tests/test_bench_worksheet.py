"""Portable bench worksheet regression tests using synthetic, explicitly measured intervals."""

import csv
import tempfile
import unittest
from pathlib import Path

from tools.bench import grade


TEMPLATE = Path(__file__).resolve().parents[1] / "docs/bench_checks.csv"


class BenchWorksheetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "readings.csv"
        with TEMPLATE.open(newline="") as src:
            self.rows = {r["check"]: r for r in csv.DictReader(src)}
        # Numeric pairs are measurement and absolute uncertainty, in worksheet units.
        measurements = {
            **{f"reset_heel_n_frame_{i}": (18, 0.1) for i in range(1, 6)},
            "coverage": "1-280",
            "reset_max_heel_gap_mm": (0, 0.2),
            "reset_max_bearing_regions": 0,
            "opposing_grip_windows": "none",
            "extra_heel_load_with_arm_contact_windows": "none",
            "dwell_max_marker_axis_excursion_mm": (1, 0.2),
            "dwell_all_slack": 1,
            "dwell_max_heel_n": (0.4, 0.2),
            "dwell_min_bearing_regions": 2,
            "dwell_max_sole_bed_gap_mm": (0.1, 0.1),
            "dwell_min_sole_up": (0.98, 0.01),
            "dwell_min_protected_gap_mm": (6, 0.4),
            "dwell_min_person_gap_mm": (12, 0.5),
            "dwell_min_heel_gap_mm": (23, 0.5),
            "dwell_min_bearing_span_mm": 86,
            "dwell_min_park_gap_mm": (40, 0.5),
        }
        self.assertEqual(set(measurements), set(self.rows))
        for name, data in measurements.items():
            value, uncertainty = data if isinstance(data, tuple) else (data, "")
            self.rows[name].update(
                value=value, uncertainty=uncertainty, evidence="synthetic test"
            )

    def evaluate(self):
        with self.path.open("w", newline="") as dst:
            writer = csv.DictWriter(
                dst, fieldnames=("check", "value", "uncertainty", "evidence")
            )
            writer.writeheader()
            writer.writerows(self.rows.values())
        return grade(self.path)

    def test_completed_sheet_grades_identically_on_reread(self):
        self.assertEqual(self.evaluate(), (1.0, "heel_floated"))
        self.assertEqual(grade(self.path), (1.0, "heel_floated"))

    def test_separated_intervals_do_not_form_a_latch(self):
        self.rows["opposing_grip_windows"]["value"] = "20-29 31-40"
        self.rows["extra_heel_load_with_arm_contact_windows"]["value"] = "49-58"
        self.assertEqual(self.evaluate(), (1.0, "heel_floated"))

    def test_earliest_eleven_sample_event_decides_reason(self):
        self.rows["opposing_grip_windows"]["value"] = "42-53"
        self.rows["extra_heel_load_with_arm_contact_windows"]["value"] = "41-51"
        self.assertEqual(self.evaluate(), (0.0, "sustained_extra_heel_load"))
        self.rows["opposing_grip_windows"]["value"] = "40-50"
        self.assertEqual(self.evaluate(), (0.0, "sustained_opposing_grip"))

    def test_dwell_uses_extrema_and_simultaneous_bearing(self):
        self.rows["dwell_min_bearing_regions"]["value"] = 1
        self.assertEqual(self.evaluate(), (0.5, "relief_incomplete"))
        self.rows["dwell_all_slack"]["value"] = 0
        self.assertEqual(self.evaluate(), (0.0, "no_sustained_relief"))
        self.rows["dwell_max_marker_axis_excursion_mm"].update(value=6, uncertainty=0.1)
        self.assertEqual(self.evaluate(), (0.0, "moving_at_end"))

    def test_uncertainty_in_both_baseline_and_dwell(self):
        self.rows["dwell_max_heel_n"].update(value=8.96, uncertainty=0.01)
        with self.assertRaisesRegex(ValueError, "half-baseline"):
            self.evaluate()

    def test_clearance_on_threshold_is_ambiguous(self):
        self.rows["dwell_min_heel_gap_mm"].update(value=10, uncertainty=0.2)
        with self.assertRaisesRegex(ValueError, "dwell_min_heel_gap_mm"):
            self.evaluate()

    def test_incomplete_evidence_and_intervals_are_unscorable(self):
        for name, column, bad in (
            ("coverage", "value", "1-279"),
            ("opposing_grip_windows", "value", "?"),
            ("opposing_grip_windows", "value", "40-49 50-53"),
            ("dwell_min_bearing_regions", "evidence", ""),
        ):
            with self.subTest(name=name, bad=bad):
                old = self.rows[name][column]
                self.rows[name][column] = bad
                with self.assertRaises(ValueError):
                    self.evaluate()
                self.rows[name][column] = old


if __name__ == "__main__":
    unittest.main()
