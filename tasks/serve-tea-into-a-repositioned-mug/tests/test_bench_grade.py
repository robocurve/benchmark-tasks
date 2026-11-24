import json
import unittest
from pathlib import Path

from tea.bench_grade import grade_sheet


def measured():
    """Synthetic worksheet values exercise the reader, without claiming a bench trial."""
    file = Path(__file__).resolve().parents[1] / "bench/physical_trial.json"
    row = json.loads(file.read_text())
    row.update(video_ref="example trial", calibration_ref="example calibration", equipment_ok=True)
    row["calibration"].update(scale_reading_error_g=0.05, balance_limit_g=0.4)
    for name, (before, after) in {"pitcher": (400, 280), "mug": (100, 220),
                                   "resident": (35, 35), "other": (45, 45)}.items():
        row["weighings"][name].update(before_g=before, after_g=after, collection_error_g=0.05)
    row["stage"].update(trigger_level_reached=True, robot_interference_observed=False,
                        onset_g=48, onset_g_error=0.3, travel_mm=83, travel_mm_error=0.4,
                        duration_s=1.01, duration_s_error=0.02)
    row["video"].update(witness_latched=False, drop_observed=False, visible_discharge=False,
                        supported_throughout=True, counter_supported_throughout=True,
                        released_throughout=True)
    for name, v, e in (("window_s", 2.10, .02), ("position_span_mm", 0.5, .2),
                       ("orientation_span_deg", .6, .1), ("max_tilt_deg", 2, .5),
                       ("min_clearance_mm", 60, 1)):
        row["video"].update({name: v, name+"_error": e})
    return row


class BenchWorksheet(unittest.TestCase):
    def test_complete_and_partial_and_fault(self):
        row = measured()
        self.assertEqual(grade_sheet(row)["score"], 1)
        row["video"].update(counter_supported_throughout=False, released_throughout=False,
                            clear_35mm_throughout=False, min_clearance_mm=0)
        self.assertEqual(grade_sheet(row)["score"], 0.5)
        row["video"]["witness_latched"] = True
        self.assertEqual(grade_sheet(row)["score"], 0)

    def test_calibration_ambiguity_and_unaccounted_transfer(self):
        for change in ("equipment", "mass_balance", "threshold", "duration", "near_clearance", "missing_input"):
            with self.subTest(change=change):
                row = measured()
                if change == "equipment":
                    row["equipment_ok"] = False
                elif change == "mass_balance":
                    row["weighings"]["pitcher"]["after_g"] -= 6
                elif change == "threshold":
                    row["weighings"]["pitcher"]["after_g"] -= 2
                    row["weighings"]["resident"]["after_g"] += 2
                elif change == "duration":
                    row["stage"]["duration_s"] = 1.16
                elif change == "near_clearance":
                    row["video"]["min_clearance_mm"] = 35
                else:
                    row["video"]["supported_throughout"] = None
                self.assertEqual(grade_sheet(row)["status"], "invalid")

    def test_unlocated_water_can_change_boundary_even_within_balance_limit(self):
        row = measured()
        row["weighings"]["pitcher"]["after_g"] -= 2.15
        row["weighings"]["resident"]["after_g"] += 1.8
        result = grade_sheet(row)
        self.assertIsNone(result["score"])
        self.assertIn("resident", result["reason"])

    def test_distinguishes_examiner_error_from_observed_interference(self):
        row = measured()
        row["stage"]["travel_mm"] = 40
        self.assertIsNone(grade_sheet(row)["score"])
        row["stage"]["robot_interference_observed"] = True
        self.assertEqual(grade_sheet(row)["score"], 0)
