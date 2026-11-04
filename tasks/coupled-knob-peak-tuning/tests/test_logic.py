"""Unit tests for Task G hidden-state logic: ridge function, hysteresis, press budgeting, and numerical study."""

from __future__ import annotations

import unittest
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import logic


class TestTaskGLogic(unittest.TestCase):
    def test_peak_and_start_values(self):
        # Exact peak at (80, 50) mm
        r_peak = logic.reading(80.0, 50.0)
        self.assertAlmostEqual(r_peak, 1.0, places=6)

        # Initial setting at (30, 30) mm is near 0.0 (~0.0340)
        r_start = logic.reading(30.0, 30.0)
        self.assertAlmostEqual(r_start, 0.033955, places=4)
        self.assertLess(r_start, 0.05)

    def test_ridge_anisotropy(self):
        # Along the ridge: delta = (+10, +10) mm => u = 20 / sqrt(2) ~ 14.14 mm, v = 0
        r_along = logic.reading(80.0 + 10.0, 50.0 + 10.0)
        # Across the ridge: delta = (-10, +10) mm => u = 0, v = 20 / sqrt(2) ~ 14.14 mm
        r_across = logic.reading(80.0 - 10.0, 50.0 + 10.0)

        # Response along major axis should be vastly higher than across minor axis (su=45 vs sv=10)
        self.assertGreater(r_along, 0.95)
        self.assertLess(r_across, 0.40)
        self.assertGreater(r_along / r_across, 2.5)

    def test_button_hysteresis_state_machine(self):
        tracker = logic.KnobTuningLogic()
        self.assertEqual(tracker.press_count, 0)
        self.assertEqual(tracker.needle_r, 0.0)

        # 1. At rest (z = 0.0 m)
        s = tracker.step(0.0, 30.0, 30.0)
        self.assertFalse(s.rising_edge)
        self.assertFalse(s.button_pressed)
        self.assertEqual(s.press_count, 0)

        # 2. Descend partially (z = -0.004 m, 50% travel) -> still not triggered
        s = tracker.step(-0.004, 30.0, 30.0)
        self.assertFalse(s.rising_edge)
        self.assertFalse(s.button_pressed)
        self.assertEqual(s.press_count, 0)

        # 3. Descend past pressed threshold (z = -0.007 m > -0.0064 m) -> RISING EDGE
        s = tracker.step(-0.007, 50.0, 25.0)
        self.assertTrue(s.rising_edge)
        self.assertTrue(s.button_pressed)
        self.assertEqual(s.press_count, 1)
        self.assertEqual(s.counted_presses, 1)
        self.assertTrue(s.is_counted)
        self.assertAlmostEqual(s.needle_r, logic.reading(50.0, 25.0), places=5)
        # Only 1 press so far -> not yet distinct readings
        self.assertFalse(s.has_distinct_readings)

        # 4. Hold at bottom (z = -0.008 m) -> no new rising edge
        s = tracker.step(-0.008, 50.0, 25.0)
        self.assertFalse(s.rising_edge)
        self.assertTrue(s.button_pressed)
        self.assertEqual(s.press_count, 1)

        # 5. Partial release (z = -0.005 m < -0.0035 m) -> still pressed due to hysteresis
        s = tracker.step(-0.005, 50.0, 25.0)
        self.assertFalse(s.rising_edge)
        self.assertTrue(s.button_pressed)

        # 6. Release (z = -0.002 m, the cap's resting level, > -0.0035 m) -> released and re-armed
        s = tracker.step(-0.002, 50.0, 25.0)
        self.assertFalse(s.rising_edge)
        self.assertFalse(s.button_pressed)
        # Needle reading remains stored from previous press!
        self.assertAlmostEqual(s.needle_r, logic.reading(50.0, 25.0), places=5)

        # 7. Second press at SAME settings -> has_distinct_readings remains False!
        s = tracker.step(-0.007, 50.0, 25.0)
        self.assertTrue(s.rising_edge)
        self.assertEqual(s.press_count, 2)
        self.assertFalse(s.has_distinct_readings)
        tracker.step(0.0, 50.0, 25.0)

        # 8. Third press at DIFFERENT settings (> 10 mm diff) -> has_distinct_readings becomes True!
        s = tracker.step(-0.007, 65.0, 35.0)
        self.assertTrue(s.rising_edge)
        self.assertEqual(s.press_count, 3)
        self.assertTrue(s.has_distinct_readings)

    def test_press_budget_and_ignored_trace(self):
        tracker = logic.KnobTuningLogic()

        # Perform 8 presses within budget at varied settings
        for i in range(1, 9):
            s = tracker.step(-0.007, 25.0 + i * 5.0, 25.0)
            self.assertTrue(s.rising_edge)
            self.assertEqual(s.press_count, i)
            self.assertEqual(s.counted_presses, i)
            self.assertTrue(s.is_counted)
            # release
            tracker.step(0.0, 25.0 + i * 5.0, 25.0)

        self.assertTrue(s.has_distinct_readings)

        # 9th press: moves needle, but exceeds budget (is_counted = False)
        s9 = tracker.step(-0.007, 80.0, 50.0)
        self.assertTrue(s9.rising_edge)
        self.assertEqual(s9.press_count, 9)
        self.assertEqual(s9.counted_presses, 8)  # capped at 8
        self.assertFalse(s9.is_counted)
        self.assertAlmostEqual(s9.needle_r, 1.0, places=5)

    def test_variants_are_distinct_and_hard_to_memorise(self):
        """A policy that memorises the variant-0 peak (80, 50) must not reach the 0.95 rung on the other variants."""
        self.assertEqual(len(logic.VARIANTS), 3)
        for v in (1, 2):
            self.assertLess(logic.reading(80.0, 50.0, v), 0.80)
        for v in range(3):
            self.assertAlmostEqual(logic.reading(*logic.VARIANTS[v], v), 1.0, places=9)
            self.assertLess(logic.reading(30.0, 30.0, v), 0.5)  # the start must not already be a 0.5 rung

    def test_axis_search_cannot_finish_in_eight_readings(self):
        """The oracle single-axis search stays below 0.95 after 8 readings on every variant (the ablation's premise)."""
        for v in range(3):
            path = logic.coordinate_search_path(v, readings=8)
            self.assertLess(path[8][2], 0.95, f"variant {v}")
            self.assertLess(path[7][2], 0.95)
            # and every move is forward (the scripted plan only needs forward pushes)
            for a, b in zip(path, path[1:]):
                self.assertGreaterEqual(b[0], a[0] - 1e-9)
                self.assertGreaterEqual(b[1], a[1] - 1e-9)

    def test_ridge_path_reaches_the_band_and_the_peak(self):
        for v in range(3):
            pts = logic.ridge_path(v)
            rs = [logic.reading(x, y, v) for x, y in pts]
            self.assertLess(rs[0], 0.5)
            self.assertAlmostEqual(rs[-1], 1.0, places=6)
            self.assertGreaterEqual(logic.count_separated(pts), logic.MIN_SEPARATED_FOR_1_00)

    def test_count_separated(self):
        self.assertEqual(logic.count_separated([(30, 30), (33, 30), (30, 36)]), 1)
        self.assertEqual(logic.count_separated([(30, 30), (40, 30), (40, 41)]), 3)


if __name__ == "__main__":
    unittest.main()
