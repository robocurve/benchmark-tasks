"""Unit tests verifying all demo plan modes generate valid waypoints for segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import unittest
import demo_plan


class TestModes(unittest.TestCase):
    def test_all_modes_generate_waypoints(self):
        modes = ["demo", "same_bay_violation", "one_bottle_only", "settle"]
        for mode in modes:
            plans = demo_plan._plans(mode=mode)
            left_wps = plans["left"]
            right_wps = plans["right"]
            self.assertGreater(len(left_wps), 0, f"Left arm plan empty for mode {mode}")
            self.assertGreater(len(right_wps), 0, f"Right arm plan empty for mode {mode}")
            t_l = sum(w.duration for w in left_wps)
            t_r = sum(w.duration for w in right_wps)
            self.assertAlmostEqual(t_l, t_r, places=2, msg=f"Duration mismatch for mode {mode}: {t_l} vs {t_r}")


if __name__ == "__main__":
    unittest.main()
