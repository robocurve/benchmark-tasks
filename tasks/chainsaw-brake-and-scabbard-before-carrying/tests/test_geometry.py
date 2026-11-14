"""Sanity checks of the layout, the real sizes and the fits."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import geometry as g


class Geometry(unittest.TestCase):
    def test_real_sizes(self):
        # a 10 in (25 cm) cordless chainsaw: 2.3 to 3.0 kg, 40 to 50 cm overall
        self.assertGreaterEqual(g.SAW_MASS, 2.3)
        self.assertLessEqual(g.SAW_MASS, 3.0)
        overall = g.BAR_LEN + g.BATTERY_Y[1]
        self.assertTrue(0.40 <= overall <= 0.55, overall)
        self.assertAlmostEqual(g.BAR_LEN, 0.25, places=3)
        self.assertTrue(0.08 <= g.GUARD_H <= 0.12)

    def test_scabbard_fit_clearance_is_at_least_4_mm(self):
        side = (g.SCAB_IN_W - 2 * g.BAR_OUT_HALF_W) / 2
        vert = (g.SCAB_IN_H - g.CHAIN_H) / 2
        self.assertGreaterEqual(side, 0.004)
        self.assertGreaterEqual(vert, 0.004)
        self.assertGreater(g.SCAB_LEN, g.BAR_LEN)

    def test_scabbard_rests_level_with_the_bar(self):
        # the holder's top is the scabbard's underside, and the scabbard's channel mid-height is the bar's
        self.assertAlmostEqual(g.HOLDER_TOP, g.SCAB_CENTER_Z - g.SCAB_OUT_H / 2)
        self.assertLess(g.HOLDER_W, g.SCAB_OUT_W)          # the gripper's pads clear the holder
        self.assertAlmostEqual(g.SCAB_CENTER_Z, g.BAR_Z)

    def test_scabbard_starts_beyond_the_tip_and_can_cover_the_bar(self):
        tip = g.SAW_Y - g.S * g.BAR_LEN
        self.assertAlmostEqual(g.SCAB_OPEN_Y_START, tip - g.S * g.SCAB_START_GAP)
        slide = abs((g.SAW_Y - g.S * (0.004 + g.SCAB_LEN / 2)) - g.SCAB_CENTER_Y_START)
        self.assertGreater(slide, 0.25)                    # it travels about 30 cm
        self.assertGreater(g.S * (tip - (g.SCAB_CENTER_Y_START + g.S * g.SCAB_LEN / 2)), 0)   # and starts clear of the bar

    def test_hand_guard_geometry(self):
        self.assertLess(g.GUARD_LIMITS[0], g.BRAKE_STOP)
        self.assertLess(g.BRAKE_STOP, g.BRAKE_ON)
        self.assertLess(g.BRAKE_ON, g.BRAKE_DETENT)
        self.assertLess(g.BRAKE_DETENT, g.BRAKE_RELEASE)
        self.assertLessEqual(g.BRAKE_RELEASE, 0.0)
        # the guard's hinge clears the housing's edge by more than its boss radius
        self.assertGreater(g.GUARD_PIVOT[0] - g.HOUSING_X[1], 0.006)
        # room for a gripper's pad between the guard and the front handle
        gap = g.FRONT_HANDLE["y0"] - g.FRONT_HANDLE["r"] - (g.GUARD_PIVOT[1] + g.GUARD_BAR_T / 2)
        self.assertGreater(gap, 0.030)

    def test_everything_is_in_reach(self):
        base = {"left": 0.305, "right": -0.305}
        gx = g.SAW_X + g.GUARD_PIVOT[0] + g.GUARD_LEN
        gy = g.SAW_Y + g.S * g.GUARD_PIVOT[1]
        self.assertLess(math.hypot(gx, gy - base[g.GUARD_ARM]), 0.50)
        sx, sy = g.SCAB_X, g.SCAB_CENTER_Y_START
        self.assertLess(math.hypot(sx, sy - base[g.COVER_ARM]), 0.50)

    def test_the_saw_rests_on_the_table(self):
        self.assertEqual(g.HOUSING_Z[0], 0.0)
        self.assertGreater(g.BAR_Z, g.COVER_Z[1])           # the bar sits on the sprocket cover


if __name__ == "__main__":
    unittest.main()
