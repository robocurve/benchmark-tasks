"""Sanity checks of the geometry: real sizes, the layouts, what the arms can reach, and that nothing is wider than
the real part."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import geometry as g  # noqa: E402

REACH = 0.50                                   # an arm's reach from its base (m); the demo's margin is checked offline too
BASES = {"left": (0.0, 0.305), "right": (0.0, -0.305)}


def dist_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class TestGeometry(unittest.TestCase):
    def test_real_sizes(self):
        self.assertAlmostEqual(2 * g.DISC_R, 0.115)           # a 115 mm disc
        self.assertAlmostEqual(g.DISC_T, 0.006)               # 6 mm grinding disc
        self.assertAlmostEqual(2 * g.MOTOR_R, 0.058, places=3)
        self.assertAlmostEqual(2 * g.NECK_R, 0.043, places=3)   # the 43 mm euro collar
        total = g.HEAD_Y1 - g.CAP_Y0
        self.assertTrue(0.26 <= total <= 0.30, total)         # a 115 mm grinder is about 270 mm long
        self.assertTrue(1.4 <= g.BODY_MASS <= 1.9)

    def test_the_guard_clears_the_disc_by_4mm_or_more(self):
        self.assertGreaterEqual(g.SKIRT_R_IN - g.DISC_R, 0.004)
        self.assertGreaterEqual(g.DISC_Z0 - (g.PLATE_Z0 + g.PLATE_T), 0.0)

    def test_hood_covers_about_190_degrees_and_the_skirt_is_above_the_disc(self):
        self.assertAlmostEqual(math.degrees(g.SKIRT_SPAN), 190.0)
        self.assertGreater(g.SKIRT_Z1, g.NUT_Z1)              # the skirt stands above the nut

    def test_lug_and_lever_are_outside_the_disc_and_graspable(self):
        self.assertGreaterEqual(g.LEVER_W, 0.012)             # a gripper's bite
        self.assertGreaterEqual(g.LUG_W, 0.006)
        self.assertGreater(g.LEVER_HINGE_R, g.DISC_R + 0.004)
        self.assertGreater(g.SKIRT_R_IN + g.SKIRT_T + g.LUG_L, g.DISC_R + 0.020)

    def test_the_lug_stands_above_the_rim_for_the_fingers(self):
        # the pads reach 13 mm below the tool point, which sits 10 mm over the lug's middle: they clear the rim by 4 mm or more
        import demo

        pad_bottom = g.LUG_MID + demo.LUG_ABOVE - 0.013
        self.assertGreaterEqual(pad_bottom - g.SKIRT_Z1, 0.004)
        self.assertGreater(g.LUG_Z0 + g.LUG_H, pad_bottom + 0.008)          # and at least 8 mm of the tab is in the bite

    def test_the_lever_clears_the_skirt_when_pinched(self):
        # pinched 45 mm from the hinge, the pad (13 mm long, 10 mm wide in the radial direction) stays 4 mm off the skirt's
        # outer face or above its rim, at every angle of the swing
        import demo

        for deg in range(0, 91, 2):
            b = math.radians(deg)
            radial = g.LEVER_HINGE_R + demo.LEVER_GRASP * math.cos(b) - 0.010
            pad_bottom = g.LEVER_HINGE_Z + demo.LEVER_GRASP * math.sin(b) + demo.LEVER_ABOVE - 0.013
            self.assertTrue(radial >= g.SKIRT_R_IN + g.SKIRT_T + 0.004 or pad_bottom >= g.SKIRT_Z1 + 0.004, deg)

    def test_the_lever_toggle_has_hysteresis(self):
        self.assertLess(g.LEVER_TOGGLE_DOWN, g.LEVER_TOGGLE_UP)
        self.assertGreater(g.LEVER_TOGGLE_DOWN, g.LEVER_FLAT_MAX)
        self.assertLess(g.LEVER_TOGGLE_UP, g.LEVER_OPENED)

    def test_layouts(self):
        for v in "AB":
            L = g.layout(v)
            start_err = g.bearing_error(g.guard_bearing(L["yaw"], L["guard_q0"]), L["operator_bearing"])
            self.assertGreaterEqual(start_err, g.BEARING_HALF + math.radians(20), v)   # the start is the wrong way round
            self.assertAlmostEqual(g.guard_bearing(L["yaw"], L["guard_q0"]), g.wrap(L["start_bearing"]))
        A, B = g.layout("A"), g.layout("B")
        self.assertAlmostEqual(math.degrees(g.bearing_error(A["start_bearing"], A["operator_bearing"])), 180.0)
        self.assertAlmostEqual(math.degrees(g.bearing_error(B["start_bearing"], B["operator_bearing"])), 120.0)

    def test_the_operator_mat_is_where_the_bearing_says(self):
        for v in "AB":
            L = g.layout(v)
            sx, sy = L["spindle"]
            ang = math.atan2(L["mat"][1] - sy, L["mat"][0] - sx)
            self.assertAlmostEqual(g.wrap(ang - L["operator_bearing"]), 0.0, places=6)
            self.assertAlmostEqual(dist_xy(L["mat"], L["spindle"]), g.MAT_DIST)

    def test_everything_is_within_reach_of_the_arm_that_works_on_it(self):
        for v in "AB":
            L = g.layout(v)
            op = "left" if L["spindle"][1] > 0 else "right"
            for q in (L["guard_q0"], L["guard_q0"] + math.pi):   # the guard at its start and turned half way round
                for pt in (g.lug_point(L, q), g.lever_hinge(L, q)):
                    self.assertLessEqual(dist_xy(pt, BASES[op]), REACH - 0.03, (v, op, pt))

    def test_the_grinder_rests_on_the_table(self):
        self.assertAlmostEqual(g.MOTOR_R, g.HEAD_H / 2, places=2)     # the motor's top meets the head's top
        self.assertLessEqual(g.HANDLE_Z + g.HANDLE_R, g.HEAD_H)

    def test_thresholds_are_ordered(self):
        self.assertLess(g.BEARING_OK, g.BEARING_NEAR)
        self.assertLess(g.BEARING_NEAR, g.BEARING_HALF)
        self.assertLess(g.MOVED_OK, g.MOVED_BAD)
        self.assertLess(g.YAW_OK, g.YAW_BAD)
        self.assertLess(g.LEVER_FLAT_MAX, g.LEVER_OPENED)


if __name__ == "__main__":
    unittest.main()
