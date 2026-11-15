"""The scene's numbers hang together: the cap fits the bar, the staging and grasp fit the gripper."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import geometry as g  # noqa: E402


class TestGeometry(unittest.TestCase):
    def test_bar_is_number_four_rebar(self):
        self.assertAlmostEqual(2 * g.BAR_R, 0.0127, places=4)

    def test_cap_bore_and_fins_fit_the_bar(self):
        play = (g.CAP_BORE_Y - 2 * g.BAR_R) / 2
        self.assertGreater(play, 0.0008)                              # the bar fits between the bore's walls across y ... with room for a small tilt
        self.assertLess(play, 0.0015)
        interference = (2 * g.BAR_R - g.JAW_REST_GAP) / 2             # ... and each fin is pushed out by the bar
        self.assertAlmostEqual(interference, 0.001, places=4)
        self.assertGreater(g.JAW_ENGAGED, 0.0)
        self.assertLess(g.JAW_ENGAGED, interference)
        self.assertGreater(g.JAW_TRAVEL, interference)

    def test_fins_hold_the_cap_but_the_cap_cannot_be_dropped_on(self):
        weight = g.CAP_MASS * 9.81
        grip = 2 * g.JAW_K * ((2 * g.BAR_R - g.JAW_REST_GAP) / 2)    # N pressing on the bar from both fins
        self.assertGreater(2 * 0.6 * grip / 2, weight)                # friction retention beats the cap's weight

    def test_cap_meets_the_minimum_size_of_8_ccr_344_90(self):
        self.assertGreaterEqual(g.CAP_PLATE, 4 * 0.0254 - 1e-6)       # 4 x 4 in square plate
        self.assertAlmostEqual(g.CAP_MASS / 0.4536, 0.5, delta=0.02)  # about 0.5 lb, the listed weight of a 4 x 4 in cap

    def test_funnel_catches_the_demo_landing_error(self):
        capture = g.CAP_FUNNEL_HALF - g.BAR_R
        self.assertGreaterEqual(capture, 0.017 + 0.003)              # the pinch leaves up to 17 mm; soft arms land 3 to 6 mm off

    def test_seated_height_puts_the_bar_up_the_bore(self):
        inside = g.BAR_TOP - g.SEATED_Z
        self.assertGreater(inside, g.SEAT_INSIDE_MIN)
        self.assertLess(inside, g.CAP_BORE_DEPTH + 0.002)

    def test_the_boss_is_pinchable_and_the_plate_is_too_wide_to_pinch(self):
        self.assertLessEqual(g.CAP_BOSS, 0.080)                      # pinched width at most 80 mm
        self.assertGreater(g.CAP_PLATE, 0.094)                       # the 4 in plate is wider than the open gripper: hence the boss
        self.assertGreaterEqual(g.CAP_BOSS_H, 0.020)                 # tall enough for the pads to take hold
        self.assertLess(g.CAP_BOSS_H + g.CAP_PLATE_T + g.CAP_BODY_H, 0.08)

    def test_neighbouring_caps_leave_room_for_the_fingers(self):
        gap = min(abs(a - b) for a, b in zip(g.STUB_Y, g.STUB_Y[1:])) - g.CAP_PLATE
        self.assertGreater(gap, 0.015)                               # the plates of neighbouring caps do not touch

    def test_loose_caps_stand_in_front_of_the_bare_stubs(self):
        self.assertEqual(len(g.CAP_START_Y), len(g.BARE))
        self.assertEqual(tuple(g.STUB_Y[i] for i in g.BARE), g.CAP_START_Y)
        self.assertLess(g.PAD_X + g.PAD_SIZE[0] / 2, g.FORM_X - g.FORM_SIZE[0] / 2 + 0.001)   # pads clear of the form

    def test_two_stubs_start_capped(self):
        self.assertEqual(len(g.PRE_CAPPED), 2)
        self.assertEqual(len(g.BARE), 3)


if __name__ == "__main__":
    unittest.main()
