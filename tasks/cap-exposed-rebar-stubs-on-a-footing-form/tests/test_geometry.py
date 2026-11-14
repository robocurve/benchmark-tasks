"""The scene's numbers hang together: the cap fits the bar, the staging and grasp fit the gripper."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import geometry as g  # noqa: E402


class TestGeometry(unittest.TestCase):
    def test_bar_is_number_four_rebar(self):
        self.assertAlmostEqual(2 * g.BAR_R, 0.0127, places=4)

    def test_cap_fits_the_bar_with_a_little_play(self):
        play = (g.CAP_BORE - 2 * g.BAR_R) / 2
        self.assertGreater(play, 0.0003)
        self.assertLess(play, 0.0015)
        self.assertGreater(g.CAP_BORE_DEPTH, g.BAR_H * 0.4)           # the bar goes well into the bore

    def test_funnel_catches_the_demo_landing_error(self):
        capture = g.CAP_FUNNEL_HALF - g.BAR_R
        self.assertGreaterEqual(capture, 0.010)                      # soft arms land 3 to 6 mm off

    def test_seated_height_puts_the_bar_up_the_bore(self):
        inside = g.BAR_TOP - g.SEATED_Z
        self.assertGreater(inside, g.SEAT_INSIDE_MIN)
        self.assertLess(inside, g.CAP_BORE_DEPTH + 0.002)

    def test_cap_head_fits_the_open_gripper_and_is_pinchable(self):
        self.assertLessEqual(g.CAP_HEAD, 0.080)                      # pinched width at most 80 mm
        self.assertLess(g.CAP_HEAD, 0.094)

    def test_neighbouring_caps_leave_room_for_the_fingers(self):
        gap = min(abs(a - b) for a, b in zip(g.STUB_Y, g.STUB_Y[1:])) - g.CAP_HEAD
        self.assertGreater(gap, 0.035)

    def test_loose_caps_stand_in_front_of_the_bare_stubs(self):
        self.assertEqual(len(g.CAP_START_Y), len(g.BARE))
        self.assertEqual(tuple(g.STUB_Y[i] for i in g.BARE), g.CAP_START_Y)
        self.assertLess(g.PAD_X + g.PAD_SIZE[0] / 2, g.FORM_X - g.FORM_SIZE[0] / 2 + 0.001)   # pads clear of the form

    def test_two_stubs_start_capped(self):
        self.assertEqual(len(g.PRE_CAPPED), 2)
        self.assertEqual(len(g.BARE), 3)


if __name__ == "__main__":
    unittest.main()
