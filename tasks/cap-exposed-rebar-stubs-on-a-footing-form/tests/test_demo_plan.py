"""The demo's plan is consistent with the scene without the IK: every pick and place lands on the geometry."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import demo  # noqa: E402
import geometry as g  # noqa: E402


class TestDemoPlan(unittest.TestCase):
    def test_modes_cover_the_task_json_ablations(self):
        for m in ("demo", "one_cap", "two_caps", "off_target", "stay", "settle"):
            self.assertIn(m, demo.MODES)

    def test_one_grasp_offset_per_loose_cap(self):
        self.assertEqual(len(demo.GRASP_DX), len(g.CAP_START_Y))
        for dx in demo.GRASP_DX:
            self.assertLessEqual(abs(dx), 0.006)                            # half the 12 mm the pinch can leave

    def test_each_arm_does_the_stubs_on_its_side(self):
        self.assertEqual(demo.side_for(-0.11), "right")
        self.assertEqual(demo.side_for(0.11), "left")

    def test_carry_height_clears_the_bar_tips(self):
        mouth = demo.CARRY_MOUTH
        self.assertGreaterEqual(mouth - g.BAR_TOP, 0.010)
        self.assertLess(demo.CARRY_MOUTH + demo.GRIP_UP, g.TOP + 0.20)       # within the arms' height range

    def test_pads_hold_the_head_not_the_stem(self):
        self.assertLess(g.CAP_STEM_H - (demo.GRIP_UP - 0.013), 0.010)      # pad tips just under the head


if __name__ == "__main__":
    unittest.main()
