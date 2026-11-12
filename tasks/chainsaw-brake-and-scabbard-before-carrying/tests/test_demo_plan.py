"""The demo plan: every mode generates its motion, stays open loop, and the IK reaches it (needs yam_arena's ik)."""

import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import demo


class DemoPlan(unittest.TestCase):
    def test_modes_exist(self):
        self.assertEqual(set(demo.MODES), {"demo", "no_brake", "no_scabbard", "half_scabbard", "touch_chain", "settle"})

    def test_the_demo_is_open_loop(self):
        text = (Path(__file__).resolve().parent.parent / "demo.py").read_text()
        self.assertIsNone(re.search(r"ctx\.states\(|\.pos\b.*states|object_states", text.replace("arm.pos", "")),
                          "the demo must not read the simulator's object states")

    def test_every_mode_plans_and_reaches(self):
        import check_plan

        for mode in demo.MODES:
            r = check_plan.check(mode)
            self.assertGreater(r["steps"], 100, mode)
            self.assertLess(r["worst_pos_mm"], 3.0, mode)
            self.assertGreater(r["min_margin_rad"], 0.02, mode)

    def test_the_demo_fits_the_run_limit(self):
        import check_plan

        self.assertLess(check_plan.check("demo")["steps"], 2400)


if __name__ == "__main__":
    unittest.main()
