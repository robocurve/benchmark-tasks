"""The scripted demonstration in the kinematic dry run, scored by the real rules (no contact physics; the hands'
collision hulls are checked against the ukulele by position only). Needs NumPy and yam_arena (the YAM runtime's
kinematics, no Isaac Sim); skipped without them."""

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

HAVE = all(importlib.util.find_spec(name) is not None for name in ("numpy", "yam_arena"))


@unittest.skipUnless(HAVE, "needs numpy and yam_arena (the YAM runtime); no simulator")
class DryRun(unittest.TestCase):
    def test_full_marks_well_within_the_run_limit(self):
        import dryrun_demo

        report = dryrun_demo.dry_run()
        self.assertEqual(report["score"], 1.)
        self.assertLess(report["steps"], 800)
        self.assertLessEqual(abs(report["cents"]["g"]), 2.)
        self.assertGreaterEqual(report["worst_limit_margin_deg"], 4.)
        self.assertLess(report["worst_ik_mm"], .5)
        self.assertGreaterEqual(report["regrips"], 1)                     # more than a turn: at least one regrip
        self.assertTrue(all(abs(v) < .5 for k, v in report["cents"].items() if k != "g"))
        # a joint-space move from the ready pose would put the left fingertips on the body's upper edge
        self.assertGreaterEqual(report["worst_body_clearance_mm"], 3.)
        self.assertGreater(report["worst_c_string_clearance_mm"], 1.)
        # the right hand on the G button, the one nearer the nut: clear of the C button beside it, the headstock,
        # its posts and gear boxes, the strings between the nut and the posts, and the tuner
        gaps = report["clearance_mm"]
        self.assertGreater(gaps["right_button"], 10.)
        self.assertGreater(gaps["right_headstock"], 3.)
        self.assertGreater(gaps["right_strings"], 10.)
        self.assertGreater(gaps["right_tuner"], 10.)
        self.assertGreater(gaps["left_headstock"], 10.)
        # every grip turns the G button, within joint 6's limits
        grips = report["grips"]
        self.assertEqual({grip["button"] for grip in grips}, {"g"})
        self.assertEqual(len(grips), report["regrips"] + 1)
        self.assertTrue(all(grip["joint6_limit_margin_deg"] >= 4. for grip in grips), grips)
        from rdb_ukulele_tuning import layout as L
        turned = L.tighten_sign("g") * sum(grip["button_turned_deg"] for grip in grips)     # tightening degrees
        self.assertAlmostEqual(turned, -L.START_CENTS["g"] / L.CENTS_PER_DEG, delta=3.)

    def test_slow_sagging_arm(self):
        import dryrun_demo

        report = dryrun_demo.dry_run(arm_lag_s=.2, sag=(.002, -.002, -.004))
        self.assertEqual(report["score"], 1.)
        self.assertLess(report["steps"], 1000)
        self.assertGreater(report["worst_body_clearance_mm"], 0.)
        self.assertGreater(report["clearance_mm"]["right_headstock"], 0.)
        self.assertGreater(report["clearance_mm"]["right_button"], 5.)

    def test_controls(self):
        import dryrun_demo

        wrong = dryrun_demo.dry_run(variant="wrong_button")        # the C button, farther from the nut
        self.assertEqual(wrong["score"], 0.)
        self.assertGreater(abs(wrong["cents"]["c"]), 15.)
        self.assertEqual({grip["button"] for grip in wrong["grips"]}, {"c"})
        self.assertGreater(wrong["clearance_mm"]["right_button"], 10.)    # here the G button is the neighbour
        held = dryrun_demo.dry_run(variant="hold_on", max_steps=1200)
        self.assertEqual(held["score"], .75)
        self.assertFalse(held["grade"]["grippers_clear"])
        blind = dryrun_demo.dry_run(variant="blind")               # in tune, but never plucked after the last turn
        self.assertEqual(blind["score"], .75)
        self.assertLessEqual(abs(blind["cents"]["g"]), 2.)
        self.assertFalse(blind["grade"]["final_check"])

    def test_full_run_ends_with_a_check_pluck_after_letting_go(self):
        import dryrun_demo

        report = dryrun_demo.dry_run()
        self.assertTrue(report["grade"]["final_check"])
        events = [e["event"] for e in report["log"]["events"]]
        self.assertIn("final check", events)
        self.assertGreater(len(events) - 1 - events[::-1].index("final check"),
                           len(events) - 1 - events[::-1].index("let go of the button"))


if __name__ == "__main__":
    unittest.main()
