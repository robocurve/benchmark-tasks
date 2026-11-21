"""The scripted demonstration in the kinematic dry run, scored by the real rules (no contact physics). Needs NumPy and
yam_arena (the YAM runtime's kinematics, no Isaac Sim); skipped without them."""

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
    def test_full_marks_within_the_run_limit(self):
        import dryrun_demo
        from rdb_tlc_spotting import fixed_scene as F

        report = dryrun_demo.dry_run()
        self.assertEqual(report["score"], 1.)
        self.assertLess(report["steps"], F.MAX_STEPS * .8)
        # behind the bar's far end the right arm's joint 4 reaches its limit (tool point up to 4.3 mm off), and the
        # bar's end face stops the plate there anyway; everywhere else the inverse kinematics is exact
        self.assertGreaterEqual(report["worst_limit_margin_deg"], 0.)
        self.assertLess(report["worst_ik_mm"], 5.)
        for lane in report["grade"]["lanes"]:
            self.assertTrue(lane["covered"] and lane["small"] and lane["clean"])
        self.assertEqual(report["grade"]["racked"], [0, 1, 2])

    def test_controls(self):
        import dryrun_demo

        hard = dryrun_demo.dry_run(variant="press_hard")
        self.assertFalse(hard["grade"]["lanes"][1]["clean"])
        self.assertLess(hard["score"], 1.)
        dry = dryrun_demo.dry_run(variant="no_dip")
        self.assertEqual(dry["grade"]["spots"], [])
        self.assertLessEqual(dry["points"], 3)


if __name__ == "__main__":
    unittest.main()
