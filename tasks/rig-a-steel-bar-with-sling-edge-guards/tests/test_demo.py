"""The scripted demonstration in the kinematic dry run (tools/dryrun_demo.py): every pose reachable, every guard
seated and both fittings hooked by position, the arms clear of the bar, blocks, trays and table. No contact physics.
Needs NumPy and yam_arena (the YAM runtime's kinematics, no Isaac Sim); skipped without them."""

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tools", ROOT / "probe"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
if str(ROOT / "tools/stubs") not in sys.path:       # last: a real Inspect Robots, where installed, comes first (the
    sys.path.append(str(ROOT / "tools/stubs"))       # Dream tests in the same run need it)

HAVE = all(importlib.util.find_spec(name) is not None for name in ("numpy", "yam_arena"))


@unittest.skipUnless(HAVE, "needs numpy and yam_arena (the YAM runtime); no simulator")
class DryRun(unittest.TestCase):
    def test_full_demonstration(self):
        import dryrun_demo

        report = dryrun_demo.dry_run("full", clearance_every=8)
        self.assertTrue(report["done"])
        self.assertEqual(report["stopped"], [])
        self.assertEqual(set(report["seated"].values()), {"TL", "BL", "TR", "BR"})
        self.assertEqual(report["hooked"], {"fitA": ["L"], "fitB": ["R"]})
        self.assertLess(report["steps"], 2500)
        self.assertLess(report["worst_ik_mm"], 1.)
        for arm in ("left", "right"):
            self.assertGreater(report["worst_clearance"][f"{arm}_scene_mm"][0][2], 0., arm)
            self.assertGreater(report["worst_clearance"][f"{arm}_table_mm"][0][2], 0., arm)


if __name__ == "__main__":
    unittest.main()
