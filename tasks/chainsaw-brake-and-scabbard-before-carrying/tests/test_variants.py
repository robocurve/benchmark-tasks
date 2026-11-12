"""Layout B (TASK_VARIANT=B, the mirror image): the scoring, geometry and plan tests pass again with the layout mirrored."""

import os
import subprocess
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent


class MirrorLayout(unittest.TestCase):
    def run_variant(self, variant: str) -> subprocess.CompletedProcess:
        env = dict(os.environ, TASK_VARIANT=variant, PYTHONDONTWRITEBYTECODE="1")
        return subprocess.run([sys.executable, "-m", "unittest", "test_logic", "test_geometry", "test_demo_plan"],
                              cwd=HERE, env=env, capture_output=True, text=True)

    def test_layout_b_passes_the_same_tests(self):
        r = self.run_variant("B")
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])

    def test_layout_b_is_a_mirror_of_layout_a(self):
        code = ("import geometry as g; print(g.SAW_X, g.SAW_Y, g.SCAB_CENTER_Y_START, g.HOLDER_Y, g.GUARD_ARM, g.COVER_ARM)")
        out = {}
        for v in ("A", "B"):
            env = dict(os.environ, TASK_VARIANT=v, PYTHONPATH=str(HERE.parent), PYTHONDONTWRITEBYTECODE="1")
            out[v] = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True,
                                    cwd=HERE.parent).stdout.split()
        a, b = out["A"], out["B"]
        self.assertEqual(float(a[0]), float(b[0]))
        for k in (1, 2, 3):
            self.assertAlmostEqual(float(a[k]), -float(b[k]), places=9)
        self.assertEqual((a[4], a[5]), ("left", "right"))
        self.assertEqual((b[4], b[5]), ("right", "left"))

    def test_an_unknown_variant_is_refused(self):
        env = dict(os.environ, TASK_VARIANT="C", PYTHONPATH=str(HERE.parent), PYTHONDONTWRITEBYTECODE="1")
        r = subprocess.run([sys.executable, "-c", "import geometry"], env=env, capture_output=True, text=True,
                           cwd=HERE.parent)
        self.assertNotEqual(r.returncode, 0)


if __name__ == "__main__":
    unittest.main()
