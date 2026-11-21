"""The scene's numbers: the published part sizes, the scene file built from them, where things start. Standard library
only (the scene file is read as text)."""

import math
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_sling_guards import fixed_scene as F  # noqa: E402
from rdb_sling_guards import grading  # noqa: E402
from rdb_sling_guards import hook as HK  # noqa: E402
from rdb_sling_guards import layout as L  # noqa: E402

USD = F.USD.read_text()
IN = .0254


class PublishedSizes(unittest.TestCase):
    def test_bar_and_sling(self):
        self.assertAlmostEqual(L.BAR_S, 4 * IN)
        self.assertAlmostEqual(L.BAR_L, 12 * IN)
        self.assertAlmostEqual(L.SLING_W, 2 * IN)
        self.assertAlmostEqual(L.BEARING_L, 36 * IN)
        self.assertAlmostEqual(L.WEB_L + 2 * L.FIT_H, L.BEARING_L)
        self.assertAlmostEqual(L.N_LINKS * L.LINK_L, L.WEB_L)
        self.assertLess(L.LINK_L, .0155)                       # 15 mm links

    def test_double_hook_is_the_condorlift_1000_kg(self):
        self.assertAlmostEqual(L.RAM_B, .080)                 # B
        self.assertAlmostEqual(L.BOWL_R_IN, .032)             # R
        self.assertAlmostEqual(L.HOOK_T, .020)                # X1
        # the horns are a 20 mm round bar bent on a 42 mm radius: 184 mm overall against the published A of 175 mm
        # (TECHNICAL.md, "The double hook")
        tip = max(abs(y) for _, (y, _), (y2, _), r in HK.hook_capsules() for y in (y, y2))
        self.assertAlmostEqual(2 * (tip + L.HOOK_R), .175, delta=.010)

    def test_guard_fits_the_gripper_and_covers_the_web(self):
        self.assertAlmostEqual(L.GUARD_LEN, 3 * IN)
        self.assertLess(L.GUARD_LEN, 2 * .04695)               # the YAM's pads open 93.9 mm
        self.assertGreater(L.GUARD_LEN / 2 - L.TAB_T, L.SLING_W / 2)   # end walls outside the web
        self.assertAlmostEqual(L.MAGNET_PULL_N, 17.08, places=2)
        self.assertGreater(L.GUARD_MASS, .035)
        self.assertLess(L.GUARD_MASS, .050)


class SceneFile(unittest.TestCase):
    def test_bodies(self):
        for name in (F.BAR, F.SLING, F.GUARDS, F.RIG, *F.FITS, *F.LINKS, *F.GUARD_NAMES, "hook", "latchL", "latchR"):
            self.assertIn(f'"{name}"', USD, name)
        for joint in F.RIG_JOINTS:
            self.assertIn(f'"{joint}"', USD, joint)

    def test_guard_mass_and_parts(self):
        masses = re.findall(r'def Xform "guard_\d"[^}]*?float physics:mass = ([0-9.e-]+)', USD, re.S)
        self.assertEqual(len(masses), 4)
        for value in masses:
            self.assertAlmostEqual(float(value), L.GUARD_MASS, places=4)
        for part in ("face_flange", "side_flange", "nose", "tab_0", "tab_1", "lobe_0", "lobe_1"):
            self.assertEqual(USD.count(f'"{part}"'), 4, part)

    def test_bar_is_kinematic(self):
        block = USD[USD.index(f'def Xform "{F.BAR}"'):]
        self.assertIn("bool physics:kinematicEnabled = 1", block[:2000])


class StartState(unittest.TestCase):
    def test_trays_hold_their_guards(self):
        for name, (side, edge, corner) in F.tray_slots().items():
            x, y = L.TRAY_XY[side]
            self.assertLess(abs(corner[0] - x), L.TRAY_SIZE[0] / 2)
            self.assertLess(abs(corner[1] - y), L.TRAY_SIZE[1] / 2 - L.TRAY_WALL[0])
            self.assertEqual(edge[1], "L" if side == "left" else "R")
            self.assertIsNone(grading.guard_seat(corner, (0., 0., 0., 1.))[0])

    def test_sling_lies_under_the_bar_with_fittings_folded_back(self):
        bodies = {n: p for n, p, _ in F.sling_start()}
        mid = bodies[F.SLING_ROOT]
        self.assertLess(abs(mid[1]), L.HALF)
        self.assertLess(mid[2], L.Z_BOT)
        self.assertGreater(bodies["fitA"][1], L.HALF + .15)
        self.assertLess(bodies["fitB"][1], -L.HALF - .15)
        bends = F.sling_start_bends()
        self.assertEqual(len(bends), L.N_LINKS + 1)
        self.assertGreater(abs(math.degrees(bends[0])), 150.)     # fitting A turned back over its leg
        self.assertGreater(abs(math.degrees(bends[-1])), 150.)

    def test_hook_hangs_over_the_sling(self):
        self.assertAlmostEqual(F.HOOK_O[0], L.XS)
        self.assertAlmostEqual(F.HOOK_O[2], L.HOOK_Z + L.BOWL_R_IN)
        self.assertGreater(L.HOOK_Z - L.HOOK_R, L.Z_TOP + .03)     # well clear of the bar's top

    def test_instruction_does_not_mention_guards(self):
        text = F.INSTRUCTION.lower()
        for word in ("guard", "edge", "pad", "protect", "safety", "osha"):
            self.assertNotIn(word, text)


if __name__ == "__main__":
    unittest.main()
