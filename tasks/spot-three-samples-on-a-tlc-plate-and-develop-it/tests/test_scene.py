"""The scene file is what tools/build_assets.py writes from layout.py, and has the bodies, colliders and marks the
adapter and the rules expect. Standard library only."""

import importlib.util
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_tlc_spotting import fixed_scene as F  # noqa: E402
from rdb_tlc_spotting import layout as L  # noqa: E402
from rdb_tlc_spotting import skin  # noqa: E402

TEXT = F.USD.read_text()


def builder():
    spec = importlib.util.spec_from_file_location("build_assets", ROOT / "tools" / "build_assets.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def block(name):
    """The text of the prim ``name`` (its first definition)."""
    start = TEXT.index(f'"{name}" (')
    depth, i = 0, TEXT.index("{", start)
    for j in range(i, len(TEXT)):
        depth += {"{": 1, "}": -1}.get(TEXT[j], 0)
        if depth == 0:
            return TEXT[start:j + 1]
    raise AssertionError(name)


class Scene(unittest.TestCase):
    def test_file_is_what_the_builder_writes(self):
        self.assertEqual(builder().usd(), TEXT)

    def test_loose_bodies(self):
        for name in F.ACTORS:
            text = block(name)
            self.assertIn("PhysicsRigidBodyAPI", text)
            self.assertIn("PhysxContactReportAPI", text)       # the adapter's contact views need it
        masses = {"plate": L.PLATE_MASS, **{n: L.CAP_MASS for n in F.SPOTTERS}, "lid": L.LID_MASS}
        for name, mass in masses.items():
            self.assertIn(f"float physics:mass = {mass:.6g}", block(name))

    def test_start_poses(self):
        for name, ((x, y, z), _) in F.START.items():
            m = re.search(r"double3 xformOp:translate = \(([^)]*)\)", block(name))
            values = [float(v) for v in m.group(1).split(",")]
            self.assertAlmostEqual(values[0], x, places=5)
            self.assertAlmostEqual(values[1], y, places=5)
            self.assertAlmostEqual(values[2], z + F.TOP, places=5)

    def test_colliders(self):
        plate = block("plate")
        self.assertIn(f"float3 xformOp:scale = ({L.PLATE_LEN / 2:.6g}, {L.PLATE_WIDE / 2:.6g}, {L.PLATE_T / 2:.6g})",
                      plate)
        spotter = block("spotter_1")
        self.assertIn(f"({L.CAP_OD / 2:.6g}, {L.CAP_OD / 2:.6g}, {L.CAP_LEN / 2:.6g})", spotter)
        tank = block("tank")
        for name in ("floor", "front_wall", "back_wall", "left_wall", "right_wall"):
            self.assertIn(f'def Cube "{name}"', tank)
            self.assertIn(f'def Cube "pane_{name}"', tank)                     # drawn as see-through glass
        self.assertIn(f"({L.LID[0] / 2:.6g}, {L.LID[1] / 2:.6g}, {L.LID[2] / 2:.6g})", block("lid"))
        self.assertIn("OmniGlass.mdl", TEXT)
        for k in (1, 2, 3):
            self.assertIn(f'"vial_{k}_wall_00"', TEXT)
            self.assertIn(f'"vial_{k}_neck_00"', TEXT)
        self.assertIn('"rack_floor"', TEXT)
        self.assertIn('"bar"', TEXT)

    def test_marks_the_skin_switches(self):
        for k in (1, 2, 3):
            for c in skin.COLOURS:
                for s in range(len(skin.SPOT_SIZES)):
                    self.assertIn(f'"spot_{k}_{c}_{s}"', TEXT)
                self.assertIn(f'"load_{c}"', block(f"spotter_{k}"))
            self.assertIn(f'"bare_{k}"', TEXT)
        self.assertEqual(skin.SPOT_SIZES, builder().SPOT_SIZES)

    def test_no_third_party_assets(self):
        self.assertNotRegex(TEXT, r"@[^@]+\.(usd|usda|usdc|png|jpg|obj|stl)@")


if __name__ == "__main__":
    unittest.main()
