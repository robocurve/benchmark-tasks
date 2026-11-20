"""The generated scene file matches the layout, the pads sit where the card says, and the frozen stroke poses still
put each stick's tip on its pad. Standard library only (the pose check needs yam_arena and is skipped without it)."""

import math
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tools"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import build_assets  # noqa: E402
from rdb_drum_groove import fixed_scene as F  # noqa: E402
from rdb_drum_groove import layout as L  # noqa: E402

TEXT = F.USD.read_text()


def block(name):
    """The text of the first prim called ``name`` (up to the next prim at the same depth)."""
    match = re.search(rf'def (\w+) "{re.escape(name)}"', TEXT)
    if match is None:
        raise KeyError(name)
    start = match.start()
    depth = start - (TEXT.rfind("\n", 0, start) + 1)
    following = re.search(rf'\n {{{depth}}}def \w+ "', TEXT[start + 5:])
    return TEXT[start:start + 5 + following.start()] if following else TEXT[start:]


def translate(text):
    return tuple(float(v) for v in re.search(r"double3 xformOp:translate = \(([^)]*)\)", text).group(1).split(","))


class SceneFile(unittest.TestCase):
    def test_scene_file_is_what_the_builder_writes(self):
        built = build_assets.usd()
        number = re.compile(r"-?\d+(?:\.\d*)?(?:e[-+]?\d+)?")
        self.assertEqual(number.sub("#", TEXT), number.sub("#", built))
        for a, b in zip(number.findall(TEXT), number.findall(built)):
            a, b = float(a), float(b)
            self.assertLessEqual(abs(a - b), 1.01e-5 * max(abs(a), abs(b)), (a, b))

    def test_one_fixed_kinematic_body_per_zone(self):
        kit = block(F.KIT)
        bodies = re.findall(r'def Xform "(\w+)" \(\n\s+prepend apiSchemas = \["PhysicsRigidBodyAPI"', kit)
        self.assertEqual(tuple(bodies), F.SURFACES)
        self.assertEqual(kit.count("bool physics:kinematicEnabled = 1"), len(F.SURFACES))
        self.assertNotIn("Joint", TEXT)                              # nothing is ever joined to anything

    def test_pads_where_the_layout_says(self):
        for zone in F.SURFACES:
            pad = F.ZONE_PAD[zone]
            (x, y, z), _ = F.pad_frame(pad)
            tx, ty, tz = translate(block(zone))
            self.assertAlmostEqual(tx, x, places=6)
            self.assertAlmostEqual(ty, y, places=6)
            self.assertAlmostEqual(tz, z + F.TOP, places=4)
            self.assertGreater(tz - F.TOP, .05)                      # above the table

    def test_layout_is_real_size_and_open_handed(self):
        self.assertAlmostEqual(2 * L.SNARE_HEAD_R, .254)
        self.assertAlmostEqual(2 * L.TOM_HEAD_R, .2032)
        self.assertAlmostEqual(2 * L.HIHAT_R, .254)
        (hx, hy), hz, _ = L.HIHAT
        (sx, sy), sz, _ = L.SNARE
        (tx, ty), tz, _ = L.TOM
        self.assertGreater(hy, sy)                                   # hi-hat left of the snare ...
        self.assertGreater(hz, sz)                                   # ... and higher
        self.assertLess(ty, sy)                                      # tom right of the snare ...
        self.assertLessEqual(tz, sz)                                 # ... no higher
        self.assertGreater(hy, 0.)                                   # in front of the left arm
        self.assertLess(sy, 0.)                                      # in front of the right arm
        gap_hs = math.hypot(hx - sx, hy - sy) - F.pad_outer_r("hihat") - F.pad_outer_r("snare")
        gap_st = math.hypot(sx - tx, sy - ty) - F.pad_outer_r("snare") - F.pad_outer_r("tom")
        self.assertAlmostEqual(gap_hs, .054, delta=.0006)
        self.assertAlmostEqual(gap_st, .0274, delta=.0006)
        self.assertAlmostEqual(ty - F.pad_outer_r("tom"), -.5336, delta=.0006)   # the tom's rim, right of the centre line

    def test_sticks(self):
        sticks = block(F.ACTORS)
        self.assertEqual(re.findall(r'def Xform "(\w+)" \(', sticks), list(F.STICKS.values()))
        self.assertEqual([float(v) for v in re.findall(r"float physics:mass = ([\d.e-]+)", sticks)], [L.STICK_MASS] * 2)
        offsets = {float(v) for v in re.findall(r"physxCollision:contactOffset = ([\d.e-]+)", sticks)}
        self.assertEqual(offsets, {L.CONTACT_OFFSET})
        self.assertEqual(sticks.count('def Capsule "shaft"'), 2)
        self.assertEqual(sticks.count('def Sphere "tip"'), 2)
        caps = re.findall(r'double height = ([\d.e-]+)\n\s+double radius = ([\d.e-]+)', sticks)
        self.assertIn((f"{L.TAPER_FROM_GRIP + L.GRIP_FROM_BUTT - L.STICK_R:g}", f"{L.STICK_R:g}"), caps)

    def test_materials(self):
        mats = block("Materials")
        hickory = block("hickory")
        self.assertIn(f"physics:staticFriction = {L.STICK_MU}", hickory)
        self.assertIn('frictionCombineMode = "max"', hickory)
        self.assertIn("physics:restitution = 0", hickory)
        self.assertIn('restitutionCombineMode = "min"', hickory)
        for name in ("mesh", "rubber", "hoop", "shell"):
            m = block(name)
            self.assertIn(f"physics:restitution = {L.PAD_RESTITUTION}", m)
            self.assertIn('restitutionCombineMode = "max"', m)
        self.assertNotIn("compliantContact", mats)

    def test_drawn_hardware_has_no_colliders(self):
        drawn = block("Drawn")
        self.assertNotIn("PhysicsCollisionAPI", drawn)
        for part in ("rack_tube", "c_clamp_left_frame", "c_clamp_right_frame", "hihat_arm_out", "snare_arm_out",
                     "tom_arm_out"):
            self.assertIn(f'"{part}"', drawn)


class StrokePoses(unittest.TestCase):
    def test_frozen_poses_put_the_tip_on_the_pad(self):
        try:
            import numpy as np  # noqa: F401
            from yam_arena import ik
        except ImportError:
            self.skipTest("needs numpy and yam_arena")
        from rdb_drum_groove import poses

        for (arm, pad), (st, margin) in poses.contact_poses().items():
            h, p = math.degrees(st.heading), math.degrees(st.pitch)
            want = poses.aim_point(pad, st.heading, st.pitch)
            self.assertLess(float(np.linalg.norm(st.contact_tip - want)), .002, (arm, pad))
            q = st.q_c
            self.assertGreaterEqual(math.degrees(float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())), poses.MARGIN_DEG)
            self.assertAlmostEqual(st.gain(st.phi_for(.08)), .08, delta=.001)
            side = 1. if arm == "left" else -1.
            self.assertGreater(side * ik.tcp_pose(q, st.base)[0][1], poses.HAND_Y - .001)
        start = {arm: np.asarray(F.START_Q[arm]) for arm in F.ARMS}
        cp = poses.contact_poses()
        for arm, pad in (("left", "hihat"), ("right", "snare")):
            st = cp[(arm, pad)][0]
            self.assertLess(float(np.abs(start[arm] - st.q_at(st.phi_for(F.START_TIP_LIFT))).max()), 1e-4)


if __name__ == "__main__":
    unittest.main()
