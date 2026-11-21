"""Offline checks (NumPy only) of the capping rule and the scorer.  python3 -m unittest discover -s tests -v"""

from __future__ import annotations

import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import capping  # noqa: E402
import layout as L  # noqa: E402
import scoring  # noqa: E402

MM = L.MM


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# The copper points along -x (out of the box toward the robot); its end at the origin. A nut capping it faces it:
# the nut's axis (mouth to closed end) is the copper's direction, -x: the nut frame is turned 180 deg about z.
TIP = np.zeros(3)
TIP_DIR = np.array([-1.0, 0.0, 0.0])
TIP_R = np.eye(3)
FACING = rot_z(math.pi)


def nut_pose(depth: float, turn: float = 0.0, radial: float = 0.0, tilt: float = 0.0):
    """The nut's (R, origin) with the copper's end ``depth`` into its bore, turned ``turn`` rad clockwise seen from
    its closed end, off its axis by ``radial`` and tilted by ``tilt``."""
    R = FACING @ rot_z(tilt) @ rot_x(-turn)  # clockwise from the closed end = right-handed about the nut's -x
    origin = TIP - R[:, 0] * depth + R[:, 1] * radial
    return R, origin


class EngagementTest(unittest.TestCase):
    def test_depth_radial_angle(self):
        e = capping.engagement(*nut_pose(10 * MM, radial=1 * MM), TIP, TIP_DIR)
        self.assertAlmostEqual(e.depth, 10 * MM, places=9)
        self.assertAlmostEqual(e.radial, 1 * MM, places=9)
        self.assertAlmostEqual(e.angle, 0.0, places=6)
        self.assertTrue(e.engaged)

    def test_shallow_offaxis_tilted_not_engaged(self):
        self.assertFalse(capping.engagement(*nut_pose(4 * MM), TIP, TIP_DIR).engaged)
        self.assertFalse(capping.engagement(*nut_pose(10 * MM, radial=3 * MM), TIP, TIP_DIR).engaged)
        self.assertFalse(capping.engagement(*nut_pose(10 * MM, tilt=math.radians(30)), TIP, TIP_DIR).engaged)

    def test_backwards_nut_not_engaged(self):
        R, o = nut_pose(10 * MM)
        R2 = R @ rot_z(math.pi)  # its closed end toward the copper
        e = capping.engagement(R2, o, TIP, TIP_DIR)
        self.assertFalse(e.engaged)


def twist(capper, name, depth, start, end, steps, radial=0.0, tilt=0.0):
    bit = False
    for k in range(steps + 1):
        a = start + (end - start) * k / steps
        R, o = nut_pose(depth, a, radial, tilt)
        bit = capper.step(name, R, o, TIP, TIP_DIR, TIP_R, k) or bit
    return bit


class CapperTest(unittest.TestCase):
    def test_two_clockwise_turns_bite(self):
        c = capping.Capper()
        self.assertFalse(twist(c, "n", 14 * MM, 0.0, math.radians(710), 71))
        self.assertTrue(twist(c, "n", 14 * MM, math.radians(710), math.radians(730), 4))
        self.assertTrue(c.nuts["n"].bit)

    def test_counterclockwise_does_not_bite(self):
        c = capping.Capper()
        self.assertFalse(twist(c, "n", 14 * MM, 0.0, -math.radians(720), 72))
        self.assertLess(c.nuts["n"].twist, 0.0)

    def test_regrips_add_up(self):
        """Eight quarter turns with the nut held still between them (a regrip) bite."""
        c = capping.Capper()
        bit = False
        for k in range(8):
            a0 = math.radians(90 * k)
            bit = twist(c, "n", 14 * MM, a0, a0 + math.radians(91), 10) or bit
            bit = twist(c, "n", 14 * MM, a0 + math.radians(91), a0 + math.radians(91), 5) or bit
        self.assertTrue(bit)

    def test_turning_outside_the_bore_does_not_count(self):
        c = capping.Capper()
        self.assertFalse(twist(c, "n", 4 * MM, 0.0, math.radians(720), 72))  # shallow: not engaged
        self.assertAlmostEqual(c.nuts["n"].twist, 0.0)

    def test_pulled_off_starts_again(self):
        c = capping.Capper()
        twist(c, "n", 14 * MM, 0.0, math.radians(300), 30)
        R, o = nut_pose(-5 * MM)  # off the copper
        c.step("n", R, o, TIP, TIP_DIR, TIP_R)
        self.assertEqual(c.nuts["n"].twist, 0.0)
        self.assertFalse(twist(c, "n", 14 * MM, 0.0, math.radians(300), 30))

    def test_big_steps_wrap_correctly(self):
        """Turns of up to 170 deg between control steps are read the right way round."""
        c = capping.Capper()
        self.assertTrue(twist(c, "n", 14 * MM, 0.0, math.radians(800), 5))

    def test_turn_angle_independent_of_tip_frame_roll(self):
        R, _ = nut_pose(14 * MM, 0.3)
        a = capping.turn_angle(R, np.eye(3))
        b = capping.turn_angle(rot_x(0.5) @ R, rot_x(0.5))
        self.assertAlmostEqual(a, b, places=9)

    def test_covers(self):
        self.assertTrue(capping.covers(10.5 * MM))
        self.assertFalse(capping.covers(10 * MM))


class ScoreTest(unittest.TestCase):
    def test_tiers(self):
        self.assertEqual(scoring.judge(True, True, True).score, 1.0)
        self.assertEqual(scoring.judge(True, True, False).score, 0.5)

    def test_a_splice_without_its_nut_scores_zero_whatever_else(self):
        for closed in (True, False):
            self.assertEqual(scoring.judge(False, True, closed).score, 0.0)
            self.assertEqual(scoring.judge(True, False, closed).score, 0.0)
            self.assertEqual(scoring.judge(False, False, closed).score, 0.0)

    def test_cover_closed_needs_latch_and_angle(self):
        self.assertTrue(scoring.cover_closed(L.CLOSED + 0.01, True))
        self.assertFalse(scoring.cover_closed(L.CLOSED + 0.01, False))
        self.assertFalse(scoring.cover_closed(L.CLOSED + 0.2, True))

    def test_every_outcome_scored(self):
        seen = {scoring.judge(*bits).score for bits in np.ndindex(2, 2, 2)}
        self.assertEqual(seen, {0.0, 0.5, 1.0})


if __name__ == "__main__":
    unittest.main()
