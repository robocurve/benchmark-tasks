"""Offline checks of task.py's magnet and detent rules (harness Python, no simulator): python -m unittest test_task"""

from __future__ import annotations

import math
import pathlib
import sys
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts as P  # noqa: E402
import task  # noqa: E402

I3 = np.eye(3)
TOP = np.array([0.0, 0.0, P.CELL_BODY_H / 2])  # a cell at the origin: its + face's centre


def tilt_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


class TestMagnet(unittest.TestCase):
    def hold(self):
        return task.MagnetHold(P.CELL_MASS, 1e-8)

    def step(self, h, cp, tip, mR=I3, cR=I3, cv=np.zeros(3), mv=np.zeros(3)):
        return h.step(np.asarray(cp, float), cR, cv, np.zeros(3), np.asarray(tip, float), mR, mv, np.zeros(3))

    def test_takes_a_cell_just_under_it(self):
        h = self.hold()
        f, _ = self.step(h, [0, 0, 0], TOP + [0, 0, 0.0004])
        self.assertTrue(h.held)
        self.assertGreater(f[2], 0)
        self.assertLessEqual(np.linalg.norm(f), task.MAG_HOLD + 1e-9)

    def test_ignores_a_cell_further_off(self):
        h = self.hold()
        f, _ = self.step(h, [0, 0, 0], TOP + [0, 0, 0.002])
        self.assertFalse(h.held)
        self.assertEqual(np.linalg.norm(f), 0.0)

    def test_needs_most_of_the_face(self):
        h = self.hold()
        self.step(h, [0, 0, 0], TOP + [P.CELL_R + 1e-3, 0, 0.0002])
        self.assertFalse(h.held)

    def test_carries_its_weight(self):
        h = self.hold()
        self.step(h, [0, 0, 0], TOP)  # taken face to face
        f, _ = self.step(h, [0, 0, 0], TOP)  # in place: only its weight
        self.assertAlmostEqual(f[2], P.CELL_MASS * 9.81, places=6)

    def test_lets_go_when_a_wall_stops_it(self):
        h = self.hold()
        self.step(h, [0, 0, 0], TOP)
        for k in range(1, 9):  # the tip slides on 1 mm a step while the cell stays
            self.step(h, [0, 0, 0], TOP + [0.001 * k, 0, 0])
            if not h.held:
                break
        self.assertFalse(h.held)
        self.assertTrue(h.locked)
        self.step(h, [0, 0, 0], TOP + [0.0005, 0, 0])  # back over it at once: not taken again
        self.assertFalse(h.held)
        self.step(h, [0, 0, 0], TOP + [0.0, 0, 0.004])  # the tip has left it
        self.step(h, [0, 0, 0], TOP + [0.0, 0, 0.0003])
        self.assertTrue(h.held)

    def test_cap_side(self):
        flipped = np.diag([1.0, -1.0, -1.0])
        cap = np.array([0, 0, P.CELL_BODY_H / 2 + P.CAP_H])
        h = self.hold()
        self.step(h, [0, 0, 0], cap + [0, 0, 0.0003], cR=flipped)
        self.assertTrue(h.held)

    def test_needs_to_face(self):
        h = self.hold()
        self.step(h, [0, 0, 0], TOP, mR=tilt_x(90))
        self.assertFalse(h.held)


class TestDetent(unittest.TestCase):
    def run_angles(self, angles):
        tr = task.Trial()
        seen = []
        with mock.patch.object(task.bk, "set_hinge_drive", lambda *a: None):
            for d in angles:
                tr.detent(d)
                seen.append(tr.latched)
        return seen

    def test_opens_past_3(self):
        self.assertEqual(self.run_angles([0, 2.5, 3.5, 20])[-1], False)

    def test_clicks_under_2(self):
        self.assertEqual(self.run_angles([0, 4, 40, 5, 2.5, 1.5]), [True, False, False, False, False, True])

    def test_ramp_pushes_open(self):
        tr = task.Trial()
        with mock.patch.object(task.bk, "set_hinge_drive", lambda *a: None):
            tr.detent(4.0)
            tr.detent(4.0)
        self.assertEqual(tr.drive[0], task.RAMP_DEG)


if __name__ == "__main__":
    unittest.main()
