"""Offline checks of the scorer (rules.py) on hand-built trials, without the simulator: python -m unittest test_rules"""

from __future__ import annotations

import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import rules  # noqa: E402

MM = 0.001
Z0 = 0.7505
CUP = (0.28, -0.065)
PIVOT_W = np.array([0.3465, 0.027, Z0])
CRADLE_REL = (-6.5 * MM, 6.1 * MM, 5.55 * MM)


def rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])


def geom():
    aid_pts = np.mgrid[-1:1.01:0.5, -1:1.01:0.25, 0:1.01:0.5].reshape(3, -1).T * np.array([6.5 * MM, 18 * MM, 11 * MM])
    return rules.Geometry(z0=Z0, top=0.7475, mat_c=(0.30, -0.01), mat_size=(0.26, 0.30), cup_xy=CUP, cup_r=0.020, cup_h=0.030,
                          cradle_rel=CRADLE_REL, cell_r=3.95 * MM, cell_body_h=4.7 * MM, cap_h=0.7 * MM,
                          tab_rel=(0, 0, 2.5 * MM), aid_pts=aid_pts, door_pts=np.zeros((1, 3)), stick_pts=np.array([[0, 0, z] for z in np.linspace(0, 0.08, 9)]))


class World:
    """A trial state built by hand: every part's pose, the door's angle and latch, and the grippers' boxes."""

    def __init__(self):
        self.door = 0.0
        self.latched = True
        self.p = {
            "aid": rules.Pose(np.array([0.34, 0.045, Z0]), np.eye(3)),
            "old_cell": None, "new_cell": None, "tab": None,
            "magnet": rules.Pose(np.array([0.215, -0.12, Z0 + 0.005]), np.eye(3)),
        }
        self.flip = False
        self.old_in_door()
        self.new_in_pack()
        self.boxes = []  # no gripper near anything

    def door_pose(self):
        return rules.Pose(PIVOT_W.copy(), rz(self.door))

    def seat(self):
        d = self.door_pose()
        return d.world(np.asarray(CRADLE_REL)[None])[0]

    def old_in_door(self):
        self.p["old_cell"] = rules.Pose(self.seat(), rz(self.door))

    def new_in_pack(self):
        c = np.array([0.28, 0.02, Z0 + 0.006])
        self.p["new_cell"] = rules.Pose(c, np.eye(3))
        self.p["tab"] = rules.Pose(c + np.array([0, 0, 2.5 * MM]), np.eye(3))

    def to_cup(self, name, dz=0.003):
        self.p[name] = rules.Pose(np.array([CUP[0], CUP[1], Z0 + dz]), np.eye(3))

    def new_in_door(self, upside_down=False):
        R = rz(self.door) @ (np.diag([1.0, -1.0, -1.0]) if upside_down else np.eye(3))
        self.p["new_cell"] = rules.Pose(self.seat(), R)

    def hold_aid(self):
        self.boxes = [rules.Box(np.eye(3), np.array([0.34, 0.05, Z0 + 0.015]), np.array([-0.01, -0.02, -0.013]), np.array([0.01, 0.02, 0.03]), True)]

    def snap(self, t):
        pose = dict(self.p)
        pose["door"] = self.door_pose()
        return rules.Snapshot(t, pose, self.door, self.latched, list(self.boxes))


def run(steps):
    """steps: list of (t, World). Returns the final tier and the judge."""
    j = rules.Judge(geom())
    tier = (0.0, "")
    for t, w in steps:
        s = w.snap(t)
        j.update(s)
        tier = j.tier(s)
    return tier, j


def trial(finish, hold=1.2):
    """A whole trial: the door opened, old cell to the cup, tab to the cup, new cell seated, then `finish(w)` sets the end."""
    w = World()
    seq = [(0.05, _copy(w))]
    w.hold_aid()
    w.door, w.latched = 90.0, False
    w.old_in_door()
    seq.append((1.0, _copy(w)))
    w.to_cup("old_cell")
    seq.append((2.0, _copy(w)))
    w.to_cup("tab", dz=0.0015)
    seq.append((3.0, _copy(w)))
    w.new_in_door()
    seq.append((4.0, _copy(w)))
    finish(w)
    t = 5.0
    while t < 5.0 + hold:
        seq.append((t, _copy(w)))
        t += 0.05
    return run(seq)


def _copy(w):
    import copy

    return copy.deepcopy(w)


def shut(w):
    w.door, w.latched = 0.0, True
    w.new_in_door()
    w.boxes = []


class TestTiers(unittest.TestCase):
    def test_full(self):
        (score, why), _ = trial(shut)
        self.assertEqual(score, 1.0, why)

    def test_needs_the_hold(self):
        (score, _), _ = trial(shut, hold=0.5)
        self.assertEqual(score, 0.5)

    def test_not_clicked(self):
        def f(w):
            w.door, w.latched = 8.0, False
            w.new_in_door()
            w.boxes = []
        (score, why), _ = trial(f)
        self.assertEqual(score, 0.5, why)

    def test_left_open(self):
        def f(w):
            w.boxes = []
        (score, why), _ = trial(f)
        self.assertEqual(score, 0.25, why)

    def test_still_held(self):
        def f(w):
            shut(w)
            w.hold_aid()
        (score, why), _ = trial(f)
        self.assertEqual(score, 0.0, why)

    def test_upside_down(self):
        def f(w):
            shut(w)
            w.new_in_door(upside_down=True)
        (score, why), _ = trial(f)
        self.assertEqual(score, 0.0, why)
        self.assertIn("upside down", why)

    def test_tab_left_on(self):
        w = World()
        seq = [(0.05, _copy(w))]
        w.door, w.latched = 90.0, False
        seq.append((1.0, _copy(w)))
        w.to_cup("old_cell")
        seq.append((2.0, _copy(w)))
        w.new_in_door()
        w.p["tab"] = rules.Pose(w.p["new_cell"].world(np.array([[0, 0, 2.5 * MM]]))[0], w.p["new_cell"].R)
        seq.append((3.0, _copy(w)))
        w.door, w.latched = 0.0, True
        w.new_in_door()
        w.p["tab"] = rules.Pose(w.p["new_cell"].world(np.array([[0, 0, 2.5 * MM]]))[0], w.p["new_cell"].R)
        for k in range(30):
            seq.append((4.0 + 0.05 * k, _copy(w)))
        (score, why), _ = run(seq)
        self.assertEqual(score, 0.0, why)
        self.assertIn("tab on", why)

    def test_dropped(self):
        def f(w):
            shut(w)
        w = World()
        seq = [(0.05, _copy(w))]
        w.p["new_cell"] = rules.Pose(np.array([0.33, 0.0, Z0 + 2.35 * MM + 0.7 * MM]), np.eye(3))  # lying on the mat
        seq.append((1.0, _copy(w)))
        w.door = 90.0
        w.latched = False
        w.to_cup("old_cell")
        w.to_cup("tab", dz=0.0015)
        seq.append((2.0, _copy(w)))
        shut(w)
        for k in range(30):
            seq.append((3.0 + 0.05 * k, _copy(w)))
        (score, why), j = run(seq)
        self.assertEqual(score, 0.0, why)
        self.assertIn("new_cell", j.dropped)

    def test_finger_touch(self):
        w = World()
        seq = [(0.05, _copy(w))]
        c = w.p["new_cell"].p
        w.boxes = [rules.Box(np.eye(3), c + np.array([0, 0.0042, 0]), np.array([-0.01, 0, -0.01]), np.array([0.01, 0.002, 0.01]), True)]
        seq.append((1.0, _copy(w)))
        (score, why), j = run(seq)
        self.assertEqual(j.touched, ["new_cell"])

    def test_body_box_is_not_a_touch(self):
        w = World()
        c = w.p["new_cell"].p
        w.boxes = [rules.Box(np.eye(3), c + np.array([0, 0.0042, 0]), np.array([-0.01, 0, -0.01]), np.array([0.01, 0.002, 0.01]), False)]
        (score, why), j = run([(0.05, w)])
        self.assertEqual(j.touched, [])

    def test_door_opened_old_in_cup(self):
        w = World()
        seq = [(0.05, _copy(w))]
        w.door, w.latched = 90.0, False
        w.old_in_door()
        seq.append((1.0, _copy(w)))
        w.to_cup("old_cell")
        seq.append((2.0, _copy(w)))
        (score, why), _ = run(seq)
        self.assertEqual(score, 0.1, why)

    def test_nothing(self):
        (score, why), _ = run([(0.05, World())])
        self.assertEqual(score, 0.0)

    def test_aid_off_mat(self):
        def f(w):
            shut(w)
            w.p["aid"] = rules.Pose(np.array([0.50, 0.045, Z0]), np.eye(3))
        (score, why), _ = trial(f)
        self.assertEqual(score, 0.0, why)


if __name__ == "__main__":
    unittest.main()
