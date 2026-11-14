"""Checks the physical end-state gates, including orientation and release."""
import unittest
from types import SimpleNamespace
import numpy as np
import geometry as g
import scoring


class Board(SimpleNamespace):
    def resting(self, *args):
        return self.still


def board(pos=g.BOARD_GOAL, quat=(0, 0, 0, 1), still=True):
    return Board(pos=np.array(pos, dtype=float), quat_xyzw=np.array(quat, dtype=float), still=still)


class ScoreTests(unittest.TestCase):
    def test_full_and_release(self):
        obj = {"motherboard": board()}
        self.assertEqual(scoring.grade(obj, True), 1.0)
        self.assertEqual(scoring.grade(obj, False), .75)

    def test_wrong_way_around(self):
        obj = {"motherboard": board(quat=(0, 0, 1, 0))}
        self.assertEqual(scoring.grade(obj, True), 0.0)

    def test_not_supported_by_standoffs(self):
        p = list(g.BOARD_GOAL)
        p[2] += .012
        self.assertLess(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_holes_misaligned(self):
        p = list(g.BOARD_GOAL)
        p[1] += .008
        self.assertLess(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_square_holes_accept_diagonal_pin_offset(self):
        p = list(g.BOARD_GOAL)
        p[0] += .0017
        p[1] += .0017
        self.assertEqual(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_pin_cannot_cross_square_hole_edge(self):
        p = list(g.BOARD_GOAL)
        p[0] += .0021
        self.assertLess(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_moving_board(self):
        self.assertLess(scoring.grade({"motherboard": board(still=False)}, True), 1.)

    def test_malformed_pose(self):
        obj = board()
        obj.quat_xyzw[3] = np.nan
        self.assertEqual(scoring.grade({"motherboard": obj}, True), 0.)


if __name__ == "__main__":
    unittest.main()
