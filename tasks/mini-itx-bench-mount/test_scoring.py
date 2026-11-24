"""Checks the physical end-state gates, including orientation and release."""
import math
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
    def test_standard_pattern_and_pin_fit(self):
        self.assertAlmostEqual(g.HOLE_RADIUS * 2, .00396)
        self.assertAlmostEqual(g.PIN_RADIUS * 2, .0024)
        self.assertEqual(len(g.HOLES), 4)
        self.assertAlmostEqual(max(x for x, _ in g.HOLES) - min(x for x, _ in g.HOLES), .15494)
        self.assertAlmostEqual(max(y for _, y in g.HOLES) - min(y for _, y in g.HOLES), .15748)
        self.assertEqual(len({round(x, 5) for x, _ in g.HOLES}), 3)
        # The recessed F hole is 22.86 mm farther from the rear I/O edge than C.
        self.assertAlmostEqual(g.HOLES[0][0] - g.HOLES[1][0], .02286)
        self.assertGreater(g.HOLES[0][0], g.HOLES[2][0])

    def test_pcb_collision_shapes_leave_each_hole_open(self):
        for hx, hy in g.HOLES:
            nearest = min(math.hypot(
                max(abs(cx - hx) - sx/2, 0),
                max(abs(cy - hy) - sy/2, 0))
                for (cx, cy, _), (sx, sy, _), _ in g._pcb)
            self.assertGreaterEqual(nearest + 1e-9, g.HOLE_RADIUS)

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

    def test_round_holes_accept_small_diagonal_offset(self):
        p = list(g.BOARD_GOAL)
        p[0] += .0002
        p[1] += .0002
        self.assertEqual(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_pin_outside_its_hole_is_not_engaged(self):
        p = list(g.BOARD_GOAL)
        p[0] += .0011  # pin axis 1.1 mm off: past the 0.78 mm clearance and the 0.2 mm strip allowance
        self.assertLess(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_engaged_limit_is_the_clearance_plus_strip_allowance(self):
        self.assertAlmostEqual(scoring.ENGAGED, .00098)
        p = list(g.BOARD_GOAL)
        p[0] += .00097
        self.assertEqual(scoring.grade({"motherboard": board(pos=p)}, True), 1.)
        p[0] += .00002
        self.assertLess(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_seated_needs_every_standoff_within_the_feeler_gauge(self):
        p = list(g.BOARD_GOAL)
        p[2] += .0009
        self.assertEqual(scoring.grade({"motherboard": board(pos=p)}, True), 1.)
        p[2] += .0002  # 1.1 mm up: a 1.0 mm feeler gauge enters under every standoff
        self.assertEqual(scoring.grade({"motherboard": board(pos=p)}, True), .5)

    def test_round_fit_accepts_offset_inside_pin_clearance(self):
        p = list(g.BOARD_GOAL)
        p[0] += .00076
        self.assertEqual(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_round_fit_rejects_diagonal_offset(self):
        p = list(g.BOARD_GOAL)
        p[0] += .00075
        p[1] += .00075  # 1.06 mm off the pin axis
        self.assertLess(scoring.grade({"motherboard": board(pos=p)}, True), 1.)

    def test_tilt_lifts_one_hole_from_its_standoff(self):
        angle = np.deg2rad(2)
        tilted = (np.sin(angle/2), 0, 0, np.cos(angle/2))
        self.assertLess(scoring.grade({"motherboard": board(quat=tilted)}, True), 1.)

    def test_moving_board(self):
        self.assertLess(scoring.grade({"motherboard": board(still=False)}, True), 1.)

    def test_malformed_pose(self):
        obj = board()
        obj.quat_xyzw[3] = np.nan
        self.assertEqual(scoring.grade({"motherboard": obj}, True), 0.)


if __name__ == "__main__":
    unittest.main()
