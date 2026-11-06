"""Unit tests for geometry constants and fixture layout in two-key-authorization-hold."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestTwoKeyGeometry(unittest.TestCase):
    def test_fixture_dimensions(self):
        self.assertGreaterEqual(g.FIXTURE_MASS, 5.0)
        self.assertGreater(g.BASE_SIZE[0], 0.15)
        self.assertGreaterEqual(g.BASE_SIZE[1], 0.56)  # spans at least 56 cm between keys
        self.assertGreaterEqual(g.BASE_SIZE[2], 0.015)
        self.assertGreater(g.BASE_POS[2], g.TOP)

    def test_button_spacing_and_reach(self):
        pos_l = np.array(g.BUTTON_POS_XY["key_left"])
        pos_r = np.array(g.BUTTON_POS_XY["key_right"])
        dist = float(np.linalg.norm(pos_l - pos_r))
        self.assertGreaterEqual(dist, 0.50, f"Key distance {dist} m is too small for bimanual requirement")
        self.assertLessEqual(dist, 0.65)

        base_l = np.array([0.0, +0.305])
        base_r = np.array([0.0, -0.305])
        reach_l = float(np.linalg.norm(pos_l - base_l))
        reach_r = float(np.linalg.norm(pos_r - base_r))
        self.assertLessEqual(reach_l, 0.45)
        self.assertLessEqual(reach_r, 0.45)

    def test_guards_dimensions_and_travel(self):
        self.assertGreaterEqual(g.GUARD_TRAVEL, 0.060)
        self.assertEqual(g.GUARD_OPEN_THRESHOLD, 0.060)
        self.assertGreaterEqual(g.GUARD_SIZE[0], g.BUTTON_CAP_SIZE[0])
        self.assertGreaterEqual(g.GUARD_SIZE[1], g.BUTTON_CAP_SIZE[1])
        self.assertGreater(g.GUARD_REST_Z, g.BUTTON_REST_TOP_Z)

    def test_lamps(self):
        self.assertEqual(len(g.PROGRESS_LAMP_Y), 4)
        self.assertGreaterEqual(g.LAMP_DIAMETER_PROGRESS, 0.020)
        self.assertGreaterEqual(g.LAMP_DIAMETER_STATUS, 0.025)
        self.assertGreater(g.LAMP_ON_Z, g.TOP)
        self.assertLess(g.LAMP_OFF_Z, g.TOP)


if __name__ == "__main__":
    unittest.main()
