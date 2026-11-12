"""Unit tests for geometry.py in segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import unittest
import geometry as g


class TestGeometry(unittest.TestCase):
    def test_clearance_meets_four_mm_rule(self):
        self.assertGreaterEqual(g.RADIAL_CLEARANCE, 0.004, "Recessed well radial clearance must be >= 4 mm")
        self.assertAlmostEqual(g.RADIAL_CLEARANCE, 0.006)

    def test_bottle_dimensions(self):
        self.assertAlmostEqual(g.BOTTLE_DIAMETER, 0.084)
        self.assertAlmostEqual(g.BOTTLE_RADIUS, 0.042)
        self.assertAlmostEqual(g.BOTTLE_HEIGHT, 0.210)
        self.assertAlmostEqual(g.BOTTLE_MASS, 0.42)

    def test_cabinet_wells(self):
        self.assertAlmostEqual(g.WELL_DIAMETER, 0.096)
        self.assertAlmostEqual(g.WELL_RADIUS, 0.048)
        self.assertAlmostEqual(g.WELL_DEPTH, 0.030)
        self.assertAlmostEqual(g.CABINET_X, 0.250)
        self.assertAlmostEqual(g.BAY_LEFT_Y, 0.130)
        self.assertAlmostEqual(g.BAY_RIGHT_Y, -0.130)
        self.assertAlmostEqual(g.BAY_DISTANCE, 0.260)

    def test_staging_positions(self):
        self.assertAlmostEqual(g.STAGING_X, 0.220)
        self.assertAlmostEqual(g.STAGING_LEFT_Y, 0.290)
        self.assertAlmostEqual(g.STAGING_RIGHT_Y, -0.290)

    def test_hold_and_violation_thresholds(self):
        self.assertAlmostEqual(g.HOLD_TIME_S, 2.0)
        self.assertAlmostEqual(g.MIN_BOTTLE_SEPARATION_VIOLATION, 0.120)
        self.assertAlmostEqual(g.MAX_TILT_PLACED_DEG, 6.0)
        self.assertAlmostEqual(g.MAX_TILT_SUCCESS_DEG, 5.0)


if __name__ == "__main__":
    unittest.main()
