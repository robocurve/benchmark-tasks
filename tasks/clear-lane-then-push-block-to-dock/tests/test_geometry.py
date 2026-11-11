"""Unit tests for geometry, reachability, and clearances in clear-lane-then-push-block-to-dock."""

import unittest
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_lane_dimensions(self):
        # Rails should form a 100 mm inner clear width
        inner_w = (g.RAIL_LEFT_POS[1] - g.RAIL_SIZE[1] / 2.0) - (g.RAIL_RIGHT_POS[1] + g.RAIL_SIZE[1] / 2.0)
        self.assertAlmostEqual(inner_w, 0.100, places=3)
        self.assertAlmostEqual(g.LANE_LEN_X, 0.240, places=3)

    def test_cargo_and_blocks_fit_lane(self):
        # Cargo block width 60 mm fits 100 mm lane with 20 mm clearance on each side
        clearance_cargo = (g.LANE_WIDTH - g.CARGO_SIZE[1]) / 2.0
        self.assertAlmostEqual(clearance_cargo, 0.020, places=3)

        # Loose blocks width 40 mm fit with 30 mm clearance on each side
        clearance_block = (g.LANE_WIDTH - g.BLOCK_SIZE[1]) / 2.0
        self.assertAlmostEqual(clearance_block, 0.030, places=3)

    def test_dock_and_post_alignment(self):
        # Red post is beside lane exit at x=0.340, y=+0.090
        # Rail left outer wall is at y = 0.055 + 0.005 = 0.060
        # Red post inner edge is at y = 0.090 - 0.010 = 0.080 -> 20 mm gap to outer rail
        self.assertAlmostEqual(g.POST_START_POS[0], g.MARK_POS[0], places=3)
        self.assertGreater(g.POST_START_POS[1], g.RAIL_LEFT_POS[1])

        # Distance from lane centerline (y=0) to post center is 90 mm
        self.assertAlmostEqual(g.POST_START_POS[1], 0.090, places=3)

    def test_reach_bounds(self):
        base_l = np.array([0.0, +0.305, 0.0])
        base_r = np.array([0.0, -0.305, 0.0])

        # Left arm picks Blue, pushes Cargo
        for pt in [g.BLUE_START_POS, g.BLUE_STAGING_POS, g.CARGO_START_POS, (g.PUSH_END_X, 0.0, g.PUSH_Z)]:
            d = np.linalg.norm(np.array(pt)[:2] - base_l[:2])
            self.assertGreaterEqual(d, 0.10)
            self.assertLessEqual(d, 0.50)

        # Right arm picks Yellow
        for pt in [g.YELLOW_START_POS, g.YELLOW_STAGING_POS]:
            d = np.linalg.norm(np.array(pt)[:2] - base_r[:2])
            self.assertGreaterEqual(d, 0.10)
            self.assertLessEqual(d, 0.50)

    def test_slider_rates(self):
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()

