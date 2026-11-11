"""Tests for geometry constants and station dimensions in bridge-plate-across-trench."""

import unittest
import math
import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_and_platform_height(self):
        self.assertAlmostEqual(g.TOP, 0.7475, places=4)
        self.assertAlmostEqual(g.PLATFORM_HEIGHT, 0.030, places=4)
        self.assertAlmostEqual(g.PLATFORM_TOP_Z, 0.7775, places=4)

    def test_platforms_and_trench_dimensions(self):
        # Platform dimensions: 120 (y) x 260 (x) x 30 (z) mm
        self.assertAlmostEqual(g.PLATFORM_X_SIZE, 0.260, places=3)
        self.assertAlmostEqual(g.PLATFORM_Y_SIZE, 0.120, places=3)
        self.assertEqual(g.PLATFORM_SIZE, (0.260, 0.120, 0.030))

        # Trench: 90 mm (y) at y=0 -> [-0.045, +0.045]
        self.assertAlmostEqual(g.TRENCH_WIDTH, 0.090, places=3)
        self.assertAlmostEqual(g.TRENCH_Y_MIN, -0.045, places=3)
        self.assertAlmostEqual(g.TRENCH_Y_MAX, 0.045, places=3)

        # Near platform: y in [0.045, 0.165]
        self.assertAlmostEqual(g.NEAR_PLATFORM_Y_MIN, 0.045, places=3)
        self.assertAlmostEqual(g.NEAR_PLATFORM_Y_MAX, 0.165, places=3)
        self.assertAlmostEqual(g.NEAR_PLATFORM_Y_MAX - g.NEAR_PLATFORM_Y_MIN, 0.120, places=3)

        # Far platform: y in [-0.165, -0.045]
        self.assertAlmostEqual(g.FAR_PLATFORM_Y_MIN, -0.165, places=3)
        self.assertAlmostEqual(g.FAR_PLATFORM_Y_MAX, -0.045, places=3)
        self.assertAlmostEqual(g.FAR_PLATFORM_Y_MAX - g.FAR_PLATFORM_Y_MIN, 0.120, places=3)

    def test_bridge_plate_dimensions_and_overlap_math(self):
        # Bridge plate: 80 (x) x 130 (y) x 6 mm
        self.assertEqual(g.BRIDGE_SIZE, (0.080, 0.150, 0.002))
        self.assertAlmostEqual(g.BRIDGE_MASS, 0.150, places=3)
        self.assertAlmostEqual(g.BRIDGE_FRICTION, 1.5, places=2)

        # Start position at (0.30, +0.20, TOP + 0.030 + 0.003)
        self.assertAlmostEqual(g.BRIDGE_START_POS[0], 0.250, places=3)
        self.assertAlmostEqual(g.BRIDGE_START_POS[1], 0.270, places=3)
        self.assertAlmostEqual(g.BRIDGE_START_POS[2], g.TOP + 0.030 + g.BRIDGE_SIZE[2] / 2, places=4)

        # When bridge plate (length 130 mm) is centered across trench at y=0:
        # y span is [-0.065, +0.065]
        h_y = g.BRIDGE_SIZE[1] / 2.0  # 0.065 m
        near_lip_overlap = (+h_y) - g.NEAR_PLATFORM_LIP_Y  # 0.075 - 0.045 = 0.030 m
        far_lip_overlap = g.FAR_PLATFORM_LIP_Y - (-h_y)    # -0.045 - (-0.075) = 0.030 m
        self.assertAlmostEqual(near_lip_overlap, 0.030, places=4)
        self.assertAlmostEqual(far_lip_overlap, 0.030, places=4)
        self.assertGreaterEqual(near_lip_overlap, g.BRIDGE_MIN_OVERLAP)
        self.assertGreaterEqual(far_lip_overlap, g.BRIDGE_MIN_OVERLAP)

    def test_block_and_target_mark(self):
        # Block: 40 x 40 x 30 mm, mass 0.05, friction 0.5
        self.assertEqual(g.BLOCK_SIZE, (0.040, 0.040, 0.030))
        self.assertAlmostEqual(g.BLOCK_MASS, 0.050, places=3)
        self.assertAlmostEqual(g.BLOCK_FRICTION, 0.3, places=2)

        # Start position at (0.25, +0.14, TOP + 0.030 + 0.015)
        self.assertAlmostEqual(g.BLOCK_START_POS[0], 0.250, places=3)
        self.assertAlmostEqual(g.BLOCK_START_POS[1], 0.140, places=3)
        self.assertAlmostEqual(g.BLOCK_START_POS[2], g.TOP + 0.030 + 0.015, places=4)

        # Target mark: 70 x 70 mm white mark on far platform at (0.30, -0.14, TOP + 0.030 + 0.001)
        self.assertEqual(g.MARK_SIZE, (0.070, 0.070, 0.002))
        self.assertAlmostEqual(g.TARGET_MARK_POS[0], 0.250, places=3)
        self.assertAlmostEqual(g.TARGET_MARK_POS[1], -0.140, places=3)
        self.assertAlmostEqual(g.TARGET_MARK_POS[2], g.TOP + 0.030 + 0.001, places=4)

    def test_tolerances(self):
        self.assertAlmostEqual(g.BRIDGE_MIN_OVERLAP, 0.020, places=4)
        self.assertAlmostEqual(g.BRIDGE_MAX_TILT_DEG, 3.0, places=2)
        self.assertAlmostEqual(g.TARGET_XY_TOL, 0.015, places=4)
        self.assertAlmostEqual(g.STILL_LIN_VEL, 0.005, places=4)
        self.assertAlmostEqual(g.HOLD_TIME_S, 2.0, places=2)
        self.assertAlmostEqual(g.ARMS_CLEAR_DIST, 0.060, places=4)
        self.assertAlmostEqual(g.FELL_Z_THRESHOLD, g.TOP + 0.030 - 0.020, places=4)


if __name__ == "__main__":
    unittest.main()
