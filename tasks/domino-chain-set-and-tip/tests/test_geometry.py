"""Tests for geometry constants and station dimensions in domino-chain-set-and-tip."""

import math
import unittest
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_height(self):
        self.assertAlmostEqual(g.TOP, 0.7475, places=4)

    def test_marks_layout(self):
        self.assertEqual(len(g.MARK_POSITIONS), 4)
        for p in g.MARK_POSITIONS:
            self.assertEqual(len(p), 3)
            self.assertAlmostEqual(p[1], 0.000, places=3)
            self.assertAlmostEqual(p[2], g.TOP + 0.001, places=3)

        self.assertAlmostEqual(g.MARK_0_POS[0], 0.260, places=3)
        self.assertAlmostEqual(g.MARK_1_POS[0], 0.310, places=3)
        self.assertAlmostEqual(g.MARK_2_POS[0], 0.360, places=3)
        self.assertAlmostEqual(g.MARK_3_POS[0], 0.410, places=3)

        # 50 mm pitch, 30 mm gap
        self.assertAlmostEqual(g.MARK_PITCH, 0.050, places=3)
        self.assertAlmostEqual(g.MARK_GAP, 0.030, places=3)
        self.assertAlmostEqual(g.MARK_1_POS[0] - g.MARK_0_POS[0], 0.050, places=3)
        self.assertAlmostEqual(g.MARK_2_POS[0] - g.MARK_1_POS[0], 0.050, places=3)
        self.assertAlmostEqual(g.MARK_3_POS[0] - g.MARK_2_POS[0], 0.050, places=3)

    def test_domino_proportions_and_properties(self):
        self.assertEqual(g.DOMINO_SIZE, (0.020, 0.040, 0.070))
        self.assertAlmostEqual(g.DOMINO_THICKNESS, 0.020, places=4)
        self.assertAlmostEqual(g.DOMINO_WIDTH, 0.040, places=4)
        self.assertAlmostEqual(g.DOMINO_HEIGHT, 0.070, places=4)
        self.assertAlmostEqual(g.DOMINO_MASS, 0.020, places=4)
        self.assertAlmostEqual(g.DOMINO_FRICTION, 0.45, places=4)

        self.assertEqual(len(g.DOMINO_COLORS), 4)

    def test_start_positions_layout(self):
        self.assertEqual(len(g.DOMINO_START_POSITIONS), 4)
        # Two near left y = +0.20
        self.assertAlmostEqual(g.DOMINO_0_START_POS[1], 0.200, places=3)
        self.assertAlmostEqual(g.DOMINO_1_START_POS[1], 0.200, places=3)
        # Two near right y = -0.20
        self.assertAlmostEqual(g.DOMINO_2_START_POS[1], -0.200, places=3)
        self.assertAlmostEqual(g.DOMINO_3_START_POS[1], -0.200, places=3)

    def test_bimanual_reach_separation(self):
        # Right arm cannot reach dominoes on left staging (> 0.50 m)
        d_r_to_d0 = math.hypot(g.DOMINO_0_START_POS[0] - g.RIGHT_BASE[0], g.DOMINO_0_START_POS[1] - g.RIGHT_BASE[1])
        d_r_to_d1 = math.hypot(g.DOMINO_1_START_POS[0] - g.RIGHT_BASE[0], g.DOMINO_1_START_POS[1] - g.RIGHT_BASE[1])
        self.assertGreater(d_r_to_d0, g.MAX_ARM_REACH)
        self.assertGreater(d_r_to_d1, g.MAX_ARM_REACH)

        # Left arm cannot reach dominoes on right staging (> 0.50 m)
        d_l_to_d2 = math.hypot(g.DOMINO_2_START_POS[0] - g.LEFT_BASE[0], g.DOMINO_2_START_POS[1] - g.LEFT_BASE[1])
        d_l_to_d3 = math.hypot(g.DOMINO_3_START_POS[0] - g.LEFT_BASE[0], g.DOMINO_3_START_POS[1] - g.LEFT_BASE[1])
        self.assertGreater(d_l_to_d2, g.MAX_ARM_REACH)
        self.assertGreater(d_l_to_d3, g.MAX_ARM_REACH)

    def test_free_body_box_centres(self):
        # Lesson 1: box centres are local to body origin
        import task
        for obj in (task.DOMINO_0, task.DOMINO_1, task.DOMINO_2, task.DOMINO_3):
            for (bx, by, bz), sz, col in obj.boxes:
                self.assertLessEqual(abs(bx), sz[0] / 2 + 1e-4)
                self.assertLessEqual(abs(by), sz[1] / 2 + 1e-4)
                self.assertLessEqual(abs(bz), sz[2] / 2 + 1e-4)


if __name__ == "__main__":
    unittest.main()
