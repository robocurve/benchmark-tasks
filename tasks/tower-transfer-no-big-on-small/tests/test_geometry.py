"""Tests for geometry constants and station dimensions in tower-transfer-no-big-on-small."""

import unittest
import math
import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_height(self):
        self.assertAlmostEqual(g.TOP, 0.7475, places=4)

    def test_marks_layout(self):
        self.assertEqual(len(g.LEFT_MARK_POS), 3)
        self.assertEqual(len(g.MID_MARK_POS), 3)
        self.assertEqual(len(g.RIGHT_MARK_POS), 3)

        self.assertAlmostEqual(g.LEFT_MARK_POS[0], 0.240, places=3)
        self.assertAlmostEqual(g.LEFT_MARK_POS[1], 0.090, places=3)
        self.assertAlmostEqual(g.MID_MARK_POS[0], 0.320, places=3)
        self.assertAlmostEqual(g.MID_MARK_POS[1], 0.000, places=3)
        self.assertAlmostEqual(g.RIGHT_MARK_POS[0], 0.240, places=3)
        self.assertAlmostEqual(g.RIGHT_MARK_POS[1], -0.090, places=3)

    def test_blocks_proportions(self):
        # Large > Medium > Small in footprint
        self.assertGreater(g.LARGE_SIZE[0], g.MEDIUM_SIZE[0])
        self.assertGreater(g.MEDIUM_SIZE[0], g.SMALL_SIZE[0])
        self.assertGreater(g.LARGE_MASS, g.MEDIUM_MASS)
        self.assertGreater(g.MEDIUM_MASS, g.SMALL_MASS)

        self.assertEqual(g.LARGE_SIZE, (0.080, 0.080, 0.025))
        self.assertEqual(g.MEDIUM_SIZE, (0.060, 0.060, 0.025))
        self.assertEqual(g.SMALL_SIZE, (0.045, 0.045, 0.025))

        self.assertAlmostEqual(g.LARGE_FRICTION, 1.2)
        self.assertAlmostEqual(g.MEDIUM_FRICTION, 1.2)
        self.assertAlmostEqual(g.SMALL_FRICTION, 1.2)

    def test_initial_stacked_positions(self):
        # All initially on LEFT mark
        self.assertAlmostEqual(g.LARGE_START_POS[0], g.LEFT_MARK_POS[0])
        self.assertAlmostEqual(g.LARGE_START_POS[1], g.LEFT_MARK_POS[1])
        self.assertAlmostEqual(g.MEDIUM_START_POS[0], g.LEFT_MARK_POS[0])
        self.assertAlmostEqual(g.MEDIUM_START_POS[1], g.LEFT_MARK_POS[1])
        self.assertAlmostEqual(g.SMALL_START_POS[0], g.LEFT_MARK_POS[0])
        self.assertAlmostEqual(g.SMALL_START_POS[1], g.LEFT_MARK_POS[1])

        # Large bottom, Med middle, Small top
        self.assertAlmostEqual(g.LARGE_START_POS[2], g.LEVEL_1_TARGET_Z)
        self.assertAlmostEqual(g.MEDIUM_START_POS[2], g.LEVEL_2_TARGET_Z)
        self.assertAlmostEqual(g.SMALL_START_POS[2], g.LEVEL_3_TARGET_Z)

        self.assertLess(g.LARGE_START_POS[2], g.MEDIUM_START_POS[2])
        self.assertLess(g.MEDIUM_START_POS[2], g.SMALL_START_POS[2])

    def test_grasp_constants(self):
        self.assertLess(g.FINGER_GRASP_LARGE, 0.0)
        self.assertLess(g.FINGER_GRASP_MED, 0.0)
        self.assertLess(g.FINGER_GRASP_SMALL, 0.0)
        self.assertLess(g.FINGER_OPEN_VAL, g.FINGER_GRASP_LARGE)
        self.assertLess(g.FINGER_GRASP_LARGE, g.FINGER_GRASP_MED)
        self.assertLess(g.FINGER_GRASP_MED, g.FINGER_GRASP_SMALL)


if __name__ == "__main__":
    unittest.main()
