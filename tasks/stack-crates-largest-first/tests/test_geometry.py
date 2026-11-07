"""Tests for geometry constants and layout in stack-crates-largest-first."""

import math
import unittest
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_dimensions(self):
        self.assertEqual(len(g.PALLET_SIZE), 3)
        self.assertEqual(len(g.LARGE_SIZE), 3)
        self.assertEqual(len(g.MEDIUM_SIZE), 3)
        self.assertEqual(len(g.SMALL_SIZE), 3)

        # Sizes must strictly decrease
        self.assertGreater(g.LARGE_SIZE[0], g.MEDIUM_SIZE[0])
        self.assertGreater(g.MEDIUM_SIZE[0], g.SMALL_SIZE[0])
        self.assertGreater(g.LARGE_SIZE[1], g.MEDIUM_SIZE[1])
        self.assertGreater(g.MEDIUM_SIZE[1], g.SMALL_SIZE[1])

    def test_gripper_span_clearance(self):
        # Large crate is 80 mm; gripper max span is 94 mm -> >= 14 mm clearance when open
        self.assertLess(g.LARGE_SIZE[0], 0.094)
        self.assertEqual(g.LARGE_SIZE[0], 0.080)

    def test_bimanual_reach_separation(self):
        # Right arm cannot reach Large crate start (> 0.50 m)
        d_right_to_large = math.hypot(g.LARGE_START_POS[0] - g.RIGHT_BASE[0], g.LARGE_START_POS[1] - g.RIGHT_BASE[1])
        self.assertGreater(d_right_to_large, g.MAX_ARM_REACH)

        # Left arm cannot reach Medium crate start (> 0.50 m)
        d_left_to_medium = math.hypot(g.MEDIUM_START_POS[0] - g.LEFT_BASE[0], g.MEDIUM_START_POS[1] - g.LEFT_BASE[1])
        self.assertGreater(d_left_to_medium, g.MAX_ARM_REACH)

    def test_free_body_box_centres(self):
        # Lesson 1: box centres are local to the body origin
        import task
        for obj in (task.CRATE_LARGE, task.CRATE_MEDIUM, task.CRATE_SMALL):
            for (bx, by, bz), sz, col in obj.boxes:
                # Local box centre must be within the body size bounds
                self.assertLessEqual(abs(bx), sz[0] / 2 + 1e-4)
                self.assertLessEqual(abs(by), sz[1] / 2 + 1e-4)
                self.assertLessEqual(abs(bz), sz[2] / 2 + 1e-4)


if __name__ == "__main__":
    unittest.main()

