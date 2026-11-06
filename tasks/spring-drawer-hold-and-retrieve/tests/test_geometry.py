"""Unit tests for geometry in spring-drawer-hold-and-retrieve."""
import math
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_dimensions(self):
        self.assertEqual(g.DRAWER_TRAVEL, 0.140)
        self.assertEqual(g.DRAWER_SPRING_K, 18.0)
        max_force = g.DRAWER_SPRING_K * g.DRAWER_TRAVEL
        self.assertLessEqual(max_force, 2.55)  # <= 2.5 N per spec

        # Handle >= 40 mm wide
        self.assertGreaterEqual(g.HANDLE_SIZE[1], 0.040)
        # Block 40 mm cube, 0.1 kg
        self.assertEqual(g.BLOCK_SIZE, (0.040, 0.040, 0.040))
        self.assertEqual(g.BLOCK_MASS, 0.100)

    def test_reach_separation(self):
        # Left arm cannot reach green shelf on right
        d_left_to_shelf = math.hypot(g.SHELF_SPOT[0] - g.LEFT_BASE[0], g.SHELF_SPOT[1] - g.LEFT_BASE[1])
        self.assertGreater(d_left_to_shelf, g.MAX_ARM_REACH)

        # Right arm reaches shelf
        d_right_to_shelf = math.hypot(g.SHELF_SPOT[0] - g.RIGHT_BASE[0], g.SHELF_SPOT[1] - g.RIGHT_BASE[1])
        self.assertLess(d_right_to_shelf, g.MAX_ARM_REACH)

        # Left arm reaches handle when closed and open
        d_left_to_closed_handle = math.hypot(g.HANDLE_REST_X - g.LEFT_BASE[0], 0.0 - g.LEFT_BASE[1])
        d_left_to_open_handle = math.hypot((g.HANDLE_REST_X - g.DRAWER_TRAVEL) - g.LEFT_BASE[0], 0.0 - g.LEFT_BASE[1])
        self.assertLess(d_left_to_closed_handle, g.MAX_ARM_REACH)
        self.assertLess(d_left_to_open_handle, g.MAX_ARM_REACH)


if __name__ == "__main__":
    unittest.main()

