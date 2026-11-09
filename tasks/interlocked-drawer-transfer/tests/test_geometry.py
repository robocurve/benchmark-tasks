"""Tests for geometry constants and station layout in interlocked-drawer-transfer."""

import math
import unittest

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_and_cabinet_extents(self):
        self.assertEqual(g.TOP, 0.7475)
        self.assertAlmostEqual(g.CABINET_FRONT_X, 0.290)
        self.assertAlmostEqual(g.CABINET_BACK_X, 0.440)
        self.assertAlmostEqual(g.CABINET_SIZE_X, 0.150)
        self.assertAlmostEqual(g.CABINET_WIDTH, 0.400)
        self.assertAlmostEqual(g.CABINET_HEIGHT, 0.080)
        self.assertAlmostEqual(g.CABINET_TOP_Z, g.TOP + g.CABINET_HEIGHT)

    def test_drawer_dimensions_and_travel(self):
        self.assertEqual(g.DRAWER_INNER_X, 0.130)
        self.assertEqual(g.DRAWER_INNER_Y, 0.110)
        self.assertEqual(g.DRAWER_INNER_Z, 0.060)
        self.assertEqual(g.DRAWER_TRAVEL, 0.110)
        self.assertLessEqual(g.DRAWER_SPRING_K, 0.02)
        self.assertGreaterEqual(g.DRAWER_DAMPING, 10.0)
        self.assertEqual(g.DRAWER_MASS, 0.350)
        self.assertEqual(g.DRAWER_LEFT_Y, +0.120)
        self.assertEqual(g.DRAWER_RIGHT_Y, -0.120)

    def test_handles_and_block(self):
        self.assertEqual(g.HANDLE_SIZE, (0.020, 0.045, 0.030))
        self.assertAlmostEqual(g.HANDLE_REST_X, g.CABINET_FRONT_X - 0.010)
        self.assertEqual(g.BLOCK_SIZE, (0.040, 0.040, 0.040))
        self.assertEqual(g.BLOCK_MASS, 0.050)
        self.assertEqual(g.BLOCK_FRICTION, 1.5)
        self.assertEqual(g.BLOCK_START_POS[1], g.DRAWER_LEFT_Y)

    def test_staging_pad_and_lamps(self):
        self.assertEqual(g.STAGING_PAD_SIZE, (0.060, 0.060, 0.004))
        self.assertEqual(g.STAGING_PAD_POS[0], 0.310)
        self.assertEqual(g.STAGING_PAD_POS[1], 0.000)
        self.assertEqual(g.LAMP_OK_POS[0], 0.420)
        self.assertEqual(g.LAMP_TIP_POS[0], 0.420)
        self.assertGreater(g.LAMP_OK_POS[1], 0.0)
        self.assertLess(g.LAMP_TIP_POS[1], 0.0)

    def test_thresholds(self):
        self.assertEqual(g.DRAWER_OPEN_THRESHOLD, 0.015)
        self.assertEqual(g.DRAWER_CLOSED_THRESHOLD, 0.005)
        self.assertEqual(g.DRAWER_OPENED_GOAL, 0.070)
        self.assertEqual(g.TIP_FAULT_TIME, 0.200)
        self.assertEqual(g.SUCCESS_HOLD_S, 1.0)

    def test_arm_reachability(self):
        # Left arm reaches left handle at rest and pulled open
        d_l_handle_closed = math.hypot(g.HANDLE_REST_X - g.LEFT_BASE[0], g.DRAWER_LEFT_Y - g.LEFT_BASE[1])
        self.assertLess(d_l_handle_closed, g.MAX_ARM_REACH)

        d_l_handle_open = math.hypot(g.HANDLE_REST_X - g.DRAWER_TRAVEL - g.LEFT_BASE[0], g.DRAWER_LEFT_Y - g.LEFT_BASE[1])
        self.assertLess(d_l_handle_open, g.MAX_ARM_REACH)

        # Right arm reaches right handle at rest and pulled open
        d_r_handle_closed = math.hypot(g.HANDLE_REST_X - g.RIGHT_BASE[0], g.DRAWER_RIGHT_Y - g.RIGHT_BASE[1])
        self.assertLess(d_r_handle_closed, g.MAX_ARM_REACH)

        # Both arms reach staging pad
        d_l_pad = math.hypot(g.STAGING_PAD_POS[0] - g.LEFT_BASE[0], g.STAGING_PAD_POS[1] - g.LEFT_BASE[1])
        d_r_pad = math.hypot(g.STAGING_PAD_POS[0] - g.RIGHT_BASE[0], g.STAGING_PAD_POS[1] - g.RIGHT_BASE[1])
        self.assertLess(d_l_pad, g.MAX_ARM_REACH)
        self.assertLess(d_r_pad, g.MAX_ARM_REACH)

    def test_bimanual_separation(self):
        """Cross reach: Left arm cannot reach into right drawer when closed."""
        d_l_to_r_drawer = math.hypot(g.CABINET_FRONT_X - g.LEFT_BASE[0], g.DRAWER_RIGHT_Y - g.LEFT_BASE[1])
        self.assertGreater(d_l_to_r_drawer, g.MAX_ARM_REACH)

    def test_boxes_positive_extents(self):
        for box in g.cabinet_world_boxes():
            centre, size, colour = box
            self.assertEqual(len(centre), 3)
            self.assertEqual(len(size), 3)
            self.assertTrue(all(s > 0 for s in size))
            self.assertEqual(len(colour), 3)

        for box in g.drawer_world_boxes(g.DRAWER_LEFT_Y):
            centre, size, colour = box
            self.assertEqual(len(centre), 3)
            self.assertEqual(len(size), 3)
            self.assertTrue(all(s > 0 for s in size))
            self.assertEqual(len(colour), 3)


if __name__ == "__main__":
    unittest.main()
