"""Tests for geometry constants and station dimensions in hook-block-from-guarded-zone."""

import unittest
import math
import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_height(self):
        self.assertAlmostEqual(g.TOP, 0.7475, places=4)

    def test_yellow_line_layout(self):
        self.assertEqual(len(g.LINE_POS), 3)
        self.assertAlmostEqual(g.LINE_POS[0], 0.400, places=3)
        self.assertAlmostEqual(g.LINE_POS[1], 0.000, places=3)
        self.assertEqual(g.LINE_SIZE, (0.008, 0.400, 0.002))
        self.assertAlmostEqual(g.GUARDED_ZONE_X, 0.400, places=3)
        self.assertAlmostEqual(g.FAULT_BOUNDARY_X, 0.410, places=3)

    def test_boundary_helpers(self):
        self.assertFalse(g.in_guarded_zone((0.399, 0.0, 0.8)))
        self.assertTrue(g.in_guarded_zone((0.401, 0.0, 0.8)))

        self.assertFalse(g.is_boundary_violation((0.400, 0.0, 0.8)))
        self.assertFalse(g.is_boundary_violation((0.410, 0.0, 0.8)))
        self.assertTrue(g.is_boundary_violation((0.411, 0.0, 0.8)))

    def test_block_properties_and_reach(self):
        self.assertEqual(g.BLOCK_SIZE, (0.040, 0.040, 0.040))
        self.assertAlmostEqual(g.BLOCK_MASS, 0.050, places=3)
        self.assertAlmostEqual(g.BLOCK_FRICTION, 0.40, places=2)
        self.assertAlmostEqual(g.BLOCK_INIT_POS[0], 0.580, places=3)
        self.assertAlmostEqual(g.BLOCK_INIT_POS[1], 0.000, places=3)

        # Verify block is strictly beyond direct arm reach (> 0.50 m)
        d_left = math.hypot(g.BLOCK_INIT_POS[0] - g.LEFT_BASE[0], g.BLOCK_INIT_POS[1] - g.LEFT_BASE[1])
        d_right = math.hypot(g.BLOCK_INIT_POS[0] - g.RIGHT_BASE[0], g.BLOCK_INIT_POS[1] - g.RIGHT_BASE[1])
        self.assertGreater(d_left, 0.50, f"Block should be beyond left arm reach: {d_left:.3f} m")
        self.assertGreater(d_right, 0.50, f"Block should be beyond right arm reach: {d_right:.3f} m")

    def test_hooked_rod_dimensions_and_reach(self):
        self.assertEqual(g.ROD_SHAFT_SIZE, (0.300, 0.020, 0.020))
        self.assertEqual(g.ROD_HOOK_SIZE, (0.020, 0.060, 0.020))
        self.assertAlmostEqual(g.ROD_MASS, 0.060, places=3)
        self.assertAlmostEqual(g.ROD_FRICTION, 1.50, places=2)
        self.assertAlmostEqual(g.ROD_INIT_POS[0], 0.300, places=3)
        self.assertAlmostEqual(g.ROD_INIT_POS[1], 0.250, places=3)

        # Grasp offset 30 mm from handle end: the cross-piece front must clear the block back face (0.60)
        self.assertAlmostEqual(g.ROD_GRASP_OFFSET, 0.030, places=3)
        self.assertAlmostEqual(g.ROD_GRASP_START_POS[0], 0.180, places=3)
        tcp_x = 0.360
        cross_front = tcp_x + (0.150 - g.ROD_GRASP_OFFSET) + 0.140 - g.ROD_HOOK_SIZE[0] / 2
        self.assertGreater(cross_front, g.BLOCK_INIT_POS[0] + 0.020 + 0.005)
        self.assertLess(tcp_x, g.FAULT_BOUNDARY_X)

    def test_tray_dimensions_and_containment(self):
        self.assertEqual(g.TRAY_SIZE, (0.130, 0.130, 0.020))
        self.assertAlmostEqual(g.TRAY_POS[0], 0.280, places=3)
        self.assertAlmostEqual(g.TRAY_POS[1], -0.300, places=3)

        # Point inside tray
        in_pt = (0.280, -0.300, g.TRAY_SEAT_Z)
        self.assertTrue(g.in_tray(in_pt))

        # Point outside in X
        self.assertFalse(g.in_tray((0.350, -0.300, g.TRAY_SEAT_Z)))
        # Point outside in Y
        self.assertFalse(g.in_tray((0.280, -0.230, g.TRAY_SEAT_Z)))
        # Point too high
        self.assertFalse(g.in_tray((0.280, -0.300, g.TOP + 0.150)))

    def test_rod_boxes_construction(self):
        boxes = g.rod_local_boxes()
        self.assertEqual(len(boxes), 2)
        shaft_centre, shaft_size, _col = boxes[0]
        hook_centre, hook_size, _col2 = boxes[1]

        self.assertEqual(shaft_size, g.ROD_SHAFT_SIZE)
        self.assertEqual(hook_size, g.ROD_HOOK_SIZE)
        self.assertAlmostEqual(shaft_centre[0], 0.000)
        self.assertAlmostEqual(hook_centre[0], 0.140)


if __name__ == "__main__":
    unittest.main()
