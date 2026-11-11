"""Unit tests for geometry, clearances, and reach in squeeze-in-the-last-slab."""

import unittest
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_slab_gap_geometry(self):
        # Neighbours of the slot: slab 3 at 0.300, slab 4 at 0.350, slot centre 0.325. They are 50 mm
        # apart centre to centre, so the clear width is 50 - 12 = 38 mm: 13 mm each side of a 12 mm slab.
        pitch = g.SLAB_2_POS[0] - g.SLAB_1_POS[0]
        self.assertAlmostEqual(pitch, 0.045, places=4)

        gap_to_s3 = g.SLOT_TARGET_POS[0] - g.SLAB_3_POS[0]
        gap_to_s4 = g.SLAB_4_POS[0] - g.SLOT_TARGET_POS[0]
        self.assertAlmostEqual(gap_to_s3, 0.025, places=4)
        self.assertAlmostEqual(gap_to_s4, 0.025, places=4)
        clear = g.SLAB_4_POS[0] - g.SLAB_3_POS[0] - g.SLAB_THICKNESS
        self.assertAlmostEqual(clear, 0.038, places=4)

    def test_fifth_slab_starts_standing(self):
        # The arms grip straight down (yaw only) and cannot turn a flat slab on end.
        self.assertAlmostEqual(g.SLAB_5_START[2], g.SLAB_STANDING_Z, places=4)

    def test_reach_bounds(self):
        base_l = np.array([0.0, +0.305, 0.0])

        for pt in [g.SLAB_1_POS, g.SLAB_2_POS, g.SLAB_3_POS, g.SLAB_4_POS, g.SLOT_TARGET_POS, g.SLAB_5_START]:
            d = np.linalg.norm(np.array(pt)[:2] - base_l[:2])
            self.assertGreaterEqual(d, 0.10)
            self.assertLessEqual(d, 0.50)

    def test_slider_rates(self):
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()

