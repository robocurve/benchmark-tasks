"""Unit tests for geometry in weigh-and-reject-heavy-block."""
import math
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_block_specs(self):
        self.assertEqual(len(g.BLOCK_NAMES), 3)
        self.assertAlmostEqual(g.BLOCK_MASSES["block_0"], 0.10)
        self.assertAlmostEqual(g.BLOCK_MASSES["block_1"], 0.30)
        self.assertAlmostEqual(g.BLOCK_MASSES["block_2"], 0.10)
        self.assertEqual(g.HEAVY_INDEX, 1)

    def test_spring_and_sag(self):
        self.assertAlmostEqual(g.SCALE_SPRING_K, 600.0)
        empty_sag = (g.SCALE_PLATFORM_MASS * 9.81) / g.SCALE_SPRING_K
        self.assertAlmostEqual(g.SAG_PLATFORM_EMPTY, empty_sag, places=5)
        light_sag = (g.BLOCK_MASSES["block_0"] * 9.81) / g.SCALE_SPRING_K
        self.assertAlmostEqual(g.SAG_LIGHT_BLOCK, light_sag, places=5)
        heavy_sag = (g.BLOCK_MASSES["block_1"] * 9.81) / g.SCALE_SPRING_K
        self.assertAlmostEqual(g.SAG_HEAVY_BLOCK, heavy_sag, places=5)

        # Check meter gain and red band
        deflection_light = light_sag * g.READOUT_GAIN
        deflection_heavy = heavy_sag * g.READOUT_GAIN
        self.assertLess(deflection_light, g.READOUT_HEAVY_THRESHOLD_M)
        self.assertGreater(deflection_heavy, g.READOUT_HEAVY_THRESHOLD_M)

    def test_bimanual_relay_reach(self):
        left_base = (0.0, +0.305)
        right_base = (0.0, -0.305)
        max_reach = 0.50

        # Left arm cannot reach reject tray
        d_left_to_tray = math.hypot(g.TRAY_POS[0] - left_base[0], g.TRAY_POS[1] - left_base[1])
        self.assertGreater(d_left_to_tray, max_reach)

        # Right arm cannot reach supply rack
        for b_pos in g.BLOCK_SUPPLY_POS.values():
            d_right_to_supply = math.hypot(b_pos[0] - right_base[0], b_pos[1] - right_base[1])
            self.assertGreater(d_right_to_supply, max_reach)

        # Both arms can reach scale
        d_left_to_scale = math.hypot(g.SCALE_CENTER_XY[0] - left_base[0], g.SCALE_CENTER_XY[1] - left_base[1])
        d_right_to_scale = math.hypot(g.SCALE_CENTER_XY[0] - right_base[0], g.SCALE_CENTER_XY[1] - right_base[1])
        self.assertLess(d_left_to_scale, max_reach)
        self.assertLess(d_right_to_scale, max_reach)


if __name__ == "__main__":
    unittest.main()
