"""Unit tests for geometry in hot-swap-failed-drive-sled."""
import math
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_bay_layout(self):
        self.assertEqual(len(g.BAY_Y), 3)
        self.assertEqual(g.FAILED_BAY, "bay_1")
        self.assertAlmostEqual(g.BAY_Y["bay_1"], 0.0)
        self.assertAlmostEqual(g.BAY_Y["bay_0"], 0.120)
        self.assertAlmostEqual(g.BAY_Y["bay_2"], -0.120)

    def test_dimensions(self):
        # Sled dimensions: 150 x 80 x 35 mm
        self.assertEqual(g.SLED_SIZE, (0.150, 0.080, 0.025))
        # Handle dimensions: 20 x 45 x 30 mm
        self.assertEqual(g.HANDLE_SIZE, (0.030, 0.045, 0.030))
        # Button: 40 x 40 x 12 mm
        self.assertEqual(g.BUTTON_SIZE, (0.040, 0.040, 0.012))
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)

    def test_bimanual_relay_reach(self):
        # Left arm cannot reach the discard pad on the right
        d_left_to_discard = math.hypot(g.DISCARD_SPOT[0] - g.LEFT_BASE[0], g.DISCARD_SPOT[1] - g.LEFT_BASE[1])
        self.assertGreater(d_left_to_discard, g.MAX_ARM_REACH)

        # Right arm cannot reach the spare stand on the left
        for sp in g.SPARE_POS.values():
            d_right_to_spare = math.hypot(sp[0] - g.RIGHT_BASE[0], sp[1] - g.RIGHT_BASE[1])
            self.assertGreater(d_right_to_spare, g.MAX_ARM_REACH)
