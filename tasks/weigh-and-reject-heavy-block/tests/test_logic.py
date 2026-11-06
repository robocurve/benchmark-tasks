"""Unit tests for logic in weigh-and-reject-heavy-block."""
import sys
from pathlib import Path
import unittest
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.wl = logic.WeighLogic()

    def test_initial_state(self):
        st = self.wl.state
        self.assertFalse(st.all_three_weighed)
        self.assertFalse(st.heavy_in_tray)
        self.assertFalse(st.both_lights_on_shelf)
        self.assertFalse(st.wrong_rejection)
        self.assertFalse(st.block_dropped)

    def test_weighing_detection(self):
        dt = 0.05
        # Simulate light block 0 on scale, arms retracted
        block_positions = {
            "block_0": (g.SCALE_CENTER_XY[0], g.SCALE_CENTER_XY[1], g.SCALE_PLATFORM_TOP_REST_Z + g.BLOCK_SIZE[2] / 2),
            "block_1": g.BLOCK_SUPPLY_POS["block_1"],
            "block_2": g.BLOCK_SUPPLY_POS["block_2"],
        }
        arm_tcps = {
            "left": (0.0, 0.305, 0.8),
            "right": (0.0, -0.305, 0.8),
        }
        # Step for 1.1 seconds (22 steps) with light block sag
        sag = -(g.SAG_PLATFORM_EMPTY + g.SAG_LIGHT_BLOCK)
        st = None
        for _ in range(22):
            st = self.wl.step(dt, scale_disp=sag, block_positions=block_positions, arm_tcps=arm_tcps)
        self.assertTrue(st.block_weighed["block_0"])

    def test_wrong_rejection_detected(self):
        # Light block 0 placed in reject tray after being weighed
        self.wl.state.block_weighed["block_0"] = True
        block_positions = {
            "block_0": g.TRAY_SPOT,
            "block_1": g.BLOCK_SUPPLY_POS["block_1"],
            "block_2": g.BLOCK_SUPPLY_POS["block_2"],
        }
        st = self.wl.step(0.05, scale_disp=0.0, block_positions=block_positions)
        self.assertTrue(st.wrong_rejection)

    def test_dropped_block_detected(self):
        # Block falls below table
        block_positions = {
            "block_0": (0.3, 0.0, g.TOP - 0.10),
            "block_1": g.BLOCK_SUPPLY_POS["block_1"],
            "block_2": g.BLOCK_SUPPLY_POS["block_2"],
        }
        st = self.wl.step(0.05, scale_disp=0.0, block_positions=block_positions)
        self.assertTrue(st.block_dropped)


if __name__ == "__main__":
    unittest.main()
