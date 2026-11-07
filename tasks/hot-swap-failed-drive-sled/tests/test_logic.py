"""Unit tests for logic in hot-swap-failed-drive-sled."""
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.hl = logic.HotSwapLogic()
        self.default_latch = {"bay_0": 0.0, "bay_1": 0.0, "bay_2": 0.0}
        self.default_sleds = {
            bay: (g.SLED_SEATED_X, g.BAY_Y[bay], g.SLED_REST_Z)
            for bay in ("bay_0", "bay_1", "bay_2")
        }
        self.default_spares = {
            color: g.SPARE_POS[color]
            for color in ("blue", "orange")
        }

    def test_initial_state(self):
        st = self.hl.state
        self.assertFalse(st.pulled_60mm)
        self.assertFalse(st.old_sled_on_discard)
        self.assertFalse(st.spare_inserted_95pct)
        self.assertFalse(st.lamp_green)
        self.assertFalse(st.fault)

    def test_forced_pull_without_latch_fault(self):
        # Pull failed sled 10 mm forward without latch pressed (latch = 0.0)
        sleds = dict(self.default_sleds)
        sleds[g.FAILED_BAY] = (g.SLED_SEATED_X - 0.010, 0.0, g.SLED_REST_Z)
        st = self.hl.step(0.05, self.default_latch, sleds, self.default_spares)
        self.assertTrue(st.fault)
        self.assertEqual(st.fault_reason, "forced_pull_no_latch")

    def test_valid_pull_with_latch(self):
        # Pull failed sled with latch pressed (latch = -0.007)
        latch = {"bay_0": 0.0, "bay_1": -0.007, "bay_2": 0.0}
        sleds = dict(self.default_sleds)
        sleds[g.FAILED_BAY] = (g.SLED_SEATED_X - 0.065, 0.0, g.SLED_REST_Z)
        st = self.hl.step(0.05, latch, sleds, self.default_spares)
        self.assertFalse(st.fault)
        self.assertTrue(st.pulled_60mm)

    def test_healthy_bay_moved_fault(self):
        # Move bay_0 sled by 10 mm
        sleds = dict(self.default_sleds)
        sleds["bay_0"] = (g.SLED_SEATED_X - 0.010, g.BAY_Y["bay_0"], g.SLED_REST_Z)
        st = self.hl.step(0.05, self.default_latch, sleds, self.default_spares)
        self.assertTrue(st.fault)
        self.assertIn("healthy", st.fault_reason)

    def test_keying_blue_vs_orange(self):
        # Place failed sled on discard pad
        sleds = dict(self.default_sleds)
        sleds[g.FAILED_BAY] = g.DISCARD_SPOT

        # Blue spare seated >= 95% with latch released
        spares_blue = dict(self.default_spares)
        spares_blue["blue"] = (g.SLED_SEATED_X, g.BAY_Y[g.FAILED_BAY], g.SLED_REST_Z)
        hl_blue = logic.HotSwapLogic()
        st_blue = hl_blue.step(0.05, self.default_latch, sleds, spares_blue)
        self.assertTrue(st_blue.spare_inserted_95pct)
        self.assertTrue(st_blue.blue_spare_used)
        self.assertTrue(st_blue.lamp_green)

        # Orange spare seated: stays red
        spares_orange = dict(self.default_spares)
        spares_orange["orange"] = (g.SLED_SEATED_X, g.BAY_Y[g.FAILED_BAY], g.SLED_REST_Z)
        hl_orange = logic.HotSwapLogic()
        st_orange = hl_orange.step(0.05, self.default_latch, sleds, spares_orange)
        self.assertTrue(st_orange.spare_inserted_95pct)
        self.assertTrue(st_orange.orange_spare_used)
        self.assertFalse(st_orange.lamp_green)


if __name__ == "__main__":
    unittest.main()

