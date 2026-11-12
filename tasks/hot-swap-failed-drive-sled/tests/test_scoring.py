"""Unit tests for scoring in hot-swap-failed-drive-sled."""
import sys
from pathlib import Path
import unittest
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import scoring


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.s = scoring.Scorer()
        self.default_latch = {"bay_0": 0.0, "bay_1": 0.0, "bay_2": 0.0}
        self.default_sleds = {
            bay: (g.SLED_SEATED_X, g.BAY_Y[bay], g.SLED_REST_Z)
            for bay in ("bay_0", "bay_1", "bay_2")
        }
        self.default_spares = {
            color: g.SPARE_POS[color]
            for color in ("blue", "orange")
        }

    def test_ladder_levels(self):
        # Initial: 0.0
        res = self.s.update(self.default_latch, self.default_sleds, self.default_spares, True)
        self.assertEqual(res.score, 0.0)

        # 0.25: pulled >= 60 mm with latch
        latch_pressed = {"bay_0": 0.0, "bay_1": -0.007, "bay_2": 0.0}
        sleds_pulled = dict(self.default_sleds)
        sleds_pulled[g.FAILED_BAY] = (g.SLED_SEATED_X - 0.070, 0.0, g.SLED_REST_Z)
        res = self.s.update(latch_pressed, sleds_pulled, self.default_spares, True)
        self.assertEqual(res.score, 0.25)

        # 0.50: old sled on discard pad
        sleds_discarded = dict(self.default_sleds)
        sleds_discarded[g.FAILED_BAY] = g.DISCARD_SPOT
        res = self.s.update(self.default_latch, sleds_discarded, self.default_spares, True)
        self.assertEqual(res.score, 0.50)

        # 0.75: blue spare inserted >= 95%
        spares_blue = dict(self.default_spares)
        spares_blue["blue"] = (g.SLED_SEATED_X, 0.0, g.SLED_REST_Z)
        res = self.s.update(self.default_latch, sleds_discarded, spares_blue, True)
        self.assertEqual(res.score, 0.75)

        # 1.00: lamp green, arms still, settled for 20 steps
        zero_vel = (np.zeros(3), np.zeros(3))
        arm_vels = {"left": zero_vel, "right": zero_vel}
        arm_tcps_clear = {"left": (0.2, 0.3, 0.8), "right": (0.2, -0.3, 0.8)}
        for _ in range(21):
            res = self.s.update(
                self.default_latch,
                sleds_discarded,
                spares_blue,
                True,
                arm_tcps=arm_tcps_clear,
                arm_velocities=arm_vels,
            )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_zero_gate_forced_pull(self):
        # Pull without latch
        sleds_pulled = dict(self.default_sleds)
        sleds_pulled[g.FAILED_BAY] = (g.SLED_SEATED_X - 0.015, 0.0, g.SLED_REST_Z)
        res = self.s.update(self.default_latch, sleds_pulled, self.default_spares, True)
        self.assertEqual(res.score, 0.0)

    def test_wrong_spare_caps_at_0_50(self):
        s = scoring.Scorer()
        latch_pressed = {"bay_0": 0.0, "bay_1": -0.007, "bay_2": 0.0}
        sleds_discarded = dict(self.default_sleds)
        sleds_discarded[g.FAILED_BAY] = g.DISCARD_SPOT
        s.update(latch_pressed, sleds_discarded, self.default_spares, True)

        # Insert orange spare
        spares_orange = dict(self.default_spares)
        spares_orange["orange"] = (g.SLED_SEATED_X, 0.0, g.SLED_REST_Z)
        res = s.update(self.default_latch, sleds_discarded, spares_orange, True)
        self.assertEqual(res.score, 0.50)

    def _score_sequence(self, s, sled_pos, spare_blue_pos, latch=None, steps=1):
        sleds = dict(self.default_sleds)
        sleds[g.FAILED_BAY] = sled_pos
        spares = dict(self.default_spares)
        spares["blue"] = spare_blue_pos
        for _ in range(steps):
            res = s.update(latch or self.default_latch, sleds, spares, True)
        return res

    def test_undoing_a_goal_lowers_the_score(self):
        # end-state scoring: spare pulled out again after being seated must lose the 0.75
        s = scoring.Scorer()
        latch_pressed = {"bay_0": 0.0, "bay_1": -0.007, "bay_2": 0.0}
        pulled = (g.SLED_SEATED_X - 0.070, 0.0, g.SLED_REST_Z)
        self._score_sequence(s, pulled, g.SPARE_POS["blue"], latch=latch_pressed)
        res = self._score_sequence(s, g.DISCARD_SPOT, (g.SLED_SEATED_X, 0.0, g.SLED_REST_Z))
        self.assertEqual(res.score, 0.75)
        res = self._score_sequence(s, g.DISCARD_SPOT, g.SPARE_POS["blue"])
        self.assertEqual(res.score, 0.50)
        # and moving the removed sled off the pad lowers 0.50 back to the 0.25 pull event
        off_pad = (g.DISCARD_SPOT[0] + 0.100, g.DISCARD_SPOT[1], g.SLED_REST_Z)
        res = self._score_sequence(s, off_pad, g.SPARE_POS["blue"])
        self.assertEqual(res.score, 0.25)

    def test_published_075_needs_the_old_sled_on_the_pad(self):
        # the spare seated alone is not 0.75: the removed sled must also rest on the discard pad
        s = scoring.Scorer()
        latch_pressed = {"bay_0": 0.0, "bay_1": -0.007, "bay_2": 0.0}
        pulled = (g.SLED_SEATED_X - 0.070, 0.0, g.SLED_REST_Z)
        self._score_sequence(s, pulled, g.SPARE_POS["blue"], latch=latch_pressed)
        res = self._score_sequence(s, pulled, (g.SLED_SEATED_X, 0.0, g.SLED_REST_Z))
        self.assertEqual(res.score, 0.25)

    def test_discard_pad_tolerance_is_40_mm_and_10_mm(self):
        latch_pressed = {"bay_0": 0.0, "bay_1": -0.007, "bay_2": 0.0}
        pulled = (g.SLED_SEATED_X - 0.070, 0.0, g.SLED_REST_Z)
        for dx, dz, expected in ((0.030, 0.0, 0.50), (0.050, 0.0, 0.25), (0.0, 0.015, 0.25)):
            s = scoring.Scorer()
            self._score_sequence(s, pulled, g.SPARE_POS["blue"], latch=latch_pressed)
            pos = (g.DISCARD_SPOT[0] + dx, g.DISCARD_SPOT[1], g.DISCARD_SPOT[2] + dz)
            res = self._score_sequence(s, pos, g.SPARE_POS["blue"])
            self.assertEqual(res.score, expected, (dx, dz))

    def test_start_state_scores_zero(self):
        res = self.s.update(self.default_latch, self.default_sleds, self.default_spares, True)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_key_tabs_match_the_spare_colours(self):
        self.assertEqual(g.KEY_TAB_COLORS[g.FAILED_BAY], g.HANDLE_BLUE)
        self.assertEqual(g.KEY_TAB_COLORS["bay_0"], g.HANDLE_ORANGE)
        tabs = [b for b in g.rack_world_boxes() if b[1] == g.KEY_TAB_SIZE]
        self.assertEqual(len(tabs), 3)


if __name__ == "__main__":
    unittest.main()

