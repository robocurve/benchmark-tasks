"""Unit tests for the thermal soak task scoring ladder, boundary conditions, zero-gates, and fuzzing."""

from __future__ import annotations

import random
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
from scoring import Scorer, is_off_table, is_on_cool_pad, is_on_hot_plate


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer()
        self.fix_pos = (0.0, 0.0, 0.0)
        self.s1_start = (g.START_POS_1, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        self.s2_start = (g.START_POS_2, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))

    def test_initial_state(self):
        res = self.scorer.update(self.fix_pos, {}, self.s1_start, self.s2_start, True)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_level_025_button_press_or_move(self):
        # HEAT button pressed
        res = self.scorer.update(self.fix_pos, {"heat_btn": -0.007}, self.s1_start, self.s2_start, True)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Or sample moved > 10 mm
        self.scorer.reset()
        moved_s1 = ((g.START_POS_1[0] + 0.015, g.START_POS_1[1], g.START_POS_1[2]), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        res = self.scorer.update(self.fix_pos, {}, moved_s1, self.s2_start, True)
        self.assertEqual(res.score, 0.25)

    def test_level_050_sample1_procedure_latched(self):
        # 1. Sample 1 placed on hot plate
        s1_plate = ((g.PAD_POS[0], g.PAD_POS[1], g.PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        self.scorer.update(self.fix_pos, {}, s1_plate, self.s2_start, True)
        self.assertTrue(self.scorer.logic.state.s1_was_on_plate)

        # 2. Sample 1 lifted
        s1_air = ((g.PAD_POS[0], g.PAD_POS[1], g.TOP + 0.15), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        self.scorer.update(self.fix_pos, {}, s1_air, self.s2_start, True)
        self.assertTrue(self.scorer.logic.state.s1_was_lifted)

        # 3. Sample 1 placed on cool pad
        s1_cool = ((g.COOL_SPOT_1[0], g.COOL_SPOT_1[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        res = self.scorer.update(self.fix_pos, {}, s1_cool, self.s2_start, True)
        self.assertEqual(res.score, 0.50)
        self.assertTrue(res.details["s1_complete"])

    def test_level_075_sample2_on_cool_pad_within_45_80(self):
        # Satisfy s1 milestone first
        self.test_level_050_sample1_procedure_latched()
        s1_cool = ((g.COOL_SPOT_1[0], g.COOL_SPOT_1[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))

        # Sample 2 on cool pad with core at 55 C (in 45-80)
        s2_cool = ((g.COOL_SPOT_2[0], g.COOL_SPOT_2[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        self.scorer.logic.state.tc2 = 55.0
        res = self.scorer.update(self.fix_pos, {}, s1_cool, s2_cool, True)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_level_100_full_success_hold(self):
        # Satisfy s1 milestone
        self.test_level_050_sample1_procedure_latched()
        s1_cool = ((g.COOL_SPOT_1[0], g.COOL_SPOT_1[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        s2_cool = ((g.COOL_SPOT_2[0], g.COOL_SPOT_2[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))

        # Set target conditions: core in [60, 70], rate < 0.2, heater off, grippers open
        self.scorer.logic.state.tc2 = 65.0
        self.scorer.logic.state.ts2 = 65.0
        self.scorer.logic.state.prev_tc2 = 65.0
        self.scorer.logic.state.heater_on = False

        # Hold for 19 steps: score remains 0.75
        for _ in range(19):
            res = self.scorer.update(self.fix_pos, {}, s1_cool, s2_cool, True)
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 20th step (1.0 s): score reaches 1.00 and success is True
        res = self.scorer.update(self.fix_pos, {}, s1_cool, s2_cool, True)
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)
        self.assertEqual(res.details["status_lamp"], "green")

    def test_boundaries_around_60_and_70(self):
        self.test_level_050_sample1_procedure_latched()
        s1_cool = ((g.COOL_SPOT_1[0], g.COOL_SPOT_1[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        s2_cool = ((g.COOL_SPOT_2[0], g.COOL_SPOT_2[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        self.scorer.logic.state.heater_on = False

        # 59.9 C: inside [45, 80], outside [60, 70] -> 0.75
        self.scorer.logic.state.tc2 = 59.9
        self.scorer.logic.state.prev_tc2 = 59.9
        for _ in range(25):
            res = self.scorer.update(self.fix_pos, {}, s1_cool, s2_cool, True)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # 70.1 C: inside [45, 80], outside [60, 70] -> 0.75
        self.scorer.logic.state.tc2 = 70.1
        self.scorer.logic.state.prev_tc2 = 70.1
        for _ in range(25):
            res = self.scorer.update(self.fix_pos, {}, s1_cool, s2_cool, True)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_zero_gates(self):
        # 1. Fire zero-gate: core > 90 C
        self.scorer.logic.state.tc2 = 91.0
        res = self.scorer.update(self.fix_pos, {}, self.s1_start, self.s2_start, True)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details["fire_tripped"])

        # 2. Sample off table
        self.scorer.reset()
        off_table_s1 = ((g.START_POS_1[0], g.START_POS_1[1], g.TOP - 0.10), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        res = self.scorer.update(self.fix_pos, {}, off_table_s1, self.s2_start, True)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details["off_table"])

        # 3. Fixture displaced > 5 mm
        self.scorer.reset()
        # Initialize at nominal position
        self.scorer.update((0.0, 0.0, 0.0), {}, self.s1_start, self.s2_start, True)
        # Move fixture > 5 mm
        displaced_fix = (0.008, 0.0, 0.0)
        res = self.scorer.update(displaced_fix, {}, self.s1_start, self.s2_start, True)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details["fixture_displaced"])

    def test_fuzz_never_produces_invalid_score(self):
        rng = random.Random(42)
        valid_scores = {0.0, 0.25, 0.50, 0.75, 1.00}
        for _ in range(200):
            fix_p = (
                g.FIXTURE_POS[0] + rng.uniform(-0.01, 0.01),
                g.FIXTURE_POS[1] + rng.uniform(-0.01, 0.01),
                g.FIXTURE_POS[2],
            )
            s1_p = (
                rng.uniform(0.1, 0.5),
                rng.uniform(-0.4, 0.4),
                rng.uniform(g.TOP - 0.1, g.TOP + 0.2),
            )
            s2_p = (
                rng.uniform(0.1, 0.5),
                rng.uniform(-0.4, 0.4),
                rng.uniform(g.TOP - 0.1, g.TOP + 0.2),
            )
            s1_d = (s1_p, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
            s2_d = (s2_p, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
            sliders = {"heat_btn": rng.uniform(-0.01, 0.002), "off_btn": rng.uniform(-0.01, 0.002)}
            g_open = rng.choice([True, False])
            res = self.scorer.update(fix_p, sliders, s1_d, s2_d, g_open)
            self.assertIn(res.score, valid_scores)


if __name__ == "__main__":
    unittest.main()
