"""Scoring: start state, the ladder, end-state (undoing lowers the score), the hold, and each latched violation."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import geometry as g
import logic

S = g.S                                                  # +1 for layout A, -1 for the mirror-image layout B
SAW = (g.SAW_X, g.SAW_Y, g.TOP)
Q0 = (0.0, 0.0, 0.0, 1.0)
QS = (0.0, 0.0, math.sin(g.SCAB_YAW / 2), math.cos(g.SCAB_YAW / 2))      # the scabbard's resting orientation
HOME = [(0.30, 0.30, g.TOP + 0.30), (0.30, -0.30, g.TOP + 0.30)]
SCAB_START = (g.SCAB_X, g.SCAB_CENTER_Y_START, g.TOP + g.SCAB_CENTER_Z)
SCAB_ON = (g.SAW_X, g.SAW_Y - S * (0.004 + g.SCAB_LEN / 2), g.TOP + g.SCAB_CENTER_Z)
ENGAGED = g.BRAKE_STOP


def step(s, *, guard=0.0, scab=SCAB_START, tcps=HOME, saw=SAW, saw_q=Q0, scab_q=QS, n=1):
    r = None
    for _ in range(n):
        r = s.step(saw_pos=saw, saw_quat=saw_q, guard_angle=guard, scab_pos=scab, scab_quat=scab_q, tcps=tcps)
    return r


class Scoring(unittest.TestCase):
    def setUp(self):
        self.s = logic.Scorer()

    def test_start_state_scores_zero(self):
        r = step(self.s, n=60)
        self.assertEqual(r["score"], 0.0)
        self.assertFalse(r["success"])

    def test_brake_only_is_025(self):
        self.assertEqual(step(self.s, guard=ENGAGED, n=60)["score"], 0.25)

    def test_cover_only_is_05(self):
        r = step(self.s, scab=SCAB_ON, n=60)
        self.assertTrue(r["cover_full"])
        self.assertEqual(r["score"], 0.5)

    def test_brake_and_half_a_cover_is_025(self):
        half = (g.SAW_X, g.SAW_Y - S * (g.BAR_LEN * 0.55 + g.SCAB_LEN / 2 - 0.05), g.TOP + g.SCAB_CENTER_Z)
        r = step(self.s, guard=ENGAGED, scab=half)
        self.assertLess(r["covered"], 0.95)
        self.assertEqual(r["score"], 0.25)

    def test_both_is_075_until_held_two_seconds(self):
        r = step(self.s, guard=ENGAGED, scab=SCAB_ON, n=39)
        self.assertEqual(r["score"], 0.75)
        self.assertFalse(r["success"])
        r = step(self.s, guard=ENGAGED, scab=SCAB_ON)               # the 40th step: 2.0 s at 20 Hz
        self.assertEqual(r["score"], 1.0)
        self.assertTrue(r["success"])

    def test_arms_in_the_way_stop_the_hold(self):
        in_the_way = [(0.30, 0.0, g.TOP + 0.06), (0.30, -0.30, g.TOP + 0.30)]
        r = step(self.s, guard=ENGAGED, scab=SCAB_ON, tcps=in_the_way, n=80)
        self.assertEqual(r["score"], 0.75)
        self.assertFalse(r["success"])

    def test_undoing_a_step_lowers_the_score(self):
        step(self.s, guard=ENGAGED, scab=SCAB_ON, n=60)
        r = step(self.s, guard=0.0, scab=SCAB_ON)                  # the brake released again
        self.assertEqual(r["score"], 0.5)
        r = step(self.s, guard=0.0, scab=SCAB_START)               # and the cover taken off
        self.assertEqual(r["score"], 0.0)

    def test_the_hold_restarts_after_an_interruption(self):
        step(self.s, guard=ENGAGED, scab=SCAB_ON, n=30)
        step(self.s, guard=0.0, scab=SCAB_ON)
        r = step(self.s, guard=ENGAGED, scab=SCAB_ON, n=39)
        self.assertFalse(r["success"])

    def test_a_scabbard_not_on_the_bar_does_not_count(self):
        off_axis = (g.SAW_X + 0.03, SCAB_ON[1], SCAB_ON[2])         # beside the bar
        self.assertEqual(step(self.s, scab=off_axis)["score"], 0.0)
        above = (SCAB_ON[0], SCAB_ON[1], SCAB_ON[2] + 0.03)         # lifted over it
        self.assertEqual(step(self.s, scab=above)["score"], 0.0)
        tilted = (math.sin(math.radians(10)), 0.0, 0.0, math.cos(math.radians(10)))   # 20 degrees about x
        self.assertEqual(step(self.s, scab=SCAB_ON, scab_q=tilted)["score"], 0.0)

    def test_a_scabbard_turned_end_for_end_does_not_count(self):
        turned = (0.0, 0.0, -QS[3], QS[2])                           # a half turn about the vertical from its rest pose
        r = step(self.s, scab=SCAB_ON, scab_q=turned)
        self.assertFalse(r["aligned"])
        self.assertEqual(r["score"], 0.0)

    def test_brake_threshold(self):
        self.assertFalse(logic.brake_engaged(g.BRAKE_DETENT))        # at the detent: not yet on
        self.assertFalse(logic.brake_engaged(g.BRAKE_ON + 0.01))
        self.assertTrue(logic.brake_engaged(g.BRAKE_ON))
        self.assertTrue(logic.brake_engaged(g.BRAKE_STOP))


class Violations(unittest.TestCase):
    def setUp(self):
        self.s = logic.Scorer()

    def test_moving_the_saw_is_latched_and_caps_at_025(self):
        step(self.s)
        moved = (g.SAW_X + 0.05, g.SAW_Y, g.TOP)
        r = step(self.s, saw=moved, guard=ENGAGED, scab=SCAB_ON)
        self.assertTrue(r["moved"])
        r = step(self.s, saw=SAW, guard=ENGAGED, scab=SCAB_ON, n=80)     # put back: still failed
        self.assertTrue(r["moved"])
        self.assertEqual(r["score"], 0.25)
        self.assertFalse(r["success"])

    def test_a_small_shift_is_fine(self):
        step(self.s)
        r = step(self.s, saw=(g.SAW_X + 0.02, g.SAW_Y, g.TOP), guard=ENGAGED, scab=SCAB_ON, n=45)
        self.assertFalse(r["moved"])
        self.assertTrue(r["success"])

    def test_tipping_the_saw_is_latched(self):
        step(self.s)
        tipped = (math.sin(math.radians(20) / 2), 0.0, 0.0, math.cos(math.radians(20) / 2))
        r = step(self.s, saw_q=tipped)
        self.assertTrue(r["tipped"])
        r = step(self.s, guard=ENGAGED, scab=SCAB_ON, n=80)
        self.assertEqual(r["score"], 0.25)

    def test_a_hand_on_the_chain_is_latched(self):
        on_chain = [(g.SAW_X, g.SAW_Y - S * 0.12, g.TOP + g.BAR_Z + 0.004), HOME[1]]
        r = step(self.s, tcps=on_chain)
        self.assertTrue(r["chain_touched"])
        r = step(self.s, guard=ENGAGED, scab=SCAB_ON, n=80)
        self.assertEqual(r["score"], 0.25)
        self.assertFalse(r["success"])

    def test_holding_the_scabbard_over_the_bar_is_not_a_chain_touch(self):
        holding = [(g.SAW_X, SCAB_ON[1], g.TOP + g.BAR_Z + 0.004), HOME[1]]
        r = step(self.s, scab=SCAB_ON, tcps=holding)
        self.assertFalse(r["chain_touched"])

    def test_touching_the_chain_beyond_the_scabbard_is_a_touch(self):
        # the scabbard covers only the housing end; a tool point at the bar's tip is outside it
        half = (g.SAW_X, g.SAW_Y - S * (0.002 + g.SCAB_LEN / 2 - 0.15), g.TOP + g.SCAB_CENTER_Z)
        tip = [(g.SAW_X, g.SAW_Y - S * (g.BAR_LEN - 0.01), g.TOP + g.BAR_Z), HOME[1]]
        r = step(self.s, scab=half, tcps=tip)
        self.assertTrue(r["chain_touched"])

    def test_tool_point_beside_the_zone_is_fine(self):
        far = [(g.SAW_X + 0.07, g.SAW_Y - S * 0.1, g.TOP + g.BAR_Z), HOME[1]]
        self.assertFalse(step(self.s, tcps=far)["chain_touched"])


class Helpers(unittest.TestCase):
    def test_cover_fraction(self):
        full, ok = logic.scabbard_cover(SCAB_ON, QS)
        self.assertGreaterEqual(full, 0.95)
        self.assertTrue(ok)
        none, _ = logic.scabbard_cover(SCAB_START, QS)
        self.assertEqual(none, 0.0)

    def test_arms_clear(self):
        self.assertTrue(logic.arms_clear(HOME))
        self.assertFalse(logic.arms_clear([(0.3, 0.0, g.TOP + 0.05)]))
        self.assertTrue(logic.arms_clear([(0.10, 0.0, g.TOP + 0.05)]))     # outside the work box


if __name__ == "__main__":
    unittest.main()
