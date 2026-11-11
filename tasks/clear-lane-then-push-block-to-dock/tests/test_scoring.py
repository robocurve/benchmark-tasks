"""Unit tests for Scorer glue and ladder in clear-lane-then-push-block-to-dock."""

import unittest
import numpy as np

import geometry as g
import scoring


class TestScoring(unittest.TestCase):
    def test_ladder_values(self):
        scores = [s for s, _ in scoring.LADDER]
        self.assertEqual(scores, [0.25, 0.50, 0.75, 1.00])

    def test_zero_gates(self):
        names = [name for name, _, _ in scoring.ZERO_GATES]
        self.assertIn("post_hit", names)

    def test_scorer_step_and_reset(self):
        scorer = scoring.Scorer(hold_s=g.HOLD_TIME_S, hz=20.0)
        neutral_quat = np.array([0.0, 0.0, 0.0, 1.0])

        res = scorer.step(
            cargo_pos=np.array(g.CARGO_START_POS),
            cargo_quat=neutral_quat,
            blue_pos=np.array(g.BLUE_START_POS),
            yellow_pos=np.array(g.YELLOW_START_POS),
            post_pos=np.array(g.POST_START_POS),
            post_quat=neutral_quat,
        )
        self.assertEqual(res.score, 0.0)

        scorer.reset()
        self.assertEqual(scorer.logic.hold_counter, 0)
        self.assertFalse(scorer.logic.ever_post_hit)


if __name__ == "__main__":
    unittest.main()

