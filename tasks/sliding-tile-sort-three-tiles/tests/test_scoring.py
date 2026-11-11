"""Unit tests for Scorer wrapper."""

import unittest
import numpy as np

import geometry as g
import scoring


class TestScoring(unittest.TestCase):
    def test_scorer_instantiation_and_reset(self):
        s = scoring.Scorer()
        s.reset()
        res = s.update(
            np.array(g.TILE_BLUE_START), np.array([0, 0, 0, 1]), np.zeros(3),
            np.array(g.TILE_ORANGE_START), np.array([0, 0, 0, 1]), np.zeros(3),
            np.array(g.TILE_YELLOW_START), np.array([0, 0, 0, 1]), np.zeros(3),
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()

