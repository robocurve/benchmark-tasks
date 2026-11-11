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
            np.array(g.SLAB_1_POS), np.array([0, 0, 0, 1]),
            np.array(g.SLAB_2_POS), np.array([0, 0, 0, 1]),
            np.array(g.SLAB_3_POS), np.array([0, 0, 0, 1]),
            np.array(g.SLAB_4_POS), np.array([0, 0, 0, 1]),
            np.array(g.SLAB_5_START), np.array([0, 0, 0, 1]),
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()

