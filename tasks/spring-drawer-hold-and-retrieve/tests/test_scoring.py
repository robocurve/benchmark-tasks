"""Unit tests for scoring in spring-drawer-hold-and-retrieve."""
import sys
from pathlib import Path
import unittest
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import scoring


class TestScoring(unittest.TestCase):
    def test_ladder_levels(self):
        s = scoring.Scorer()
        # Initial: 0.0
        res = s.update(0.0, g.BLOCK_START_POS, True)
        self.assertEqual(res.score, 0.0)

        # 0.25: drawer opened >= 90 mm for 1 s
        for _ in range(21):
            res = s.update(0.100, g.BLOCK_START_POS, True)
        self.assertEqual(res.score, 0.25)

        # 0.50: block lifted >= 30 mm clear of floor
        block_lifted = (0.310, 0.0, g.BLOCK_REST_Z + 0.040)
        res = s.update(0.100, block_lifted, False)
        self.assertEqual(res.score, 0.50)

        # 0.75: block resting on green shelf
        res = s.update(0.100, g.SHELF_SPOT, True)
        self.assertEqual(res.score, 0.75)

        # 1.00: drawer closed (< 5 mm), grippers open, arms still 1.0 s
        zero_vel = (np.zeros(3), np.zeros(3))
        arm_vels = {"left": zero_vel, "right": zero_vel}
        arm_tcps = {"left": (0.2, 0.28, 0.8), "right": (0.2, -0.28, 0.8)}
        for _ in range(21):
            res = s.update(0.002, g.SHELF_SPOT, True, arm_tcps=arm_tcps, arm_velocities=arm_vels)
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_zero_gate_pinch(self):
        s = scoring.Scorer()
        for _ in range(21):
            s.update(0.100, g.BLOCK_START_POS, True)

        # Drawer snaps closed on block at lip
        lip_x = g.CABINET_FRONT_X - 0.010
        res = s.update(0.010, (lip_x, 0.0, g.BLOCK_REST_Z), True)
        self.assertEqual(res.score, 0.0)


if __name__ == "__main__":
    unittest.main()

