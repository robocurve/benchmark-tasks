"""Unit tests for scoring in weigh-and-reject-heavy-block."""
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
        res = s.update(
            scale_disp=0.0,
            block_positions=g.BLOCK_SUPPLY_POS,
            grippers_open=True,
        )
        self.assertEqual(res.score, 0.0)

        # 0.25: all 3 blocks weighed
        s.logic.state.block_weighed = {b: True for b in g.BLOCK_NAMES}
        res = s.update(
            scale_disp=0.0,
            block_positions=g.BLOCK_SUPPLY_POS,
            grippers_open=True,
        )
        self.assertEqual(res.score, 0.25)

        # 0.50: heavy block rejected AND weighed
        pos_half = {
            "block_0": g.BLOCK_SUPPLY_POS["block_0"],
            "block_1": g.TRAY_SPOT,
            "block_2": g.BLOCK_SUPPLY_POS["block_2"],
        }
        res = s.update(
            scale_disp=0.0,
            block_positions=pos_half,
            grippers_open=True,
        )
        self.assertEqual(res.score, 0.50)

        # 0.75: light blocks accepted on green shelf
        pos_three_q = {
            "block_0": g.SHELF_SPOTS["block_0"],
            "block_1": g.TRAY_SPOT,
            "block_2": g.SHELF_SPOTS["block_2"],
        }
        res = s.update(
            scale_disp=0.0,
            block_positions=pos_three_q,
            grippers_open=True,
        )
        self.assertEqual(res.score, 0.75)

        # 1.00: scale empty, grippers open, settled for 20 steps
        zero_vel = (np.zeros(3), np.zeros(3))
        arm_vels = {"left": zero_vel, "right": zero_vel}
        for _ in range(21):
            res = s.update(
                scale_disp=0.0,
                block_positions=pos_three_q,
                grippers_open=True,
                arm_velocities=arm_vels,
            )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_two_valid_solutions(self):
        pos_final = {
            "block_0": g.SHELF_SPOTS["block_0"],
            "block_1": g.TRAY_SPOT,
            "block_2": g.SHELF_SPOTS["block_2"],
        }
        zero_vel = (np.zeros(3), np.zeros(3))
        arm_vels = {"left": zero_vel, "right": zero_vel}

        # Solution A: weigh order 0 -> 1 -> 2
        s_a = scoring.Scorer()
        s_a.logic.state.block_weighed = {"block_0": True, "block_1": True, "block_2": True}
        for _ in range(21):
            res_a = s_a.update(0.0, pos_final, True, arm_velocities=arm_vels)
        self.assertEqual(res_a.score, 1.00)

        # Solution B: weigh order 2 -> 1 -> 0
        s_b = scoring.Scorer()
        s_b.logic.state.block_weighed = {"block_2": True, "block_1": True, "block_0": True}
        for _ in range(21):
            res_b = s_b.update(0.0, pos_final, True, arm_velocities=arm_vels)
        self.assertEqual(res_b.score, 1.00)

    def test_zero_gates(self):
        # Wrong rejection caps at 0.25
        s = scoring.Scorer()
        s.logic.state.block_weighed = {b: True for b in g.BLOCK_NAMES}
        wrong_pos = {
            "block_0": g.TRAY_SPOT,
            "block_1": g.TRAY_SPOT,
            "block_2": g.SHELF_SPOTS["block_2"],
        }
        res = s.update(0.0, wrong_pos, True)
        self.assertLessEqual(res.score, 0.25)

        # No-weigh shortcut gives no credit (heavy placed without weighing)
        s2 = scoring.Scorer()
        no_weigh_pos = {
            "block_0": g.BLOCK_SUPPLY_POS["block_0"],
            "block_1": g.TRAY_SPOT,
            "block_2": g.BLOCK_SUPPLY_POS["block_2"],
        }
        res2 = s2.update(0.0, no_weigh_pos, True)
        self.assertEqual(res2.score, 0.0)

        # Dropped block = 0.0
        s3 = scoring.Scorer()
        s3.logic.state.block_weighed = {b: True for b in g.BLOCK_NAMES}
        drop_pos = {
            "block_0": (0.3, 0.0, g.TOP - 0.10),
            "block_1": g.TRAY_SPOT,
            "block_2": g.SHELF_SPOTS["block_2"],
        }
        res3 = s3.update(0.0, drop_pos, True)
        self.assertEqual(res3.score, 0.0)


if __name__ == "__main__":
    unittest.main()
