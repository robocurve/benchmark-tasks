"""Tests for stateful logic and rule verification in domino-chain-set-and-tip.

Includes tests for both valid orders of arm placement (Left arm first vs Right arm first).
"""

import math
import unittest
import numpy as np

import geometry as g
import logic


def pitch_quat(deg: float) -> tuple[float, float, float, float]:
    """Quaternion for pitch about Y axis (rotation in XZ plane)."""
    rad = math.radians(deg)
    return (0.0, math.sin(rad / 2.0), 0.0, math.cos(rad / 2.0))


def roll_quat(deg: float) -> tuple[float, float, float, float]:
    """Quaternion for roll about X axis (rotation in YZ plane)."""
    rad = math.radians(deg)
    return (math.sin(rad / 2.0), 0.0, 0.0, math.cos(rad / 2.0))


UPRIGHT_QUAT = (0.0, 0.0, 0.0, 1.0)
FALLEN_PLUS_X_QUAT = pitch_quat(70.0)      # > 60 deg pitch toward +x
FALLEN_MINUS_X_QUAT = pitch_quat(-70.0)    # > 60 deg pitch toward -x
FALLEN_SIDEWAYS_QUAT = roll_quat(70.0)     # > 60 deg roll toward +y


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.DominoLogic(rest_s=0.3, max_tip_time_s=5.0)
        self.arm_tcps_clear = {
            "left": (0.200, +0.280, g.HOVER_Z),
            "right": (0.200, -0.280, g.HOVER_Z),
        }

    def test_initial_state(self):
        st = self.lg.step(
            dt=0.05,
            domino_poses=list(g.DOMINO_START_POSITIONS),
            domino_quats=[UPRIGHT_QUAT] * 4,
            domino_vels=[(0.0, 0.0, 0.0)] * 4,
            arm_tcps=self.arm_tcps_clear,
        )
        self.assertFalse(st.ever_standing_2)
        self.assertFalse(st.ever_standing_4)
        self.assertFalse(st.first_tipped)
        self.assertFalse(st.ever_fallen_4)
        self.assertFalse(st.premature)
        self.assertFalse(st.solved)
        self.assertEqual(st.standing_count, 0)

    def test_valid_order_left_first(self):
        """Valid sequence 1: Left arm places first 2 dominoes, then Right arm places second 2."""
        lg = logic.DominoLogic(rest_s=0.3, max_tip_time_s=5.0)

        # Step 1: Left arm places Domino 0 on Mark 0 and Domino 1 on Mark 1
        pos_0 = (g.MARK_0_POS[0], g.MARK_0_POS[1], g.STANDING_Z)
        pos_1 = (g.MARK_1_POS[0], g.MARK_1_POS[1], g.STANDING_Z)
        poses = [pos_0, pos_1, g.DOMINO_2_START_POS, g.DOMINO_3_START_POS]
        quats = [UPRIGHT_QUAT] * 4
        vels = [(0.0, 0.0, 0.0)] * 4

        # Settle for 7 steps (0.35 s > 0.3 s)
        st = None
        for _ in range(7):
            st = lg.step(0.05, poses, quats, vels, self.arm_tcps_clear)

        self.assertEqual(st.standing_count, 2)
        self.assertTrue(st.ever_standing_2)
        self.assertFalse(st.ever_standing_4)
        self.assertFalse(st.premature)

        # Step 2: Right arm places Domino 2 on Mark 2 and Domino 3 on Mark 3
        pos_2 = (g.MARK_2_POS[0], g.MARK_2_POS[1], g.STANDING_Z)
        pos_3 = (g.MARK_3_POS[0], g.MARK_3_POS[1], g.STANDING_Z)
        poses = [pos_0, pos_1, pos_2, pos_3]

        for _ in range(7):
            st = lg.step(0.05, poses, quats, vels, self.arm_tcps_clear)

        self.assertEqual(st.standing_count, 4)
        self.assertTrue(st.ever_standing_4)
        self.assertFalse(st.premature)
        self.assertFalse(st.first_tipped)

        # Step 3: Tip first domino (tilt > 20 deg) while others are standing
        quats_tipped = [pitch_quat(25.0), UPRIGHT_QUAT, UPRIGHT_QUAT, UPRIGHT_QUAT]
        st = lg.step(0.05, poses, quats_tipped, vels, self.arm_tcps_clear)
        self.assertTrue(st.first_tipped)
        self.assertFalse(st.ever_fallen_4)

        # Step 4: Cascade topples all 4 dominoes forward toward +x within 1 second
        quats_all_fallen = [FALLEN_PLUS_X_QUAT] * 4
        for _ in range(10):  # 0.5 s elapsed
            st = lg.step(0.05, poses, quats_all_fallen, vels, self.arm_tcps_clear)

        self.assertTrue(st.ever_fallen_4)
        self.assertTrue(st.solved)
        self.assertFalse(st.premature)

    def test_valid_order_right_first(self):
        """Valid sequence 2: Right arm places second 2 dominoes first, then Left arm places first 2."""
        lg = logic.DominoLogic(rest_s=0.3, max_tip_time_s=5.0)

        # Step 1: Right arm places Domino 2 on Mark 2 and Domino 3 on Mark 3
        pos_2 = (g.MARK_2_POS[0], g.MARK_2_POS[1], g.STANDING_Z)
        pos_3 = (g.MARK_3_POS[0], g.MARK_3_POS[1], g.STANDING_Z)
        poses = [g.DOMINO_0_START_POS, g.DOMINO_1_START_POS, pos_2, pos_3]
        quats = [UPRIGHT_QUAT] * 4
        vels = [(0.0, 0.0, 0.0)] * 4

        st = None
        for _ in range(7):
            st = lg.step(0.05, poses, quats, vels, self.arm_tcps_clear)

        self.assertEqual(st.standing_count, 2)
        self.assertTrue(st.ever_standing_2)
        self.assertFalse(st.ever_standing_4)
        self.assertFalse(st.premature)

        # Step 2: Left arm places Domino 0 on Mark 0 and Domino 1 on Mark 1
        pos_0 = (g.MARK_0_POS[0], g.MARK_0_POS[1], g.STANDING_Z)
        pos_1 = (g.MARK_1_POS[0], g.MARK_1_POS[1], g.STANDING_Z)
        poses = [pos_0, pos_1, pos_2, pos_3]

        for _ in range(7):
            st = lg.step(0.05, poses, quats, vels, self.arm_tcps_clear)

        self.assertEqual(st.standing_count, 4)
        self.assertTrue(st.ever_standing_4)
        self.assertFalse(st.premature)

        # Step 3: Tip first domino
        quats_tipped = [pitch_quat(25.0), UPRIGHT_QUAT, UPRIGHT_QUAT, UPRIGHT_QUAT]
        st = lg.step(0.05, poses, quats_tipped, vels, self.arm_tcps_clear)
        self.assertTrue(st.first_tipped)

        # Step 4: Cascade topples all 4 forward
        quats_all_fallen = [FALLEN_PLUS_X_QUAT] * 4
        for _ in range(10):
            st = lg.step(0.05, poses, quats_all_fallen, vels, self.arm_tcps_clear)

        self.assertTrue(st.ever_fallen_4)
        self.assertTrue(st.solved)
        self.assertFalse(st.premature)

    def test_premature_fall_zero_gate(self):
        """Any domino falling (tilt > 45 deg) before all four are standing latches premature."""
        lg = logic.DominoLogic(rest_s=0.3, max_tip_time_s=5.0)

        pos_0 = (g.MARK_0_POS[0], g.MARK_0_POS[1], g.STANDING_Z)
        pos_1 = (g.MARK_1_POS[0], g.MARK_1_POS[1], g.STANDING_Z)
        poses = [pos_0, pos_1, g.DOMINO_2_START_POS, g.DOMINO_3_START_POS]
        quats = [UPRIGHT_QUAT] * 4
        vels = [(0.0, 0.0, 0.0)] * 4

        # 2 dominoes standing
        for _ in range(7):
            st = lg.step(0.05, poses, quats, vels, self.arm_tcps_clear)
        self.assertTrue(st.ever_standing_2)
        self.assertFalse(st.premature)

        # Domino 0 falls over (tilt = 50 deg > 45 deg) before all 4 standing
        quats_fallen = [pitch_quat(50.0), UPRIGHT_QUAT, UPRIGHT_QUAT, UPRIGHT_QUAT]
        st = lg.step(0.05, poses, quats_fallen, vels, self.arm_tcps_clear)
        self.assertTrue(st.premature)

    def test_fallen_direction_checks(self):
        """is_fallen_plus_x requires tilt > 60 deg AND direction along +x."""
        self.assertTrue(logic.is_fallen_plus_x(FALLEN_PLUS_X_QUAT))
        self.assertFalse(logic.is_fallen_plus_x(FALLEN_MINUS_X_QUAT))
        self.assertFalse(logic.is_fallen_plus_x(FALLEN_SIDEWAYS_QUAT))
        self.assertFalse(logic.is_fallen_plus_x(UPRIGHT_QUAT))

    def test_tip_timeout_no_fall_within_5s(self):
        """If > 5 s elapse after tip before all 4 fall, ever_fallen_4 does not latch."""
        lg = logic.DominoLogic(rest_s=0.3, max_tip_time_s=5.0)
        poses = [(m[0], m[1], g.STANDING_Z) for m in g.MARK_POSITIONS]
        quats = [UPRIGHT_QUAT] * 4
        vels = [(0.0, 0.0, 0.0)] * 4

        for _ in range(7):
            lg.step(0.05, poses, quats, vels, self.arm_tcps_clear)

        # Tip
        quats_tipped = [pitch_quat(25.0), UPRIGHT_QUAT, UPRIGHT_QUAT, UPRIGHT_QUAT]
        lg.step(0.05, poses, quats_tipped, vels, self.arm_tcps_clear)

        # Wait 5.5 seconds (110 steps at 0.05 s) without all falling
        st = None
        for _ in range(110):
            st = lg.step(0.05, poses, quats_tipped, vels, self.arm_tcps_clear)

        self.assertGreater(st.tip_timer, 5.0)
        # Now dominoes fall late
        quats_late = [FALLEN_PLUS_X_QUAT] * 4
        st = lg.step(0.05, poses, quats_late, vels, self.arm_tcps_clear)
        self.assertFalse(st.ever_fallen_4)

    def test_arms_must_be_clear(self):
        """Arms must be clear (> 60 mm) for ever_fallen_4 to latch."""
        lg = logic.DominoLogic(rest_s=0.3, max_tip_time_s=5.0)
        poses = [(m[0], m[1], g.STANDING_Z) for m in g.MARK_POSITIONS]
        quats = [UPRIGHT_QUAT] * 4
        vels = [(0.0, 0.0, 0.0)] * 4

        for _ in range(7):
            lg.step(0.05, poses, quats, vels, self.arm_tcps_clear)

        # Tip
        quats_tipped = [pitch_quat(25.0), UPRIGHT_QUAT, UPRIGHT_QUAT, UPRIGHT_QUAT]
        lg.step(0.05, poses, quats_tipped, vels, self.arm_tcps_clear)

        # Dominoes fall, but Left arm is hovering directly over Mark 0 (distance < 60 mm)
        arm_in_way = {
            "left": (g.MARK_0_POS[0], g.MARK_0_POS[1], g.TOP + 0.050),
            "right": (0.200, -0.280, g.HOVER_Z),
        }
        quats_fallen = [FALLEN_PLUS_X_QUAT] * 4
        st = lg.step(0.05, poses, quats_fallen, vels, arm_in_way)
        self.assertFalse(st.arms_clear)
        self.assertFalse(st.ever_fallen_4)

        # Once arm retreats clear
        st = lg.step(0.05, poses, quats_fallen, vels, self.arm_tcps_clear)
        self.assertTrue(st.arms_clear)
        self.assertTrue(st.ever_fallen_4)


if __name__ == "__main__":
    unittest.main()
