"""Tests for state machine, overlap math, zero-gates, and valid order execution in bridge-plate-across-trench."""

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.BridgeLogic(hold_s=2.0, rest_s=0.3)

    def test_initial_state(self):
        st = self.lg.step(
            dt=0.05,
            bridge_pos=g.BRIDGE_START_POS,
            bridge_quat=(0.0, 0.0, 0.0, 1.0),
            bridge_vel=(0.0, 0.0, 0.0),
            block_pos=g.BLOCK_START_POS,
            block_quat=(0.0, 0.0, 0.0, 1.0),
            block_vel=(0.0, 0.0, 0.0),
        )
        self.assertFalse(st.fell)
        self.assertFalse(st.bridge_seated)
        self.assertFalse(st.block_on_bridge)
        self.assertFalse(st.block_on_far)
        self.assertFalse(st.ever_25)
        self.assertFalse(st.ever_50)
        self.assertFalse(st.ever_75)
        self.assertFalse(st.ever_100)
        self.assertFalse(st.solved)

    def test_zero_gate_fell_in_trench(self):
        # Bridge drops into trench (z < FELL_Z_THRESHOLD)
        st = self.lg.step(
            dt=0.05,
            bridge_pos=(0.30, 0.00, g.FELL_Z_THRESHOLD - 0.010),
            bridge_quat=(0.0, 0.0, 0.0, 1.0),
            bridge_vel=(0.0, 0.0, 0.0),
            block_pos=g.BLOCK_START_POS,
            block_quat=(0.0, 0.0, 0.0, 1.0),
            block_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.fell)

        # Latch test: even if z goes back up, fell remains True
        st = self.lg.step(
            dt=0.05,
            bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
            bridge_quat=(0.0, 0.0, 0.0, 1.0),
            bridge_vel=(0.0, 0.0, 0.0),
            block_pos=g.BLOCK_START_POS,
            block_quat=(0.0, 0.0, 0.0, 1.0),
            block_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.fell)

    def test_bridge_overlap_math(self):
        lg = logic.BridgeLogic(hold_s=2.0, rest_s=0.3)
        seated_pos = g.BRIDGE_SEATED_TARGET_POS  # (0.30, 0.00, TOP + 0.033)

        # Settle bridge resting for >= 0.3 s (7 steps at 0.05 s)
        st = None
        for _ in range(8):
            st = lg.step(
                dt=0.05,
                bridge_pos=seated_pos,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=g.BLOCK_START_POS,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
            )
        self.assertAlmostEqual(st.near_overlap, 0.030, places=3)
        self.assertAlmostEqual(st.far_overlap, 0.030, places=3)
        self.assertTrue(st.bridge_seated)
        self.assertTrue(st.ever_25)

        # If bridge is shifted by +15 mm along Y, far overlap is only 15 mm < 20 mm
        lg.reset()
        shifted_pos = (0.25, 0.015, seated_pos[2])
        for _ in range(8):
            st = lg.step(
                dt=0.05,
                bridge_pos=shifted_pos,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=g.BLOCK_START_POS,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
            )
        self.assertLess(st.far_overlap, g.BRIDGE_MIN_OVERLAP)
        self.assertFalse(st.bridge_seated)

    def test_bridge_tilt(self):
        lg = logic.BridgeLogic(hold_s=2.0, rest_s=0.3)
        # 5 degrees tilt around X axis (sin(2.5 deg) = 0.0436)
        tilted_quat = (0.0436, 0.0, 0.0, 0.999)
        st = None
        for _ in range(8):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=tilted_quat,
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=g.BLOCK_START_POS,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
            )
        self.assertGreater(st.bridge_tilt, g.BRIDGE_MAX_TILT_DEG)
        self.assertFalse(st.bridge_seated)

    def test_valid_order_1_bridge_laid_first(self):
        """Simulate Order 1: Bridge is laid across trench first, then block is pushed across."""
        lg = logic.BridgeLogic(hold_s=2.0, rest_s=0.3)
        arm_tcps = {"left": (0.20, 0.28, 0.88), "right": (0.20, -0.28, 0.85)}

        # 1. Lay bridge across trench
        for _ in range(8):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=g.BLOCK_START_POS,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps,
            )
        self.assertTrue(st.bridge_seated)
        self.assertTrue(st.ever_25)
        self.assertFalse(st.ever_50)

        # 2. Block pushed onto bridge (crossing zone)
        block_on_bridge_pos = (0.280, 0.000, g.PLATFORM_TOP_Z + g.BRIDGE_SIZE[2] + g.BLOCK_SIZE[2] / 2.0)
        st = lg.step(
            dt=0.05,
            bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
            bridge_quat=(0.0, 0.0, 0.0, 1.0),
            bridge_vel=(0.0, 0.0, 0.0),
            block_pos=block_on_bridge_pos,
            block_quat=(0.0, 0.0, 0.0, 1.0),
            block_vel=(0.02, -0.05, 0.0),
            arm_tcps=arm_tcps,
        )
        self.assertTrue(st.block_on_bridge)
        self.assertTrue(st.ever_50)
        self.assertFalse(st.ever_75)

        # 3. Block pushed onto far platform
        block_far_pos = (0.300, -0.080, g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)
        st = lg.step(
            dt=0.05,
            bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
            bridge_quat=(0.0, 0.0, 0.0, 1.0),
            bridge_vel=(0.0, 0.0, 0.0),
            block_pos=block_far_pos,
            block_quat=(0.0, 0.0, 0.0, 1.0),
            block_vel=(0.01, -0.03, 0.0),
            arm_tcps=arm_tcps,
        )
        self.assertTrue(st.block_on_far)
        self.assertTrue(st.ever_75)

        # 4. Block on target mark, arms clear, held 2.0 s (40 steps)
        block_target = (g.TARGET_MARK_POS[0], g.TARGET_MARK_POS[1], g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)
        for _ in range(40):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=block_target,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps,
            )
        self.assertTrue(st.block_centered)
        self.assertTrue(st.ever_100)
        self.assertTrue(st.solved)

    def test_valid_order_2_block_moved_beside_trench_first(self):
        """Simulate Order 2: Block is first moved beside trench lip, then bridge is laid, then block crosses."""
        lg = logic.BridgeLogic(hold_s=2.0, rest_s=0.3)
        arm_tcps = {"left": (0.20, 0.28, 0.88), "right": (0.20, -0.28, 0.85)}

        # 1. Right arm moves block near trench lip first (y = +0.055, still on near platform)
        block_near_lip = (0.250, +0.055, g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)
        st = None
        for _ in range(8):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_START_POS,  # bridge still at start!
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=block_near_lip,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps,
            )
        self.assertFalse(st.fell)
        self.assertFalse(st.ever_25)
        self.assertFalse(st.ever_50)

        # 2. Left arm lays bridge across trench
        for _ in range(8):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=block_near_lip,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps,
            )
        self.assertTrue(st.bridge_seated)
        self.assertTrue(st.ever_25)

        # 3. Block is now pushed across bridge
        block_on_bridge_pos = (0.280, 0.000, g.PLATFORM_TOP_Z + g.BRIDGE_SIZE[2] + g.BLOCK_SIZE[2] / 2.0)
        st = lg.step(
            dt=0.05,
            bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
            bridge_quat=(0.0, 0.0, 0.0, 1.0),
            bridge_vel=(0.0, 0.0, 0.0),
            block_pos=block_on_bridge_pos,
            block_quat=(0.0, 0.0, 0.0, 1.0),
            block_vel=(0.02, -0.05, 0.0),
            arm_tcps=arm_tcps,
        )
        self.assertTrue(st.block_on_bridge)
        self.assertTrue(st.ever_50)

        # 4. Block onto far platform and onto target
        block_target = (g.TARGET_MARK_POS[0], g.TARGET_MARK_POS[1], g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)
        for _ in range(40):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=block_target,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps,
            )
        self.assertTrue(st.ever_75)
        self.assertTrue(st.ever_100)
        self.assertTrue(st.solved)

    def test_arms_clear_requirement(self):
        lg = logic.BridgeLogic(hold_s=2.0, rest_s=0.3)
        block_target = (g.TARGET_MARK_POS[0], g.TARGET_MARK_POS[1], g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)

        # First seat bridge
        for _ in range(8):
            lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=g.BLOCK_START_POS,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
            )

        # Right TCP right on top of target mark (< 60 mm): arms not clear!
        arm_tcps_touching = {"left": (0.20, 0.28, 0.88), "right": (g.TARGET_MARK_POS[0], g.TARGET_MARK_POS[1], 0.80)}
        for _ in range(40):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=block_target,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps_touching,
            )
        self.assertFalse(st.ever_100)
        self.assertFalse(st.solved)

        # Move arms away (> 60 mm) and hold 2.0 s: triggers ever_100
        arm_tcps_clear = {"left": (0.20, 0.28, 0.88), "right": (0.20, -0.28, 0.85)}
        for _ in range(40):
            st = lg.step(
                dt=0.05,
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                bridge_quat=(0.0, 0.0, 0.0, 1.0),
                bridge_vel=(0.0, 0.0, 0.0),
                block_pos=block_target,
                block_quat=(0.0, 0.0, 0.0, 1.0),
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps_clear,
            )
        self.assertTrue(st.ever_100)
        self.assertTrue(st.solved)


if __name__ == "__main__":
    unittest.main()
