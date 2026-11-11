"""Tests for state machine, boundary safety checks, and dual valid execution orders in hook-block-from-guarded-zone."""

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.HookBlockLogic(hold_s=2.0, rest_s=0.3)

    def test_initial_state(self):
        st = self.lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            block_vel=(0.0, 0.0, 0.0),
            rod_vel=(0.0, 0.0, 0.0),
            arm_tcps={"left": (0.20, 0.28, 0.85), "right": (0.20, -0.28, 0.85)},
        )
        self.assertFalse(st.fault)
        self.assertFalse(st.rod_held)
        self.assertFalse(st.block_moved_60mm)
        self.assertFalse(st.block_outside_zone)
        self.assertFalse(st.block_in_tray)
        self.assertFalse(st.solved)
        self.assertEqual(st.score, 0.0)

    def test_boundary_check_left_tcp_fault(self):
        # Left TCP crosses past yellow line threshold (x > 0.410)
        st = self.lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            arm_tcps={"left": (0.415, 0.10, 0.80), "right": (0.20, -0.28, 0.85)},
        )
        self.assertTrue(st.fault)
        self.assertIn("left", st.fault_reason)
        self.assertEqual(st.score, 0.0)
        self.assertFalse(st.solved)

    def test_boundary_check_right_tcp_fault(self):
        # Right TCP crosses past yellow line threshold
        st = self.lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            arm_tcps={"left": (0.20, 0.28, 0.85), "right": (0.420, -0.10, 0.80)},
        )
        self.assertTrue(st.fault)
        self.assertIn("right", st.fault_reason)
        self.assertEqual(st.score, 0.0)

    def test_boundary_check_gripper_body_fault(self):
        # Gripper matrix x > 0.410
        grip_mat = np.eye(4)
        grip_mat[0, 3] = 0.415
        st = self.lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            arm_tcps={"left": (0.36, 0.10, 0.80), "right": (0.20, -0.28, 0.85)},
            gripper_poses={"left": grip_mat},
        )
        self.assertTrue(st.fault)
        self.assertIn("gripper", st.fault_reason)
        self.assertEqual(st.score, 0.0)

    def test_boundary_safe_boundary_edge(self):
        # TCP exactly at x = 0.400 or 0.409 does NOT fault
        st = self.lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            arm_tcps={"left": (0.400, 0.10, 0.80), "right": (0.409, -0.10, 0.80)},
        )
        self.assertFalse(st.fault)

    def test_fault_latches_permanently(self):
        # Once faulted, returning to safe stance still leaves fault latched
        self.lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            arm_tcps={"left": (0.440, 0.18, 0.82), "right": (0.20, -0.28, 0.85)},
        )
        self.assertTrue(self.lg.state.fault)

        st = self.lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            arm_tcps={"left": (0.20, 0.28, 0.85), "right": (0.20, -0.28, 0.85)},
        )
        self.assertTrue(st.fault)
        self.assertEqual(st.score, 0.0)

    def test_ladder_milestone_progression(self):
        safe_tcps = {"left": (0.21, 0.25, 0.80), "right": (0.20, -0.28, 0.85)}

        # 1. Rod held (lifted) -> 0.25
        rod_lifted = (0.21, 0.25, g.TOP + 0.05)
        st = self.lg.step(dt=0.05, block_pos=g.BLOCK_INIT_POS, rod_pos=rod_lifted, arm_tcps=safe_tcps)
        self.assertTrue(st.rod_held)
        self.assertEqual(st.score, 0.25)

        # 2. Block moved >= 60 mm (from 0.58 to 0.51) -> 0.50
        block_moved = (0.510, 0.0, g.TOP + g.BLOCK_HEIGHT / 2)
        st = self.lg.step(dt=0.05, block_pos=block_moved, rod_pos=rod_lifted, arm_tcps=safe_tcps)
        self.assertTrue(st.block_moved_60mm)
        self.assertEqual(st.score, 0.50)

        # 3. Block outside zone (x < 0.380 resting) -> 0.75
        block_outside = (0.360, 0.0, g.TOP + g.BLOCK_HEIGHT / 2)
        for _ in range(7):  # 7 * 0.05 = 0.35 s resting > 0.3 s
            st = self.lg.step(
                dt=0.05,
                block_pos=block_outside,
                rod_pos=rod_lifted,
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=safe_tcps,
            )
        self.assertTrue(st.block_outside_zone)
        self.assertEqual(st.score, 0.75)

    def test_two_valid_orders_right_arm_placing_block(self):
        """Standard sequence: Left arm retrieves block with rod; Right arm places in tray."""
        lg = logic.HookBlockLogic(hold_s=2.0)

        # 1. Left arm lifts rod
        st = lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=(0.21, 0.25, g.TOP + 0.05),
            arm_tcps={"left": (0.21, 0.25, g.TOP + 0.05), "right": (0.20, -0.28, 0.85)},
        )
        self.assertEqual(st.score, 0.25)

        # 2. Left arm pulls block out to x=0.36
        st = lg.step(
            dt=0.05,
            block_pos=(0.36, 0.0, g.TOP + g.BLOCK_HEIGHT / 2),
            rod_pos=(0.16, 0.0, g.TOP + 0.01),
            arm_tcps={"left": (0.16, 0.0, g.TOP + 0.01), "right": (0.20, -0.28, 0.85)},
        )
        self.assertEqual(st.score, 0.50)

        # Settle block resting outside zone
        for _ in range(7):
            st = lg.step(
                dt=0.05,
                block_pos=(0.36, 0.0, g.TOP + g.BLOCK_HEIGHT / 2),
                rod_pos=(0.22, 0.25, g.ROD_PARK_POS[2]),
                block_vel=(0.0, 0.0, 0.0),
                rod_vel=(0.0, 0.0, 0.0),
                arm_tcps={"left": (0.20, 0.28, 0.85), "right": (0.20, -0.28, 0.85)},
            )
        self.assertEqual(st.score, 0.75)

        # 3. Right arm transports block to green tray and releases
        block_in_tray = (g.TRAY_POS[0], g.TRAY_POS[1], g.TRAY_SEAT_Z)
        clear_tcps = {"left": (0.20, 0.28, 0.85), "right": (0.20, -0.28, 0.85)}

        # Hold for 2.0 s (40 steps)
        for _ in range(40):
            st = lg.step(
                dt=0.05,
                block_pos=block_in_tray,
                rod_pos=g.ROD_PARK_POS,
                block_vel=(0.0, 0.0, 0.0),
                rod_vel=(0.0, 0.0, 0.0),
                arm_tcps=clear_tcps,
            )
        self.assertTrue(st.solved)
        self.assertEqual(st.score, 1.00)

    def test_two_valid_orders_left_arm_placing_block(self):
        """Alternative sequence: Left arm retrieves block with rod, parks rod, then Left arm places in tray."""
        lg = logic.HookBlockLogic(hold_s=2.0)

        # 1. Left arm lifts rod
        st = lg.step(
            dt=0.05,
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=(0.21, 0.25, g.TOP + 0.05),
            arm_tcps={"left": (0.21, 0.25, g.TOP + 0.05), "right": (0.20, -0.28, 0.85)},
        )
        self.assertEqual(st.score, 0.25)

        # 2. Left arm pulls block to x=0.36
        st = lg.step(
            dt=0.05,
            block_pos=(0.36, 0.0, g.TOP + g.BLOCK_HEIGHT / 2),
            rod_pos=(0.16, 0.0, g.TOP + 0.01),
            arm_tcps={"left": (0.16, 0.0, g.TOP + 0.01), "right": (0.20, -0.28, 0.85)},
        )
        self.assertEqual(st.score, 0.50)

        # Left arm sets rod down at park
        for _ in range(7):
            st = lg.step(
                dt=0.05,
                block_pos=(0.36, 0.0, g.TOP + g.BLOCK_HEIGHT / 2),
                rod_pos=g.ROD_PARK_POS,
                block_vel=(0.0, 0.0, 0.0),
                rod_vel=(0.0, 0.0, 0.0),
                arm_tcps={"left": (0.20, 0.28, 0.85), "right": (0.20, -0.28, 0.85)},
            )
        self.assertEqual(st.score, 0.75)

        # 3. Left arm picks block and places into tray
        block_in_tray = (g.TRAY_POS[0], g.TRAY_POS[1], g.TRAY_SEAT_Z)
        clear_tcps = {"left": (0.20, 0.28, 0.85), "right": (0.20, -0.28, 0.85)}

        for _ in range(40):
            st = lg.step(
                dt=0.05,
                block_pos=block_in_tray,
                rod_pos=g.ROD_PARK_POS,
                block_vel=(0.0, 0.0, 0.0),
                rod_vel=(0.0, 0.0, 0.0),
                arm_tcps=clear_tcps,
            )
        self.assertTrue(st.solved)
        self.assertEqual(st.score, 1.00)


if __name__ == "__main__":
    unittest.main()
