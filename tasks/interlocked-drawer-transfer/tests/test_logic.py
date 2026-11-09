"""Unit tests for DrawerTransferLogic in interlocked-drawer-transfer."""

import unittest
import numpy as np

import geometry as g
from logic import DrawerTransferLogic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.logic = DrawerTransferLogic()
        self.dt = 0.05

    def test_initial_state(self):
        st = self.logic.step(
            dt=self.dt,
            disp_left=0.0,
            disp_right=0.0,
            block_pos=g.BLOCK_START_POS,
        )
        self.assertTrue(st.is_left_closed)
        self.assertTrue(st.is_right_closed)
        self.assertFalse(st.is_left_open)
        self.assertFalse(st.is_right_open)
        self.assertFalse(st.tip_fault)
        self.assertFalse(st.block_dropped)
        self.assertFalse(st.stage_0_25)
        self.assertFalse(st.stage_0_50)
        self.assertFalse(st.stage_0_75)
        self.assertFalse(st.solved)
        self.assertTrue(st.lamp_ok)
        self.assertFalse(st.lamp_tip)

    def test_open_and_closed_thresholds(self):
        # 10 mm is not open (> 15 mm required)
        st = self.logic.step(self.dt, disp_left=0.010, disp_right=0.0, block_pos=g.BLOCK_START_POS)
        self.assertFalse(st.is_left_open)
        self.assertFalse(st.is_left_closed)  # not closed either (> 5 mm)

        # 16 mm is open
        st = self.logic.step(self.dt, disp_left=0.016, disp_right=0.0, block_pos=g.BLOCK_START_POS)
        self.assertTrue(st.is_left_open)

        # 4 mm is closed
        st = self.logic.step(self.dt, disp_left=0.004, disp_right=0.0, block_pos=g.BLOCK_START_POS)
        self.assertTrue(st.is_left_closed)

    def test_tip_fault_requires_simultaneous_open_duration(self):
        # Only left open for 1.0 s -> NO tip fault
        for _ in range(20):
            st = self.logic.step(self.dt, disp_left=0.080, disp_right=0.0, block_pos=g.BLOCK_START_POS)
        self.assertFalse(st.tip_fault)
        self.assertTrue(st.lamp_ok)

        # Transient crossing: both open for 0.1 s (2 steps at 0.05 s < 0.2 s), then right closes
        self.logic.step(self.dt, disp_left=0.080, disp_right=0.020, block_pos=g.BLOCK_START_POS)
        st = self.logic.step(self.dt, disp_left=0.080, disp_right=0.020, block_pos=g.BLOCK_START_POS)
        self.assertFalse(st.tip_fault)

        # Right closes
        st = self.logic.step(self.dt, disp_left=0.080, disp_right=0.000, block_pos=g.BLOCK_START_POS)
        self.assertFalse(st.tip_fault)

        # Now both open continuously for >= 0.2 s (4 steps of 0.05 s = 0.20 s)
        for _ in range(4):
            st = self.logic.step(self.dt, disp_left=0.080, disp_right=0.030, block_pos=g.BLOCK_START_POS)
        self.assertTrue(st.tip_fault)
        self.assertFalse(st.lamp_ok)
        self.assertTrue(st.lamp_tip)

        # Latching: closing drawers does not clear tip fault
        st = self.logic.step(self.dt, disp_left=0.0, disp_right=0.0, block_pos=g.BLOCK_START_POS)
        self.assertTrue(st.tip_fault)

    def test_stage_0_25_left_open_and_block_lifted(self):
        # Left open < 70 mm -> no stage 0.25
        st = self.logic.step(self.dt, disp_left=0.050, disp_right=0.0, block_pos=(0.350, g.DRAWER_LEFT_Y, 0.860))
        self.assertFalse(st.stage_0_25)

        # Left open >= 70 mm but block still on floor -> no stage 0.25
        st = self.logic.step(self.dt, disp_left=0.080, disp_right=0.0, block_pos=g.BLOCK_START_POS)
        self.assertFalse(st.stage_0_25)

        # Left open >= 70 mm AND block lifted above rim -> stage 0.25
        st = self.logic.step(self.dt, disp_left=0.080, disp_right=0.0, block_pos=(0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        self.assertTrue(st.stage_0_25)

    def test_stage_0_50_two_valid_orders(self):
        # Solution 1: Carry order (block held outside while left drawer closed)
        logic_carry = DrawerTransferLogic()
        # 1. Open left and lift block
        logic_carry.step(self.dt, disp_left=0.080, disp_right=0.0, block_pos=(0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        # 2. Close left drawer while holding block at carry height
        st_carry = logic_carry.step(self.dt, disp_left=0.002, disp_right=0.0, block_pos=(0.280, 0.050, g.HOVER_Z))
        self.assertTrue(st_carry.stage_0_50)

        # Solution 2: Staging pad order (block parked on staging pad while left drawer closed)
        logic_park = DrawerTransferLogic()
        # 1. Open left and lift block
        logic_park.step(self.dt, disp_left=0.080, disp_right=0.0, block_pos=(0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        # 2. Park on staging pad and close left drawer
        st_park = logic_park.step(self.dt, disp_left=0.002, disp_right=0.0, block_pos=g.STAGING_SPOT)
        self.assertTrue(st_park.stage_0_50)
        self.assertTrue(st_park.block_on_staging)

    def test_stage_0_75_right_drawer_open(self):
        # Reach 0.50
        self.logic.step(self.dt, disp_left=0.080, disp_right=0.0, block_pos=(0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        self.logic.step(self.dt, disp_left=0.002, disp_right=0.0, block_pos=g.STAGING_SPOT)

        # Open right drawer >= 70 mm
        st = self.logic.step(self.dt, disp_left=0.002, disp_right=0.085, block_pos=g.STAGING_SPOT)
        self.assertTrue(st.stage_0_75)

    def test_solved_condition(self):
        # Execute valid sequence
        self.logic.step(self.dt, disp_left=0.080, disp_right=0.0, block_pos=(0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        self.logic.step(self.dt, disp_left=0.002, disp_right=0.0, block_pos=g.STAGING_SPOT)
        self.logic.step(self.dt, disp_left=0.002, disp_right=0.085, block_pos=g.STAGING_SPOT)

        # Place block in right drawer and close both
        block_in_right = (0.350, g.DRAWER_RIGHT_Y, g.BLOCK_REST_Z)
        arm_tcps = {"left": (0.200, 0.280, g.HOVER_Z), "right": (0.200, -0.280, g.HOVER_Z)}
        st = self.logic.step(self.dt, disp_left=0.002, disp_right=0.002, block_pos=block_in_right, arm_tcps=arm_tcps)

        self.assertTrue(st.block_in_right_drawer)
        self.assertTrue(st.both_drawers_closed)
        self.assertTrue(st.arms_clear)
        self.assertFalse(st.tip_fault)
        self.assertFalse(st.block_dropped)
        self.assertTrue(st.solved)

    def test_block_dropped_zero_gate(self):
        st = self.logic.step(self.dt, disp_left=0.0, disp_right=0.0, block_pos=(0.200, 0.0, g.TOP - 0.080))
        self.assertTrue(st.block_dropped)
        self.assertFalse(st.solved)


if __name__ == "__main__":
    unittest.main()
