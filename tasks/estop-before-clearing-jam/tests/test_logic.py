"""Unit tests for state machine, pusher motion, interlocks, and safety logic."""

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.JamClearLogic()

    def test_initial_state(self):
        st = self.lg.step(
            dt=0.05,
            estop_disp=0.0,
            reset_disp=0.0,
            start_disp=0.0,
            block_pos=g.BLOCK_INIT_POS,
            arm_tcps={"left": g.HOME_L, "right": g.HOME_R},
        )
        self.assertTrue(st.pusher_moving)
        self.assertFalse(st.ever_stopped)
        self.assertTrue(st.run_lamp_lit)
        self.assertFalse(st.stop_lamp_lit)
        self.assertFalse(st.fault_lamp_lit)
        self.assertTrue(st.block_in_lane)
        self.assertFalse(st.block_cleared)
        self.assertFalse(st.fault)
        self.assertFalse(st.solved)

    def test_pusher_sweeps_over_time(self):
        # Step 1.0 s forward
        for _ in range(20):
            st = self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.pusher_moving)
        self.assertGreater(st.pusher_x, g.PUSHER_X_MIN)

    def test_estop_press_stops_pusher_within_0_3_s(self):
        # Press E-STOP at step 0
        st = self.lg.step(0.05, -0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.estop_pressed)
        # Should be stopping
        self.assertTrue(st.pusher_stopping)

        # After 6 steps (0.30 s total)
        for _ in range(5):
            st = self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})

        self.assertFalse(st.pusher_moving)
        self.assertTrue(st.ever_stopped)
        self.assertFalse(st.run_lamp_lit)
        self.assertTrue(st.stop_lamp_lit)
        self.assertFalse(st.fault)

    def test_reaching_in_while_pusher_moving_causes_fault(self):
        # Left arm reaches into lane at step 0 while pusher is moving
        tcp_in_danger = (0.400, 0.100, g.TOP + 0.050)
        st = self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": tcp_in_danger, "right": g.HOME_R})
        self.assertTrue(st.fault)
        self.assertIn("danger_zone", st.fault_reason)
        self.assertTrue(st.fault_lamp_lit)
        self.assertFalse(st.run_lamp_lit)
        self.assertFalse(st.stop_lamp_lit)

    def test_reaching_in_after_pusher_stopped_is_safe(self):
        # 1. Stop pusher with E-STOP
        self.lg.step(0.05, -0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            st = self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.ever_stopped)
        self.assertFalse(st.pusher_moving)

        # 2. Left arm enters danger zone to pick block
        tcp_in_danger = (0.400, 0.100, g.TOP + 0.050)
        st = self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": tcp_in_danger, "right": g.HOME_R})
        self.assertFalse(st.fault)
        self.assertFalse(st.fault_lamp_lit)

    def test_reset_before_estop_has_no_effect(self):
        # Press RESET while pusher is running
        st = self.lg.step(0.05, 0.0, -0.007, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertFalse(st.reset_pressed)

    def test_start_without_reset_has_no_effect(self):
        # Stop pusher
        self.lg.step(0.05, -0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})

        # Clear block to tray
        tray_block_pos = (g.TRAY_POS[0], g.TRAY_POS[1], g.TOP + 0.020)

        # Press START directly without RESET
        st = self.lg.step(0.05, 0.0, 0.0, -0.007, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertFalse(st.start_pressed)
        self.assertFalse(st.restarted)
        self.assertFalse(st.pusher_moving)

    def test_start_with_block_still_in_lane_causes_fault(self):
        # Stop pusher
        self.lg.step(0.05, -0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})

        # Press RESET (block still in lane)
        st = self.lg.step(0.05, 0.0, -0.007, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.reset_pressed)

        # Press START while block is still in lane -> FAULT
        st = self.lg.step(0.05, 0.0, 0.0, -0.007, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.fault)
        self.assertEqual(st.fault_reason, "start_pressed_with_block_in_lane")
        self.assertTrue(st.fault_lamp_lit)

    def test_full_successful_recovery_sequence_tray(self):
        # 1. Press E-STOP
        self.lg.step(0.05, -0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            st = self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.ever_stopped)

        # 2. Move block to green tray
        tray_block_pos = (g.TRAY_POS[0], g.TRAY_POS[1], g.TOP + 0.020)
        st = self.lg.step(0.05, 0.0, 0.0, 0.0, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.block_cleared)

        # 3. Press RESET
        st = self.lg.step(0.05, 0.0, -0.007, 0.0, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.reset_pressed)

        # 4. Press START
        st = self.lg.step(0.05, 0.0, 0.0, -0.007, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.start_pressed)
        self.assertTrue(st.restarted)
        self.assertTrue(st.pusher_moving)
        self.assertTrue(st.run_lamp_lit)
        self.assertFalse(st.stop_lamp_lit)
        self.assertTrue(st.solved)
        self.assertFalse(st.fault)

    def test_alternative_solution_parked_pad(self):
        """Second valid order: block extracted to parked pad instead of green tray."""
        # 1. Press E-STOP
        self.lg.step(0.05, -0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            self.lg.step(0.05, 0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})

        # 2. Move block to parked pad
        pad_block_pos = (g.PARK_PAD_POS[0], g.PARK_PAD_POS[1], g.TOP + 0.015)
        st = self.lg.step(0.05, 0.0, 0.0, 0.0, pad_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.block_cleared)

        # 3. Press RESET then START
        self.lg.step(0.05, 0.0, -0.007, 0.0, pad_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        st = self.lg.step(0.05, 0.0, 0.0, -0.007, pad_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertTrue(st.solved)


if __name__ == "__main__":
    unittest.main()
