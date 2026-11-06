"""Tests for state machine and safety logic in lockout-tagout-before-service."""

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.LotoLogic()

    def test_initial_state(self):
        st = self.lg.step(
            dt=0.05,
            breaker_disp=0.0,
            pin_pos=g.PIN_STAND_POS,
            test_btn_disp=0.0,
            cover_disp=0.0,
            service_btn_disp=0.0,
        )
        self.assertFalse(st.breaker_off)
        self.assertTrue(st.live_lamp_lit)
        self.assertFalse(st.pin_seated)
        self.assertFalse(st.dead_lamp_lit)
        self.assertFalse(st.fault)
        self.assertFalse(st.solved)

    def test_breaker_switch_off(self):
        st = self.lg.step(
            dt=0.05,
            breaker_disp=0.035,
            pin_pos=g.PIN_STAND_POS,
            test_btn_disp=0.0,
            cover_disp=0.0,
            service_btn_disp=0.0,
        )
        self.assertTrue(st.breaker_off)
        self.assertFalse(st.live_lamp_lit)
        self.assertFalse(st.fault)

    def test_pin_insertion_requires_breaker_off(self):
        # Breaker ON, pin at hole -> hole blocked
        st = self.lg.step(
            dt=0.05,
            breaker_disp=0.0,
            pin_pos=(g.HOLE_POS[0], g.HOLE_POS[1], g.PIN_SEATED_Z),
            test_btn_disp=0.0,
            cover_disp=0.0,
            service_btn_disp=0.0,
        )
        self.assertFalse(st.pin_seated)

        # Breaker OFF, pin at hole -> pin seated!
        st = self.lg.step(
            dt=0.05,
            breaker_disp=0.035,
            pin_pos=(g.HOLE_POS[0], g.HOLE_POS[1], g.PIN_SEATED_Z),
            test_btn_disp=0.0,
            cover_disp=0.0,
            service_btn_disp=0.0,
        )
        self.assertTrue(st.pin_seated)

    def test_test_button_pressed_live_causes_fault(self):
        # TEST pressed while breaker is ON -> FAULT
        st = self.lg.step(
            dt=0.05,
            breaker_disp=0.0,
            pin_pos=g.PIN_STAND_POS,
            test_btn_disp=-0.007,
            cover_disp=0.0,
            service_btn_disp=0.0,
        )
        self.assertTrue(st.fault)
        self.assertEqual(st.fault_reason, "test_pressed_live")

    def test_test_button_pressed_dead_lights_dead_lamp(self):
        # Switch OFF first
        self.lg.step(0.05, 0.035, g.PIN_STAND_POS, 0.0, 0.0, 0.0)
        # Press TEST
        st = self.lg.step(0.05, 0.035, g.PIN_STAND_POS, -0.007, 0.0, 0.0)
        self.assertTrue(st.test_pressed_dead)
        self.assertTrue(st.dead_lamp_lit)
        self.assertAlmostEqual(st.dead_lamp_timer, 2.95, places=2)

    def test_cover_opened_before_prerequisites_causes_fault(self):
        # Case 1: Cover opened while breaker is ON
        lg1 = logic.LotoLogic()
        st1 = lg1.step(0.05, 0.0, g.PIN_STAND_POS, 0.0, 0.025, 0.0)
        self.assertTrue(st1.fault)
        self.assertEqual(st1.fault_reason, "cover_opened_breaker_on")

        # Case 2: Cover opened while breaker OFF but pin not seated
        lg2 = logic.LotoLogic()
        lg2.step(0.05, 0.035, g.PIN_STAND_POS, 0.0, 0.0, 0.0)
        st2 = lg2.step(0.05, 0.035, g.PIN_STAND_POS, 0.0, 0.025, 0.0)
        self.assertTrue(st2.fault)
        self.assertEqual(st2.fault_reason, "cover_opened_pin_not_seated")

        # Case 3: Cover opened while breaker OFF, pin seated, but before TEST
        lg3 = logic.LotoLogic()
        lg3.step(0.05, 0.035, (g.HOLE_POS[0], g.HOLE_POS[1], g.PIN_SEATED_Z), 0.0, 0.0, 0.0)
        st3 = lg3.step(0.05, 0.035, (g.HOLE_POS[0], g.HOLE_POS[1], g.PIN_SEATED_Z), 0.0, 0.025, 0.0)
        self.assertTrue(st3.fault)
        self.assertEqual(st3.fault_reason, "cover_opened_before_test")

    def test_full_successful_loto_sequence(self):
        pin_hole = (g.HOLE_POS[0], g.HOLE_POS[1], g.PIN_SEATED_Z)
        # Step 1: Switch breaker OFF
        self.lg.step(0.05, 0.035, g.PIN_STAND_POS, 0.0, 0.0, 0.0)
        # Step 2: Seat pin
        self.lg.step(0.05, 0.035, pin_hole, 0.0, 0.0, 0.0)
        # Step 3: Press TEST
        self.lg.step(0.05, 0.035, pin_hole, -0.007, 0.0, 0.0)
        # Step 4: Open cover >= 70 mm
        self.lg.step(0.05, 0.035, pin_hole, 0.0, 0.075, 0.0)
        # Step 5: Press and hold SERVICE button for 1.0 s (20 steps)
        for _ in range(20):
            st = self.lg.step(0.05, 0.035, pin_hole, 0.0, 0.075, -0.007)
        self.assertTrue(st.service_pressed)
        self.assertTrue(st.solved)
        self.assertFalse(st.fault)


if __name__ == "__main__":
    unittest.main()

