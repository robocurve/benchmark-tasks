"""Unit tests for the thermal soak task logic: thermal ODEs, lag, hysteresis, and numerical calibration study."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
from logic import ThermalLogic


class TestThermalLogic(unittest.TestCase):
    def setUp(self):
        self.logic = ThermalLogic()

    def test_initial_state(self):
        s = self.logic.state
        self.assertEqual(s.tp, 20.0)
        self.assertFalse(s.heater_on)
        self.assertEqual(s.ts1, 20.0)
        self.assertEqual(s.tc1, 20.0)
        self.assertEqual(s.ts2, 20.0)
        self.assertEqual(s.tc2, 20.0)
        self.assertEqual(s.bar_count, 0)
        self.assertEqual(s.status_lamp, "off")
        self.assertFalse(s.fire_tripped)
        self.assertFalse(s.s1_complete)

    def test_button_hysteresis_and_edge_detection(self):
        # 1. Cap moves down to -5 mm: not triggered yet (-6.4 mm threshold)
        s = self.logic.step(0.05, -0.005, 0.0, False, False, False, False)
        self.assertFalse(s.heater_on)
        self.assertEqual(s.heat_count, 0)

        # 2. Cap moves to -7 mm (past -6.4 mm): rising edge triggers HEAT
        s = self.logic.step(0.05, -0.007, 0.0, False, False, False, False)
        self.assertTrue(s.heater_on)
        self.assertEqual(s.heat_count, 1)

        # 3. Holding at -7 mm: does NOT re-trigger (edge detector)
        s = self.logic.step(0.05, -0.007, 0.0, False, False, False, False)
        self.assertEqual(s.heat_count, 1)

        # 4. Moving up to -4 mm: below re-arm threshold (-3.5 mm) -> still pressed
        s = self.logic.step(0.05, -0.004, 0.0, False, False, False, False)
        self.assertTrue(s.heat_pressed)

        # 5. Moving up to -2 mm: above -3.5 mm -> re-arms
        s = self.logic.step(0.05, -0.002, 0.0, False, False, False, False)
        self.assertFalse(s.heat_pressed)

        # 6. Depressing again: second rising edge
        s = self.logic.step(0.05, -0.008, 0.0, False, False, False, False)
        self.assertEqual(s.heat_count, 2)

        # 7. Pressing OFF button turns heater OFF
        s = self.logic.step(0.05, 0.0, -0.007, False, False, False, False)
        self.assertFalse(s.heater_on)
        self.assertEqual(s.off_count, 1)

    def test_plate_heating_and_cooling_rates(self):
        # Turn heater ON and run for 10 s: heats at 10.0 C/s to 110 C setpoint
        self.logic.step(0.05, -0.007, 0.0, False, False, False, False)
        dt = 0.05
        for _ in range(int(10.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, False, False, False, False)

        # 20 + 10.0 * 9.0 = 110.0 C (clamped by thermostat setpoint)
        self.assertAlmostEqual(self.logic.state.tp, 110.0, delta=0.5)

        # Hold at setpoint 110 C
        for _ in range(int(20.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, False, False, False, False)
        self.assertAlmostEqual(self.logic.state.tp, 110.0, delta=0.1)

        # Turn OFF: falls at 1.5 C/s toward 20
        self.logic.step(dt, 0.0, -0.007, False, False, False, False)
        for _ in range(int(20.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, False, False, False, False)
        # 110 - 1.5 * 20 = 80 C
        self.assertAlmostEqual(self.logic.state.tp, 80.0, delta=0.5)

    def test_cool_pad_core_band_observable(self):
        """Status lamp shows core band while sample rests on cool pad: amber <60, green 60-70, red >70."""
        self.logic.reset()
        dt = 0.05
        # Set sample 1 on cool pad with varying core temp
        self.logic.state.s1_was_on_plate = True
        self.logic.state.s1_was_lifted = True

        # 1. Tc < 60 C -> amber
        self.logic.state.ts1 = 70.5
        self.logic.state.tc1 = 50.0
        s = self.logic.step(dt, 0.0, 0.0, False, True, False, False)
        self.assertEqual(s.status_lamp, "amber")
        self.assertEqual(s.bar_count, 5)  # 70 C = 5 discs (30,40,50,60,70)

        # 2. 60 <= Tc <= 70 C -> green
        self.logic.state.ts1 = 65.0
        self.logic.state.tc1 = 64.0
        s = self.logic.step(dt, 0.0, 0.0, False, True, False, False)
        self.assertEqual(s.status_lamp, "green")
        self.assertEqual(s.bar_count, 4)  # 65 C = 4 discs

        # 3. Tc > 70 C -> red
        self.logic.state.ts1 = 80.5
        self.logic.state.tc1 = 75.0
        s = self.logic.step(dt, 0.0, 0.0, False, True, False, False)
        self.assertEqual(s.status_lamp, "red")
        self.assertEqual(s.bar_count, 6)

    def test_dwell_robustness_window_pm_1_5s(self):
        """Verify that dwell +/- 1.5s around nominal 14.0s (12.5s, 14.0s, 15.5s) stays within 60-70 C."""
        dt = 0.05
        peaks = {}
        for dwell in (12.5, 14.0, 15.5):
            self.logic.reset()
            # Turn heat ON, plate at thermostat setpoint
            self.logic.step(dt, -0.007, 0.0, False, False, False, False)
            for _ in range(int(10.0 / dt)):
                self.logic.step(dt, 0.0, 0.0, False, False, False, False)

            # Sample on plate for dwell
            for _ in range(int(dwell / dt)):
                self.logic.step(dt, 0.0, 0.0, True, False, False, False)

            # Sample moved to cool pad: find peak core
            peak_tc = self.logic.state.tc1
            for _ in range(int(50.0 / dt)):
                s = self.logic.step(dt, 0.0, 0.0, False, True, False, False)
                peak_tc = max(peak_tc, s.tc1)
            peaks[dwell] = peak_tc

        # Verify +/- 1.5s stays inside 60-70 C
        self.assertGreaterEqual(peaks[12.5], 60.0, f"12.5s peak {peaks[12.5]:.2f} < 60.0")
        self.assertLessEqual(peaks[15.5], 70.0, f"15.5s peak {peaks[15.5]:.2f} > 70.0")
        self.assertTrue(60.0 <= peaks[14.0] <= 70.0)

    def test_study_condition_a_naive_lift_short_of_band(self):
        """Condition (a): lifting sample 1 early (dwell=5s) leaves core short of 60-70 band."""
        self.logic.reset()
        dt = 0.05
        self.logic.step(dt, -0.007, 0.0, False, False, False, False)
        for _ in range(int(10.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, False, False, False, False)

        # Early lift at 5.0 s
        for _ in range(int(5.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, True, False, False, False)

        # Lift from plate (not on plate) and monitor core
        peak_tc1 = self.logic.state.tc1
        for _ in range(int(30.0 / dt)):
            s = self.logic.step(dt, 0.0, 0.0, False, True, False, False)
            peak_tc1 = max(peak_tc1, s.tc1)

        self.assertLess(peak_tc1, 60.0)
        shortfall = 60.0 - peak_tc1
        self.assertGreaterEqual(shortfall, 8.0)

    def test_study_condition_b_study_lift_hits_target_band(self):
        """Condition (b): nominal dwell (14.0 s) on thermostat plate puts core inside 60-70 within 20 s post-lift."""
        self.logic.reset()
        dt = 0.05
        self.logic.step(dt, -0.007, 0.0, False, False, False, False)
        for _ in range(int(10.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, False, False, False, False)

        for _ in range(int(14.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, True, False, False, False)

        # Lift to cool pad
        tc_at_20s = 0.0
        for step in range(int(30.0 / dt)):
            s = self.logic.step(dt, 0.0, 0.0, False, True, False, False)
            if step == int(20.0 / dt):
                tc_at_20s = s.tc1

        # Inside 60-70 C at 20 s post-lift
        self.assertGreaterEqual(tc_at_20s, 60.0)
        self.assertLessEqual(tc_at_20s, 70.0)

    def test_study_condition_c_leaving_on_plate_trips_fire(self):
        """Condition (c): leaving a sample on heating plate until surface shows 100+ C gives core > 90 (FIRE)."""
        self.logic.reset()
        dt = 0.05
        self.logic.step(dt, -0.007, 0.0, True, False, False, False)
        for _ in range(int(60.0 / dt)):
            self.logic.step(dt, 0.0, 0.0, True, False, False, False)

        self.assertTrue(self.logic.state.fire_tripped)
        self.assertGreater(self.logic.state.tc1, 90.0)

    def test_temperature_bar_and_status_lamp(self):
        self.logic.reset()
        dt = 0.05

        # Ambient: 0 discs
        s = self.logic.step(dt, 0.0, 0.0, True, False, False, False)
        self.assertEqual(s.bar_count, 0)
        self.assertEqual(s.status_lamp, "off")

        # Heating with sample on plate: status turns amber
        s = self.logic.step(dt, -0.007, 0.0, True, False, False, False)
        self.assertEqual(s.status_lamp, "amber")

        # Heat until Ts >= 30: 1 disc
        while self.logic.state.ts1 < 30.5:
            s = self.logic.step(dt, 0.0, 0.0, True, False, False, False)
        self.assertEqual(s.bar_count, 1)
        self.assertTrue(s.bar_lit[0])

        # Heat until Ts >= 80: 6 discs (discs > 70 are red)
        while self.logic.state.ts1 < 80.5:
            s = self.logic.step(dt, 0.0, 0.0, True, False, False, False)
        self.assertEqual(s.bar_count, 6)
        # Discs 1-5 (k=0..4) and Disc 6 (k=5) lit
        self.assertTrue(all(s.bar_lit[:6]))
        self.assertFalse(any(s.bar_lit[6:]))


if __name__ == "__main__":
    unittest.main()
