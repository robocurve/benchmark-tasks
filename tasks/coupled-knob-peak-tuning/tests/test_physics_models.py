"""Models that decide whether the first GPU run can work: READ cap travel against arm compliance (N1), the wedge-shaped
closed fingers against the knobs and end stops (N2/N4), push forces (N1 follow-up), spawn overlaps. Pure Python."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import demo_plan
import geometry as g
import logic


class TestButtonCompliance(unittest.TestCase):
    def test_equilibrium_cap_travel_reaches_trigger_with_margin(self):
        """N1: the arm (319.6 N/m at the press pose) must push the cap past the 6.4 mm trigger by >= 1 mm."""
        trigger = g.BUTTON_TRIGGER_TRAVEL
        self.assertAlmostEqual(trigger, -logic.BUTTON_PRESSED_THRESHOLD, places=9)
        travel = g.button_equilibrium_travel()
        self.assertGreaterEqual(travel, trigger + 0.001, f"cap travel {travel * 1000:.2f} mm")
        # still enough if the arm were only half as stiff as computed
        self.assertGreaterEqual(g.button_equilibrium_travel(k_arm=0.5 * g.ARM_Z_STIFFNESS), trigger + 0.001)

    def test_old_design_could_not_register(self):
        """Regression for N1: 625 N/m cap spring with a 7 mm command gives about 2.4 mm of travel, never 6.4 mm."""
        old = g.button_equilibrium_travel(depth_cmd=0.007, k_spring=625.0)
        self.assertAlmostEqual(old * 1000, 2.4, delta=0.1)
        self.assertLess(old, g.BUTTON_TRIGGER_TRAVEL)

    def test_spring_is_much_softer_than_the_arm(self):
        self.assertLessEqual(g.BUTTON_RATE, 100.0)
        self.assertLess(g.BUTTON_RATE, g.ARM_Z_STIFFNESS / 3)

    def test_plan_commands_the_cap_to_its_end_stop(self):
        """The commanded press z puts the pad BUTTON_PRESS_OVERSHOOT below the stop (the stop sets the travel)."""
        os.environ.pop("DEMO_PRESS_OVERSHOOT_MM", None)
        z = demo_plan._z_press()
        pad_bottom = z - g.PAD_TO_TCP
        stop_top = g.BUTTON_CAP_TOP_REST_Z - g.BUTTON_TRAVEL
        self.assertAlmostEqual(stop_top - pad_bottom, g.BUTTON_PRESS_OVERSHOOT, places=9)
        # the arm's force beyond the stop is positive and gentle
        f_arm = g.BUTTON_PRESS_OVERSHOOT * g.ARM_Z_STIFFNESS + g.BUTTON_RATE * g.BUTTON_TRAVEL
        self.assertGreater(f_arm, g.BUTTON_RATE * g.BUTTON_TRAVEL)
        self.assertLess(f_arm, 5.0)

    def test_cap_stop_is_flush_with_the_pocket_ring_and_nothing_solid_is_under_the_cap(self):
        self.assertAlmostEqual(g.BUTTON_CAP_TOP_REST_Z - g.BUTTON_TRAVEL, g.BUTTON_BASE_TOP_Z, places=9)
        cap_lo = g.BUTTON_CAP_REST_CZ - g.BUTTON_CAP_H / 2 - g.BUTTON_TRAVEL
        cap_hi = g.BUTTON_CAP_REST_CZ + g.BUTTON_CAP_H / 2
        half = g.BUTTON_CAP_L / 2
        for name, c, sz, _col in g.fixture_named_boxes_world():
            overlap = all(
                min(c[i] + sz[i] / 2, hi) - max(c[i] - sz[i] / 2, lo) > 1e-9
                for i, (lo, hi) in enumerate(
                    ((g.BUTTON_BASE_CX - half, g.BUTTON_BASE_CX + half), (g.BUTTON_BASE_CY - half, g.BUTTON_BASE_CY + half), (cap_lo, cap_hi))
                )
            )
            self.assertFalse(overlap, f"fixture box {name!r} overlaps the swept volume of the READ cap")
        self.assertAlmostEqual((g.BUTTON_BASE_L - 2 * g.POCKET_WALL_T - g.BUTTON_CAP_L) / 2, 0.001, places=9)


class TestKnobPush(unittest.TestCase):
    def test_push_forces_are_gentle_and_keep_the_finger_low(self):
        m = g.push_force_model()
        self.assertLess(m["horizontal"], 1.0)  # N
        self.assertLess(m["finger_lift_mm"], 1.0)  # the finger rides up less than 1 mm (pad starts 2.5 mm up)
        # the command overshoot is half the stall lag, so the knob lands within +/- overshoot of the plan
        self.assertAlmostEqual(g.PUSH_OVERSHOOT * 1000, m["stall_lag_mm"] / 2, delta=0.1)

    def test_contact_is_below_the_tipping_height_even_for_the_worst_friction_combine(self):
        # tipping when the push height exceeds (L/2)/mu_eff; mu_eff in {0.5 (min/multiply), 0.75 (average), 1.0 (max)}
        self.assertLessEqual(g.KNOB_H, g.KNOB_L / 2 / 1.0)

    def test_wedge_profile_and_contact_point(self):
        # the fingers are wider higher up
        self.assertLess(g.finger_half_depth(0.003), g.finger_half_depth(0.0115))
        self.assertAlmostEqual(g.push_contact_half_depth() * 1000, 11.25, delta=0.2)

    def test_descent_does_not_touch_the_knob_forward_and_backward(self):
        os.environ.pop("DEMO_PUSH_OVERSHOOT_MM", None)
        w = g.push_contact_half_depth()
        for s_from in (30.0, 55.0):
            x_from, _ = demo_plan.push_targets(s_from, s_from + 10)
            rear = g.CHANNEL_NEAR_STOP_X + s_from / 1000
            self.assertAlmostEqual(rear - (x_from + w), g.PUSH_APPROACH_CLEAR, places=9)
        for s_from in (60.0, 72.0):
            x_from, _ = demo_plan.push_targets(s_from, s_from - 10)
            front = g.CHANNEL_NEAR_STOP_X + s_from / 1000 + g.KNOB_L
            self.assertAlmostEqual((x_from - w) - front, g.PUSH_APPROACH_CLEAR, places=9)

    def test_first_forward_push_clears_the_near_end_stop_by_2mm(self):
        """N2/REVIEW 2: at wall-top height the wedge is wider than at the knob top edge; it must still clear the stop."""
        wall_h = g.TRACK_WALL_H - g.TRACK_FLOOR_T - g.PUSH_PAD_HEIGHT
        x_from, _ = demo_plan.push_targets(g.KNOB_START_SETTING_MM, g.KNOB_START_SETTING_MM + 10)
        gap = (x_from - g.finger_half_depth(wall_h)) - g.CHANNEL_NEAR_STOP_X
        self.assertGreaterEqual(gap, 0.002, f"gap {gap * 1000:.2f} mm")

    def test_backward_push_limit_matches_the_far_end_stop(self):
        wall_h = g.TRACK_WALL_H - g.TRACK_FLOOR_T - g.PUSH_PAD_HEIGHT
        s = demo_plan.BACKWARD_MAX_S_MM
        x_from, _ = demo_plan.push_targets(s, s - 10)
        gap = g.CHANNEL_FAR_STOP_X - (x_from + g.finger_half_depth(wall_h))
        self.assertGreaterEqual(gap, 0.001, f"gap {gap * 1000:.2f} mm")
        # one more 5 mm and it would not fit with 1 mm to spare: the limit is not set loosely
        x2, _ = demo_plan.push_targets(s + 5, s - 5)
        self.assertLess(g.CHANNEL_FAR_STOP_X - (x2 + g.finger_half_depth(wall_h)), 0.001)

    def test_knobs_never_overlap_the_fixture_at_any_setting(self):
        """REVIEW 8: no spawn penetration; the knob box is clear of every fixture box over the whole travel."""
        for side in ("left", "right"):
            for s in range(0, 101, 5):
                c = (g.knob_cx_from_setting_mm(s), g.TRACK_Y[side], g.KNOB_CZ)
                for name, fc, sz, _col in g.fixture_named_boxes_world():
                    ov = all(
                        min(c[i] + g.KNOB_SIZE[i] / 2, fc[i] + sz[i] / 2) - max(c[i] - g.KNOB_SIZE[i] / 2, fc[i] - sz[i] / 2) > 1e-9
                        for i in range(3)
                    )
                    self.assertFalse(ov, f"{side} knob at {s} mm overlaps {name}")

    def test_knob_extremes_stay_inside_the_channel_stops(self):
        self.assertAlmostEqual(g.knob_cx_from_setting_mm(0) - g.KNOB_L / 2, g.CHANNEL_NEAR_STOP_X, places=9)
        self.assertAlmostEqual(g.knob_cx_from_setting_mm(100) + g.KNOB_L / 2, g.CHANNEL_FAR_STOP_X, places=9)


class TestMeterGeometry(unittest.TestCase):
    def test_needle_at_threshold_is_on_the_green_band(self):
        p = g.needle_pos_from_r(0.95)[1]
        lo = g.SCALE_GREEN_CY - g.SCALE_GREEN_LEN / 2
        self.assertAlmostEqual(p, lo, places=9)  # needle CENTRE on the band's left edge at r = 0.95
        self.assertGreater(g.needle_pos_from_r(0.97)[1], lo)
        self.assertLess(g.needle_pos_from_r(0.93)[1] + g.NEEDLE_LY / 2 - lo, g.NEEDLE_LY / 2)

    def test_needle_clears_the_scale_and_the_band(self):
        self.assertGreaterEqual(g.NEEDLE_CZ - g.NEEDLE_LZ / 2, g.SCALE_TOP_Z - 1e-9)
        self.assertGreater(g.NEEDLE_LX, g.SCALE_L)  # longer than the bar is wide: sticks out on both sides


if __name__ == "__main__":
    unittest.main()
