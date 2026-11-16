"""Unit tests for air_fryer_basket_release_button_v2 benchmark task."""

from __future__ import annotations

import math
import os
import pathlib
import sys
import unittest

import numpy as np

# Set up import paths
HERE = pathlib.Path(__file__).resolve()
TASK_DIR = HERE.parents[1]
sys.path.insert(0, str(TASK_DIR))

# Ensure yam_arena can be imported
_ROOT = HERE.parents[3].parent
_KNOWN = _ROOT / "robocurve-windows-transfer-2026-11-05" / "project" / "robocurve" / "tmp" / "yam-upstream"
for cand in [_KNOWN] if (_KNOWN / "yam_arena").is_dir() else _ROOT.rglob("yam-upstream"):
    if (cand / "yam_arena").is_dir():
        sys.path.insert(0, str(cand))
        break

import geometry as g
import logic as l
from demo_plan import plans
from scoring import distance, score, success
from task import OBJECTS


class TestGeometry(unittest.TestCase):
    """Verify geometry constants, collision box counts, and physical properties."""

    def test_hot_box_count_is_nineteen(self):
        """Must have exactly 19 hot collision boxes (5 housing, 9 drawer, 5 basket)."""
        housing_hot = [b for b in g.HOUSING_BOXES if b[2]]
        drawer_hot = [b for b in g.DRAWER_BOXES if b[2]]
        basket_hot = [b for b in g.BASKET_BODY_BOXES if b[2]]
        self.assertEqual(len(housing_hot), 5)
        self.assertEqual(len(drawer_hot), 9)
        self.assertEqual(len(basket_hot), 5)
        self.assertEqual(len(housing_hot) + len(drawer_hot) + len(basket_hot), 19)

    def test_touchable_boxes_are_not_hot(self):
        """Handle, button, and mat boxes must NOT be marked hot."""
        for b in g.BASKET_HANDLE_BOXES:
            self.assertFalse(b[2])
        for b in g.BUTTON_BOXES:
            self.assertFalse(b[2])
        for b in g.MAT_BOXES:
            self.assertFalse(b[2])

    def test_slide_properties(self):
        """Drawer and button slides must match design specifications."""
        self.assertAlmostEqual(g.DRAWER_TRAVEL_M, 0.225)
        self.assertAlmostEqual(g.DRAWER_RATE_N_M, 0.0)   # not sprung
        self.assertAlmostEqual(g.DRAWER_DAMPING_NS_M, 30.0)

        self.assertAlmostEqual(g.BUTTON_TRAVEL_M, 0.014)
        self.assertAlmostEqual(g.BUTTON_RATE_N_M, 8.0)
        self.assertLessEqual(g.BUTTON_RATE_N_M, 10.0)  # Arm-held <= 10 N/m limit
        self.assertAlmostEqual(g.BUTTON_DAMPING_NS_M, 5.0)

    def test_pad_dimensions_and_tolerances(self):
        """Fingertip pad is 12 mm cube and hot inflation is 2 mm."""
        self.assertEqual(g.PAD_BOX_SIZE, (0.012, 0.012, 0.012))
        self.assertEqual(g.PAD_BOX_HALF, (0.006, 0.006, 0.006))
        self.assertAlmostEqual(g.HOT_INFLATION_M, 0.002)
        self.assertGreaterEqual(g.MAT_Z_TOL, 0.005)


class TestLogicCatchesAndButtons(unittest.TestCase):
    """Test physical release latch logic and button extension thresholds."""

    def test_catches_hold_basket_until_pressed(self):
        """Catches are scored as engaged below 8 mm of travel (they physically clear after 7 mm)."""
        self.assertTrue(l.catches_hold_basket(0.000))
        self.assertTrue(l.catches_hold_basket(0.003))
        self.assertTrue(l.catches_hold_basket(0.0079))
        self.assertFalse(l.catches_hold_basket(0.0080))
        self.assertFalse(l.catches_hold_basket(0.0140))

    def test_button_out_threshold(self):
        """BUTTON_OUT requires the button >= 90% extended (displacement <= 1.4 mm of the 14 mm stroke)."""
        self.assertTrue(l.button_out(0.000))
        self.assertTrue(l.button_out(0.0005))
        self.assertTrue(l.button_out(0.0014))
        self.assertFalse(l.button_out(0.0015))
        self.assertFalse(l.button_out(0.0090))


class TestLogicOnMat(unittest.TestCase):
    """Test ON_MAT corner containment, underside elevation, and tilt tolerances."""

    def setUp(self):
        # Basket centered perfectly on the mat at resting height
        self.mat_b_pos = (g.MAT_CENTRE[0], g.MAT_CENTRE[1], g.MAT_TOP_Z + 0.005)
        self.mat_b_quat = (0.0, 0.0, 0.0, 1.0)

    def test_perfect_placement_on_mat(self):
        self.assertTrue(l.on_mat(self.mat_b_pos, self.mat_b_quat))

    def test_corner_out_of_x_bounds_fails(self):
        pos_too_far_x = (g.MAT_X_MAX - g.BASKET_FLOOR_BACK_X + 0.010, g.MAT_CENTRE[1], self.mat_b_pos[2])  # back edge 10 mm off
        self.assertFalse(l.on_mat(pos_too_far_x, self.mat_b_quat))
        pos_front_off = (g.MAT_X_MIN + g.BASKET_FLOOR_HALF_X - 0.010, g.MAT_CENTRE[1], self.mat_b_pos[2])  # front edge 10 mm off
        self.assertFalse(l.on_mat(pos_front_off, self.mat_b_quat))

    def test_corner_out_of_y_bounds_fails(self):
        pos_too_far_y = (g.MAT_CENTRE[0], g.MAT_Y_MIN + g.BASKET_FLOOR_HALF_Y - 0.010, self.mat_b_pos[2])
        self.assertFalse(l.on_mat(pos_too_far_y, self.mat_b_quat))

    def test_elevation_too_high_or_low_fails(self):
        pos_too_high = (self.mat_b_pos[0], self.mat_b_pos[1], self.mat_b_pos[2] + 0.006)
        pos_too_low = (self.mat_b_pos[0], self.mat_b_pos[1], self.mat_b_pos[2] - 0.006)
        self.assertFalse(l.on_mat(pos_too_high, self.mat_b_quat))
        self.assertFalse(l.on_mat(pos_too_low, self.mat_b_quat))

    def test_excessive_tilt_fails(self):
        # Tilt > 5 degrees
        angle = math.radians(6.0)
        tilt_quat = (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))
        self.assertFalse(l.on_mat(self.mat_b_pos, tilt_quat))


class TestLogicHotContact(unittest.TestCase):
    """Test fingertip pad hot collision detection on all 19 hot boxes."""

    def test_housing_hot_contact(self):
        """Pad touching housing floor or walls registers hot contact."""
        h_origin = np.array(g.HOUSING_POS)
        # Pad placed at housing back wall
        pad_at_housing = [h_origin + np.array([0.122, 0.000, 0.047])]
        self.assertTrue(l.hot_contact(pad_at_housing, g.BASKET_START_POS, (0, 0, 0, 1)))

    def test_drawer_hot_contact(self):
        """Pad touching drawer floor or walls registers hot contact."""
        d_origin = np.array(g.HOUSING_POS) + np.array(g.DRAWER_AT)
        pad_at_drawer_floor = [d_origin + np.array([0.000, 0.000, 0.003])]
        self.assertTrue(l.hot_contact(pad_at_drawer_floor, g.BASKET_START_POS, (0, 0, 0, 1)))

    def test_basket_body_hot_contact(self):
        """Pad touching orange basket body floor or side walls registers hot contact."""
        b_origin = np.array(g.BASKET_START_POS)
        # Right wall of basket
        pad_at_basket_wall = [b_origin + np.array([0.000, -0.100, 0.035])]
        self.assertTrue(l.hot_contact(pad_at_basket_wall, g.BASKET_START_POS, (0, 0, 0, 1)))

    def test_touchable_handle_contact_is_not_hot(self):
        """Pad touching cool-touch handle bar or button cap does NOT trigger hot contact."""
        b_origin = np.array(g.BASKET_START_POS)
        # the vertical cool-touch grip bar: x = -0.1855, y = 0.065, z = 0.0575
        pad_at_handle = [b_origin + np.array([-0.1855, 0.065, 0.0575])]
        # Button cap: x = -0.1815, y = -0.070, z = 0.038
        pad_at_button = [b_origin + np.array([-0.1815, -0.070, 0.038])]
        self.assertFalse(l.hot_contact(pad_at_handle, g.BASKET_START_POS, (0, 0, 0, 1)))
        self.assertFalse(l.hot_contact(pad_at_button, g.BASKET_START_POS, (0, 0, 0, 1)))


class TestScoringLevels(unittest.TestCase):
    """Test evaluation and full scoring levels (1.0, 0.75, 0.5, 0.25, 0.0)."""

    def setUp(self):
        self.mat_pos = (g.MAT_CENTRE[0], g.MAT_CENTRE[1], g.MAT_TOP_Z + 0.005)
        self.mat_quat = (0.0, 0.0, 0.0, 1.0)

    def test_full_success_is_one(self):
        score, reason = l.evaluate_state(
            basket_pos=self.mat_pos,
            basket_quat=self.mat_quat,
            button_disp=0.000,
            hot_contact_ever=False,
            cleared_drawer_ever=True,
            unhook_lift_ever=True,
            grippers_released=True,
            basket_still=True,
        )
        self.assertEqual(score, 1.0)

    def test_not_released_or_not_still_is_three_quarters(self):
        score_unreleased, _ = l.evaluate_state(
            basket_pos=self.mat_pos,
            basket_quat=self.mat_quat,
            button_disp=0.000,
            hot_contact_ever=False,
            cleared_drawer_ever=True,
            unhook_lift_ever=True,
            grippers_released=False,
            basket_still=True,
        )
        self.assertEqual(score_unreleased, 0.75)

        score_unstill, _ = l.evaluate_state(
            basket_pos=self.mat_pos,
            basket_quat=self.mat_quat,
            button_disp=0.000,
            hot_contact_ever=False,
            cleared_drawer_ever=True,
            unhook_lift_ever=True,
            grippers_released=True,
            basket_still=False,
        )
        self.assertEqual(score_unstill, 0.75)

    def test_button_held_at_end_is_three_quarters(self):
        score_button_held, _ = l.evaluate_state(
            basket_pos=self.mat_pos,
            basket_quat=self.mat_quat,
            button_disp=0.005,  # Still held
            hot_contact_ever=False,
            cleared_drawer_ever=True,
            unhook_lift_ever=True,
            grippers_released=True,
            basket_still=True,
        )
        self.assertEqual(score_button_held, 0.75)

    def test_cleared_drawer_not_on_mat_is_half(self):
        bench_pos = (0.25, 0.0, g.TABLE_TOP + 0.010)
        score_cleared, _ = l.evaluate_state(
            basket_pos=bench_pos,
            basket_quat=self.mat_quat,
            button_disp=0.000,
            hot_contact_ever=False,
            cleared_drawer_ever=True,
            unhook_lift_ever=True,
            grippers_released=True,
            basket_still=True,
        )
        self.assertEqual(score_cleared, 0.5)

    def test_unhook_lift_in_drawer_is_quarter(self):
        score_quarter, _ = l.evaluate_state(
            basket_pos=g.BASKET_START_POS,
            basket_quat=self.mat_quat,
            button_disp=0.000,
            hot_contact_ever=False,
            cleared_drawer_ever=False,
            unhook_lift_ever=True,
            grippers_released=True,
            basket_still=True,
        )
        self.assertEqual(score_quarter, 0.25)

    def test_hot_contact_is_permanent_zero(self):
        """Any hot contact by either pad at any step latches permanent zero."""
        score_hot, _ = l.evaluate_state(
            basket_pos=self.mat_pos,
            basket_quat=self.mat_quat,
            button_disp=0.000,
            hot_contact_ever=True,  # Hot contact latched!
            cleared_drawer_ever=True,
            unhook_lift_ever=True,
            grippers_released=True,
            basket_still=True,
        )
        self.assertEqual(score_hot, 0.0)


class TestAblationAndReach(unittest.TestCase):
    """Test ablation mode, TCP reach limits, and arm separation."""

    def test_ablation_mode_plan(self):
        """Ablation: the right arm stays idle at ready, so the release button is never pressed and the catches hold."""
        os.environ["ABLATION"] = "1"
        try:
            p = plans()
            self.assertEqual(len(p["right"]), 1)
            self.assertIn("idle", p["right"][0].label)
            self.assertIn("lift out of drawer", [w.label for w in p["left"]])
        finally:
            os.environ.pop("ABLATION", None)

    def test_demo_reach_limits(self):
        """Every TCP waypoint must satisfy x <= 0.30 and z <= 0.19 relative to table (IK is exact to 0.20)."""
        demo_plans = plans()
        for arm in ("left", "right"):
            for wp in demo_plans[arm]:
                self.assertLessEqual(wp.pos[0], 0.30, f"{arm} waypoint x exceeds 0.30: {wp.pos}")
                self.assertLessEqual(wp.pos[2] - g.TABLE_TOP, 0.19 + 1e-4, f"{arm} waypoint z exceeds 0.19: {wp.pos}")

    def test_button_is_small_and_separate_from_the_lifting_grip(self):
        """The release is a thumb-sized button (<= 3 cm either way) at the handle's right end; the lifting grip is the
        handle's left post, at least 12.5 cm away from the button so the two hands do not meet."""
        import demo_plan as dp
        size = g.BUTTON_BOXES[0][1]
        self.assertLessEqual(size[1], 0.030)
        self.assertLessEqual(size[2], 0.030)
        self.assertGreaterEqual(abs(dp.POST_Y - dp.BUTTON_Y), 0.125)   # the offline arm check needs the hands >= 120 mm apart in y
        bar = g.BASKET_HANDLE_BOXES[0][1]
        self.assertGreater(bar[2], 2.5 * max(bar[0], bar[1]))   # the grip is vertical: much taller than wide
        self.assertLessEqual(bar[2], 0.120)                    # a realistic grip (6 to 12 cm)
        self.assertLessEqual(bar[1], 0.030)


    def test_vertical_grip_is_reached_from_above_and_stays_in_reach(self):
        """The left fingertips hover above the vertical bar's top, descend either side of it, and the lift never passes the 19 cm reach limit."""
        import demo_plan as dp
        bar = g.BASKET_HANDLE_BOXES[0]
        bar_top = g.BASKET_START_POS[2] - g.TABLE_TOP + bar[0][2] + bar[1][2] / 2
        self.assertGreater(dp.HOVER_LEFT_Z - 0.013, bar_top)             # fingertips 13 mm below the tool point
        grip_tip = dp.GRIP_Z - 0.013
        self.assertGreater(grip_tip, g.BASKET_START_POS[2] - g.TABLE_TOP + bar[0][2] - bar[1][2] / 2)
        self.assertLess(grip_tip, bar_top - 0.02)                        # at least 2 cm of the bar between the fingertips and its top
        self.assertLessEqual(dp.LIFT_Z, 0.19)
        inner = g.BASKET_HANDLE_BOXES[0][1][1] / 2
        self.assertGreater(abs(dp.POST_OPEN), inner + 0.006)             # open fingers clear the bar's sides

    def test_handle_clears_the_drawer_strip_and_grippers_stay_apart(self):
        """Handle parts over the drawer's front strip sit above it; the lifting hand grips 4 cm or more behind the pressing
        hand's furthest point, so the two grippers do not touch (measured on the GPU: 0 cm apart in x jammed them)."""
        import demo_plan as dp
        b = g.BASKET_START_POS
        strip_top = g.HOUSING_POS[2] + g.DRAWER_AT[2] + 0.032
        drawer_front = g.HOUSING_POS[0] + g.DRAWER_AT[0] - 0.120
        for c, sz, _ in g.BASKET_HANDLE_BOXES:
            if b[0] + c[0] + sz[0] / 2 > drawer_front:
                self.assertGreater(b[2] + c[2] - sz[2] / 2, strip_top + 0.001)
        stop = max(w.pos[0] for w in plans()["right"])
        self.assertGreaterEqual(dp.POST_X - stop, 0.040)

    def test_demo_press_is_commanded_past_the_stop(self):
        """The right hand presses 4 to 10 mm past the button's full stroke (more shoves the whole basket back through the plate)."""
        import demo_plan as dp
        plan = plans()["right"]
        touch = next(w for w in plan if "ready to press" in w.label).pos[0] + 0.030
        stop = next(w for w in plan if "press button to stop" in w.label).pos[0]
        past = stop - touch - g.BUTTON_TRAVEL_M
        self.assertGreaterEqual(past, 0.004)
        self.assertLessEqual(past, 0.010)

    def test_left_pads_clear_the_hot_front_wall_and_the_right_post(self):
        """Left fingers open 20 mm each side along y: the outer pad stays clear of the bar end, the inner pad clear of the
        right post; in x the pad sits well behind the hot front wall zone."""
        import demo_plan as dp
        half = abs(dp.POST_OPEN)
        inner_edge = dp.POST_Y - half - 0.012
        right_post_edge = g.BASKET_START_POS[1] + g.BASKET_HANDLE_BOXES[1][0][1] + g.BASKET_HANDLE_BOXES[1][1][1] / 2
        self.assertGreater(inner_edge, right_post_edge + 0.010)
        basket_front = g.BASKET_START_POS[0] + g.BASKET_BODY_BOXES[4][0][0] - g.BASKET_BODY_BOXES[4][1][0] / 2
        zone = g.HOT_INFLATION_M + g.PAD_BOX_HALF[0]
        self.assertLess(dp.POST_X, basket_front - zone - 0.008)   # pad centre (the pad box is centred on the tool point)


class TestTaskSpec(unittest.TestCase):
    """Test yam task specification objects and structure."""

    def test_task_objects_structure(self):
        self.assertEqual(len(OBJECTS), 3)
        names = {o.name for o in OBJECTS}
        self.assertEqual(names, {"af2_housing", "af2_basket", "af2_mat"})

    def test_display_lamp_and_tiles_positions(self):
        """Verify lamp and vent tiles are located on housing."""
        self.assertEqual(len(g.VENT_GLOW_TILES_LOCAL), 4)
        for tile in g.VENT_GLOW_TILES_LOCAL:
            self.assertAlmostEqual(tile[2], 0.281)
        self.assertAlmostEqual(g.LAMP_TILE_LOCAL[2], 0.281)




class TestCatchGeometry(unittest.TestCase):
    """The catch lugs physically hold the basket until the button is pressed (computed from the world boxes)."""

    def _x_ranges(self, travel):
        lugs, ledges = [], []
        for c, s, _ in g.BUTTON_BOXES[2:]:
            x = g.BASKET_START_POS[0] + g.BUTTON_AT[0] + c[0] + travel
            z = g.BASKET_START_POS[2] + g.BUTTON_AT[2] + c[2]
            lugs.append((x - s[0] / 2, x + s[0] / 2, z - s[2] / 2, z + s[2] / 2))
        drawer_x = g.HOUSING_POS[0] + g.DRAWER_AT[0]
        drawer_z = g.HOUSING_POS[2] + g.DRAWER_AT[2]
        for c, s, _ in g.DRAWER_BOXES[-2:]:
            ledges.append((drawer_x + c[0] - s[0] / 2, drawer_x + c[0] + s[0] / 2, drawer_z + c[2] - s[2] / 2, drawer_z + c[2] + s[2] / 2))
        return lugs, ledges

    def test_catch_lugs_hold_then_clear(self):
        lugs, ledges = self._x_ranges(0.0)
        for lug, ledge in zip(lugs, ledges):
            self.assertGreaterEqual(min(lug[1], ledge[1]) - max(lug[0], ledge[0]), 0.005)  # >= 5 mm under the ledge at rest
            self.assertLess(lug[3], ledge[2])  # below the ledge: a lift catches it
        lugs, ledges = self._x_ranges(g.BUTTON_TRAVEL_M)
        for lug, ledge in zip(lugs, ledges):
            self.assertGreaterEqual(lug[0] - ledge[1], 0.005)  # >= 5 mm past the ledge's rear edge at full travel: the basket lifts free
        lugs, ledges = self._x_ranges(g.BUTTON_CLEARANCE_DISPLACEMENT_M)
        for lug, ledge in zip(lugs, ledges):
            self.assertGreater(lug[0], ledge[1])   # the scoring threshold is only reached once the lugs are physically clear


class TestEndStateRungsAndBypass(unittest.TestCase):
    """The partial rungs must not reward an unsafe end state, and the latch cannot be bypassed."""

    q0 = (0.0, 0.0, 0.0, 1.0)

    def _score(self, pos, bypass=False, dropped=False, cleared=True, unhook=True, quat=None):
        s, _ = l.evaluate_state(pos, quat or self.q0, 0.0, False, cleared, unhook, True, True, bypass, dropped)
        return s

    def test_bypass_is_a_permanent_zero(self):
        self.assertEqual(self._score(g.MAT_CENTRE[:2] + (g.MAT_TOP_Z + 0.005,), bypass=True), 0.0)

    def test_half_credit_needs_the_basket_resting_on_the_bench_not_dropped(self):
        off = (g.BASKET_START_POS[0] - 0.20, g.BASKET_START_POS[1], g.TABLE_TOP + 0.015)
        self.assertEqual(self._score(off), 0.5)
        self.assertEqual(self._score(off, dropped=True), 0.0)   # dropped: no 0.5, and it is not seated either
        back_in_drawer = tuple(g.BASKET_START_POS)
        self.assertEqual(self._score(back_in_drawer), 0.25)   # lifted and put back: only the 0.25 rung
        on_housing = (g.HOUSING_POS[0], g.HOUSING_POS[1], g.TABLE_TOP + 0.35)
        self.assertEqual(self._score(on_housing), 0.0)         # left on the hot housing: nothing

    def test_quarter_credit_needs_the_basket_seated_back(self):
        half_out = (g.BASKET_START_POS[0], g.BASKET_START_POS[1], g.BASKET_START_POS[2] + 0.03)
        self.assertEqual(self._score(half_out, cleared=False), 0.0)

    def test_tipped_basket_gets_nothing_above_zero(self):
        off = (g.BASKET_START_POS[0] - 0.20, g.BASKET_START_POS[1], g.TABLE_TOP + 0.015)
        tipped = (0.0, 0.7071, 0.0, 0.7071)   # on its side
        self.assertEqual(self._score(off, quat=tipped), 0.0)


class TestReleasedUsesFingerPads(unittest.TestCase):
    """RELEASED must see the finger pads, not only the gripper centres (an earlier rejection was for a one-point scorer)."""

    def test_pad_touching_the_basket_is_not_released(self):
        pos = (g.MAT_CENTRE[0], g.MAT_CENTRE[1], g.MAT_TOP_Z + 0.005)
        far_centre = (0.10, 0.14, g.TABLE_TOP + 0.12)
        pad_on_basket = (pos[0], pos[1] + 0.100, pos[2] + 0.03)   # on the basket's left wall
        self.assertTrue(l.released([far_centre], pos, (0, 0, 0, 1)))
        self.assertFalse(l.released([far_centre, pad_on_basket], pos, (0, 0, 0, 1)))



# --- gripper orientation convention: (x, y, z, w) as Isaac Lab 3 reports body_quat_w -----------------------------------
# Captured from the vertical-handle demo run (step 400): left gripper holding the grip bar, body position
# (0.271, 0.0447, 0.9718) m, raw quaternion below. The commanded tool point was (0.2710, 0.045, 0.8375) m (grip height 9.0 cm).
CAPTURED_Q_XYZW = (0.7072, -0.70702, 0.00032, -0.00093)
CAPTURED_BODY = (0.271, 0.0447, 0.9718)
COMMANDED_TCP = (0.271, 0.045, 0.8375)


def _quat_about_z(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def test_captured_quaternion_reproduces_commanded_tool_point():
    tcp, pads, tilt = l.hand_pose_boxes(CAPTURED_BODY, CAPTURED_Q_XYZW, 0.02)
    assert np.allclose(tcp, COMMANDED_TCP, atol=0.002)
    assert tilt < 1.0
    assert len(pads) == 8


def test_wxyz_reading_of_captured_quaternion_would_misplace_the_hand():
    x, y, z, w = CAPTURED_Q_XYZW
    tcp_wrong, _, tilt_wrong = l.hand_pose_boxes(CAPTURED_BODY, (y, z, w, x), 0.02)
    assert np.linalg.norm(tcp_wrong - np.array(COMMANDED_TCP)) > 0.10
    assert tilt_wrong > 45.0


def test_pad_boxes_follow_a_rotated_gripper():
    # Yaw 0: the closing axis is world y (pads either side in y); yaw +90 degrees: along world x.
    _, pads0, _ = l.hand_pose_boxes(CAPTURED_BODY, CAPTURED_Q_XYZW, 0.02)
    quat90 = _qmul(_quat_about_z(math.pi / 2), CAPTURED_Q_XYZW)
    _, pads90, _ = l.hand_pose_boxes(CAPTURED_BODY, quat90, 0.02)
    d0 = np.array(pads0[0]) - np.array(pads0[1])
    d90 = np.array(pads90[0]) - np.array(pads90[1])
    assert abs(d0[1]) > 0.04 and abs(d0[0]) < 1e-3
    assert abs(d90[0]) > 0.04 and abs(d90[1]) < 1e-3


def test_hot_contact_detected_with_rotated_gripper():
    # A hot box (the drawer floor) at the left finger of a yaw-90 hand is hit; moving the hand 10 cm away clears it.
    quat90 = _qmul(_quat_about_z(math.pi / 2), CAPTURED_Q_XYZW)
    hot_xy = np.array([0.271 + 0.026, 0.0449])
    body = np.array(CAPTURED_BODY)
    _, pads, _ = l.hand_pose_boxes(body, quat90, 0.02)
    nearest = min(np.linalg.norm(np.array(p)[:2] - hot_xy) for p in pads)
    assert nearest < 0.02
    _, pads_far, _ = l.hand_pose_boxes(body + np.array([0.0, 0.10, 0.0]), quat90, 0.02)
    assert min(np.linalg.norm(np.array(p)[:2] - hot_xy) for p in pads_far) > 0.05


class TestBypassBandOnlyOverTheLedges(unittest.TestCase):
    """The latch band (lift 8 to 14 mm with the button out) is a bypass only over the drawer's ledges, not when the basket is
    later lowered through the same heights onto the mat (measured on the GPU: a level-hanging basket lowered with the button
    out was wrongly flagged)."""

    def _run(self, path):
        import scoring as sc
        sc._reset_run()
        q = (0.0, 0.0, 0.0, 1.0)
        for pos, btn in path:
            sc.advance(pos, q, btn, [], [])
        return sc._RUN["bypass_ever"]

    def test_lift_in_the_drawer_with_the_button_out_is_a_bypass(self):
        x, y, z = g.BASKET_START_POS
        self.assertTrue(self._run([((x, y, z + dz), 0.0) for dz in (0.0, 0.004, 0.009, 0.012, 0.020)]))

    def test_lift_with_the_button_held_is_not(self):
        x, y, z = g.BASKET_START_POS
        self.assertFalse(self._run([((x, y, z + dz), 0.012) for dz in (0.0, 0.004, 0.009, 0.012, 0.020)]))

    def test_lowering_onto_the_mat_with_the_button_out_is_not(self):
        x, y, z = g.BASKET_START_POS
        path = [((x, y, z), 0.0), ((x, y, z + 0.005), 0.012), ((x, y, z + 0.010), 0.012), ((x, y, z + 0.016), 0.010),
                ((x, y, z + 0.075), 0.003)]
        mx, my = g.MAT_CENTRE[0] - 0.015, g.MAT_CENTRE[1]
        path += [((mx, my, z + dz), 0.003) for dz in (0.075, 0.040, 0.014, 0.011, 0.008, -0.020)]
        self.assertFalse(self._run(path))

    def test_shove_back_then_lift_with_the_button_out_is_a_bypass(self):
        x, y, z = g.BASKET_START_POS
        path = [((x + dx, y, z), 0.0) for dx in (0.0, 0.015, 0.030, 0.045)]
        path += [((x + 0.045, y, z + dz), 0.0) for dz in (0.004, 0.009, 0.012, 0.030)]
        self.assertTrue(self._run(path))

if __name__ == "__main__":
    unittest.main()
