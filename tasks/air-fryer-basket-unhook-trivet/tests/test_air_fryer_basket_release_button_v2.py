"""Unit tests for the air fryer fry-basket task (handle release button, sliding button guard, outer basket on a trivet)."""

from __future__ import annotations

import math
import os
import pathlib
import sys
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve()
TASK_DIR = HERE.parents[1]
sys.path.insert(0, str(TASK_DIR))

# yam_arena: installed, or a source checkout named by YAM_ARENA_SRC
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.environ["YAM_ARENA_SRC"])

import demo_plan as dp
import geometry as g
import logic as l
import scoring as sc
from demo_plan import plans
from task import OBJECTS

Q0 = (0.0, 0.0, 0.0, 1.0)
B0, P0 = g.BASKET_START_POS, g.PAN_START_POS


def _box_range(centre, size, axis):
    return centre[axis] - size[axis] / 2, centre[axis] + size[axis] / 2


def _lever_box(i, theta):
    """Basket-frame centre and size of lever box i at lever angle theta (sizes unrotated, as in logic)."""
    return l.moving_part_boxes(theta, 0.0)[i]


def _hook_tip_x(theta):
    """Basket-frame x of the hook foot's leading (inner) edge at lever angle theta: max over its two right-hand corners."""
    (cx, _, cz), (sx, _, sz), _ = g.LEVER_BOXES[4]
    c, s = math.cos(theta), math.sin(theta)
    xs = []
    for z in (cz - sz / 2, cz + sz / 2):
        x = cx + sx / 2
        xs.append(g.LEVER_AT[0] + c * x + s * z)
    return max(xs)


def _in_pan_frame_basket_z(z_basket):
    return z_basket + g.BASKET_ABOVE_PAN_M


class TestSourceMechanismGeometry(unittest.TestCase):
    """The modelled handle and release follow the CM1708 manual: an inverted-L handle (arm plus vertical grip), the
    release button on top of the arm where the thumb rests, a sliding guard over it."""

    def test_hot_box_count_is_sixteen(self):
        self.assertEqual(l.HOT_BOX_COUNT, 16)
        for boxes in (g.BASKET_HANDLE_BOXES, g.LEVER_BOXES, g.GUARD_BOXES, g.TRIVET_BOXES, g.MAT_BOXES):
            self.assertTrue(all(not hot for _, _, hot in boxes))

    def test_handle_is_an_inverted_l_with_a_vertical_grip(self):
        arm = g.BASKET_HANDLE_BOXES[3]
        grip = g.BASKET_HANDLE_BOXES[4]
        self.assertGreater(grip[1][2], 3 * max(grip[1][0], grip[1][1]))          # grip much taller than wide
        self.assertAlmostEqual(_box_range(*grip[:2], 2)[1], _box_range(*arm[:2], 2)[0], places=6)   # grip hangs from the arm
        self.assertAlmostEqual(_box_range(*grip[:2], 0)[0], _box_range(*arm[:2], 0)[0], places=6)   # at the arm's outer end

    def test_release_button_is_on_top_of_the_arm_between_grip_and_basket(self):
        pad = _lever_box(2, 0.0)
        rails_top = _box_range(*g.BASKET_HANDLE_BOXES[1][:2], 2)[1]
        self.assertGreaterEqual(_box_range(*pad, 2)[0] - rails_top, 0.010)      # stands 1 cm proud of the arm
        self.assertLessEqual(max(pad[1][0], pad[1][1]), 0.025)                   # thumb sized
        grip_x = g.GRIP_CENTRE[0]
        self.assertGreater(pad[0][0], grip_x)
        front = g.BASKET_BODY_BOXES[4][0][0]
        self.assertLess(abs(pad[0][0] - front), 0.025)                           # at the handle's root on the rim, where the thumb rests
        self.assertLessEqual(pad[0][0] - grip_x, 0.080)                          # on the same handle, 7.5 cm from the grip

    def test_guard_covers_the_pad_at_rest_and_uncovers_it_when_slid(self):
        plate = g.GUARD_BOXES[0]
        pad = _lever_box(2, 0.0)
        px0, px1 = _box_range(*pad, 0)
        gx0, gx1 = _box_range(*plate[:2], 0)
        self.assertLessEqual(gx0, px0)
        self.assertGreaterEqual(gx1, px1)
        self.assertGreater(_box_range(*plate[:2], 2)[0], _box_range(*pad, 2)[1])   # above it, not touching
        # slid open: the guard's back (its tab) is past the pad by more than a pressing fingertip's half width (1.8 cm)
        tab_back = g.GUARD_TAB_BACK_X + g.GUARD_TRAVEL_M
        self.assertGreater(tab_back - (dp.PAD_X - g.BASKET_START_POS[0]), 0.018 + 0.009 + 0.005)   # plus the 9 mm forward drift
        self.assertGreater(gx0 + g.GUARD_OPEN_M, px0)

    def test_latch_holds_at_rest(self):
        lip_c, lip_s, _ = g.PAN_BOXES[5]
        lip_x0 = lip_c[0] - lip_s[0] / 2
        lip_z0 = lip_c[2] - lip_s[2] / 2 - g.BASKET_ABOVE_PAN_M                  # lip underside in the basket frame
        hook_c, hook_s = _lever_box(4, 0.0)
        self.assertGreater(_hook_tip_x(0.0), lip_x0 + 0.003)                     # 3 mm or more under the lip
        hook_top = hook_c[2] + hook_s[2] / 2
        self.assertGreater(hook_top, lip_z0 - 0.002)                              # it catches within 2 mm of lift
        self.assertLess(hook_top, lip_z0)

    def test_pressing_swings_the_hook_clear(self):
        lip_x0 = g.PAN_BOXES[5][0][0] - g.PAN_BOXES[5][1][0] / 2
        self.assertLess(_hook_tip_x(g.LEVER_RELEASE_RAD), lip_x0)
        self.assertLess(_hook_tip_x(g.LEVER_TRAVEL_RAD), lip_x0 - 0.0035)
        self.assertGreater(_hook_tip_x(0.05), lip_x0)                             # a light touch does not release it

    def test_lever_never_hits_the_arm_or_the_outer_basket_wall(self):
        rails_top = _box_range(*g.BASKET_HANDLE_BOXES[1][:2], 2)[1]
        pan_wall_out = g.PAN_BOXES[4][0][0] - g.PAN_BOXES[4][1][0] / 2
        for theta in np.linspace(0.0, g.LEVER_TRAVEL_RAD, 12):
            c, s = math.cos(theta), math.sin(theta)
            (cx, _, cz), (sx, _, sz), _ = g.LEVER_BOXES[2]
            far = g.LEVER_AT[2] - s * (cx + sx / 2) + c * (cz - sz / 2)
            self.assertGreater(far, rails_top)
            self.assertLess(_hook_tip_x(theta), pan_wall_out)

    def test_lever_parts_stay_in_the_slot(self):
        inner = g.BASKET_HANDLE_BOXES[1][0][1] - g.BASKET_HANDLE_BOXES[1][1][1] / 2
        for i in (0, 1, 3, 4):
            self.assertLess(g.LEVER_BOXES[i][1][1] / 2, inner - 0.001)

    def test_handle_clears_the_outer_basket_wall(self):
        pan_top_b = g.PAN_WALL_TOP - g.BASKET_ABOVE_PAN_M
        for c, s, _ in g.BASKET_HANDLE_BOXES[1:4]:
            self.assertGreaterEqual(c[2] - s[2] / 2 - pan_top_b, 0.004)

    def test_stack_rests_on_the_trivet_and_clears_its_neighbours(self):
        self.assertAlmostEqual(g.PAN_START_POS[2], g.TRIVET_TOP_Z)
        self.assertAlmostEqual(g.BASKET_START_POS[2] - 0.005, g.PAN_START_POS[2] + 0.006)
        trivet_y = g.TRIVET_POS[1] + 0.130
        self.assertGreater(g.MAT_Y_MIN, trivet_y + 0.01)
        pan_back = g.PAN_START_POS[0] + 0.120
        housing_front = g.HOUSING_POS[0] - 0.130
        self.assertGreater(housing_front - pan_back, 0.02)


class TestLogic(unittest.TestCase):

    def test_on_mat(self):
        z = g.MAT_TOP_Z + 0.005
        self.assertTrue(l.on_mat((g.MAT_CENTRE[0], g.MAT_CENTRE[1], z), Q0))
        self.assertFalse(l.on_mat((g.MAT_CENTRE[0], g.MAT_CENTRE[1], z + 0.02), Q0))
        self.assertFalse(l.on_mat((g.MAT_CENTRE[0], g.MAT_Y_MIN + 0.05, z), Q0))
        tilt = (math.sin(math.radians(4)), 0.0, 0.0, math.cos(math.radians(4)))    # 8 degrees
        self.assertFalse(l.on_mat((g.MAT_CENTRE[0], g.MAT_CENTRE[1], z), tilt))

    def test_pan_home_and_seated(self):
        self.assertTrue(l.pan_home(P0, Q0))
        self.assertFalse(l.pan_home((P0[0], P0[1], P0[2] + 0.02), Q0))
        self.assertFalse(l.pan_home((P0[0] + 0.03, P0[1], P0[2]), Q0))
        self.assertTrue(l.seated_in_pan(B0, Q0, P0))
        self.assertAlmostEqual(l.basket_rise_over_pan(B0, P0), 0.0, places=6)

    def test_hot_contact_on_hot_parts_only(self):
        wall_top = (P0[0] + 0.05, P0[1] - 0.109, P0[2] + g.PAN_WALL_TOP + 0.004)
        self.assertTrue(l.hot_contact([wall_top], B0, Q0, P0, Q0))
        lip = (P0[0] + g.PAN_BOXES[5][0][0], P0[1], P0[2] + g.PAN_BOXES[5][0][2])
        self.assertTrue(l.hot_contact([lip], B0, Q0, P0, Q0))
        grip = (B0[0] + g.GRIP_CENTRE[0], B0[1], B0[2] + g.GRIP_CENTRE[2])
        self.assertFalse(l.hot_contact([grip], B0, Q0, P0, Q0))
        pad_top = (B0[0] + g.PAD_CENTRE[0], B0[1], B0[2] + g.PAD_CENTRE[2] + 0.010)
        self.assertFalse(l.hot_contact([pad_top], B0, Q0, P0, Q0))
        housing_top = (g.HOUSING_POS[0], g.HOUSING_POS[1], g.HOUSING_POS[2] + 0.283)
        self.assertTrue(l.hot_contact([housing_top], B0, Q0, P0, Q0))

    def test_the_demo_pressing_and_gripping_pads_are_not_hot(self):
        # right hand closed on the pad, left hand closed across the grip (straight-down grippers, yaw 0)
        grip_q = (0.70711, -0.70711, 0.0, 0.0)   # measured yaw-0 straight-down grip (closing axis world y)
        r_tcp = (dp.PAD_X, B0[1], g.TABLE_TOP + dp.PAD_TOP_Z - g.TABLE_TOP + dp.TIP_BELOW_TCP - 0.006)
        l_tcp = (dp.GRIP_X, B0[1], g.TABLE_TOP + dp.GRIP_Z)
        pads = []
        for tcp, gap in ((r_tcp, 0.0), (l_tcp, 0.012)):
            body = np.asarray(tcp) + np.array([0.0, 0.0, g.TCP_BELOW_LINK_M])
            pads += l.hand_pose_boxes(body, grip_q, gap)[1]
        self.assertFalse(l.hot_contact(pads, B0, Q0, P0, Q0))

    def test_released(self):
        grip = (B0[0] + g.GRIP_CENTRE[0], B0[1], B0[2] + g.GRIP_CENTRE[2])
        self.assertFalse(l.released([grip], B0, Q0))
        pad = (B0[0] + g.PAD_CENTRE[0], B0[1], B0[2] + g.PAD_CENTRE[2] + 0.02)
        self.assertFalse(l.released([pad], B0, Q0))
        over_open_guard = (B0[0] + g.GUARD_BOXES[0][0][0] + g.GUARD_TRAVEL_M, B0[1], B0[2] + 0.140)
        self.assertFalse(l.released([over_open_guard], B0, Q0, 0.0, g.GUARD_TRAVEL_M))
        self.assertTrue(l.released([(0.10, 0.14, g.TABLE_TOP + 0.12), (0.10, -0.22, g.TABLE_TOP + 0.12)], B0, Q0))


def _run(path):
    """Feed (basket_pos, pan_pos, lever, guard) steps through the scorer's latches; return the final score."""
    sc._reset_run()
    for b, p, lev, gd in path:
        sc.advance(b, Q0, p, Q0, lev, gd, [])
    b, p, lev, gd = path[-1]
    return l.evaluate_state(b, Q0, p, Q0, lev, sc._RUN, True, l.still(sc._RUN["hist"]))


def _lift(dz, x=None, y=None):
    return (B0[0] if x is None else x, B0[1] if y is None else y, B0[2] + dz)


class TestScoringLadder(unittest.TestCase):

    def _success_path(self):
        on_mat = (g.MAT_CENTRE[0], g.MAT_CENTRE[1], g.MAT_TOP_Z + 0.005)
        path = [(B0, P0, 0.0, 0.0), (B0, P0, 0.0, 0.045), (B0, P0, 0.2, 0.045), (_lift(0.01), P0, 0.2, 0.045),
                (_lift(0.025), P0, 0.2, 0.045), (_lift(0.025), P0, 0.0, 0.045), (_lift(0.10), P0, 0.0, 0.045),
                (_lift(0.10, y=0.3), P0, 0.0, 0.045)]
        path += [((on_mat[0], on_mat[1], on_mat[2] + dz), P0, 0.0, 0.045) for dz in np.linspace(0.08, 0.0, 17)]
        path += [(on_mat, P0, 0.0, 0.045)] * 25
        return path

    def test_full_score(self):
        s, reason = _run(self._success_path())
        self.assertEqual(s, 1.0, reason)

    def test_lever_still_pressed_on_the_mat_is_three_quarters(self):
        path = self._success_path()
        path[-1] = (path[-1][0], P0, 0.2, 0.045)
        self.assertEqual(_run(path)[0], 0.75)

    def test_pressing_with_the_outer_basket_lifted_is_a_permanent_zero(self):
        # the whole latched stack carried 3 cm up, then the release pressed: the manual's caution
        up = 0.03
        path = [(B0, P0, 0.0, 0.045), (_lift(up), (P0[0], P0[1], P0[2] + up), 0.0, 0.045),
                (_lift(up), (P0[0], P0[1], P0[2] + up), 0.2, 0.045)] + self._success_path()[3:]
        s, reason = _run(path)
        self.assertEqual(s, 0.0)
        self.assertIn("off its trivet", reason)

    def test_lifting_out_without_the_lever_is_a_permanent_zero(self):
        path = [(B0, P0, 0.0, 0.0), (_lift(0.006), P0, 0.0, 0.0), (_lift(0.015), P0, 0.0, 0.0)] + self._success_path()[6:]
        self.assertEqual(_run(path)[0], 0.0)

    def test_carrying_the_latched_stack_is_not_the_bypass(self):
        # lifted together (outer basket still under it): no rise out of the outer basket, no permanent zero
        path = [(B0, P0, 0.0, 0.0), (_lift(0.05), (P0[0], P0[1], P0[2] + 0.05), 0.0, 0.0), (B0, P0, 0.0, 0.0)]
        sc._reset_run()
        for b, p, lev, gd in path:
            sc.advance(b, Q0, p, Q0, lev, gd, [])
        self.assertFalse(sc._RUN["unlatched_lift_ever"])
        self.assertFalse(sc._RUN["released_in_air_ever"])

    def test_basket_off_the_mat_on_the_bench_is_one_half(self):
        bench = (0.25, -0.25, g.TABLE_TOP + 0.005)
        path = self._success_path()[:7] + [((bench[0], bench[1], bench[2] + dz), P0, 0.0, 0.045) for dz in np.linspace(0.12, 0.0, 25)]
        path += [(bench, P0, 0.0, 0.045)] * 25
        self.assertEqual(_run(path)[0], 0.5)

    def test_put_back_after_unhooking_is_one_quarter(self):
        path = self._success_path()[:6] + [(B0, P0, 0.0, 0.045)] * 25
        self.assertEqual(_run(path)[0], 0.25)

    def test_nothing_done_is_zero(self):
        self.assertEqual(_run([(B0, P0, 0.0, 0.0)] * 25)[0], 0.0)

    def test_dropping_a_hot_basket_is_a_permanent_zero(self):
        path = self._success_path()[:7] + [(_lift(0.10), P0, 0.0, 0.045), (_lift(0.03), P0, 0.0, 0.045)] + self._success_path()[8:]
        self.assertEqual(_run(path)[0], 0.0)


class TestDemoPlan(unittest.TestCase):

    def test_reach_limits(self):
        for mode in ("", "idle", "noguard"):
            os.environ["ABLATION"] = mode
            try:
                for arm, wps in plans().items():
                    for wp in wps:
                        self.assertLessEqual(wp.pos[0], 0.325, (arm, wp.label))   # every pose also solved exactly offline (yam_arena ik)
                        self.assertLessEqual(wp.pos[2] - g.TABLE_TOP, 0.19 + 1e-6, (arm, wp.label))
            finally:
                os.environ.pop("ABLATION", None)

    def test_hands_on_one_handle_stay_apart(self):
        # finger roots are 6.8 cm wide across the closing axis: the grip and press points are 7.5 cm or more apart
        self.assertGreaterEqual(dp.PAD_X - dp.GRIP_X, 0.070)

    def test_left_palm_clears_the_arm_and_fingertips_hold_the_grip(self):
        tip = dp.GRIP_Z - dp.TIP_BELOW_TCP
        grip_lo = g.BASKET_START_POS[2] - g.TABLE_TOP + g.GRIP_CENTRE[2] - g.BASKET_HANDLE_BOXES[4][1][2] / 2
        arm_top = dp.ARM_TOP_Z - g.TABLE_TOP
        self.assertGreater(tip, grip_lo + 0.01)
        self.assertGreater(tip + 0.070, arm_top + 0.005)                       # finger root 7 cm above the tip
        self.assertGreater(abs(dp.GRIP_OPEN), g.BASKET_HANDLE_BOXES[4][1][1] / 2 + 0.006)

    def test_pushing_fingertip_rides_over_the_pad_and_on_the_tab(self):
        tab = g.GUARD_BOXES[1]
        tab_lo = g.BASKET_START_POS[2] - g.TABLE_TOP + tab[0][2] - tab[1][2] / 2
        tab_hi = tab_lo + tab[1][2]
        self.assertGreater(dp.PUSH_TIP_Z, dp.PAD_TOP_Z - g.TABLE_TOP)
        self.assertLess(dp.PUSH_TIP_Z, tab_hi - 0.005)
        self.assertGreater(dp.PUSH_TIP_Z, tab_lo)
        r = plans()["right"]
        x0 = next(w for w in r if w.label == "down behind the tab").pos[0]
        x1 = next(w for w in r if w.label == "slide the guard forward").pos[0]
        self.assertGreaterEqual(x1 - x0, g.GUARD_TRAVEL_M)

    def test_ablation_plans(self):
        os.environ["ABLATION"] = "idle"
        try:
            self.assertTrue(all("ready" in w.label for w in plans()["right"]))
        finally:
            os.environ.pop("ABLATION", None)
        os.environ["ABLATION"] = "noguard"
        try:
            labels = [w.label for w in plans()["right"]]
            self.assertNotIn("slide the guard forward", labels)
            self.assertIn("press the release button", labels)
        finally:
            os.environ.pop("ABLATION", None)

    def test_press_overtravel(self):
        stroke = (g.PAD_CENTRE[0] - g.LEVER_AT[0]) * math.sin(g.LEVER_TRAVEL_RAD)
        self.assertGreater(dp.PRESS_OVER, stroke)
        self.assertLessEqual(dp.PRESS_OVER, 0.040)


class TestTaskSpec(unittest.TestCase):

    def test_objects(self):
        self.assertEqual({o.name for o in OBJECTS}, {"af3_housing", "af3_trivet", "af3_pan", "af3_basket", "af3_mat"})
        basket = next(o for o in OBJECTS if o.name == "af3_basket")
        self.assertEqual({j.name for j in basket.joints}, {"lever", "guard"})
        lever = next(j for j in basket.joints if j.name == "lever")
        self.assertEqual((lever.kind, lever.axis), ("hinge", "y"))
        self.assertEqual(lever.spring, g.LEVER_RATE_NM_RAD)
        self.assertGreater(g.LEVER_RATE_NM_RAD * g.LEVER_TRAVEL_RAD / 0.032, 2.0)   # about 2.8 N on the pad at the stop

    def test_lever_spring_holds_against_its_own_weight(self):
        # lever weight acts about 2 cm out from the pivot: its sag is under a tenth of the release angle
        sag = g.LEVER_MASS * 9.81 * 0.02 / g.LEVER_RATE_NM_RAD
        self.assertLess(sag, g.LEVER_RELEASE_RAD / 10)


# Simulator ground truth from earlier GPU runs of the same robot (the station and the gripper are unchanged): per arm the
# gripper body position and body_quat_w exactly as the simulator returned them, the finger joints (joint7, joint8) and the
# tip_left / tip_right link positions the simulator reported (env origin removed). The link positions do not depend on how
# the quaternion is read.
SIM = {
    "straight_down_left": ((0.2753, 0.0424, 0.982), (0.71738, -0.69662, -0.00931, 0.00177), (-0.00981, -0.01201),
                           ((0.2994, 0.009, 0.9246), (0.2495, 0.0747, 0.926))),
    "straight_down_right": ((0.2221, -0.0855, 0.9877), (0.70744, -0.70669, -0.00572, 0.00893), (-2e-05, -0.0),
                            ((0.2449, -0.1302, 0.9307), (0.1968, -0.0413, 0.9313))),
    "turned_left": ((0.2711, 0.0449, 1.0459), (-0.00412, -0.99999, -0.00033, -0.00109), (-0.02, -0.02),
                    ((0.2465, 0.0211, 0.9892), (0.296, 0.0687, 0.9893))),
    "turned_right_free": ((0.2005, -0.2201, 1.0318), (0.99599, -0.08943, 0.00168, -0.00022), (-0.01188, -0.01188),
                          ((0.2371, -0.2023, 0.9753), (0.1643, -0.2379, 0.975))),
    "turned_left_ready": ((0.1003, 0.1399, 1.0016), (-0.00347, -0.99999, -2e-05, -0.00125), (-0.04695, -0.04695),
                          ((0.1027, 0.1158, 0.9449), (0.0982, 0.1639, 0.9449))),
    "turned_right_free_closed": ((0.2005, -0.2201, 1.0318), (0.996, -0.08933, 0.00167, -0.00024), (0.0, 0.0),
                                 ((0.2488, -0.2044, 0.9753), (0.1527, -0.2358, 0.975))),
}


def _as_wxyz(q):
    """The same four numbers read as (w, x, y, z), rewritten in (x, y, z, w) order."""
    return (q[1], q[2], q[3], q[0])


def _hand(name, order="xyzw"):
    bp, q, q78, _ = SIM[name]
    return l.hand_pose_boxes(bp, q if order == "xyzw" else _as_wxyz(q), max(abs(v) for v in q78))


class TestQuaternionOrderAgainstSimulator(unittest.TestCase):
    """body_quat_w is (x, y, z, w): read that way, the URDF finger joints put the tip_left / tip_right links exactly where
    the simulator reported them; read as (w, x, y, z) they land centimetres away."""

    def test_xyzw_reproduces_simulator_finger_links(self):
        for name, (bp, q, (q7, q8), tips) in SIM.items():
            pred = l.tip_link_positions(bp, q, q7, q8)
            for p, m in zip(pred, tips):
                self.assertLess(float(np.linalg.norm(np.asarray(p) - np.asarray(m))), 0.0005, name)

    def test_wxyz_misplaces_simulator_finger_links(self):
        for name, (bp, q, (q7, q8), tips) in SIM.items():
            pred = l.tip_link_positions(bp, _as_wxyz(q), q7, q8)
            err = max(float(np.linalg.norm(np.asarray(p) - np.asarray(m))) for p, m in zip(pred, tips))
            self.assertGreater(err, 0.040, name)

    def test_pads_open_along_the_measured_finger_travel(self):
        _, _, _, tips_open = SIM["turned_right_free"]
        _, _, _, tips_closed = SIM["turned_right_free_closed"]
        travel = np.asarray(tips_open[0]) - np.asarray(tips_closed[0])
        self.assertGreater(float(np.linalg.norm(travel)), 0.010)
        _, pads, _ = _hand("turned_right_free")
        pad_dir = np.asarray(pads[0]) - np.asarray(pads[1])
        cos = abs(float(travel @ pad_dir)) / (np.linalg.norm(travel) * np.linalg.norm(pad_dir))
        self.assertGreater(cos, 0.98)


if __name__ == "__main__":
    unittest.main()
