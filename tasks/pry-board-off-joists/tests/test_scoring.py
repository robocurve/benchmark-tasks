"""Offline tests for layout.py, scoring.py and task.py: synthetic board states (mechkit AssemblyStates from the task's
own specs), synthetic arm snapshots (rigstate), and the nails' rule driven by hand. No simulator.

    PYTHONPATH=<yam-isaac-lab-arena> python3 -m unittest discover -s tests -v

Every number here is a design value; nothing was measured on the simulator."""

import math
import pathlib
import sys
import unittest

sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

try:
    import layout as L  # noqa: E402
    import mechkit as mk  # noqa: E402
    import rigstate  # noqa: E402
    import scoring as S  # noqa: E402
    import task as T  # noqa: E402
except ModuleNotFoundError as e:
    if e.name != "yam_arena" and not str(e.name).startswith("yam_arena."):
        raise
    raise unittest.SkipTest("yam_arena is not importable: put yam-isaac-lab-arena on PYTHONPATH")

MM = L.MM
OUT = {"left": True, "right": True}
IN = {"left": False, "right": False}


def nailed(up_mm=0.0, tip=0.0):
    """The board on its nails, ``up_mm`` straight up from where it was nailed, turned ``tip`` about its long centre
    line on the joists' tops (+tip raises its left end)."""
    x, y, z = L.board_start_centre()
    r = mk.axis_angle_matrix((1.0, 0.0, 0.0), tip)
    return mk.state_from_poses(T.BOARD, (x, y, z + up_mm * MM, *mk.matrix_to_quat(r)), source="test")


def lying(face_up=True, x=0.15, y=0.0, lift=0.0, tilt=0.0, yaw=0.0):
    """The board lying on a broad face on the table at (x, y), turned ``yaw`` about the vertical."""
    r = mk.axis_angle_matrix((0.0, 0.0, 1.0), yaw) @ mk.axis_angle_matrix((1.0, 0.0, 0.0), tilt)
    if not face_up:
        r = r @ mk.axis_angle_matrix((1.0, 0.0, 0.0), math.pi)
    z = L.TOP + L.BOARD_T / 2 + lift + abs(math.sin(tilt)) * L.BOARD_L / 2
    return mk.state_from_poses(T.BOARD, (x, y, z, *mk.matrix_to_quat(r)), source="test")


def judge(state, out=IN, released=True, still=True):
    return S.judge(state.rotation(), state.origin, out, released, still)


class Layout(unittest.TestCase):
    def test_lumber_is_standard_actual_size(self):
        self.assertAlmostEqual(L.JOIST_W, 38.1 * MM, delta=1e-6)  # 2x4: 1-1/2 x 3-1/2 in
        self.assertAlmostEqual(L.JOIST_H, 88.9 * MM, delta=1e-6)
        self.assertAlmostEqual(L.BOARD_T, 19.05 * MM, delta=1e-6)  # 1x4: 3/4 x 3-1/2 in
        self.assertAlmostEqual(L.BOARD_W, 88.9 * MM, delta=1e-6)
        self.assertAlmostEqual(L.JOIST_Y[1] - L.JOIST_Y[0], 16 * L.INCH, delta=1e-6)  # 16 in on centre

    def test_board_starts_on_the_joists(self):
        b = nailed()
        c = S.board_corners(b.rotation(), b.origin)
        self.assertAlmostEqual(float(c[:, 2].min()) - L.JOIST_TOP, L.SPAWN_CLEAR, delta=1e-6)
        overhang = L.BOARD_L / 2 - (L.NAIL_DY + L.JOIST_W / 2)
        self.assertAlmostEqual(overhang, 77.8 * MM, delta=0.5 * MM)
        self.assertTrue(L.JOIST_X[0] < float(c[:, 0].min()) and float(c[:, 0].max()) < L.JOIST_X[1])

    def test_nails_go_into_the_joists(self):
        self.assertAlmostEqual(L.EMBED, 12.7 * MM, delta=0.01 * MM)
        for name, (lo, hi) in L.joist_boxes().items():
            self.assertTrue(lo[1] < L.NAILS[name] < hi[1])
        # the withdrawal estimate: Wood Handbook p = 54.12 G^2.5 D L (N, mm), SPF G = 0.42
        p = 54.12 * 0.42 ** 2.5 * (L.NAIL_D / MM) * (L.EMBED / MM)
        self.assertAlmostEqual(L.PULL_FULL, p, delta=5.0)

    def test_carrier_stays_hidden_inside_the_joists(self):
        # its boxes at CARRIER_Z, raised at most by LIFT plus the turn at the nails, stay under the joists' tops
        top = L.CARRIER_Z + 3 * MM + L.LIFT_RANGE[1] + L.NAIL_DY * math.sin(L.TIP_RANGE[1])
        self.assertLess(top, L.JOIST_H)
        # and sideways: the turn about the axis on the joists' tops swings them by (JOIST_H - CARRIER_Z) sin(tip)
        side = 3 * MM + (L.JOIST_H - L.CARRIER_Z) * math.sin(L.TIP_RANGE[1])
        self.assertLess(side, L.JOIST_W / 2)

    def test_every_carrier_box_is_inside_a_joist(self):
        boxes = list(L.joist_boxes().values())
        for link in T.FRAME.joints:
            for s in link.shapes:
                c = np.asarray(s.center) + T.FRAME_ORIGIN
                h = s.half_extents()
                inside = any(np.all(c - h > np.asarray(lo)) and np.all(c + h < np.asarray(hi)) for lo, hi in boxes)
                self.assertTrue(inside, f"{link.name} box at {c} shows outside the joists")

    def test_bar_lies_on_the_table_in_front_of_the_frame(self):
        lo, hi = T.BAR.world_bounds()
        self.assertGreater(lo[2], L.TOP)
        self.assertLess(lo[2] - L.TOP, 2 * MM)
        self.assertLess(hi[0], L.JOIST_X[0] - 0.05)  # at least 5 cm in front of the joists
        self.assertGreater(lo[0], 0.05)  # clear of the arm bases' line

    def test_blade_underside_runs_flush_from_the_shaft(self):
        # a flat bar is ground on its top face: every blade segment's underside is the shaft's, 3 mm off the centre line
        for c, s, a, _col in L.bar_shapes():
            if not math.isclose(a, math.radians(L.BLADE_BEND)):
                continue
            n = (math.sin(a), math.cos(a))
            under = (c[0] - n[0] * s[2] / 2, c[2] - n[1] * s[2] / 2)
            self.assertAlmostEqual(under[0] * n[0] + under[1] * n[1], -L.SHAFT_T / 2, delta=1e-9)
        # and the heel (the bend's underside) is the bar's lowest point with the blade laid flat
        z = -sum(length * math.sin(math.radians(angle)) for length, _t, angle, _c in L.BAR_SEGMENTS[:3])
        self.assertAlmostEqual(L.bar_point("heel")[2], z - L.SHAFT_T / 2, delta=1e-9)

    def test_eased_edge_takes_the_blade(self):
        # the blade's edge (1.6 mm) fits under the board's eased edge (3 mm step) where it sits on a joist
        self.assertLess(L.BAR_SEGMENTS[0][1], L.EASE)
        self.assertAlmostEqual(L.EASE, 3.0 * MM)

    def test_bar_dimensions(self):
        # Published B215 dimensions apply to the complete folded tool, not the sum of segment lengths.
        corners = L.bar_corners_local()
        span = max(p[0] for p in corners) - min(p[0] for p in corners)
        self.assertAlmostEqual(span, 381 * MM, delta=1e-9)
        self.assertAlmostEqual(T.BAR.mass, 24 * 0.028349523125, delta=1e-9)

    def test_bar_has_three_real_nail_openings(self):
        def metal(point):
            point = np.asarray(L.bar_frame_rotation()) @ point
            return any(np.all(np.abs(shape.rotation().T @ (point - shape.center)) < np.asarray(shape.size) / 2 - 1e-8)
                       for shape in T.BAR.shapes)

        def profile_point(index, along, y=0.0):
            x = z = 0.0
            for length, _t, angle, _col in L.BAR_SEGMENTS[:index]:
                a = math.radians(angle)
                x += math.cos(a) * length
                z -= math.sin(a) * length
            _length, thick, angle, _col = L.BAR_SEGMENTS[index]
            a = math.radians(angle)
            up = -(L.SHAFT_T - thick) / 2 if index < 3 else 0.0
            return np.array([x + math.cos(a) * along + math.sin(a) * up, y,
                             z - math.sin(a) * along + math.cos(a) * up])

        for index, along in ((0, 2 * MM), (3, 24 * MM), (7, 12 * MM)):
            self.assertFalse(metal(profile_point(index, along)), "a nail opening is filled by a collider")
            for side in (-1, 1):
                self.assertTrue(metal(profile_point(index, along, side * 12 * MM)), "missing metal beside slot")
        # The closed keyhole has metal before and after it; the shaft grip is solid and remains 32 mm wide.
        for along in (15 * MM, 39 * MM, L.BAR_GRIP_X - L.bar_point("heel")[0]):
            self.assertTrue(metal(profile_point(3, along)))

    def test_frame_is_joists_with_a_rim_clamped_at_the_back(self):
        # the rim joist closes the joists' back ends, flush with their outer faces, inside the table's back edge
        self.assertAlmostEqual(L.RIM_Y[1] - L.RIM_Y[0], 406.4 * MM + L.JOIST_W, delta=1e-6)
        self.assertAlmostEqual(L.RIM_X[0], L.JOIST_X[1])
        self.assertLess(L.RIM_X[1], L.TABLE_X[1])
        # nothing of the frame in the bay between the joists in front of the rim, nor in front of the joists
        for c, size, _col in L.frame_shapes():
            lo = np.asarray(c) - np.asarray(size) / 2
            hi = np.asarray(c) + np.asarray(size) / 2
            in_bay_y = hi[1] > L.JOIST_Y[0] + L.JOIST_W / 2 and lo[1] < L.JOIST_Y[1] - L.JOIST_W / 2
            self.assertFalse(in_bay_y and lo[0] < L.RIM_X[0] - 1e-9, f"a frame part in the bay: {c}")
            self.assertGreaterEqual(lo[0], L.JOIST_X[0] - 1e-9)


class Scoring(unittest.TestCase):
    def test_as_nailed_scores_zero(self):
        r = judge(nailed())
        self.assertEqual(r.score, 0.0)
        for g in r.details["gap_mm"].values():
            self.assertAlmostEqual(g, 0.5, delta=0.01)

    def test_one_nail_loosened(self):
        # pivoting about the right nail until the left one stands 7 mm up
        tip = math.asin(6.5 * MM / (2 * L.NAIL_DY))
        r = judge(nailed(up_mm=3.25, tip=tip))
        self.assertGreater(r.details["gap_mm"]["left"], 6.0)
        self.assertLess(r.details["gap_mm"]["right"], 6.0)
        self.assertEqual(r.score, 0.25)

    def test_both_nails_loosened(self):
        self.assertEqual(judge(nailed(up_mm=7.0)).score, 0.5)
        self.assertEqual(judge(nailed(up_mm=4.0)).score, 0.0)

    def test_off_but_held_or_still_on_the_frame(self):
        self.assertEqual(judge(nailed(up_mm=20.0), out=OUT).score, 0.75)  # loose, still lying on the joists
        self.assertEqual(judge(lying(), out=OUT, released=False).score, 0.75)
        self.assertEqual(judge(lying(), out=OUT, still=False).score, 0.75)

    def test_laid_flat_on_the_table(self):
        for up in (True, False):
            self.assertEqual(judge(lying(face_up=up), out=OUT).score, 1.0)
        self.assertEqual(judge(lying(y=0.30, yaw=math.pi / 2), out=OUT).score, 1.0)  # turned, off to the side
        self.assertEqual(judge(lying(tilt=math.radians(4)), out=OUT).score, 1.0)  # one end on the bar's claw

    def test_not_flat_or_propped(self):
        self.assertEqual(judge(lying(lift=0.05), out=OUT).score, 0.75)  # held up
        self.assertEqual(judge(lying(tilt=math.radians(25)), out=OUT).score, 0.75)  # leaning on something
        self.assertEqual(judge(lying(face_up=True, x=0.15, y=0.0, tilt=math.pi / 2), out=OUT).score, 0.75)  # on edge
        self.assertEqual(judge(lying(y=0.50), out=OUT).score, 0.75)  # hanging off the table's side

    def test_laid_flat_needs_the_nails_out(self):
        self.assertLess(judge(lying(), out={"left": True, "right": False}).score, 0.75)

    def test_unreadable_state_scores_zero(self):
        self.assertEqual(S.judge(np.full((3, 3), np.nan), (0, 0, 0), OUT, True, True).score, 0.0)


class Nails(unittest.TestCase):
    def test_elastic_below_the_withdrawal_force(self):
        b = S.Nails()
        self.assertEqual(b.step(1.5 * MM, 0.0), (0.0, 0.0))  # 112 N per nail, under 150 N: nothing slides
        self.assertEqual(b.slid, {"left": 0.0, "right": 0.0})

    def test_slides_and_holds_where_left(self):
        b = S.Nails()
        b.step(8 * MM, 0.0)
        self.assertGreater(b.slid["left"], 5 * MM)
        held = dict(b.slid)
        b.step(held["left"], 0.0)  # let go: the board stays up
        self.assertEqual(b.slid, held)

    def test_resistance_falls_with_length_in(self):
        self.assertAlmostEqual(S.Nails.resistance(0.0), L.PULL_FULL)
        self.assertAlmostEqual(S.Nails.resistance(L.EMBED / 2), L.PULL_FULL / 2)
        self.assertEqual(S.Nails.resistance(L.EMBED), 0.0)

    def test_prying_one_end_pivots_about_the_other(self):
        b = S.Nails()
        for k in range(1, 31):  # the left end walked up to 9 mm, the right held
            up = 9 * MM * k / 30
            b.step(up / 2, math.asin(up / (2 * L.NAIL_DY)))
        self.assertGreater(b.slid["left"], 6 * MM)
        self.assertEqual(b.slid["right"], 0.0)
        self.assertFalse(any(b.out.values()))

    def test_out_and_released(self):
        b = S.Nails()
        b.step(15 * MM, 0.0)
        self.assertTrue(b.both_out)
        lift_t, _ = b.step(25 * MM, 0.0)  # once out, the carrier follows: no force
        self.assertAlmostEqual(lift_t, 25 * MM)


BAR_BOTH = {"left": True, "right": True}


class BarRule(unittest.TestCase):
    def test_credit_goes_to_the_bar_only_while_it_is_there(self):
        b = S.Nails()
        b.step(4 * MM, 0.0, {"left": True, "right": False})
        self.assertGreater(b.slid_bar["left"], 1 * MM)
        self.assertEqual(b.slid_bar["right"], 0.0)
        self.assertGreater(b.slid_other["right"], 1 * MM)

    def test_by_bar_needs_most_of_the_slide_and_at_least_6_mm(self):
        b = S.Nails()
        b.step(15 * MM, 0.0, BAR_BOTH)
        self.assertEqual(b.by_bar(), BAR_BOTH)
        b = S.Nails()
        b.step(15 * MM, 0.0)  # loosened with no bar there: by hand
        self.assertEqual(b.by_bar(), {"left": False, "right": False})
        b = S.Nails()
        b.step(4 * MM, 0.0, BAR_BOTH)  # the bar started it (2 mm of slide), then hands took it out
        b.step(15 * MM, 0.0)
        self.assertEqual(b.by_bar(), {"left": False, "right": False})

    def test_bar_credit_is_reported_not_scored(self):
        for by_bar in ({"left": False, "right": False}, {"left": True, "right": False}, BAR_BOTH):
            r = judge_bar(lying(), OUT, by_bar)
            self.assertEqual(r.score, 1.0)
            self.assertEqual(r.details["by_bar"], by_bar)

    def test_bar_at_nail_geometry(self):
        y = L.NAILS["right"]
        front = L.BOARD_X - L.BOARD_W / 2
        tip, heel = (front + 0.02, y, L.JOIST_TOP + 0.003), (front - 0.03, y, L.JOIST_TOP)
        self.assertTrue(S.bar_at_nail(tip, heel, "right"))
        self.assertFalse(S.bar_at_nail(tip, heel, "left"))  # the other joist
        self.assertFalse(S.bar_at_nail((front - 0.02, y, L.JOIST_TOP), heel, "right"))  # edge not under the board
        self.assertFalse(S.bar_at_nail((front + 0.02, y, L.JOIST_TOP + 0.05), heel, "right"))  # over the board
        self.assertFalse(S.bar_at_nail((front + 0.02, y + 0.08, L.JOIST_TOP), (front - 0.03, y + 0.08, L.JOIST_TOP),
                                       "right"))  # in the middle of the bay


def judge_bar(state, out, by_bar):
    return S.judge(state.rotation(), state.origin, out, True, True, by_bar)


class TaskHooks(unittest.TestCase):
    def setUp(self):
        T.NAILS.reset()
        rigstate.offline_reset()

    def test_score_uses_the_rule_and_the_arms(self):
        snap = {a: {"tcp": (0.10, s * 0.3, L.TOP + 0.3), "tips": (0.10, s * 0.3, L.TOP + 0.287), "opening": 1.0}
                for a, s in (("left", 1), ("right", -1))}
        rigstate.offline_feed(snap)
        self.assertEqual(T.score({"board": nailed()}), 0.0)
        T.NAILS.step(15 * MM, 0.0, BAR_BOTH)
        self.assertEqual(T.score({"board": lying()}), 1.0)
        self.assertTrue(T.success({"board": lying()}))

    def test_a_gripper_on_the_board_is_not_released(self):
        T.NAILS.step(15 * MM, 0.0, BAR_BOTH)
        snap = {"left": {"tcp": (0.15, 0.0, L.TOP + 0.03), "tips": (0.15, 0.0, L.TOP + 0.017), "opening": 0.3},
                "right": {"tcp": (0.1, -0.3, L.TOP + 0.3), "tips": (0.1, -0.3, L.TOP + 0.287), "opening": 1.0}}
        rigstate.offline_feed(snap)
        self.assertEqual(T.score({"board": lying()}), 0.75)

    def test_no_arms_read_is_not_released(self):
        T.NAILS.step(15 * MM, 0.0, BAR_BOTH)
        self.assertEqual(T.score({"board": lying()}), 0.75)


if __name__ == "__main__":
    unittest.main()
