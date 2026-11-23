"""Scoring checks with no simulator: python -m pytest -q test_score.py"""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

import task as t
from tack_t_joint import mech as M
from yam_arena import ik

DT = 0.05
PARK = {"left_tcp": (0.12, 0.34, t.PARK[2]), "right_tcp": (0.12, -0.34, t.PARK[2])}
B = np.array((0.0, math.sin(t.WORK), math.cos(t.WORK)))  # the corner's bisector, out of the root on the left side


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))


def quat(R):
    return np.asarray(ik.matrix_to_quat_xyzw(np.asarray(R, float)), float)


def state(pos, R=np.eye(3), v=0.0):
    p = np.array(pos, float)
    return NS(pos=p, quat_xyzw=quat(R), lin_vel=np.array((v, 0.0, 0.0)), ang_vel=np.zeros(3), low=p, high=p)


def upright(lean=0.0, dy=0.0, dx=0.0, lift=0.0, end_up=0.0, on_end=False):
    """The coupon standing on the line: leaned ``lean`` (rad, top toward the torch's side), shifted, lifted, one end raised
    by turning ``end_up`` (rad) about its far bottom corner, or standing on a saw-cut end instead of a rolled edge."""
    R = rx(math.pi / 2 - lean)
    if on_end:
        R = R @ np.array(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    foot = np.array((M.XC + dx, M.YC + dy, t.BASE_Z + lift))
    c = foot + (M.W / 2) * np.array((0.0, math.sin(lean), math.cos(lean)))
    if end_up:
        ca, sa = math.cos(end_up), math.sin(end_up)
        Ry = np.array(((ca, 0.0, -sa), (0.0, 1.0, 0.0), (sa, 0.0, ca)))  # the +x end rises
        pivot = np.array((M.X0 + dx, M.YC + dy, t.BASE_Z))
        c, R = pivot + Ry @ (c - pivot), Ry @ R
    return state(c, R)


def torch_at(tip, held="left_tcp"):
    """The torch at its work angle with the tungsten tip at ``tip``; ``held`` names the hand on it (None: nobody's)."""
    R, g, T = t.torch_pose(tip, t.TORCH_TILT)
    return state(g, R), (held, state(g + T @ np.array((0.0, 0.0, t.GRIP_DZ))))


def world(coupon=None, holding=True, **over):
    w = {a: state(p) for a, p in PARK.items()}
    w["left_grip"], w["right_grip"] = state((0.0, 0.0, 0.0)), state((0.0, 0.0, 0.0))  # N on each finger from the coupon
    w["torch"] = state((M.TORCH_REST["x"], M.TORCH_REST["y"], t.TOP + M.TORCH_REST_Z), t._rz(M.TORCH_REST["lean"]))
    w["coupon"] = coupon if coupon is not None else state((M.COUPON_START[0], M.COUPON_START[1], t.TOP + M.COUPON_START[2]))
    if coupon is not None and holding:
        w["right_tcp"], w["right_grip"] = state(coupon.pos), state((15.0, 15.0, 0.0))
    w.update(over)
    return w


def root(x, side=1, coupon=None):
    """The root point at world x on the coupon's left (side 1) or right (side 0) face."""
    y = t.Y_LINE if side else t.Y_LINE - M.T
    if coupon is not None:
        j = t.joint({"coupon": coupon})
        f = int(np.argmax(j["fitted"].sum(axis=1)))
        pts = j["root"][f]
        s = int(np.argmax(pts[0, :, 1])) if side else int(np.argmin(pts[0, :, 1]))
        k = int(np.argmin(np.abs(pts[:, s, 0] - x)))
        return pts[k, s].copy()
    return np.array((x, y, t.BASE_Z))


def tack(w, x, dwell=2.5, dist=0.003, held="left_tcp", side=1, dz=0.0):
    out = B if side else B * (1.0, -1.0, 1.0)
    tip = root(x, side, w["coupon"]) + dist * out + (0.0, 0.0, dz)
    torch, (hand, tcp) = torch_at(tip, held)
    w = dict(w, torch=torch)
    if hand:
        w[hand] = tcp
    for _ in range(round(dwell / DT)):
        t.note_step(w, DT)
    return w


def three(coupon, **kw):
    w = world(coupon)
    for i in (0, 2, 1):
        tack(w, M.MARKS[i], **kw)
    return w


def settle(coupon, n=30, **over):
    w = world(coupon, holding=False, **over)
    for _ in range(n):
        t.note_step(w, DT)
    return t.score(w)


@pytest.fixture(autouse=True)
def fresh():
    t.reset_latch()


def test_start_scores_zero():
    assert settle(None) == 0.0


def test_full_marks():
    c = upright()
    three(c)
    assert t.sound_tacks() == [True] * 3 and not t.faults() and settle(c) == 1.0


def test_three_tacks_lock_commands():
    c = upright()
    three(c)
    cmds = t.take_commands()
    assert [d["do"] for d in cmds] == ["lock"] * 3 and [d["slot"] for d in cmds] == [0, 1, 2]
    assert sorted(round(float(d["anchor"][0]), 3) for d in cmds) == pytest.approx(sorted(M.MARKS), abs=0.002)
    assert all(abs(d["anchor"][1] - t.Y_LINE) < 1e-6 and abs(d["anchor"][2] - t.BASE_Z) < 1e-9 for d in cmds)


def test_needs_a_second_of_rest():
    c = upright()
    three(c)
    assert settle(c, n=10) == 0.8


def test_torch_not_back_scores_point_eight():
    c = upright()
    three(c)
    torch, (hand, tcp) = torch_at((0.25, 0.10, t.TOP + 0.10))
    assert settle(c, torch=torch, left_tcp=tcp) == 0.8


def test_torch_back_but_hand_still_on_it():
    c = upright()
    three(c)
    g = (M.TORCH_REST["x"], M.TORCH_REST["y"], t.TOP + M.TORCH_REST_Z)
    assert settle(c, left_tcp=state(np.add(g, (0.0, 0.0, 0.05)))) == 0.8


def test_hand_still_holding_the_coupon():
    c = upright()
    three(c)
    assert settle(c, right_tcp=state(c.pos)) == 0.45


def test_hand_just_clear_of_the_coupon():
    c = upright()
    three(c)
    assert settle(c, right_tcp=state(c.pos + (0.0, -(M.T / 2 + 0.055), 0.0))) == 0.45  # 5.5 cm from its face: fingers on it
    t.reset_latch()
    three(c)
    assert settle(c, right_tcp=state(c.pos + (0.0, -(M.T / 2 + 0.065), 0.0))) == 1.0


def test_one_and_two_tacks_then_let_go():
    c = upright()
    tack(world(c), M.MARKS[0])
    assert settle(c) == 0.4
    tack(world(c), M.MARKS[2])
    assert settle(c) == 0.6


def test_one_tack_still_held():
    c = upright()
    w = tack(world(c), M.MARKS[0])
    assert t.score(w) == 0.15


def test_out_of_square_stays_out_of_square():
    c = upright(lean=math.radians(5.0))
    three(c)
    assert t.sound_tacks() == [True] * 3 and settle(c) == 0.45
    assert all(np.allclose(tk["R"], t._rot(c.quat_xyzw), atol=1e-6) for tk in t._trial["tacks"])  # the tacks hold the pose they were made in


def test_square_limit_is_two_degrees():
    for deg, want in ((1.5, 1.0), (-1.5, 1.0), (2.5, 0.45), (-2.5, 0.45)):
        t.reset_latch()
        c = upright(lean=math.radians(deg))
        three(c)
        assert settle(c) == want, deg


def test_far_from_square_scores_zero():
    c = upright(lean=math.radians(12.0))
    three(c)
    assert settle(c) == 0.0


def test_off_the_line():
    for dy, want in ((0.001, 1.0), (0.003, 0.45), (-0.003, 0.45), (0.012, 0.0)):
        t.reset_latch()
        c = upright(dy=dy)
        three(c)
        assert settle(c) == want, dy


def test_ends_off_the_ticks():
    c = upright(dx=0.004)
    three(c)
    assert t.sound_tacks() == [True] * 3 and settle(c) == 0.45


def test_hover_makes_no_arc():
    """The torch held 10 mm from the root for as long as three tacks take: no arc, nothing fused, so nothing scored even
    though the coupon is left standing square on the line."""
    c = upright()
    three(c, dist=0.010)
    assert not t._trial["fused"].any() and not t.take_commands() and settle(c) == 0.0


def test_arc_length_window():
    for dist, arc in ((0.001, False), (0.002, True), (0.004, True), (0.005, False)):
        t.reset_latch()
        tack(world(upright()), M.MARKS[1], dwell=0.5, dist=dist)
        assert t._trial["arc"] == arc, dist


def test_torch_not_held_makes_no_tack():
    c = upright()
    three(c, held=None)
    assert not t._trial["fused"].any() and settle(c) == 0.0


def test_coupon_held_off_the_plate_makes_no_tack():
    c = upright(lift=0.003)
    three(c)
    assert not t._trial["fused"].any() and settle(c) == 0.0


def test_root_gap_limit_is_one_mm():
    for lift, fused in ((0.0008, True), (0.0012, False)):
        t.reset_latch()
        w = world(upright(lift=lift))
        tip = np.array((M.MARKS[1], t.Y_LINE, t.BASE_Z)) + 0.003 * B
        torch, (hand, tcp) = torch_at(tip)
        w.update(torch=torch, left_tcp=tcp)
        for _ in range(50):
            t.note_step(w, DT)
        assert bool(t._trial["fused"].any()) == fused, lift


def test_one_end_lifted_tacks_only_where_it_touches():
    c = upright(end_up=math.radians(1.5))  # the far end stands 2 mm off the plate
    three(c)
    assert t.sound_tacks() == [True, True, False]
    assert t.fit({"coupon": c})["gap"] > t.GAP_MAX and settle(c) == 0.30


def test_coupon_lying_flat_on_the_base_plate_cannot_be_tacked():
    c = state((M.XC, M.YC, t.BASE_Z + M.T / 2))
    w = world(c, holding=False)
    for x in M.MARKS:  # the torch dwells at its edge, as if to tack a lap joint
        tip = np.array((x, M.YC + M.W / 2 + 0.002, t.BASE_Z + 0.002))
        torch, (hand, tcp) = torch_at(tip)
        w.update(torch=torch, left_tcp=tcp)
        for _ in range(50):
            t.note_step(w, DT)
    assert not t._trial["fused"].any() and settle(c) == 0.0


def test_short_dwell_makes_no_tack():
    c = upright()
    three(c, dwell=1.0)
    assert not t._trial["fused"].any() and settle(c) == 0.0


def test_long_dwell_overheats_and_caps():
    c = upright()
    w = world(c)
    tack(w, M.MARKS[0], dwell=6.0)
    tack(w, M.MARKS[2])
    tack(w, M.MARKS[1])
    assert "overheated" in t.faults() and t.sound_tacks() == [False, True, True] and settle(c) == 0.4


def test_tungsten_dip_contaminates_and_stops_the_arc():
    c = upright()
    w = world(c)
    tack(w, M.MARKS[0])
    tack(w, M.MARKS[2], dist=0.0003, dwell=0.2)  # the tip into the root
    assert t._trial["dirty"]
    tack(w, M.MARKS[2])
    tack(w, M.MARKS[1])
    assert t.sound_tacks() == [True, False, False] and settle(c) == 0.4


def test_tip_touching_the_coupon_contaminates():
    c = upright()
    w = world(c)
    torch, (hand, tcp) = torch_at((M.XC, t.Y_LINE + 0.0003, t.BASE_Z + 0.03))
    w.update(torch=torch, left_tcp=tcp)
    t.note_step(w, DT)
    assert t._trial["dirty"]


def test_contamination_caps_a_finished_joint():
    c = upright()
    w = three(c)
    tack(w, M.MARKS[1], dist=0.0003, dwell=0.1)
    assert settle(c) == 0.4


def test_a_bead_run_along_the_joint_is_stray_and_capped():
    c = upright()
    w = world(c)
    for x in np.arange(M.X0 + 0.006, M.X1 - 0.005, 0.004):
        tack(w, float(x), dwell=1.0)
    assert t.sound_tacks() == [True] * 3 and "stray" in t.faults() and settle(c) == 0.4


def test_a_fourth_tack_away_from_the_marks_is_stray():
    c = upright()
    w = three(c)
    tack(w, M.MARKS[0] + 0.013)
    assert "stray" in t.faults() and settle(c) == 0.4


def test_tacks_from_the_right_side_count():
    c = upright()
    three(c, side=0)
    assert t.sound_tacks() == [True] * 3 and settle(c) == 1.0


def test_standing_on_a_saw_cut_end_counts():
    c = upright(on_end=True)
    three(c)
    assert settle(c) == 1.0


def test_either_hand_may_hold_the_torch():
    c = upright()
    w = world(c, holding=False, left_tcp=state(c.pos), left_grip=state((15.0, 15.0, 0.0)))
    for i in (0, 2, 1):
        tack(w, M.MARKS[i], held="right_tcp")
    assert settle(c) == 1.0


def test_a_tack_bent_two_degrees_cracks_and_lets_go():
    c = upright()
    three(c)
    t.take_commands()
    bent = upright(lean=math.radians(2.5))  # folded about the line after tacking
    assert settle(bent) == 0.0
    assert [d["do"] for d in t.take_commands()] == ["free"] * 3 and t._trial["cracked"] == 3 and not t._trial["fused"].any()


def test_a_tack_bent_one_degree_holds():
    c = upright()
    three(c)
    assert settle(upright(lean=math.radians(1.0))) == 1.0 and t._trial["cracked"] == 0


def test_knocked_over_after_one_tack_scores_zero():
    c = upright()
    tack(world(c), M.MARKS[0])
    flat = state((M.XC, M.YC + 0.04, t.BASE_Z + M.T / 2))
    assert settle(flat) == 0.0 and t._trial["cracked"] == 1


def test_a_cracked_tack_can_be_made_again():
    c = upright()
    w = three(c)
    for _ in range(3):
        t.note_step(world(upright(lean=math.radians(3.0))), DT)
    assert t.sound_tacks() == [False] * 3
    for i in (0, 2, 1):
        tack(world(c), M.MARKS[i])
    assert t.sound_tacks() == [True] * 3 and settle(c) == 1.0 and [tk["intact"] for tk in t._trial["tacks"]] == [False] * 3 + [True] * 3


def test_tacked_without_holding_is_pulled_out_of_square():
    """Nobody grips the coupon when the first tack sets: the joint is set 3 degrees toward the tack's side, so the tacks
    hold it out of square however well it stood."""
    c = upright()
    tack(world(c, holding=False), M.MARKS[0], dwell=1.6)  # just fused; the coupon then moves to where the tack set
    (cmd,) = t.take_commands()
    Rt = t._rot(cmd["quat"])
    assert t._trial["tacks"][0]["pulled"] and math.degrees(t._angle(Rt, t._rot(c.quat_xyzw))) == pytest.approx(3.0, abs=1e-6)
    assert Rt[1, 1] > 0.05  # the coupon's top (its +y edge) leans toward the torch's side
    pulled = state(cmd["pos"], Rt)
    assert math.degrees(t.fit({"coupon": pulled})["lean"]) == pytest.approx(3.0, abs=0.01)
    for i in (2, 1):  # the later tacks find it restrained by the first and hold it where it is
        tack(world(pulled, holding=False), M.MARKS[i])
    assert [tk["pulled"] for tk in t._trial["tacks"]] == [True, False, False] and settle(pulled) == 0.45


def test_a_tack_from_the_right_pulls_to_the_right():
    c = upright()
    tack(world(c, holding=False), M.MARKS[0], side=0, dwell=1.6)
    assert t._rot(t.take_commands()[0]["quat"])[1, 1] < -0.05


def test_a_hand_round_the_coupon_without_pressing_does_not_hold_it():
    """Holding is contact, not nearness: the hand is where a holding hand would be, but its fingers press with nothing
    (or one finger only, or under 0.5 N each), so the first tack pulls the coupon."""
    for forces in ((0.0, 0.0), (20.0, 0.0), (0.0, 20.0), (0.4, 0.4)):
        t.reset_latch()
        c = upright()
        tack(world(c, holding=False, right_tcp=state(c.pos), right_grip=state((*forces, 0.0))), M.MARKS[0], dwell=1.6)
        assert t._trial["tacks"][0]["pulled"], forces


def test_both_fingers_pressing_half_a_newton_holds_it():
    c = upright()
    tack(world(c, holding=False, right_tcp=state(c.pos), right_grip=state((0.6, 0.6, 0.0))), M.MARKS[0], dwell=1.6)
    assert not t._trial["tacks"][0]["pulled"]


def test_the_torch_hand_cannot_be_the_holding_hand():
    c = upright()
    w = world(c, holding=False, left_grip=state((15.0, 15.0, 0.0)))  # forces on the hand that also carries the torch
    tack(w, M.MARKS[0], dwell=1.6)
    assert t._trial["tacks"][0]["pulled"]


def test_gripped_coupon_is_tacked_where_it_is():
    c = upright()
    tack(world(c), M.MARKS[0])
    assert not t._trial["tacks"][0]["pulled"] and np.allclose(t._rot(t.take_commands()[0]["quat"]), t._rot(c.quat_xyzw), atol=1e-6)


def test_a_new_tack_is_not_cracked_while_the_coupon_moves_to_it():
    c = upright()
    tack(world(c, holding=False), M.MARKS[0], dwell=1.6)
    assert t._trial["tacks"][0]["intact"] and t._trial["cracked"] == 0


def test_pulled_tack_cracks_if_the_coupon_is_kept_from_following():
    c = upright()
    tack(world(c, holding=False), M.MARKS[0], dwell=3.0)  # the coupon stays square: 3 degrees off where the tack set
    assert t._trial["cracked"] == 1 and [d["do"] for d in t.take_commands()] == ["lock", "free"]


def test_standing_untacked_scores_zero():
    assert settle(upright()) == 0.0


def test_a_part_off_the_table_scores_zero():
    c = upright()
    three(c)
    w = world(c, holding=False, torch=state((0.5, 0.6, t.TOP - 0.3)))
    t.note_step(w, DT)
    assert settle(c) == 0.0


def test_coupon_moving_is_not_standing():
    c = upright()
    three(c)
    moving = NS(**{**vars(c), "lin_vel": np.array((0.05, 0.0, 0.0))})
    assert settle(moving) == 0.45


def test_score_bands_cover_every_count():
    seen = set()
    for marks in ((), (0,), (0, 2), (0, 2, 1)):
        for held in (True, False):
            t.reset_latch()
            c = upright()
            w = world(c)
            for i in marks:
                tack(w, M.MARKS[i])
            seen.add(settle(c, right_tcp=state(c.pos)) if held else settle(c))
    assert seen == {0.0, 0.15, 0.3, 0.45, 0.4, 0.6, 1.0}


def test_steel_masses_and_torch_balance():
    assert M.COUPON_MASS == pytest.approx(7850.0 * 0.0762 * 0.0762 * 0.00635) and 0.28 < M.COUPON_MASS < 0.30
    assert 0.2 <= M.TORCH_TOTAL <= 0.35 and np.linalg.norm(M.TORCH_COM[:2]) < 0.0015


def test_the_torch_reaches_the_root_without_its_cup_touching():
    tip = np.array((M.MARKS[1], t.Y_LINE, t.BASE_Z)) + 0.003 * B
    assert t.cup_clearance(tip) > 0.0015
    torch, _ = torch_at(tip)
    assert np.allclose(t.tungsten({"torch": torch}), tip, atol=1e-9)


def test_tack_strength_is_one_newton_metre():
    assert M.TACK_K * M.TACK_BREAK_DEG == pytest.approx(1.0)
    plastic = 400e6 * 0.004 * 0.0015 ** 2 / 4  # ultimate strength x length x throat^2 / 4, N m
    assert 0.5 < plastic < 1.5


def test_untacked_coupon_tips_past_two_and_a_half_degrees():
    assert math.degrees(math.atan2(M.LAND / 2, M.W / 2)) == pytest.approx(2.5, abs=0.1)


def test_demo_plan_reaches_and_fits_the_run_limit():
    wps = t.demo()
    pe, re, bad, _ = t.ik_errors(wps)
    assert not bad and max(sum(w.duration for w in ws) for ws in wps.values()) * 20 < 1800
