"""Scoring checks with no simulator: python -m pytest -q test_score.py"""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

import task as t
from pan_toss import pan as P

DT = 0.05
PARK = {"left_tcp": (0.05, 0.305, 1.05), "right_tcp": (0.05, -0.305, 1.05)}
DOWN, SIDE, UP = 0.0, 90.0, 180.0  # a die's roll about its own x axis: browned face down, on a side, up


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[v * math.sin(a), math.cos(a)]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array((aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz))


def state(pos, q=(0.0, 0.0, 0.0, 1.0), v=0.0):
    p = np.array(pos, float)
    return NS(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.array((v, 0.0, 0.0)), ang_vel=np.zeros(3), low=p, high=p)


def world(rolls=(DOWN,) * 12, pan_at=None, pan_tilt=0.0, hands=None, where=None, moving=False):
    """The pan (by default at rest on the burner) with each die at its starting spot on the cooking surface, rolled by
    ``rolls`` degrees. ``where`` moves chosen dice to world points; ``hands`` replaces the parked grippers."""
    pan_xyz = np.array((*t.PAN_XY, t.PAN_Z)) if pan_at is None else np.array(pan_at, float)
    pq = qmul(quat((0, 0, 1), math.degrees(t.HEADING)), quat((0, 1, 0), pan_tilt))
    w = {h: state(p) for h, p in (hands or PARK).items()}
    w["pan"] = state(pan_xyz, pq)
    r = t._rot(pq)
    for i, name in enumerate(t.DICE):
        x, y, yaw = P.DICE_START[i]
        p = pan_xyz + r @ np.array((x, y, P.DIE / 2))
        w[name] = state(p, qmul(pq, qmul(quat((0, 0, 1), yaw), quat((1, 0, 0), rolls[i]))), v=0.05 if moving else 0.0)
    for i, p in (where or {}).items():
        w[t.DICE[i]] = state(p, w[t.DICE[i]].quat_xyzw)
    return w


def aloft_world(n=t.TOSS_K):
    """The pan at rest on the burner with its first ``n`` dice in the air over it, centres 1 cm above the rim's plane."""
    spots = {i: tuple(t.pan_to_world((P.DICE_START[i][0], P.DICE_START[i][1], P.DEPTH + 0.010))) for i in range(n)}
    return world(where=spots)


def play(w, seconds=1.5, touched=False, tossed=True):
    """Let the record run on a fixed world: latches, and the at-rest timer. ``tossed`` first shows the record one step
    with TOSS_K dice above the rim, as a real toss would."""
    if tossed:
        t.note_step(aloft_world(), DT)
    for _ in range(round(seconds / DT)):
        t.note_step(w, DT, touched=touched)
    return t.score(w)


def rolls(n_turned, how=SIDE):
    return (how,) * n_turned + (DOWN,) * (12 - n_turned)


@pytest.fixture(autouse=True)
def fresh():
    t.reset_trial()
    yield
    t.reset_trial()


TABLE_SPOT = (t.PAN_XY[0], t.PAN_XY[1] - 0.25, t.TOP + P.DIE / 2)
BURNER_SPOT = tuple(t.pan_to_world((0.0, 0.112, -P.T_WALL + P.DIE / 2)))  # on the burner's top, beside the pan
HELD = (t.PAN_XY[0], t.PAN_XY[1], t.PAN_Z + 0.08)


# -- the toss gate (review TK-HNGF9K) ---------------------------------------------------------------------------------
def test_no_toss_scores_zero_however_many_dice_end_turned():
    """Sliding or shaking the dice about, with none ever above the rim, earns nothing even if all twelve end turned."""
    w = world(rolls(12))
    assert play(w, tossed=False) == 0.0 and t.counts(w)["turned"] == 12 and not t.tossed()


def test_toss_needs_enough_dice_above_the_rim_at_once():
    for n in range(t.TOSS_K):
        t.note_step(aloft_world(n), DT)
    w = world(rolls(12))
    assert play(w, tossed=False) == 0.0 and t.trial()["aloft"] == t.TOSS_K - 1
    t.note_step(aloft_world(t.TOSS_K), DT)
    assert play(w, tossed=False) == 1.0


@pytest.mark.parametrize("z_mm", [6.5, 13.0, 30.0, 39.0])
def test_dice_jostled_below_the_rim_are_not_a_toss(z_mm):
    """The level shake lifts die centres to 13 mm above the cooking surface at most; nothing at or below the rim counts."""
    spots = {i: tuple(t.pan_to_world((P.DICE_START[i][0], P.DICE_START[i][1], z_mm / 1000))) for i in range(12)}
    t.note_step(world(where=spots), DT)
    assert t.trial()["aloft"] == 0 and not t.tossed()


def test_die_above_the_rim_plane_but_outside_the_pan_is_not_counted():
    spots = {i: tuple(t.pan_to_world((P.R_RIM + 0.02, 0.0, P.DEPTH + 0.01 * (i + 1)))) for i in range(t.TOSS_K)}
    t.note_step(world(where=spots), DT)
    assert t.trial()["aloft"] == 0


def test_toss_latch_survives_until_reset():
    t.note_step(aloft_world(), DT)
    for _ in range(40):
        t.note_step(world(rolls(12)), DT)
    assert t.tossed()
    t.reset_trial()
    assert not t.tossed() and t.trial()["aloft"] == 0


def test_toss_with_the_pan_in_hand_counts():
    hand = {**PARK, "left_tcp": tuple(np.array(HELD) + t._rz(t.HEADING) @ P.GRASP)}
    spots = {i: tuple(np.array(HELD) + t._rz(t.HEADING) @ np.array((P.DICE_START[i][0], P.DICE_START[i][1], P.DEPTH + 0.02))) for i in range(t.TOSS_K)}
    t.note_step(world(pan_at=HELD, hands=hand, where=spots), DT)
    assert t.tossed() and not t.trial()["lost"].any()


# -- the bands ---------------------------------------------------------------------------------------------------------
def test_start_scores_zero():
    w = world()
    assert play(w) == 0.0 and t.counts(w) == {"turned": 0, "lost": 0, "inside": 12, "net": 0} and not t.success(w)


@pytest.mark.parametrize("n, want", [(0, 0.0), (1, 0.0), (2, 0.0), (3, 0.2), (4, 0.2), (5, 0.4), (6, 0.4), (7, 1.0), (8, 1.0), (10, 1.0), (12, 1.0)])
def test_bands_with_the_pan_back(n, want):
    w = world(rolls(n))
    assert play(w) == pytest.approx(want)
    assert t.success(w) == (want == 1.0)


@pytest.mark.parametrize("n, want", [(2, 0.0), (3, 0.2), (4, 0.2), (5, 0.4), (6, 0.4), (7, 0.6), (12, 0.6)])
def test_bands_with_the_pan_still_in_hand(n, want):
    hand = {**PARK, "left_tcp": tuple(np.array(HELD) + t._rz(t.HEADING) @ P.GRASP)}
    assert play(world(rolls(n), pan_at=HELD, hands=hand)) == pytest.approx(want)


@pytest.mark.parametrize("how", [SIDE, UP, 61.0, 120.0])
def test_turned_poses(how):
    w = world(rolls(12, how))
    assert t.counts(w)["turned"] == 12


@pytest.mark.parametrize("how", [DOWN, 20.0, 59.0, -59.0])
def test_not_turned_poses(how):
    assert t.counts(world(rolls(12, how)))["turned"] == 0


def test_browned_face_normal():
    w = world(rolls(3, UP))
    assert t.brown_nz(w["die_00"]) == pytest.approx(1.0) and t.brown_nz(w["die_05"]) == pytest.approx(-1.0)
    assert t.brown_nz(world(rolls(1, SIDE))["die_00"]) == pytest.approx(0.0, abs=1e-9)


# -- lost dice ---------------------------------------------------------------------------------------------------------
def test_die_on_the_table_is_lost_and_costs_a_turned_one():
    w = world(rolls(9), where={11: TABLE_SPOT})
    assert play(w) == pytest.approx(0.8)  # net 8, pan back, but not the full score: a die is lost
    assert t.counts(w) == {"turned": 9, "lost": 1, "inside": 11, "net": 8}
    t.reset_trial()
    assert play(world(rolls(7), where={11: TABLE_SPOT})) == pytest.approx(0.4)  # net 6: one band down, no bonus below the top band


def test_net_is_turned_minus_lost():
    w = world(rolls(8), where={11: TABLE_SPOT, 10: BURNER_SPOT})
    assert t.counts(w)["net"] == 6 and play(w) == pytest.approx(0.4)
    t.reset_trial()
    w = world(rolls(6), where={11: TABLE_SPOT, 10: BURNER_SPOT, 9: TABLE_SPOT})
    assert t.counts(w)["net"] == 3 and play(w) == pytest.approx(0.2)


def test_die_put_back_after_touching_the_table_stays_lost():
    out = world(rolls(12), where={0: TABLE_SPOT})
    t.note_step(out, DT)
    back = world(rolls(12))
    assert play(back) == pytest.approx(0.8) and t.counts(back)["lost"] == 1 and t.counts(back)["turned"] == 11


def test_die_on_the_burner_beside_the_pan_is_lost():
    out = world(rolls(12), where={0: BURNER_SPOT})
    assert t.spilled(out, "die_00")
    t.note_step(out, DT)
    assert t.counts(world(rolls(12)))["lost"] == 1


def test_all_dice_shaken_out_and_scooped_back_score_zero():
    spots = {i: (TABLE_SPOT[0] + 0.02 * i, TABLE_SPOT[1], TABLE_SPOT[2]) for i in range(12)}
    t.note_step(world(rolls(12), where=spots), DT)
    back = world(rolls(12))
    assert play(back) == 0.0 and t.counts(back)["lost"] == 12


def test_dice_in_flight_are_not_lost():
    air = tuple(t.pan_to_world((0.02, 0.0, 0.12)))  # 12 cm above the cooking surface, over the pan on the burner
    lifted = world(rolls(12), pan_at=HELD, where={0: tuple(np.array(HELD) + (0.0, 0.0, 0.10))})
    for w in (world(rolls(12), where={0: air}), lifted):
        assert not t.spilled(w, "die_00")
        t.note_step(w, DT)
    assert not t.trial()["lost"].any()


def test_dice_in_the_pan_on_the_burner_are_not_spilled():
    w = world()
    assert not any(t.spilled(w, n) for n in t.DICE) and all(t.inside(w, n) for n in t.DICE)


def test_die_left_on_the_rim_or_outside_at_the_end_is_lost():
    on_rim = tuple(t.pan_to_world((P.R_RIM + 0.004, 0.0, P.DEPTH + P.DIE / 2)))
    w = world(rolls(12), where={0: on_rim})
    assert t.counts(w)["lost"] == 1 and play(w) == pytest.approx(0.8)


def test_die_through_the_floor_is_not_inside():
    w = world(rolls(12), pan_at=HELD, where={0: tuple(np.array(HELD) - (0.0, 0.0, 0.004))})
    assert not t.inside(w, "die_00")


def test_pan_tipped_over_scores_zero():
    spots = {i: (t.PAN_XY[0] + 0.15 + 0.015 * i, t.PAN_XY[1] + 0.2, t.TOP + P.DIE / 2) for i in range(12)}
    w = world(rolls(12), pan_at=(t.PAN_XY[0], t.PAN_XY[1] + 0.1, t.TOP + 0.10), pan_tilt=95.0, where=spots)
    assert play(w) == 0.0


# -- touching the food -------------------------------------------------------------------------------------------------
def test_touching_the_food_zeroes_the_trial_for_good():
    w = world(rolls(12))
    t.note_step(w, DT, touched=True)
    assert play(w) == 0.0 and not t.success(w)


def test_touch_latch_survives_until_reset():
    t.note_step(world(), DT, touched=True)
    assert play(world(rolls(12)), seconds=5.0) == 0.0
    t.reset_trial()
    assert play(world(rolls(12))) == 1.0


# -- the pan back on the burner ----------------------------------------------------------------------------------------
@pytest.mark.parametrize("dx, dz, tilt, home", [(0.0, 0.0, 0.0, True), (0.029, 0.0, 0.0, True), (0.031, 0.0, 0.0, False), (0.0, 0.006, 0.0, False),
                                                 (0.0, 0.0, 4.0, True), (0.0, 0.0, 6.0, False), (0.0, -0.006, 0.0, False)])
def test_pan_home(dx, dz, tilt, home):
    w = world(rolls(12), pan_at=(t.PAN_XY[0] + dx, t.PAN_XY[1], t.PAN_Z + dz), pan_tilt=tilt)
    assert t.pan_home(w) == home
    assert play(w) == pytest.approx(1.0 if home else 0.6)


def test_pan_set_down_on_the_table_gets_no_bonus():
    w = world(rolls(12), pan_at=(t.PAN_XY[0], t.PAN_XY[1] - 0.25, t.TOP + P.T_WALL))
    assert not t.pan_home(w) and play(w) == pytest.approx(0.6)


@pytest.mark.parametrize("hand", ["left_tcp", "right_tcp"])
def test_hand_still_on_the_handle_gets_no_bonus(hand):
    w = world(rolls(12), hands={**PARK, hand: tuple(t.pan_to_world(P.GRASP))})
    assert not t.hands_clear(w) and play(w) == pytest.approx(0.6)


def test_hand_clear_distance():
    near = tuple(t.pan_to_world(P.HANDLE_END + (t.CLEAR - 0.005) * P.HANDLE_DIR))
    far = tuple(t.pan_to_world(P.HANDLE_END + (t.CLEAR + 0.005) * P.HANDLE_DIR))
    over = tuple(t.pan_to_world((0.0, 0.0, P.DEPTH + t.CLEAR - 0.005)))
    assert not t.hands_clear(world(hands={**PARK, "left_tcp": near})) and t.hands_clear(world(hands={**PARK, "left_tcp": far}))
    assert not t.hands_clear(world(hands={**PARK, "right_tcp": over}))


def test_still_moving_gets_no_bonus_and_the_timer_restarts():
    assert play(world(rolls(12), moving=True)) == pytest.approx(0.6)
    w = world(rolls(12))
    assert play(w, seconds=0.5) == pytest.approx(0.6)  # at rest for only 0.5 s
    assert play(w, seconds=0.6) == pytest.approx(1.0)
    t.note_step(world(rolls(12), moving=True), DT)
    assert t.score(w) == pytest.approx(0.6)


def test_pan_back_with_nothing_turned_scores_zero():
    assert play(world(rolls(2))) == 0.0


def test_one_lost_die_in_the_top_band_with_the_pan_back_scores_point_eight():
    assert play(world(rolls(8), where={11: TABLE_SPOT})) == pytest.approx(0.8)  # net 7, one lost
    t.reset_trial()
    assert play(world(rolls(8), where={11: TABLE_SPOT}, pan_at=HELD)) == pytest.approx(0.6)  # the same, pan not back


def test_score_never_leaves_zero_to_one():
    for n in range(13):
        for lost in range(0, 13 - n):
            t.reset_trial()
            spots = {11 - k: (TABLE_SPOT[0] + 0.02 * k, TABLE_SPOT[1], TABLE_SPOT[2]) for k in range(lost)}
            s = play(world(rolls(n), where=spots))
            assert 0.0 <= s <= 1.0 and (s < 1.0 or (lost == 0 and n >= t.FULL_NET))


# -- the scene is the real thing ---------------------------------------------------------------------------------------
def test_pan_is_a_true_size_eight_inch_pan():
    assert 2 * P.R_RIM == pytest.approx(0.2032, abs=0.001)  # 8 in across the rim
    assert 0.40 <= P.PAN_MASS <= 0.55 and P.DEPTH == pytest.approx(0.040)
    assert len(P.shell_pieces()) == 4 * P.N_SEG and all(v.shape == (8, 3) for v in P.shell_pieces())
    assert P.floor_piece().shape == (2 * P.N_SEG, 3)


def test_wall_profile_is_continuous_and_flares_out():
    r = [P.wall_radius(z) for z in np.linspace(0.0, P.DEPTH, 41)]
    assert r[0] == pytest.approx(P.R_FLOOR) and r[-1] == pytest.approx(P.R_RIM) and all(b > a for a, b in zip(r, r[1:]))


def test_dice_are_true_size_and_start_apart_inside_the_pan():
    assert 0.012 <= P.DIE <= 0.015 and P.DIE_MASS == pytest.approx(960.0 * P.DIE**3, rel=0.05)
    xy = np.array([(x, y) for x, y, _ in P.DICE_START])
    d = np.linalg.norm(xy[:, None] - xy[None], axis=2) + 10 * np.eye(len(xy))
    assert d.min() > P.DIE * math.sqrt(2) and np.linalg.norm(xy, axis=1).max() + P.DIE * math.sqrt(0.5) < P.R_FLOOR
    assert len(P.DICE_START) == P.N_DICE == len(t.DICE) == 12


def test_wrist_load_is_well_inside_the_motor_limit():
    """Static torque of pan and food about the wrist pitch joint, 0.244 m behind the fingertip midpoint: under 7 N m."""
    mass, (cx, _), _ = t.PAYLOAD
    torque = mass * 9.81 * (0.244 + cx)
    assert 0.40 < mass < 0.60 and torque < 0.35 * 7.0


def test_layout():
    names = [o.name for o in t.OBJECTS]
    assert names[:2] == ["burner", "pan"] and names[2:] == list(t.DICE)
    assert math.hypot(t.PAN_XY[0] - t.BASE[0], t.PAN_XY[1] - t.BASE[1]) == pytest.approx(t.REACH)
    assert "touch" in t.INSTRUCTION and "burner" in t.INSTRUCTION


# -- the open-loop demo's plans (needs yam_arena for the arm's kinematics) ---------------------------------------------
@pytest.mark.parametrize("variant", ["", "shake", "poke", "touch", "none"])
def test_plans_stay_inside_the_joint_limits(variant, monkeypatch):
    ik = pytest.importorskip("yam_arena.ik")
    monkeypatch.setattr(t, "VARIANT", variant)
    monkeypatch.delenv("PAN_TOSS", raising=False)
    plan = t.demo()["left"][0]
    assert np.all(plan.q >= ik.LOWER - 1e-9) and np.all(plan.q <= ik.UPPER + 1e-9)
    assert np.all((plan.grip >= 0.0) & (plan.grip <= 1.0)) and len(plan.q) * t.DT < 60.0
    a, label = plan.action7(t.DT)
    assert a.shape == (7,) and np.allclose(a[:6], plan.q[0])
