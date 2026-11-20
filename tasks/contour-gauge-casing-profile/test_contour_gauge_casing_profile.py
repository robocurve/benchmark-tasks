"""CPU tests of the finger latch and the scorer (no simulator). The copy's finger pushes are built from the scene's own
geometry: a gauge pushed square until its frame seats on the casing's outer band, relaxed by the latch's elastic give
(fingers in front of the wall pushed PUSH_NOM, those in front of the casing that much more, those in the opening none)."""

import math

import numpy as np
import pytest

import contour_gauge_casing_profile as t
from labkit import PartState

CLEAR = ((0.20, 0.30, t.TOP + 0.20), (0.16, -0.32, t.TOP + 0.20))


def _ideal_mm(skew_deg=0.0, shift=0.0):
    """Pushes (mm) of a gauge pushed home with its centre `shift` m along the wall from the casing's centre line, its
    frame skewed `skew_deg` out of square with the wall."""
    out = []
    for u in t.FINGER_U:
        y = u + shift
        if y + t.F_THICK / 2 <= t.JAMB_Y:  # in the door opening: nothing in front of it
            out.append(0.0)
            continue
        out.append(1000 * (t.PUSH_NOM + t.finger_reading(y - t.CAS_Y)) + math.tan(math.radians(skew_deg)) * u * 1000)
    return out


COPY_MM = _ideal_mm()


def _q(mm):
    return {n: v / 1000.0 for n, v in zip(t.FINGERS, mm)}


X_GOAL = t.CUT_X + t.PUSH_NOM  # finger-tip line with the ideal copy's wall line on the pencilled line


def _gauge(mm, x=X_GOAL, y=t.G0_Y, z=None, yaw=0.0, tilt=0.0, flip=False):
    """Gauge lying flat (turned `yaw` about the vertical, tipped `tilt` about its long axis; `flip` = upside down)."""
    q = _q(mm)
    if z is None:
        z = t.PL_TOP
    r = np.eye(3)
    if flip:
        r = np.diag([1.0, -1.0, -1.0])
    if tilt:
        c, s = math.cos(tilt), math.sin(tilt)
        r = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]) @ r
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ r
    w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-3:
        quat = ((r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w)
    else:  # 180 deg turn about x (upside down)
        quat = (1.0, 0.0, 0.0, 0.0)
    z0 = np.zeros(3)
    return PartState(pos=np.array([x, y, z]), quat_xyzw=np.array(quat), lin_vel=z0, ang_vel=z0, low=z0, high=z0, joints=q)


@pytest.fixture(autouse=True)
def trial():
    t._reset(None)
    t._set_test_trial(tcps=CLEAR)
    yield
    t._reset(None)


def _score(g):
    return t.score({"gauge": g, "casing": None, "wall": None, "plank": None})


def test_latch_holds_then_slips():
    lim = t.FINGER_HOLD_N / t.K_FINGER
    assert t.latch(0.001, 0.0) == pytest.approx(0.0)  # within the elastic band: held
    assert t.latch(0.010, 0.0) == pytest.approx(0.010 - lim)  # pushed past FINGER_HOLD_N: slips, latch follows


def test_profile_table_matches_casing():
    assert t.profile_h(-t.CAS_W / 2 + 0.005) == pytest.approx(0.012)  # raised bead
    assert t.profile_h(-t.CAS_W / 2 + 0.020) == pytest.approx(0.006)  # field
    assert t.profile_h(t.CAS_W / 2 - 0.002) == pytest.approx(t.CAS_H)
    assert t.profile_h(t.CAS_W / 2 + 0.001) == 0.0


def test_casing_boxes_reproduce_the_profile():
    """The casing's boxes stand off the wall by the profile's height at every point across it (the bevel's plate
    included): sample 1 mm steps and take the most proud box face."""
    for k in range(0, 57):
        c = -t.CAS_W / 2 + (k + 0.5) / 1000.0  # mid-millimetre: off the steps
        proud = 0.0
        for centre, size, _, (yaw, _p) in t.CASING.boxes:
            ca, sa = math.cos(yaw), math.sin(yaw)
            # box local frame: x across its thickness, y along it; find its most proud (most -x) point at y = c
            for sx in (-1, 1):
                for sy in np.linspace(-1, 1, 201):
                    px = centre[0] + ca * sx * size[0] / 2 - sa * sy * size[1] / 2
                    py = centre[1] + sa * sx * size[0] / 2 + ca * sy * size[1] / 2
                    if abs(py - c) < 0.0003:
                        proud = max(proud, -px)
        assert proud == pytest.approx(t.profile_h(c), abs=0.0006), c


def test_board_edge_in_line_with_the_casing_inner_edge():
    assert t.PL_EDGE_Y == pytest.approx(t.CAS_Y - t.CAS_W / 2)
    assert t.JAMB_Y < t.PL_EDGE_Y  # the opening lies beyond the casing's inner edge, off the board


def test_start_scores_zero():
    """Do nothing: every finger fully out, the gauge where it lies at the start."""
    g = _gauge([0.0] * t.N_FINGERS, x=t.G0_X, y=t.G0_Y, z=t.TOP)
    assert _score(g) == 0.0 and not t.copied(g)


def test_goal_scores_one():
    g = _gauge(COPY_MM)
    f = t.profile_fit(g.joints)
    assert f["rms_mm"] <= t.RMS_MM and f["max_mm"] <= t.MAX_MM and f["slope_deg"] <= t.SQUARE_DEG
    assert f["on_casing"] >= 7 and len(f["wall_idx"]) >= 3
    m = t.marking(g)
    assert max(abs(d) for d in m["line_dx"]) < 1e-6 and m["inner_y"] == pytest.approx(t.PL_EDGE_Y, abs=0.001)
    assert _score(g) == 1.0 and t.success({"gauge": g})


def test_partial_copied_not_placed():
    g = _gauge(COPY_MM, x=t.SEAT_X - 0.06, y=0.0, z=t.TOP + 0.08)
    assert _score(g) == pytest.approx(t.PARTIAL)


def test_hands_still_on_the_gauge_is_partial():
    t._set_test_trial(tcps=((t.PLACE_X + t.GRIP_DX, t.GRIP_U, t.PL_TOP + 0.03),
                            (t.PLACE_X + t.GRIP_DX, -t.GRIP_U, t.PL_TOP + 0.03)))
    assert _score(_gauge(COPY_MM)) == pytest.approx(t.PARTIAL)


def _hands_at(dist, z):
    c = t._frame_centre(_gauge(COPY_MM))
    return ((c[0] + dist, c[1], z), (c[0] - dist, c[1], z))


def test_hands_clear_gate_distances():
    low = t.PL_TOP + 0.03
    g = _gauge(COPY_MM)
    t._set_test_trial(tcps=_hands_at(t.HAND_CLEAR_M - 0.01, low))
    assert _score(g) == pytest.approx(t.PARTIAL)
    t._set_test_trial(tcps=_hands_at(t.HAND_CLEAR_M + 0.01, low))
    assert _score(g) == 1.0
    t._set_test_trial(tcps=_hands_at(0.02, t.PL_TOP + 0.20))
    assert _score(g) == 1.0


def test_hands_clear_negative_control_gate_removed(monkeypatch):
    g = _gauge(COPY_MM)
    t._set_test_trial(tcps=_hands_at(0.02, t.PL_TOP + 0.03))
    assert _score(g) == pytest.approx(t.PARTIAL)
    monkeypatch.setattr(t, "_hands_clear", lambda tcps, ps: True)
    assert _score(g) == 1.0


def test_demo_final_hands_clear():
    d = t.demo()
    tcps = tuple(tuple(float(v) for v in d[side][-1].pos) for side in ("left", "right"))
    assert t._hands_clear(tcps, _gauge(COPY_MM))


@pytest.mark.parametrize("deg, ok", [(2.0, True), (4.0, False), (8.0, False)])
def test_skewed_push(deg, ok):
    """A copy pushed with the frame skewed against the wall (one hand ahead of the other): up to 3 deg is square enough."""
    assert bool(t.copied(_gauge(_ideal_mm(skew_deg=deg)))) == ok


def test_off_centre_push_fails():
    """Pushed with the casing in front of the gauge's end: too few fingers read it."""
    assert _score(_gauge(_ideal_mm(shift=-0.110))) == 0.0


def test_pushed_against_bare_wall_fails():
    """Pushed against the wall away from the casing: every finger pushed level, no outline."""
    assert _score(_gauge([12.0] * t.N_FINGERS)) == 0.0


@pytest.mark.parametrize("dx_mm, ok", [(1.5, True), (2.5, False), (-2.5, False)])
def test_wall_line_on_the_pencilled_line(dx_mm, ok):
    g = _gauge(COPY_MM, x=X_GOAL + dx_mm / 1000)
    assert (_score(g) == 1.0) == ok and _score(g) >= t.PARTIAL


@pytest.mark.parametrize("dy_mm, ok", [(2.5, True), (-2.5, True), (4.0, False), (-4.0, False)])
def test_inner_edge_at_the_board_edge(dy_mm, ok):
    g = _gauge(COPY_MM, y=t.G0_Y + dy_mm / 1000)
    assert (_score(g) == 1.0) == ok and _score(g) >= t.PARTIAL


def test_gauge_not_flat_or_turned_is_partial():
    assert _score(_gauge(COPY_MM, tilt=math.radians(8))) == pytest.approx(t.PARTIAL)  # propped on its edge
    assert _score(_gauge(COPY_MM, z=t.PL_TOP + 0.01)) == pytest.approx(t.PARTIAL)  # held above the board
    assert _score(_gauge(COPY_MM, yaw=math.radians(8))) == pytest.approx(t.PARTIAL)  # fingers off the board's axis


def test_turned_end_for_end_is_partial():
    """Fingers pointing back toward the robot: the outline is no longer at the board's end."""
    g = _gauge(COPY_MM, x=t.CUT_X - t.PUSH_NOM, yaw=math.pi)
    assert _score(g) == pytest.approx(t.PARTIAL)


def test_upside_down_mirrors_the_outline():
    """Laid face down the copy is mirrored: the outer band lands off the board, so it is not in the marking position."""
    g = _gauge(COPY_MM, flip=True, z=t.PL_TOP + t.FRAME_W)
    assert _score(g) == pytest.approx(t.PARTIAL)


def test_demo_reads_no_state():
    plan = t.demo()
    assert set(plan) == {"left", "right"}
    for wps in plan.values():
        assert sum(w.duration for w in wps[-1:]) >= 8.0  # parked >= 8 s at the end
    assert sum(w.duration for w in plan["left"]) == pytest.approx(sum(w.duration for w in plan["right"]))
    assert sum(w.duration for w in plan["left"]) < 1200 / 20  # within the run limit


def test_demo_press_reaches_the_seat():
    """The push is commanded past the point where the frame seats on the outer band; the approach stops short of the
    casing; the draw-back clears every finger tip from it."""
    xs = [w.pos[0] - t.GRIP_DX for w in t.demo()["left"][:10]]
    assert max(xs) == pytest.approx(t.SEAT_X + t.PRESS_OVER)
    assert xs[4] < t.WALL_X - t.CAS_H
    assert xs[8] - t.PUSH_NOM < t.WALL_X - t.CAS_H


NOISY_MM = [v + d for v, d in zip(COPY_MM, np.random.default_rng(3).uniform(-0.6, 0.6, t.N_FINGERS))]


def test_fit_screens_every_gate_before_lowest_rms():
    f = t.profile_fit(_q(NOISY_MM))
    assert f["ref_spread_mm"] <= t.REF_SPREAD_MM and f["max_mm"] <= t.MAX_MM and f["rms_mm"] <= t.RMS_MM
    assert t.copied(_gauge(NOISY_MM))


def test_count_gate_is_a_guard_only():
    assert int(t._N_ON.min()) >= 7 >= t.MIN_ON_CASING


def _reading(c, h):
    return max(max(h(c + d) - r, 0.0) for d, r in zip(t._FOOT, t._ROUND))


def _ramp(c):
    x = c + t.CAS_W / 2
    return 0.006 + 0.0115 * x / t.CAS_W if 0.0 <= x <= t.CAS_W else 0.0


@pytest.mark.parametrize("off_mm", [-3.0, -1.5, 1.5, 3.0])
def test_ramp_stand_in_fails(off_mm):
    """A plain bevel of the same rise (6 to 17.5 mm) pushed perfectly: not the casing's profile, fails the gate."""
    q = [12.0 + 1000 * _reading(u - off_mm / 1000, _ramp) for u in t.FINGER_U]
    assert not t.copied(_gauge(q))


def test_shortcut_flat_block_fails():
    """A flat block of the casing's width pushed perfectly (all fingers on it level): errs by > 3 mm."""
    flat = lambda c: 0.012 if abs(c) <= t.CAS_W / 2 else 0.0  # noqa: E731
    q = [12.0 + 1000 * _reading(u, flat) for u in t.FINGER_U]
    assert t.profile_fit(_q(q))["max_mm"] > t.MAX_MM and _score(_gauge(q)) == 0.0
