"""CPU tests of the finger latch and the scorer (no simulator). Finger pushes in mm are MuJoCo end states of the
two-hand demo and of the one-end press (right hand alone), copied from mj_check rollouts."""

import math

import numpy as np
import pytest

import contour_gauge_casing_profile as t
from labkit import PartState

# mj_check demo, both hands: end state (gauge stood on the plank)
TWO_HAND_MM = [14.3, 14.3, 14.2, 14.2, 14.1, 14.0, 14.0, 13.6, 13.6, 13.5, 13.5, 13.5, 25.6, 23.0, 19.4, 19.9, 23.8, 30.5, 30.4, 13.4, 13.4, 13.4, 13.0, 13.0, 13.6, 13.0, 13.0, 13.0, 12.9, 12.9, 12.8]
# the same plan with the left hand parked: the right hand alone presses its end down, the frame lies 8 deg off level
ONE_END_MM = [27.8, 26.3, 24.9, 23.4, 21.9, 20.5, 19.0, 17.8, 16.5, 15.3, 14.1, 12.8, 12.7, 21.0, 16.7, 12.0, 11.5, 13.8, 19.4, 17.9, 1.9, 0.8, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
CLEAR = ((0.20, 0.26, t.TOP + 0.20), (0.16, -0.32, t.TOP + 0.20))
HELD = ((t.PRESS_X, t.GRIP_U, t.TOP + 0.10), (t.PRESS_X, -t.GRIP_U, t.TOP + 0.10))


def _q(mm):
    return {n: v / 1000.0 for n, v in zip(t.FINGERS, mm)}


def _gauge(mm, x=t.CUT_X, y=t.PL_Y, z=None, yaw=t.TURN, tilt=0.0):
    q = _q(mm)
    if z is None:  # standing on its lowest finger tips on the plank
        z = t.PL_TOP + t.LINE_H - min(q.values())
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    if tilt:  # lean about the world x axis, then turn
        qx = np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])
        x1, y1, z1, w1 = qx
        x2, y2, z2, w2 = qz
        qz = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
              w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
    z0 = np.zeros(3)
    return PartState(pos=np.array([x, y, z]), quat_xyzw=np.array(qz), lin_vel=z0, ang_vel=z0, low=z0, high=z0, joints=q)


def _casing(dx=0.0, dyaw=0.0):
    a = t.TURN + dyaw
    z0 = np.zeros(3)
    return PartState(pos=np.array([t.CAS_X + dx, t.CAS_Y, t.TOP]), quat_xyzw=np.array([0, 0, math.sin(a / 2), math.cos(a / 2)]),
                     lin_vel=z0, ang_vel=z0, low=z0, high=z0, joints={})


@pytest.fixture(autouse=True)
def trial():
    t._reset(None)
    t._set_test_trial(casing0=((t.CAS_X, t.CAS_Y, t.TOP), t.TURN), tcps=CLEAR)
    yield
    t._reset(None)


def _score(g, c=None):
    return t.score({"gauge": g, "casing": c or _casing(), "plank": None})


def test_latch_holds_then_slips():
    lim = t.FINGER_HOLD_N / t.K_FINGER
    assert t.latch(0.001, 0.0) == pytest.approx(0.0)  # within the elastic band: held
    assert t.latch(0.010, 0.0) == pytest.approx(0.010 - lim)  # pushed past FINGER_HOLD_N: slips, latch follows


def test_profile_table_matches_casing():
    assert t.profile_h(-t.CAS_W / 2 + 0.005) == pytest.approx(0.012)  # raised bead
    assert t.profile_h(-t.CAS_W / 2 + 0.020) == pytest.approx(0.006)  # field
    assert t.profile_h(t.CAS_W / 2 - 0.002) == pytest.approx(t.CAS_H)
    assert t.profile_h(t.CAS_W / 2 + 0.001) == 0.0


def test_start_scores_zero():
    """Do nothing: every finger fully out, the gauge where it stands at the start."""
    g = _gauge([0.0] * t.N_FINGERS, x=t.G0_X, y=t.G0_Y, z=t.TOP)
    assert _score(g) == 0.0 and not t.copied(g)


def test_goal_scores_one():
    g = _gauge(TWO_HAND_MM)
    f = t.profile_fit(g.joints)
    assert f["rms_mm"] <= t.RMS_MM and f["max_mm"] <= t.MAX_MM and f["slope_deg"] <= t.SQUARE_DEG
    assert _score(g) == 1.0 and t.success({"gauge": g, "casing": _casing()})


def test_partial_copied_not_placed():
    g = _gauge(TWO_HAND_MM, x=t.PRESS_X, y=0.0, z=t.TOP + 0.08)
    assert _score(g) == pytest.approx(t.PARTIAL)


def test_hands_still_on_the_gauge_is_partial():
    t._set_test_trial(tcps=((t.CUT_X, t.GRIP_U, t.TOP + 0.06), (t.CUT_X, -t.GRIP_U, t.TOP + 0.06)))
    assert _score(_gauge(TWO_HAND_MM)) == pytest.approx(t.PARTIAL)


def _hands_at(dist, z):
    """Both TCPs `dist` metres from the gauge centre in the table plane, at height z above the bench."""
    return ((t.CUT_X + dist, t.PL_Y + 0.0, z), (t.CUT_X - dist, t.PL_Y + 0.0, z))


def test_hands_clear_gate_distances():
    """The full score needs both tool points more than HAND_CLEAR_M from the gauge centre in plan (or high above the
    plank). A distance gate on the tool points only: it says nothing about the camera view (see the proximity proxy below)."""
    low = t.TOP + 0.03
    g = _gauge(TWO_HAND_MM)
    t._set_test_trial(tcps=_hands_at(t.HAND_CLEAR_M - 0.01, low))
    assert _score(g) == pytest.approx(t.PARTIAL)
    t._set_test_trial(tcps=_hands_at(t.HAND_CLEAR_M + 0.01, low))
    assert _score(g) == 1.0
    t._set_test_trial(tcps=_hands_at(0.02, t.PL_TOP + 0.20))
    assert _score(g) == 1.0


def test_hands_clear_negative_control_gate_removed(monkeypatch):
    """With the clearance gate switched off, the same hands-on-gauge state would be scored 1.0."""
    g = _gauge(TWO_HAND_MM)
    t._set_test_trial(tcps=_hands_at(0.02, t.TOP + 0.03))
    assert _score(g) == pytest.approx(t.PARTIAL)
    monkeypatch.setattr(t, "_hands_clear", lambda tcps, ps: True)
    assert _score(g) == 1.0


# -- proximity proxy for occlusion: sight lines from the top camera to the scored points ----------------------------
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station as _station  # noqa: E402

CAM = np.array(_station.top_camera("calibrated").pos, float)  # the task's own top camera (SCENE = "station")
GRIP_R = abs(float(_C.FINGER_OPEN))  # declared engineering approximation of a hand's half-width (finger-pad half spacing)
GRIP_LEN = float(_C.TCP_OFFSET[2])  # declared approximation of the gripper body: tool point up 134 mm (a link-frame offset)


def _scored_points(g):
    """World points the score reads off the gauge: the eight corners of its frame and every finger tip (39 points)."""
    r = g.rotation()
    pos = np.asarray(g.pos, float)
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for z in (t.FRAME_Z0, t.FRAME_Z0 + t.FRAME_H):
                pts.append(r @ np.array([sx * t.FRAME_L / 2, sy * t.FRAME_W / 2, z]) + pos)
    for u, mm in zip(t.FINGER_U, TWO_HAND_MM):
        pts.append(r @ np.array([u, 0.0, mm / 1000.0]) + pos)
    return pts


def _seg_dist(p0, p1, q0, q1):
    """Exact smallest distance between segments p0-p1 and q0-q1 (closest-point algorithm for two segments)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s_, t_ = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if e <= eps:
            t_, s_ = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s_ = min(max((b * f - c * e) / den, 0.0), 1.0) if den > eps else 0.0
            t_ = (b * s_ + f) / e
            if t_ < 0.0:
                t_, s_ = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t_ > 1.0:
                t_, s_ = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p0 + d1 * s_) - (q0 + d2 * t_)))


def _occluded(tcps, g):
    """Proximity proxy, NOT a render check: True if any of the 39 camera-to-point rays passes within GRIP_R of a hand,
    where a hand is a vertical capsule (tool point up GRIP_LEN). It does not test the camera frustum, the arm or wrist,
    gripper self-occlusion or rendered visibility; the capsule is a declared approximation, not the official hand geometry."""
    for tcp in tcps:
        q0 = np.asarray(tcp, float)
        q1 = q0 + np.array([0.0, 0.0, GRIP_LEN])
        for pt in _scored_points(g):
            if _seg_dist(CAM, pt, q0, q1) <= GRIP_R:
                return True
    return False


def _demo_final_tcps():
    d = t.demo()
    return tuple(tuple(float(v) for v in d[side][-1].pos) for side in ("left", "right"))


def test_seg_dist_exact():
    z = np.zeros(3)
    assert _seg_dist(z, np.array([1.0, 0, 0]), np.array([0.5, 0.2, 0]), np.array([0.5, 1.0, 0])) == pytest.approx(0.2)
    assert _seg_dist(z, np.array([1.0, 0, 0]), np.array([2.0, 0, 0]), np.array([3.0, 0, 0])) == pytest.approx(1.0)
    assert _seg_dist(z, np.array([1.0, 1.0, 1.0]), np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 1.0])) == pytest.approx(0.0, abs=1e-9)


def test_occlusion_proxy_rays_to_scored_points_clear_of_hands_at_the_end():
    """Proximity proxy: at the end of the scripted demo none of the 39 rays from the top camera to the gauge's frame
    corners and finger tips passes within a hand capsule's radius of either parked hand. Not tested: camera frustum, arm
    and wrist, gripper self-occlusion, rendered visibility."""
    g = _gauge(TWO_HAND_MM)
    assert not _occluded(_demo_final_tcps(), g)


def test_occlusion_proxy_negative_control_hand_on_a_ray():
    """The proxy can fail: a hand placed on a ray to a finger tip or a frame corner counts as occluding, and so do the
    hands still holding the gauge at the press pose."""
    g = _gauge(TWO_HAND_MM)
    pts = _scored_points(g)
    for pt in (pts[0], pts[8], pts[-1]):
        on_ray = CAM + 0.8 * (pt - CAM)
        assert _occluded((tuple(on_ray), _demo_final_tcps()[1]), g)
    assert _occluded(((t.CUT_X, t.PL_Y + t.GRIP_U, t.TOP + 0.10), _demo_final_tcps()[1]), g)


def test_casing_shoved_is_capped():
    assert _score(_gauge(TWO_HAND_MM), _casing(dx=0.012)) == pytest.approx(t.CAP_SHOVED)
    assert _score(_gauge(TWO_HAND_MM), _casing(dyaw=math.radians(5))) == pytest.approx(t.CAP_SHOVED)


def test_shortcut_one_end_press_right_alone_fails():
    """Right hand alone: its end goes down first, the frame is pressed 8 deg out of square."""
    g = _gauge(ONE_END_MM)
    assert t.profile_fit(g.joints)["slope_deg"] > t.SQUARE_DEG
    assert _score(g) == 0.0


@pytest.mark.parametrize("deg, ok", [(2.0, True), (4.0, False), (8.0, False)])
def test_tilted_press(deg, ok):
    """An ideal copy pressed with the frame tilted along its length: up to 3 deg is square enough."""
    base = np.array(TWO_HAND_MM)
    mm = base + np.tan(math.radians(deg)) * np.array(t.FINGER_U) * 1000.0
    mm = mm - mm.min() + base.min()
    assert bool(t.copied(_gauge(list(mm)))) == ok


def test_off_centre_press_fails():
    """Pressed with the casing under the gauge's end: too few fingers read it."""
    mm = [12.5] * t.N_FINGERS
    mm[-1], mm[-2] = 28.0, 26.0
    assert _score(_gauge(mm)) == 0.0


def test_flattened_on_plank_fails():
    """Pressed down onto the plank (or the bench) after the copy: every finger pushed level."""
    assert _score(_gauge([24.0] * t.N_FINGERS)) == 0.0


def test_dropped_or_tilted_gauge_not_placed():
    assert _score(_gauge(TWO_HAND_MM, tilt=math.radians(8))) == pytest.approx(t.PARTIAL)
    assert _score(_gauge(TWO_HAND_MM, x=t.CUT_X + 0.02)) == pytest.approx(t.PARTIAL)
    assert _score(_gauge(TWO_HAND_MM, yaw=t.TURN + math.radians(10))) == pytest.approx(t.PARTIAL)


def test_demo_reads_no_state():
    plan = t.demo()
    assert set(plan) == {"left", "right"}
    for wps in plan.values():
        assert sum(w.duration for w in wps[-1:]) >= 8.0  # parked >= 8 s at the end
    assert sum(w.duration for w in plan["left"]) == pytest.approx(sum(w.duration for w in plan["right"]))


# a valid copy (demo pushes plus up to 0.6 mm noise): the fit screens every gate first, then takes the lowest RMS
NOISY_MM = [14.45, 14.777, 14.531, 13.87, 13.86, 14.448, 13.406, 13.985, 13.956, 13.462, 13.264, 13.234, 25.306, 22.934, 19.405, 19.964, 24.395, 30.851, 30.547, 13.987, 13.058, 12.992, 13.135, 12.453, 13.043, 13.018, 12.959, 13.501, 13.055, 12.917, 12.796]


def test_fit_screens_every_gate_before_lowest_rms():
    f = t.profile_fit(_q(NOISY_MM))
    assert f["ref_spread_mm"] <= t.REF_SPREAD_MM and f["max_mm"] <= t.MAX_MM and f["rms_mm"] <= t.RMS_MM
    assert _score(_gauge(NOISY_MM)) == 1.0


def test_count_gate_is_a_guard_only():
    """7 or 8 fingers read the casing at every candidate offset (the casing fully under the gauge): MIN_ON_CASING
    never narrows where along the gauge the casing may be pressed."""
    assert int(t._N_ON.min()) >= 7 >= t.MIN_ON_CASING


def _reading(c, h):
    return max(max(h(c + d) - r, 0.0) for d, r in zip(t._FOOT, t._ROUND))


def _ramp(c):
    x = c + t.CAS_W / 2
    return 0.006 + 0.0115 * x / t.CAS_W if 0.0 <= x <= t.CAS_W else 0.0


@pytest.mark.parametrize("off_mm", [-3.0, -1.5, 1.5, 3.0])
def test_ramp_stand_in_fails(off_mm):
    """A plain bevel of the same rise (6 to 17.5 mm) pressed perfectly: not the casing's profile, fails the gate."""
    q = {n: 0.013 + _reading(u - off_mm / 1000, _ramp) for n, u in zip(t.FINGERS, t.FINGER_U)}
    assert not t.copied(_gauge([v * 1000 for v in q.values()]))


def test_shortcut_flat_press_fails():
    """A flat block of the casing's width pressed perfectly (all fingers on it level): errs by > 3 mm."""
    flat = lambda c: 0.012 if abs(c) <= t.CAS_W / 2 else 0.0  # noqa: E731
    q = [13.0 + 1000 * _reading(u, flat) for u in t.FINGER_U]
    assert t.profile_fit(_q(q))["max_mm"] > t.MAX_MM and _score(_gauge(q)) == 0.0
