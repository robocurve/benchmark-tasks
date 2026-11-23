"""Scoring checks without the simulator: task.py's Camera rule (Monitor) driven by hand-built phone trajectories, taps,
button forces and robot poses (yam_arena's IK and forward kinematics place the collision hulls; NumPy only).

    python tests/test_scoring.py          # or: pytest tests/
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geometry as g  # noqa: E402
import task  # noqa: E402
from yam_arena import analysis, ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, READY  # noqa: E402

L3 = np.array([g.CROSS[0], g.CROSS[1], g.LENS_H])         # table frame (z above the table)
BASES = {k: np.array(v) - np.array([0.0, 0.0, g.TOP]) for k, v in station.BASES.items()}
PINCH_Y = -g.PW / 2 + 0.0165
C0, C1 = 100.0, -14.0
CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def R_from(a, c):
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def phone_for_lens(c_deg, lens=L3):
    R = g.phone_upright(c_deg - 180.0)
    return lens - R @ g.lens_local(), R


def arm_poses(side, tcp=None, rot=None, gap=0.0074, q0=None):
    """Link poses of one arm (table frame): at READY, or with its tool point at ``tcp``/``rot``, the pads ``gap``
    apart."""
    if tcp is None:
        q = np.array(READY, float)
    else:
        q, err, rerr = ik.solve(tcp, ik.matrix_to_quat_xyzw(rot), q_init=q0, base_pos=BASES[side], iters=400)
        assert err < 0.002 and rerr < math.radians(1.0), (side, tcp, err, rerr)
    qd = dict(zip(ARM_JOINTS, q))
    qd.update(joint7=-gap / 2, joint8=-gap / 2)
    return analysis.fk_link_poses_world(qd, BASES[side])


READY_POSES = {"right": arm_poses("right", gap=0.094)}       # the right arm at the ready pose (the left holds the phone)
READY_HULLS = task.robot_hulls(READY_POSES)
READY_POINTS = task.robot_points(READY_POSES)


def pinch_rot(R):
    return R_from(-R[:, 2], R[:, 0])


def moved_hulls(hulls, T):
    """Hulls [(side, link, Hull)] moved rigidly by a 4x4."""
    out = []
    for s, n, h in hulls:
        v = h.vertices @ T[:3, :3].T + T[:3, 3]
        nw = h.planes[:, :3] @ T[:3, :3].T
        hh = analysis.Hull(vertices=v, planes=np.c_[nw, h.planes[:, 3] + nw @ T[:3, 3]])
        hh.lo, hh.hi = v.min(0), v.max(0)
        out.append((s, n, hh))
    return out


def rigid(p0, R0, p1, R1):
    """The 4x4 taking a body at (p0, R0) to (p1, R1)."""
    T = np.eye(4)
    T[:3, :3] = R1 @ R0.T
    T[:3, 3] = p1 - T[:3, :3] @ p0
    return T


class Hand:
    """The left arm pinching the phone across its thickness on the top edge, placed by IK at every pose (warm
    started), so the whole arm's hulls and points follow."""

    def __init__(self, y_pinch=PINCH_Y, depth=0.003):
        self.y, self.depth, self.q = y_pinch, depth, None

    def poses(self, p, R):
        tcp = p + R @ np.array([0.0, self.y, g.PH / 2 - self.depth])
        q, err, rerr = ik.solve(tcp, ik.matrix_to_quat_xyzw(pinch_rot(R)), q_init=self.q, base_pos=BASES["left"],
                                iters=200 if self.q is not None else 400)
        assert err < 0.002 and rerr < math.radians(1.0), (tcp, err, rerr)
        self.q = q
        qd = dict(zip(ARM_JOINTS, q))
        qd.update(joint7=-g.PT / 2, joint8=-g.PT / 2)
        return {"left": analysis.fk_link_poses_world(qd, BASES["left"])}

    def at(self, p, R):
        P = self.poses(p, R)
        return task.robot_hulls(P), task.robot_points(P)




def tap_points(p, R, off=(0.0, 0.0)):
    """A right fingertip's end face flat on the glass at the shutter (+ an offset in the glass's y, z): 8 points."""
    c = g.shutter_local() + np.array([0.0, off[0], off[1]])
    loc = np.array([[g.PT / 2 + 0.0002, c[1] + dy, c[2] + dz] for dy in (-0.006, 0.006) for dz in (-0.003, 0.003)])
    return {("right", "tip_left"): p + loc @ R.T}


class Trial:
    """Feeds the monitor step by step."""

    def __init__(self):
        self.M = task.Monitor()

    def step(self, p, R, hulls=None, points=None, ends=None, forces=None, vel=(0, 0, 0), angvel=(0, 0, 0)):
        self.M.update(p, R, np.asarray(vel, float), np.asarray(angvel, float), READY_HULLS if hulls is None else hulls,
                      READY_POINTS if points is None else points, ends or {}, forces or {})

    def tap(self, p, R, hulls=None, steps=4, off=(0.0, 0.0), points=None):
        for _ in range(steps):
            self.step(p, R, hulls, points, ends=tap_points(p, R, off))
        self.step(p, R, hulls, points)

    def sweep(self, c_from, c_to, seconds, lens_fn=None, hand=None, ramp=True, pitch=0.0, points_fn=None):
        n = max(1, round(seconds * 20))
        last = None
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s) if ramp else s
            c = c_from + s * (c_to - c_from)
            p, R = pose_at(c, lens_fn, pitch)
            hh, pts = (None, None) if hand is None else hand_hulls(hand, p, R)
            if points_fn is not None:
                pts = points_fn(c, p, R)
            self.step(p, R, hh, pts)
            last = (p, R)
        return last

    def set_down(self, seconds=1.5):
        """The phone put in the stand by the left hand (one step held there), then let go and left."""
        p, R = g.phone_in_stand()
        self.step(p, R, STAND_HAND[0], STAND_HAND[1])
        for _ in range(round(seconds * 20)):
            self.step(p, R)

    @property
    def score(self):
        return self.M.tier


def hand_hulls(hand, p, R):
    """(hulls, points) of the right arm at the ready pose and the left arm holding the phone."""
    hs, pts = hand.at(p, R)
    return READY_HULLS + hs, np.vstack([READY_POINTS, pts])


STAND_HAND = hand_hulls(Hand(), *g.phone_in_stand())


def pose_at(c, lens_fn=None, pitch=0.0):
    lens = L3 if lens_fn is None else lens_fn(c)
    R = g.phone_upright(c - 180.0)
    if pitch:
        R = R @ g.rot_y(math.radians(pitch))                 # tip the camera's axis up by ``pitch`` degrees
    return lens - R @ g.lens_local(), R


def full_trial(lens_fn=None, c0=C0, c1=C1, seconds=10.0, hand=None, ramp=True, set_down=True, stop=True,
               pitch=0.0, points_fn=None, start_points=None):
    t = Trial()
    p, R = pose_at(c0, lens_fn, pitch)
    hh, pts = (None, None) if hand is None else hand_hulls(hand, p, R)
    for _ in range(5):
        t.step(p, R, hh, pts)
    t.tap(p, R, hh, points=pts if start_points is None else start_points)
    assert t.M.capture is not None, t.M.events
    p, R = t.sweep(c0, c1, seconds, lens_fn, hand, ramp, pitch, points_fn)
    if stop:
        hh, pts = (None, None) if hand is None else hand_hulls(hand, p, R)
        t.tap(p, R, hh, points=pts)
    if set_down:
        t.set_down()
    return t


def j6_only(h_lateral, axis_xy):
    """A wrist-only turn: the lens circles a vertical axis at ``axis_xy`` at ``h_lateral`` to the camera's right."""
    def lens(c):
        b = math.radians(c)
        f = np.array([math.cos(b), math.sin(b)])
        right = np.array([math.sin(b), -math.cos(b)])
        xy = np.asarray(axis_xy) + right * h_lateral
        return np.array([xy[0], xy[1], g.LENS_H])
    return lens


def best_axis(h_lateral, c0=C0, c1=C1):
    """The wrist's axis that minimises the worst sideways error over the three boards for a wrist-only turn from c0
    to c1 (board 1 judged from the first frame, board 2 at its crossing, board 3 from the last frame)."""
    V = np.array([[-math.sin(math.radians(b)), math.cos(math.radians(b))] for b in g.BEARINGS])
    xs = np.linspace(-0.12, 0.12, 481)
    X, Y = np.meshgrid(xs, xs)
    W = np.stack([X.ravel(), Y.ravel()], 1)

    def right(c):
        return np.array([math.sin(math.radians(c)), -math.cos(math.radians(c))])
    E = np.stack([np.abs((W + right(c) * h_lateral) @ V[i]) for i, c in enumerate((c0, g.BEARINGS[1], c1))], 1)
    k = int(E.max(1).argmin())
    return np.asarray(g.CROSS) + W[k], float(E.max(1)[k])


# -- the cases -------------------------------------------------------------------------------------------------------
@case
def test_lens_pivot_full_score():
    t = full_trial()
    r = t.M.saved[-1]
    assert t.score == 1.0 and r["perfect"] and all(r["full"]) and max(r["err_mm"]) < 0.1, (t.score, r["err_mm"],
                                                                                        t.M.events)


@case
def test_lens_pivot_not_put_back():
    t = full_trial(set_down=False)
    assert t.score == 0.75, (t.score, t.M.events)


@case
def test_lens_pivot_with_the_hand_on():
    """The pivot with the left hand's pinch carried along (16.5 mm from the screen-left edge): hands clear."""
    t = full_trial(hand=Hand())
    r = t.M.saved[-1]
    assert r["clear"] and not r["corner"] and t.score == 1.0, (r["clear"], r["corner"], r["seen"], t.M.events)


@case
def test_wrist_only_demo_pinch():
    """--wrist: the demo's pinch (lens 42.3 mm to the side of the wrist's axis), the lens over the cross with the
    camera at 0 degrees, the capture from 104 degrees: boards 1 and 2 not aligned."""
    h = g.PW / 2 - g.LENS_FROM_SIDE - PINCH_Y
    b = math.radians(0.0)
    axis = L3[:2] - np.array([math.sin(b), -math.cos(b)]) * h
    t = full_trial(lens_fn=j6_only(h, axis), c0=104.0)
    r = t.M.saved[-1]
    assert all(r["full"]) and r["aligned"] == [False, False, True] and t.score == 0.5, (r["err_mm"], r["full"],
                                                                                         t.M.events)


@case
def test_wrist_only_best_placements():
    """A wrist-only turn with the lens 29.6 mm or more to the side of the wrist's axis (a pinch on the top edge off
    the camera corner: 35 mm from the camera side less the lens's 12.7 mm, plus the fingertip end's 7.3 mm half-width)
    misses alignment on some board even with the axis at its best place."""
    for h in (0.0296, 0.0355, 0.0423):
        axis, worst = best_axis(h)
        assert worst > g.ALIGN_T + 0.0003, (h, worst)
        t = full_trial(lens_fn=j6_only(h, axis))
        r = t.M.saved[-1]
        assert all(r["full"]) and not all(r["aligned"]) and t.score == 0.5, (h, r["err_mm"])


def wrist_only_floor(h, starts, ends, grid=0.10, n=401):
    """The smallest worst sideways error a wrist-only turn can reach (the lens circling a vertical axis at h to the
    camera's right), over the axis's place, the start heading and the end heading, each line judged from the frame
    that puts it in the panorama (as evaluate_capture())."""
    V = np.array([[-math.sin(math.radians(b)), math.cos(math.radians(b))] for b in g.BEARINGS])
    xs = np.linspace(-grid, grid, n)
    X, Y = np.meshgrid(xs, xs)
    W = np.stack([X.ravel(), Y.ravel()], 1)

    def right(c):
        return np.array([math.sin(math.radians(c)), -math.cos(math.radians(c))])
    best = 1.0
    for cs in starts:
        for ce in ends:
            E = np.zeros(len(W))
            for b, v in zip(g.BEARINGS, V):
                c = cs if b >= cs else (ce if b <= ce else b)
                E = np.maximum(E, np.abs((W + right(c) * h) @ v))
            best = min(best, float(E.min()))
    return best


@case
def test_wrist_only_floor_any_start_and_end():
    """With any start that keeps board 1 in full (95.1 degrees or more) and any end that keeps board 3 in full (-9.1
    or less), a wrist-only turn with the lens 29.6 mm or more to the side of the axis stays 11 mm or more off some
    board's sight line (T is 6 mm)."""
    hw = g.board_half_angle()
    s0, e0 = g.BEARINGS[0] + hw - g.FRAME_W / 2, g.BEARINGS[2] - hw + g.FRAME_W / 2
    f = wrist_only_floor(0.0296, np.arange(s0, s0 + 15.0, 0.5), np.arange(e0 - 120.0, e0 + 0.01, 1.0))
    assert f > 0.011, f


@case
def test_plateau_pinch_wrist_only():
    """--plateau: a pinch over the camera plateau puts the wrist's axis 3.7 mm from the lens (along the view), so a
    wrist-only turn aligns, but the hand is on the camera corner: 0.5."""
    hand = Hand(y_pinch=g.PW / 2 - g.LENS_FROM_SIDE)
    axis, worst = best_axis(0.0)
    t = full_trial(lens_fn=j6_only(0.0, axis), hand=hand)
    r = t.M.saved[-1]
    assert all(r["aligned"]) and r["corner"] and not r["clear"] and t.score == 0.5, (r, t.M.events)


def width_grip(tcp, R):
    """An arm gripping the phone across its width from above (pads on the side edges), whichever arm and closing
    sense reaches."""
    for side in ("left", "right"):
        for sgn in (1, -1):
            rot = R_from(-R[:, 2], sgn * R[:, 1])
            q, err, rerr = ik.solve(tcp, ik.matrix_to_quat_xyzw(rot), base_pos=BASES[side], iters=400)
            if err < 0.002 and rerr < math.radians(1.0):
                return {side: arm_poses(side, tcp, rot, gap=g.PW, q0=q)}
    raise AssertionError("no arm reaches a grip across the width")


@case
def test_side_edge_grip_touches_corner():
    """Across the width at the top (fingers on the side edges 15 mm below the top): on the camera corner."""
    p, R = phone_for_lens(C0)
    tcp = p + R @ np.array([0.0, 0.0, g.PH / 2 - 0.015])
    poses = width_grip(tcp, R)
    hulls = [h for h in task.robot_hulls(poses) if h[1] in g.HAND_LINKS]
    c, s = task.corner_box()
    assert task.hulls_box_gap([h for _, _, h in hulls], p + R @ c, R, s) <= 0.001


@case
def test_off_sideways_6mm_boundary():
    """The pivot point off the cross sideways to board 2's sight line: 5.5 mm is aligned, 6.5 mm is not."""
    v2 = np.array([-math.sin(math.radians(g.BEARINGS[1])), math.cos(math.radians(g.BEARINGS[1]))])
    for d, ok in ((0.0055, True), (0.0065, False)):
        lens = np.r_[np.asarray(g.CROSS) + d * v2, g.LENS_H]
        t = full_trial(lens_fn=lambda c, lens=lens: lens)
        r = t.M.saved[-1]
        assert r["aligned"][1] == ok and t.score == (1.0 if ok else 0.5), (d, r["err_mm"], t.score)


@case
def test_height_does_not_count():
    """Only the sideways offset is checked: the lens 25 mm higher, turned about the point above the cross: 1.0."""
    lens = L3 + np.array([0.0, 0.0, 0.025])
    t = full_trial(lens_fn=lambda c: lens)
    assert t.score == 1.0, (t.M.saved[-1]["err_mm"], t.M.events)


@case
def test_too_fast():
    """--fast: 60 deg/s: "slow down"."""
    t = full_trial(seconds=(C0 - C1) / 60.0, ramp=False)
    r = t.M.saved[-1]
    assert "slow down" in r["warn"] and t.score == 0.5, (r["warn"], r["max_rate"])


@case
def test_17_deg_s_is_clean():
    t = full_trial(seconds=10.0)
    assert t.M.saved[-1]["max_rate"] < 20.0 and not t.M.saved[-1]["warn"]


@case
def test_short_two_boards():
    """--short: the stop tap with the camera at 40 degrees: boards 1 and 2 in full, 0.25."""
    t = full_trial(c1=40.0, seconds=6.0)
    r = t.M.saved[-1]
    assert r["full"] == [True, True, False] and t.score == 0.25, (r["full"], r["cover"])


@case
def test_one_board():
    t = full_trial(c1=70.0, seconds=3.0)
    assert sum(t.M.saved[-1]["full"]) == 1 and t.score == 0.0


@case
def test_start_too_late_cuts_board_1():
    """Started with the camera at 90 degrees: board 1's far edge is outside the first frame: 0.25."""
    t = full_trial(c0=90.0)
    assert t.M.saved[-1]["full"] == [False, True, True] and t.score == 0.25


@case
def test_no_stop():
    """--nostop: the capture still running at the end is lost: 0."""
    t = full_trial(stop=False, set_down=False)
    t.sweep(C1, 160.0, 6.0)                 # turned back toward the stand (the capture ends unsaved)
    t.set_down()
    assert not t.M.saved and t.score == 0.0, t.M.events


@case
def test_turned_back_unsaved():
    t = Trial()
    p, R = pose_at(C0)
    t.tap(p, R)
    t.sweep(C0, 40.0, 4.0)
    t.sweep(40.0, 50.0, 1.0)                # back 10 degrees
    p, R = pose_at(50.0)
    t.tap(p, R)                             # this tap starts a new capture
    assert not t.M.saved and t.score == 0.0, t.M.events


@case
def test_tilted_not_level():
    """The camera tipped up 5 degrees through the capture: not level: 0.5."""
    t = full_trial(pitch=5.0)
    r = t.M.saved[-1]
    assert not r["level"] and t.score == 0.5, (r["level"], r["max_tilt"])


@case
def test_rising_off_the_line():
    """The lens rising 50 mm during the sweep: "off the line"."""
    t = full_trial(lens_fn=lambda c: L3 + np.array([0, 0, 0.05 * (C0 - c) / (C0 - C1)]))
    r = t.M.saved[-1]
    assert "off the line" in r["warn"] and t.score == 0.5, (r["warn"], r["max_drift"])


@case
def test_long_touch_is_not_a_tap():
    t = Trial()
    p, R = pose_at(C0)
    t.tap(p, R, steps=16)                   # 0.8 s on the shutter
    assert t.M.capture is None and not any("-> tap" in e for _, e in t.M.events), t.M.events


@case
def test_touch_sliding_off_the_shutter():
    """A touch that starts on the shutter and slides 8 mm off it before lifting: no tap."""
    t = Trial()
    p, R = pose_at(C0)
    for k in range(4):
        t.step(p, R, ends=tap_points(p, R, (0.0, 0.0025 * k)))
    t.step(p, R)
    assert t.M.capture is None and not any("-> tap" in e for _, e in t.M.events), t.M.events


@case
def test_touch_off_the_shutter():
    t = Trial()
    p, R = pose_at(C0)
    t.tap(p, R, off=(0.0, 0.008))           # 8 mm above the shutter's centre
    assert t.M.capture is None


@case
def test_volume_presses_start_and_stop():
    """A volume press (3 N or more on the band) starts and the next one stops: saved, as taps."""
    hand = Hand()
    t = Trial()
    p, R = pose_at(C0)
    # a right fingertip on the volume-up band (-y edge, 39 mm below the top), pressing 4 N
    tcp = p + R @ np.array([0.0, -g.PW / 2 - 0.013, g.PH / 2 - 0.0393])
    poses = arm_poses("right", tcp, R_from(R[:, 1], R[:, 2]), gap=0.0)
    finger = [h for h in task.robot_hulls({"right": poses}) if h[1] in ("tip_left", "tip_right")]
    T0 = (p, R)

    def press(p, R):
        hs = hand_hulls(hand, p, R)[0] + moved_hulls(finger, rigid(T0[0], T0[1], p, R))
        for f in (4.0, 4.0, 0.0, 0.0):
            t.step(p, R, hs, forces={("right", "tip_left"): f})

    press(p, R)
    assert t.M.capture is not None, t.M.events
    p, R = t.sweep(C0, C1, 10.0, hand=hand)
    press(p, R)
    t.set_down()
    assert t.M.saved and t.score == 1.0, (t.score, t.M.events)


@case
def test_side_button_locks():
    """--button: a side-button press (the pads squeezing the side edges) locks the phone; later taps do nothing."""
    p, R = pose_at(C0)
    tcp = p + R @ np.array([0.0, 0.0, g.PH / 2 - 0.045])
    poses = width_grip(tcp, R)
    side = next(iter(poses))
    hulls = task.robot_hulls(poses)
    t = Trial()
    t.step(p, R, hulls, forces={(side, "tip_left"): 18.0, (side, "tip_right"): 18.0})
    assert t.M.locked and t.M.pressing["side"] and t.M.pressing["volume"], (t.M.events, t.M.info)
    t2 = full_trial()
    t.tap(p, R, hulls)
    t.sweep(C0, C1, 10.0)
    p, R = pose_at(C1)
    t.tap(p, R)
    t.set_down()
    assert t.M.capture is None and not t.M.saved and t.score == 0.0 and t2.score == 1.0


@case
def test_robot_in_first_frame():
    """A robot point 0.4 m away inside the first frame's left half: not hands clear: 0.5."""
    def first(c, p, R):
        return READY_POINTS
    p, R = pose_at(C0)
    f, r, u = g.camera_axes(R)
    pt = L3 + 0.4 * (math.cos(math.radians(15)) * f - math.sin(math.radians(15)) * r)
    t = full_trial(start_points=np.vstack([READY_POINTS, pt]))
    r_ = t.M.saved[-1]
    assert not r_["clear"] and t.score == 0.5, (r_["seen"], t.M.events)


@case
def test_robot_in_a_slit():
    """A robot point in the view's centre half-way through the sweep: 0.5."""
    hit = {"done": False}

    def pts(c, p, R):
        if not hit["done"] and c < 45.0:
            hit["done"] = True
            f, _, _ = g.camera_axes(R)
            return np.vstack([READY_POINTS, L3 + 0.6 * f])
        return READY_POINTS
    t = full_trial(points_fn=pts)
    assert not t.M.saved[-1]["clear"] and t.score == 0.5


@case
def test_robot_beyond_range_is_ignored():
    def pts(c, p, R):
        f, _, _ = g.camera_axes(R)
        return np.vstack([READY_POINTS, L3 + 1.7 * f])
    t = full_trial(points_fn=pts)
    assert t.score == 1.0


@case
def test_ready_pose_arms_out_of_view():
    """Both arms at the ready pose through a full sweep: none of their points in a slit, the first or last frame."""
    t = full_trial()
    assert t.M.saved[-1]["seen"] == 0


@case
def test_rod_touched():
    """A gripper touching rod 1: board 1 is not aligned: 0.5."""
    rx, ry = g.rod_xy(0)
    poses = arm_poses("left", np.array([rx - 0.012, ry, 0.20]), R_from([0, 0, -1.0], [1.0, 0, 0]), gap=0.094)
    t = Trial()
    hs = task.robot_hulls({"left": poses, "right": READY_POSES["right"]})
    gap = task.hulls_box_gap([h for _, _, h in hs], (rx, ry, g.ROD_LEN / 2), np.eye(3), (g.ROD_D, g.ROD_D, g.ROD_LEN))
    p0, R0 = g.phone_in_stand()
    t.step(p0, R0, hs)
    assert 0 in t.M.moved, (gap, t.M.events)
    t2 = full_trial()
    t.M.moved, t2.M.moved = {0}, set()
    t3 = Trial()
    t3.M.moved = {0}
    p, R = pose_at(C0)
    t3.tap(p, R)
    p, R = t3.sweep(C0, C1, 10.0)
    t3.tap(p, R)
    t3.set_down()
    r = t3.M.saved[-1]
    assert r["aligned"] == [False, True, True] and t3.score == 0.5, r["aligned"]


@case
def test_two_panoramas_best_counts():
    t = Trial()
    p, R = pose_at(C0)
    t.tap(p, R)
    p, R = t.sweep(C0, 40.0, 4.0)
    t.tap(p, R)                              # first: two boards
    p, R = t.sweep(40.0, C0, 4.0)           # back (no capture running)
    t.tap(p, R)
    p, R = t.sweep(C0, C1, 10.0)
    t.tap(p, R)
    t.set_down()
    assert len(t.M.saved) == 2 and t.score == 1.0, ([r["tier"] for r in t.M.saved], t.M.events)


@case
def test_fall_is_zero():
    """A perfect panorama, then the phone let go and dropping 60 mm with no gripper on it: 0."""
    hand = Hand()
    t = full_trial(hand=hand, set_down=False)
    p_air, R = pose_at(C1)                  # held where the sweep ended, the lens 185 mm up
    t.step(p_air, R, hand_hulls(hand, p_air, R)[0])
    for k in range(20):                     # let go: it drops 60 mm
        p_k = p_air - np.array([0, 0, min(0.06, 0.5 * 9.81 * (k * 0.05) ** 2)])
        t.step(p_k, R)
    assert t.M.fell and t.score == 0.0


@case
def test_set_down_predicate():
    p, R = g.phone_in_stand()
    assert task.phone_set_down(p, R)
    assert not task.phone_set_down(p + np.array([0, 0, 0.02]), R)          # held 20 mm up
    flat = g.rot_y(math.radians(90.0))
    assert not task.phone_set_down(np.array([0.3, 0.2, 0.004]), flat)       # lying on the mat


@case
def test_nothing():
    t = Trial()
    p, R = g.phone_in_stand()
    for _ in range(100):
        t.step(p, R)
    assert t.score == 0.0 and not t.M.fell


@case
def test_threshold_formula():
    """T = 1.5 w - 3 mm for a w mm rod (the rod 300 mm and the 5 mm line 450 mm from the cross, a 1 mm sliver)."""
    assert abs(g.align_threshold() - 0.006) < 1e-9 and abs(g.ALIGN_T - g.align_threshold()) < 1e-12
    assert abs(g.align_threshold(rod_d=0.008) - 0.009) < 1e-9


if __name__ == "__main__":
    bad = 0
    for fn in CASES:
        try:
            fn()
            print("PASS", fn.__name__, flush=True)
        except AssertionError as e:
            bad += 1
            print("FAIL", fn.__name__, str(e)[:900], flush=True)
    print(f"{len(CASES) - bad}/{len(CASES)} passed")
    sys.exit(1 if bad else 0)
