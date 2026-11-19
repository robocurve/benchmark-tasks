"""Offline checks of task.py and demo.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states,
the window's geometry (nothing overlaps at the start or upright, the latch noses meet the jamb liners short of
upright and pass once pushed in level with them, the pivot bars sit on their shoes), the latch's size against the
part it is modelled on, and the demo's plan.

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from sashkit import PartState  # noqa: E402

T = importlib.import_module("task")
PIVOT = np.array(T.PIVOT)


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _sash(tilt_deg=0.0, latches=(0.0, 0.0), offset=(0, 0, 0)):
    a = -math.radians(tilt_deg)
    quat = np.array([0.0, math.sin(a / 2), 0.0, math.cos(a / 2)])
    p = PIVOT + np.asarray(offset, float)
    return PartState(pos=p, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p,
                     sliders={"latch_l": latches[0], "latch_r": latches[1]})


def _run(frames, near=0.20):
    """Feed the frames to the scorer. ``near``: the grippers' distance from the sash (m) at every frame, or one value
    per frame (in the simulator a per-step hook measures it)."""
    T._reset_state()
    out = (0.0, False)
    for i, s in enumerate(frames):
        T._STATE["near"] = near[i] if isinstance(near, (list, tuple)) else near
        objs = {"window_frame": None, "block": None, "sash": s}
        out = (T.score(objs), T.success(objs))
    return out


def test_full_score_needs_both_grippers_clear_for_the_whole_second():
    frames = [_sash(90)] + [_sash(0.4)] * 39
    assert _run(frames, near=0.029) == (0.75, False)                  # latched and still, a gripper within 3 cm
    assert _run(frames, near=0.0) == (0.75, False)                    # still holding it
    assert _run(frames, near=None) == (0.75, False)                   # not measured: not counted as clear
    assert _run(frames, near=0.030) == (1.0, True)
    # hands leave at frame 30: the 1 s hold starts then, so 39 frames are not enough and 50 are
    late = [0.0] * 30 + [0.05] * 30
    assert _run([_sash(90)] + [_sash(0.4)] * 39, near=late) == (0.75, False)
    assert _run([_sash(90)] + [_sash(0.4)] * 49, near=late) == (1.0, True)
    # a hand coming back within the hold restarts it
    back = [0.05] * 30 + [0.01] + [0.05] * 19
    assert _run([_sash(90)] + [_sash(0.4)] * 49, near=back) == (0.75, False)


def test_distance_from_the_grippers_to_the_sash():
    up = _sash(0.0)
    face = PIVOT + np.array([-T.SASH_T / 2, 0.0, 0.0])                # on the bottom rail's interior face
    assert abs(T.hands_near([face + np.array([-0.030, 0, 0])], up) - 0.030) < 1e-9
    assert T.hands_near([face], up) < 1e-9
    top = PIVOT + np.array([0.0, 0.0, T.SASH_TOP])                    # middle of the top rail's top
    assert abs(T.hands_near([top + np.array([0, 0, 0.05])], up) - 0.05) < 1e-9
    tab = PIVOT + np.array([0.0, (T.TAB_Y[0] + T.TAB_Y[1]) / 2, T.SASH_TOP + T.TAB_HEIGHT])
    assert abs(T.hands_near([tab + np.array([0, 0, 0.02])], up) - 0.02) < 1e-9       # measured from the tab's top
    flat = _sash(90.0)                                                # lying flat: its interior face is underneath
    under = PIVOT + np.array([-0.26, 0.0, -T.SASH_T / 2 - 0.04])     # 4 cm below the top rail
    assert abs(T.hands_near([under], flat) - 0.04) < 1e-9
    assert abs(T.hands_near([under, face + np.array([-0.5, 0, 0])], flat) - 0.04) < 1e-9   # the nearest point counts


def test_start_scores_zero():
    assert _run([_sash(90.0)]) == (0.0, False)


def test_tilt_matches_published_sense():
    assert T.tilt_deg(_sash(90.0)) == np.float64(90.0) or abs(T.tilt_deg(_sash(90.0)) - 90) < 1e-9
    assert abs(T.tilt_deg(_sash(12.5)) - 12.5) < 1e-9


def test_milestones():
    assert _run([_sash(90), _sash(59)])[0] == 0.25
    assert _run([_sash(90), _sash(61)])[0] == 0.0
    assert _run([_sash(90), _sash(19)])[0] == 0.5
    assert _run([_sash(90), _sash(2.9, (-0.012, 0.012))])[0] == 0.75


def test_milestone_is_kept_after_dropping_back():
    assert _run([_sash(90), _sash(19), _sash(90)])[0] == 0.5


def test_closed_and_latched_succeeds_after_one_second():
    frames = [_sash(90)] + [_sash(0.4)] * 39          # still over 1 s (20 steps), then that held for 1 s
    assert _run(frames) == (1.0, True)
    assert _run(frames[:-1]) == (0.75, False)         # not yet held for 1 s, so no success
    assert _run([_sash(90)] + [_sash(0.4)] * 5) == (0.75, False)         # not yet still for a second
    assert _run([_sash(90)] + [_sash(-0.9)] * 39) == (1.0, True)         # against the upper sash


def test_one_latch_out_of_its_slot_is_not_latched():
    assert _run([_sash(90)] + [_sash(0.4, (-0.003, 0.0))] * 21) == (0.75, False)
    assert _run([_sash(90)] + [_sash(0.4, (0.0, 0.0015))] * 21) == (0.75, False)


def test_not_upright_is_not_closed():
    assert _run([_sash(90)] + [_sash(1.6)] * 21) == (0.75, False)


def test_off_pivots_is_sticky_zero():
    assert _run([_sash(90), _sash(30, offset=(0, 0, 0.011)), _sash(0.0)] + [_sash(0.0)] * 21) == (0.0, False)
    assert _run([_sash(90, offset=(0, 0, 0.009))] + [_sash(0.0)] * 39) == (1.0, True)


# -- geometry ---------------------------------------------------------------------------------------------------------

def _boxes_world(boxes, pos, rot, shift=(0, 0, 0)):
    out = []
    for spec in boxes:
        centre, size = np.asarray(spec[0], float) + np.asarray(shift, float), np.asarray(spec[1], float)
        yaw = spec[3] if len(spec) > 3 else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        out.append((np.asarray(pos) + rot @ centre, rot @ rz, size / 2))
    return out


def _overlap(a, b, margin=0.0) -> bool:
    """Separating-axis test for two oriented boxes (centre, rotation, half sizes); True if they interpenetrate by more
    than ``margin``."""
    (ca, ra, ha), (cb, rb, hb) = a, b
    axes = [ra[:, i] for i in range(3)] + [rb[:, i] for i in range(3)]
    axes += [np.cross(ra[:, i], rb[:, j]) for i in range(3) for j in range(3)]
    d = cb - ca
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        ra_ = sum(ha[i] * abs(ax @ ra[:, i]) for i in range(3))
        rb_ = sum(hb[i] * abs(ax @ rb[:, i]) for i in range(3))
        if abs(d @ ax) > ra_ + rb_ - margin:
            return False
    return True


def _sash_world(tilt_deg, latch_shift=0.0, extra=()):
    rot = _rot_y(-math.radians(tilt_deg))
    boxes = _boxes_world(T.SASH.boxes, PIVOT, rot)
    for sl in T.SASH.sliders:
        sign = 1 if sl.name.endswith("l") else -1
        boxes += _boxes_world(sl.boxes, PIVOT, rot, shift=(0, -sign * latch_shift, 0))
    return boxes


def _nose_points(tilt_deg, latch_shift=0.0):
    """Points filling the two wedge noses (world), 0.5 mm apart."""
    rot = _rot_y(-math.radians(tilt_deg))
    pts = []
    for sl in T.SASH.sliders:
        sign = 1 if sl.name.endswith("l") else -1
        for tri, z0, z1, _ in sl.prisms:
            a, b, c = (np.array(v) for v in tri)
            for u in np.linspace(0, 1, 25):
                for v in np.linspace(0, 1 - u, max(2, int(25 * (1 - u)))):
                    xy = a + u * (b - a) + v * (c - a)
                    for z in np.linspace(z0, z1, 5):
                        pts.append(PIVOT + rot @ np.array([xy[0], xy[1] - sign * latch_shift, z]))
    return np.array(pts)


def _frame_world():
    return _boxes_world(T.FRAME.boxes, PIVOT, np.eye(3)) + _boxes_world(T.BLOCK_PART.boxes, T.BLOCK_PART.pos, np.eye(3))


def _hits(tilt_deg, latch_shift=0.0, margin=0.0002):
    frame = _frame_world()
    n = sum(_overlap(s, f, margin) for s in _sash_world(tilt_deg, latch_shift) for f in frame)
    pts = _nose_points(tilt_deg, latch_shift)
    for c, r, h in frame:
        loc = (pts - c) @ r
        n += int(np.any(np.all(np.abs(loc) < h - margin, axis=1)))
    return n


def test_start_pose_clear():
    assert _hits(90.0) == 0


def test_upright_clear_with_noses_out_or_in():
    assert _hits(0.0) == 0                                # out: they stand in the jamb slots
    assert _hits(0.0, latch_shift=T.LATCH_TRAVEL) == 0


def test_noses_meet_the_liners_short_of_upright_and_pass_once_pushed_in():
    # with the noses out the sash meets the jamb liners about 9.5 degrees short of upright ...
    assert _hits(9.0) > 0 and _hits(10.5) == 0
    # ... pushed in level with the liners (8.5 mm) it swings all the way through
    level = T.SASH_W / 2 + T.BOLT_OUT - T.LINER_Y + 0.0005
    for t in np.arange(0.0, 91.0, 1.0):
        assert _hits(float(t), latch_shift=level) == 0, t
    # ... and on the way the ramp has pushed them in: 4 mm in, they pass 9 degrees but not 7
    assert _hits(9.0, latch_shift=0.004) == 0 and _hits(7.0, latch_shift=0.004) > 0


def test_sash_rests_on_block_and_shoes():
    # the start pose: interior face 0.3 mm above the block's top, pivot bars 0.3 mm above the shoes
    face_z = T.SASH.pos[2] - T.SASH_T / 2
    assert 0 < face_z - (T.TOP + T.BLOCK[1][2]) < 0.002
    shoe_top = PIVOT[2] - 0.006
    assert abs((T.SASH.pos[2] - 0.006) - shoe_top - 0.0003) < 1e-9


def test_latch_is_the_size_of_the_part_and_low():
    inch = 0.0254
    assert abs(T.LATCH_LEN - (3 + 5 / 32) * inch) < 1e-4 and abs(T.LATCH_WID - (1 + 5 / 16) * inch) < 1e-4
    assert abs(T.BOLT_OUT - 7 / 16 * inch) < 2e-4            # latch extension 7/16 in
    assert T.LATCH_WID <= T.SASH_T                           # the housing fits on the top rail
    assert T.TAB_HEIGHT <= 0.010                             # the finger tab is low: nothing to lift the sash by
    for sl in T.SASH.sliders:
        tab = sl.boxes[0]
        assert tab[0][2] + tab[1][2] / 2 <= T.SASH_TOP + T.TAB_HEIGHT + 1e-9


def test_nose_is_a_45_degree_wedge_that_draws_inside_the_sash():
    tip = T.SASH_W / 2 + T.BOLT_OUT
    assert tip - T.LATCH_TRAVEL < T.SASH_W / 2          # drawn in, the nose is inside the sash's side
    assert tip > T.LINER_Y                               # out, it reaches into the liner's slot
    assert abs((T.BOLT_OUT - T.NOSE_RAMP_FROM) - 2 * T.NOSE_X) < 1e-9     # 12 mm of ramp over 12 mm of thickness
    for sl in T.SASH.sliders:
        (a, b, c), z0, z1, _ = sl.prisms[0]
        assert a[0] == T.NOSE_X and b[0] == c[0] == -T.NOSE_X             # ramp on the exterior side
        assert abs(abs(c[1]) - tip) < 1e-9 and abs(a[1]) < T.SASH_W / 2   # the ramp starts inside the sash's side
    assert 2 * T.NOSE_X < 2 * T.SLOT_X                                    # the nose fits the 14 mm slot


def test_layouts():
    assert set(T.LAYOUTS) == {"A", "B"} and T.LAYOUTS[T.LOT] == (T.WINDOW_X, T.WINDOW_Y)
    assert abs(T.BLOCK[0][1] - T.WINDOW_Y) < 1e-12 and abs(T.PIVOT[1] - T.WINDOW_Y) < 1e-12


# -- the demo's plan ---------------------------------------------------------------------------------------------------

D = importlib.import_module("demo")


def test_fork_holds_the_rail_edge_then_the_face():
    piv = PIVOT
    for side in (1, -1):
        for td in (90, 75, 61):                              # leaning more than the hand: the edge lies on the lower pad
            t = math.radians(td)
            w = D.opening(t)
            tcp, rot = D.fork(piv, side, t, w)
            a, c = rot[:, 2], np.array([math.cos(D.PITCH), 0.0, math.sin(D.PITCH)])
            edge = piv + _rot_y(-t) @ np.array([-0.0175, side * D.HOLD_Y, 0.275])
            assert abs((edge - tcp) @ c + w / 2) < 1e-9      # on the lower pad's plane
            assert abs((tcp + D.TIPS * a - edge) @ a - D.BELOW_EDGE) < 1e-9
        for td in (45, 12, 0):                               # nearer upright: the pad's tip is on the interior face
            t = math.radians(td)
            w = D.opening(t)
            tcp, rot = D.fork(piv, side, t, w)
            a, c = rot[:, 2], np.array([math.cos(D.PITCH), 0.0, math.sin(D.PITCH)])
            tip = tcp + D.TIPS * a - (w / 2) * c
            local = _rot_y(-t).T @ (tip - piv)
            assert abs(local[0] + 0.0175) < 1e-9 and abs(local[2] - (0.275 - D.BELOW_EDGE)) < 1e-9
            assert abs(abs(rot[:, 0] @ c) - 1) < 1e-9        # closing axis in the vertical plane


def test_opening_keeps_the_upper_finger_off_the_upper_sash():
    assert D.opening(math.radians(90)) == 0.075 and D.opening(math.radians(10)) == 0.050
    assert all(0.050 <= D.opening(math.radians(d)) <= 0.075 for d in range(0, 91, 5))
    # upright, the upper finger's pad tip is over the lower sash's own top rail, short of the upper sash
    tcp, rot = D.fork(PIVOT, 1, 0.0, 0.050)
    c = np.array([math.cos(D.PITCH), 0.0, math.sin(D.PITCH)])
    upper_tip = tcp + D.TIPS * rot[:, 2] + 0.025 * c
    assert upper_tip[0] - PIVOT[0] < 0.0205 - 0.008          # 8 mm short of the upper sash's face


def test_way_in_joint_angles_match_the_layout():
    from yam_arena import ik, station

    for name, side in (("left", 1), ("right", -1)):
        q = np.array(D.START_QS[T.LOT][name], float)
        pos, rot = ik.tcp_pose(q, np.array(station.BASES[name], float))
        want, wrot = D.fork(PIVOT, side, T.START_TILT, D.opening(T.START_TILT), **D.WAY_IN)
        assert np.linalg.norm(pos - want) < 0.0005 and np.abs(rot - wrot).max() < 0.002


def test_push_is_sent_past_upright_and_the_lift_stops_short_of_the_liners():
    assert D.PUSH_PAST < 0 and 10.5 < D.LIFT_TO < 20


def test_task_json_limits():
    import json

    d = json.loads((pathlib.Path(__file__).resolve().parent / "task.json").read_text())
    assert len(d["title"]) <= 120 and len(d["fidelity"]) <= 500 and len(d["context"]) <= 1000 and len(d["scores"]) <= 300
    assert d["category"] == "Societal relatability" and d["instruction"] == T.INSTRUCTION
