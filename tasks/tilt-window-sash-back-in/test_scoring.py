"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states, and the
window's geometry (nothing overlaps at the start or upright, a bolt that is out stops the sash short of the frame, the
pivot bars sit on their shoes).

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


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for s in frames:
        objs = {"window_frame": None, "block": None, "sash": s}
        out = (T.score(objs), T.success(objs))
    return out


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
    assert _run(frames[:-1]) == (0.75, True)          # success, but not yet held for 1 s
    assert _run([_sash(90)] + [_sash(0.4)] * 5) == (0.75, False)         # not yet still for a second
    assert _run([_sash(90)] + [_sash(-0.9)] * 39) == (1.0, True)         # against the upper sash


def test_latch_held_in_is_not_latched():
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


def _frame_world():
    return _boxes_world(T.FRAME.boxes, PIVOT, np.eye(3)) + _boxes_world(T.BLOCK_PART.boxes, T.BLOCK_PART.pos, np.eye(3))


def _hits(tilt_deg, latch_shift=0.0, margin=0.0002):
    frame = _frame_world()
    return sum(_overlap(s, f, margin) for s in _sash_world(tilt_deg, latch_shift) for f in frame)


def test_start_pose_clear():
    assert _hits(90.0) == 0


def test_upright_clear_with_bolts_out_or_in():
    assert _hits(0.0) == 0
    assert _hits(0.0, latch_shift=T.LATCH_TRAVEL) == 0


def test_bolts_out_stop_the_sash_short_of_the_frame():
    # with the bolts out the sash hits the jamb liners about 10 degrees short of upright ...
    assert _hits(9.5) > 0 and _hits(11.0) == 0
    # ... with both drawn in it swings all the way through
    for t in np.arange(0.0, 91.0, 1.0):
        assert _hits(float(t), latch_shift=T.LATCH_TRAVEL) == 0, t


def test_sash_rests_on_block_and_shoes():
    # the start pose: interior face 0.3 mm above the block's top, pivot bars 0.3 mm above the shoes
    face_z = T.SASH.pos[2] - T.SASH_T / 2
    assert 0 < face_z - (T.TOP + T.BLOCK[1][2]) < 0.002
    shoe_top = PIVOT[2] - 0.006
    assert abs((T.SASH.pos[2] - 0.006) - shoe_top - 0.0003) < 1e-9


def test_latch_bolt_geometry():
    tip = T.SASH_W / 2 + T.BOLT_OUT
    assert tip - T.LATCH_TRAVEL < T.SASH_W / 2          # drawn in, the nose is inside the sash's side
    assert tip > T.LINER_Y                               # out, it reaches into the liner's slot
