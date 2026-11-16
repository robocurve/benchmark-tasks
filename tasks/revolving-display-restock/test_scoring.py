"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states, and the
starting layout's geometry.

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from spinkit import PartState  # noqa: E402

T = importlib.import_module("task")


def _quat(yaw=0.0, tilt=0.0):
    """Yaw about z, then tilt about x."""
    cy, sy, cx, sx = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
    w, x, y, z = cy * cx, cy * sx, sy * sx, sy * cx
    return np.array([x, y, z, w])


def _state(pos, yaw=0.0, tilt=0.0, tray=None):
    p = np.asarray(pos, float)
    return PartState(pos=p, quat_xyzw=_quat(yaw, tilt), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p,
                     sliders={} if tray is None else {"tray": tray})


def _display(tray=0.0, base_xy=T.DISPLAY_XY, base_yaw=0.0, tilt=0.0):
    return _state((*base_xy, T.TOP), base_yaw, tilt, tray)


def _in_pocket(display, k, dx=0.0, dy=0.0, dz=0.0, tilt=0.0):
    """A box state seated in empty pocket k (offset in the turntable frame)."""
    origin, rot = T.tray_frame(display)
    px, py = T.EMPTY_POCKETS[k]
    pos = origin + rot @ np.array([px + dx, py + dy, T.DECK_TOP + dz])
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return _state(pos, yaw, tilt)


def _objects(display=None, left=None, right=None):
    d = display if display is not None else _display()
    return {"display": d,
            "charger_left": left if left is not None else _state(T.BOX_L.pos),
            "charger_right": right if right is not None else _state(T.BOX_R.pos)}


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        out = (T.score(objs), T.success(objs))
    return out


TURNED = _display(math.pi)


def test_start_scores_zero():
    assert T.facing_deg(_display()) == 180.0
    assert _run([_objects()]) == (0.0, False)


def test_turned_round_scores_quarter():
    assert _run([_objects(TURNED)]) == (0.25, False)
    assert _run([_objects(_display(math.pi - math.radians(19)))])[0] == 0.25
    assert _run([_objects(_display(math.pi - math.radians(25)))])[0] == 0.0
    assert _run([_objects(_display(-math.pi + math.radians(10)))])[0] == 0.25   # turned the other way round


def test_one_box_seated_half():
    assert _run([_objects(TURNED, left=_in_pocket(TURNED, 1))]) == (0.5, False)


def test_both_seated_success_after_one_second():
    objs = _objects(TURNED, left=_in_pocket(TURNED, 1), right=_in_pocket(TURNED, 0))
    assert _run([objs]) == (0.75, False)              # seated, not yet still for 1 s
    assert _run([objs] * 20) == (0.75, True)          # still for 1 s: success, not yet held for 1 s
    assert _run([objs] * 39) == (1.0, True)


def test_score_alone_gives_the_same():
    steps = [_objects(TURNED, left=_in_pocket(TURNED, 1), right=_in_pocket(TURNED, 0)) for _ in range(39)]
    T._reset_state()
    assert [T.score(o) for o in steps][-21:] == [0.75] * 20 + [1.0]


def test_either_box_in_either_pocket():
    objs = _objects(TURNED, left=_in_pocket(TURNED, 0), right=_in_pocket(TURNED, 1))
    assert _run([objs] * 39) == (1.0, True)


def test_both_boxes_in_one_pocket_count_once():
    objs = _objects(TURNED, left=_in_pocket(TURNED, 0), right=_in_pocket(TURNED, 0, dz=0.0))
    assert T.filled(objs) == 1


def test_seated_limits():
    d = TURNED
    assert T.seated(d, _in_pocket(d, 0, dx=0.0055), T.EMPTY_POCKETS[0])
    assert not T.seated(d, _in_pocket(d, 0, dx=0.0065), T.EMPTY_POCKETS[0])
    assert not T.seated(d, _in_pocket(d, 0, dy=-0.0065), T.EMPTY_POCKETS[0])
    assert not T.seated(d, _in_pocket(d, 0, dz=0.009), T.EMPTY_POCKETS[0])     # resting on a wall, not in
    assert not T.seated(d, _in_pocket(d, 0, tilt=math.radians(12)), T.EMPTY_POCKETS[0])


def test_moving_boxes_not_still():
    a = _objects(TURNED, left=_in_pocket(TURNED, 1), right=_in_pocket(TURNED, 0))
    b = _objects(TURNED, left=_in_pocket(TURNED, 1, dy=0.002), right=_in_pocket(TURNED, 0))
    assert _run([a, b] * 20) == (0.75, False)


def test_pockets_follow_the_base():
    """Turning or sliding the whole display (base and all) is allowed: pockets are found from its actual pose."""
    d = _display(math.pi / 2, base_xy=(0.33, 0.02), base_yaw=math.pi / 2)
    assert T.facing_deg(d) < 1e-6
    objs = _objects(d, left=_in_pocket(d, 0), right=_in_pocket(d, 1))
    assert _run([objs] * 39) == (1.0, True)


def test_tipped_display_zero_for_the_trial():
    full = _objects(TURNED, left=_in_pocket(TURNED, 1), right=_in_pocket(TURNED, 0))
    tipped = _objects(_display(math.pi, tilt=math.radians(25)))
    assert _run([tipped] + [full] * 25) == (0.0, False)
    slight = _objects(_display(math.pi, tilt=math.radians(5)))
    assert _run([slight] + [full] * 39) == (1.0, True)


# -- geometry ------------------------------------------------------------------------------------------------------

def _aabbs(boxes, offset=(0, 0, 0)):
    out = []
    for c, s, *_ in boxes:
        c, s = np.asarray(c) + offset, np.asarray(s)
        out.append((c - s / 2, c + s / 2))
    return out


def _overlap(a, b, tol=1e-6):
    return bool(np.all(a[0] < b[1] - tol) and np.all(b[0] < a[1] - tol))


def test_stocked_boxes_fit_their_pockets():
    walls = [b for b in T.TRAY_BOXES if b[2] == T.STEEL]
    stock = [b for b in T.TRAY_BOXES if b[2] == T.CARTON]
    assert len(stock) == 6
    for s in _aabbs(stock):
        assert not any(_overlap(s, w) for w in _aabbs(walls))


def test_empty_pocket_clearances():
    """A 70 x 35 mm box in an empty pocket has 5 mm to spare all round."""
    inner_t = 0.0815 - 0.0015
    inner_r = T.POCKET_OUT / 1000 - T.COL / 1000
    assert abs((inner_t - T.BOX_W) / 2 - 0.005) < 1e-9
    assert abs((inner_r - T.BOX_D) / 2 - 0.005) < 1e-9
    for px, py in T.EMPTY_POCKETS:
        box = (np.array([px - T.BOX_D / 2, py - T.BOX_W / 2, T.DECK_TOP]),
               np.array([px + T.BOX_D / 2, py + T.BOX_W / 2, T.DECK_TOP + T.BOX_H]))
        assert not any(_overlap(box, a) for a in _aabbs(T.TRAY_BOXES[1:]))   # (the deck is under it)


def test_loose_boxes_clear_of_the_display_as_it_turns():
    reach = math.sqrt(2) * (T.POST_XY + T.POST_R)          # the uprights sweep this radius
    for b in (T.BOX_L, T.BOX_R):
        d = math.hypot(b.pos[0] - T.DISPLAY_XY[0], b.pos[1] - T.DISPLAY_XY[1])
        assert d - math.hypot(T.BOX_W, T.BOX_D) / 2 > reach + 0.02


def test_back_pockets_out_of_reach_band():
    """The empty pockets start 0.456 m in front of the arms, past where the arms reach at their height."""
    assert T.DISPLAY_XY[0] + T.POCKET_MID > 0.45
