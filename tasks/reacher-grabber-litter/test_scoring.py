"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states, the gripper
touch check, the trigger-to-jaw coupling and the starting layout's geometry.

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from litterkit import PartState  # noqa: E402

T = importlib.import_module("task")

BIN_IN = np.array([T.BIN_XY[0], T.BIN_XY[1], T.TOP + 0.04])


def _state(name, pos, yaw=0.0):
    p = np.asarray(pos, float)
    quat = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    s = PartState(pos=p, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)
    half = T.LITTER_HALF.get(name, np.zeros(3))
    corners = np.array([s.to_world(np.array([a, b, c]) * half) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
    return PartState(pos=p, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=corners.min(0),
                     high=corners.max(0))


def _objects(**moved):
    out = {"picker": _state("picker", T.TOOL.pos), "bin": None}
    for obj in (T.CAN, T.BOTTLE, T.CUP):
        out[obj.name] = _state(obj.name, moved.get(obj.name, obj.pos), obj.yaw)
    if "picker" in moved:
        out["picker"] = _state("picker", moved["picker"])
    return out


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        out = (T.score(objs), T.success(objs))
    return out


def _lifted(name):
    p = np.array(dict(can=T.CAN.pos, bottle=T.BOTTLE.pos, cup=T.CUP.pos)[name], float)
    return p + (0.0, 0.0, T.LIFT_H + 0.01)


def test_start_scores_zero():
    assert _run([_objects()]) == (0.0, False)


def test_one_in_bin():
    assert _run([_objects(can=BIN_IN)])[0] == 0.3


def test_two_in_bin():
    assert _run([_objects(can=BIN_IN, bottle=BIN_IN + (0.03, 0, 0))])[0] == 0.6


def test_lifted_but_dropped():
    frames = [_objects(cup=_lifted("cup")), _objects(cup=np.array(T.CUP.pos) + (-0.1, 0, 0))]
    assert _run(frames)[0] == 0.1


def test_lifted_four_cm_is_not_lifted():
    p = np.array(T.CUP.pos) + (0, 0, T.LIFT_H - 0.01)
    assert _run([_objects(cup=p), _objects()])[0] == 0.0


def test_mixed_partial():
    frames = [_objects(cup=_lifted("cup")), _objects(can=BIN_IN, bottle=BIN_IN + (0.04, 0, 0))]
    assert _run(frames)[0] == 0.7


def test_all_in_bin_needs_to_be_still():
    objs = _objects(can=BIN_IN, bottle=BIN_IN + (0.04, 0.02, 0.03), cup=BIN_IN + (-0.07, 0, 0.02))
    assert _run([objs] * 5) == (0.9, False)              # all in, not yet still: 3 x 0.3
    assert _run([objs] * 20) == (0.9, False)             # still for 1 s, not yet held for 1 s: no success
    assert _run([objs] * 39) == (1.0, True)
    steps = [_objects(can=BIN_IN, bottle=BIN_IN + (0.04, 0.02, 0.03), cup=BIN_IN + (-0.07, 0, 0.02)) for _ in range(39)]
    T._reset_state()
    assert [T.score(o) for o in steps][-2:] == [0.9, 1.0]   # score() alone gives the same


def test_resting_on_the_rim_or_outside_is_not_in():
    on_rim = BIN_IN + (0.0, 0.0, T.BIN_H)
    beside = BIN_IN + (0.0, T.BIN_W / 2 + 0.01, 0.0)
    assert not T.in_bin(_state("can", on_rim))
    assert not T.in_bin(_state("can", beside))
    assert T.in_bin(_state("can", BIN_IN + (T.BIN_L / 2 - 0.01, 0, 0)))


def test_tool_off_table_is_zero():
    frames = [_objects(can=BIN_IN), _objects(can=BIN_IN, picker=(0.5, 0.0, T.TOP - 0.2))]
    assert _run(frames)[0] == 0.0
    # and it stays 0 once the tool is back
    frames.append(_objects(can=BIN_IN, bottle=BIN_IN, cup=BIN_IN))
    assert _run(frames)[0] == 0.0


def test_tool_lying_on_its_side_is_fine():
    assert _run([_objects(can=BIN_IN, picker=(0.3, 0.1, T.TOP + 0.015))])[0] == 0.3


def _gripper_at(tcp, yaw=0.0, finger=-0.04):
    """A gripper pointing straight down with its TCP at ``tcp``: link origin 0.134 m above it."""
    from yam_arena.demo import grip_rotation

    rot = grip_rotation(yaw)
    pos = np.asarray(tcp, float) - rot @ np.array([0.0, 0.0, 0.134])
    return pos, rot, finger


def test_gripper_touch():
    can = _state("can", T.CAN.pos)
    far = {"left": _gripper_at((0.3, 0.2, T.TOP + 0.2)), "right": _gripper_at((0.3, -0.2, T.TOP + 0.2))}
    assert not T.gripper_touches(far, can, "can")
    around = dict(far, right=_gripper_at(np.array(T.CAN.pos) + (0, 0, 0.01)))
    assert T.gripper_touches(around, can, "can")
    pushing = dict(far, left=_gripper_at(np.array(T.CAN.pos) + (-T.CAN_R - 0.012, 0, 0.01), finger=0.0))
    assert T.gripper_touches(pushing, can, "can")
    clear = dict(far, left=_gripper_at(np.array(T.CAN.pos) + (0, 0, T.CAN_H / 2 + 0.03)))
    assert not T.gripper_touches(clear, can, "can")


def test_touch_is_sticky_zero():
    T._reset_state()
    T._STATE["touched"] = True
    assert T.score(_objects(can=BIN_IN, bottle=BIN_IN, cup=BIN_IN)) == 0.0


def test_trigger_drives_jaws():
    assert T.jaw_target(0.0) == 0.0
    assert T.jaw_target(-T.TRIG_TRAVEL / 2) == T.JAW_TRAVEL / 2
    assert T.jaw_target(-T.TRIG_TRAVEL) == T.JAW_TRAVEL
    assert T.jaw_target(-0.05) == T.JAW_TRAVEL
    # jaws fully shut leave 6 mm between them; open, 13 cm: wide enough for the 9 cm cup rim
    assert abs(2 * (T.JAW_OPEN_Y - T.JAW_TRAVEL) - 0.006) < 1e-9
    assert 2 * T.JAW_OPEN_Y > 2 * 0.045 + 0.03


def test_tool_rests_on_grip_and_jaws():
    from litterkit import start_quat_xyzw

    q = start_quat_xyzw(T.TOOL)
    s = PartState(pos=np.array(T.TOOL.pos), quat_xyzw=np.array(q), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                  low=np.zeros(3), high=np.zeros(3))
    foot = s.to_world((T.GRIP_FOOT[0], 0.0, T.GRIP_FOOT[1]))
    jaw = s.to_world((T.JAW_FOOT[0], 0.0, T.JAW_FOOT[1]))
    assert abs(foot[2] - T.TOP - 0.001) < 1e-6 and abs(jaw[2] - T.TOP - 0.001) < 1e-6
    assert abs(T.TOOL_LEN - (T.JAW_FOOT[0] + 0.030)) < 1e-9   # the handle's back end is at x -0.03


def test_tool_mass_is_real():
    total = T.TOOL.mass + sum(s.mass for s in T.TOOL.sliders)
    assert abs(total - 0.22) < 1e-9


def test_litter_out_of_reach_and_apart():
    pts = {o.name: np.array(o.pos[:2]) for o in (T.CAN, T.BOTTLE, T.CUP)}
    for p in pts.values():
        assert 0.55 <= p[0] <= 0.85
    names = list(pts)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            assert np.linalg.norm(pts[a] - pts[b]) > 0.15
    # nothing starts inside the bin's footprint
    for p in pts.values():
        assert abs(p[0] - T.BIN_XY[0]) > T.BIN_L / 2 + 0.03 or abs(p[1] - T.BIN_XY[1]) > T.BIN_W / 2 + 0.03


def test_tool_starts_clear_of_litter():
    from litterkit import start_quat_xyzw

    tool = PartState(pos=np.array(T.TOOL.pos), quat_xyzw=np.array(start_quat_xyzw(T.TOOL)), lin_vel=np.zeros(3),
                     ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))
    # points over the jaws (open) and the jaw head, in the tool frame
    pts = [tool.to_world((x, y, z)) for x in np.linspace(0.46, 0.58, 7) for y in np.linspace(-0.075, 0.075, 7)
           for z in (-0.05, -0.02, 0.02)]
    for obj in (T.CAN, T.BOTTLE, T.CUP):
        piece = _state(obj.name, obj.pos, obj.yaw)
        local = np.array([piece.to_local(p) for p in pts])
        inside = np.all(np.abs(local) <= T.LITTER_HALF[obj.name] + 0.02, axis=1)
        assert not inside.any(), obj.name
