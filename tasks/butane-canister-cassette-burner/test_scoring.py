"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states, and the
starting layout's geometry.

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import itertools
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from burnerkit import PartState  # noqa: E402

T = importlib.import_module("task")

STOVE0 = np.array([*T.STOVE_XY, T.TOP])


def _quat(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.array([*(a * math.sin(angle / 2)), math.cos(angle / 2)])


def _qmul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _state(pos, quat, sliders=None):
    p = np.asarray(pos, float)
    return PartState(pos=p, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p,
                     sliders=sliders or {})


NOTCH_UP = _quat((1, 0, 0), -math.pi / 2)     # turns the lug (canister -y) to point up


def _objects(can=None, roll=0.0, lock=0.0, stove_dx=0.0, stove_tilt=0.0, can_yaw=0.0):
    """``can``: the canister's centre in the burner's frame (None: where it starts on the table). ``roll``: the
    notch's angle from straight up (degrees, about the canister's axis)."""
    stove_pos = STOVE0 + np.array([stove_dx, 0, 0])
    stove_q = _quat((1, 0, 0), math.radians(stove_tilt))
    if can is None:
        can_pos, can_q = np.array(T.CAN.pos), _quat((1, 0, 0), T.CAN_ROLL)
    else:
        can_pos = stove_pos + np.asarray(can, float)
        can_q = _qmul(_quat((0, 0, 1), math.radians(can_yaw)), _qmul(_quat((1, 0, 0), math.radians(roll)), NOTCH_UP))
    return {"burner": _state(stove_pos, stove_q, {"plug": 0.0, "lock": lock}), "canister": _state(can_pos, can_q)}


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        ok = T.success(objs)          # the runtime's order: success() then score() on the same states
        out = (T.score(objs), ok)
    return out


ON_RAILS = (-0.025, T.COMP_Y, T.AXIS_Z)
ENGAGED = (T.ENGAGED_X + 0.001, T.COMP_Y, T.AXIS_Z)
HOME = (T.LOCKED_X, T.COMP_Y, T.AXIS_Z)
LOCK = -T.LOCK_TRAVEL


def test_notch_starts_35_degrees_off():
    objs = _objects()
    assert abs(T.notch_angle(objs["canister"], objs["burner"]) - 35) < 1e-6
    lug = objs["canister"].rotation() @ np.array(T.LUG_DIR)
    assert lug[1] < 0 and lug[2] > 0                                  # up and to the right


def test_start_scores_zero():
    assert _run([_objects()]) == (0.0, False)


def test_on_the_rails():
    assert _run([_objects(can=ON_RAILS)])[0] == 0.25
    assert _run([_objects(can=(-0.025, T.COMP_Y + 0.012, T.AXIS_Z))])[0] == 0.0       # off the rails' line
    assert _run([_objects(can=ON_RAILS, can_yaw=15)])[0] == 0.0                      # across the compartment


def test_nozzle_engaged():
    assert _run([_objects(can=ON_RAILS), _objects(can=ENGAGED)])[0] == 0.5
    assert _run([_objects(can=ON_RAILS), _objects(can=ENGAGED), _objects(can=ON_RAILS)])[0] == 0.5   # sticky


def test_home_not_locked():
    assert _run([_objects(can=HOME)] * 21)[0] == 0.75
    assert _run([_objects(can=HOME), _objects(can=ON_RAILS)])[0] == 0.75


def test_home_and_locked_succeeds_after_one_second():
    assert _run([_objects(can=HOME, lock=LOCK)] * 21) == (1.0, True)
    assert _run([_objects(can=HOME, lock=LOCK)] * 5) == (1.0, False)
    assert _run([_objects(can=HOME)] * 21 + [_objects(can=HOME, lock=LOCK)] * 19) == (1.0, False)   # locked < 1 s
    assert _run([_objects(can=HOME)] * 21 + [_objects(can=HOME, lock=LOCK)] * 20) == (1.0, True)
    pressed = (T.HOME_X[1] - 0.0005, T.COMP_Y, T.AXIS_Z)                           # still held against the plunger
    assert _run([_objects(can=pressed, lock=LOCK)] * 21) == (1.0, True)


def test_lever_not_far_enough():
    assert _run([_objects(can=HOME, lock=T.LOCKED + 0.002)] * 21)[0] == 0.75


def test_notch_off_the_guide_is_not_home():
    assert _run([_objects(can=HOME, lock=LOCK, roll=20)] * 21)[0] == 0.5
    assert _run([_objects(can=HOME, lock=LOCK, roll=10)] * 21) == (1.0, True)


def test_short_of_home():
    short = (T.HOME_X[0] - 0.001, T.COMP_Y, T.AXIS_Z)
    assert _run([_objects(can=short, lock=LOCK)] * 21)[0] == 0.5


def test_burner_knocked_is_zero_for_good():
    frames = [_objects(stove_dx=0.031)] + [_objects(can=HOME, lock=LOCK)] * 21
    assert _run(frames) == (0.0, False)
    assert _run([_objects(stove_dx=0.02)] + [_objects(can=HOME, lock=LOCK, stove_dx=0.02)] * 21) == (1.0, True)
    assert _run([_objects(stove_tilt=11)] + [_objects(can=HOME, lock=LOCK)] * 21) == (0.0, False)


def test_canister_off_the_table_is_zero():
    objs = _objects()
    objs["canister"] = _state(np.array([0.3, -0.5, T.TOP - 0.3]), np.array([0, 0, 0, 1.0]))
    assert _run([objs] + [_objects(can=HOME, lock=LOCK)] * 21) == (0.0, False)


# -- geometry ---------------------------------------------------------------------------------------------------------

def _world_boxes(p):
    base = np.array(p.pos)
    out = [(p.name, base + np.array(b[0]), np.array(b[1])) for b in p.boxes if len(b) < 4 or not b[3]]
    for c in p.cylinders:
        r, h, axis = c[1], c[2], c[4] if len(c) > 4 else "Z"
        size = np.array([h if axis == "X" else 2 * r, h if axis == "Y" else 2 * r, h if axis == "Z" else 2 * r])
        out.append((p.name, base + np.array(c[0]), size))
    for sl in p.sliders:
        out += [(f"{p.name}:{sl.name}", base + np.array(b[0]), np.array(b[1])) for b in sl.boxes]
    return out


def _overlap(a, b):
    depth = np.minimum(a[1] + a[2] / 2, b[1] + b[2] / 2) - np.maximum(a[1] - a[2] / 2, b[1] - b[2] / 2)
    return bool((depth > 1e-5).all())


def test_nothing_overlaps_at_the_start():
    boxes = [b for p in T.OBJECTS for b in _world_boxes(p)]
    hits = {tuple(sorted((a[0], b[0]))) for a, b in itertools.combinations(boxes, 2)
            if a[0].split(":")[0] != b[0].split(":")[0] and _overlap(a, b)}
    assert hits == set()
    slider_hits = {(a[0], b[0]) for a in boxes for b in boxes if ":" in a[0] and b[0] == "burner" and _overlap(a, b)}
    assert slider_hits == set()


def test_socket_passes_the_cup_and_the_lug_only_with_the_notch_up():
    socket = [b for b in T._socket_boxes()]

    def blocked(points):
        for c, s, _ in socket:
            lo, hi = np.array(c) - np.array(s) / 2, np.array(c) + np.array(s) / 2
            for p in points:
                if (p[1:] > lo[1:] + 1e-6).all() and (p[1:] < hi[1:] - 1e-6).all():
                    return True
        return False

    def lug_points(roll):
        a = math.radians(roll)
        up, side = np.array([math.cos(a), -math.sin(a)]), np.array([math.sin(a), math.cos(a)])   # (z, y)
        pts = []
        for r in np.linspace(0.018, 0.028, 6):
            for t in np.linspace(-0.005, 0.005, 6):
                z, y = r * up + t * side
                pts.append(np.array([0.0, T.COMP_Y + y, T.AXIS_Z + z]))
        return pts

    cup = [np.array([0.0, T.COMP_Y + T.CUP_R * math.cos(t), T.AXIS_Z + T.CUP_R * math.sin(t)])
           for t in np.linspace(0, 2 * math.pi, 73)]
    assert not blocked(cup)
    assert not blocked(lug_points(0)) and not blocked(lug_points(10)) and not blocked(lug_points(-10))
    for roll in (16, -16, 30, 45, 90, 180):
        assert blocked(lug_points(roll)), roll


def test_lock_holds_a_homed_canister_and_is_blocked_by_one_that_is_not():
    bar = T._lock().boxes[0]
    bar_back = bar[0][0] + bar[1][0] / 2
    assert abs(bar_back - T.LOCK_BACK_X) < 1e-9
    assert abs(T.LOCKED_X + T.BASE_X - bar_back) < 1e-9          # a locked canister's base rests on the bar
    full_push = T.PLUG_REST_X + T.PLUG_TRAVEL - T.NOSE_X           # the plunger bottomed: as far as it goes
    assert abs(full_push - T.LOCKED_X - 0.0035) < 1e-9            # 3.5 mm of push past the lock position to slide it
    assert full_push + 0.075 < T.SOCKET_X                          # the shoulder never reaches the socket face
    assert T.LOCKED_X + T.NOSE_X > T.PLUG_REST_X                   # locked, the plunger is still pressed (sealed)
    assert T.HOME_X[0] <= T.LOCKED_X <= full_push <= T.HOME_X[1]
    # Locked, the bar covers the canister's base from its centre outward (toward +y)
    lo, hi = bar[0][1] - bar[1][1] / 2 - T.LOCK_TRAVEL, bar[0][1] + bar[1][1] / 2 - T.LOCK_TRAVEL
    assert lo <= T.COMP_Y and hi >= T.COMP_Y + T.CAN_R
    # Unlocked, it is clear of the canister's path (the canister reaches y COMP_Y + 33 mm)
    assert bar[0][1] - bar[1][1] / 2 > T.COMP_Y + T.CAN_R + 0.005


def test_canister_rests_on_the_rails_at_the_inlet_axis():
    rail = T._stove_boxes()[6]
    top = rail[0][2] + rail[1][2] / 2
    dy = abs(rail[0][1] - T.COMP_Y) - rail[1][1] / 2
    assert abs(T.AXIS_Z - math.sqrt(T.CAN_R ** 2 - dy ** 2) - top) < 0.0003
