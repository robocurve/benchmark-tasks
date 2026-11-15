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
from chairkit import PartState  # noqa: E402

T = importlib.import_module("task")

START = np.array(T.ARMREST.pos, float)
SOCKET_TOP_Z = T.TOP + T.SOCKET_TOP


def _state(pos, tilt=0.0, vz=0.0, lock=0.0, spin=0.0):
    """The armrest at ``pos`` (its origin, midway between the post feet), tipped ``tilt`` about y (top toward +x)."""
    quat = np.array([0.0, math.sin(tilt / 2), 0.0, math.cos(tilt / 2)])
    p = np.asarray(pos, float)
    return PartState(pos=p, quat_xyzw=quat, lin_vel=np.array([0.0, 0.0, vz]), ang_vel=np.array([0.0, spin, 0.0]), low=p, high=p,
                     sliders={"lock": lock})


def _objects(**kw):
    return {"wheelchair": None, "armrest": _state(**kw)}


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        out = (T.score(objs), T.success(objs))
    return out


LIFTED = START + (0, 0, 0.125)                         # posts out of their sockets
CARRIED = LIFTED + (-0.12, 0, 0)                       # and carried 12 cm toward the arms
LYING = np.array([0.10, 0.0, T.TOP + T.PAD_X / 2])     # on its side, pad toward the chair, posts toward the arms
UP = math.radians(90)


def test_start_scores_zero():
    assert _run([_objects(pos=START)]) == (0.0, False)
    assert _run([_objects(pos=START + (0, 0, 0.0019), lock=0.007)])[0] == 0.0   # catch against the window's edge


def test_lifted_two_cm():
    assert _run([_objects(pos=START + (0, 0, 0.021))])[0] == 0.25


def test_posts_out():
    assert _run([_objects(pos=LIFTED)])[0] == 0.5
    assert _run([_objects(pos=START + (0, 0, T.DEPTH + 0.004))])[0] == 0.25   # feet still 1 mm short of 5 mm clear


def test_clear_of_the_chair():
    assert _run([_objects(pos=LIFTED), _objects(pos=CARRIED)])[0] == 0.75
    assert _run([_objects(pos=LIFTED), _objects(pos=LIFTED + (-0.05, 0, 0))])[0] == 0.5   # centre not 10 cm out


def test_lying_clear_succeeds_once_still():
    frames = [_objects(pos=LIFTED), _objects(pos=CARRIED)] + [_objects(pos=LYING, tilt=UP)] * 21
    assert _run(frames) == (1.0, True)
    frames = [_objects(pos=LIFTED), _objects(pos=CARRIED), _objects(pos=LYING, tilt=UP, vz=-0.02)]
    assert _run(frames) == (1.0, False)          # full score, but still settling: not at rest
    frames = [_objects(pos=LIFTED), _objects(pos=CARRIED), _objects(pos=LYING, tilt=UP, spin=0.1)]
    assert _run(frames) == (1.0, False)


def test_lying_either_way_counts():
    frames = [_objects(pos=LYING + (0.30, 0, 0), tilt=-UP)] * 21     # on its other side, posts toward the chair
    assert _run(frames) == (1.0, True)


def test_standing_or_held_up_is_not_lying():
    standing = CARRIED - (0, 0, CARRIED[2] - T.TOP)
    assert _run([_objects(pos=LIFTED)] + [_objects(pos=standing)] * 21)[0] == 0.75
    leaning = np.array([0.16, 0.0, T.TOP + 0.011 * math.sin(math.radians(65))])   # held tipped 65 degrees on its feet
    assert _run([_objects(pos=LIFTED)] + [_objects(pos=leaning, tilt=math.radians(65))] * 21)[0] == 0.75
    held = LYING + (0, 0, 0.005)                 # on its side but 5 mm up, in the grippers
    assert _run([_objects(pos=LIFTED)] + [_objects(pos=held, tilt=UP)] * 21)[0] == 0.75


def test_lying_over_the_chair_is_not_clear():
    over = np.array([T.SOCKETS_X - 0.05, 0.0, T.TOP + 0.20])
    assert _run([_objects(pos=over, tilt=UP)] * 21)[0] == 0.5


def test_dropped_is_zero_for_good():
    frames = [_objects(pos=LIFTED), _objects(pos=CARRIED, vz=-0.9)] + [_objects(pos=LYING, tilt=UP)] * 21
    assert _run(frames) == (0.0, False)


def test_toppling_from_its_feet_is_not_a_drop():
    # Tipping over from standing on its feet: falling fast, but its lowest point is on the table.
    standing = CARRIED - (0, 0, CARRIED[2] - T.TOP)
    frames = [_objects(pos=LIFTED), _objects(pos=standing, tilt=math.radians(-60), vz=-1.5)]
    frames += [_objects(pos=LYING - (0.08, 0, 0), tilt=-UP)] * 21
    assert _run(frames) == (1.0, True)


def test_slow_lowering_is_not_a_drop():
    frames = [_objects(pos=LIFTED), _objects(pos=CARRIED, vz=-0.5)] + [_objects(pos=LYING, tilt=UP)] * 21
    assert _run(frames) == (1.0, True)


# -- geometry ---------------------------------------------------------------------------------------------------------

def _world_boxes(p, lock=0.0):
    base = np.array(p.pos)
    out = [(p.name, base + np.array(b[0]), np.array(b[1])) for b in p.boxes if len(b) < 4 or not b[3]]
    for sl in p.sliders:
        out += [(f"{p.name}:{sl.name}", base + np.array(b[0]) + (0, lock, 0), np.array(b[1])) for b in sl.boxes]
    return out


def _overlaps(boxes):
    hits = set()
    for a, b in itertools.combinations(boxes, 2):
        if a[0].split(":")[0] == b[0].split(":")[0]:
            continue
        depth = np.minimum(a[1] + a[2] / 2, b[1] + b[2] / 2) - np.maximum(a[1] - a[2] / 2, b[1] - b[2] / 2)
        if (depth > 1e-5).all():
            hits.add(tuple(sorted((a[0], b[0]))))
    return hits


def test_nothing_overlaps_at_the_start():
    assert _overlaps([b for p in T.OBJECTS for b in _world_boxes(p)]) == set()


def test_posts_clear_their_sockets():
    inside = T.SOCKET_IN / 2 - T.POST / 2
    assert abs(inside - 0.0015) < 1e-9               # 1.5 mm each side
    assert math.degrees(2 * inside / T.DEPTH) < 2.0  # the posts bind if the armrest tips 1.7 degrees in its sockets


def test_lock_holds_and_releases():
    catch = T._lock().boxes[1]
    front = catch[0][1] - catch[1][1] / 2            # the catch's front face, armrest frame
    wall_in = T.POST_Y[0] - T.SOCKET_IN / 2          # the socket's inner front face
    assert front < wall_in - 0.002                   # at rest it stands into the window
    assert front + T.LOCK_TRAVEL > wall_in + 0.002   # pushed back 7 mm, it is 2 mm inside the wall
    top = T.FLOOR + T.CATCH_Z[1]
    assert 0.0 < T.WINDOW_Z[1] - top <= 0.0025       # lifting meets the window's top edge within 2.5 mm
    assert T.STARTED > T.WINDOW_Z[1] - top           # so 0.25 needs the lock released
    # nothing overlaps with the lock pushed back either
    assert _overlaps([b for p in T.OBJECTS for b in _world_boxes(p, lock=T.LOCK_TRAVEL)]) == set()


def test_finger_pads_clear_the_posts_and_socket():
    # Level grips (fingers closing in z, pads 27 mm wide in y and flat 24 mm back from their tips, 6.4 mm thick).
    # Tab: tips 4 mm short of the post; pad: tips 15.5 mm onto the pad, short of the posts.
    tab_tip_x = -0.015
    assert tab_tip_x < -T.POST / 2 - 0.003
    assert tab_tip_x - 0.024 >= T.TAB[0] - 0.001     # the pads' flat lies on the tab
    pad_tip_x = -0.012
    assert pad_tip_x < -T.POST / 2
    # the lower finger under the tab stays above the socket's mouth even with the armrest seated
    assert T.FLOOR + T.TAB[4] - 0.0064 > T.SOCKET_TOP + 0.05


def test_lying_spot_is_clear_of_the_chair():
    rot = np.array([[0.0, 0, 1], [0, 1, 0], [-1, 0, 0]])   # tipped 90 degrees about y, top toward +x
    far = max((LYING + rot @ c)[0] for c in T._CORNERS)
    assert far < T.SOCKETS_X - T.SOCKET_IN / 2 - T.WALL - 0.01
    assert (LYING + rot @ (0, 0, T.CENTRE_Z))[0] <= T.CLEAR_X
