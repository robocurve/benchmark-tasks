"""On the station table, a scale model of a guard fence runs across the table in front of the arms, with two
identical yellow gates in it: one in front of each arm. Each gate is hung on a dark hinge post at its outer end,
swings open toward the robot, and has a black knob post near its free end; both start shut against their latch
posts. A grey panel fills the fence between the two gates and more panel runs outward beyond each hinge post.
On the table just outside each hinge post, in front of the fence, is a white pad. A loose red tag block stands
on the table between the arms, nearer the robot.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Part frames sit on the table top.
FENCE_X = 0.33  # centre plane of the fence and of each shut gate
HINGE_Y = 0.195  # hinge post axis, +/- (left gate on +y)
LEAF = (0.010, 0.120, 0.080)  # thickness, width (along the fence when shut), height
LEAF_Z0 = 0.010  # leaf bottom above the table
LEAF_NEAR = 0.010  # leaf's hinge-side edge, from the hinge axis
LEAF_TOP = LEAF_Z0 + LEAF[2]  # 0.090
KNOB_R, KNOB_H = 0.008, 0.045  # black knob post, vertical, on the leaf's centre line
KNOB_FROM_HINGE = 0.105
KNOB_Z = (0.075, 0.120)
POST_R, POST_H = 0.012, 0.110
LATCH = (0.020, 0.020, LEAF_TOP)  # latch post just past each leaf's free end
FREE_END = LEAF_NEAR + LEAF[1]  # 0.130 from the hinge axis
PANEL_T = 0.010
PAD = (0.060, 0.060, 0.002)
PAD_XY = (0.270, 0.250)  # +/- y
TAG = (0.030, 0.030, 0.030)
TAG_MASS = 0.02
TAG_XY = (0.170, 0.000)

GATE_YELLOW, POST_DARK, PANEL_GREY = (0.95, 0.78, 0.10), (0.20, 0.21, 0.23), (0.68, 0.69, 0.70)
BLACK, WHITE, RED = (0.06, 0.06, 0.06), (0.95, 0.95, 0.95), (0.85, 0.10, 0.10)

WORKS = {"left_gate": True, "right_gate": False}  # the right gate has no closer spring
SPRING, PRELOAD = 0.10, 0.5  # N m/rad and rad: the working gate's closer, pre-loaded past shut
DAMP_WORKING, DAMP_FAULTY = 0.05, 0.30  # N m s/rad


def _gate(name: str, side: int) -> parts.Part:
    """side +1: left gate (hinge at +y, leaf toward -y); -1: right gate. Opening (positive angle) swings the free
    end toward the robot (-x)."""
    s = -side  # direction from the hinge toward the free end, in y
    works = WORKS[name]
    leaf = parts.Hinge(
        "gate",
        shapes=[
            parts.box(LEAF, at=(0.0, s * (LEAF_NEAR + LEAF[1] / 2), LEAF_Z0 + LEAF[2] / 2), color=GATE_YELLOW),
            parts.cylinder(KNOB_R, KNOB_H, at=(0.0, s * KNOB_FROM_HINGE, sum(KNOB_Z) / 2), color=BLACK, axis="z"),
        ],
        pivot=(0.0, 0.0, LEAF_Z0 + LEAF[2] / 2),
        axis=(0.0, 0.0, -float(side)),
        lower=0.0,
        upper=math.radians(92.0),
        mass=0.10,
        spring=SPRING if works else 0.0,
        damping=DAMP_WORKING if works else DAMP_FAULTY,
    )
    return parts.Part(
        name,
        pos=(FENCE_X, side * HINGE_Y, TOP),
        shapes=[
            parts.cylinder(POST_R, POST_H, at=(0.0, 0.0, POST_H / 2), color=POST_DARK, axis="z"),
            parts.box(LATCH, at=(0.0, s * (FREE_END + LATCH[1] / 2), LATCH[2] / 2), color=POST_DARK),
        ],
        joints=[leaf],
    )


LEFT_GATE, RIGHT_GATE = _gate("left_gate", +1), _gate("right_gate", -1)
GATES = {"left_gate": LEFT_GATE, "right_gate": RIGHT_GATE}
FAULTY = "right_gate"
_inner = HINGE_Y - FREE_END - LATCH[1]  # 0.045: inner face of each latch post
_outer0, _outer1 = HINGE_Y + POST_R, 0.300
CENTRE_PANEL = Box("centre_panel", pos=(FENCE_X, 0.0, TOP + LEAF_TOP / 2), size=(PANEL_T, 2 * _inner, LEAF_TOP),
                   color=PANEL_GREY)
OUTER_PANELS = [
    Box(f"{side}_outer_panel", pos=(FENCE_X, k * (_outer0 + _outer1) / 2, TOP + LEAF_TOP / 2),
        size=(PANEL_T, _outer1 - _outer0, LEAF_TOP), color=PANEL_GREY)
    for side, k in (("left", 1), ("right", -1))
]
PADS = {
    "left_gate": Box("left_pad", pos=(PAD_XY[0], PAD_XY[1], TOP + PAD[2] / 2), size=PAD, color=WHITE),
    "right_gate": Box("right_pad", pos=(PAD_XY[0], -PAD_XY[1], TOP + PAD[2] / 2), size=PAD, color=WHITE),
}
TAG_BOX = Box("red_tag", pos=(*TAG_XY, TOP + TAG[2] / 2), size=TAG, mass=TAG_MASS, color=RED)

OBJECTS = [LEFT_GATE, RIGHT_GATE, CENTRE_PANEL, *OUTER_PANELS, *PADS.values(), TAG_BOX]

# -- scoring ---------------------------------------------------------------------------------------------------

EPS = 1e-6
SUCCESS_HOLD_S = 3.0  # long enough to see the arm let go of the last gate before the trial ends
TESTED = math.radians(15.0)  # a gate counts as opened when it reached at least 15 degrees
SHUT = math.radians(5.0)  # shut: within 5 degrees of the closed stop
STILL_STEP = math.radians(0.1)  # the faulty gate "stood still": moved under 0.1 degree per control step
STILL_STEPS = 20  # ... for 20 control steps (1 s at 20 Hz) while open at least 15 degrees
PAD_TOP = TOP + PAD[2]
ON_PAD_Z = 0.001  # tag's lowest corner within 1 mm of the pad's top
FLAT = math.radians(10.0)

TOUCH_Z = 0.005  # the tag "touched a pad": its bottom within 5 mm of the pad's top, over any part of the pad

_H = {}  # trial history, cleared by a scene reset


def _clear():
    _H.clear()
    _H.update(armed=False, self_closed=False, still=0, stood_open=False, last=None, tag=None, seen=None,
              early_tag=False)


_clear()


def _tag_corners(st) -> np.ndarray:
    r = parts._quat_to_matrix(st.quat_xyzw)
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    centre = (st.low + st.high) / 2  # a Box reports its bounding-box centre: its centre
    return centre + (signs * np.asarray(TAG) / 2) @ r.T


def _tag_on_pad(objects, gate: str) -> bool:
    """Released and at rest flat on that gate's pad: every corner over the pad, lowest corner within 1 mm of its
    top, one face within 10 degrees of level."""
    st = objects["red_tag"]
    if not st.resting():
        return False
    c = _tag_corners(st)
    pad = PADS[gate]
    lo = np.asarray(pad.pos[:2]) - np.asarray(PAD[:2]) / 2 - EPS
    hi = np.asarray(pad.pos[:2]) + np.asarray(PAD[:2]) / 2 + EPS
    over = bool(np.all(c[:, :2] >= lo) and np.all(c[:, :2] <= hi))
    r = parts._quat_to_matrix(st.quat_xyzw)
    flat = float(np.max(np.abs(r[2, :]))) >= math.cos(FLAT) - EPS
    return over and flat and abs(float(c[:, 2].min()) - PAD_TOP) <= ON_PAD_Z + EPS


def _touches_a_pad(objects) -> bool:
    """The tag is down on or just above either pad: some of it over the pad, its bottom within 5 mm of the top."""
    st = objects["red_tag"]
    for pad in PADS.values():
        lo = np.asarray(pad.pos[:2]) - np.asarray(PAD[:2]) / 2
        hi = np.asarray(pad.pos[:2]) + np.asarray(PAD[:2]) / 2
        overlap = bool(np.all(st.low[:2] < hi) and np.all(st.high[:2] > lo))
        if overlap and float(st.low[2]) <= PAD_TOP + TOUCH_Z + EPS:
            return True
    return False


def _update(objects) -> None:
    """Record, once per control step (success() and score() get the same states each step):
    - the left gate tested: it went from at least 15 degrees open to within 5 degrees of shut in a step in which
      the right gate moved under 0.1 degree (taken as a self-close; a hand pushing it shut would also count);
    - the right gate stood open: at least 15 degrees open and moving under 0.1 degree per step for 20 steps (1 s);
    - the tag touched a pad before both of those had happened (this scores 0 for the rest of the trial).
    A scene reset clears the record: every gate back within 0.1 degree of shut and the tag at rest within 0.5 mm of
    its start after jumping over 20 mm in one step (from the robot's side, this could only clear credit or the
    early-tag mark together with the tests, so it gains nothing)."""
    if objects is _H["seen"]:
        return
    _H["seen"] = objects
    q = {g: float(objects[g].joints["gate"]) for g in GATES}
    tag = np.asarray(objects["red_tag"].pos, float)
    start = np.asarray(TAG_BOX.pos, float)
    if (
        _H["tag"] is not None
        and np.linalg.norm(tag - _H["tag"]) > 0.020
        and np.linalg.norm(tag - start) <= 0.0005
        and objects["red_tag"].resting()
        and all(abs(v) <= math.radians(0.1) for v in q.values())
    ):
        _clear()
    _H["tag"] = tag
    f, w = q[FAULTY], q["left_gate"]
    right_still = _H["last"] is not None and abs(f - _H["last"]) < STILL_STEP
    if w >= TESTED - EPS:
        _H["armed"] = True
    elif w <= SHUT + EPS:
        if _H["armed"] and right_still:
            _H["self_closed"] = True
        _H["armed"] = False
    if right_still and f >= TESTED - EPS:
        _H["still"] += 1
    else:
        _H["still"] = 0
    _H["last"] = f
    if _H["still"] >= STILL_STEPS:
        _H["stood_open"] = True
    if not _tested() and _touches_a_pad(objects):
        _H["early_tag"] = True


def _tested() -> bool:
    return _H["self_closed"] and _H["stood_open"]


def _shut(objects, g) -> bool:
    return float(objects[g].joints["gate"]) <= SHUT + EPS


def _grade(objects) -> float:
    if _H["early_tag"] or not (_tested() and _tag_on_pad(objects, FAULTY)):
        return 0.0
    return 1.0 if all(_shut(objects, g) for g in GATES) else 0.0


def success(objects):
    """Both gates tested, the tag on the faulty gate's pad, both gates shut."""
    _update(objects)
    return _grade(objects) == 1.0


def score(objects):
    """1.0 as success(); else 0 (pass/fail)."""
    _update(objects)
    return _grade(objects)


@parts.on_reset
def _on_reset(sim):
    """A new episode: clear the record and pre-load the working gate's closer past shut."""
    _clear()
    sim.set_target("left_gate", "gate", -PRELOAD)


# -- demo ------------------------------------------------------------------------------------------------------

TIP = 0.013  # finger tips below the TCP
GRIP_Z = TOP + LEAF_TOP + 0.012 + TIP  # tips 12 mm above the leaf's top, around the knob
HIGH = TOP + 0.18
OPEN_TO = math.radians(46.0)
FAULTY_LEFT_AT = math.radians(44.6)  # where the faulty gate stood after release (trace); regrip there
CLOSE_TO = math.radians(-8.0)  # command 8 degrees past shut: the arm gives about 7 degrees (trace)


def _knob(side: int, theta: float, z: float):
    """TCP over the knob of the gate on ``side`` opened ``theta``; the grip yaw follows the leaf."""
    x = FENCE_X - KNOB_FROM_HINGE * math.sin(theta)
    y = side * (HINGE_Y - KNOB_FROM_HINGE * math.cos(theta))
    return (x, y, z), -side * theta


def _arc(side, a, b, n, dt, finger, label):
    out = []
    for k in range(1, n + 1):
        p, yaw = _knob(side, a + (b - a) * k / n, GRIP_Z)
        out.append(Waypoint(dt, p, finger, f"{label} {k}", yaw))
    return out


def _open_and_let_go(side):
    p0, y0 = _knob(side, 0.0, HIGH)
    g0, _ = _knob(side, 0.0, GRIP_Z)
    pe, ye = _knob(side, OPEN_TO, GRIP_Z)
    return [
        Waypoint(1.5, p0, FINGER_OPEN, "over the knob", y0),
        Waypoint(1.2, g0, FINGER_OPEN, "down round the knob", y0),
        Waypoint(0.6, g0, FINGER_CLOSED, "grip the knob", y0),
        *_arc(side, 0.0, OPEN_TO, 6, 0.5, FINGER_CLOSED, "open"),
        Waypoint(0.5, pe, FINGER_CLOSED, "hold open", ye),
        Waypoint(0.6, pe, FINGER_OPEN, "let go", ye),
        Waypoint(0.8, (pe[0], pe[1], HIGH), FINGER_OPEN, "up and watch", ye),
    ]


def demo():
    """Left arm: waits 3 s, opens the left gate 46 degrees by its knob, lets go and lifts clear (it swings shut), then
    parks.
    Right arm: the same on the right gate (it stays open), waits, puts the tag on the right pad (released 7 mm up),
    then grips the knob where the gate stood and closes it to its stop."""
    left = [
        # Wait so the left gate is let go after the right one has come to rest: the left gate's self-close only
        # counts in a step in which the right gate is still.
        Waypoint(3.0, _knob(+1, 0.0, HIGH)[0], FINGER_OPEN, "wait for the right gate", 0.0),
        *_open_and_let_go(+1),
        Waypoint(1.5, (0.20, 0.30, HIGH), FINGER_OPEN, "park"),
    ]
    tag_grip = TOP + TAG[2] / 2 + TIP
    tag_place = PAD_TOP + 0.007 + TAG[2] / 2 + TIP
    tx, ty = TAG_XY
    px, py = PAD_XY[0], -PAD_XY[1]
    carry = TOP + 0.16
    r0, ry0 = _knob(-1, FAULTY_LEFT_AT, HIGH)
    rg, _ = _knob(-1, FAULTY_LEFT_AT, GRIP_Z)
    rc, ryc = _knob(-1, CLOSE_TO, GRIP_Z)
    right = [
        *_open_and_let_go(-1),
        Waypoint(2.0, (r0[0] - 0.03, r0[1], HIGH), FINGER_OPEN, "watch it stay open", ry0),
        Waypoint(1.5, (tx, ty, carry), FINGER_OPEN, "over the tag"),
        Waypoint(1.2, (tx, ty, tag_grip), FINGER_OPEN, "down to the tag"),
        Waypoint(0.8, (tx, ty, tag_grip), FINGER_CLOSED, "grip the tag"),
        Waypoint(1.0, (tx, ty, carry), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (px, py, carry), FINGER_CLOSED, "carry to the right pad"),
        Waypoint(1.2, (px, py, tag_place), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (px, py, tag_place), FINGER_OPEN, "release"),
        Waypoint(1.0, (px, py, carry), FINGER_OPEN, "up"),
        Waypoint(1.5, r0, FINGER_OPEN, "over the knob again", ry0),
        Waypoint(1.2, rg, FINGER_OPEN, "down round the knob", ry0),
        Waypoint(0.6, rg, FINGER_CLOSED, "grip the knob", ry0),
        *_arc(-1, FAULTY_LEFT_AT, CLOSE_TO, 6, 0.5, FINGER_CLOSED, "close"),
        Waypoint(0.8, rc, FINGER_CLOSED, "hold it against the stop", ryc),
        Waypoint(0.6, rc, FINGER_OPEN, "let go", ryc),
        Waypoint(0.8, (rc[0], rc[1], HIGH), FINGER_OPEN, "up"),
        Waypoint(1.5, (0.20, -0.30, HIGH), FINGER_OPEN, "park"),
    ]
    return {"left": left, "right": right}
