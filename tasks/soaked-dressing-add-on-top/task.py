"""A first-aid station with a training arm. The patient's forearm (skin-colored, about 7 cm thick and 32 cm long,
tapering to the wrist) lies on a dark padded armrest, pointing at the robot. On the forearm, over a cut, is a gauze
pad soaked through with blood: a thin white pad, 5 x 5 cm, with a dark red stain across its top. On the robot's right is
a tray with a fresh, folded gauze pad (white, 4 x 4 x 2.5 cm).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below and adds the grippers to `objects`)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "The dressing on the training arm has soaked through. Put on a fresh pad and hold it in place."

REST_X, REST_L, REST_W, REST_H = 0.29, 0.36, 0.12, 0.01
ARM_X, ARM_L, ARM_R = 0.29, 0.32, 0.035
ARM_Z = TOP + REST_H + ARM_R
ARM_TOP = ARM_Z + ARM_R
WOUND = (0.30, 0.0)
SOAKED = (0.05, 0.05, 0.01)
SOAKED_START = (*WOUND, ARM_TOP + SOAKED[2] / 2)
SOAKED_TOP = ARM_TOP + SOAKED[2]
FRESH = (0.04, 0.04, 0.025)
TRAY_X, TRAY_Y, TRAY_H = 0.32, -0.22, 0.01
FRESH_START = (TRAY_X, TRAY_Y, TOP + TRAY_H + FRESH[2] / 2)
VIOLATION_STEPS = 2
PRESS_STEPS = 40  # pressure held for 40 control steps in a row (2 s at 20 Hz)
GRIPPERS = ("left_gripper", "right_gripper")

SKIN = kit.Look((0.87, 0.68, 0.55), 0.65)
PAD = kit.Look((0.16, 0.18, 0.24), 0.9)
GAUZE = kit.Look((0.97, 0.97, 0.95), 0.95)
STAIN = kit.Look((0.50, 0.03, 0.05), 0.6)
STAIN_EDGE = kit.Look((0.72, 0.22, 0.22), 0.7)
STAIN_DARK = kit.Look((0.32, 0.01, 0.03), 0.5)
TRAY_LOOK = kit.Look((0.62, 0.76, 0.90), 0.5)


def _forearm():
    profile = [(0.0, -0.165), (0.022, -0.163), (0.027, -0.155), (0.030, -0.10), (0.034, -0.02), (0.036, 0.06),
               (0.037, 0.15), (0.033, 0.16), (0.0, 0.162)]
    return kit.Prop("forearm", pos=(ARM_X, 0.0, ARM_Z), size=(ARM_L, 2 * ARM_R, 2 * ARM_R),
                    meshes=(kit.lathe(profile, SKIN, axis="x", n=48),),
                    colliders=(kit.Collider((0, 0, 0), size=(ARM_L, 2 * ARM_R, 2 * ARM_R)),))


def _soaked():
    """A thin gauze pad, its top soaked through: overlapping blotches of blood, darker where they overlap, laid just
    on top of the gauze."""
    h = SOAKED[2] / 2
    blotches = [  # (x, y, radius, look), drawn in order, each a hair above the last
        (0.002, 0.001, 0.021, STAIN_EDGE), (-0.010, 0.012, 0.010, STAIN_EDGE), (0.013, -0.011, 0.011, STAIN_EDGE),
        (0.000, 0.000, 0.016, STAIN), (-0.008, 0.009, 0.008, STAIN), (0.011, -0.008, 0.008, STAIN),
        (0.006, 0.010, 0.006, STAIN), (-0.002, -0.003, 0.009, STAIN_DARK)]
    meshes = [kit.box((0, 0, 0), SOAKED, GAUZE)]
    for i, (x, y, r, look) in enumerate(blotches):
        z = h + 0.0003 + 0.0001 * i
        meshes.append(kit.lathe([(0.0, z), (r, z)], look, center=(x, y, 0), n=20))
    return kit.Prop("soaked_pad", pos=SOAKED_START, size=SOAKED, mass=0.02, meshes=tuple(meshes),
                    colliders=(kit.Collider((0, 0, 0), size=SOAKED),))


def _fresh():
    """A folded gauze pad: the folds show as a slightly smaller layer on top."""
    return kit.Prop("fresh_pad", pos=FRESH_START, size=FRESH, mass=0.04,
                    meshes=(kit.box((0, 0, -0.003), (FRESH[0], FRESH[1], FRESH[2] - 0.006), GAUZE),
                            kit.box((0, 0, FRESH[2] / 2 - 0.003), (FRESH[0] - 0.004, FRESH[1] - 0.002, 0.006), GAUZE)),
                    colliders=(kit.Collider((0, 0, 0), size=FRESH),))


OBJECTS = [
    kit.Prop("armrest", pos=(REST_X, 0.0, TOP + REST_H / 2), size=(REST_L, REST_W, REST_H),
             meshes=(kit.box((0, 0, 0), (REST_L, REST_W, REST_H), PAD),),
             colliders=(kit.Collider((0, 0, 0), size=(REST_L, REST_W, REST_H)),)),
    _forearm(),
    kit.Prop("tray", pos=(TRAY_X, TRAY_Y, TOP + TRAY_H / 2), size=(0.12, 0.10, TRAY_H),
             meshes=(kit.box((0, 0, 0), (0.12, 0.10, TRAY_H), TRAY_LOOK),),
             colliders=(kit.Collider((0, 0, 0), size=(0.12, 0.10, TRAY_H)),)),
    _soaked(),
    _fresh(),
]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


# --- What happened during the run. success() and score() are called after every control step, with no
# reset hook, so the run's history is kept here. A reset puts every free object back at its start, at rest, in one
# step; that jump is how a new trial is recognized. Objects moved by the arms travel a few cm per step at most.
class _History:
    def __init__(self):
        self.last = None
        self.run = 0
        self.soaked_disturbed = False  # never forgiven
        self.press_run = 0
        self.pressed = False  # the fresh pad held in place long enough, on top of the soaked one


_H = _History()


def _disturbed(s):
    """The soaked pad moved more than 1.5 cm across the arm or lifted more than 1 cm off it."""
    return bool(np.hypot(s.pos[0] - SOAKED_START[0], s.pos[1] - SOAKED_START[1]) > 0.015
                or s.pos[2] > SOAKED_START[2] + 0.01)


def _fresh_on_top(f, s):
    """The fresh pad on the soaked pad: centers within 2 cm, the fresh pad lying on the soaked pad's top (1 cm)."""
    return bool(np.hypot(f.pos[0] - s.pos[0], f.pos[1] - s.pos[1]) < 0.02
                and abs(f.pos[2] - (s.pos[2] + SOAKED[2] / 2 + FRESH[2] / 2)) < 0.01)


def _pressing(g, f):
    """A gripper holding the fresh pad in place, pressing on it or gripping it (the caller checks the fingers are
    closed: open less than a quarter of full travel): its tool point within 2 cm of the pad's center across the arm,
    above the pad's center, and its finger tips within 5 mm of the pad's top or lower."""
    return bool(np.hypot(g.pos[0] - f.pos[0], g.pos[1] - f.pos[1]) < 0.02 and g.low[2] < f.pos[2] + FRESH[2] / 2 + 0.005
                and g.pos[2] > f.pos[2])


def _update(objects):
    global _H
    now = {n: np.asarray(objects[n].pos, dtype=float) for n in _START}
    if _H.last is not None:
        jumped = any(np.linalg.norm(now[n] - _H.last[n]) > 0.06 for n in now)
        at_start = all(np.linalg.norm(now[n] - _START[n]) < 0.015 for n in now)
        still = all(objects[n].resting() for n in now)
        if jumped and at_start and still:
            _H = _History()
    _H.last = now
    s, f = objects["soaked_pad"], objects["fresh_pad"]
    _H.run = _H.run + 1 if _disturbed(s) else 0
    if _H.run >= VIOLATION_STEPS:
        _H.soaked_disturbed = True
    pressing = _fresh_on_top(f, s) and any(
        k in objects and kit.opening(objects, k) < 0.25 and _pressing(objects[k], f) for k in GRIPPERS)
    _H.press_run = _H.press_run + 1 if pressing else 0
    if _H.press_run >= PRESS_STEPS:
        _H.pressed = True


def _score(objects):
    s, f = objects["soaked_pad"], objects["fresh_pad"]
    if _H.soaked_disturbed or not (_fresh_on_top(f, s) and f.resting()):
        return 0.0
    return 1.0 if _H.pressed else 0.5


def success(objects):
    """1.0: the soaked pad never moved, the fresh pad rests on top of it, and it was held in place for 2 s.
    Called first each step, so it also records the step's history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the soaked pad was ever lifted or moved off the wound, even if it was put back. Otherwise 0.5 with the
    fresh pad resting on top of the soaked pad, and 1.0 if a closed gripper also held the fresh pad in place (on top
    of the soaked one) for 2 s, pressing on it or gripping it. 0 if the fresh pad is anywhere else at the end.
    Possible scores: 0, 0.5, 1."""
    return _score(objects)


def demo():
    """Right arm: set the fresh pad on top of the soaked one without touching it, close the fingers and press down
    on the new pad for 3 s, then let go."""

    def grip(top, below=0.012):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.22
    lag = 0.012  # the arm falls ~1.5 cm short of a target near the middle of the bench, toward its own base
    site = (WOUND[0], WOUND[1] + lag)
    stack_top = SOAKED_TOP + FRESH[2]
    pad = pick_place_waypoints((TRAY_X, TRAY_Y, grip(FRESH_START[2] + FRESH[2] / 2)),
                               (*site, grip(stack_top) + 0.004), travel_z=travel)
    press_z = stack_top + 0.013 - 0.003  # closed finger tips 3 mm into the pad's top: firm pressure
    o, c = FINGER_OPEN, FINGER_CLOSED
    press = [
        Waypoint(0.8, (*site, stack_top + 0.05), c, "close the fingers above the pad"),
        Waypoint(1.2, (*site, press_z), c, "press down"),
        Waypoint(3.0, (*site, press_z), c, "hold pressure"),
        Waypoint(1.0, (*site, stack_top + 0.06), c, "lift off"),
    ]
    park = (0.16, -0.26, TOP + 0.22)
    right = pad + press + [Waypoint(1.2, park, o, "park"), Waypoint(2.0, park, o, "hold")]
    return {"right": right}
