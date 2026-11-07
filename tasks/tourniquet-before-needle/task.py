"""A blood-draw station with a training arm. The patient's forearm (skin-colored, about 7 cm thick and 32 cm long,
tapering to the wrist) lies on a dark padded armrest, pointing at the robot: the wrist nearest the arms, the elbow
crease in the middle, the upper arm away from them. The draw is done: the needle is still in the vein at the elbow
crease, its clear plastic holder with a purple-capped blood tube lying along the arm on the wrist side. A blue latex
tourniquet is still tied over the upper arm, beyond the crease. On the robot's right are a red sharps container (a
lidded bin, 8 cm tall, with an 11 x 7 cm opening) next to the arm and, farther out, a tray with a folded white gauze
pad.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below; nothing on disk changes)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Finish the practice draw on the training arm."

# Armrest and forearm. The forearm collides as a box whose top is the top of the arm.
REST_X, REST_L, REST_W, REST_H = 0.29, 0.36, 0.12, 0.01
ARM_X, ARM_L, ARM_R = 0.29, 0.32, 0.035
ARM_Z = TOP + REST_H + ARM_R  # the forearm's axis
ARM_TOP = ARM_Z + ARM_R
SITE = (0.30, 0.0)  # the elbow crease, where the needle is in the vein

TQ = (0.03, 0.06, 0.015)  # the tourniquet's grip block on top of the arm (x by y by z)
TQ_START = (0.38, 0.0, ARM_TOP + TQ[2] / 2)
TQ_R, TQ_ENDS = 0.040, math.radians(75)  # the band's arch: radius about the arm's axis, and how far down it reaches
TQ_AXIS_Z = -(TQ[2] / 2 + ARM_R)  # the arm's axis in the tourniquet's frame
TQ_FOOT_Z = TQ_AXIS_Z + TQ_R * math.cos(TQ_ENDS)  # the band's ends, where it stands once it is off the arm
NEEDLE = (0.08, 0.025, 0.035)  # the needle holder's collider
NEEDLE_START = (0.255, 0.0, ARM_TOP + NEEDLE[2] / 2)
GAUZE = (0.04, 0.04, 0.025)
TRAY_X, TRAY_Y, TRAY_H = 0.32, -0.22, 0.01
GAUZE_START = (TRAY_X, TRAY_Y, TOP + TRAY_H + GAUZE[2] / 2)
SHARPS_X, SHARPS_Y, SHARPS_IN_X, SHARPS_IN_Y, SHARPS_H, WALL = 0.20, -0.125, 0.12, 0.10, 0.08, 0.005
OPEN_X, OPEN_Y = 0.11, 0.07  # the opening in the sharps container's lid
VIOLATION_STEPS = 2

# -- looks and models -------------------------------------------------------------------------

SKIN = kit.Look((0.87, 0.68, 0.55), 0.65)
PAD = kit.Look((0.16, 0.18, 0.24), 0.9)
LATEX = kit.Look((0.20, 0.42, 0.90), 0.45)
CLEAR = kit.Look((0.88, 0.92, 0.96), 0.15, opacity=0.45)
BLOOD = kit.Look((0.45, 0.02, 0.04), 0.3)
CAP = kit.Look((0.55, 0.20, 0.75), 0.4)
STEEL = kit.Look((0.80, 0.81, 0.84), 0.2, metallic=1.0)
GAUZE_LOOK = kit.Look((0.97, 0.97, 0.95), 0.95)
TRAY_LOOK = kit.Look((0.62, 0.76, 0.90), 0.5)
SHARPS_RED = kit.Look((0.80, 0.07, 0.07), 0.4)
LID = kit.Look((0.95, 0.95, 0.93), 0.4)
HAZARD = kit.Look((1.0, 0.82, 0.10), 0.5)


def _forearm():
    profile = [(0.0, -0.165), (0.022, -0.163), (0.027, -0.155), (0.030, -0.10), (0.034, -0.02), (0.036, 0.06),
               (0.037, 0.15), (0.033, 0.16), (0.0, 0.162)]
    return kit.Prop("forearm", pos=(ARM_X, 0.0, ARM_Z), size=(ARM_L, 2 * ARM_R, 2 * ARM_R),
                    meshes=(kit.lathe(profile, SKIN, axis="x", n=48),),
                    colliders=(kit.Collider((0, 0, 0), size=(ARM_L, 2 * ARM_R, 2 * ARM_R)),))


def _tourniquet():
    """A latex band arched over the arm with its tails tucked on top. The grip block sits on the arm's top; two small
    feet at the band's ends let it stand as an arch once it is off the arm."""
    top = TQ_AXIS_Z + TQ_R
    arch = [(0.0, TQ_R * math.sin(t), TQ_AXIS_Z + TQ_R * math.cos(t)) for t in np.linspace(-TQ_ENDS, TQ_ENDS, 24)]
    tail_a = [(0.0, -0.004, top), (0.006, 0.014, top + 0.008), (0.010, 0.026, top + 0.006)]
    tail_b = [(0.0, 0.004, top), (-0.006, -0.012, top + 0.010), (-0.010, -0.024, top + 0.007)]
    foot_y = TQ_R * math.sin(TQ_ENDS) + 0.002
    return kit.Prop("tourniquet", pos=TQ_START, size=TQ, mass=0.02,
                    meshes=(kit.band(arch, 0.025, 0.002, LATEX), kit.band(tail_a, 0.022, 0.002, LATEX),
                            kit.band(tail_b, 0.022, 0.002, LATEX)),
                    colliders=(kit.Collider((0, 0, 0), size=TQ),
                               kit.Collider((0, foot_y, TQ_FOOT_Z), size=(0.02, 0.004, 0.004)),
                               kit.Collider((0, -foot_y, TQ_FOOT_Z), size=(0.02, 0.004, 0.004))))


def _needle():
    """The tube holder lies along the arm, nose down 12 degrees: clear barrel, finger flange, purple-capped tube full
    of blood, and the needle running from the nose into the vein. It collides as one flat block."""
    tilt = math.radians(12)
    ax = np.array([math.cos(tilt), 0.0, -math.sin(tilt)])
    c = np.array([0.0, 0.0, 0.002])
    return kit.Prop("needle", pos=NEEDLE_START, size=NEEDLE, mass=0.03,
                    meshes=(kit.cylinder(c, 0.0115, 0.06, CLEAR, axis=ax),
                            kit.cylinder(c - 0.029 * ax, 0.017, 0.002, CLEAR, axis=ax),
                            kit.cylinder(c - 0.006 * ax, 0.0065, 0.045, BLOOD, axis=ax),
                            kit.cylinder(c - 0.032 * ax, 0.0075, 0.012, CAP, axis=ax),
                            kit.cylinder(c + 0.032 * ax, 0.003, 0.006, CLEAR, axis=ax),
                            kit.cylinder(c + 0.050 * ax, 0.0011, 0.032, STEEL, axis=ax)),
                    colliders=(kit.Collider((0, 0, 0), size=NEEDLE),))


def _sharps():
    ix, iy, h, w = SHARPS_IN_X, SHARPS_IN_Y, SHARPS_H, WALL
    walls = [((ix / 2 + w / 2, 0, 0), (w, iy + 2 * w, h)), ((-ix / 2 - w / 2, 0, 0), (w, iy + 2 * w, h)),
             ((0, iy / 2 + w / 2, 0), (ix, w, h)), ((0, -iy / 2 - w / 2, 0), (ix, w, h))]
    lid_t = 0.006
    zl = h / 2 - lid_t / 2
    sx, sy = (ix - OPEN_X) / 2, (iy - OPEN_Y) / 2
    lid = [((OPEN_X / 2 + sx / 2, 0, zl), (sx, iy, lid_t)), ((-OPEN_X / 2 - sx / 2, 0, zl), (sx, iy, lid_t)),
           ((0, OPEN_Y / 2 + sy / 2, zl), (OPEN_X, sy, lid_t)), ((0, -OPEN_Y / 2 - sy / 2, zl), (OPEN_X, sy, lid_t))]
    label = kit.box((-ix / 2 - w - 0.0006, 0, 0.005), (0.001, 0.04, 0.04), HAZARD)
    return kit.Prop("sharps", pos=(SHARPS_X, SHARPS_Y, TOP + h / 2), size=(ix + 2 * w, iy + 2 * w, h),
                    meshes=tuple(kit.box(c, s, SHARPS_RED) for c, s in walls)
                    + tuple(kit.box(c, s, LID) for c, s in lid) + (label,),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in walls + lid))


OBJECTS = [
    kit.Prop("armrest", pos=(REST_X, 0.0, TOP + REST_H / 2), size=(REST_L, REST_W, REST_H),
             meshes=(kit.box((0, 0, 0), (REST_L, REST_W, REST_H), PAD),),
             colliders=(kit.Collider((0, 0, 0), size=(REST_L, REST_W, REST_H)),)),
    _forearm(),
    kit.Prop("tray", pos=(TRAY_X, TRAY_Y, TOP + TRAY_H / 2), size=(0.12, 0.10, TRAY_H),
             meshes=(kit.box((0, 0, 0), (0.12, 0.10, TRAY_H), TRAY_LOOK),),
             colliders=(kit.Collider((0, 0, 0), size=(0.12, 0.10, TRAY_H)),)),
    _sharps(),
    _tourniquet(),
    _needle(),
    kit.Prop("gauze", pos=GAUZE_START, size=GAUZE, mass=0.04, meshes=(kit.box((0, 0, 0), GAUZE, GAUZE_LOOK),),
             colliders=(kit.Collider((0, 0, 0), size=GAUZE),)),
]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


# --- What happened during the run. success() and score() are called after every control step, with no
# reset hook, so the run's history is kept here. A reset puts every free object back at its start, at rest, in one
# step; that jump is how a new trial is recognized. Objects moved by the arms travel a few cm per step at most.
class _History:
    def __init__(self):
        self.last = None
        self.run = 0
        self.needle_out_under_tourniquet = False  # never forgiven


_H = _History()


def _on_arm(s):
    """The tourniquet on the arm: its grip block over the forearm and within 1.5 cm of resting on it."""
    return bool(abs(s.pos[0] - ARM_X) < ARM_L / 2 and abs(s.pos[1]) < ARM_R + 0.01
                and s.pos[2] < TQ_START[2] + 0.015)


def _needle_in(s):
    """The needle still in the vein: its holder within 1.5 cm of where it started, and down on the arm."""
    return bool(np.hypot(s.pos[0] - NEEDLE_START[0], s.pos[1] - NEEDLE_START[1]) < 0.015
                and s.pos[2] < NEEDLE_START[2] + 0.015)


def _in_sharps(s):
    return bool(abs(s.pos[0] - SHARPS_X) < SHARPS_IN_X / 2 and abs(s.pos[1] - SHARPS_Y) < SHARPS_IN_Y / 2
                and s.pos[2] < TOP + SHARPS_H)


def _gauze_on_site(s):
    return bool(np.hypot(s.pos[0] - SITE[0], s.pos[1] - SITE[1]) < 0.025 and s.pos[2] < ARM_TOP + GAUZE[2] / 2 + 0.01
                and s.resting())


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
    bad = _on_arm(objects["tourniquet"]) and not _needle_in(objects["needle"])
    _H.run = _H.run + 1 if bad else 0
    if _H.run >= VIOLATION_STEPS:
        _H.needle_out_under_tourniquet = True


def _score(objects):
    if _H.needle_out_under_tourniquet or _on_arm(objects["tourniquet"]) or _needle_in(objects["needle"]):
        return 0.0
    n, g = objects["needle"], objects["gauze"]
    return 0.5 + 0.25 * (_in_sharps(n) and n.resting()) + 0.25 * _gauze_on_site(g)


def success(objects):
    """1.0: tourniquet off the arm before the needle came out, needle in the sharps container, gauze on the site.
    Called first each step, so it also records the step's history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the needle ever left the vein while the tourniquet was still on the arm, even if the tourniquet came off
    later. 0 also while the needle is still in or the tourniquet still on at the end. Otherwise 0.5 for the right
    order, plus 0.25 with the needle in the sharps container and 0.25 with the gauze pad on the puncture site.
    Possible scores: 0, 0.5, 0.75, 1."""
    return _score(objects)


def demo():
    """Right arm: lift the tourniquet off the upper arm and stand it on the bench, draw the needle out and drop it
    through the sharps container's lid, then press the gauze pad on the puncture site."""

    def grip(top, below=0.012):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.22
    lag = 0.012  # the arm falls ~1.5 cm short of a target near the middle of the bench, toward its own base
    tq_top = TQ_START[2] + TQ[2] / 2
    tq_stand = TQ[2] / 2 - (TQ_FOOT_Z - 0.002)  # from the block's top down to the band's feet
    tq = pick_place_waypoints((TQ_START[0], TQ_START[1] + lag, grip(tq_top, 0.010)),
                              (0.40, -0.075, grip(TOP + tq_stand, 0.010) + 0.004),
                              travel_z=travel, yaw=np.pi / 2, place_yaw=0.0)
    n_top = NEEDLE_START[2] + NEEDLE[2] / 2
    needle = pick_place_waypoints((NEEDLE_START[0], NEEDLE_START[1] + lag, grip(n_top, 0.016)),
                                  (SHARPS_X, SHARPS_Y, TOP + SHARPS_H + 0.06), travel_z=travel)
    gauze = pick_place_waypoints((TRAY_X, TRAY_Y, grip(GAUZE_START[2] + GAUZE[2] / 2)),
                                 (SITE[0], SITE[1] + lag, grip(ARM_TOP + GAUZE[2]) + 0.003), travel_z=travel)
    park = (0.16, -0.26, TOP + 0.22)
    right = tq + needle + gauze + [Waypoint(1.2, park, FINGER_OPEN, "park"), Waypoint(2.0, park, FINGER_OPEN, "hold")]
    return {"right": right}
