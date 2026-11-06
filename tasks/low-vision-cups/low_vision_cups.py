"""At a care-home dining table: three residents' places, each with a plate and a drink mat, two of the mats
charcoal and one ivory. Two white tumblers and one black tumbler wait near the robot. Nobody says so, but for
residents with low vision or dementia a cup has to contrast with what it stands on, so the white tumblers go on
the charcoal mats and the black one on the ivory mat.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CLOTH = TOP + 0.003
MAT_X, MAT_SIDE, MAT_H = 0.36, 0.13, 0.002
CUP_R, CUP_H = 0.034, 0.09
CHARCOAL, IVORY = (0.13, 0.13, 0.15), (0.93, 0.90, 0.80)
DARK, LIGHT = "dark", "light"

MATS = {"mat_left": (0.20, DARK), "mat_middle": (0.0, LIGHT), "mat_right": (-0.20, DARK)}
CUPS = {"black_tumbler": ((0.22, 0.24), DARK), "white_tumbler_a": ((0.24, 0.10), LIGHT),
        "white_tumbler_b": ((0.22, -0.22), LIGHT)}

PLACES = [
    *[Box(name, pos=(MAT_X, y, CLOTH + MAT_H / 2), size=(MAT_SIDE, MAT_SIDE, MAT_H),
          color=CHARCOAL if tone == DARK else IVORY) for name, (y, tone) in MATS.items()],
    *[Cylinder(f"plate_{i}", pos=(0.53, y, CLOTH + 0.0075), radius=0.07, height=0.015, color=(0.96, 0.96, 0.94))
      for i, (y, _) in enumerate(MATS.values())],
    *[Box(f"napkin_{i}", pos=(0.53, y + 0.09, CLOTH + 0.003), size=(0.10, 0.035, 0.006), color=(0.62, 0.30, 0.35))
      for i, (y, _) in enumerate(MATS.values())],
]
CENTREPIECE = [
    Cylinder("vase", pos=(0.70, 0.10, CLOTH + 0.06), radius=0.03, height=0.12, color=(0.30, 0.45, 0.70)),
    *[Sphere(f"flower_{i}", pos=(0.70 + dx, 0.10 + dy, CLOTH + 0.14), radius=0.022, color=c)
      for i, (dx, dy, c) in enumerate([(0.0, 0.0, (0.95, 0.80, 0.20)), (0.03, 0.02, (0.90, 0.45, 0.55)),
                                       (-0.02, 0.03, (0.95, 0.95, 0.95))])],
    Box("menu_stand", pos=(0.70, -0.10, CLOTH + 0.06), size=(0.02, 0.10, 0.12), color=(0.90, 0.86, 0.76)),
    Cylinder("salt", pos=(0.68, -0.26, CLOTH + 0.03), radius=0.015, height=0.06, color=(0.95, 0.95, 0.95)),
    Cylinder("pepper", pos=(0.68, -0.22, CLOTH + 0.03), radius=0.015, height=0.06, color=(0.25, 0.25, 0.25)),
    Box("walking_stick", pos=(0.50, -0.44, CLOTH + 0.01), size=(0.42, 0.02, 0.02), color=(0.40, 0.25, 0.15)),
]
TUMBLERS = [Cylinder(name, pos=(*xy, CLOTH + CUP_H / 2), radius=CUP_R, height=CUP_H, mass=0.15,
                     color=CHARCOAL if tone == DARK else (0.98, 0.98, 0.98)) for name, (xy, tone) in CUPS.items()]
OBJECTS = [
    Box("tablecloth", pos=(0.45, 0.0, TOP + 0.0015), size=(0.90, 1.0, 0.003), color=(0.55, 0.66, 0.55)),
    *PLACES, *CENTREPIECE, *TUMBLERS,
]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _mat_under(cup):
    for name, (y, _) in MATS.items():
        if abs(cup.pos[0] - MAT_X) <= MAT_SIDE / 2 and abs(cup.pos[1] - y) <= MAT_SIDE / 2 \
                and cup.low[2] < CLOTH + MAT_H + 0.01 and cup.resting():
            return name
    return None


def score(objects):
    """0 if any tumbler ends tipped (more than 30 degrees) or stands on a mat of its own tone (white on ivory,
    black on charcoal). Otherwise one third for each mat with a contrasting tumbler standing upright on it (its
    centre over the mat). Scored at the end of the trial."""
    cups = {name: objects[name] for name in CUPS}
    if any(_tilt_deg(c) > 30 for c in cups.values()):
        return 0.0
    served = set()
    for name, cup in cups.items():
        mat = _mat_under(cup)
        if mat is None:
            continue
        if MATS[mat][1] == CUPS[name][1]:
            return 0.0
        served.add(mat)
    return len(served) / 3


def demo():
    """The right arm sets its white tumbler on the right charcoal mat while the left arm sets the other white
    tumbler on the left charcoal mat, then the left arm sets the black tumbler on the ivory mat. Both arms then
    move clear of the camera's view."""
    grip = CLOTH + 0.013 + 0.02
    place = CLOTH + MAT_H + 0.013 + 0.02 + 0.005

    def serve(cup, mat):
        plan = pick_place_waypoints((*CUPS[cup][0], grip), (MAT_X, MATS[mat][0], place), travel_z=TOP + 0.17)
        i = next(k for k, w in enumerate(plan) if w.label == "descend") + 1
        return [*plan[:i], Waypoint(0.6, plan[i - 1].pos, FINGER_OPEN, "settle"), *plan[i:]]

    left = [*serve("white_tumbler_a", "mat_left"), *serve("black_tumbler", "mat_middle"),
            Waypoint(2.0, (0.12, 0.42, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view")]
    right = [*serve("white_tumbler_b", "mat_right"),
             Waypoint(2.0, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view")]
    return {"left": left, "right": right}
