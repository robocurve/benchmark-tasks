"""On the station table: a small model of four glued 4 cm cubes stands fixed on the left half of the table, and four
matching loose cubes lie in front of the arms. The robot builds the model's mirror image from the loose cubes on the
grey build plate on the right half of the table, anywhere on the plate and turned any way about the vertical.

Model (fixed; centres in cm, x forward, y left): red (32, 22) and blue (32, 18) side by side on the table, yellow
(28, 18) on the table in front of blue, green on top of red. Every cube touches another face to face; the shape is
chiral, so no turn of a copy is its mirror image.
Loose cubes (4 cm, 50 g, painted wood): red (16, 20) and green (16, 10) in front of the left arm, blue (16, -24) and
yellow (14, -15) in front of the right arm. Neither arm can reach in front of the other, so cubes change hands.
Build plate (fixed, grey, 20 x 20 x 0.5 cm): centred at (30, -20).

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/mirror_build/mirror_build.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --instruction "build the mirror image of the model" --scorer task_score \\
    --max-steps 2400 --store-frames --log-dir runs/mirror-build-demo
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EDGE, MASS = 0.04, 0.05
COLORS = {
    "red": (0.85, 0.12, 0.12),
    "blue": (0.15, 0.30, 0.85),
    "green": (0.10, 0.65, 0.20),
    "yellow": (0.95, 0.80, 0.10),
}

# The model: (x, y, layer) per cube; layer 0 stands on the table, layer 1 on the cube below it.
MODEL = {"red": (0.32, 0.22, 0), "blue": (0.32, 0.18, 0), "yellow": (0.28, 0.18, 0), "green": (0.32, 0.22, 1)}
START = {"red": (0.16, 0.20), "green": (0.16, 0.10), "blue": (0.16, -0.24), "yellow": (0.14, -0.15)}


PLATE_XY, PLATE_SIZE, PLATE_T = (0.30, -0.20), 0.20, 0.005
PLATE_TOP = TOP + PLATE_T


def _z(layer, floor=TOP):
    return floor + EDGE / 2 + layer * EDGE


PLATE = Box(
    "build_plate", pos=(*PLATE_XY, TOP + PLATE_T / 2), size=(PLATE_SIZE, PLATE_SIZE, PLATE_T), color=(0.5, 0.5, 0.5)
)


REFERENCE = [
    Box(f"model_{c}", pos=(x, y, _z(layer)), size=(EDGE,) * 3, color=COLORS[c]) for c, (x, y, layer) in MODEL.items()
]
CUBES = [Box(c, pos=(x, y, _z(0)), size=(EDGE,) * 3, mass=MASS, color=COLORS[c]) for c, (x, y) in START.items()]
OBJECTS = [PLATE, *REFERENCE, *CUBES]

# -- scoring: the base (red, blue, yellow on the plate) must match the mirrored model's base after some turn about
# the vertical and some shift; green must sit on red. Only proper turns are tried, so a copy never matches. --------
BASE = ("red", "blue", "yellow")
MIRROR_XY = np.array([[MODEL[c][0], -MODEL[c][1]] for c in BASE])  # the model mirrored across y = 0
XY_TOL, Z_TOL = 0.015, 0.01


def base_error(objects) -> float:
    """Largest distance (m) of a base cube from where the best-fitting turned and shifted mirror base puts it."""
    p = np.array([objects[c].pos[:2] for c in BASE])
    pc, qc = p - p.mean(0), MIRROR_XY - MIRROR_XY.mean(0)
    angle = np.arctan2(np.sum(qc[:, 0] * pc[:, 1] - qc[:, 1] * pc[:, 0]), np.sum(qc * pc))
    c, s = np.cos(angle), np.sin(angle)
    fitted = qc @ np.array([[c, s], [-s, c]])
    return float(np.max(np.linalg.norm(pc - fitted, axis=1)))


def on_plate(s) -> bool:
    """The cube's centre is over the plate."""
    return bool(np.all(np.abs(s.pos[:2] - np.array(PLATE_XY)) < PLATE_SIZE / 2))


def base_built(objects) -> bool:
    standing = all(on_plate(objects[c]) and abs(objects[c].pos[2] - _z(0, PLATE_TOP)) < Z_TOL for c in BASE)
    return standing and base_error(objects) < XY_TOL


def green_error(objects) -> float:
    """Distance (m) of green from the top of red."""
    return float(np.linalg.norm(objects["green"].pos - objects["red"].pos - np.array([0.0, 0.0, EDGE])))


def green_on_red(objects) -> bool:
    off = objects["green"].pos - objects["red"].pos
    return bool(np.hypot(off[0], off[1]) < XY_TOL and abs(off[2] - EDGE) < Z_TOL)


def success(objects):
    """The whole mirror image built, and every cube at rest."""
    return base_built(objects) and green_on_red(objects) and all(objects[c].resting() for c in COLORS)


def score(objects):
    """0.5 for the mirrored base, 1.0 with green on red as well; 0 otherwise."""
    if not base_built(objects):
        return 0.0
    return 1.0 if green_on_red(objects) else 0.5


def distance(objects):
    """How far (m) the build is from the mirror image: the base's misfit plus green's offset from the top of red."""
    return base_error(objects) + green_error(objects)


# -- demo: first the left arm sets red and green down on the centre line and moves away; then the right arm builds
# the mirror image on the plate, where the model mirrored across the centre line would stand -------------------------
TRAVEL_Z = TOP + 0.14
HANDOVER = {"red": (0.16, 0.0), "green": (0.24, 0.0)}
RELEASE_GAP, SPACING = 0.005, 0.003  # let go this far up; leave this much room beside an already placed cube
ACROSS = np.pi / 2  # grip yaw that closes the fingers along x, clear of a neighbour beside the cube in y
BUILD = {  # (x, y, z) where the right arm sets each cube down
    "red": (0.32, -0.22, _z(0, PLATE_TOP)),
    "blue": (0.32, -0.18 + SPACING, _z(0, PLATE_TOP)),
    "yellow": (0.28 - SPACING, -0.18 + SPACING, _z(0, PLATE_TOP)),
    "green": (0.32, -0.22, _z(1, PLATE_TOP)),
}


def _put(pick_xy, name, yaw):
    x, y, z = BUILD[name]
    return pick_place_waypoints((*pick_xy, _z(0)), (x, y, z + RELEASE_GAP), travel_z=TRAVEL_Z, place_yaw=yaw)


def _rest(arm, seconds):
    return [Waypoint(seconds, (0.20, 0.30 if arm == "left" else -0.30, TRAVEL_Z), FINGER_OPEN, "rest")]


def _duration(waypoints):
    return sum(w.duration for w in waypoints)


def demo():
    left = [
        *pick_place_waypoints((*START["red"], _z(0)), (*HANDOVER["red"], _z(0) + RELEASE_GAP), travel_z=TRAVEL_Z),
        *pick_place_waypoints((*START["green"], _z(0)), (*HANDOVER["green"], _z(0) + RELEASE_GAP), travel_z=TRAVEL_Z),
        *_rest("left", 2.0),
    ]
    right = [
        *_rest("right", _duration(left)),  # wait until the left arm is out of the way
        *_put(HANDOVER["red"], "red", ACROSS),
        *_put(START["blue"], "blue", ACROSS),
        *_put(START["yellow"], "yellow", 0.0),
        *_put(HANDOVER["green"], "green", ACROSS),
        *_rest("right", 2.0),
    ]
    return {"left": left, "right": right}
