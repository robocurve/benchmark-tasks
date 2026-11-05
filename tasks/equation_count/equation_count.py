"""On the station table: an addition of two single-digit numbers is laid out in flat black disks, glued to the far
part of the table and read from the arms' side (dot-matrix glyphs, 5 rows x 3 columns each, left to right: a digit,
a plus sign, a digit). Eight identical loose red 4 cm cubes lie in front of the arms, four in front of each arm, and a
fixed grey plate (16 x 16 x 0.5 cm) lies on the centre line between them. The robot puts as many cubes on the grey
plate as the sum the disks spell out, no more and no fewer.

Loose cubes (4 cm, 50 g, painted wood; centres in cm, x forward, y left): left (14, 19), (14, 27), (22, 19), (22, 27);
right (14, -19), (14, -27), (22, -19), (22, -27). Neither arm can reach in front of the other, so each arm can bring
at most its own four cubes.
Plate (fixed): centred at (25, 0).

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/equation_count/equation_count.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --instruction "Put as many cubes on the grey plate as the sum on the table." \\
    --scorer task_score --max-steps 2400 --store-frames --log-dir runs/equation-count-demo
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the equation: dot-matrix glyphs of flat disks. Row 0 is the far row (largest x); columns run left to right as
# seen from the arms (decreasing y). Glyphs are 3 columns wide with one blank column between them. -------------------
GLYPHS = {
    "2": ("###", "..#", "###", "#..", "###"),
    "3": ("###", "..#", "###", "..#", "###"),
    "+": ("...", ".#.", "###", ".#.", "..."),
}
EQUATION = "2+3"
ANSWER = 5
PITCH = 0.03  # disk centre spacing (m)
DISK_R, DISK_H = 0.012, 0.003
DISK_COLOR = (0.05, 0.05, 0.05)
ROW0_X = 0.54  # far row; the bottom row is at ROW0_X - 4 * PITCH = 0.42
N_COLS = 4 * len(EQUATION) - 1
COL0_Y = (N_COLS - 1) * PITCH / 2  # centres the equation on y = 0


def _disks():
    disks = []
    for g, ch in enumerate(EQUATION):
        name = {"+": "plus"}.get(ch, f"d{ch}")
        for r, row in enumerate(GLYPHS[ch]):
            for c, mark in enumerate(row):
                if mark == "#":
                    x, y = ROW0_X - r * PITCH, COL0_Y - (4 * g + c) * PITCH
                    disks.append(
                        Cylinder(
                            f"disk_{g}_{name}_r{r}c{c}",
                            pos=(x, y, TOP + DISK_H / 2),
                            radius=DISK_R,
                            height=DISK_H,
                            color=DISK_COLOR,
                        )
                    )
    return disks


# -- plate and cubes -------------------------------------------------------------------------------------------------
PLATE_XY, PLATE_SIZE, PLATE_T = (0.25, 0.0), 0.16, 0.005
PLATE_TOP = TOP + PLATE_T
PLATE = Box("plate", pos=(*PLATE_XY, TOP + PLATE_T / 2), size=(PLATE_SIZE, PLATE_SIZE, PLATE_T), color=(0.5, 0.5, 0.5))

EDGE, MASS = 0.04, 0.05
CUBE_COLOR = (0.85, 0.12, 0.12)
START = {
    "cube_l1": (0.14, 0.19),
    "cube_l2": (0.14, 0.27),
    "cube_l3": (0.22, 0.19),
    "cube_l4": (0.22, 0.27),
    "cube_r1": (0.14, -0.19),
    "cube_r2": (0.14, -0.27),
    "cube_r3": (0.22, -0.19),
    "cube_r4": (0.22, -0.27),
}
CUBES = [Box(n, pos=(x, y, TOP + EDGE / 2), size=(EDGE,) * 3, mass=MASS, color=CUBE_COLOR) for n, (x, y) in START.items()]

OBJECTS = [PLATE, *CUBES, *_disks()]

# -- scoring: count the cubes on the plate. A cube counts when its centre is over the plate and it stands on the plate
# (its lowest point within 1 cm of the plate's top) or on a counted cube (its lowest point within 1 cm of that cube's
# top, centres within half an edge horizontally). A cube held up in the air never counts. -------------------------
Z_TOL = 0.01


def _over_plate(s) -> bool:
    return bool(np.all(np.abs(s.pos[:2] - np.array(PLATE_XY)) < PLATE_SIZE / 2))


def counted(objects) -> list[str]:
    """Names of the cubes on the plate, directly or stacked on another counted cube."""
    candidates = [n for n in START if _over_plate(objects[n])]
    on = [n for n in candidates if abs(objects[n].low[2] - PLATE_TOP) < Z_TOL]
    grew = True
    while grew:
        grew = False
        for n in candidates:
            if n in on:
                continue
            s = objects[n]
            for m in on:
                b = objects[m]
                if abs(s.low[2] - b.high[2]) < Z_TOL and np.hypot(*(s.pos[:2] - b.pos[:2])) < EDGE / 2:
                    on.append(n)
                    grew = True
                    break
    return on


def count(objects) -> int:
    return len(counted(objects))


def success(objects):
    """Exactly the sum on the plate, and every cube at rest."""
    return count(objects) == ANSWER and all(objects[n].resting() for n in START)


def score(objects):
    """1.0 for exactly the sum; 0.1 per cube below it (at most 0.4); 0 for more cubes than the sum."""
    n = count(objects)
    if n == ANSWER:
        return 1.0
    return round(0.1 * n, 2) if n < ANSWER else 0.0


def distance(objects):
    """How many cubes (as a number) the plate is off from the sum."""
    return float(abs(count(objects) - ANSWER))


# -- demo: the left arm brings three of its cubes, then moves aside; the right arm brings two of its own --------------
TRAVEL_Z = TOP + 0.14
RELEASE_GAP = 0.005
PLACE_Z = PLATE_TOP + EDGE / 2 + RELEASE_GAP
PX, PY = PLATE_XY
LEFT_JOBS = [("cube_l1", (PX - 0.045, PY + 0.045)), ("cube_l3", (PX + 0.045, PY + 0.045)), ("cube_l2", (PX, PY))]
RIGHT_JOBS = [("cube_r1", (PX - 0.045, PY - 0.045)), ("cube_r3", (PX + 0.045, PY - 0.045))]


def _jobs(jobs):
    out = []
    for name, xy in jobs:
        out += pick_place_waypoints((*START[name], TOP + EDGE / 2), (*xy, PLACE_Z), travel_z=TRAVEL_Z)
    return out


def _rest(arm, seconds):
    return [Waypoint(seconds, (0.20, 0.30 if arm == "left" else -0.30, TRAVEL_Z), FINGER_OPEN, "rest")]


def demo():
    left = [*_jobs(LEFT_JOBS), *_rest("left", 2.0)]
    right = [*_rest("right", sum(w.duration for w in left)), *_jobs(RIGHT_JOBS), *_rest("right", 2.0)]
    return {"left": left, "right": right}
