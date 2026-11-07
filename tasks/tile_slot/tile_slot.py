"""On the station table: a 4 x 4 grid of painted wooden tiles glued to the table on the left half,
with one gap in the grid's left edge, and three loose tiles (red, blue, yellow) in front of the right arm. The robot
fills the gap with the loose tile that makes the largest single-colour region of the grid as large as possible.

The grid tiles stand 2 mm apart and the gap leaves 2.5 mm on each side of a loose tile, less than the thickness of a
YAM fingertip (about 4 mm at its end), so no finger fits beside a tile in the gap. A tile can be set down outside the
gap but has to be slid into it from the open (left) side.

Coordinates in cm, x forward, y left. Grid tiles 6.1 x 6.1 x 3 cm, pitch 6.3: rows at x 37, 30.7, 24.4, 18.1 (far
to near), columns at y 26, 19.7, 13.4, 7.1 (left to right, as the arms see it). Colours by row, left to right
(- is the gap): R R Y Y / R B B Y / - B B B / R R R Y. Loose tiles (6 x 6 x 3 cm, 65 g): blue (17, -25),
red (26, -25), yellow (35, -25).
Neither arm can reach in front of the other, so the chosen tile changes hands.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/tile_slot/tile_slot.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
    --instruction "Slide one loose tile into the gap so that the largest single-colour region is as large as possible." \\
    --scorer task_score --max-steps 2400 --store-frames --log-dir runs/tile-slot-demo
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

TILE, T = 0.06, 0.03  # loose tile edge and thickness (m)
GLUED = 0.061  # grid tile edge: 2 mm gaps at this pitch, and 2.5 mm on each side of a loose tile in the gap
PITCH = 0.063
MASS = 0.065
COLORS = {"R": (0.85, 0.12, 0.12), "B": (0.15, 0.30, 0.85), "Y": (0.95, 0.80, 0.10)}
NAMES = {"R": "red", "B": "blue", "Y": "yellow"}
ROWS_X = (0.37, 0.307, 0.244, 0.181)  # far to near
COLS_Y = (0.26, 0.197, 0.134, 0.071)  # left to right, as the arms see it
GRID = ("RRYY", "RBBY", "-BBB", "RRRY")
GAP = (2, 0)
GAP_XY = np.array([ROWS_X[GAP[0]], COLS_Y[GAP[1]]])

GLUED_TILES = [
    Box(f"tile_{r}{c}_{NAMES[k]}", pos=(ROWS_X[r], COLS_Y[c], TOP + T / 2), size=(GLUED, GLUED, T), color=COLORS[k])
    for r, row in enumerate(GRID)
    for c, k in enumerate(row)
    if k != "-"
]
START = {"blue": (0.17, -0.25), "red": (0.26, -0.25), "yellow": (0.35, -0.25)}
LOOSE = {"blue": "B", "red": "R", "yellow": "Y"}
OPTIONS = [
    Box(name, pos=(*xy, TOP + T / 2), size=(TILE, TILE, T), mass=MASS, color=COLORS[LOOSE[name]])
    for name, xy in START.items()
]
OBJECTS = [*GLUED_TILES, *OPTIONS]


# -- scoring --------------------------------------------------------------------------------------------------------
def largest_region(fill: str) -> int:
    """Tiles in the largest 4-connected single-colour region once the gap holds colour ``fill``."""
    grid = [list(row) for row in GRID]
    grid[GAP[0]][GAP[1]] = fill
    seen, best = set(), 0
    for r0 in range(4):
        for c0 in range(4):
            if (r0, c0) in seen:
                continue
            stack, size = [(r0, c0)], 0
            seen.add((r0, c0))
            while stack:
                r, c = stack.pop()
                size += 1
                for rn, cn in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                    if 0 <= rn < 4 and 0 <= cn < 4 and (rn, cn) not in seen and grid[rn][cn] == grid[r][c]:
                        seen.add((rn, cn))
                        stack.append((rn, cn))
            best = max(best, size)
    return best


# Largest region per loose tile: yellow 5 (the blue region stays largest), blue 6, red 7 (joins two red regions).
REGION = {name: largest_region(k) for name, k in LOOSE.items()}
CREDIT = {5: 0.25, 6: 0.5, 7: 1.0}
XY_TOL, Z_TOL = 0.01, 0.005


def in_gap(s) -> bool:
    """The tile's centre is over the gap's cell (at any height)."""
    return bool(np.all(np.abs(s.pos[:2] - GAP_XY) < PITCH / 2))


def seated(s) -> bool:
    """Lying flat on the table in the gap: centre within 1 cm of the gap's centre, bottom on the table, 3 cm tall,
    at rest."""
    return bool(
        np.linalg.norm(s.pos[:2] - GAP_XY) < XY_TOL
        and abs(s.low[2] - TOP) < Z_TOL
        and abs(s.high[2] - s.low[2] - T) < Z_TOL
        and s.resting()
    )


def score(objects):
    """By the largest single-colour region the seated tile makes: 0.25 for 5 tiles (yellow), 0.5 for 6 (blue), 1.0
    for 7 (red). 0 unless exactly one loose tile is over the gap and it is seated."""
    over = [n for n in START if in_gap(objects[n])]
    if len(over) != 1 or not seated(objects[over[0]]):
        return 0.0
    return CREDIT[REGION[over[0]]]


def distance(objects):
    """Metres from the red tile to its seated position in the gap."""
    return float(np.linalg.norm(objects["red"].pos - np.array([*GAP_XY, TOP + T / 2])))


# -- demo: the right arm sets the red tile down near the centre line and moves aside. The left arm sets it down just
# outside the gap's open side, grips it again across x (the parallel fingers square it up and centre it on the gap's
# row), slides it to the gap's mouth, lets go and pushes it in with closed fingers. ---------------------------------
TRAVEL_Z = TOP + 0.14
GRASP_Z = TOP + 0.022  # finger pads 9 mm above the table
PUSH_Z = TOP + 0.021
HANDOVER = (0.25, -0.04)
OUTSIDE = (GAP_XY[0], 0.345)  # tile centre: 2.5 cm clear of the gap's mouth
MOUTH_Y = COLS_Y[GAP[1]] + GLUED / 2 + TILE / 2 + 0.005  # tile centre 5 mm short of the gap's mouth
PUSH_FROM, PUSH_TO = 0.40, 0.30  # TCP y while pushing
ACROSS = np.pi / 2  # closed fingers side by side along x: a wider pushing face


def _rest(arm, seconds):
    return [Waypoint(seconds, (0.20, 0.30 if arm == "left" else -0.30, TRAVEL_Z), FINGER_OPEN, "rest")]


def demo():
    right = pick_place_waypoints((*START["red"], GRASP_Z), (*HANDOVER, GRASP_Z + 0.005), travel_z=TRAVEL_Z)
    right += _rest("right", 2.0)
    x = GAP_XY[0]
    left = _rest("left", sum(w.duration for w in right))  # wait until the right arm is out of the way
    left += pick_place_waypoints((*HANDOVER, GRASP_Z), (*OUTSIDE, GRASP_Z + 0.005), travel_z=TRAVEL_Z)
    left += [
        Waypoint(1.5, (*OUTSIDE, TRAVEL_Z), FINGER_OPEN, "turn the grip", ACROSS),
        Waypoint(1.5, (*OUTSIDE, GRASP_Z), FINGER_OPEN, "descend", ACROSS),
        Waypoint(0.8, (*OUTSIDE, GRASP_Z), FINGER_CLOSED, "square up", ACROSS),
        Waypoint(2.0, (x, MOUTH_Y, GRASP_Z + 0.003), FINGER_CLOSED, "slide to the gap", ACROSS),
        Waypoint(0.6, (x, MOUTH_Y, GRASP_Z + 0.003), FINGER_OPEN, "release", ACROSS),
        Waypoint(1.0, (x, MOUTH_Y, TRAVEL_Z), FINGER_OPEN, "up", ACROSS),
        Waypoint(1.5, (x, PUSH_FROM, TRAVEL_Z), FINGER_CLOSED, "behind the tile", ACROSS),
        Waypoint(1.5, (x, PUSH_FROM, PUSH_Z), FINGER_CLOSED, "lower", ACROSS),
        Waypoint(4.0, (x, PUSH_TO, PUSH_Z), FINGER_CLOSED, "push in", ACROSS),
        Waypoint(1.0, (x, PUSH_TO + 0.01, PUSH_Z), FINGER_CLOSED, "back off", ACROSS),
        Waypoint(1.5, (x, PUSH_TO + 0.01, TRAVEL_Z), FINGER_CLOSED, "up", ACROSS),
        *_rest("left", 2.0),
    ]
    return {"left": left, "right": right}
