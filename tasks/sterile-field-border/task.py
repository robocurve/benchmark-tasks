"""A sterile field: a blue drape (fixed, 24 x 36 cm, 3 mm thick) spread on the table in front of the arms.
Off the drape, on a grey side-table tray to the robot's right, lie three sterile instruments, each along x:
forceps (silver, 12 x 1.6 x 1.2 cm) and a clamp (dark steel, 11 x 2 x 1.2 cm) at the tray's near end, and a
retractor (light steel, 12 x 2.4 x 1.2 cm) at its far end.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Set the instruments out on the sterile field."  # read by our run script, not the framework

# Drape: x 20..44, y -14..22.
DRAPE_X0, DRAPE_X1, DRAPE_Y0, DRAPE_Y1, DRAPE_T = 0.20, 0.44, -0.14, 0.22, 0.003
DRAPE_TOP = TOP + DRAPE_T
BORDER = 0.025  # the outer 2.5 cm (1 inch) of a sterile drape counts as unsterile
DRAPE = Box("drape", pos=((DRAPE_X0 + DRAPE_X1) / 2, (DRAPE_Y0 + DRAPE_Y1) / 2, TOP + DRAPE_T / 2),
            size=(DRAPE_X1 - DRAPE_X0, DRAPE_Y1 - DRAPE_Y0, DRAPE_T), color=(0.25, 0.45, 0.80))

TRAY = Box("side_tray", pos=(0.22, -0.225, TOP + 0.005), size=(0.30, 0.16, 0.01), color=(0.55, 0.55, 0.57))
H = 0.012
INSTRUMENTS = {  # name: (start x, start y, length x, width y, colour); spaced so open fingers clear neighbours
    "forceps": (0.15, -0.18, 0.12, 0.016, (0.80, 0.80, 0.82)),
    "clamp": (0.15, -0.27, 0.11, 0.020, (0.35, 0.36, 0.40)),
    "retractor": (0.30, -0.225, 0.12, 0.024, (0.65, 0.66, 0.70)),
}
ITEMS = {n: Box(n, pos=(x, y, TOP + 0.01 + H / 2), size=(lx, wy, H), mass=0.03, color=c)
         for n, (x, y, lx, wy, c) in INSTRUMENTS.items()}
OBJECTS = [DRAPE, TRAY, *ITEMS.values()]


def _on_drape(s):
    """Lying still and flat (tilted under 20 degrees) directly on the drape: centre over it, bottom within 8 mm
    of its top."""
    qx, qy, _qz, _qw = s.quat_xyzw
    return bool(
        s.resting() and 1 - 2 * (qx * qx + qy * qy) > np.cos(np.radians(20))
        and DRAPE_X0 < s.pos[0] < DRAPE_X1 and DRAPE_Y0 < s.pos[1] < DRAPE_Y1
        and s.low[2] < DRAPE_TOP + 0.008
    )


def _clear_of_border(s):
    """The whole footprint (bounding box) is at least 2.5 cm inside every drape edge."""
    return bool(
        s.low[0] >= DRAPE_X0 + BORDER and s.high[0] <= DRAPE_X1 - BORDER
        and s.low[1] >= DRAPE_Y0 + BORDER and s.high[1] <= DRAPE_Y1 - BORDER
    )


def _good(s):
    return _on_drape(s) and _clear_of_border(s)


def success(objects):
    """1.0: all three instruments lie on the drape, each wholly inside the 2.5 cm border."""
    return all(_good(objects[n]) for n in ITEMS)


def score(objects):
    """1/3 per instrument lying still directly on the drape with its whole footprint at least 2.5 cm inside
    every edge. An instrument on the border, hanging over an edge, on another instrument, off the drape or
    held earns nothing (anything touching the unsterile border is treated as contaminated)."""
    return sum(_good(objects[n]) for n in ITEMS) / 3.0


def demo():
    """Right arm: each instrument from the tray to the drape's centre line, spaced 6 cm apart along y, released
    1 cm above the drape."""
    right = []
    targets = {"forceps": 0.06, "clamp": 0.0, "retractor": -0.06}
    for n, ty in targets.items():
        x, y = INSTRUMENTS[n][:2]
        grip = (x, y, TOP + 0.01 + 0.004 + 0.013)
        place = (0.32, ty, DRAPE_TOP + 0.01 + 0.004 + 0.013)
        right += pick_place_waypoints(grip, place, travel_z=TOP + 0.12)
    right.append(Waypoint(1.5, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"right": right}
