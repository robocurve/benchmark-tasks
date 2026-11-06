"""On the station table stands the top section of a clothes dryer: a grey cabinet top with a narrow
slot along its far edge, holding a lint screen — a flat white tray whose handle end protrudes two
centimetres — with a scatter of grey lint flakes settled on it. NSPIRE D021/D023: a lint screen
blocked with visible lint accumulation is a fire hazard; verifying it means pulling the screen out,
clearing the lint off it, and reseating it fully.

The routine has three stages, all checkable in a photo: the screen comes out of its slot, the lint
ends up off the screen, and the screen goes back fully seated. A screen reseated dirty is half the
job; a screen left out is none of it.

Run it:  inspect-robots run --embodiment yam_arena -E task=lint_screen_clean.py \
           --instruction "clean the lint screen and reseat it fully" --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

DRYER = Box("dryer", pos=(0.40, 0.0, TOP + 0.05), size=(0.30, 0.26, 0.10), color=(0.85, 0.85, 0.86))
SLOT_L = Box("slot_left", pos=(0.374, 0.0, TOP + 0.11), size=(0.008, 0.26, 0.02), color=(0.60, 0.62, 0.65))
SLOT_R = Box("slot_right", pos=(0.426, 0.0, TOP + 0.11), size=(0.008, 0.26, 0.02), color=(0.60, 0.62, 0.65))
# The lint screen is a 2 cm-thick tray with a long handle bar; it slides along y in the 2.8 cm slot
# gap and, seated, its handle end protrudes 7 cm (y = +0.20) past the slot mouth at y = +0.13 — the
# jaw grips it well clear of the wall ends, whose pads otherwise stall on them. Spawn 2 mm proud of
# the slot floor and settle.
SCREEN_REST = (0.40, 0.10, TOP + 0.11)
SCREEN = Box("screen", pos=(0.40, 0.10, TOP + 0.112), size=(0.024, 0.22, 0.02), mass=0.12,
             color=(0.94, 0.94, 0.94))
# Six lint flakes settled on the screen inside the slot (y 0.07..0.11); they ride the screen when it
# is pulled out. Spawn 2 mm proud of the screen's top and settle.
_FLAKES_Y = (0.07, 0.08, 0.09, 0.10, 0.105, 0.11)
_FLAKES_X = (0.396, 0.404, 0.398, 0.402, 0.395, 0.405)
FLAKES = [Box(f"lint{i}", pos=(x, y, TOP + 0.126), size=(0.008, 0.008, 0.008), mass=0.002,
              color=(0.55, 0.53, 0.50))
          for i, (x, y) in enumerate(zip(_FLAKES_X, _FLAKES_Y))]

OBJECTS = [DRYER, SLOT_L, SLOT_R, SCREEN] + FLAKES

SEATED_XY_TOL = 0.006
SEATED_Z_TOL = 0.004
CLEAR_XY = 0.016          # a flake within the screen's footprint (+4 mm) of its centre...
CLEAR_Z = 0.018           # ...and this close in z, is still on the screen; flakes knocked into the
                          # slot's side cracks sit ~2 cm off-centre and count as cleared

SUCCESS_HOLD_S = 0.4      # the final state is static; the plans end right after the reseat


def _seated(objects):
    s = objects["screen"]
    rx, ry, rz = SCREEN_REST
    return (abs(s.pos[0] - rx) < SEATED_XY_TOL and abs(s.pos[1] - ry) < SEATED_XY_TOL
            and abs(s.pos[2] - rz) < SEATED_Z_TOL)


def _clear(objects):
    """True when no lint flake is still sitting on the screen."""
    s = objects["screen"]
    for i in range(len(FLAKES)):
        f = objects[f"lint{i}"]
        dxy = np.hypot(f.pos[0] - s.pos[0], f.pos[1] - s.pos[1])
        if dxy < CLEAR_XY and abs(f.pos[2] - s.pos[2]) < CLEAR_Z:
            return False
    return True


def success(objects):
    """The screen is fully reseated and carries no lint."""
    return _seated(objects) and _clear(objects)


def distance(objects):
    """0.0 when done; otherwise 1 if the lint is still on the screen, else the seat offset (m)."""
    if not _clear(objects):
        return 1.0
    s = objects["screen"]
    rx, ry, rz = SCREEN_REST
    return max(abs(s.pos[0] - rx), abs(s.pos[1] - ry))


def score(objects):
    """0.5 for the screen fully reseated; 0.5 for the lint cleared off it; 1.0 for both."""
    return 0.5 * _seated(objects) + 0.5 * _clear(objects)


SCORE_OVER = "end"      # the final state persists (nothing swings back)


def demo():
    """The left arm side-grips the screen's handle end (finger target tuned to the 2.4 cm width),
    slides it out of the slot, sweeps the lint off the screen's end, then pushes the screen back
    fully home and releases."""
    z_grip = TOP + 0.11                        # the 2 cm tray's centre height
    z_wipe = TOP + 0.137                       # pads' bottoms 4 mm into the lint flakes' tops
    grip = -0.008                              # commanded ~1.6 cm aperture: stalls firmly on the 2.4 cm tray
    open_ = -0.04695                           # FINGER_OPEN
    return {
        "left": [
            Waypoint(2.0, (0.40, 0.185, z_grip), open_, "descend open beside the handle", yaw=1.5708),
            Waypoint(0.6, (0.40, 0.185, z_grip), grip, "close on the handle", yaw=1.5708),
            Waypoint(1.2, (0.40, 0.30, z_grip), grip, "slide the screen out", yaw=1.5708),
            Waypoint(1.0, (0.40, 0.185, z_grip), grip, "push the screen home", yaw=1.5708),
            Waypoint(0.5, (0.40, 0.185, z_grip), open_, "release", yaw=1.5708),
            Waypoint(1.0, (0.395, 0.05, z_wipe), -0.006, "lower the closed sweep behind the lint"),
            Waypoint(1.5, (0.395, 0.32, z_wipe), -0.006, "sweep the lint off the screen's end"),
            Waypoint(0.8, (0.405, 0.05, z_wipe), -0.006, "second sweep line"),
            Waypoint(1.5, (0.405, 0.32, z_wipe), -0.006, "sweep the remaining lint off"),
            Waypoint(1.0, (0.40, 0.20, TOP + 0.20), open_, "retreat"),
        ],
        "right": [],
    }
