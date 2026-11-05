"""On the station table: three thin orange survey marks stand fixed to the table between the arms, spread from
near the robot to far out, and nothing is drawn on the table. A loose blue pin stands upright on the table in front of the
left arm, near the robot. Stand the blue pin upright on the table at the one spot that is the same distance from
all three survey marks, then let go.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
MARK_R, MARK_H = 0.008, 0.080
MARKS = {"mark_a": (0.139, -0.021), "mark_b": (0.306, -0.118), "mark_c": (0.428, 0.004)}
PIN_R, PIN_H, PIN_MASS = 0.015, 0.060, 0.05
PIN_START = (0.15, 0.20)

ORANGE, BLUE = (0.98, 0.42, 0.05), (0.15, 0.30, 0.85)

OBJECTS = [
    *[Cylinder(n, pos=(*xy, TOP + MARK_H / 2), radius=MARK_R, height=MARK_H, color=ORANGE) for n, xy in MARKS.items()],
    Cylinder("pin", pos=(*PIN_START, TOP + PIN_H / 2), radius=PIN_R, height=PIN_H, mass=PIN_MASS, color=BLUE),
]

_M = np.array(list(MARKS.values()))


def _circumcentre(p) -> np.ndarray:
    (ax, ay), (bx, by), (cx, cy) = p
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d
    return np.array([ux, uy])


CENTRE = _circumcentre(_M)  # about (0.280, 0.030): the marks lie on a circle of radius about 150 mm around it
RADIUS = float(np.linalg.norm(_M[0] - CENTRE))

TIERS = ((0.010, 1.0), (0.018, 0.5), (0.028, 0.25))  # spread of the three distances -> score
MAX_TILT_DEG = 5.0
BOTTOM_TOL = 0.001  # bottom within 1 mm of the table top: standing on the table, not held above it
EPS = 1e-6  # stated limits pass exactly at the limit


def spread(xy) -> float:
    """Largest minus smallest of the three distances from a point to the mark centres (m)."""
    d = np.linalg.norm(_M - np.asarray(xy, float)[:2], axis=1)
    return float(d.max() - d.min())


def _tilt_deg(st) -> float:
    x, y, z, w = st.quat_xyzw
    axis_z = 1.0 - 2.0 * (x * x + y * y)  # the pin axis's vertical component
    return float(np.degrees(np.arccos(np.clip(abs(axis_z), 0.0, 1.0))))  # the pin is the same at both ends


def _standing(st) -> bool:
    """Upright (axis within 5 degrees of vertical), bottom within 1 mm of the table top, and at rest."""
    return bool(_tilt_deg(st) <= MAX_TILT_DEG + EPS and abs(st.low[2] - TOP) <= BOTTOM_TOL + EPS and st.resting())


def score(objects):
    """By the spread of the pin's three distances (its axis at half its height) to the mark centres : at most 10 mm 1.0, at most 18 mm 0.5, at
    most 28 mm 0.25, otherwise 0. 0 unless the pin stands upright on the table at rest."""
    pin = objects["pin"]
    if not _standing(pin):
        return 0.0
    s = spread(pin.pos)
    for tol, value in TIERS:
        if s <= tol + EPS:
            return value
    return 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return float(np.linalg.norm(objects["pin"].pos[:2] - CENTRE))


GRIP_Z = TOP + PIN_H - 0.020  # TCP 20 mm below the pin's top; finger tips 33 mm below it, 27 mm above the table
PLACE_Z = GRIP_Z + 0.007  # pin bottom about 7 mm above the table at release: it drops, so a held pin never counts
TRAVEL_Z = TOP + 0.15  # a carried pin's bottom is 110 mm up, clear of the 80 mm marks
# Measured tracking error of the left arm carrying a light object near x 0.30 m: about 2.4 mm to the robot's
# left of its target. The demo aims that much to the right.
SAG_Y = 0.0024


def demo():
    (sx, sy), (gx, gy) = PIN_START, (CENTRE[0], CENTRE[1] - SAG_Y)
    left = [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), FINGER_OPEN, "over the pin"),
        Waypoint(1.5, (sx, sy, GRIP_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (sx, sy, GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (gx, gy, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.5, (gx, gy, PLACE_Z), FINGER_CLOSED, "lower"),
        Waypoint(1.0, (gx, gy, PLACE_Z), FINGER_CLOSED, "settle"),
        Waypoint(0.6, (gx, gy, PLACE_Z), FINGER_OPEN, "release"),
        Waypoint(1.2, (gx, gy, TRAVEL_Z), FINGER_OPEN, "up"),
        Waypoint(1.5, (0.15, 0.25, TRAVEL_Z), FINGER_OPEN, "clear"),
    ]
    return {"left": left}
