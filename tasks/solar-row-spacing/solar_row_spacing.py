"""On the station table: a flat roof deck (180 mm front to back, 140 mm side to side, 20 mm thick) is fixed in front
of the arms. A 20 mm red fire-access border runs round its edge; the grey area inside it is where solar modules may
stand. The sun is on the arms' side: a module's shadow falls behind it, away from the arms. Two dark blue solar
module blocks (each 30 mm front to back, 60 mm side to side) lie on the table to either side of the roof: a short
one (25 mm tall) on the left and a tall one (50 mm tall) on the right.

Goal: both modules standing on the grey area, long side facing the sun, with the gap behind each module at least
twice its height. The deck is only deep enough for that with the short module in front.

Run it:  inspect-robots run --embodiment yam_arena -E task=solar_row_spacing.py -E object_jitter=0 \\
           --instruction "Install both solar modules on the roof's grey area, long side facing you, clear of the red \\
           border. Leave a gap behind each module at least twice its height before the next module, so no module \\
           shades another." --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the roof: grey usable area plus four flush red border strips, all fixed ---------------------------------
ROOF_X0, ROOF_DEPTH, ROOF_W, ROOF_T = 0.230, 0.180, 0.140, 0.020  # front edge x, depth, width, thickness
BORDER = 0.020
ROOF_TOP = TOP + ROOF_T
GREY = (0.55, 0.56, 0.58)
RED = (0.80, 0.12, 0.10)
USE_X = (ROOF_X0 + BORDER, ROOF_X0 + ROOF_DEPTH - BORDER)  # usable area, front and back
USE_Y = (-ROOF_W / 2 + BORDER, ROOF_W / 2 - BORDER)  # right and left
_cx, _zc = ROOF_X0 + ROOF_DEPTH / 2, TOP + ROOF_T / 2
_ux, _uw = USE_X[1] - USE_X[0], USE_Y[1] - USE_Y[0]

# -- the modules -----------------------------------------------------------------------------------------------
MOD_DEPTH, MOD_W = 0.030, 0.060  # front to back, side to side
MODULES = {
    "short_module": {"h": 0.025, "mass": 0.06, "start": (0.30, 0.17)},
    "tall_module": {"h": 0.050, "mass": 0.10, "start": (0.30, -0.17)},
}
PV_BLUE = (0.08, 0.12, 0.35)

OBJECTS = [
    Box("roof_area", pos=(_cx, 0.0, _zc), size=(_ux, _uw, ROOF_T), color=GREY),
    Box("border_front", pos=(ROOF_X0 + BORDER / 2, 0.0, _zc), size=(BORDER, ROOF_W, ROOF_T), color=RED),
    Box("border_back", pos=(ROOF_X0 + ROOF_DEPTH - BORDER / 2, 0.0, _zc), size=(BORDER, ROOF_W, ROOF_T), color=RED),
    Box("border_left", pos=(_cx, ROOF_W / 2 - BORDER / 2, _zc), size=(_ux, BORDER, ROOF_T), color=RED),
    Box("border_right", pos=(_cx, -ROOF_W / 2 + BORDER / 2, _zc), size=(_ux, BORDER, ROOF_T), color=RED),
    *[Box(n, pos=(m["start"][0], m["start"][1], TOP + m["h"] / 2), size=(MOD_DEPTH, MOD_W, m["h"]),
          mass=m["mass"], color=PV_BLUE) for n, m in MODULES.items()],
]

# -- scoring ---------------------------------------------------------------------------------------------------
GAP_FACTOR = 2.0  # clear gap behind a module >= 2 x its height
FLAT_DZ, FLAT_TILT_DEG, FACE_YAW_DEG = 0.004, 5.0, 10.0


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _footprint(name, st):
    """Top-down bounding box (x0, x1, y0, y1) of a module's corners."""
    h = np.array([MOD_DEPTH, MOD_W, MODULES[name]["h"]]) / 2
    c = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]) * h @ _rot(st.quat_xyzw).T
    xy = st.pos[:2] + c[:, :2]
    return xy[:, 0].min(), xy[:, 0].max(), xy[:, 1].min(), xy[:, 1].max()


def installed(name, st):
    """Standing on the grey area: centre within 4 mm of its resting height, tilted under 5 degrees, long side within
    10 degrees of side to side (facing the sun), and its whole footprint inside the grey area."""
    R = _rot(st.quat_xyzw)
    tilt = math.degrees(math.acos(min(1.0, abs(R[2, 2]))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0])) % 180.0  # the module's front-to-back axis vs world x
    yaw = min(yaw, 180.0 - yaw)
    x0, x1, y0, y1 = _footprint(name, st)
    on_grey = x0 >= USE_X[0] and x1 <= USE_X[1] and y0 >= USE_Y[0] and y1 <= USE_Y[1]
    rest = abs(st.pos[2] - (ROOF_TOP + MODULES[name]["h"] / 2)) < FLAT_DZ
    return rest and tilt < FLAT_TILT_DEG and yaw < FACE_YAW_DEG and on_grey


def unshaded(objects):
    """No module is in another's shadow: wherever two modules overlap side to side, the clear gap from the front one's
    back face to the rear one's front face is at least twice the front one's height."""
    (a, b) = MODULES
    fa, fb = _footprint(a, objects[a]), _footprint(b, objects[b])
    if min(fa[3], fb[3]) - max(fa[2], fb[2]) <= 0:  # no side-to-side overlap
        return True
    front, rear = ((a, fa), (b, fb)) if objects[a].pos[0] < objects[b].pos[0] else ((b, fb), (a, fa))
    gap = rear[1][0] - front[1][1]
    return gap >= GAP_FACTOR * MODULES[front[0]]["h"]


def score(objects):
    """0.25 per module installed on the grey area; +0.5 if both are installed and neither shades the other."""
    ok = [installed(n, objects[n]) for n in MODULES]
    s = 0.25 * sum(ok)
    if all(ok) and unshaded(objects):
        s += 0.5
    return s


def success(objects):
    return score(objects) >= 0.999 and all(objects[n].resting() for n in MODULES)


TARGET = {"short_module": (0.272, 0.010), "tall_module": (0.3635, -0.010)}


def distance(objects):
    return float(sum(math.hypot(objects[n].pos[0] - x, objects[n].pos[1] - y) for n, (x, y) in TARGET.items()))


# -- scripted demo ---------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.08  # higher, the wrist (joint4) nears its limit on the back row
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along world x, across the 30 mm depth
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.010
RELEASE_GAP = 0.003
SETTLE_S = 0.8
OPEN_W = -(MOD_DEPTH / 2 + 0.012)


def _install(name, yaw):
    sx, sy = MODULES[name]["start"]
    gx, gy = TARGET[name]
    pick_z = TOP + GRIP_ABOVE_BOTTOM
    drop_z = ROOF_TOP + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), OPEN_W, f"to the {name}", yaw),
        Waypoint(1.2, (sx, sy, pick_z), OPEN_W, "descend", yaw),
        Waypoint(0.8, (sx, sy, pick_z), 0.0, "grip", yaw),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), 0.0, "lift", yaw),
        Waypoint(1.8, (gx, gy, TRAVEL_Z), 0.0, "over its row", yaw),
        Waypoint(1.0, (gx, gy, drop_z + 0.012), 0.0, "lower", yaw),
        Waypoint(SETTLE_S, (gx, gy, drop_z + 0.012), 0.0, "settle", yaw),
        Waypoint(0.8, (gx, gy, drop_z), 0.0, "to the roof", yaw),
        Waypoint(SETTLE_S, (gx, gy, drop_z), 0.0, "settle", yaw),
        Waypoint(0.6, (gx, gy, drop_z), OPEN_W, "let go", yaw),
        Waypoint(1.0, (gx, gy, TRAVEL_Z), OPEN_W, "up", yaw),
    ]


def demo():
    """Left arm stands the short module in the front row, 7 mm behind the red border, and moves clear; the right arm
    then stands the tall module in the back row, leaving a 62 mm gap behind the short module (50 mm needed)."""
    left = _install("short_module", YAW["left"]) + [Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["left"])]
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")] + _install(
        "tall_module", YAW["right"]
    ) + [Waypoint(1.5, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["right"])]
    return {"left": left, "right": right}
