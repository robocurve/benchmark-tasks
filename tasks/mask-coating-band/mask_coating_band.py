"""On the station table: a steel flat bar (140 x 40 x 15 mm) is clamped flat in front of the arms, its long side
running left to right. Its top face has a 50 mm orange band across the middle that is to be spray-coated; the
45 mm of grey bare steel on each side of the band must stay uncoated. Two loose aluminium masking plates
(55 x 50 x 12 mm) lie on the table nearer the arms, one in front of each arm.

Goal: one plate lying flat on each grey zone, covering it, with the orange band left uncovered.

Run it:  inspect-robots run --embodiment yam_arena -E task=mask_coating_band.py -E object_jitter=0 \\
           --instruction "Mask the bar for coating: lay one masking plate flat on each grey zone so both grey zones \\
           are covered and the orange band stays fully uncovered." --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the bar: three fixed boxes flush side by side (grey zone, orange band, grey zone) --------------------------
BAR_X, BAR_DEPTH, BAR_T = 0.33, 0.040, 0.015  # centre forward, front-to-back depth, thickness
BAND_W, ZONE_W = 0.050, 0.045  # orange band width, each grey zone's width (side to side)
BAR_TOP = TOP + BAR_T
STEEL = (0.55, 0.56, 0.58)
ORANGE = (0.95, 0.45, 0.08)
ZONE_Y = {"left": BAND_W / 2 + ZONE_W / 2, "right": -(BAND_W / 2 + ZONE_W / 2)}  # zone centres

# -- the masking plates ---------------------------------------------------------------------------------------
PLATE_SIZE = (0.050, 0.055, 0.012)  # front to back, side to side, thick
PLATE_MASS = 0.09  # aluminium
PLATE_START = {"left_plate": (0.22, 0.15), "right_plate": (0.22, -0.15)}
PLATE_COLOR = (0.80, 0.82, 0.86)

OBJECTS = [
    Box("bar_left_zone", pos=(BAR_X, ZONE_Y["left"], TOP + BAR_T / 2), size=(BAR_DEPTH, ZONE_W, BAR_T), color=STEEL),
    Box("bar_band", pos=(BAR_X, 0.0, TOP + BAR_T / 2), size=(BAR_DEPTH, BAND_W, BAR_T), color=ORANGE),
    Box("bar_right_zone", pos=(BAR_X, ZONE_Y["right"], TOP + BAR_T / 2), size=(BAR_DEPTH, ZONE_W, BAR_T), color=STEEL),
    *[Box(n, pos=(x, y, TOP + PLATE_SIZE[2] / 2), size=PLATE_SIZE, mass=PLATE_MASS, color=PLATE_COLOR)
      for n, (x, y) in PLATE_START.items()],
]
PLATES = tuple(PLATE_START)

# -- scoring --------------------------------------------------------------------------------------------------
ZONE_MIN_COVERED = 0.85  # a grey zone counts as masked at 85 % of its area covered by flat plates
ZONE_PARTIAL = 0.50
BAND_MIN_EXPOSED = 0.80  # each half of the band must stay at least 80 % uncovered
FLAT_DZ, FLAT_TILT_DEG = 0.003, 5.0
REST_Z = BAR_TOP + PLATE_SIZE[2] / 2  # a plate's centre height lying flat on the bar
GRID = 0.001


def _grid(y0, y1):
    """1 mm sample points over the bar's top between y0 and y1 (m), as an (n, 2) array of x, y."""
    xs = np.arange(BAR_X - BAR_DEPTH / 2 + GRID / 2, BAR_X + BAR_DEPTH / 2, GRID)
    ys = np.arange(y0 + GRID / 2, y1, GRID)
    gx, gy = np.meshgrid(xs, ys)
    return np.c_[gx.ravel(), gy.ravel()]


ZONE_PTS = {
    "left": _grid(BAND_W / 2, BAND_W / 2 + ZONE_W),
    "right": _grid(-BAND_W / 2 - ZONE_W, -BAND_W / 2),
}
BAND_PTS = {"left": _grid(0.0, BAND_W / 2), "right": _grid(-BAND_W / 2, 0.0)}


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _flat(st):
    """Lying flat on the bar's top: centre within 3 mm of its resting height, tilted under 5 degrees."""
    tilt = math.degrees(math.acos(min(1.0, abs(_rot(st.quat_xyzw)[2, 2]))))
    return abs(st.pos[2] - REST_Z) < FLAT_DZ and tilt < FLAT_TILT_DEG


def _inside_footprint(st, pts):
    """Which points lie under a flat plate's actual top-face outline (its yaw counts), as a boolean mask."""
    R = _rot(st.quat_xyzw)
    d = pts - st.pos[:2]
    u = d @ R[:2, 0]  # along the plate's own x
    v = d @ R[:2, 1]
    return (np.abs(u) <= PLATE_SIZE[0] / 2) & (np.abs(v) <= PLATE_SIZE[1] / 2)


def _under_shadow(st, pts):
    """Which points lie under a plate's top-down shadow at any height or tilt (its corners' bounding box)."""
    h = np.array(PLATE_SIZE) / 2
    corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * h @ _rot(st.quat_xyzw).T
    lo, hi = st.pos[:2] + corners[:, :2].min(0), st.pos[:2] + corners[:, :2].max(0)
    return np.all((pts >= lo) & (pts <= hi), axis=1)


def _fractions(objects, side):
    """(fraction of the grey zone covered by flat plates, fraction of that half of the band left uncovered)."""
    zone, band = ZONE_PTS[side], BAND_PTS[side]
    covered = np.zeros(len(zone), bool)
    shaded = np.zeros(len(band), bool)
    for n in PLATES:
        st = objects[n]
        if _flat(st):
            covered |= _inside_footprint(st, zone)
        shaded |= _under_shadow(st, band)
    return float(covered.mean()), 1.0 - float(shaded.mean())


def _side_score(objects, side):
    """0.5 if the zone is masked (>= 85 % covered) with its half of the band >= 80 % uncovered; 0.2 if the zone is
    at least half covered with the band clear; 0 if the band is covered or the zone mostly bare."""
    zone, band = _fractions(objects, side)
    if band < BAND_MIN_EXPOSED:
        return 0.0
    if zone >= ZONE_MIN_COVERED:
        return 0.5
    return 0.2 if zone >= ZONE_PARTIAL else 0.0


def score(objects):
    return round(_side_score(objects, "left") + _side_score(objects, "right"), 3)


def success(objects):
    return score(objects) >= 0.999 and all(objects[n].resting() for n in PLATES)


TARGET_Y = {"left_plate": BAND_W / 2 + PLATE_SIZE[1] / 2 + 0.001, "right_plate": -(BAND_W / 2 + PLATE_SIZE[1] / 2 + 0.001)}


def distance(objects):
    return float(sum(math.hypot(objects[n].pos[0] - BAR_X, objects[n].pos[1] - y) for n, y in TARGET_Y.items()))


# -- scripted demo --------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.09  # higher, the wrist (joint4) nears its limit
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along world x, across the plate's 50 mm side
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.006  # pad tips 6 mm above the plate's underside
RELEASE_GAP = 0.003  # let go with the plate's underside this far above the bar
SETTLE_S = 0.8
OPEN_W = -(PLATE_SIZE[0] / 2 + 0.012)  # pads 12 mm clear of each side of the plate


def _mask(name, yaw):
    sx, sy = PLATE_START[name]
    gy = TARGET_Y[name]
    pick_z = TOP + GRIP_ABOVE_BOTTOM
    drop_z = BAR_TOP + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), OPEN_W, f"to the {name}", yaw),
        Waypoint(1.2, (sx, sy, pick_z), OPEN_W, "descend", yaw),
        Waypoint(0.8, (sx, sy, pick_z), 0.0, "grip", yaw),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), 0.0, "lift", yaw),
        Waypoint(1.8, (BAR_X, gy, TRAVEL_Z), 0.0, "over its zone", yaw),
        Waypoint(1.0, (BAR_X, gy, drop_z + 0.012), 0.0, "lower", yaw),
        Waypoint(SETTLE_S, (BAR_X, gy, drop_z + 0.012), 0.0, "settle", yaw),
        Waypoint(0.8, (BAR_X, gy, drop_z), 0.0, "to the bar", yaw),
        Waypoint(SETTLE_S, (BAR_X, gy, drop_z), 0.0, "settle", yaw),
        Waypoint(0.6, (BAR_X, gy, drop_z), OPEN_W, "let go", yaw),
        Waypoint(1.0, (BAR_X, gy, TRAVEL_Z), OPEN_W, "up", yaw),
    ]


def demo():
    """Left arm lays its plate on the left grey zone, its inner edge 1 mm outside the band, and moves clear; then the
    right arm does the same on the right. Each plate is let go 3 mm above the bar and drops flat."""
    left = _mask("left_plate", YAW["left"]) + [Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["left"])]
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")] + _mask(
        "right_plate", YAW["right"]
    ) + [Waypoint(1.5, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["right"])]
    return {"left": left, "right": right}
