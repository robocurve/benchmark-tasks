"""On the station table, between the arms: an open-top mock kiln, fixed in place. Its inside is 21 cm front to back
(14 to 35 cm in front of the arm bases) and 33 cm side to side, centred on the midline; a 5 mm tan kiln shelf
covers its floor and grey walls 2 cm thick rise 6 cm above the shelf. Three dark grey shelf posts (2 cm across,
10 cm tall) stand on the shelf: one at the back centre, 31 cm in front of the arm bases, and one in each front
corner, touching the front and side walls. In front of the left arm stand a green pot (7 cm across, 9 cm tall) and
a blue pot (6 cm across, 8 cm tall); in front of the right arm stand a yellow pot (6 cm across, 7 cm tall), a red
pot (5 cm across, 6 cm tall) and a purple pot (5 cm across, 4.5 cm tall). The pots are loose upright cylinders.
Loaded, all five stand upright on the shelf with at least 2 cm of clear air to every other pot and post and at
least 1.5 cm to every wall.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The kiln: a 5 mm shelf on the table, four 2 cm walls 6 cm taller than the shelf, an open top.
IN_X = (0.14, 0.35)  # inside, front to back (in front of the arm bases)
IN_Y = (-0.165, 0.165)  # inside, side to side (left of the midline is +)
IN_D, IN_W = IN_X[1] - IN_X[0], IN_Y[1] - IN_Y[0]  # 21 cm deep, 33 cm wide
KILN_X = (IN_X[0] + IN_X[1]) / 2
SHELF_T, WALL_T, WALL_H = 0.005, 0.020, 0.060
FLOOR = TOP + SHELF_T  # the shelf's top, where the pots stand
WALL_TOP = FLOOR + WALL_H
KILN_GREY, SHELF_TAN, POST_GREY = (0.42, 0.42, 0.45), (0.62, 0.46, 0.28), (0.20, 0.20, 0.22)
_WZ, _WH = (TOP + WALL_TOP) / 2, WALL_TOP - TOP  # walls stand on the table, beside the shelf
KILN = [
    Box("kiln_shelf", pos=(KILN_X, 0.0, TOP + SHELF_T / 2), size=(IN_D, IN_W, SHELF_T), color=SHELF_TAN),
    Box("kiln_wall_front", pos=(IN_X[0] - WALL_T / 2, 0.0, _WZ), size=(WALL_T, IN_W + 2 * WALL_T, _WH),
        color=KILN_GREY),
    Box("kiln_wall_back", pos=(IN_X[1] + WALL_T / 2, 0.0, _WZ), size=(WALL_T, IN_W + 2 * WALL_T, _WH),
        color=KILN_GREY),
    Box("kiln_wall_left", pos=(KILN_X, IN_Y[1] + WALL_T / 2, _WZ), size=(IN_D, WALL_T, _WH), color=KILN_GREY),
    Box("kiln_wall_right", pos=(KILN_X, IN_Y[0] - WALL_T / 2, _WZ), size=(IN_D, WALL_T, _WH), color=KILN_GREY),
]

# Shelf posts: the back one 4 cm (centre) from the back wall on the midline; the front ones touch two walls each.
POST_R, POST_H = 0.010, 0.100
POST_XY = {"post_back": (0.31, 0.0), "post_front_left": (0.15, 0.155), "post_front_right": (0.15, -0.155)}
POSTS = [
    Cylinder(name, pos=(x, y, FLOOR + POST_H / 2), radius=POST_R, height=POST_H, color=POST_GREY)
    for name, (x, y) in POST_XY.items()
]

# The pots: radius, height (m), mass (kg), colour, start on the table. Green and blue go with the left arm.
POT_SPECS = {
    "pot_green": (0.035, 0.090, 0.35, (0.10, 0.50, 0.20), (0.18, 0.32)),
    "pot_blue": (0.030, 0.080, 0.28, (0.10, 0.25, 0.80), (0.30, 0.32)),
    "pot_yellow": (0.030, 0.070, 0.25, (0.90, 0.70, 0.10), (0.17, -0.32)),
    "pot_red": (0.025, 0.060, 0.18, (0.75, 0.18, 0.10), (0.265, -0.32)),
    "pot_purple": (0.025, 0.045, 0.15, (0.50, 0.15, 0.65), (0.355, -0.32)),
}
POT_NAMES = tuple(POT_SPECS)
POT_R = {n: s[0] for n, s in POT_SPECS.items()}
POT_H = {n: s[1] for n, s in POT_SPECS.items()}
START = {n: s[4] for n, s in POT_SPECS.items()}
POTS = [
    Cylinder(n, pos=(x, y, TOP + h / 2 + 0.001), radius=r, height=h, mass=kg, color=c)
    for n, (r, h, kg, c, (x, y)) in POT_SPECS.items()
]  # each starts 1 mm above the table, and settles

OBJECTS = [*KILN, *POSTS, *POTS]
SUCCESS_HOLD_S = 2.5  # loaded and still for 2.5 s

POT_CREDIT = 0.2  # per pot loaded cleanly
POT_GAP = 0.020  # at least 20 mm to every other pot (centre distance minus both radii)
POST_GAP = 0.020  # at least 20 mm to every post (centre distance minus pot and post radii)
WALL_GAP = 0.015  # at least 15 mm from the pot's side to every wall's inner face
MAX_TILT_DEG = 5.0  # upright: tilted at most 5 degrees
FLOOR_TOL = 0.003  # standing on the shelf: the pot's bottom within 3 mm of the shelf top
TOUCH = 0.002  # closer than 2 mm counts as touching (glaze contact)


EPS = 1e-6  # 1 micrometre of floating-point slack on the gap comparisons, far below what a ruler shows


def _tilt_deg(state) -> float:
    """Angle between the pot's axis and the vertical, either end up (the pots are closed cylinders, the same at both
    ends, so one standing upside down stands as flat as one the right way up)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(0.0, min(1.0, abs(1 - 2 * (x * x + y * y))))))


def _xy_gap(a_xy, ra, b_xy, rb) -> float:
    """Surface-to-surface gap seen from above: centre distance minus both radii."""
    return math.hypot(float(a_xy[0]) - float(b_xy[0]), float(a_xy[1]) - float(b_xy[1])) - ra - rb


def _in_kiln(state) -> bool:
    """Centre inside the kiln's inner outline, seen from above."""
    x, y = float(state.pos[0]), float(state.pos[1])
    return bool(IN_X[0] <= x <= IN_X[1] and IN_Y[0] <= y <= IN_Y[1])


def _post_gaps(objects, p):
    return [_xy_gap(objects[p].pos, POT_R[p], xy, POST_R) for xy in POST_XY.values()]


# Contact (glaze touching) is measured between the actual shapes, whatever the pots' tilt: a lying or leaning pot's
# ends and rims count, not just its upright radius from the centre, and each post is a flat-topped solid cylinder.
def _rotation(state):
    """3x3 rotation matrix; its columns are the object's own x, y and up axes in the world."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _surface_points(r, h):
    """Points on a cylinder's surface in its own frame: 25 rings down the side (rims included) and 2 rings plus
    the centre on each end, 96 points per ring (under 2.3 mm apart around each ring; rings 1.9 to 3.75 mm apart
    down the side, r/3 apart across the ends)."""
    ang = np.linspace(0.0, 2 * math.pi, 96, endpoint=False)
    c, s = np.cos(ang), np.sin(ang)
    rings = [(r, t) for t in np.linspace(-h / 2, h / 2, 25)]
    rings += [(f * r, t) for f in (1 / 3, 2 / 3) for t in (-h / 2, h / 2)]
    pts = [np.stack([rho * c, rho * s, np.full_like(c, t)], axis=1) for rho, t in rings]
    return np.concatenate(pts + [np.array([[0.0, 0.0, -h / 2], [0.0, 0.0, h / 2]])])


_SURFACE = {n: _surface_points(POT_R[n], POT_H[n]) for n in POT_NAMES}


def _extent(state, name):
    """The pot's lowest and highest corner in the world, worked out from its centre, tilt and size (not the
    simulator's bounding box, which is padded by a few millimetres). Along each world axis the cylinder reaches
    H/2 * |cos a| + R * sin a from its centre, where a is the angle between its axis and that world axis: upright,
    its bottom is H/2 below the centre; tilted by t, it is H/2 * cos t + R * sin t below."""
    up = _rotation(state)[:, 2]  # the pot's axis in the world
    half = POT_H[name] / 2 * np.abs(up) + POT_R[name] * np.sqrt(np.clip(1.0 - up * up, 0.0, 1.0))
    pos = np.asarray(state.pos, dtype=float)[:3]
    return pos - half, pos + half


def _wall_gap(state, name) -> float:
    """Smallest gap from the pot's side (its outermost point) to a wall's inner face; negative outside the kiln."""
    lo, hi = _extent(state, name)
    return min(float(lo[0]) - IN_X[0], IN_X[1] - float(hi[0]), float(lo[1]) - IN_Y[0], IN_Y[1] - float(hi[1]))


def _same_level(objects, a, b) -> bool:
    """Height ranges overlap, or are within 2 mm (a pot resting on another counts; one held clear above doesn't)."""
    (la, ha), (lb, hb) = _extent(objects[a], a), _extent(objects[b], b)
    return bool(la[2] <= hb[2] + TOUCH and lb[2] <= ha[2] + TOUCH)


def _post_surface_points():
    """Points on a post's side and flat top, relative to its foot's centre: 101 rings 1 mm apart up the side (the
    top rim included), 4 rings and the centre across the top, 64 points per ring (under 1 mm apart)."""
    ang = np.linspace(0.0, 2 * math.pi, 64, endpoint=False)
    c, s = np.cos(ang), np.sin(ang)
    rings = [(POST_R, t) for t in np.linspace(0.0, POST_H, 101)]
    rings += [(f * POST_R, POST_H) for f in (0.25, 0.5, 0.75)] + [(0.0, POST_H)]
    return np.concatenate([np.stack([rho * c, rho * s, np.full_like(c, t)], axis=1) for rho, t in rings])


_POST_SURFACE = _post_surface_points()


def _dist_to_post(points, xy):
    """Distance from each world point to a solid, flat-topped post (0 inside it)."""
    dr = np.maximum(np.hypot(points[:, 0] - xy[0], points[:, 1] - xy[1]) - POST_R, 0.0)
    dz = np.maximum(np.maximum(points[:, 2] - (FLOOR + POST_H), FLOOR - points[:, 2]), 0.0)
    return np.hypot(dr, dz)


def _dist_to_pot(points, state, name):
    """Distance from each world point to the solid pot (0 inside it)."""
    v = (points - np.asarray(state.pos, dtype=float)[:3]) @ _rotation(state)  # in the pot's own frame
    dr = np.maximum(np.hypot(v[:, 0], v[:, 1]) - POT_R[name], 0.0)
    da = np.maximum(np.abs(v[:, 2]) - POT_H[name] / 2, 0.0)
    return np.hypot(dr, da)


def _pot_world_surface(state, name):
    return np.asarray(state.pos, dtype=float)[:3] + _SURFACE[name] @ _rotation(state).T


def _boxes_apart(objects, a, b) -> bool:
    """Outlines (from each pot's pose) more than 2 mm apart along some axis: the shapes cannot touch."""
    (la, ha), (lb, hb) = _extent(objects[a], a), _extent(objects[b], b)
    return any(la[k] > hb[k] + TOUCH or lb[k] > ha[k] + TOUCH for k in range(3))


def _surface_gap(objects, a, b) -> float:
    """Shortest distance between two pots' actual surfaces, whatever their tilt (0 if they overlap)."""
    sa, sb = objects[a], objects[b]
    return min(float(_dist_to_pot(_pot_world_surface(sa, a), sb, b).min()),
               float(_dist_to_pot(_pot_world_surface(sb, b), sa, a).min()))


def _contact_pot_pot(objects, a, b) -> bool:
    if _boxes_apart(objects, a, b):
        return False
    return _surface_gap(objects, a, b) < TOUCH


def _contact_pot_post(objects, p) -> bool:
    s = objects[p]
    lo, hi = _extent(s, p)
    pot_pts = None
    for x, y in POST_XY.values():
        if (lo[0] > x + POST_R + TOUCH or hi[0] < x - POST_R - TOUCH or lo[1] > y + POST_R + TOUCH
                or hi[1] < y - POST_R - TOUCH or lo[2] > FLOOR + POST_H + TOUCH):
            continue  # outlines more than 2 mm apart
        if pot_pts is None:
            pot_pts = _pot_world_surface(s, p)
        post_pts = _POST_SURFACE + np.array([x, y, FLOOR])
        gap = min(float(_dist_to_post(pot_pts, (x, y)).min()), float(_dist_to_pot(post_pts, s, p).min()))
        if gap < TOUCH:
            return True
    return False


def _contact_pot_wall(state, name) -> bool:
    """Closer than 2 mm to a wall: to its inner face (from the pot's outermost point), or, for a pot above the
    walls, to their top edge (from the pot's bottom)."""
    side = max(_wall_gap(state, name), 0.0)  # 0 once the pot reaches over the wall
    above = max(float(_extent(state, name)[0][2]) - WALL_TOP, 0.0)
    return math.hypot(side, above) < TOUCH


def _touching(objects) -> bool:
    """Glaze contact: a pot in the kiln closer than 2 mm, surface to surface, to another pot, a post or a wall."""
    for i, a in enumerate(POT_NAMES):
        sa = objects[a]
        if _in_kiln(sa) and (_contact_pot_post(objects, a) or _contact_pot_wall(sa, a)):
            return True
        for b in POT_NAMES[i + 1:]:
            if (_in_kiln(sa) or _in_kiln(objects[b])) and _contact_pot_pot(objects, a, b):
                return True
    return False


def _upright(state) -> bool:
    return _tilt_deg(state) <= MAX_TILT_DEG


def _pot_gap(objects, a, b) -> float:
    """Gap between two pots: centre distance minus both radii when both are upright; otherwise the shortest
    distance between their actual surfaces (a lying pot's end, side or rim)."""
    if _upright(objects[a]) and _upright(objects[b]):
        return _xy_gap(objects[a].pos, POT_R[a], objects[b].pos, POT_R[b])
    return _surface_gap(objects, a, b)


def _loaded(objects, p) -> bool:
    """Upright on the shelf, 15 mm clear of the walls, 20 mm clear of every post and every pot at its level."""
    s = objects[p]
    bottom = float(_extent(s, p)[0][2])  # pos z - (H/2 cos t + R sin t), from the pose
    return bool(
        _upright(s)
        and abs(bottom - FLOOR) <= FLOOR_TOL
        and _wall_gap(s, p) >= WALL_GAP - EPS
        and min(_post_gaps(objects, p)) >= POST_GAP - EPS
        and all(_pot_gap(objects, p, q) >= POT_GAP - EPS
                for q in POT_NAMES if q != p and _same_level(objects, p, q))
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each pot loaded cleanly (upright either end up, on the shelf,
    at least 15 mm from every wall, at least 20 mm from every post and every other pot at its level). 0 overall if
    any pot in the kiln touches (closer than 2 mm, between the actual surfaces at any tilt) another pot, a post (its
    side or flat top) or a wall (its inner face or top edge)."""
    if _touching(objects):
        return 0.0
    return round(POT_CREDIT * sum(_loaded(objects, p) for p in POT_NAMES), 6)


def success(objects):
    """All five pots loaded cleanly and still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[p].resting() for p in POT_NAMES)


# The demo: the left arm loads green (front left) then blue (back left) and parks; then the right arm loads yellow
# (front right), red (back right) and purple (front centre). Each grip is turned so the opening fingers point into
# free space: across the kiln (world y) for the four side pots, front to back for purple, whose side neighbours are
# taller than its fingers are high.
TARGET = {
    "pot_green": (0.200, 0.100),  # 25 mm to the front wall, 29 mm to the front-left post, 30 mm to blue
    "pot_blue": (0.295, 0.095),  # 25 mm to the back wall
    "pot_yellow": (0.205, -0.100),  # 35 mm to the right wall, 35 mm to red
    "pot_red": (0.295, -0.095),  # 30 mm to the back wall
    "pot_purple": (0.205, 0.000),  # 40 mm to green, 45 mm to yellow
}
GRIP = {"pot_green": 0.050, "pot_blue": 0.045, "pot_yellow": 0.050, "pot_red": 0.045, "pot_purple": 0.038}
# TCP above each pot's bottom: the pads cover 20 to 33 mm of its side, and at release the palm (50 mm above the TCP)
# clears the 10 cm posts for every pot but purple, which is placed well away from them.
PLACE_YAW = {"pot_green": 0.0, "pot_blue": 0.0, "pot_yellow": 0.0, "pot_red": 0.0, "pot_purple": math.pi / 2}
TRAVEL_Z = TOP + 0.165  # a carried pot's bottom clears the posts' tops by at least 1 cm
DROP = 0.008  # each pot is let go this far above the shelf
LEFT_POTS, RIGHT_POTS = ("pot_green", "pot_blue"), ("pot_yellow", "pot_red", "pot_purple")
LEFT_PARK = (0.20, 0.32, TRAVEL_Z)  # over green's empty start mark
RIGHT_PARK = (0.17, -0.32, TRAVEL_Z)  # over yellow's empty start mark
RIGHT_WAIT_S = 21.0  # the right arm waits above yellow until the left arm has parked (its plan takes 20.7 s)


def _load(names):
    plan = []
    for n in names:
        (sx, sy), (tx, ty) = START[n], TARGET[n]
        plan += pick_place_waypoints(
            (sx, sy, TOP + GRIP[n]), (tx, ty, FLOOR + GRIP[n] + DROP), travel_z=TRAVEL_Z, yaw=0.0,
            place_yaw=PLACE_YAW[n],
        )
    return plan


def demo():
    left = _load(LEFT_POTS) + [Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park")]
    right = [Waypoint(RIGHT_WAIT_S, (*START["pot_yellow"], TRAVEL_Z), FINGER_OPEN, "wait for the left arm")]
    right += _load(RIGHT_POTS) + [Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park")]
    return {"left": left, "right": right}
