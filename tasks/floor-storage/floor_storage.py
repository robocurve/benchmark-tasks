"""On the station table: a 1:50 grey concrete floor slab, fixed and 4 mm thick, from 8 to 40 cm out from the arm
bases and 90 cm wide. A 50 mm square hoistway (the shaft of a material hoist), 3 mm deep and black, is cut through it
21.5 to 26.5 cm out, just left of the midline. The slab's far edge is the building's outside edge: a fixed concrete wall, 61 mm tall above the
slab, runs along it right of the midline, and a fixed parapet, 12 mm tall, left of it. Three bundles of building
material (24 x 30 x 18 mm: tan lumber, white drywall and pink insulation) lie on the slab: the lumber and the
insulation beside the hoistway, the drywall beside the parapet.
"""

import itertools
import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SCALE = 50
FT = 0.3048 / SCALE  # one foot at scale (m)

CONCRETE, DARK = (0.62, 0.62, 0.60), (0.05, 0.05, 0.05)
SLAB_T = 0.004
SLAB_TOP = TOP + SLAB_T
SLAB_X, SLAB_Y = (0.08, 0.40), (-0.45, 0.45)  # the slab's near and far edges, and its sides
HOIST_X, HOIST_Y = (0.215, 0.265), (0.095, 0.145)  # the hoistway, 2.5 m square


def _slab(name, x, y):
    return Box(name, pos=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, TOP + SLAB_T / 2),
               size=(x[1] - x[0], y[1] - y[0], SLAB_T), color=CONCRETE)


# WHY: the table can't have a hole, so the slab is four pieces around the hoistway, and the hoistway's floor is a black
# plate 1 mm thick: a recess 3 mm deep.
SLAB = [
    _slab("slab_near", (SLAB_X[0], HOIST_X[0]), SLAB_Y),
    _slab("slab_far", (HOIST_X[1], SLAB_X[1]), SLAB_Y),
    _slab("slab_left", HOIST_X, (HOIST_Y[1], SLAB_Y[1])),
    _slab("slab_right", HOIST_X, (SLAB_Y[0], HOIST_Y[0])),
    Box("hoistway", pos=(sum(HOIST_X) / 2, sum(HOIST_Y) / 2, TOP + 0.0005),
        size=(HOIST_X[1] - HOIST_X[0], HOIST_Y[1] - HOIST_Y[0], 0.001), color=DARK),
]

WALL_T = 0.012
WALL_HEIGHTS = {"full_wall": 0.061, "parapet": 0.012}  # above the slab: a 10 ft storey, a 2 ft parapet
WALL_Y = {"full_wall": (SLAB_Y[0], 0.0), "parapet": (0.0, SLAB_Y[1])}
WALLS = [Box(n, pos=(SLAB_X[1] + WALL_T / 2, sum(WALL_Y[n]) / 2, TOP + (SLAB_T + h) / 2),
             size=(WALL_T, WALL_Y[n][1] - WALL_Y[n][0], SLAB_T + h), color=CONCRETE)
         for n, h in WALL_HEIGHTS.items()]

BUNDLE = (0.024, 0.030, 0.018)  # deep, wide, tall: 1.2 x 1.5 x 0.9 m at scale
START = {"lumber": (0.24, 0.070), "drywall": (0.380, 0.200), "insulation": (0.292, 0.120)}
COLORS = {"lumber": (0.76, 0.60, 0.38), "drywall": (0.92, 0.92, 0.88), "insulation": (0.95, 0.60, 0.70)}
BUNDLES = [Box(n, pos=(x, y, SLAB_TOP + BUNDLE[2] / 2 + 0.001), size=BUNDLE, mass=0.06, color=COLORS[n])
           for n, (x, y) in START.items()]
OBJECTS = [*SLAB, *WALLS, *BUNDLES]

# 29 CFR 1926.250(b)(1): "Material stored inside buildings under construction shall not be placed within 6 feet of
# any hoistway or inside floor openings, nor within 10 feet of an exterior wall which does not extend above the top of
# the material stored."
HOISTWAY_FT = 6.0
WALL_FT = 10.0
ON_MM = 2.0  # a bundle's bottom within this of what it stands on
FLAT_DEG = 10.0


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _outline(bundle) -> list[np.ndarray]:
    """The bundle's outline seen from above: the convex hull of its corners, counter-clockwise."""
    r, c = _rotation(bundle.quat_xyzw), np.asarray(bundle.pos, dtype=float)
    pts = [c[:2] + (r @ (np.array(s) * BUNDLE / 2))[:2] for s in itertools.product((-1, 1), repeat=3)]
    pts.sort(key=lambda p: (p[0], p[1]))

    def half(points):  # monotone chain
        out = []
        for p in points:
            while len(out) >= 2 and _cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return half(pts) + half(pts[::-1])


def _cross(o, a, b) -> float:
    return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))


def _rect(x, y) -> list[np.ndarray]:
    return [np.array(p, dtype=float) for p in ((x[0], y[0]), (x[1], y[0]), (x[1], y[1]), (x[0], y[1]))]


def _inside(p, poly) -> bool:
    return all(_cross(poly[i], poly[(i + 1) % len(poly)], p) >= 0 for i in range(len(poly)))


def _to_segment(p, a, b) -> float:
    ab = b - a
    t = min(1.0, max(0.0, float((p - a) @ ab / (ab @ ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


def _gap(p, q) -> float:
    """The shortest distance between two convex polygons; 0 if they overlap."""
    edges = lambda poly: [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]  # noqa: E731
    if (any(_inside(v, q) for v in p) or any(_inside(v, p) for v in q)
            or any(_crosses(a, b, c, d) for a, b in edges(p) for c, d in edges(q))):
        return 0.0
    return min([_to_segment(v, a, b) for v in p for a, b in edges(q)]
               + [_to_segment(v, a, b) for v in q for a, b in edges(p)])


def _crosses(a, b, c, d) -> bool:
    return (_cross(a, b, c) * _cross(a, b, d) < 0) and (_cross(c, d, a) * _cross(c, d, b) < 0)


def _flat(bundle) -> bool:
    """Lying on any of its faces, within FLAT_DEG."""
    up = _rotation(bundle.quat_xyzw)[2]  # world z of each of the bundle's axes
    return float(np.max(np.abs(up))) >= math.cos(math.radians(FLAT_DEG))


def _on_floor(objects, name, below=()) -> bool:
    """Standing on the slab, or on a bundle that is itself on the floor, with its centre over the slab: not in the
    hoistway, not off the slab, not held up."""
    b = objects[name]
    x, y, bottom = float(b.pos[0]), float(b.pos[1]), float(b.low[2])
    if not (SLAB_X[0] <= x <= SLAB_X[1] and SLAB_Y[0] <= y <= SLAB_Y[1]):
        return False
    if abs(bottom - SLAB_TOP) <= ON_MM / 1000:
        return True
    for other in BUNDLES:
        o = objects[other.name]
        if (other.name != name and other.name not in below and abs(bottom - float(o.high[2])) <= ON_MM / 1000
                and float(o.low[0]) <= x <= float(o.high[0]) and float(o.low[1]) <= y <= float(o.high[1])):
            return _on_floor(objects, other.name, (*below, name))
    return False


def stack_top(objects, name) -> float:
    """The top of the material stored with this bundle: the highest top among the bundles stacked with it (those whose
    outlines overlap it, directly or through others)."""
    group, grew = {name}, True
    while grew:
        grew = False
        for b in BUNDLES:
            if b.name not in group and any(_gap(_outline(objects[b.name]), _outline(objects[g])) == 0 for g in group):
                group.add(b.name)
                grew = True
    return max(float(objects[g].high[2]) for g in group)


def hoistway_ft(bundle) -> float:
    """How far the bundle's outline is from the hoistway's edge, in feet at scale."""
    return _gap(_outline(bundle), _rect(HOIST_X, HOIST_Y)) / FT


def walls_in_force(objects, name) -> list[str]:
    """The outside walls that don't extend above the top of the material stored with this bundle."""
    return [n for n, h in WALL_HEIGHTS.items() if SLAB_TOP + h <= stack_top(objects, name)]


def wall_ft(bundle, wall) -> float:
    """How far the bundle's outline is from the wall, in feet at scale."""
    return _gap(_outline(bundle), _rect((SLAB_X[1], SLAB_X[1] + WALL_T), WALL_Y[wall])) / FT


def _stored(objects, name) -> bool:
    b = objects[name]
    return (_on_floor(objects, name) and _flat(b) and hoistway_ft(b) >= HOISTWAY_FT
            and all(wall_ft(b, w) >= WALL_FT for w in walls_in_force(objects, name)))


def success(objects):
    """All three bundles stored where the rule allows, and still; holds 1 s."""
    return all(_stored(objects, b.name) and objects[b.name].resting() for b in BUNDLES)


def distance(objects):
    """Bundles not yet stored where the rule allows."""
    return sum(not _stored(objects, b.name) for b in BUNDLES)


def score(objects):
    """Partial credit (success scores 1.0): a third for each bundle stored where the rule allows."""
    return sum(_stored(objects, b.name) for b in BUNDLES) / 3


TRAVEL_Z = TOP + 0.08
GRIP_Z = SLAB_TOP + 0.017  # pad tips 13 mm below the TCP: 4 mm above the slab, gripping the 18 mm bundle
PLACE_Z = SLAB_TOP + 0.019  # set down from 2 mm above the slab
PLACE = {"drywall": (0.30, 0.26), "lumber": (0.17, 0.25), "insulation": (0.24, 0.24)}


def _move(name):
    (px, py), (qx, qy) = START[name], PLACE[name]
    return pick_place_waypoints((px, py, GRIP_Z), (qx, qy, PLACE_Z), travel_z=TRAVEL_Z)


def demo():
    """The left arm moves the drywall back from the parapet, then the lumber and the insulation away from the
    hoistway."""
    left = [*_move("drywall"), *_move("lumber"), *_move("insulation")]
    left.append(Waypoint(1.0, (0.20, 0.20, TRAVEL_Z), FINGER_OPEN, "clear"))
    return {"left": left}
