"""On the station table: a low fixed counter block, 30 cm deep, 50 cm wide and 30 mm tall, its far edge 45 cm in
front of the arm bases. Five loose kitchen items stand on it near that far edge: a blue mug, a black knife lying
in its sheath, an orange medicine bottle, a green spray-cleaner box and a grey frying pan.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
COUNTER_X = (0.15, 0.45)  # near edge, far edge
COUNTER_HALF_Y = 0.25
COUNTER_H = 0.030
CT = TOP + COUNTER_H  # counter top
REACH = 0.15  # a toddler reaches 15 cm in from the far edge
SAFE_X = COUNTER_X[1] - REACH  # 0.30: every part of an item must be at x <= 0.30

WOOD, WHITE = (0.80, 0.70, 0.55), (0.93, 0.93, 0.90)

COUNTER = Box("counter", pos=((COUNTER_X[0] + COUNTER_X[1]) / 2, 0.0, TOP + COUNTER_H / 2),
              size=(COUNTER_X[1] - COUNTER_X[0], 2 * COUNTER_HALF_Y, COUNTER_H), color=WOOD)

MUG = Cylinder("mug", pos=(0.38, 0.12, CT + 0.045), radius=0.035, height=0.090, mass=0.20, color=(0.15, 0.35, 0.80))
KNIFE = Box("knife", pos=(0.36, 0.21, CT + 0.012), size=(0.140, 0.024, 0.024), mass=0.08, color=(0.10, 0.10, 0.10))
BOTTLE = Cylinder("medicine_bottle", pos=(0.345, 0.035, CT + 0.035), radius=0.020, height=0.070, mass=0.06,
                  color=(0.90, 0.50, 0.10))
SPRAY = Box("spray_box", pos=(0.36, -0.07, CT + 0.045), size=(0.050, 0.050, 0.090), mass=0.15,
            color=(0.20, 0.70, 0.30))
PAN = Cylinder("pan", pos=(0.35, -0.19, CT + 0.0225), radius=0.050, height=0.045, mass=0.40,
               color=(0.45, 0.45, 0.48))

ITEMS = [MUG, KNIFE, BOTTLE, SPRAY, PAN]
ITEM_NAMES = tuple(o.name for o in ITEMS)
OBJECTS = [COUNTER, *ITEMS]

BOTTOM_TOL = 0.001  # an item's lowest point within 1 mm of the counter top: resting on it, not held or stacked
FALLEN = 0.010  # an item whose lowest point is over 10 mm below the counter top has left it
FLAT_DEG = 5.0  # resting flat: a box face, or a cylinder's end or side, within 5 degrees of the counter
RESET_TOL = 0.0005  # every item within 0.5 mm of its start, at rest: a new trial
SHAPES = {o.name: o for o in ITEMS}
_START = {o.name: np.asarray(o.pos, float) for o in ITEMS}


def _rot(st) -> np.ndarray:
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def reach(name, st, d) -> float:
    """How far the item extends along the world unit vector d: the largest d . p over every point p of its
    actual shape (a box's corners, a cylinder's rims), from its centre, orientation and dimensions."""
    shape, r, d = SHAPES[name], _rot(st), np.asarray(d, float)
    c = float(np.asarray(st.pos, float) @ d)
    if isinstance(shape, Box):
        return c + float(np.abs(r.T @ d) @ (np.asarray(shape.size) / 2))
    a = float(r[:, 2] @ d)  # cylinder axis component along d
    return c + shape.height / 2 * abs(a) + shape.radius * float(np.sqrt(max(0.0, 1.0 - a * a)))


def _bottom(name, st) -> float:
    return -reach(name, st, (0.0, 0.0, -1.0))


def _flat(name, st) -> bool:
    """Lying or standing flat on a face: a box with some face normal within 5 degrees of vertical; a cylinder
    with its axis within 5 degrees of vertical (on its end) or of horizontal (on its side)."""
    zc = np.abs(_rot(st)[2, :])  # vertical component of each body axis
    lim = np.cos(np.radians(FLAT_DEG))
    if isinstance(SHAPES[name], Box):
        return bool(zc.max() >= lim)
    return bool(zc[2] >= lim or zc[2] <= np.sin(np.radians(FLAT_DEG)))


def _off_counter(name, st) -> bool:
    """Left the counter: lowest point more than 10 mm below the counter top, or centre outside its footprint."""
    c = st.pos
    return bool(
        _bottom(name, st) < CT - FALLEN
        or not (COUNTER_X[0] < c[0] < COUNTER_X[1] and abs(c[1]) < COUNTER_HALF_Y)
    )


def _safe(name, st) -> bool:
    """Every part at least 15 cm from the far edge and over the counter, lying flat directly on it, at rest."""
    return bool(
        reach(name, st, (1.0, 0.0, 0.0)) <= SAFE_X + 1e-9
        and -reach(name, st, (-1.0, 0.0, 0.0)) >= COUNTER_X[0] - 1e-9
        and -reach(name, st, (0.0, -1.0, 0.0)) >= -COUNTER_HALF_Y - 1e-9
        and reach(name, st, (0.0, 1.0, 0.0)) <= COUNTER_HALF_Y + 1e-9
        and abs(_bottom(name, st) - CT) <= BOTTOM_TOL + 1e-9
        and _flat(name, st)
        and st.resting()
    )


# Trial history: whether any item has left the counter since the trial started. The scorer sees one step at a
# time, so it is kept here and cleared only when every item is back within 0.5 mm of its start and at rest.
_HISTORY = {"left_counter": False}


def _update(objects) -> None:
    if all(np.linalg.norm(np.asarray(objects[n].pos) - p) < RESET_TOL and objects[n].resting()
           for n, p in _START.items()):
        _HISTORY["left_counter"] = False
    if any(_off_counter(n, objects[n]) for n in ITEM_NAMES):
        _HISTORY["left_counter"] = True


def score(objects):
    """0.2 per item that is safe; 0 if any item has left the counter at any time during the trial."""
    _update(objects)
    if _HISTORY["left_counter"]:
        return 0.0
    return round(sum(_safe(n, objects[n]) for n in ITEM_NAMES) / len(ITEM_NAMES), 6)


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """Metres the furthest item still reaches past the 15 cm line."""
    return float(max(0.0, max(reach(n, objects[n], (1.0, 0.0, 0.0)) - SAFE_X for n in ITEM_NAMES)))


# Demo. The left arm picks the knife (turning it to lie left to right), the mug and the medicine bottle; the
# right arm picks the spray box and then drags the pan back with its closed fingers against the pan's far side.
TRAVEL_Z = CT + 0.12  # finger tips 17 mm above the tallest item (90 mm)
KNIFE_TO = (0.175, 0.13)  # lies left to right: x 0.163 to 0.187, y 0.06 to 0.20
MUG_TO = (0.25, 0.195)
BOTTLE_TO = (0.25, 0.07)
SPRAY_TO = (0.24, -0.07)
PAN_TO_X = 0.22  # aimed pan centre; in the L40S trace its far side stopped at x 0.289, 11 mm inside the line
RELEASE_UP = 0.007  # released 7 mm above where it was gripped: it drops about 3 mm once the arm's ~4 mm sag is
# taken off, so it is never standing on the counter while still in the fingers
FINGER_T = 0.008  # assumed half-thickness of the closed fingers along x


def _grip_z(item, height):
    """TCP height for a grip near mid-height of an item standing on the counter, tips at least 5 mm above it."""
    return max(CT + height / 2, CT + 0.005 + 0.013)


def _pick(item, height, to, yaw=0.0, place_yaw=None, finger=FINGER_OPEN):
    z = _grip_z(item, height)
    w = pick_place_waypoints((item.pos[0], item.pos[1], z), (*to, z + RELEASE_UP), travel_z=TRAVEL_Z, yaw=yaw,
                             place_yaw=place_yaw)
    return [Waypoint(x.duration, x.pos, finger if x.finger == FINGER_OPEN else x.finger, x.label, x.yaw) for x in w]


def _drag_pan():
    px, py, _ = PAN.pos
    behind = px + PAN.radius + FINGER_T + 0.006
    end = PAN_TO_X + PAN.radius + FINGER_T
    z = CT + 0.020 + 0.013  # finger tips 20 mm above the counter, against the pan's 45 mm side
    return [
        Waypoint(1.8, (behind, py, TRAVEL_Z), FINGER_CLOSED, "over the pan's far side"),
        Waypoint(1.5, (behind, py, z), FINGER_CLOSED, "down behind the pan"),
        Waypoint(3.5, (end, py, z), FINGER_CLOSED, "drag back"),
        Waypoint(0.8, (end + 0.02, py, z), FINGER_CLOSED, "off the pan"),
        Waypoint(1.2, (end + 0.02, py, TRAVEL_Z), FINGER_CLOSED, "up"),
    ]


def demo():
    left = (
        _pick(KNIFE, KNIFE.size[2], KNIFE_TO, yaw=0.0, place_yaw=-np.pi / 2, finger=-0.025)
        + _pick(MUG, MUG.height, MUG_TO)
        + _pick(BOTTLE, BOTTLE.height, BOTTLE_TO, finger=-0.030)
    )
    right = _pick(SPRAY, SPRAY.size[2], SPRAY_TO, finger=-0.035) + _drag_pan()
    return {"left": left, "right": right}
