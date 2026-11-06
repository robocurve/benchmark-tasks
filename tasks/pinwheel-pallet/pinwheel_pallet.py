"""On the station table: four identical corrugated cartons (100 x 50 x 40 mm), lying flat with their 100 mm sides
pointing away from the arms, two in front of each arm. A fixed blue plastic pallet (170 x 170 mm, 20 mm tall) stands on
the midline between the arms. A carton fills a pallet corner when it lies flat on the pallet (a 100 x 50 mm face down),
square to the pallet, with its two outer sides each at most 15 mm inside the pallet's edges (3 mm past is allowed).
Two cartons that share a pallet edge can't both lie along it (2 x 100 mm is more than 170 mm), so four filled corners
make a pinwheel: two cartons turned 90 degrees from the other two, each one's end next to the next one's side. Set
4 mm inside the edges, the cartons have 12 mm between them along each edge, room for a finger.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CARTON_SIZE = (0.100, 0.050, 0.040)  # long side, short side, height
KRAFT = (0.66, 0.50, 0.33)
STARTS = {  # carton name: its centre on the table (x, y), the 100 mm side along x
    "carton_1": (0.18, 0.25),
    "carton_2": (0.33, 0.25),
    "carton_3": (0.18, -0.25),
    "carton_4": (0.33, -0.25),
}
CARTONS = [Box(name, pos=(x, y, TOP + CARTON_SIZE[2] / 2 + 0.001), size=CARTON_SIZE, mass=0.15, color=KRAFT)
           for name, (x, y) in STARTS.items()]

PALLET_XY = (0.30, 0.0)
# WHY: 100 + 50 mm along each edge in a pinwheel, plus 20 mm of play for the fingers between cartons.
PALLET_SIDE = 0.170
PALLET_H = 0.020
PALLET_TOP = TOP + PALLET_H
PALLET = Box("pallet", pos=(*PALLET_XY, TOP + PALLET_H / 2), size=(PALLET_SIDE, PALLET_SIDE, PALLET_H),
             color=(0.15, 0.32, 0.65))  # WHY: saturated blue; a pale pallet vanishes into the table on the top camera
OBJECTS = [*CARTONS, PALLET]

# WHY: a pinwheel packed tight, the cartons touching and centred, sits 10 mm inside every edge. 15 mm lets it sit up to
# 5 mm off centre and still count; a larger limit would not undo the pinwheel (two cartons along one edge only overlap
# more the further inside they sit).
FLUSH_MM = 15.0  # an outer side at most this far inside the pallet's edge
OVERHANG_MM = 3.0  # or at most this far past it
MAX_SQUARE_DEG = 5.0
MAX_TILT_DEG = 5.0
# WHY: 1 mm, not more: a carton set down reads 0.0 mm (traced on the official stack, 2026-11-03), and a gripper can
# hold one still a few mm up, which the hooks can't tell from lying on the pallet.
ON_PALLET_MM = 1.0  # the carton's bottom within this of the pallet's top
CORNERS = [(sx, sy) for sx in (-1, 1) for sy in (-1, 1)]  # the pallet's corners, as signs of x and y from its centre


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(carton) -> float:
    """How far the carton's 40 mm axis is from vertical (deg), either way up."""
    return math.degrees(math.acos(min(1.0, abs(_rotation(carton.quat_xyzw)[2, 2]))))


def _long_axis_yaw(carton) -> float:
    """Direction of the carton's 100 mm axis in the table plane (rad)."""
    r = _rotation(carton.quat_xyzw)
    return math.atan2(r[1, 0], r[0, 0])


def _square_deg(carton) -> float:
    """How far the carton's sides are from parallel to the pallet's edges (deg)."""
    d = math.degrees(_long_axis_yaw(carton)) % 90.0
    return min(d, 90.0 - d)


def _footprint_xy(carton) -> np.ndarray:
    """The four corners of the carton's 100 x 50 mm footprint, world x and y (m)."""
    yaw = _long_axis_yaw(carton)
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.array([[i, j] for i in (-1, 1) for j in (-1, 1)]) * np.array(CARTON_SIZE[:2]) / 2
    return local @ np.array([[c, -s], [s, c]]).T + np.asarray(carton.pos[:2], dtype=float)


def _insets_mm(carton, corner) -> tuple[float, float]:
    """How far the carton's outermost point toward a pallet corner is inside the pallet's x edge and its y edge there
    (mm); negative past the edge. From the carton's bounding box, which is exact for a box, so a tilted carton's
    raised corner counts too."""
    half, (sx, sy) = PALLET_SIDE / 2, corner
    reach = [float(carton.high[i]) - PALLET_XY[i] if s > 0 else PALLET_XY[i] - float(carton.low[i])
             for i, s in enumerate((sx, sy))]
    return 1000.0 * (half - reach[0]), 1000.0 * (half - reach[1])


def _fills(carton, corner) -> bool:
    return (
        all(-OVERHANG_MM <= g <= FLUSH_MM for g in _insets_mm(carton, corner))
        and _square_deg(carton) <= MAX_SQUARE_DEG
        and _tilt_deg(carton) <= MAX_TILT_DEG
        # on the pallet, not on another carton or held above it (low is exact for a box)
        and abs(float(carton.low[2]) - PALLET_TOP) <= ON_PALLET_MM / 1000
    )


def _filled(objects) -> int:
    """How many pallet corners have a carton filling them. Two cartons can't fill one corner without overlapping, and
    one can't fill two (it would need to span 150 mm), so this counts cartons too."""
    cartons = [objects[c.name] for c in CARTONS]
    return sum(any(_fills(c, corner) for c in cartons) for corner in CORNERS)


def success(objects):
    """All four corners filled, every carton still; holds 1 s."""
    return _filled(objects) == 4 and all(objects[c.name].resting() for c in CARTONS)


def distance(objects):
    """Mean over the pallet's corners of how far the nearest carton corner is from it (m)."""
    half = PALLET_SIDE / 2
    feet = np.vstack([_footprint_xy(objects[c.name]) for c in CARTONS])
    return float(np.mean([np.linalg.norm(feet - (np.array(PALLET_XY) + half * np.array(k)), axis=1).min()
                          for k in CORNERS]))


def score(objects):
    """Partial credit (success scores 1.0): 0.25 for each filled corner."""
    return 0.25 * _filled(objects)


TRAVEL_Z = TOP + 0.13
INSET = 0.004  # aim each outer side this far inside the pallet edge
PLACE_CLEAR = 0.003  # let go with the carton this far above the pallet
# WHY: each arm turns the fingers 90 degrees its own way; the other way needs a wrist joint past its limit (checked
# with ArmPlan: 7.5 degrees of IK error for the left arm turning +90, 18 for the right turning -90).
TURN = {"left": -math.pi / 2, "right": math.pi / 2}
NEAR, FAR, LEFT, RIGHT = -1, 1, 1, -1
# WHY: in the air a gripped carton hangs 22 mm off the TCP along the fingers' closing axis, toward the gripper's -y
# (world -y with the fingers unturned), on both arms; it centres again when it touches down (trace, 2026-11-03). A
# carton lowered with that offset toward a neighbour lands on it: the first demo's last carton rolled onto its side
# on the one beside it. So the cartons go in the order far left, near left, near right, far right. Each but the last
# goes down before the one whose end will come next to its side, and the last one's offset points away from its
# neighbour.
# WHY: the last carton's inner side ends 12 mm from its neighbour, so it is gripped 25 mm off centre, toward the
# pallet's open middle, where nothing stands next to the finger on that side.
LAST_GRIP_SHIFT = -0.025
# WHY: where the first demo's cartons came down off their aim (trace, 2026-11-03): the left arm 6.9 mm short in x at
# the far left corner, the right arm 4.3 mm past in y at the far right one. Both come from the sim's in-hand drift
# (above), so these offsets only make the scripted demo land; the scorer doesn't know them. Each aim stays inside the
# scoring limits on its own (2 mm and 0 mm past the edge).
AIM = {(FAR, LEFT): (0.006, 0.0), (FAR, RIGHT): (0.0, 0.004)}


def _home(corner, along_x):
    """The centre of a carton filling ``corner`` with each outer side INSET inside the edge, its 100 mm side along x
    (``along_x``) or y."""
    (sx, sy), half = corner, PALLET_SIDE / 2
    lx, ly = (CARTON_SIZE[0], CARTON_SIZE[1]) if along_x else (CARTON_SIZE[1], CARTON_SIZE[0])
    return PALLET_XY[0] + sx * (half - INSET - lx / 2), PALLET_XY[1] + sy * (half - INSET - ly / 2)


def _move(arm, carton, corner, along_x, shift=0.0):
    """Pick ``carton`` at its centre (``shift`` m further along x), put it in ``corner``; the fingers turn 90 degrees
    on the way for a carton that ends along y."""
    x, y = STARTS[carton]
    hx, hy = _home(corner, along_x)
    ax, ay = AIM.get(corner, (0.0, 0.0))
    pick = (x + shift, y, TOP + 0.001 + CARTON_SIZE[2] / 2)
    place = (hx + shift + ax, hy + ay, PALLET_TOP + PLACE_CLEAR + CARTON_SIZE[2] / 2)
    return pick_place_waypoints(pick, place, travel_z=TRAVEL_Z, place_yaw=0.0 if along_x else TURN[arm])


def demo():
    """A pinwheel. The left arm puts one carton in its far corner turned 90 degrees (along y), then one in its near
    corner along x, and moves back over its side. The right arm then puts one in its near corner turned 90 degrees and
    the last in its far corner along x, gripped off centre (see LAST_GRIP_SHIFT). The fingers close across each
    carton's 50 mm width."""
    left = _move("left", "carton_2", (FAR, LEFT), False) + _move("left", "carton_1", (NEAR, LEFT), True)
    left.append(Waypoint(1.5, (0.25, 0.25, TRAVEL_Z), FINGER_OPEN, "back over its side"))
    wait_s = sum(w.duration for w in left)
    right = [Waypoint(wait_s, (*STARTS["carton_3"], TRAVEL_Z), FINGER_OPEN, "wait for the left arm")]
    right += _move("right", "carton_3", (NEAR, RIGHT), False)
    right += _move("right", "carton_4", (FAR, RIGHT), True, shift=LAST_GRIP_SHIFT)
    return {"left": left, "right": right}
