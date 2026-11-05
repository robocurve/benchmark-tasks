"""On the station table, in front of the left arm (positions in cm from the midpoint between the arm bases, x forward,
y to the robot's left): a black photo-eye on a post (3 x 3 cm, 7 cm tall) fixed at (16, 10), and a red reflector on an
identical post fixed at (42, 30), the two turned to face each other. The photo-eye's beam runs straight from it to the
reflector, 4 cm above the table; nothing on the table marks it. Four blue cartons (6 x 6 cm across, 8 cm tall) stand
upright, square to the table edge, near the line between the posts. The posts are fixed; the cartons are loose.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

EYE_XY, REFLECTOR_XY = (0.16, 0.10), (0.42, 0.30)
BEAM_Z = TOP + 0.04
_BEAM = np.array(REFLECTOR_XY) - np.array(EYE_XY)
BEAM_YAW = math.atan2(_BEAM[1], _BEAM[0])  # 37.6 degrees
ACROSS = np.array([-math.sin(BEAM_YAW), math.cos(BEAM_YAW)])  # unit vector across the beam, to its left
POST = (0.03, 0.03, 0.07)
POSTS = [
    Box("photo_eye", pos=(*EYE_XY, TOP + POST[2] / 2), size=POST, color=(0.08, 0.08, 0.09), yaw=BEAM_YAW),
    Box("reflector", pos=(*REFLECTOR_XY, TOP + POST[2] / 2), size=POST, color=(0.85, 0.10, 0.10), yaw=BEAM_YAW),
]  # fixed: no mass

CARTON, CARTON_KG = (0.06, 0.06, 0.08), 0.15
CARTON_BLUE = (0.12, 0.35, 0.80)
# carton_1 clips the beam by 1 cm near the photo-eye and carton_4 stands across it near the reflector; carton_3, the
# carton nearest the beam's middle, misses it by 2.0 cm, and carton_2 by 3.8 cm. So the blockers are neither the two
# nearest the middle nor the middle two along the beam (review): only judging the line tells them apart.
START = {
    "carton_1": (0.239, 0.121),
    "carton_2": (0.210, 0.239),
    "carton_3": (0.341, 0.161),
    "carton_4": (0.351, 0.266),
}
NAMES = tuple(START)
CARTONS = [Box(name, pos=(*xy, TOP + CARTON[2] / 2 + 0.001), size=CARTON, mass=CARTON_KG, color=CARTON_BLUE)
           for name, xy in START.items()]  # each starts 1 mm above the table, and settles
OBJECTS = [*POSTS, *CARTONS]

CLEAR = 0.01  # a carton blocks the beam if any part of it comes within 1 cm of the beam's line
MAX_MOVE = 0.10  # a moved carton's centre ends at most 10 cm from where it stood
UNTOUCHED = 0.005  # a carton left alone: its centre within 5 mm of where it stood, turned at most 5 degrees
UNTOUCHED_TURN_DEG = 5.0
MAX_TILT_DEG = 10.0  # standing on an end, either way up
TABLE_TOL = 0.003  # on the table: its lowest point within 3 mm of it

# Where a carton is comes from its centre and rotation, not from ``low``/``high``: Arena builds those from the carton's
# corners, turned with it, so they grow when a carton turns about the vertical.


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(carton) -> np.ndarray:
    half = np.array(CARTON) / 2
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return np.asarray(carton.pos) + (signs * half) @ _rot(carton.quat_xyzw).T


def _tilt_deg(carton) -> float:
    """How far the carton's 8 cm axis is from vertical, either way up."""
    return float(np.degrees(np.arccos(min(1.0, abs(_rot(carton.quat_xyzw)[2, 2])))))


def _turn_deg(carton) -> float:
    """How far the carton has turned about the vertical from square to the table (0 to 45 degrees: a square looks the
    same every quarter turn)."""
    r = _rot(carton.quat_xyzw)
    turn = math.degrees(math.atan2(r[1, 0], r[0, 0])) % 90.0
    return min(turn, 90.0 - turn)


def _hull(points) -> np.ndarray:
    """Convex hull of 2D points, anticlockwise (monotone chain)."""
    pts = sorted({(round(float(x), 9), round(float(y), 9)) for x, y in points})

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(pts[::-1])
    return np.array(lower[:-1] + upper[:-1])


def _point_segment(p, a, b) -> float:
    ab = b - a
    t = min(1.0, max(0.0, float(np.dot(p - a, ab) / np.dot(ab, ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


def _crosses(a, b, c, d) -> bool:
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0


def beam_gap(carton) -> float:
    """From the beam's line (eye to reflector) to the nearest point of the carton's outline seen from above (m); 0 if
    the beam passes through it. Infinite if the carton is wholly above or below the beam."""
    corners = _corners(carton)
    if not corners[:, 2].min() < BEAM_Z < corners[:, 2].max():
        return math.inf
    hull = _hull(corners[:, :2])
    a, b = np.array(EYE_XY), np.array(REFLECTOR_XY)
    edges = list(zip(hull, np.roll(hull, -1, axis=0), strict=True))
    if any(_crosses(a, b, p, q) for p, q in edges):
        return 0.0
    inside = all((q[0] - p[0]) * (a[1] - p[1]) - (q[1] - p[1]) * (a[0] - p[0]) >= 0 for p, q in edges)
    if inside:
        return 0.0
    return min([_point_segment(p, a, b) for p in hull] + [_point_segment(e, p, q) for p, q in edges for e in (a, b)])


def blocks(carton) -> bool:
    return beam_gap(carton) < CLEAR


def _moved(carton, name) -> float:
    return float(np.linalg.norm(np.asarray(carton.pos[:2]) - np.array(START[name])))


def _standing(carton) -> bool:
    return _tilt_deg(carton) <= MAX_TILT_DEG and abs(float(_corners(carton)[:, 2].min()) - TOP) <= TABLE_TOL


def _off_table(carton) -> bool:
    return float(_corners(carton)[:, 2].min()) < TOP - 0.01


BLOCKERS = ("carton_1", "carton_4")  # the cartons in the beam at the start (checked in the tests)
BYSTANDERS = ("carton_2", "carton_3")


def _grade(objects) -> float:
    cartons = {n: objects[n] for n in NAMES}
    if any(_off_table(c) or not _standing(c) for c in cartons.values()):
        return 0.0
    # The instruction says to move only the cartons that block the beam, so moving any other one earns nothing.
    if not all(_moved(cartons[n], n) <= UNTOUCHED and _turn_deg(cartons[n]) <= UNTOUCHED_TURN_DEG for n in BYSTANDERS):
        return 0.0
    clear = not any(blocks(c) for c in cartons.values())
    cleared = sum(not blocks(cartons[n]) and _moved(cartons[n], n) <= MAX_MOVE and cartons[n].resting()
                  for n in BLOCKERS)
    still = all(c.resting() for c in cartons.values())
    if clear and still and cleared == len(BLOCKERS):
        return 1.0
    if clear and still:
        return 0.5  # clear, the other two untouched, but a blocking carton moved more than 10 cm
    if cleared >= 1:
        return 0.25
    return 0.0


def success(objects):
    """The beam is clear, both cartons that blocked it moved at most 10 cm, the other two left where they stood, every
    carton standing and still. Must hold for 1 s."""
    return _grade(objects) == 1.0


def score(objects):
    """Partial credit (success scores 1.0), always with the two cartons that didn't block the beam left alone: 0.5 if
    the beam is clear but a blocking carton was moved more than 10 cm; 0.25 if one of the two blocking cartons was
    moved clear (at most 10 cm) and is still. 0 if a carton that didn't block the beam was moved, or any carton ends
    knocked over or off the table."""
    return _grade(objects)


GRIP_ABOVE = 0.045  # grip point above a carton's bottom: 3.5 cm below its top
TRAVEL_Z = TOP + 0.15  # a carried carton's bottom 2.5 cm above every carton and post top
DROP = 0.001  # each carton is let go 1 mm above the table
# Each blocking carton moves straight across the beam, to the side away from its nearer bystander, until it is 3 cm
# clear of the beam (m along ACROSS): 4.0 and 5.7 cm, inside the 10 cm limit. (With the first layout, the official run
# scored 1.0 first time; the arm stopped a few mm short, which only costs clearance.)
DEMO_SHIFT = {"carton_1": -0.040, "carton_4": 0.057}
# With this layout the official run scored 1.0 first time too (2026-11-03, replayed): the blockers moved 36.9 and
# 58.6 mm, ending 26.4 and 31.6 mm clear of the beam; carton_2 and carton_3 never moved.
# Grip by aiming the fingers 52 mm apart, 4 mm inside each face of the 60 mm carton: each finger then presses about
# 20 N, under its 50 N limit, so the pair centres the carton. Fully closed, both push at the limit and a carton can
# slide across the grip (17 mm on the official stack, in the snug-tray task).
GRIP_FINGER = FINGER_OPEN * 52 / 94
RELEASE_FINGER = FINGER_OPEN * 76 / 94  # let go gently: open slowly to 76 mm, 8 mm clear each side of a carton


def _move(pick, place) -> list[Waypoint]:
    """A top-down pick and place: wait over the carton before gripping it, carry it, set it down slowly, open the
    fingers slowly to RELEASE_FINGER, then lift slowly."""
    (px, py, pz), (qx, qy, qz) = pick, place
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "to the object"),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend"),
        Waypoint(0.5, (px, py, pz), FINGER_OPEN, "steady"),
        Waypoint(0.8, (px, py, pz), GRIP_FINGER, "close"),
        Waypoint(1.2, (px, py, TRAVEL_Z), GRIP_FINGER, "lift"),
        Waypoint(1.5, (qx, qy, TRAVEL_Z), GRIP_FINGER, "carry"),
        Waypoint(0.5, (qx, qy, TRAVEL_Z), GRIP_FINGER, "hover"),
        Waypoint(2.0, (qx, qy, qz), GRIP_FINGER, "set down"),
        Waypoint(0.3, (qx, qy, qz), GRIP_FINGER, "hold"),
        Waypoint(1.5, (qx, qy, qz), RELEASE_FINGER, "release"),
        Waypoint(0.3, (qx, qy, qz), RELEASE_FINGER, "settle"),
        Waypoint(1.5, (qx, qy, TRAVEL_Z), RELEASE_FINGER, "retreat"),
    ]


def demo_targets() -> dict[str, tuple[float, float]]:
    """Where the demo sets each blocking carton down: straight across the beam by DEMO_SHIFT."""
    return {n: tuple(np.array(START[n]) + DEMO_SHIFT[n] * ACROSS) for n in DEMO_SHIFT}


def demo():
    """Left arm: carton_1, then carton_4, each gripped across its sides 3.5 cm below its top (the fingers close along
    the table's width, aimed 52 mm apart so the grip centres it), lifted 2.5 cm above every top, carried straight
    across the beam to 3 cm clear of it, set down slowly and let go gently 1 mm above the table. carton_2 and carton_3
    are never touched."""
    waypoints = []
    for name, (tx, ty) in demo_targets().items():
        sx, sy = START[name]
        waypoints += _move((sx, sy, TOP + GRIP_ABOVE), (tx, ty, TOP + DROP + GRIP_ABOVE))
    return {"left": waypoints}
