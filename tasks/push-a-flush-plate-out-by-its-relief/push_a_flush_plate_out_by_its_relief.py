"""On the station table: a dark bench, 16 cm long and 5 cm tall, stands in front of the arms with its long axis along
y. A bright yellow plate (6 x 16 x 1 cm) sits flush in its nest on the bench, level with the two rails along its
long edges, so there is no edge to pinch. Under the plate a 4 cm slot runs along the bench. Behind the bench
(toward the right arm) the slot is open, so a finger can reach in and push the plate along the bench; a small steel
block closes the slot's left end, so no finger fits under that end, and past the front end (toward the left arm) any
part that comes out hangs over the table. A grey open tray stands nearer the arms.
All the bench parts and the tray are fixed; only the plate is loose.

Requires -E object_jitter=0: the nest is sized to the plate to 2 mm.

Run it:  inspect-robots run --embodiment yam_arena -E task=push_a_flush_plate_out_by_its_relief.py -E object_jitter=0 \\
           --instruction "Push the yellow plate along its slot until it overhangs the bench's left end, take it by
           the overhang, and lay it flat in the grey tray." --scorer task_score ...
"""

import os, sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E401,E702

import numpy as np

from belimits import PeakTracker, Rect, bottom, corners, footprint, inside, is_airborne, overlaps, tilt_deg
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SUCCESS_HOLD_S = 10.0  # the plate settles at about 44 s and the hand is clear at 52 s: the run ends at about 54 s on bare plate

SCENE = "station"
TOP = TABLE_TOP[SCENE]

DARK, YELLOW, GREY = (0.14, 0.14, 0.17), (0.97, 0.85, 0.05), (0.62, 0.63, 0.65)

# -- the bench (fixed): two floor strips, a 4 cm slot between them closed at the left end by a block, and a rail along each outer edge ---------------------
BENCH_Y = (-0.08, 0.08)
BENCH_END = BENCH_Y[1]  # the front end, toward the left arm
STRIP_H, RAIL_H = 0.04, 0.05
STRIP_X = ((0.248, 0.260), (0.300, 0.312))
RAIL_X = ((0.232, 0.248), (0.312, 0.328))


def _bar(name, xs, h):
    return Box(name, pos=((xs[0] + xs[1]) / 2, 0.0, TOP + h / 2), size=(xs[1] - xs[0], BENCH_Y[1] - BENCH_Y[0], h), color=DARK)


BENCH = [
    _bar("strip_a", STRIP_X[0], STRIP_H),
    _bar("strip_b", STRIP_X[1], STRIP_H),
    _bar("rail_a", RAIL_X[0], RAIL_H),
    _bar("rail_b", RAIL_X[1], RAIL_H),
    Box("strip_end", pos=(0.28, 0.075, TOP + STRIP_H / 2), size=(0.04, 0.01, STRIP_H), color=DARK),  # closes the slot's left end, top level with the strips
]

# -- the plate (loose): 6 x 16 x 1 cm, 60 g painted softwood, bearing 1.0 cm on each strip, top level with the rails, 2 mm clear of each rail
PLATE_W, PLATE_L, PLATE_T = 0.06, 0.16, 0.01
PLATE = Box("plate", pos=(0.28, 0.0, TOP + STRIP_H + PLATE_T / 2), size=(PLATE_W, PLATE_L, PLATE_T), mass=0.06, color=YELLOW)  # 96 cm3 of softwood at 0.625 g/cm3

# -- the tray (fixed): inner 14 x 20 cm, floor 3 mm, walls 1 cm thick and 2.5 cm tall ---------------------------------
TRAY_IN = Rect((0.07, 0.21), (0.0, 0.20))
TRAY_OUT = TRAY_IN.grown(0.01)
FLOOR_T, WALL_T, WALL_H = 0.003, 0.01, 0.025
TRAY_C = ((TRAY_IN.x0 + TRAY_IN.x1) / 2, (TRAY_IN.y0 + TRAY_IN.y1) / 2)
_wx, _wy = TRAY_IN.x1 - TRAY_IN.x0, TRAY_IN.y1 - TRAY_IN.y0
TRAY = [
    Box("tray_floor", pos=(*TRAY_C, TOP + FLOOR_T / 2), size=(_wx, _wy, FLOOR_T), color=GREY),
    Box("tray_wall_near", pos=(TRAY_IN.x0 - WALL_T / 2, TRAY_C[1], TOP + WALL_H / 2), size=(WALL_T, _wy + 2 * WALL_T, WALL_H), color=GREY),
    Box("tray_wall_far", pos=(TRAY_IN.x1 + WALL_T / 2, TRAY_C[1], TOP + WALL_H / 2), size=(WALL_T, _wy + 2 * WALL_T, WALL_H), color=GREY),
    Box("tray_wall_left", pos=(TRAY_C[0], TRAY_IN.y1 + WALL_T / 2, TOP + WALL_H / 2), size=(_wx, WALL_T, WALL_H), color=GREY),
    Box("tray_wall_right", pos=(TRAY_C[0], TRAY_IN.y0 - WALL_T / 2, TOP + WALL_H / 2), size=(_wx, WALL_T, WALL_H), color=GREY),
]
WALLS = [
    Rect((TRAY_OUT.x0, TRAY_IN.x0), (TRAY_OUT.y0, TRAY_OUT.y1)),
    Rect((TRAY_IN.x1, TRAY_OUT.x1), (TRAY_OUT.y0, TRAY_OUT.y1)),
    Rect((TRAY_IN.x0, TRAY_IN.x1), (TRAY_IN.y1, TRAY_OUT.y1)),
    Rect((TRAY_IN.x0, TRAY_IN.x1), (TRAY_OUT.y0, TRAY_IN.y0)),
]

OBJECTS = [*BENCH, PLATE, *TRAY]

# -- scoring constants ---------------------------------------------------------------------------------------------------
STRIPS = [Rect(xs, BENCH_Y) for xs in STRIP_X] + [Rect((0.26, 0.30), (0.07, 0.08))]  # the two strips and the end block
RAILS = [Rect(xs, BENCH_Y) for xs in RAIL_X]
BENCH_RECT = Rect((RAIL_X[0][0], RAIL_X[1][1]), BENCH_Y)
WINDOW_MIN = 0.04  # m: least overhang past the bench end that leaves an edge to take
MAX_TILT = 5.0  # deg
NEAR = 0.01  # m: "on" a surface, and the height above every support at which a plate counts as held in the air
FLOOR_TOL = 0.003  # m: the plate's bottom this close to the tray floor counts as lying on it
ON_BENCH_TOL = 0.006
TABLE_TOUCH = 0.005  # m: a plate point this low, outside the tray, has been set down on the table
TELEPORT = 0.05  # m: a plate position jump this big between two score() calls is a simulator reset, not motion

# Highest overhang reached while resting on the bench (cleared when the plate is back at its nest and still), and whether
# the plate ever touched the table outside the tray (cleared only by a teleport, which only a simulator reset produces).
_TRK = PeakTracker({"plate": PLATE.pos})
_DROP = {"dropped": False, "last": None}


def _supports_under(s) -> list:
    """Heights of every surface under some part of the plate's outline (the table always)."""
    z = [TOP]
    if overlaps(s, PLATE, TRAY_IN):
        z.append(TOP + FLOOR_T)
    if any(overlaps(s, PLATE, w) for w in WALLS):
        z.append(TOP + WALL_H)
    if any(overlaps(s, PLATE, r) for r in STRIPS):
        z.append(TOP + STRIP_H)
    if any(overlaps(s, PLATE, r) for r in RAILS):
        z.append(TOP + RAIL_H)
    return z


def _airborne(s) -> bool:
    """More than 1 cm above every surface under it: held, or in flight. Counts as not placed."""
    return is_airborne(s, PLATE, _supports_under(s), NEAR)


def _overhang(s) -> float:
    """How far the plate's outline reaches past the bench's front end (m)."""
    return float(footprint(s, PLATE)[:, 1].max() - BENCH_END)


def _flat(s) -> bool:
    """Tilt at most 5 degrees from level, either face up."""
    t = tilt_deg(s)
    return bool(min(t, 180.0 - t) <= MAX_TILT)


def _on_bench(s) -> bool:
    """Resting flat on the strips, its centre still over the bench."""
    return bool(
        abs(bottom(s, PLATE) - (TOP + STRIP_H)) <= ON_BENCH_TOL
        and _flat(s)
        and overlaps(s, PLATE, BENCH_RECT)
        and s.pos[1] <= BENCH_END
    )


def _on_table(s) -> bool:
    """Some point of the plate that is within 5 mm of the table lies outside the tray's outer outline (or on the floor)."""
    c = corners(s, PLATE)
    low = c[c[:, 2] <= TOP + TABLE_TOUCH]
    return bool(any(not (TRAY_OUT.x0 <= x <= TRAY_OUT.x1 and TRAY_OUT.y0 <= y <= TRAY_OUT.y1) for x, y, _ in low))


def _in_tray(s) -> bool:
    return bool(inside(s, PLATE, TRAY_IN) and bottom(s, PLATE) <= TOP + FLOOR_T + FLOOR_TOL and _flat(s))


def _centre_over_tray(s) -> bool:
    return bool(TRAY_OUT.x0 <= s.pos[0] <= TRAY_OUT.x1 and TRAY_OUT.y0 <= s.pos[1] <= TRAY_OUT.y1)


def score(objects):
    """0 if the plate ever touched the table or floor outside the tray, or is held more than 1 cm above every
    surface under it. Otherwise: 1.0 wholly inside the tray on its floor (bottom within 3 mm of the tray floor, tilt at
    most 5 deg, either face up), at rest, after it had lain flat on the bench, moving or still, with at least 4 cm past the end; 0.6 at
    rest, not flat on the floor, with its centre over the tray outline (same condition); 0.3 resting flat on the bench with at least 4 cm past the end and
    its centre still on the bench; else 0."""
    s = objects["plate"]
    pos = np.asarray(s.pos, float)
    last = _DROP["last"]
    if last is not None and np.linalg.norm(pos - last) > TELEPORT and np.linalg.norm(s.lin_vel) < 0.3:
        _DROP["dropped"] = False  # a simulator reset: the next trial starts clean
    _DROP["last"] = pos
    peaks = _TRK.update(objects, {"window": _overhang(s) if _on_bench(s) else 0.0})
    if _on_table(s):
        _DROP["dropped"] = True
    if _DROP["dropped"] or _airborne(s):
        return 0.0
    pushed = peaks["window"] >= WINDOW_MIN
    if pushed and s.resting() and _in_tray(s):
        return 1.0
    if pushed and s.resting() and _centre_over_tray(s):
        return 0.6
    if _on_bench(s) and _overhang(s) >= WINDOW_MIN:
        return 0.3
    return 0.0


def success(objects):
    """Plate in the tray after the push, and still; holds SUCCESS_HOLD_S (10 s)."""
    return score(objects) == 1.0 and objects["plate"].resting()


def distance(objects):
    """Metres from the plate's centre to the middle of the tray floor."""
    p = objects["plate"].pos
    return float(np.linalg.norm(np.asarray(p) - np.array([*TRAY_C, TOP + FLOOR_T + PLATE_T / 2])))


# The scorer reads no declared area off limits to the robot: the instruction forbids no area, so there is nothing for the
# sweep to keep the arms out of. The demo's hands stay off the table beyond the bench end and clear of the tray walls.
ARM_KEEP_OUT: list = []

# -- the demo ------------------------------------------------------------------------------------------------------------
PUSH_X = 0.28
PUSH_Y0, PUSH_Y1 = -0.105, -0.03  # the right arm's closed fingers, in the slot, behind the bench then 7.5 cm along it
PUSH_Z = TOP + 0.043  # tips at TOP + 0.030, below the plate's underside (0.040)
GRIP_Y = 0.115  # the left arm's fingers across the overhang, 3.5 cm from the plate's front end
GRIP_Z = TOP + 0.047  # tips 6 mm under the plate, so the plate sits low in the hand
DRAW_Y = 0.23  # far enough that the plate's rear end clears the bench's front end by 1.5 cm
DROP_XY = (TRAY_C[0], 0.153)  # the hand is 5.3 cm further along than the plate's centre, which lands at y 0.10
DROP_Z = TOP + 0.020  # tips at TOP + 0.007, 4 mm over the 3 mm floor; the plate's bottom is about 1 cm over it, so a held plate fails _in_tray
TRAVEL = TOP + 0.14
HOVER = TOP + 0.12
CLEAR_XY = (0.28, 0.30, TRAVEL)  # the left hand ends here, off the tray, so the top view shows the bare plate


def demo():
    """RIGHT arm, fingers closed: down behind the bench, push the plate 7.5 cm along its slot (about 6 cm of it
    overhangs), back out, up. LEFT arm, yaw -pi/2 (fingers close along x): waits above the bench end, takes the overhang across its 6 cm width,
    draws the plate clear in contact, lifts, carries it over the tray, lowers, opens, rises and moves clear of the tray. All moves are slow:
    the hands stay under 5 cm/s."""
    right = [
        Waypoint(4.0, (PUSH_X, PUSH_Y0, HOVER), FINGER_CLOSED, "to the slot", 0.0),
        Waypoint(3.0, (PUSH_X, PUSH_Y0, PUSH_Z), FINGER_CLOSED, "down behind the bench", 0.0),
        Waypoint(3.5, (PUSH_X, PUSH_Y1, PUSH_Z), FINGER_CLOSED, "push", 0.0),
        Waypoint(0.5, (PUSH_X, PUSH_Y1, PUSH_Z), FINGER_CLOSED, "settle", 0.0),
        Waypoint(3.0, (PUSH_X, PUSH_Y0, PUSH_Z), FINGER_CLOSED, "back out", 0.0),
        Waypoint(2.5, (PUSH_X, PUSH_Y0, HOVER), FINGER_CLOSED, "up", 0.0),
    ]
    q = -np.pi / 2
    left = [
        Waypoint(7.0, (PUSH_X, GRIP_Y, HOVER), FINGER_OPEN, "to the bench end", q),
        Waypoint(8.0, (PUSH_X, GRIP_Y, HOVER), FINGER_OPEN, "wait for the push", q),
        Waypoint(4.0, (PUSH_X, GRIP_Y, GRIP_Z), FINGER_OPEN, "down beside the overhang", q),
        Waypoint(1.5, (PUSH_X, GRIP_Y, GRIP_Z), FINGER_CLOSED, "close", q),
        Waypoint(6.0, (PUSH_X, DRAW_Y, GRIP_Z + 0.004), FINGER_CLOSED, "draw out", q),
        Waypoint(4.0, (PUSH_X, DRAW_Y, TRAVEL), FINGER_CLOSED, "lift", q),
        Waypoint(7.0, (*DROP_XY, TRAVEL), FINGER_CLOSED, "carry", q),
        Waypoint(5.0, (*DROP_XY, DROP_Z), FINGER_CLOSED, "lower", q),
        Waypoint(1.5, (*DROP_XY, DROP_Z), FINGER_OPEN, "release", q),
        Waypoint(3.0, (*DROP_XY, TRAVEL), FINGER_OPEN, "rise", q),
        Waypoint(5.0, CLEAR_XY, FINGER_OPEN, "clear of the tray", q),
    ]
    return {"right": right, "left": left}
