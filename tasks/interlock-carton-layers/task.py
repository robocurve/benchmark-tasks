"""On the station table: a square grey pallet board (24 x 24 x 1.2 cm, painted plywood) between the arms, and four
identical closed shipping cartons (kraft corrugated, 18.0 x 7.5 x 5.0 cm) lying flat on the table, two beside each
arm. All four cartons lie with their long side pointing away from the arms (along x), as they come off an infeed.

The board holds two cartons per layer: two cartons side by side make an 18 x 18 cm layer with a 3 cm gap between
them. The board is taped to the table so it does not move; the cartons are loose.
"""

import itertools
import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the pallet board ------------------------------------------------------------------------------------------

BOARD_CENTER = (0.250, 0.000)  # x, y (m)
BOARD_SIZE = (0.240, 0.240, 0.012)
BOARD_TOP = TOP + BOARD_SIZE[2]
BOARD = Box(
    "pallet_board",
    pos=(BOARD_CENTER[0], BOARD_CENTER[1], TOP + BOARD_SIZE[2] / 2),
    size=BOARD_SIZE,
    color=(0.42, 0.44, 0.47),
)

# -- the cartons -------------------------------------------------------------------------------------------------

CARTON_SIZE = (0.180, 0.075, 0.050)  # length (body x), width (body y, where the fingers close), height
CARTON_L, CARTON_W, CARTON_H = CARTON_SIZE
CARTON_MASS = 0.20
KRAFT = (0.70, 0.54, 0.36)
SPAWN_GAP = 0.001  # cartons spawn this far above the table and settle onto it
LAYER_GAP = 0.030  # the gap between the two cartons of a layer in the demo (room for a finger)
PITCH = CARTON_W + LAYER_GAP  # centre-to-centre spacing of the two cartons of a layer

# Starting spots, long side along x: an inner and an outer carton beside each arm, 4 cm apart.
START = {
    "carton_1": (0.250, 0.210),
    "carton_2": (0.250, 0.325),
    "carton_3": (0.250, -0.210),
    "carton_4": (0.250, -0.325),
}
CARTONS = {
    name: Box(name, pos=(x, y, TOP + CARTON_H / 2 + SPAWN_GAP), size=CARTON_SIZE, mass=CARTON_MASS, color=KRAFT)
    for name, (x, y) in START.items()
}
NAMES = tuple(CARTONS)

OBJECTS = [BOARD, *CARTONS.values()]

# -- scoring ---------------------------------------------------------------------------------------------------

EDGE_TOL = 0.005  # a footprint may poke this far past a board edge
LEVEL_BELOW, LEVEL_ABOVE = 0.005, 0.010  # a carton's bottom within -5 mm / +10 mm of its layer's support height
FLAT_DEG = 10.0  # tilted less than this: lying flat
TIPPED_DEG = 20.0  # tilted more than this anywhere: a tipped or collapsed carton
TURN_MIN_DEG = 80.0  # a top carton's long side at 90 +- 10 deg to each carton it rests on
BRIDGE_MIN = 0.20  # a top carton bridges the joint: at least this share of its length (3.6 cm) on each bottom carton
BOTTOM_PTS, TOP_PTS, STACKED_PTS = 0.2, 0.3, 0.1
PER_LAYER = 2


def _rot(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def tilt_deg(state) -> float:
    """Angle between the carton's own up axis and the vertical, ignoring which big face is up (deg)."""
    x, y, _, _ = state.quat_xyzw
    cos = abs(1.0 - 2.0 * (x * x + y * y))
    return math.degrees(math.acos(min(1.0, cos)))


def long_axis_deg(state) -> float:
    """Heading of the carton's long side in the table plane, 0..180 deg (0 = along x, away from the arms)."""
    axis = _rot(state)[:, 0]
    return math.degrees(math.atan2(axis[1], axis[0])) % 180.0


def turn_between_deg(a, b) -> float:
    """Angle between two cartons' long sides in the table plane, 0..90 deg."""
    d = abs(long_axis_deg(a) - long_axis_deg(b)) % 180.0
    return min(d, 180.0 - d)


_ALONG = np.linspace(-CARTON_L / 2, CARTON_L / 2, 73)  # 2.5 mm steps along a carton's length
_ACROSS = np.linspace(-CARTON_W / 2, CARTON_W / 2, 16)
_GRID = np.array([[x, y, -CARTON_H / 2] for x in _ALONG for y in _ACROSS])


def length_over(top, below) -> float:
    """Share of ``top``'s length that rests on ``below`` (seen from above): the cross-sections of its bottom face,
    taken along its length, that touch ``below``'s footprint. 0..1."""
    pts = _GRID @ _rot(top).T + np.asarray(top.pos)
    local = (pts - np.asarray(below.pos)) @ _rot(below)
    inside = (np.abs(local[:, 0]) <= CARTON_L / 2 + 1e-9) & (np.abs(local[:, 1]) <= CARTON_W / 2 + 1e-9)
    return float(inside.reshape(len(_ALONG), len(_ACROSS)).any(axis=1).mean())


def on_board(state) -> bool:
    """The carton's footprint is within the board's edges (5 mm tolerance)."""
    lo = np.array(BOARD_CENTER) - np.array(BOARD_SIZE[:2]) / 2 - EDGE_TOL
    hi = np.array(BOARD_CENTER) + np.array(BOARD_SIZE[:2]) / 2 + EDGE_TOL
    return bool(np.all(state.low[:2] >= lo) and np.all(state.high[:2] <= hi))


def _at_level(state, support_top: float) -> bool:
    return bool(support_top - LEVEL_BELOW <= state.low[2] <= support_top + LEVEL_ABOVE)


def is_bottom(state) -> bool:
    """A first-layer carton: lying flat and still on the board, footprint within its edges."""
    return _at_level(state, BOARD_TOP) and tilt_deg(state) < FLAT_DEG and on_board(state) and state.resting()


def is_second_layer(state) -> bool:
    """Lying flat and still one carton height up, footprint within the board's edges."""
    return (
        _at_level(state, BOARD_TOP + CARTON_H) and tilt_deg(state) < FLAT_DEG and on_board(state) and state.resting()
    )


def collapsed(objects) -> bool:
    """A carton fell below the table top or lies tipped more than 20 deg (on its side, on end, or leaning)."""
    return any(objects[n].pos[2] < TOP or tilt_deg(objects[n]) > TIPPED_DEG for n in NAMES)


def classify(objects) -> dict[str, str]:
    """Each carton's grade: 'bottom' (first layer), 'interlocked' (second layer, turned 90 +- 10 deg to and
    bridging both first-layer cartons), 'stacked' (second layer over a first-layer carton but not interlocked),
    or 'none'. Only the first two first-layer cartons (by name) count as the first layer."""
    bottoms = [n for n in NAMES if is_bottom(objects[n])][:PER_LAYER]
    grade = {n: "none" for n in NAMES}
    for n in bottoms:
        grade[n] = "bottom"
    for n in NAMES:
        if n in bottoms or not is_second_layer(objects[n]):
            continue
        top = objects[n]
        shares = {b: length_over(top, objects[b]) for b in bottoms}
        over = [b for b, s in shares.items() if s >= BRIDGE_MIN]
        if len(over) == PER_LAYER and all(turn_between_deg(top, objects[b]) >= TURN_MIN_DEG for b in over):
            grade[n] = "interlocked"
        elif over:
            grade[n] = "stacked"
    # At most two cartons count in the second layer: interlocked ones first, then by name.
    second = sorted((n for n in NAMES if grade[n] in ("interlocked", "stacked")), key=lambda n: grade[n] != "interlocked")
    for n in second[PER_LAYER:]:
        grade[n] = "none"
    return grade


POINTS = {"bottom": BOTTOM_PTS, "interlocked": TOP_PTS, "stacked": STACKED_PTS, "none": 0.0}


def score(objects):
    """0.2 per first-layer carton (at most two), 0.3 per interlocked second-layer carton and 0.1 per second-layer
    carton that rests on the first layer without interlocking (at most two in the second layer); 0 for
    everything if a carton is tipped or off the table."""
    if collapsed(objects):
        return 0.0
    return round(float(sum(POINTS[g] for g in classify(objects).values())), 6)


def success(objects):
    """Two cartons flat on the board, two on top of them turned 90 deg and bridging the joint, all still."""
    if collapsed(objects):
        return False
    grades = sorted(classify(objects).values())
    return grades == ["bottom", "bottom", "interlocked", "interlocked"]


# Where the demo puts each carton (centre x, y, layer, long-side heading): the first layer across the arms (along
# y), the second layer pointing away from the arms (along x). Turning the whole pattern 90 deg is just as valid.
BX, BY = BOARD_CENTER
GOAL = {
    "carton_1": (BX + PITCH / 2, BY, 0, 90.0),  # first layer, far
    "carton_3": (BX - PITCH / 2, BY, 0, 90.0),  # first layer, near
    "carton_2": (BX, BY + PITCH / 2, 1, 0.0),  # second layer, left
    "carton_4": (BX, BY - PITCH / 2, 1, 0.0),  # second layer, right
}


def goal_center(name: str) -> np.ndarray:
    x, y, layer, _ = GOAL[name]
    return np.array([x, y, BOARD_TOP + CARTON_H * (layer + 0.5)])


def _slots(turned: bool) -> list[np.ndarray]:
    out = []
    for name in NAMES:
        x, y, layer, _ = GOAL[name]
        if turned:  # the same pattern turned 90 deg about the board centre
            x, y = BX - (y - BY), BY + (x - BX)
        out.append(np.array([x, y, BOARD_TOP + CARTON_H * (layer + 0.5)]))
    return out


def distance(objects):
    """Metres, summed over the cartons: each carton's centre to a slot of the two-layer pattern, matching
    cartons to slots and choosing the pattern's turn to make the sum smallest."""
    pos = [np.asarray(objects[n].pos) for n in NAMES]
    best = math.inf
    for turned in (False, True):
        slots = _slots(turned)
        for perm in itertools.permutations(range(len(NAMES))):
            best = min(best, sum(float(np.linalg.norm(pos[i] - slots[j])) for i, j in enumerate(perm)))
    return best


# -- demo ------------------------------------------------------------------------------------------------------

TRAVEL_Z = TOP + 0.140  # carry height of the TCP; higher can pass joint4's limit
GRIP_BELOW_TOP = 0.014  # the TCP grips this far below a carton's top, so the pads hold its upper 2.7 cm
RELEASE_GAP = 0.005  # a carton is let go this far above its support
LOOSEN = 0.004  # each pad backs off this far to let go (the other side has a carton 3 cm away)
LIFT_AFTER_RELEASE = 0.030  # rise this far straight up before any sideways move
LOWER_S = 2.6  # lowering onto the stack: 8.7 cm (first layer) in 2.6 s, under 3.5 cm/s on average
SETTLE_S = 0.6  # after one arm has left the board, before the other lowers a carton beside the one just placed
HOLD_S = 2.0  # both arms still at the end, so the 1 s success hold completes
PICK_YAW = 0.0  # every carton starts along x: fingers close along y, across its width
FIRST_LAYER_YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # first layer along y: fingers close along x
SECOND_LAYER_YAW = 0.0
PARK = {"left": (0.200, 0.240, TRAVEL_Z), "right": (0.200, -0.240, TRAVEL_Z)}
DEMO = [  # (arm, carton) in order
    ("left", "carton_1"),
    ("right", "carton_3"),
    ("left", "carton_2"),
    ("right", "carton_4"),
]


def finger_for(width: float) -> float:
    """Finger joint target for pads ``width`` apart (FINGER_OPEN at 94 mm)."""
    return max(FINGER_OPEN, -width / 2)


def place_yaw(name: str, arm: str) -> float:
    return FIRST_LAYER_YAW[arm] if GOAL[name][2] == 0 else SECOND_LAYER_YAW


def grasp_z(support_top: float) -> float:
    return support_top + CARTON_H - GRIP_BELOW_TOP


class _Track:
    """A waypoint list with its running end time and the last pose commanded."""

    def __init__(self, start):
        self.wps, self.t = [], 0.0
        self.pos, self.finger, self.yaw = start, FINGER_OPEN, PICK_YAW

    def go(self, duration, pos=None, finger=None, yaw=None, label=""):
        if duration <= 1e-9:
            return
        self.pos = self.pos if pos is None else pos
        self.finger = self.finger if finger is None else finger
        self.yaw = self.yaw if yaw is None else yaw
        self.wps.append(Waypoint(duration, self.pos, self.finger, label, self.yaw))
        self.t += duration

    def wait_until(self, t, label="wait"):
        self.go(t - self.t, label=label)


def demo():
    """The arms take turns. Left: carton 1 to the far half of the first layer, turned to lie across the arms.
    Right: carton 3 to the near half, turned the same way, 3 cm from carton 1. Left: carton 2 on top, pointing
    away from the arms, over the left halves of cartons 1 and 3. Right: carton 4 beside it on the right, 3 cm
    away. Each arm picks its next carton while the other works at the board, and waits until the board is free.
    Grips are straight down across a carton's 7.5 cm width; every release is a 4 mm loosening 5 mm above the
    support, then a 3 cm straight lift."""
    tracks = {side: _Track(PARK[side]) for side in ("left", "right")}
    for side in tracks:
        tracks[side].go(1.5, PARK[side], FINGER_OPEN, PICK_YAW, "to the park spot")
    board_free = 0.0
    for arm, name in DEMO:
        tr = tracks[arm]
        sx, sy = START[name]
        gx, gy, layer, _ = GOAL[name]
        yaw = place_yaw(name, arm)
        support = BOARD_TOP + layer * CARTON_H
        pick_z, place_z = grasp_z(TOP), grasp_z(support) + RELEASE_GAP
        tr.go(1.5, (sx, sy, TRAVEL_Z), FINGER_OPEN, PICK_YAW, f"over {name}")
        tr.go(1.5, (sx, sy, pick_z), label="descend")
        tr.go(0.8, finger=FINGER_CLOSED, label="close")
        tr.go(1.2, (sx, sy, TRAVEL_Z), label="lift")
        tr.wait_until(board_free, "wait for the board")
        tr.go(2.0, (gx, gy, TRAVEL_Z), yaw=yaw, label="carry and turn")
        tr.go(LOWER_S, (gx, gy, place_z), label="lower onto the stack")
        tr.go(0.6, finger=finger_for(CARTON_W + 2 * LOOSEN), label="release")
        tr.go(0.6, (gx, gy, place_z + LIFT_AFTER_RELEASE), label="lift off")
        tr.go(1.0, (gx, gy, TRAVEL_Z), finger=FINGER_OPEN, label="up")
        tr.go(1.5, PARK[arm], label="back to the park spot")
        board_free = tr.t + SETTLE_S
    end = max(t.t for t in tracks.values()) + HOLD_S
    for tr in tracks.values():
        tr.wait_until(end, "hold still")
    return {side: tr.wps for side, tr in tracks.items()}
