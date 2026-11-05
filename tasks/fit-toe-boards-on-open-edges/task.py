"""On the station table, between the arms: a raised grey work platform, 200 mm deep and 240 mm wide, its top
60 mm above the table. Its far edge and its right edge (robot's view) already have fixed grey walls standing 40 mm
above the platform top. Its near edge (toward the robot) and its left edge are open. Three loose pine toe-boards,
each 15 mm thick and 40 mm tall, stand on edge on the table: two 117 mm boards in front of the open near edge,
one 182 mm board beside the open left edge. Three orange rubber balls rest on the platform top.

The job: stand the three boards on the platform along its two open edges, each board's outer face against the
edge, so that no gap between boards, at the corner or next to a wall is wider than 6 mm, without letting any ball
fall off the platform.
"""

import math

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Platform. World frame: x away from the arm bases, y to the robot's left, z up (m).
PLAT_X0, PLAT_X1 = 0.20, 0.40  # near (open) edge and far edge
PLAT_Y0, PLAT_Y1 = -0.12, 0.12  # right edge (walled) and left (open) edge
PLAT_H = 0.060
PLAT_TOP = TOP + PLAT_H
WALL_T, WALL_UP = 0.015, 0.040  # walls: 15 mm thick, standing 40 mm above the platform top

# Toe-boards: pine, 15 mm thick, 40 mm tall. Two 117 mm boards for the 240 mm near edge (6 mm of slack in all),
# one 182 mm board for the left edge behind the near boards (200 - 15 = 185 mm, 3 mm of slack).
BOARD_T, BOARD_H = 0.015, 0.040
FRONT_L, SIDE_L = 0.117, 0.182
PINE_DENSITY = 500.0  # kg/m^3
FRONT_SIZE = (BOARD_T, FRONT_L, BOARD_H)  # lies along y
SIDE_SIZE = (SIDE_L, BOARD_T, BOARD_H)  # lies along x
BOARD_START = {
    "board_front_a": (0.14, 0.10),  # in front of the near edge, left of the midline
    "board_front_b": (0.14, -0.10),  # in front of the near edge, right of the midline
    "board_side": (0.30, 0.20),  # beside the left edge
}

BALL_R, BALL_MASS = 0.020, 0.040
BALL_XY = {"ball_1": (0.31, 0.00), "ball_2": (0.35, -0.05), "ball_3": (0.36, 0.04)}

GREY, WALL_GREY, PINE, ORANGE = (0.55, 0.56, 0.58), (0.40, 0.41, 0.43), (0.85, 0.70, 0.45), (0.95, 0.45, 0.10)


def _mass(size) -> float:
    return round(PINE_DENSITY * size[0] * size[1] * size[2], 4)


PLATFORM = [
    Box("platform", pos=((PLAT_X0 + PLAT_X1) / 2, 0.0, TOP + PLAT_H / 2),
        size=(PLAT_X1 - PLAT_X0, PLAT_Y1 - PLAT_Y0, PLAT_H), color=GREY),
    # Far wall: outside the far edge, from the table to 40 mm above the platform top, across the right wall too.
    Box("far_wall", pos=(PLAT_X1 + WALL_T / 2, (PLAT_Y0 - WALL_T + PLAT_Y1) / 2, TOP + (PLAT_H + WALL_UP) / 2),
        size=(WALL_T, PLAT_Y1 - PLAT_Y0 + WALL_T, PLAT_H + WALL_UP), color=WALL_GREY),
    # Right wall: outside the right edge, the platform's full depth.
    Box("right_wall", pos=((PLAT_X0 + PLAT_X1) / 2, PLAT_Y0 - WALL_T / 2, TOP + (PLAT_H + WALL_UP) / 2),
        size=(PLAT_X1 - PLAT_X0, WALL_T, PLAT_H + WALL_UP), color=WALL_GREY),
]
BOARDS = [
    Box(name, pos=(*BOARD_START[name], TOP + BOARD_H / 2), size=size, mass=_mass(size), color=PINE)
    for name, size in (("board_front_a", FRONT_SIZE), ("board_front_b", FRONT_SIZE), ("board_side", SIDE_SIZE))
]
BALLS = [Sphere(name, pos=(*xy, PLAT_TOP + BALL_R), radius=BALL_R, mass=BALL_MASS, color=ORANGE)
         for name, xy in BALL_XY.items()]
OBJECTS = [*PLATFORM, *BOARDS, *BALLS]
SIZES = {b.name: b.size for b in BOARDS}

# Scoring tolerances.
TILT_MAX_DEG = 5.0  # a standing board: its 40 mm side within 5 deg of vertical
SUPPORT_TOL = 0.001  # its bottom within 1 mm of the platform top (a board held 2 mm up does not count)
FLUSH_TOL = 0.004  # both outer corners within 4 mm of the edge line, inside or outside it
GAP_MAX = 0.006  # widest allowed uncovered stretch of an open edge
PARTIAL_WEIGHT = 0.9

# The two open edges: (name, axis across the edge, edge line, outward sign, axis along the edge, span along it).
EDGES = (
    ("near", 0, PLAT_X0, -1.0, 1, (PLAT_Y0, PLAT_Y1)),
    ("left", 1, PLAT_Y1, +1.0, 0, (PLAT_X0, PLAT_X1)),
)
OPEN_LENGTH = sum(s[1] - s[0] for *_, s in EDGES)  # 0.44 m


def _rot(st) -> np.ndarray:
    x, y, z, w = (float(v) for v in st.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(st) -> float:
    """Angle of the board's 40 mm side from vertical, either way up (a board turned over is still upright)."""
    return float(np.degrees(np.arccos(np.clip(abs(_rot(st)[2, 2]), 0.0, 1.0))))


def _corners(st, size) -> np.ndarray:
    """The four corners of the board's bottom face (the face on the platform, whichever way up it stands) in plan
    (x, y), from its pose and its own size."""
    r = _rot(st)
    hx, hy = size[0] / 2, size[1] / 2
    hz = -np.sign(r[2, 2] or 1.0) * size[2] / 2  # local z of the lower face
    local = np.array([[sx * hx, sy * hy, hz] for sx in (-1, 1) for sy in (-1, 1)])
    return (local @ r.T)[:, :2] + st.pos[:2]


def _standing(st) -> bool:
    """Standing on edge on the platform top and at rest: its 40 mm side within 5 deg of vertical (so not lying
    flat or on end), its bottom within 1 mm of the platform top, its centre over the platform (within 10 mm of
    its outline)."""
    return bool(
        _tilt_deg(st) <= TILT_MAX_DEG
        and abs(float(st.low[2]) - PLAT_TOP) <= SUPPORT_TOL
        and PLAT_X0 - 0.01 < st.pos[0] < PLAT_X1 + 0.01
        and PLAT_Y0 - 0.01 < st.pos[1] < PLAT_Y1 + 0.01
        and st.resting()
    )


def _coverage(objects) -> dict[str, list[tuple[float, float]]]:
    """Per open edge, the stretches covered by standing boards. A board covers the stretch between the two
    outermost corners of its bottom face (toward the outside of that edge) when both are within 4 mm of the edge
    line."""
    out = {name: [] for name, *_ in EDGES}
    for b in BOARDS:
        st = objects[b.name]
        if not _standing(st):
            continue
        c = _corners(st, SIZES[b.name])
        for name, ax, line, sign, along, (s0, s1) in EDGES:
            outer = c[np.argsort(-sign * c[:, ax])[:2]]
            if np.all(np.abs(outer[:, ax] - line) <= FLUSH_TOL + 1e-9):
                lo, hi = sorted(outer[:, along])
                lo, hi = max(lo, s0), min(hi, s1)
                if hi > lo:
                    out[name].append((float(lo), float(hi)))
    return out


def _gaps(intervals, span) -> list[float]:
    """Uncovered stretches of an edge, in order along it (m)."""
    gaps, pos = [], span[0]
    for lo, hi in sorted(intervals):
        if lo > pos:
            gaps.append(lo - pos)
        pos = max(pos, hi)
    if span[1] > pos:
        gaps.append(span[1] - pos)
    return gaps


def edge_report(objects) -> tuple[float, float]:
    """(covered length of both open edges, widest gap), in metres."""
    cov = _coverage(objects)
    covered, widest = 0.0, 0.0
    for name, *_, span in EDGES:
        g = _gaps(cov[name], span)
        covered += (span[1] - span[0]) - sum(g)
        widest = max([widest, *g])
    return covered, widest


def _ball_on_platform(st) -> bool:
    """A ball is on the platform while its centre is over the platform and its bottom is not below the top by
    more than 5 mm."""
    return bool(PLAT_X0 <= st.pos[0] <= PLAT_X1 and PLAT_Y0 <= st.pos[1] <= PLAT_Y1
                and st.low[2] >= PLAT_TOP - 0.005)


def _ball_resting_on_platform(st) -> bool:
    return _ball_on_platform(st) and abs(float(st.low[2]) - PLAT_TOP) <= SUPPORT_TOL


# Trial history: whether any ball has left the platform since the trial started. The scorer sees one step at a
# time, so it is kept here and cleared whenever every loose object is back at its starting place (a new trial).
_HISTORY = {"ball_fell": False}
_START = {o.name: np.asarray(o.pos) for o in [*BOARDS, *BALLS]}


def _update(objects) -> None:
    if all(np.linalg.norm(objects[n].pos - p) < 0.002 for n, p in _START.items()):
        _HISTORY["ball_fell"] = False
    if any(not _ball_on_platform(objects[b.name]) for b in BALLS):
        _HISTORY["ball_fell"] = True


def _complete(objects) -> bool:
    covered, widest = edge_report(objects)
    return covered > 0 and widest <= GAP_MAX + 1e-9


def _balls_ok(objects) -> bool:
    return not _HISTORY["ball_fell"] and all(_ball_resting_on_platform(objects[b.name]) for b in BALLS)


def success(objects):
    """Both open edges covered by standing, flush boards with no gap wider than 6 mm; every ball resting on the
    platform; no ball has left the platform during the trial."""
    _update(objects)
    return bool(_balls_ok(objects) and _complete(objects))


def distance(objects):
    covered, _ = edge_report(objects)
    return float(OPEN_LENGTH - covered)


def score(objects):
    """1.0 as success. Otherwise 0.9 x (open-edge length covered by standing flush boards) / 440 mm, rounded down
    to 0.01. 0 for the whole trial once any ball has left the platform, and 0 while any ball is not resting on
    the platform top."""
    _update(objects)
    if not _balls_ok(objects):
        return 0.0
    if _complete(objects):
        return 1.0
    covered, _ = edge_report(objects)
    return math.floor(100 * PARTIAL_WEIGHT * covered / OPEN_LENGTH + 1e-9) / 100


# ---- scripted demo ------------------------------------------------------------------------------------------
# Each board is gripped across its 15 mm thickness, the TCP 25 mm above the board's bottom (finger tips 12 mm
# above it, 15 mm below its top), with the fingers part open so they stay clear of the balls.
GRIP_ABOVE_BOTTOM = 0.025
OPEN_PART = -0.018  # pads 36 mm apart around the 15 mm board
CLOSED = 0.0
TRAVEL_Z = TOP + 0.15
PLACE_DROP = 0.006  # release with the board's bottom commanded 6 mm above the platform top
# Fingers closing along x, for the boards that lie along y: each arm turns the way that keeps joint6 in range.
YAW_ACROSS_X = {"left": -math.pi / 2, "right": math.pi / 2}

# Target centres. Near boards: outer faces 0.5 mm inside the near edge. Board b (right) ends 2.5 mm from the right
# wall (the right arm settles about 1.7 mm toward the wall; at 1 mm the board's end caught on the wall top in a
# trace); board a (left) ends 0.5 mm short of the left edge, leaving a 3 mm joint between them. The side board's
# outer face is flush with the left edge, its near end 1.5 mm behind board a.
NEAR_X = PLAT_X0 + 0.0005 + BOARD_T / 2
TARGET = {
    "board_front_b": (NEAR_X, PLAT_Y0 + 0.0025 + FRONT_L / 2),
    "board_front_a": (NEAR_X, PLAT_Y1 - 0.0005 - FRONT_L / 2),
    "board_side": (PLAT_X0 + BOARD_T + 0.0015 + SIDE_L / 2, PLAT_Y1 - BOARD_T / 2),
}
# Aim offsets (TCP minus wanted board centre), per board, to cancel arm tracking error; set from traces.
AIM = {"board_front_a": (0.0, 0.0), "board_front_b": (0.0, 0.0), "board_side": (0.0, 0.0)}


def _move_board(name: str, yaw: float) -> list[Waypoint]:
    (px, py), (qx, qy) = BOARD_START[name], TARGET[name]
    ax, ay = AIM[name]
    qx, qy = qx + ax, qy + ay
    pick_z = TOP + GRIP_ABOVE_BOTTOM
    place_z = PLAT_TOP + GRIP_ABOVE_BOTTOM + PLACE_DROP
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), OPEN_PART, f"over {name}", yaw),
        Waypoint(1.5, (px, py, pick_z), OPEN_PART, "descend", yaw),
        Waypoint(0.8, (px, py, pick_z), CLOSED, "grip", yaw),
        Waypoint(1.2, (px, py, TRAVEL_Z), CLOSED, "lift", yaw),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), CLOSED, "carry", yaw),
        Waypoint(1.5, (qx, qy, place_z + 0.02), CLOSED, "lower", yaw),
        Waypoint(1.0, (qx, qy, place_z), CLOSED, "set down", yaw),
        Waypoint(0.6, (qx, qy, place_z), OPEN_PART, "release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), OPEN_PART, "up", yaw),
    ]


def demo():
    """Left arm: the left near board, then the side board. Right arm: waits, then the right near board."""
    left = _move_board("board_front_a", YAW_ACROSS_X["left"]) + _move_board("board_side", 0.0)
    right = _move_board("board_front_b", YAW_ACROSS_X["right"])
    # The right arm waits above its board until the left arm has set board a, so the two never work at the
    # middle joint at the same time.
    right.insert(1, Waypoint(8.0, right[0].pos, OPEN_PART, "wait", right[0].yaw))
    return {"left": left, "right": right}
