"""On the station table: a big kraft carton (12 x 20 x 8 cm tall, fixed) in front of the arms, on the centre line,
with a white shipping label (10 x 8 cm) on the left part of its top, flush with its left edge. In front of it and to
the right, nearer the arms, a small blue carton (7 x 6 x 6 cm tall) stands loose on the table.

The small carton must end up fully on the big carton's top without ever being over the label lower than 3 cm
above it: only the unlabelled right part of the top (12 cm wide) is free.

Requires -E object_jitter=0: the free part leaves 3 cm on each side of a 6 cm carton.

Run it:  inspect-robots run --embodiment yam_arena -E task=stack_without_covering_the_label.py -E object_jitter=0 \\
           --instruction "Stack the small carton on the big carton, fully on its top. The white patch is the
           shipping label: no part of the small carton may ever be over it lower than 3 cm above it, at any
           moment." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dataclasses import replace  # noqa: E402

import numpy as np  # noqa: E402

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402


# -- Shared footprint and height helpers, copied unchanged from the tested kit belimits.py. They are inlined
# so this task stays one file (task.json, README.md and this file are the whole submission).


class Rect:
    """An axis-aligned rectangle seen from above: x in (x0, x1), y in (y0, y1)."""

    def __init__(self, x, y):
        self.x0, self.x1 = float(min(x)), float(max(x))
        self.y0, self.y1 = float(min(y)), float(max(y))

    def grown(self, m: float) -> "Rect":
        return Rect((self.x0 - m, self.x1 + m), (self.y0 - m, self.y1 + m))

    def poly(self) -> np.ndarray:
        return np.array([[self.x0, self.y0], [self.x1, self.y0], [self.x1, self.y1], [self.x0, self.y1]])


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _half_extents(spec) -> tuple[str, np.ndarray]:
    if hasattr(spec, "size"):
        return "box", np.asarray(spec.size, float) / 2
    if hasattr(spec, "height"):
        return "cylinder", np.array([spec.radius, spec.radius, spec.height / 2], float)
    if hasattr(spec, "radius"):
        return "sphere", np.array([spec.radius] * 3, float)
    raise TypeError(f"unsupported object spec {spec!r}")


def corners(state, spec) -> np.ndarray:
    """3-D points on the object's surface, in the world: box corners; cylinder rims (16 points each); sphere extremes."""
    kind, h = _half_extents(spec)
    rot, c = _rot(state.quat_xyzw), np.asarray(state.pos, float)
    if kind == "box":
        local = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float) * h
    elif kind == "cylinder":
        a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        ring = np.stack([np.cos(a) * h[0], np.sin(a) * h[1]], axis=1)
        local = np.vstack([np.c_[ring, np.full(16, -h[2])], np.c_[ring, np.full(16, h[2])]])
    else:
        r = h[0]
        local = np.array([[r, 0, 0], [-r, 0, 0], [0, r, 0], [0, -r, 0], [0, 0, r], [0, 0, -r]], float)
        return c + local  # a sphere's outline does not turn
    return c + local @ rot.T


def footprint(state, spec) -> np.ndarray:
    """The object's outline seen from above: its surface points projected onto the table (x, y)."""
    return corners(state, spec)[:, :2]


def _sat_overlap(pts: np.ndarray, poly: np.ndarray) -> bool:
    """Separating-axis test between the convex hull of ``pts`` and a convex polygon."""
    axes = []
    for shape in (poly, pts):
        n = len(shape)
        for i in range(n):
            e = shape[(i + 1) % n] - shape[i]
            if np.hypot(*e) > 1e-9:
                axes.append(np.array([-e[1], e[0]]) / np.hypot(*e))
    if len(pts) > 2:  # hull edges of the point set: use all pairwise directions (small sets)
        d = pts[:, None, :] - pts[None, :, :]
        for e in d.reshape(-1, 2)[:: max(1, len(pts) // 4)]:
            if np.hypot(*e) > 1e-9:
                axes.append(np.array([-e[1], e[0]]) / np.hypot(*e))
    for a in axes:
        p, r = pts @ a, poly @ a
        if p.max() <= r.min() or r.max() <= p.min():
            return False
    return True


def overlaps(state, spec, area: Rect, z_top: float | None = None) -> bool:
    """Any part of the object's outline over ``area`` (and, if ``z_top`` is given, its bottom below ``z_top``)."""
    if z_top is not None and bottom(state, spec) >= z_top:
        return False
    return _sat_overlap(footprint(state, spec), area.poly())


def bottom(state, spec) -> float:
    return float(corners(state, spec)[:, 2].min())


def is_airborne(state, spec, supports_z, clearance: float = 0.01) -> bool:
    """More than ``clearance`` above every support height: still held, or in flight. The simulator scores the last step
    without opening the grippers, so a held object must never count as placed."""
    b = bottom(state, spec)
    return all(b - z > clearance for z in supports_z)


# -- The task.

SCENE = "station"
SCORE_OVER = "end"
# success() only sees the objects, and a carton held still just above the top looks placed. Asking for 10 s of
# stillness keeps the episode running while the hand lets go and moves clear, so the release is seen.
SUCCESS_HOLD_S = 10.0
TOP = TABLE_TOP[SCENE]

KRAFT, BLUE, WHITE = (0.72, 0.56, 0.38), (0.10, 0.30, 0.78), (0.96, 0.96, 0.94)

# The big carton: fixed (2 kg of sand inside and taped down in the real world). Top at TOP + 0.08.
BIG_H = 0.08
BIG = Box("big_carton", pos=(0.24, 0.0, TOP + BIG_H / 2), size=(0.12, 0.20, BIG_H), color=KRAFT)
BIG_TOP = TOP + BIG_H
TOP_RECT = Rect((0.18, 0.30), (-0.10, 0.10))

# The shipping label: a 2 mm white plate on the left half of the big carton's top, flush with its left edge.
LABEL_T = 0.002
LABEL = Box("shipping_label", pos=(0.24, 0.06, BIG_TOP + LABEL_T / 2), size=(0.10, 0.08, LABEL_T), color=WHITE)
LABEL_TOP = BIG_TOP + LABEL_T
LABEL_RECT = Rect((0.19, 0.29), (0.02, 0.10))
CLEARANCE = 0.03  # m: no part of the small carton over the label lower than this above its top
LABEL_LIMIT_Z = LABEL_TOP + CLEARANCE

# The small carton: loose, blue so it stands out from the table, in front of the big one and to the right of the
# label's strip, just left of the shadow the right arm casts at its ready pose. That far right, the arm's raised wrist
# never covers the label in the top camera's view while it picks the carton up.
SMALL_SIZE = (0.07, 0.06, 0.06)
SMALL = Box("small_carton", pos=(0.10, -0.105, TOP + SMALL_SIZE[2] / 2), size=SMALL_SIZE, mass=0.15, color=BLUE)

OBJECTS = [BIG, LABEL, SMALL]

SUPPORTS = (TOP, BIG_TOP, LABEL_TOP)
EDGE_TOL = 0.002  # m: a bottom corner may be this far past the top's edge
SEAT_TOL = 0.01  # m: the bottom within this of the big carton's top
MAX_TILT = 10.0  # degrees from lying flat on one face



def lowest_over_label(s) -> float | None:
    """Height of the lowest point of the small carton that lies over the label (None if no part is over it):
    the minimum z over the box clipped to the vertical prism above the label, from the vertices of that clip."""
    if not overlaps(s, SMALL, LABEL_RECT):
        return None
    rot, c, h = _rot(s.quat_xyzw), np.asarray(s.pos, float), np.asarray(SMALL.size, float) / 2
    a, b = [], []
    for i in range(3):
        n = rot[:, i]
        a += [n, -n]
        b += [n @ c + h[i], -(n @ c) + h[i]]
    for n, v in (((1, 0, 0), LABEL_RECT.x1), ((-1, 0, 0), -LABEL_RECT.x0), ((0, 1, 0), LABEL_RECT.y1),
                 ((0, -1, 0), -LABEL_RECT.y0)):
        a.append(np.array(n, float))
        b.append(v)
    a, b = np.array(a), np.array(b)
    best = None
    k = len(a)
    for i in range(k):
        for j in range(i + 1, k):
            for m in range(j + 1, k):
                sub = a[[i, j, m]]
                if abs(np.linalg.det(sub)) < 1e-9:
                    continue
                p = np.linalg.solve(sub, b[[i, j, m]])
                if np.all(a @ p <= b + 1e-7):
                    best = p[2] if best is None else min(best, p[2])
    return float(best) if best is not None else bottom(s, SMALL)


def covers_label(s) -> bool:
    """Some part of the small carton over the label lower than 3 cm above its top."""
    z = lowest_over_label(s)
    return z is not None and z < LABEL_LIMIT_Z


# Per-step latch: score() and success() see one step at a time, so a breach is remembered until the next trial.
_state = {"prev": None, "covered": False}
RESET_NEAR, RESET_FAR = 0.002, 0.03  # m: back within 2 mm of spawn after being more than 3 cm away


def _update(objects) -> None:
    s = objects[SMALL.name]
    d = float(np.linalg.norm(np.asarray(s.pos, float) - np.asarray(SMALL.pos, float)))
    prev = _state["prev"]
    if prev is None or (d <= RESET_NEAR and prev > RESET_FAR):  # first call, or the environment was reset
        _state["covered"] = False
    _state["prev"] = d
    if covers_label(s):
        _state["covered"] = True


def label_covered() -> bool:
    return _state["covered"]


def _flat_tilt_deg(s) -> float:
    """Degrees from lying flat on one of its faces."""
    r = _rot(s.quat_xyzw)
    return float(np.degrees(np.arccos(np.clip(np.max(np.abs(r[2, :])), -1.0, 1.0))))


def fully_on_top(s) -> bool:
    """Lying flat on a face on the big carton's top: its four lowest corners within 2 mm of the top's outline and
    within 1 cm above it, tilted at most 10 degrees."""
    pts = corners(s, SMALL)
    low = pts[np.argsort(pts[:, 2])[:4]]
    r = TOP_RECT.grown(EDGE_TOL)
    return bool(
        _flat_tilt_deg(s) <= MAX_TILT
        and np.all((low[:, 0] >= r.x0) & (low[:, 0] <= r.x1) & (low[:, 1] >= r.y0) & (low[:, 1] <= r.y1))
        and np.all(low[:, 2] >= BIG_TOP - 0.003)
        and np.all(low[:, 2] <= BIG_TOP + SEAT_TOL)
    )


def partly_on_top(s) -> bool:
    """Its centre over the big carton's top and its lowest point within 1 cm of the top's height."""
    x, y = float(s.pos[0]), float(s.pos[1])
    return bool(TOP_RECT.x0 <= x <= TOP_RECT.x1 and TOP_RECT.y0 <= y <= TOP_RECT.y1
                and abs(bottom(s, SMALL) - BIG_TOP) <= SEAT_TOL)


def score(objects):
    """0 if at any step part of the small carton was over the label lower than 3 cm above it, or it ends held
    over the label at any height. Otherwise, at the end: 1.0 lying flat with all four lower corners on the big
    carton's top; 0.5 with its centre over the top but overhanging or tilted; else 0 (on the table, held in the
    air, or off the table)."""
    _update(objects)
    s = objects[SMALL.name]
    if _state["covered"]:
        return 0.0
    if is_airborne(s, SMALL, SUPPORTS):
        # Still held: a real trial opens the grippers before measuring, so a carton held over the label at any
        # height would land on it (a breach), and one held anywhere else is not placed. Both score 0.
        return 0.0
    if fully_on_top(s):
        return 1.0
    if partly_on_top(s):
        return 0.5
    return 0.0


def success(objects):
    """Fully on the top, never over the label too low, and still."""
    return score(objects) == 1.0 and objects[SMALL.name].resting()


# The robot itself keeps out too: during the scripted demo no part of either arm may come within 4 cm of the label
# below 25 cm above the table, or over the label at any height (checked offline with the URDF; the scorer sees only
# objects).
ARM_KEEP_OUT = [
    {"name": "shipping label and 4 cm around it", "x": (0.15, 0.33), "y": (-0.02, 0.14), "z_top": TOP + 0.25},
    {"name": "shipping label, any height", "x": (0.19, 0.29), "y": (0.02, 0.10), "z_top": TOP + 1.0},
]

# Where the demo puts the small carton: on the free part, its whole outline at x 22.5..29.5, y -9.5..-3.5 cm, 5 mm
# from the big carton's back and right edges and 5.5 cm short of the label. That far back and right, the arm's wrist
# housing stays clear of the label's strip in the world as well as in the top camera's view.
PLACE_XY = (0.26, -0.065)
TRAVEL = TOP + 0.20
# Where the right arm goes after letting go: well to its own side, so the camera sees the carton and the label.
CLEAR_XY = (0.30, -0.24)
FIRST_MOVE_S = 3.0  # s for the first move from the ready pose: keeps the hand under the envelope's 0.25 m/s


def demo():
    """The right arm picks the small carton from above with its fingers on the 7 cm faces (closing along x), lifts it
    straight up, carries it in one straight line to above the free part of the big carton's top, lowers it straight
    down there and lets go. It then rises and moves well clear to its own side, so the open gripper and the carton are
    in plain view. The carton, the hand and the arm stay clear of the label's strip the whole way, at every height:
    in the world, not only in the top camera's view. The left arm is not needed and holds its ready pose."""
    place = (*PLACE_XY, BIG_TOP + SMALL_SIZE[2] / 2 + 0.003)
    right = pick_place_waypoints(SMALL.pos, place, travel_z=TRAVEL, yaw=np.pi / 2, speed=0.8)
    right[0] = replace(right[0], duration=FIRST_MOVE_S)
    right.append(Waypoint(2.5, (*CLEAR_XY, TRAVEL), FINGER_OPEN, "move clear", np.pi / 2))
    return {"right": right}
