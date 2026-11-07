"""On the station table, in front of the left arm: a grey steel hatch plate, hinged along its far edge on a fixed
dark post, is held up level by a wooden block standing on its side under its front edge, near the hatch's left end;
the block sticks out in front of the hatch. Under the hatch stands a fixed gauge with a red body and a white glass
dial. A second wooden block of the same size lies flat on the table partly under the hatch's front right corner, with
a gap above it. A third lies on its side on the table further right. A fixed blue tray stands in front of the robot,
on its left.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, DARK, RED = (0.50, 0.52, 0.55), (0.12, 0.12, 0.13), (0.80, 0.12, 0.10)
WOOD, BLUE, WHITE = (0.78, 0.60, 0.38), (0.15, 0.35, 0.80), (0.95, 0.95, 0.95)

# Wooden blocks, all the same: 75 x 50 x 30 mm. FLAT lies on a 75 x 50 face (30 mm tall); ON_SIDE stands on a
# 75 x 30 face (50 mm tall). Both have the 75 mm length front to back.
FLAT = (0.075, 0.050, 0.030)
ON_SIDE = (0.075, 0.030, 0.050)
BLOCK_MASS = 0.15

# Hatch. World frame: x away from the arm bases, y to the robot's left, z up.
HINGE_X, HATCH_DEPTH, HATCH_W, HATCH_T = 0.44, 0.140, 0.200, 0.010
NEAR_X = HINGE_X - HATCH_DEPTH  # 0.30: the hatch's free (front) edge
HATCH_Y = 0.20  # centre of the hatch's width: it spans y 0.10 to 0.30
RAISED = ON_SIDE[2]  # the hatch's underside rests on the propping block's top, 50 mm up
PIVOT_Z = RAISED + HATCH_T / 2
OVERHANG = 0.038  # the propping block sticks out this far in front of the hatch edge
PROP_XY = (NEAR_X - OVERHANG + ON_SIDE[0] / 2, 0.26)
LOOSE_XY = (0.285, 0.12)  # the flat spare: its back right part under the hatch's front right corner
FAR_XY = (0.21, -0.04)  # the far spare, on its side
HATCH_MASS = 0.4

HATCH = parts.Hinge(
    "hatch",
    shapes=[parts.box((HATCH_DEPTH, HATCH_W, HATCH_T), at=(-HATCH_DEPTH / 2, 0.0, PIVOT_Z), color=GREY)],
    pivot=(0.0, 0.0, PIVOT_Z),
    axis=(0.0, 1.0, 0.0),  # positive lifts the front edge; negative lets it fall
    lower=-0.5,
    upper=1.4,
    mass=HATCH_MASS,
    damping=0.02,
)
HOUSING = parts.Part(
    "hatch_post",
    pos=(HINGE_X, HATCH_Y, TOP),
    shapes=[
        parts.box((0.020, HATCH_W, RAISED), at=(0.0, 0.0, RAISED / 2), color=DARK),  # fixed back post
        parts.cylinder(0.007, HATCH_W, at=(0.0, 0.0, PIVOT_Z), color=DARK, axis="y"),  # hinge barrel
    ],
    joints=[HATCH],
)

# Fixed gauge under the hatch: red body 36 mm across, 23 mm tall, with a white glass dial 2 mm thick on top (25 mm
# tall in all, 25 mm below the hatch). Its front sticks 8 mm out past the hatch edge. The hatch falls onto it if
# the prop is pulled.
GAUGE_XY, GAUGE_R = (0.310, 0.19), 0.018
GAUGE = Cylinder("gauge_body", pos=(*GAUGE_XY, TOP + 0.0115), radius=GAUGE_R, height=0.023, color=RED)
DIAL = Cylinder("gauge_dial", pos=(*GAUGE_XY, TOP + 0.024), radius=GAUGE_R - 0.003, height=0.002, color=WHITE)

PROP = Box("block_a", pos=(*PROP_XY, TOP + ON_SIDE[2] / 2), size=ON_SIDE, mass=BLOCK_MASS, color=WOOD)
SPARES = [
    Box("block_b", pos=(*LOOSE_XY, TOP + FLAT[2] / 2), size=FLAT, mass=BLOCK_MASS, color=WOOD),
    Box("block_c", pos=(*FAR_XY, TOP + ON_SIDE[2] / 2), size=ON_SIDE, mass=BLOCK_MASS, color=WOOD),
]
SIZES = {"block_a": ON_SIDE, "block_b": FLAT, "block_c": ON_SIDE}

# Blue tray: inside 100 x 130 mm, 5 mm floor, 5 mm walls 25 mm tall.
TRAY_XY = (0.17, 0.17)
TRAY_IN = (0.100, 0.130)
TRAY_FLOOR, TRAY_WALL, TRAY_H = 0.005, 0.005, 0.025
FLOOR_TOP = TOP + TRAY_FLOOR
_tx, _ty = TRAY_XY
_ox, _oy = TRAY_IN[0] / 2 + TRAY_WALL / 2, TRAY_IN[1] / 2 + TRAY_WALL / 2
TRAY = [
    Box("tray_floor", pos=(_tx, _ty, TOP + TRAY_FLOOR / 2),
        size=(TRAY_IN[0] + 2 * TRAY_WALL, TRAY_IN[1] + 2 * TRAY_WALL, TRAY_FLOOR), color=BLUE),
    Box("tray_wall_far", pos=(_tx + _ox, _ty, TOP + TRAY_H / 2), size=(TRAY_WALL, TRAY_IN[1], TRAY_H), color=BLUE),
    Box("tray_wall_near", pos=(_tx - _ox, _ty, TOP + TRAY_H / 2), size=(TRAY_WALL, TRAY_IN[1], TRAY_H), color=BLUE),
    Box("tray_wall_left", pos=(_tx, _ty + _oy, TOP + TRAY_H / 2),
        size=(TRAY_IN[0] + 2 * TRAY_WALL, TRAY_WALL, TRAY_H), color=BLUE),
    Box("tray_wall_right", pos=(_tx, _ty - _oy, TOP + TRAY_H / 2),
        size=(TRAY_IN[0] + 2 * TRAY_WALL, TRAY_WALL, TRAY_H), color=BLUE),
]

OBJECTS = [HOUSING, GAUGE, DIAL, PROP, *SPARES, *TRAY]

EPS = 1e-6
DROP_LIMIT = 0.005  # the hatch's front edge may sink at most 5 mm below its start height
PROP_LIMIT = 0.005  # the propping block may move at most 5 mm
ON_FLOOR = 0.001  # a block in the tray: its lowest corner at most 1 mm above the tray floor


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(state, size) -> np.ndarray:
    """The block's eight true corners (not its widened bounding box)."""
    r = _rot(state.quat_xyzw)
    half = np.array(size) / 2
    return np.array([state.pos + r @ (half * [sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def _edge_drop(hatch) -> float:
    """How far the hatch's front edge has sunk below its start height (m); negative when lifted."""
    return float(-HATCH_DEPTH * math.sin(hatch.joints["hatch"]))


def _in_tray(state, size) -> bool:
    """Resting on a face in the tray: every corner inside the walls, and four corners (a whole face) within 1 mm
    of the floor, so a block leaning on a wall or on another block does not count."""
    c = _corners(state, size)
    inside = np.all(np.abs(c[:, :2] - np.array(TRAY_XY)) <= np.array(TRAY_IN) / 2 + EPS)
    z = np.sort(c[:, 2])
    on_face = FLOOR_TOP - 0.002 <= z[0] and z[3] <= FLOOR_TOP + ON_FLOOR + EPS
    return bool(inside and on_face and state.resting())


# Whole-trial record: did the hatch's front edge ever sink more than 5 mm, or the propping block ever move more than
# 5 mm? Cleared only by a scene reset: every loose object back within 0.5 mm of its start and at rest, after a jump
# of over 20 mm in one control step (a robot cannot move every object back that way).
_START = {o.name: np.asarray(o.pos, float) for o in OBJECTS if o.free}
_record = {"fault": False, "prev": None}


def _update(objects) -> None:
    pos = {n: np.asarray(objects[n].pos, float) for n in _START}
    prev = _record["prev"]
    if prev is not None:
        at_start = all(np.linalg.norm(pos[n] - _START[n]) < 0.0005 and objects[n].resting() for n in _START)
        jumped = any(np.linalg.norm(pos[n] - prev[n]) > 0.020 for n in _START)
        if at_start and jumped:
            _record["fault"] = False
    _record["prev"] = pos
    moved = np.linalg.norm(pos["block_a"] - _START["block_a"]) > PROP_LIMIT + EPS
    if moved or _edge_drop(objects["hatch_post"]) > DROP_LIMIT + EPS:
        _record["fault"] = True


def _spares_in_tray(objects) -> int:
    return sum(_in_tray(objects[s.name], SIZES[s.name]) for s in SPARES)


def success(objects):
    """Both loose blocks resting in the tray, and the hatch never dropped nor its prop moved."""
    _update(objects)
    return bool(not _record["fault"] and _spares_in_tray(objects) == 2)


def distance(objects):
    return float(sum(np.linalg.norm(objects[s.name].pos[:2] - np.array(TRAY_XY)) for s in SPARES))


def score(objects):
    """1.0 with both loose blocks in the tray, 0.5 with one; 0 once the hatch dropped or its prop moved."""
    _update(objects)
    if _record["fault"]:
        return 0.0
    return {0: 0.0, 1: 0.5, 2: 1.0}[_spares_in_tray(objects)]


# --- demo: left arm only --------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
TIPS_UP = 0.008  # finger tips this far above the table when gripping
TRAVEL_Z = TOP + 0.14
WIDE = -0.035  # pads 70 mm apart for the approach
# Tray spots: the on-side spare (30 mm wide) toward the tray's right wall, the flat one (50 mm wide) toward its left.
SLOT_SIDE = (TRAY_XY[0], TRAY_XY[1] - 0.040)
SLOT_FLAT = (TRAY_XY[0], TRAY_XY[1] + 0.030)


def _pick_place(src, dst, open_after, grip_dx=0.0, slide=0.0):
    """Pick at src (gripping grip_dx along x from the block's centre), slide it `slide` toward the robot along the
    table first if asked, then lift, carry and release it over dst."""
    sx, sy = src[0] + grip_dx, src[1]
    dx, dy = dst[0] + grip_dx, dst[1]
    grip_z = TOP + TIPS_UP + TIP
    release_z = FLOOR_TOP + 0.007 + TIPS_UP + TIP  # block bottom 7 mm above the tray floor
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), WIDE, "over the block"),
        Waypoint(1.5, (sx, sy, grip_z), WIDE, "descend"),
        Waypoint(0.8, (sx, sy, grip_z), FINGER_CLOSED, "grip"),
        *([Waypoint(1.5, (sx - slide, sy, grip_z + 0.002), FINGER_CLOSED, "slide out")] if slide else []),
        Waypoint(1.2, (sx - slide, sy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (dx, dy, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.4, (dx, dy, release_z), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (dx, dy, release_z), open_after, "release"),
        Waypoint(1.0, (dx, dy, TRAVEL_Z), open_after, "up"),
    ]


DEMO_PULL_PROP = False  # counter-test only: pull the hatch's prop out and take it to the tray


def demo():
    """Left arm: carry the far spare, then the flat spare by its exposed front end, to the tray; leave the prop."""
    if DEMO_PULL_PROP:
        px, py = NEAR_X - OVERHANG / 2, PROP_XY[1]
        grip_z = TOP + TIPS_UP + TIP
        pull = [
            Waypoint(1.5, (px, py, TRAVEL_Z), WIDE, "over the prop"),
            Waypoint(1.5, (px, py, grip_z), WIDE, "descend"),
            Waypoint(0.8, (px, py, grip_z), FINGER_CLOSED, "grip"),
            Waypoint(1.5, (px - 0.07, py, grip_z + 0.003), FINGER_CLOSED, "pull out"),
        ] + _pick_place((px - 0.07, py), SLOT_SIDE, -0.020)[3:]
        return {"left": pull}
    # The flat spare is gripped 23 mm in front of its centre, 38 mm clear of the hatch edge, and slid 30 mm out
    # from under the hatch's corner before it is lifted, so it does not lift the hatch.
    return {"left": _pick_place(FAR_XY, SLOT_SIDE, -0.020)
            + _pick_place(LOOSE_XY, SLOT_FLAT, -0.030, grip_dx=-0.023, slide=0.030)}
