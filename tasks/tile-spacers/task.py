"""On the table, centred between the arms: a grey cement backer board with four loose white 10 cm ceramic tiles
dry-laid on it in a 2 x 2 square, 3 mm apart. The four 3 mm joints between them (far, near, left and right of
the centre) are empty. In front of each arm stands a light wooden block with two parallel slots running away
from the robot; each slot holds a blue 3 mm plastic spacer upright, its top standing 12 mm proud of the block.

Put one spacer in each of the four joints, upright with its faces against the tiles' edges and pushed down onto the
board, without moving the tiles; then take the grippers away from the spacers.

Run it with this folder's run.py: its embodiment gives the tiles and spacers their eased edges and materials
and passes the grippers' positions to the scorer.
"""

from __future__ import annotations

import numpy as np

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the patch ---------------------------------------------------------------------------------------------
BOARD_SIZE = (0.300, 0.300, 0.012)
BOARD_XY = (0.270, 0.000)
BOARD_TOP = TOP + BOARD_SIZE[2]
TILE_SIZE = (0.100, 0.100, 0.007)
TILE_MASS = 0.150  # kg: 70 cm3 of fired ceramic body at about 2.1 g/cm3
TILE_CHAMFER = 0.0008  # eased (cushion) edge on the glazed face
JOINT = 0.003
PATCH_XY = np.array([0.270, 0.000])
PITCH = TILE_SIZE[0] + JOINT
SPAWN_GAP = 0.0002  # objects start this far above what they rest on and settle during the reset

TILES = {
    "tile_far_left": (+1, +1),
    "tile_far_right": (+1, -1),
    "tile_near_left": (-1, +1),
    "tile_near_right": (-1, -1),
}
TILE_START_XY = {name: PATCH_XY + np.array(s) * PITCH / 2 for name, s in TILES.items()}


def _floats(*values) -> tuple[float, ...]:
    """Plain floats: Arena builds the reset pose tensor from these, and NumPy scalars would make it float64."""
    return tuple(float(v) for v in values)


# A joint runs along `axis` (0: x, away from the robot; 1: y, to its left) on the line `line` of the other
# coordinate, between the two tile edges that face across it over `span`.
_EDGE_IN, _EDGE_OUT = JOINT / 2, JOINT / 2 + TILE_SIZE[0]
JOINTS = {
    "far": dict(axis=0, line=PATCH_XY[1], span=(PATCH_XY[0] + _EDGE_IN, PATCH_XY[0] + _EDGE_OUT)),
    "near": dict(axis=0, line=PATCH_XY[1], span=(PATCH_XY[0] - _EDGE_OUT, PATCH_XY[0] - _EDGE_IN)),
    "left": dict(axis=1, line=PATCH_XY[0], span=(PATCH_XY[1] + _EDGE_IN, PATCH_XY[1] + _EDGE_OUT)),
    "right": dict(axis=1, line=PATCH_XY[0], span=(PATCH_XY[1] - _EDGE_OUT, PATCH_XY[1] - _EDGE_IN)),
}

# -- spacers and their blocks ------------------------------------------------------------------------------
SPACER_SIZE = (0.025, 0.003, 0.020)  # length, thickness, height
SPACER_MASS = 0.0015  # kg: 1.5 cm3 of polypropylene
SPACER_CHAMFER = 0.0008
BLOCK_SIZE = (0.060, 0.070, 0.018)  # x, y, height
SLOT_WIDTH, SLOT_DEPTH = 0.004, 0.008
SLOT_OFFSETS = (-0.015, +0.015)  # slot centre lines, y from the block's centre
BLOCK_XY = {"left": (0.270, +0.215), "right": (0.270, -0.215)}
SLOT_FLOOR = TOP + BLOCK_SIZE[2] - SLOT_DEPTH
BLOCK_TOP = TOP + BLOCK_SIZE[2]


def _slot_y(arm: str, inner: bool) -> float:
    """The y of an arm's inner slot (the one nearer the patch) or its outer slot."""
    outward = 1.0 if arm == "left" else -1.0
    return BLOCK_XY[arm][1] + outward * (SLOT_OFFSETS[0] if inner else SLOT_OFFSETS[1])


SPACERS = {
    "spacer_left_inner": ("left", True),
    "spacer_left_outer": ("left", False),
    "spacer_right_inner": ("right", True),
    "spacer_right_outer": ("right", False),
}
SPACER_START = {
    name: _floats(BLOCK_XY[arm][0], _slot_y(arm, inner), SLOT_FLOOR + SPACER_SIZE[2] / 2 + SPAWN_GAP)
    for name, (arm, inner) in SPACERS.items()
}

BOARD_COLOR = (0.42, 0.43, 0.44)
TILE_COLOR = (0.93, 0.92, 0.88)
SPACER_COLOR = (0.10, 0.38, 0.90)
BLOCK_COLOR = (0.86, 0.74, 0.52)


def _block(arm: str) -> list[Box]:
    """A slotted block as fixed boxes: a base under the slots and three bars between and beside them."""
    bx, by = BLOCK_XY[arm]
    sx, sy, _ = BLOCK_SIZE
    base_h = BLOCK_SIZE[2] - SLOT_DEPTH
    parts = [Box(f"block_{arm}_base", pos=_floats(bx, by, TOP + base_h / 2), size=(sx, sy, base_h), color=BLOCK_COLOR)]
    walls = sorted([by - sy / 2, by + sy / 2] + [by + o + s * SLOT_WIDTH / 2 for o in SLOT_OFFSETS for s in (-1, 1)])
    for i in range(3):
        lo, hi = walls[2 * i], walls[2 * i + 1]
        parts.append(
            Box(
                f"block_{arm}_bar{i}",
                pos=_floats(bx, (lo + hi) / 2, TOP + base_h + SLOT_DEPTH / 2),
                size=_floats(sx, hi - lo, SLOT_DEPTH),
                color=BLOCK_COLOR,
            )
        )
    return parts


BOARD = Box("board", pos=_floats(*BOARD_XY, TOP + BOARD_SIZE[2] / 2), size=BOARD_SIZE, color=BOARD_COLOR)
OBJECTS = [
    BOARD,
    *(
        Box(name, pos=_floats(*xy, BOARD_TOP + TILE_SIZE[2] / 2 + SPAWN_GAP), size=TILE_SIZE, mass=TILE_MASS,
            color=TILE_COLOR)
        for name, xy in TILE_START_XY.items()
    ),
    *_block("left"),
    *_block("right"),
    *(Box(name, pos=pos, size=SPACER_SIZE, mass=SPACER_MASS, color=SPACER_COLOR) for name, pos in SPACER_START.items()),
]

# -- scoring -----------------------------------------------------------------------------------------------
LAYOUT_TOLERANCE = 0.001  # every tile corner within 1 mm (a third of the joint) of where it started, in plan
FLAT_TOLERANCE = 0.001  # and no tile corner more than 1 mm off the board
SEATED_TOLERANCE = 0.001  # a seated spacer's lowest point is within 1 mm of the board
IN_JOINT_TOLERANCE = 0.0015  # its centre within 1.5 mm of the joint's centre line
SPAN_GRACE = 0.001  # its whole length between the tiles' edges, give or take 1 mm
UPRIGHT_DEG = 10.0  # its faces within 10 degrees of the joint's faces
HANDS_CLEAR = 0.030  # both grippers' centre points at least 30 mm from every spacer
SUCCESS_HOLD_S = 1.0
PER_SPACER = 0.2

# The embodiment in runtime.py fills this before every scoring call: each arm's TCP (the point between the
# finger pads, 13 mm short of their tips) in the world frame, metres.
_robot: dict = {}


def _corners(state, size) -> np.ndarray:
    rot = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, dtype=float))
    half = np.asarray(size, dtype=float) / 2
    local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    return np.asarray(state.pos, dtype=float) + local @ rot.T


def tile_report(name: str, state) -> dict:
    corners = _corners(state, TILE_SIZE)
    bottom = corners[[0, 2, 4, 6]]  # local z = -1 corners
    start = TILE_START_XY[name] + np.array([[sx, sy] for sx in (-1, 1) for sy in (-1, 1)]) * TILE_SIZE[0] / 2
    shift = float(np.max(np.linalg.norm(bottom[:, :2] - start, axis=1)))
    lift = float(np.max(bottom[:, 2]) - BOARD_TOP)
    return {"corner_shift_m": shift, "corner_lift_m": lift, "in_place": shift <= LAYOUT_TOLERANCE and lift <= FLAT_TOLERANCE}


def spacer_joint(state) -> tuple[str | None, dict]:
    """The joint this spacer is seated in, if any, and what was measured."""
    rot = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, dtype=float))
    corners = _corners(state, SPACER_SIZE)
    pos = np.asarray(state.pos, dtype=float)
    bottom = float(corners[:, 2].min() - BOARD_TOP)
    info = {"bottom_above_board_m": bottom}
    for joint, spec in JOINTS.items():
        axis, other = spec["axis"], 1 - spec["axis"]
        normal = np.zeros(3)
        normal[other] = 1.0
        across = abs(float(rot[:, 1] @ normal))  # the spacer's thickness axis against the joint's normal
        lateral = abs(float(pos[other] - spec["line"]))
        lo, hi = spec["span"]
        inside = bool(corners[:, axis].min() >= lo - SPAN_GRACE and corners[:, axis].max() <= hi + SPAN_GRACE)
        # The far and near joints share a centre line (as do the left and right): the centre picks the joint.
        if lateral <= IN_JOINT_TOLERANCE and lo <= pos[axis] <= hi:
            info.update(joint=joint, lateral_m=lateral, within_span=inside,
                        tilt_deg=float(np.degrees(np.arccos(np.clip(across, -1.0, 1.0)))))
            seated = inside and across >= np.cos(np.radians(UPRIGHT_DEG)) and bottom <= SEATED_TOLERANCE
            return (joint if seated else None), info
    return None, info


def _box_distance(point, state, size) -> float:
    rot = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, dtype=float))
    local = rot.T @ (np.asarray(point, dtype=float) - np.asarray(state.pos, dtype=float))
    outside = np.maximum(np.abs(local) - np.asarray(size, dtype=float) / 2, 0.0)
    return float(np.linalg.norm(outside))


def measure(objects, robot=None) -> dict:
    """Everything the score depends on, from object poses and (for the final release) the grippers' TCPs."""
    robot = _robot if robot is None else robot
    tiles = {name: tile_report(name, objects[name]) for name in TILES}
    layout_ok = all(t["in_place"] for t in tiles.values())
    spacers, filled = {}, set()
    for name in SPACERS:
        joint, info = spacer_joint(objects[name])
        spacers[name] = {**info, "seated_in": joint}
        if joint is not None:
            filled.add(joint)
    tcps = robot.get("tcp") or {}
    clearance = (
        min(_box_distance(tcp, objects[s], SPACER_SIZE) for tcp in tcps.values() for s in SPACERS)
        if len(tcps) == 2
        else None
    )
    hands_clear = clearance is not None and clearance >= HANDS_CLEAR
    still = all(objects[n].resting() for n in (*TILES, *SPACERS))
    n = len(filled)
    return {
        "tiles": tiles,
        "spacers": spacers,
        "joints_filled": sorted(filled),
        "layout_ok": layout_ok,
        "grip_clearance_m": clearance,
        "hands_clear": hands_clear,
        "still": still,
        "success_now": bool(layout_ok and n == len(JOINTS) and hands_clear and still),
        "partial_score": PER_SPACER * n if layout_ok else 0.0,
    }


def success(objects) -> bool:
    return measure(objects)["success_now"]


def score(objects) -> float:
    return float(measure(objects)["partial_score"])


def distance(objects) -> float:
    """Metres still to go: for each empty joint, the nearest spacer's distance from that joint's seated spot."""
    filled = set(measure(objects)["joints_filled"])
    total = 0.0
    for joint, spec in JOINTS.items():
        if joint in filled:
            continue
        spot = np.zeros(3)
        spot[spec["axis"]] = sum(spec["span"]) / 2
        spot[1 - spec["axis"]] = spec["line"]
        spot[2] = BOARD_TOP + SPACER_SIZE[2] / 2
        total += min(float(np.linalg.norm(np.asarray(objects[s].pos) - spot)) for s in SPACERS)
    return total


# -- scripted demonstration (yam_scripted) ---------------------------------------------------------------
# Grip a spacer with the pad tips 3 mm above its block: its top 9 mm is between the pads and its bottom is
# GRIP_DROP (24 mm) below the TCP. Seated, the pad tips are then 3.5 mm above the glaze.
PAD_ABOVE_BLOCK = 0.003
GRASP_Z = BLOCK_TOP + PAD_ABOVE_BLOCK + PAD_TIP_PAST_TCP
GRIP_DROP = GRASP_Z - SLOT_FLOOR
TRAVEL_Z = TOP + 0.090
HOVER_Z = BOARD_TOP + TILE_SIZE[2] + 0.004 + GRIP_DROP  # the spacer's bottom 4 mm above the glaze
PRESS = 0.0005  # command the spacer 0.5 mm into the board so that it bottoms out
SEAT_Z = BOARD_TOP + GRIP_DROP - PRESS
FINGER_APPROACH = -0.008  # pads 16 mm apart: clears a spacer without reaching the next slot
YAW_ALONG_X = 0.0  # fingers close along y: the spacer's length runs along x
YAW_ALONG_Y = {"left": -np.pi / 2, "right": +np.pi / 2}  # fingers close along x (each arm's own wrist sense)


def joint_spot(joint: str) -> tuple[float, float]:
    spec = JOINTS[joint]
    xy = [0.0, 0.0]
    xy[spec["axis"]] = sum(spec["span"]) / 2
    xy[1 - spec["axis"]] = spec["line"]
    return float(xy[0]), float(xy[1])


def pick_waypoints(spacer: str) -> list[Waypoint]:
    """Take a spacer out of its slot: fingers closing along y on its faces, its top 9 mm between the pads."""
    sx, sy, _ = SPACER_START[spacer]
    return [
        Waypoint(2.0, (sx, sy, TRAVEL_Z), FINGER_APPROACH, f"over {spacer}", YAW_ALONG_X),
        Waypoint(1.5, (sx, sy, GRASP_Z), FINGER_APPROACH, "down to the spacer", YAW_ALONG_X),
        Waypoint(0.8, (sx, sy, GRASP_Z), FINGER_CLOSED, "grip", YAW_ALONG_X),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift out", YAW_ALONG_X),
    ]


def place_waypoints(arm: str, joint: str) -> list[Waypoint]:
    """Carry the held spacer over the joint, turned to match it, push it down onto the board, let go."""
    jx, jy = joint_spot(joint)
    yaw = YAW_ALONG_X if JOINTS[joint]["axis"] == 0 else YAW_ALONG_Y[arm]
    return [
        Waypoint(2.0, (jx, jy, TRAVEL_Z), FINGER_CLOSED, f"over the {joint} joint", yaw),
        Waypoint(1.5, (jx, jy, HOVER_Z), FINGER_CLOSED, "lower to the glaze", yaw),
        Waypoint(2.5, (jx, jy, SEAT_Z), FINGER_CLOSED, "push into the joint", yaw),
        Waypoint(0.6, (jx, jy, SEAT_Z), FINGER_APPROACH, "let go", yaw),
        Waypoint(1.2, (jx, jy, TRAVEL_Z), FINGER_APPROACH, "lift clear", yaw),
    ]


def _duration(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def _wait_until(plan: list[Waypoint], t: float, label: str) -> list[Waypoint]:
    """Hold the plan's last pose until time ``t``."""
    last = plan[-1]
    return [Waypoint(t - _duration(plan), last.pos, last.finger, label, last.yaw)] if t > _duration(plan) else []


PARK = {"left": (0.20, 0.24, TOP + 0.15), "right": (0.20, -0.24, TOP + 0.15)}
PARK_S = 2.0


def demo() -> dict[str, list[Waypoint]]:
    """One arm over the patch at a time (two YAM wrists over the 103 mm-apart side joints would collide):
    left joint, right joint, far joint, near joint. Each arm takes its next spacer while the other works and
    waits with it over its own block."""
    left = [*pick_waypoints("spacer_left_inner"), *place_waypoints("left", "left")]
    right = pick_waypoints("spacer_right_inner")
    right += _wait_until(right, _duration(left), "wait for the left arm")
    right += place_waypoints("right", "right")
    left += pick_waypoints("spacer_left_outer")
    left += _wait_until(left, _duration(right), "wait for the right arm")
    left += place_waypoints("left", "far")
    left.append(Waypoint(PARK_S, PARK["left"], FINGER_APPROACH, "park"))
    right += pick_waypoints("spacer_right_outer")
    right += _wait_until(right, _duration(left), "wait for the left arm")
    right += place_waypoints("right", "near")
    right.append(Waypoint(PARK_S, PARK["right"], FINGER_APPROACH, "park"))
    return {"left": left, "right": right}
