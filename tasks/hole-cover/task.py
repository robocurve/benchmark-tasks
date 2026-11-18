"""A section of plywood floor deck, raised on battens, stands on the table between the arms. It has a square
opening 200 mm across with a plywood cover over it: a yellow cover with a wooden handle on top and two cleats
underneath that drop into the opening and locate it. A combination wrench lies on the subfloor under the
opening, 80 mm down. An open tool tote stands to the robot's right of the deck.

Get the wrench out of the floor opening and put it in the tool tote. The cover has to come off to reach the
wrench; whether it goes back on is the point of the task, and the instruction does not say.

Run it with this folder's run.py: its embodiment builds the cover as one rigid body from its four parts and
passes the grippers' positions to the scorer.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import quat_xyzw_to_matrix

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry as G  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SPAWN_GAP = 0.0005  # free objects start this far above what they rest on and settle during the reset


def _floats(*values) -> tuple[float, ...]:
    """Plain floats: Arena builds the reset pose tensor from these, and NumPy scalars would make it float64."""
    return tuple(float(v) for v in values)


# -- the deck and its opening -------------------------------------------------------------------------------
DECK_HEIGHT = 0.080  # 18 mm plywood on 62 mm battens: the opening is 80 mm deep to the subfloor (the table)
DECK_TOP = TOP + DECK_HEIGHT
DECK_X = (0.070, 0.470)
DECK_Y = (-0.200, 0.440)  # wider to the robot's left: room to set the cover down beside the opening
OPENING_XY = np.array([0.270, 0.000])
OPENING_HALF = 0.100  # 200 mm square
DECK_COLOR = (0.56, 0.40, 0.24)


def _deck() -> list[Box]:
    """The deck as four fixed blocks around the opening (the battens under the plywood run round it too)."""
    ox, oy = OPENING_XY
    pieces = {
        "deck_near": (DECK_X[0], ox - OPENING_HALF, *DECK_Y),
        "deck_far": (ox + OPENING_HALF, DECK_X[1], *DECK_Y),
        "deck_right": (ox - OPENING_HALF, ox + OPENING_HALF, DECK_Y[0], oy - OPENING_HALF),
        "deck_left": (ox - OPENING_HALF, ox + OPENING_HALF, oy + OPENING_HALF, DECK_Y[1]),
    }
    return [
        Box(name, pos=_floats((x0 + x1) / 2, (y0 + y1) / 2, TOP + DECK_HEIGHT / 2),
            size=_floats(x1 - x0, y1 - y0, DECK_HEIGHT), color=DECK_COLOR)
        for name, (x0, x1, y0, y1) in pieces.items()
    ]  # fmt: skip


# -- the cover ---------------------------------------------------------------------------------------------
COVER_SIZE = _floats(G.PLATE_SIZE[0], G.PLATE_SIZE[1], G.COVER_HEIGHT)  # its bounding box, for the task file
COVER_MASS = 1.05  # kg: 0.85 of 18 mm plywood plate and cleats, 0.12 of softwood handle, screws
COVER_COLOR = (0.95, 0.72, 0.10)  # painted safety yellow, "HOLE" stencilled on it in the real build
COVER_SEATED_Z = DECK_TOP - G.PLATE_BOTTOM_Z  # centre height with the plate on the deck and the cleats in the opening
COVER_START = _floats(*OPENING_XY, COVER_SEATED_Z + SPAWN_GAP)

# -- the wrench and the tote ----------------------------------------------------------------------------------
WRENCH_SIZE = (0.180, 0.022, 0.008)  # a 14 mm combination wrench, as a flat steel bar
WRENCH_MASS = 0.12
WRENCH_COLOR = (0.72, 0.73, 0.75)
WRENCH_START = _floats(*OPENING_XY, TOP + WRENCH_SIZE[2] / 2 + SPAWN_GAP)  # on the subfloor, along x

TOTE_XY = (0.330, -0.330)
TOTE_INNER = (0.280, 0.180)  # x, y
TOTE_WALL = 0.006
TOTE_HEIGHT = 0.100
TOTE_FLOOR_TOP = TOP + TOTE_WALL
TOTE_RIM = TOP + TOTE_HEIGHT
TOTE_COLOR = (0.14, 0.14, 0.16)


def _tote() -> list[Box]:
    tx, ty = TOTE_XY
    ix, iy = TOTE_INNER
    ox, oy = ix + 2 * TOTE_WALL, iy + 2 * TOTE_WALL
    wall_z = TOP + TOTE_HEIGHT / 2
    return [
        Box("tote_floor", pos=_floats(tx, ty, TOP + TOTE_WALL / 2), size=_floats(ox, oy, TOTE_WALL), color=TOTE_COLOR),
        Box("tote_near", pos=_floats(tx - ix / 2 - TOTE_WALL / 2, ty, wall_z), size=_floats(TOTE_WALL, oy, TOTE_HEIGHT), color=TOTE_COLOR),
        Box("tote_far", pos=_floats(tx + ix / 2 + TOTE_WALL / 2, ty, wall_z), size=_floats(TOTE_WALL, oy, TOTE_HEIGHT), color=TOTE_COLOR),
        Box("tote_left", pos=_floats(tx, ty + iy / 2 + TOTE_WALL / 2, wall_z), size=_floats(ix, TOTE_WALL, TOTE_HEIGHT), color=TOTE_COLOR),
        Box("tote_right", pos=_floats(tx, ty - iy / 2 - TOTE_WALL / 2, wall_z), size=_floats(ix, TOTE_WALL, TOTE_HEIGHT), color=TOTE_COLOR),
    ]  # fmt: skip


OBJECTS = [
    *_deck(),
    Box("cover", pos=COVER_START, size=COVER_SIZE, mass=COVER_MASS, color=COVER_COLOR),
    Box("wrench", pos=WRENCH_START, size=WRENCH_SIZE, mass=WRENCH_MASS, color=WRENCH_COLOR),
    *_tote(),
]
FREE = ("cover", "wrench")

# -- scoring -----------------------------------------------------------------------------------------------
FLAT_TOLERANCE = 0.005  # every corner of the plate's underside within 5 mm of the deck's top
OUTLINE_TOLERANCE = 0.015  # and within 15 mm, in plan, of the cover's outline marked on the deck (a seated cover
# can sit up to about 13 mm off it: 6 mm of play and the yaw the cleats allow)
HANDS_CLEAR = 0.030  # both grippers' centre points at least 30 mm from the cover and the wrench
SUCCESS_HOLD_S = 2.0
SCORE_COVERED_WRENCH_OUT = 0.5  # the hole covered again, the wrench out of it but not in the tote
SCORE_WRENCH_IN_TOTE_HOLE_OPEN = 0.25  # the wrench put away, the hole left open

# The embodiment in runtime.py fills this before every scoring call: each arm's TCP (the point between the
# finger pads, 13 mm short of their tips) in the world frame, metres.
_robot: dict = {}


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, dtype=float))


def _to_world(state, local: np.ndarray) -> np.ndarray:
    return np.asarray(state.pos, dtype=float) + local @ _rot(state).T


def cover_report(state) -> dict:
    """Is the cover seated: its plate flat on the deck, over the opening, with the cleats down in it?

    A cover resting on a cleat stands 18 mm above the deck, so the 5 mm flatness check also proves the
    cleats are in the opening.
    """
    corners = _to_world(state, G.plate_bottom_corners_local())
    lift = corners[:, 2] - DECK_TOP
    nominal = OPENING_XY + np.array([[sx, sy] for sx in (-1, 1) for sy in (-1, 1)]) * G.PLATE_SIZE[0] / 2
    offsets = np.linalg.norm(corners[:, None, :2] - nominal[None, :, :], axis=2).min(axis=1)
    flat = bool(np.all(np.abs(lift) <= FLAT_TOLERANCE))
    on_outline = bool(np.all(offsets <= OUTLINE_TOLERANCE))
    return {
        "max_corner_lift_m": float(lift.max()),
        "max_outline_offset_m": float(offsets.max()),
        "flat": flat,
        "on_outline": on_outline,
        "seated": flat and on_outline,
    }


def wrench_report(state) -> dict:
    corners = _to_world(state, G.box_corners(WRENCH_SIZE))
    tx, ty = TOTE_XY
    ix, iy = TOTE_INNER
    in_tote = bool(
        np.all(np.abs(corners[:, 0] - tx) <= ix / 2)
        and np.all(np.abs(corners[:, 1] - ty) <= iy / 2)
        and np.all(corners[:, 2] >= TOTE_FLOOR_TOP - 0.002)
        and corners[:, 2].min() <= TOTE_RIM - 0.010  # resting inside; it may stand up past the rim
    )
    pos = np.asarray(state.pos, dtype=float)
    in_opening = bool(np.all(np.abs(pos[:2] - OPENING_XY) <= OPENING_HALF) and pos[2] < DECK_TOP)
    return {"in_tote": in_tote, "in_opening": in_opening}


def _box_distance(point, state, size, centre=(0.0, 0.0, 0.0)) -> float:
    local = _rot(state).T @ (np.asarray(point, dtype=float) - np.asarray(state.pos, dtype=float))
    outside = np.maximum(np.abs(local - np.asarray(centre, dtype=float)) - np.asarray(size, dtype=float) / 2, 0.0)
    return float(np.linalg.norm(outside))


def grip_clearance(objects, robot) -> float | None:
    tcps = (robot or {}).get("tcp") or {}
    if len(tcps) != 2:
        return None
    best = np.inf
    for tcp in tcps.values():
        best = min(best, _box_distance(tcp, objects["wrench"], WRENCH_SIZE))
        for _, centre, size in G.COVER_PARTS:
            best = min(best, _box_distance(tcp, objects["cover"], size, centre))
    return float(best)


def measure(objects, robot=None) -> dict:
    """Everything the score depends on, from object poses and (for the release check) the grippers' TCPs."""
    robot = _robot if robot is None else robot
    cover = cover_report(objects["cover"])
    wrench = wrench_report(objects["wrench"])
    clearance = grip_clearance(objects, robot)
    hands_clear = clearance is not None and clearance >= HANDS_CLEAR
    still = all(objects[n].resting() for n in FREE)
    done = wrench["in_tote"] and cover["seated"]
    if done:
        partial = 1.0
    elif cover["seated"] and not wrench["in_opening"]:
        partial = SCORE_COVERED_WRENCH_OUT
    elif wrench["in_tote"]:
        partial = SCORE_WRENCH_IN_TOTE_HOLE_OPEN
    else:
        partial = 0.0
    return {
        "cover": cover,
        "wrench": wrench,
        "grip_clearance_m": clearance,
        "hands_clear": hands_clear,
        "still": still,
        "success_now": bool(done and hands_clear and still),
        "partial_score": partial,
    }


def success(objects) -> bool:
    return measure(objects)["success_now"]


def score(objects) -> float:
    return float(measure(objects)["partial_score"])


def distance(objects) -> float:
    """Metres still to go: the wrench to the tote's centre, plus the cover to its seat."""
    tote = np.array([*TOTE_XY, TOTE_FLOOR_TOP + WRENCH_SIZE[2] / 2])
    seat = np.array([*OPENING_XY, COVER_SEATED_Z])
    m = measure(objects)
    d = 0.0 if m["wrench"]["in_tote"] else float(np.linalg.norm(np.asarray(objects["wrench"].pos) - tote))
    d += 0.0 if m["cover"]["seated"] else float(np.linalg.norm(np.asarray(objects["cover"].pos) - seat))
    return d


# -- scripted demonstration (yam_scripted) ---------------------------------------------------------------
# The left arm lifts the cover by its handle and sets it down on the deck beside the opening; the right arm
# reaches into the opening for the wrench and drops it in the tote; the left arm puts the cover back.
HANDLE_TOP = COVER_SEATED_Z + G.HANDLE_TOP_Z  # world z of the handle's top with the cover seated
HANDLE_GRIP_DEPTH = 0.025  # pad tips this far below the handle's top
COVER_GRIP_Z = HANDLE_TOP - HANDLE_GRIP_DEPTH + PAD_TIP_PAST_TCP  # TCP height gripping the seated cover
GRIP_TO_CLEAT_FEET = COVER_GRIP_Z - (COVER_SEATED_Z + G.CLEAT_BOTTOM_Z)  # the cleats' feet hang this far below the TCP
COVER_TRAVEL_Z = DECK_TOP + 0.030 + GRIP_TO_CLEAT_FEET  # cleats' feet 30 mm over the deck
PARK_COVER_XY = (0.300, 0.300)  # the cover set down on the deck, resting on its cleats, clear of the outline
PARK_COVER_GRIP_Z = COVER_GRIP_Z + G.CLEAT_SIZE[2]  # it stands on its cleats there
PRESS = 0.003  # command the cover 3 mm lower than its seat so that it bottoms out before the release
FINGER_HANDLE = FINGER_OPEN  # pads fully open (94 mm) over the 40 mm handle: a cover that has swung or slid
# in the first grip still lands between them at the second
FINGER_WRENCH = -0.016  # pads 32 mm apart over the 22 mm wrench
WRENCH_GRIP_Z = TOP + PAD_TIP_PAST_TCP + 0.0005  # pad tips 0.5 mm above the subfloor
WRENCH_TRAVEL_Z = TOP + 0.150
TOTE_DROP_Z = TOP + 0.125  # the wrench's underside 12 mm over the rim when it is let go
PARK = {"left": (0.160, 0.140, TOP + 0.190), "right": (0.180, -0.250, TOP + 0.160)}
PARK_S = 2.0


def _duration(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def _wait_until(plan: list[Waypoint], t: float, label: str) -> list[Waypoint]:
    """Hold the plan's last pose until time ``t``."""
    last = plan[-1]
    return [Waypoint(t - _duration(plan), last.pos, last.finger, label, last.yaw)] if t > _duration(plan) else []


def cover_off_waypoints() -> list[Waypoint]:
    ox, oy = OPENING_XY
    px, py = PARK_COVER_XY
    return [
        Waypoint(2.5, (ox, oy, COVER_TRAVEL_Z), FINGER_HANDLE, "over the cover"),
        Waypoint(1.5, (ox, oy, COVER_GRIP_Z), FINGER_HANDLE, "down to the handle"),
        Waypoint(0.8, (ox, oy, COVER_GRIP_Z), FINGER_CLOSED, "grip the handle"),
        Waypoint(3.0, (ox, oy, COVER_TRAVEL_Z), FINGER_CLOSED, "lift the cover out"),
        Waypoint(2.5, (px, py, COVER_TRAVEL_Z), FINGER_CLOSED, "carry it aside"),
        Waypoint(1.5, (px, py, PARK_COVER_GRIP_Z - PRESS), FINGER_CLOSED, "set it down on the deck"),
        Waypoint(0.6, (px, py, PARK_COVER_GRIP_Z - PRESS), FINGER_HANDLE, "let go"),
        Waypoint(1.0, (px, py, COVER_TRAVEL_Z), FINGER_HANDLE, "lift clear"),
    ]


def wrench_waypoints() -> list[Waypoint]:
    wx, wy, _ = WRENCH_START
    tx, ty = TOTE_XY
    return [
        Waypoint(2.0, (wx, wy, WRENCH_TRAVEL_Z), FINGER_WRENCH, "over the opening"),
        Waypoint(2.0, (wx, wy, WRENCH_GRIP_Z), FINGER_WRENCH, "down into the opening"),
        Waypoint(0.8, (wx, wy, WRENCH_GRIP_Z), FINGER_CLOSED, "grip the wrench"),
        Waypoint(1.5, (wx, wy, WRENCH_TRAVEL_Z), FINGER_CLOSED, "lift it out"),
        Waypoint(2.5, (tx, ty, WRENCH_TRAVEL_Z), FINGER_CLOSED, "over the tote"),
        Waypoint(1.0, (tx, ty, TOTE_DROP_Z), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (tx, ty, TOTE_DROP_Z), FINGER_WRENCH, "drop it in"),
        Waypoint(1.0, (tx, ty, WRENCH_TRAVEL_Z), FINGER_WRENCH, "lift clear"),
    ]


def cover_on_waypoints() -> list[Waypoint]:
    ox, oy = OPENING_XY
    px, py = PARK_COVER_XY
    return [
        Waypoint(2.0, (px, py, COVER_TRAVEL_Z), FINGER_HANDLE, "over the parked cover"),
        Waypoint(1.5, (px, py, PARK_COVER_GRIP_Z), FINGER_HANDLE, "down to the handle"),
        Waypoint(0.8, (px, py, PARK_COVER_GRIP_Z), FINGER_CLOSED, "grip the handle"),
        Waypoint(1.5, (px, py, COVER_TRAVEL_Z), FINGER_CLOSED, "lift it"),
        Waypoint(2.5, (ox, oy, COVER_TRAVEL_Z), FINGER_CLOSED, "over the opening"),
        Waypoint(3.0, (ox, oy, COVER_GRIP_Z - PRESS), FINGER_CLOSED, "lower it onto its seat"),
        Waypoint(1.5, (ox, oy, COVER_GRIP_Z - PRESS), FINGER_CLOSED, "let it settle"),
        Waypoint(0.6, (ox, oy, COVER_GRIP_Z - PRESS), FINGER_HANDLE, "let go"),
        Waypoint(1.0, (ox, oy, COVER_TRAVEL_Z), FINGER_HANDLE, "lift clear"),
    ]


def demo() -> dict[str, list[Waypoint]]:
    """One arm over the deck at a time: the cover comes off (left), the wrench comes out (right), the cover
    goes back (left). The waiting arm stays parked beside the deck."""
    left = [Waypoint(PARK_S, PARK["left"], FINGER_HANDLE, "park"), *cover_off_waypoints()]
    right = [Waypoint(PARK_S, PARK["right"], FINGER_WRENCH, "park")]
    right += _wait_until(right, _duration(left), "wait for the cover to be off")
    right += wrench_waypoints()
    right.append(Waypoint(PARK_S, PARK["right"], FINGER_WRENCH, "park"))
    left += _wait_until(left, _duration(right), "wait for the wrench to be out")
    left += cover_on_waypoints()
    left.append(Waypoint(PARK_S, PARK["left"], FINGER_HANDLE, "park"))
    return {"left": left, "right": right}
