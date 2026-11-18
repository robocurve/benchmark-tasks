"""A section of wall stands across the far side of the table, with two screws in it 30 cm apart at a marked
height, their pan heads standing 6 mm proud. A dark oak coat rack with three cast hooks stands on the table in
front of the wall, hooks toward the robot, resting forward on its hooks. Two keyhole slots are routed into its
back, 30 cm apart, matching the screws.

Hang the coat rack on the wall: lift it, bring its keyholes over the screw heads, push it flat to the wall so
the heads enter the slots, and lower it until the screw shanks sit in the slots' throats and it hangs level on
both. The slots are behind the rack, so both have to find their screw at once without being seen.

Run it with this folder's run.py: its embodiment builds the rack as one rigid body from its parts, gives the
screws their round shanks and domed heads, and passes the grippers' positions to the scorer.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import quat_xyzw_to_matrix

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry as G  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SPAWN_GAP = 0.0005  # the rack starts this far above the table and settles during the reset


def _floats(*values) -> tuple[float, ...]:
    """Plain floats: Arena builds the reset pose tensor from these, and NumPy scalars would make it float64."""
    return tuple(float(v) for v in values)


# -- the wall section and its screws --------------------------------------------------------------------
WALL_X = 0.380  # the wall's face (the side toward the robot)
WALL_THICKNESS = 0.018
WALL_WIDTH = 0.600
WALL_HEIGHT = 0.200
WALL_COLOR = (0.88, 0.86, 0.80)  # painted off-white
SCREW_Z = TOP + 0.100  # the screws' centre line
SCREW_Y = G.KEYHOLE_Y
SCREW_COLOR = (0.80, 0.80, 0.82)
MARK_Y = (-0.230, 0.230)  # pencil ticks at each end of where the rack's top edge goes
MARK_COLOR = (0.20, 0.20, 0.22)

# -- the rack ---------------------------------------------------------------------------------------------
RACK_SIZE = _floats(*G.RACK_SIZE)  # its bounding box, for the task file
RACK_MASS = 0.70  # kg: 0.56 of oak rail, three cast aluminium hooks of about 0.045 each
RACK_COLOR = (0.22, 0.13, 0.07)  # dark-stained oak; the hooks are black
HANG_CENTRE = np.array([WALL_X - G.BACK_X, 0.0, SCREW_Z - G.REST_Z])  # hanging on both screws
HANG_TOP_Z = float(HANG_CENTRE[2] + G.RAIL_HEIGHT / 2)  # the marked height of the rack's top edge
START_BACK_GAP = 0.050  # the rack stands upright on its bottom edge with its back 50 mm from the wall
START_CENTRE = np.array([WALL_X - START_BACK_GAP - G.BACK_X, 0.0, TOP + G.RAIL_HEIGHT / 2 + SPAWN_GAP])


def start_point(local) -> np.ndarray:
    """A point of the rack (local frame) in the world, at the start."""
    return START_CENTRE + np.asarray(local, dtype=float)


def _wall() -> list[Box]:
    parts = [
        Box("wall", pos=_floats(WALL_X + WALL_THICKNESS / 2, 0.0, TOP + WALL_HEIGHT / 2),
            size=_floats(WALL_THICKNESS, WALL_WIDTH, WALL_HEIGHT), color=WALL_COLOR),
    ]  # fmt: skip
    # The screws are declared as boxes of their exposed envelope; runtime.py gives them their real shapes.
    for name, y in zip(("screw_left", "screw_right"), SCREW_Y):
        parts.append(Box(name, pos=_floats(WALL_X - G.SCREW_PROUD / 2, y, SCREW_Z),
                         size=_floats(G.SCREW_PROUD, G.SCREW_HEAD_D, G.SCREW_HEAD_D), color=SCREW_COLOR))  # fmt: skip
    for name, y in zip(("mark_left", "mark_right"), MARK_Y):
        parts.append(Box(name, pos=_floats(WALL_X - 0.0005, y, HANG_TOP_Z), size=_floats(0.001, 0.020, 0.002),
                         color=MARK_COLOR))  # fmt: skip
    return parts


OBJECTS = [
    *_wall(),
    Box("rack", pos=_floats(*START_CENTRE), size=RACK_SIZE, mass=RACK_MASS, color=RACK_COLOR),
]
FREE = ("rack",)

# -- scoring -----------------------------------------------------------------------------------------------
THROAT_PLAY = (G.THROAT - G.SCREW_SHANK_D) / 2  # 1 mm each side of the shank in the throat
REST_TOLERANCE = 0.0015  # the screw within 1.5 mm of where the rack rests on it
FLUSH_TOLERANCE = 0.004  # the rack's back within 4 mm of the wall (a hung rack can sit 2.5 mm off: the play under the heads)
HEIGHT_TOLERANCE = 0.005  # its top edge within 5 mm of the marked height
LEVEL_DEG = 1.0
OFF_TABLE = 0.010  # hanging means its lowest point is at least 10 mm above the table
HANDS_CLEAR = 0.030  # both grippers' centre points at least 30 mm from the rack
SUCCESS_HOLD_S = 2.0
SCORE_ONE_KEYHOLE = 0.5

# The embodiment in runtime.py fills this before every scoring call: each arm's TCP (the point between the
# finger pads, 13 mm short of their tips) in the world frame, metres.
_robot: dict = {}


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, dtype=float))


def _to_world(state, local: np.ndarray) -> np.ndarray:
    return np.asarray(state.pos, dtype=float) + np.atleast_2d(local) @ _rot(state).T


def _to_local(state, world) -> np.ndarray:
    return _rot(state).T @ (np.asarray(world, dtype=float) - np.asarray(state.pos, dtype=float))


def keyhole_report(state, j: int) -> dict:
    """Where screw ``j`` is in keyhole ``j``'s frame: in its throat (hanging), in its entry pocket, or neither."""
    ky = G.KEYHOLE_Y[j]
    shank = _to_local(state, [WALL_X - G.LIP / 2, SCREW_Y[j], SCREW_Z])  # the shank where it crosses the lip
    dy, dz, dx = shank[1] - ky, shank[2], shank[0] - (G.BACK_X - G.LIP / 2)
    in_throat = bool(  # in the throat and resting on the screw, not part way up with the rack still held
        abs(dy) <= THROAT_PLAY + 0.0005 and G.REST_Z - REST_TOLERANCE <= dz <= G.REST_Z + 0.001 and abs(dx) <= G.LIP / 2 + 0.002
    )
    in_entry = bool(abs(dy) <= G.KEYHOLE_ENTRY / 2 and G.ENTRY_BOTTOM_Z <= dz <= G.ENTRY_TOP_Z and abs(dx) <= 0.006)
    return {"in_throat": in_throat, "in_entry": in_entry, "lateral_m": float(dy), "rise_m": float(dz)}


def rack_report(state) -> dict:
    rot = _rot(state)
    back = _to_world(state, G.back_face_corners_local())
    corners = np.concatenate([_to_world(state, G.box_corners(size, centre)) for _, centre, size in G.rack_parts()])
    gap = WALL_X - back[:, 0]  # each back corner's distance in front of the wall
    flush = bool(np.all(gap <= FLUSH_TOLERANCE) and np.all(gap >= -0.001))
    top = float(back[:, 2].max())
    at_height = bool(abs(top - HANG_TOP_Z) <= HEIGHT_TOLERANCE)
    level = bool(abs(rot[2, 1]) <= np.sin(np.radians(LEVEL_DEG)))  # the rail's long axis within 1 degree of level
    off_table = bool(corners[:, 2].min() >= TOP + OFF_TABLE)
    keyholes = [keyhole_report(state, j) for j in range(len(G.KEYHOLE_Y))]
    hanging = [k["in_throat"] for k in keyholes]
    return {
        "keyholes": keyholes,
        "hanging_on": int(sum(hanging)),
        "max_gap_to_wall_m": float(gap.max()),
        "top_height_error_m": float(top - HANG_TOP_Z),
        "tilt_deg": float(np.degrees(np.arcsin(np.clip(abs(rot[2, 1]), 0.0, 1.0)))),
        "flush": flush,
        "at_height": at_height,
        "level": level,
        "off_table": off_table,
        "hung": bool(all(hanging) and off_table and flush and at_height and level),
    }


def _box_distance(point, state, size, centre=(0.0, 0.0, 0.0)) -> float:
    local = _to_local(state, point)
    outside = np.maximum(np.abs(local - np.asarray(centre, dtype=float)) - np.asarray(size, dtype=float) / 2, 0.0)
    return float(np.linalg.norm(outside))


def grip_clearance(objects, robot) -> float | None:
    tcps = (robot or {}).get("tcp") or {}
    if len(tcps) != 2:
        return None
    return float(min(_box_distance(tcp, objects["rack"], size, centre)
                     for tcp in tcps.values() for _, centre, size in G.rack_parts()))  # fmt: skip


def measure(objects, robot=None) -> dict:
    """Everything the score depends on, from the rack's pose and (for the release check) the grippers' TCPs."""
    robot = _robot if robot is None else robot
    rack = rack_report(objects["rack"])
    clearance = grip_clearance(objects, robot)
    hands_clear = clearance is not None and clearance >= HANDS_CLEAR
    still = objects["rack"].resting()
    if rack["hung"]:
        partial = 1.0
    elif rack["hanging_on"] >= 1:
        partial = SCORE_ONE_KEYHOLE
    else:
        partial = 0.0
    return {
        "rack": rack,
        "grip_clearance_m": clearance,
        "hands_clear": hands_clear,
        "still": still,
        "success_now": bool(rack["hung"] and hands_clear and still),
        "partial_score": partial,
    }


def success(objects) -> bool:
    return measure(objects)["success_now"]


def score(objects) -> float:
    return float(measure(objects)["partial_score"])


def distance(objects) -> float:
    """Metres still to go: the rack's centre to where it hangs."""
    if measure(objects)["rack"]["hung"]:
        return 0.0
    return float(np.linalg.norm(np.asarray(objects["rack"].pos, dtype=float) - HANG_CENTRE))


# -- scripted demonstration (yam_scripted) ---------------------------------------------------------------
# Both arms together: each grips the upturned tip of an outer hook from above, fingers closing along x, so
# the tip is held between the pads (the pads cannot go between the rack and the wall). The rack is lifted
# to bring its keyhole entries level with the screw heads, pushed flat to the wall, lowered until the shanks
# reach the top of the throats, and let go.
GRIP_HOOKS = {"left": 2, "right": 0}  # hook index each arm takes (the outer hooks)
YAW = {"left": -np.pi / 2, "right": np.pi / 2}  # fingers closing along world x
GRIP_ABOVE_TIP = 0.001  # the TCP this far above the tip's top: the pads cover the top 12 mm of the tip
FINGER_APPROACH = -0.015  # pads 30 mm apart around the 10 mm tip; opening fully would push a pad into the rail
LIFT_CLEAR = 0.040  # the rack's lowest point this far above the table while it is carried
PUSH_EXTRA = 0.013  # the push toward the wall is commanded 13 mm past contact: the loaded arms droop back as well as
# down, and the rack pitches a few degrees in the grip, so its top edge meets the wall first and the push flattens it
PRESS = 0.004  # the lowering is commanded 4 mm past where the throats stop on the shanks
SAG_COMP_Z = 0.0155  # extra height commanded to offset what the load costs: the arms sag about 7.5 mm, the tips slip
# about 2 mm down in the pads, and the rack pitches a few degrees in the grip, which lowers its back (dev-run calibration)
TRAVEL_DZ = 0.060  # approach and leave the tips from this far above the grip height
PARK = {"left": (0.230, 0.240, TOP + 0.150), "right": (0.230, -0.240, TOP + 0.150)}
PARK_S = 2.0


def _tip_grip_point(arm: str) -> np.ndarray:
    """The TCP's position gripping the hook's tip at the start (local, rack frame)."""
    hy = G.HOOK_Y[GRIP_HOOKS[arm]]
    return np.array([G.TIP_CENTRE_X, hy, G.TIP_TOP_Z + GRIP_ABOVE_TIP])


def _hang_tcp(arm: str) -> np.ndarray:
    """The TCP's position when the rack hangs on both screws, still gripped."""
    return HANG_CENTRE + _tip_grip_point(arm)


def _entry_tcp(arm: str) -> np.ndarray:
    """The TCP's position with the rack's keyhole entries level with the screw heads, against the wall."""
    centre = HANG_CENTRE + np.array([0.0, 0.0, G.REST_Z - G.KEYHOLE_ENTRY_Z])
    return centre + _tip_grip_point(arm)


def _duration(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def arm_waypoints(arm: str) -> list[Waypoint]:
    yaw = YAW[arm]
    start = start_point(_tip_grip_point(arm))
    hang = _hang_tcp(arm)
    entry = _entry_tcp(arm)
    lift_z = max(entry[2], TOP + LIFT_CLEAR + (G.RAIL_HEIGHT / 2 + G.TIP_TOP_Z + GRIP_ABOVE_TIP))
    return [
        Waypoint(PARK_S, PARK[arm], FINGER_APPROACH, "park", yaw),
        Waypoint(2.5, (start[0], start[1], start[2] + TRAVEL_DZ), FINGER_APPROACH, "over the hook's tip", yaw),
        Waypoint(1.5, tuple(start), FINGER_APPROACH, "down around the tip", yaw),
        Waypoint(0.8, tuple(start), FINGER_CLOSED, "grip the tip", yaw),
        Waypoint(2.0, (start[0], start[1], lift_z + SAG_COMP_Z), FINGER_CLOSED, "lift the rack", yaw),
        Waypoint(2.5, (entry[0] + PUSH_EXTRA, entry[1], entry[2] + SAG_COMP_Z), FINGER_CLOSED, "push it flat to the wall over the screws", yaw),
        Waypoint(1.0, (entry[0] + PUSH_EXTRA, entry[1], entry[2] + SAG_COMP_Z), FINGER_CLOSED, "settle", yaw),
        Waypoint(2.0, (hang[0] + PUSH_EXTRA, hang[1], hang[2] - PRESS), FINGER_CLOSED, "lower it onto the shanks", yaw),
        Waypoint(1.0, (hang[0] + PUSH_EXTRA, hang[1], hang[2] - PRESS), FINGER_CLOSED, "let it settle", yaw),
        Waypoint(0.6, (hang[0] + PUSH_EXTRA, hang[1], hang[2] - PRESS), FINGER_APPROACH, "let go", yaw),
        Waypoint(1.5, (hang[0], hang[1], hang[2] + TRAVEL_DZ), FINGER_APPROACH, "lift clear", yaw),
        Waypoint(PARK_S, PARK[arm], FINGER_APPROACH, "park", yaw),
    ]  # fmt: skip


def demo() -> dict[str, list[Waypoint]]:
    """Both arms move together, one on each outer hook."""
    return {arm: arm_waypoints(arm) for arm in ("left", "right")}
