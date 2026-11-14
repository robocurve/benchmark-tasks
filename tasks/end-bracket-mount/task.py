"""On the station table, in front of the arms: a loose steel TS35 x 7.5 top-hat DIN rail, 24 cm long, lying flat
(open side up) and running left to right. A row of four beige terminal blocks is mounted on it, side by side,
the rightmost in the middle of the rail (a terminal strip, not yet closed off by an end bracket). The rail is
loose and light (130 g with its blocks): pushed, pressed or twisted without being held, it slides and turns.

In front of the right arm, on the table: a dark beige plastic end bracket for the rail (8 mm thick, 56 mm wide,
46 mm tall, 14 g) standing on its two feet, its faces to the left and right, and a flat-blade screwdriver
(a stubby: 0.8 x 4.0 mm blade, 25 mm shaft, red 26 mm handle, 85 mm long) lying with its tip toward the arms.

The end bracket is screw-fixed, not snap-on. Underneath it has a fixed hook on one side and a clamping jaw on the
other: hook the hook side under one lip of the rail, tilt the bracket down over the other lip (or slide it on from
the free right end of the rail), and it sits on the rail but still slides. Its clamping screw (M4, slotted) is on
top in the middle; turning it clockwise moves the jaw under the other lip and clamps the bracket to the rail. From
backed out it takes about 1.4 turns before the jaw touches and about 1.6-1.8 turns until it is tight.

Task: close off the terminal strip: fit the end bracket against the right end of the row of beige terminal blocks and
tighten it. Hold the rail down with one arm; with the other, put the bracket on the rail to the right of the row, push
it flush against
the block, and tighten its screw with the screwdriver to 0.5 Nm.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eb_parts as ep  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

ep.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MM = 1e-3
RAIL_ORIGIN = (0.30, 0.0, TOP + 0.2 * MM)  # on the midline: the bracket's place is reachable by both arms
# The bracket stands on its two skirts (its feet are 7 mm below its seat), its faces toward the sides (+-y), its hook
# side toward the arms.
BRACKET_ORIGIN = (0.20, -0.17, TOP + 7.0 * MM + 0.3 * MM)
BRACKET_ROLL = 0.0
# The screwdriver lies along x, its tip (its origin) toward the arms, its handle away, resting on the table.
DRIVER_ORIGIN = (0.27, -0.27, TOP + ep.HANDLE_R + 0.3 * MM)
DRIVER_PITCH = math.pi / 2

OBJECTS = [
    ep.part("rail", "rail", RAIL_ORIGIN),
    ep.part("bracket", "bracket", BRACKET_ORIGIN, roll=BRACKET_ROLL),
    ep.part("driver", "driver", DRIVER_ORIGIN, pitch=DRIVER_PITCH),
]

# -- scoring thresholds (all checkable with a ruler, a feeler gauge, a torque screwdriver and a photo) ---------
RAIL_MAX_SHIFT = 10 * MM  # the rail's origin may move less than this from where it started ...
RAIL_MAX_TURN_DEG = 3.0  # ... and turn less than this
LIFT_HEIGHT = 25 * MM  # the bracket counts as lifted when its lowest point is this far above the table
SEAT_RANGE = (-0.5 * MM, 1.0 * MM)  # the bracket's seat above the lip tops at both lip edges: seated (its hook is
# then under the lip: set straight down on the rail, the hook lands on the lip and holds that side 2.8 mm up; the
# hook's 0.8 mm play lets the hook side lift a little when the jaw clamps the other side)
MAX_ACROSS = 1.0 * MM  # the bracket's centre from the rail's centreline
MAX_SQUARE_DEG = 3.0  # square to the rail (yaw) and level (tilt)
FLUSH_GAP = 0.5 * MM  # gap to the beige block at each of the four check points of the bracket's face
# the four check points on the bracket's face toward the beige block: (across, height above the lips), near both
# ends and low and high (37.5 mm apart vertically: 0.5 mm there is 0.76 degrees)
FLUSH_POINTS = ((15 * MM, 0.5 * MM), (15 * MM, 38 * MM), (-15 * MM, 0.5 * MM), (-15 * MM, 38 * MM))
TIGHT_TURNS = ep.TIGHT_TURNS  # the screw turned at least this far from backed out (0.25 turn past the jaw's contact)
TIGHT_CLAMP = ep.TIGHT_CLAMP  # N: ... with the jaw pressing on the lip. The simulation's stand-in for a screw tightened
# to the rated 0.5 Nm; a real trial checks with a torque screwdriver set to 0.5 Nm
CREDIT = {"lifted": 0.25, "seated": 0.5, "flush": 0.75}
RAIL_MOVED_CAP = CREDIT["lifted"]

C_RAIL = np.array(ep.local_center("rail"))
C_BRACKET = np.array(ep.local_center("bracket"))


def _origin(state, center) -> tuple[np.ndarray, np.ndarray]:
    """(origin, rotation) of a part from its ObjectState (whose ``pos`` is its bounding-box centre)."""
    rot = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, dtype=float))
    return np.asarray(state.pos, dtype=float) - rot @ center, rot


def rail_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    return _origin(objects["rail"], C_RAIL)


def rail_motion(objects) -> tuple[float, float]:
    """(metres the rail's origin moved across the table, degrees it turned) since the start."""
    p, rot = rail_pose(objects)
    shift = float(np.hypot(p[0] - RAIL_ORIGIN[0], p[1] - RAIL_ORIGIN[1]))
    turn = math.degrees(math.acos(min(1.0, max(-1.0, (np.trace(rot) - 1.0) / 2.0))))
    return shift, turn


def rail_ok(objects) -> bool:
    shift, turn = rail_motion(objects)
    return shift < RAIL_MAX_SHIFT and turn < RAIL_MAX_TURN_DEG


def bracket_on_rail(objects) -> dict:
    """The bracket's pose in the rail's frame: across/along offsets, its seat's height above the lips at each lip
    edge, its yaw and tilt relative to the rail, its gap to the beige block at the four check points of its facing
    side, and its screw's torque."""
    p_r, r_r = rail_pose(objects)
    st = objects["bracket"]
    p_b, r_b = _origin(st, C_BRACKET)
    rel, pos = r_r.T @ r_b, r_r.T @ (p_b - p_r)

    def point(local):
        return pos + rel @ np.asarray(local, dtype=float)

    seat = [point((s * ep.LIP_EDGE, 0.0, 0.0))[2] - ep.RAIL_HEIGHT for s in (1, -1)]
    # the bracket's face toward the beige block (+y in the rail frame) is its +y face when it sits the right way
    # round; either face counts (the bracket is symmetric through its thickness)
    face = 1.0 if rel[1, 1] >= 0 else -1.0
    corners = [point((x, face * ep.BRACKET_WIDTH / 2, z)) for x, z in FLUSH_POINTS]
    return {
        "across": float(pos[0]),
        "along": float(pos[1]),
        "seat": [float(s) for s in seat],
        # square to the rail, either way round (turned 180 degrees the hook is on the other lip: just as valid)
        "yaw_deg": (math.degrees(math.atan2(rel[1, 0], rel[0, 0])) + 90.0) % 180.0 - 90.0,
        "tilt_deg": math.degrees(math.acos(min(1.0, max(-1.0, rel[2, 2])))),
        "gaps": [float(ep.BEIGE_FACE_Y - c[1]) for c in corners],
        "screw_turns": float(getattr(st, "screw_turns", math.nan)),
        "clamp_force": float(getattr(st, "clamp_force", math.nan)),
    }


def seated(b) -> bool:
    """On the rail to the right of the beige block, hooked under the lip: its seat flat on the lips at both lip
    edges, centred and square."""
    right_of_block = -ep.RAIL_LENGTH / 2 + ep.BRACKET_WIDTH / 2 <= b["along"] <= ep.BEIGE_FACE_Y - ep.BRACKET_WIDTH / 2 + 1 * MM
    return bool(
        right_of_block
        and abs(b["across"]) <= MAX_ACROSS
        and all(SEAT_RANGE[0] <= s <= SEAT_RANGE[1] for s in b["seat"])
        and abs(b["yaw_deg"]) <= MAX_SQUARE_DEG
        and b["tilt_deg"] <= MAX_SQUARE_DEG
    )


def flush(b) -> bool:
    return seated(b) and all(-FLUSH_GAP <= g <= FLUSH_GAP for g in b["gaps"])


def tight(b) -> bool:
    """Its screw tightened: in the simulation, turned TIGHT_TURNS or more with the jaw pressing on the far lip; on a
    real bracket, a torque screwdriver set to 0.5 Nm clicks."""
    return bool(b["screw_turns"] >= TIGHT_TURNS and b["clamp_force"] >= TIGHT_CLAMP)


def stage(objects) -> str | None:
    b = bracket_on_rail(objects)
    if flush(b):
        return "flush_tight" if tight(b) else "flush"
    if seated(b):
        return "seated"
    if objects["bracket"].low[2] >= TOP + LIFT_HEIGHT:
        return "lifted"
    return None


def success(objects) -> bool:
    """The bracket flush against the beige block, its screw tight, and still (held 1 s), with the rail where it
    started."""
    return rail_ok(objects) and stage(objects) == "flush_tight" and objects["bracket"].resting()


def score(objects) -> float:
    """Partial credit at the end of the episode (success scores 1.0): 0.75 flush (tightened or not), 0.5 seated on
    the rail, 0.25 lifted. Capped at 0.25 if the rail moved 1 cm or turned 3 degrees."""
    st = stage(objects)
    best = CREDIT.get("flush" if st == "flush_tight" else st, 0.0)
    return best if rail_ok(objects) else min(best, RAIL_MOVED_CAP)


def distance(objects) -> float:
    """Metres from the bracket's origin to its mounted place, in the rail's frame."""
    b = bracket_on_rail(objects)
    slot = ep.BEIGE_FACE_Y - ep.BRACKET_WIDTH / 2
    return float(math.sqrt(b["across"] ** 2 + (b["along"] - slot) ** 2 + max(abs(s) for s in b["seat"]) ** 2))
