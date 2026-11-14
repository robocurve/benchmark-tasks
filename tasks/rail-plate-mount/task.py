"""On the station table, in front of the arms: a galvanised steel mounting plate (200 x 300 x 2 mm, on its 15 mm
folded edges, fixed in place, its length pointing away from the arms) with two tapped M5 holes 200 mm apart on its
centreline, along the plate. On it, loose:
a slotted steel TS35 x 7.5 DIN rail, 250 mm long, lying web-down along the plate, its slots (6.2 x 15 mm, every 25 mm)
not over the holes and turned 3 degrees. Left of the plate: a steel screw holder (90 x 24 x 30 mm) with two M5 x 6
hexagon socket head cap screws (zinc-plated steel, head 8.5 mm across, 5 mm high, 4 mm socket) standing in it head up
and a stubby hex screwdriver (4 mm hexagon blade, red 26 mm handle, 85 mm long) standing in it blade down.

Each screw catches its thread when it stands upright in a hole and is turned clockwise; from there it takes about 4
turns until its head sits on the rail and a quarter turn more until it is tight.

Task: fix the rail to the mounting plate with the two screws: line the rail up so a slot is over each hole, hold it,
put a screw through a slot into each hole and tighten both with the hex screwdriver.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rp_parts as rp  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

rp.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MM = 1e-3
PLATE_ORIGIN = (0.25, 0.0, TOP + rp.PLATE_FLANGE)  # the plate's top face centre, midway between the arms ...
PLATE_YAW = math.radians(-90.0)  # ... its length along x (away from the arms): holes at x 0.35 and 0.15 (the left hand
# didn't reach above x 0.38 at carrying height, run 29).
# (At y 0.09 the right hand, holding the rail 43 cm sideways from its base, let it jump ~1 mm when setting it down,
# demo runs 27-28.)
# (Lying along y, the near-side hole was beyond where the left hand reaches straight down with its fingers across the
# rail: it stood 13 degrees turned and its fingers caught on the rail's sides, demo runs 23-26; reach scan.)


def plate_point(along: float, across: float = 0.0, up: float = 0.0):
    """A point in the plate's frame (along: its centreline, local y; across: local x) in env coordinates."""
    c, s = math.cos(PLATE_YAW), math.sin(PLATE_YAW)
    return (PLATE_ORIGIN[0] + c * across - s * along, PLATE_ORIGIN[1] + s * across + c * along, PLATE_ORIGIN[2] + up)


HOLES = tuple(plate_point(y) for y in rp.HOLE_Y)
RAIL_YAW = PLATE_YAW + math.radians(3.0)
RAIL_ORIGIN = plate_point(0.0, up=0.1 * MM)  # both holes under the web between slots
HOLDER_ORIGIN = (0.25, 0.29, TOP + rp.HOLDER_SIZE[2] + 0.1 * MM)  # its top face centre, beside the plate's +y end
SCREWS = ("screw_1", "screw_2")
PARK_Y = (-0.3, -0.2)  # (the threaded twins wait out of sight)


def _screw_in_holder(i: int):
    return (HOLDER_ORIGIN[0] + rp.HOLDER_HOLES[i], HOLDER_ORIGIN[1],
            HOLDER_ORIGIN[2] + rp.HOLDER_BOSS_H - rp.SCREW_LEN + 0.2 * MM)


DRIVER_ORIGIN = (HOLDER_ORIGIN[0] + rp.HOLDER_DRIVER, HOLDER_ORIGIN[1], HOLDER_ORIGIN[2] - rp.SHAFT_TOP + 0.3 * MM)

OBJECTS = [
    rp.part("plate", "plate", PLATE_ORIGIN, yaw=PLATE_YAW, fixed=True),
    rp.part("rail", "rail", RAIL_ORIGIN, yaw=RAIL_YAW),
    rp.part("holder", "holder", HOLDER_ORIGIN),
    *rp.screw_parts(SCREWS[0], _screw_in_holder(0), park_y=PARK_Y[0]),
    *rp.screw_parts(SCREWS[1], _screw_in_holder(1), park_y=PARK_Y[1]),
    rp.part("driver", "driver", DRIVER_ORIGIN),
]

# -- scoring (each screw on its own; checkable by hand and eye: is the screw in its hole through a slot of the rail,
# does it turn further at the tightening torque) -------------------------------------------------------------------
STARTED_TURNS = 1.0  # a screw counts as started once it has turned this far into its thread
RAIL_FLAT = 1.0 * MM  # the rail's web lies on the plate: its underside within this of the plate's top ...
RAIL_FLAT_DEG = 2.0  # ... and tilted less than this
CREDIT_STARTED, CREDIT_TIGHT = 0.25, 0.5

C_RAIL = np.array(rp.local_center("rail"))


def rail_pose(objects):
    """(origin, rotation) of the rail (its origin: the centre of its web's underside)."""
    st = objects["rail"]
    rot = quat_xyzw_to_matrix(np.asarray(st.quat_xyzw, dtype=float))
    return np.asarray(st.pos, dtype=float) - rot @ C_RAIL, rot


def rail_flat(objects) -> bool:
    p, rot = rail_pose(objects)
    return bool(abs(p[2] - PLATE_ORIGIN[2]) <= RAIL_FLAT and math.degrees(math.acos(min(1.0, rot[2, 2]))) <= RAIL_FLAT_DEG)


def screw_tip(state):
    rot = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, dtype=float))
    return np.asarray(state.pos, dtype=float) - rot @ np.array(rp.local_center("screw"))


def through_slot(objects, name: str) -> bool:
    """The screw passes through one of the rail's slots (its axis inside the slot, in the rail's frame)."""
    p, rot = rail_pose(objects)
    local = rot.T @ (screw_tip(objects[name]) - p)
    if abs(local[0]) > rp.SLOT_HALF_W:
        return False
    return any(abs(local[1] - y) <= rp.SLOT_LEN / 2 for y in rp.SLOTS)


def screw_credit(objects, name: str) -> float:
    st = objects[name]
    if not getattr(st, "engaged", False) or not rail_flat(objects) or not through_slot(objects, name):
        return 0.0
    if st.tight():
        return CREDIT_TIGHT
    return CREDIT_STARTED if st.turns >= STARTED_TURNS else 0.0


def score(objects) -> float:
    """0.25 for each screw started (at least one turn into its thread) through a slot of the rail, 0.5 instead for
    each one tight; the sum."""
    return float(sum(screw_credit(objects, n) for n in SCREWS))


def success(objects) -> bool:
    """Both screws tight through the rail's slots, and still (held 1 s)."""
    return score(objects) >= 1.0 - 1e-9 and all(objects[n].resting() for n in SCREWS)


def distance(objects) -> float:
    """Metres from each screw's tip to its nearest hole, summed (a progress measure, not the score)."""
    total = 0.0
    for n in SCREWS:
        tip = screw_tip(objects[n])
        total += min(float(np.linalg.norm(tip - (np.array(h) - (0, 0, rp.DOG_LEN)))) for h in HOLES)
    return total
