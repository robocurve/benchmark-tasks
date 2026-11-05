"""On the station table, in front of the arms and to the left of the midline: a pile of three aluminium plates of the
same size in a destacking cradle; the top plate is blue anodised, the two under it bare aluminium. The plates lie
flat, stacked squarely, long side running away from the robot. Two fixed steel guide rails,
as tall as the pile, run along its left and right sides with 3 mm of clearance; both ends of the cradle are open.
A green pad lies on the table to the robot's left of the pile.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
PLATE = (0.160, 0.080, 0.012)  # long side along x
PLATE_MASS = 0.415  # aluminium, 2.7 g/cm3
PILE_X, PILE_Y = 0.24, 0.14
NAMES = ("plate_bottom", "plate_middle", "plate_top")
PLATE_Z = {n: TOP + PLATE[2] * (i + 0.5) for i, n in enumerate(NAMES)}
PILE_TOP = TOP + 3 * PLATE[2]
UNDER_TOP = TOP + 2 * PLATE[2]  # top face of the middle plate: where the top plate rests
PILE_X0, PILE_X1 = PILE_X - PLATE[0] / 2, PILE_X + PLATE[0] / 2
PILE_Y0, PILE_Y1 = PILE_Y - PLATE[1] / 2, PILE_Y + PLATE[1] / 2

CLEAR = 0.003  # between a plate's side and a rail
RAIL = (PLATE[0], 0.008, 3 * PLATE[2])  # as long and as tall as the pile
RAIL_Y = {"rail_left": PILE_Y1 + CLEAR + RAIL[1] / 2, "rail_right": PILE_Y0 - CLEAR - RAIL[1] / 2}

PAD_XY = (0.24, 0.33)
PAD_SIZE = (0.200, 0.120, 0.002)
PAD_TOP = TOP + PAD_SIZE[2]

ALU, BLUE, STEEL, GREEN = (0.76, 0.77, 0.79), (0.18, 0.36, 0.72), (0.30, 0.32, 0.35), (0.20, 0.65, 0.30)

PLATES = [Box(n, pos=(PILE_X, PILE_Y, PLATE_Z[n]), size=PLATE, mass=PLATE_MASS,
              color=BLUE if n == "plate_top" else ALU) for n in NAMES]
RAILS = [Box(n, pos=(PILE_X, y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL) for n, y in RAIL_Y.items()]
PAD = Box("green_pad", pos=(*PAD_XY, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=GREEN)
OBJECTS = [*PLATES, *RAILS, PAD]
SPEC = {o.name: o for o in PLATES}
LOWER = ("plate_bottom", "plate_middle")

MOVE_TOL = 0.003  # a lower plate's centre may move at most 3 mm (any direction) at any time in the trial
FLAT_DEG = 3.0  # lying flat: the plate's faces within 3 degrees of horizontal
ON_TOL = 0.001  # resting on a surface: the plate's lowest point within 1 mm of it
FOOT_SLACK = 0.001
OVERHANG = 0.040  # the 0.25 tier: an end of the top plate at least 40 mm past the end of the pile
EPS = 1e-6  # every limit is inclusive: a value exactly at a stated limit passes


def _moved(st, name) -> float:
    return float(np.linalg.norm(np.asarray(st.pos) - np.asarray(SPEC[name].pos)))


def tilt_deg(st) -> float:
    """Angle between the plate's thickness axis and the vertical (either face up counts as flat)."""
    x, y, _, _ = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return float(np.degrees(np.arccos(min(1.0, abs(1.0 - 2.0 * (x * x + y * y))))))


def _flat(st) -> bool:
    return tilt_deg(st) <= FLAT_DEG + EPS


# Trial history: whether either lower plate has moved more than 3 mm or tilted since the trial started. The scorer
# sees one step at a time, so it is kept here. It is cleared only by a scene reset: every plate back within 0.5 mm
# of its starting pose, flat and at rest, reached by a jump of more than 20 mm in a single step for some plate (the
# simulator's reset). Rebuilding the pile by hand does not clear it.
RESET_TOL, RESET_JUMP = 0.0005, 0.020
_HISTORY = {"violated": False, "prev": None}


def _update(objects) -> None:
    prev = _HISTORY["prev"]
    _HISTORY["prev"] = {n: np.asarray(objects[n].pos, float).copy() for n in NAMES}
    at_start = all(_moved(objects[n], n) <= RESET_TOL and _flat(objects[n]) and objects[n].resting() for n in NAMES)
    jumped = prev is None or any(np.linalg.norm(prev[n] - _HISTORY["prev"][n]) > RESET_JUMP for n in NAMES)
    if at_start and jumped:
        _HISTORY["violated"] = False
    for n in LOWER:
        st = objects[n]
        if _moved(st, n) > MOVE_TOL + EPS or not _flat(st):
            _HISTORY["violated"] = True


def _lower_ok(objects) -> bool:
    return not _HISTORY["violated"] and all(objects[n].resting() for n in LOWER)


def _on(st, surface_z) -> bool:
    return bool(abs(st.low[2] - surface_z) <= ON_TOL + EPS)


def _on_pad(st) -> bool:
    """Wholly on the pad: for a flat plate the world-aligned box is the box of its four corners, however it is
    turned, so this is the plate's true footprint."""
    px, py = PAD_XY
    hx, hy = PAD_SIZE[0] / 2 + FOOT_SLACK + EPS, PAD_SIZE[1] / 2 + FOOT_SLACK + EPS
    return bool(st.low[0] >= px - hx and st.high[0] <= px + hx and st.low[1] >= py - hy and st.high[1] <= py + hy
                and _on(st, PAD_TOP))


LANE_Y0 = RAIL_Y["rail_right"] - RAIL[1] / 2  # the cradle's lane: from the right rail's outer face ...
LANE_Y1 = RAIL_Y["rail_left"] + RAIL[1] / 2  # ... to the left rail's, along the whole table


def _clear_of_lane(st) -> bool:
    """No part of the plate's footprint in the cradle's lane, over the cradle or beyond either end of it (a plate
    pushed straight off the pile without a grasp lands in the lane)."""
    return bool(st.low[1] > LANE_Y1 or st.high[1] < LANE_Y0)


def _tier(objects) -> float:
    _update(objects)
    if not _lower_ok(objects):
        return 0.0
    top = objects["plate_top"]
    if not (top.resting() and _flat(top)):
        return 0.0
    if _on_pad(top):
        return 1.0
    if _clear_of_lane(top) and (_on(top, TOP) or _on(top, PAD_TOP)):
        return 0.5
    overhang = max(top.high[0] - PILE_X1, PILE_X0 - top.low[0])
    if _on(top, UNDER_TOP) and overhang >= OVERHANG - EPS:
        return 0.25
    return 0.0


def success(objects):
    """The top plate lying flat, wholly on the pad and at rest; the two lower plates at rest, and neither moved
    more than 3 mm nor tilted more than 3 degrees at any time in the trial."""
    return _tier(objects) == 1.0


def distance(objects):
    return float(np.linalg.norm(objects["plate_top"].pos[:2] - np.asarray(PAD_XY)))


def score(objects):
    """1.0 as success; 0.5 with the top plate lying flat on the table outside the cradle's lane; 0.25 with it still
    lying flat on the pile but slid at least 40 mm past an end of it; 0 otherwise, and 0 for the whole trial once
    either lower plate has moved more than 3 mm or tilted more than 3 degrees."""
    return _tier(objects)


# ---- scripted demo ------------------------------------------------------------------------------------------
# Left arm: closed finger tips against the top plate's near end at its mid-height push it 60 mm along the cradle,
# so its far end overhangs the pile by 60 mm (its centre stays 20 mm inside the pile's end). Then the open fingers
# come down past the end of the rails, close across the overhang's 80 mm width, lift the plate and lay it on the pad.
TIP = 0.013  # finger tips below the TCP
FINGER_HALF_X = 0.020  # the finger tips' half-depth along x
TRAVEL_Z = TOP + 0.14
PUSH = 0.060
PUSH_Z = UNDER_TOP + PLATE[2] / 2 + TIP  # tips at the top plate's mid-height, 6 mm above the middle plate
GRIP_X = PILE_X1 + 0.027  # finger tips 7 mm clear of the rail ends
GRIP_TIPS = UNDER_TOP + 0.001
GRIP_Z = GRIP_TIPS + TIP
TIPS_TO_BOTTOM = GRIP_TIPS - UNDER_TOP  # the plate's underside 1 mm below the tips
PLACE_DROP = 0.007  # released with its underside 7 mm above the pad, so it drops free of the fingers
# Measured on the L40S with the trace wrapper: pushing the plate, the arm's TCP stops 50-65 mm short of its target
# along the push (the plate's friction against a soft position-controlled arm). The demo aims that much further.
PUSH_LAG = 0.065
CENTRE_BEHIND_GRIP = GRIP_X - (PILE_X + PUSH)  # the plate's centre this far toward the robot from the grip


def _wp(t, x, y, z, f, label, yaw=0.0):
    return Waypoint(t, (float(x), float(y), float(z)), f, label, yaw)


def demo():
    y = PILE_Y
    x_touch = PILE_X0 - FINGER_HALF_X
    place_x = PAD_XY[0] + CENTRE_BEHIND_GRIP
    place_z = PAD_TOP + PLACE_DROP - TIPS_TO_BOTTOM + TIP
    left = [
        _wp(1.5, x_touch - 0.015, y, TRAVEL_Z, FINGER_CLOSED, "in front of the pile"),
        _wp(1.5, x_touch - 0.015, y, PUSH_Z, FINGER_CLOSED, "down to the top plate's mid-height"),
        _wp(4.0, x_touch + PUSH + PUSH_LAG, y, PUSH_Z, FINGER_CLOSED, "push the top plate along the cradle"),
        _wp(1.5, x_touch + PUSH + PUSH_LAG, y, PUSH_Z, FINGER_CLOSED, "hold"),
        _wp(0.6, x_touch + PUSH - 0.020, y, PUSH_Z, FINGER_CLOSED, "back off"),
        _wp(1.0, x_touch + PUSH - 0.020, y, TRAVEL_Z, FINGER_OPEN, "up and open"),
        _wp(1.5, GRIP_X, y, TRAVEL_Z, FINGER_OPEN, "over the overhang"),
        _wp(1.5, GRIP_X, y, GRIP_Z, FINGER_OPEN, "down beside it, past the rail ends"),
        _wp(0.8, GRIP_X, y, GRIP_Z, FINGER_CLOSED, "close across the width"),
        _wp(1.2, GRIP_X, y, TRAVEL_Z, FINGER_CLOSED, "lift"),
        _wp(1.8, place_x, PAD_XY[1], TRAVEL_Z, FINGER_CLOSED, "over the pad"),
        _wp(1.5, place_x, PAD_XY[1], place_z, FINGER_CLOSED, "lower"),
        _wp(0.6, place_x, PAD_XY[1], place_z, FINGER_OPEN, "release"),
        _wp(1.0, place_x, PAD_XY[1], TRAVEL_Z, FINGER_OPEN, "up"),
    ]
    return {"left": left}
