"""On the station table: a grey induction lane (a 10 mm slab, 16 cm deep, 90 cm long) runs side to side across the
front of both arms, its centre line 30 cm in front of the arm bases, from 52 cm right of centre to 38 cm left of it.
Two black rails, 15 mm tall, run along its near and far edges and leave 14 cm clear between them. Beyond the lane's
left end, in front of the left arm, stands an open blue tote (inside 24 cm front to back, 23.5 cm side to side, walls
6 cm tall). In it lie four parcels, jumbled: a brown carton (12 x 6 x 4 cm) with an orange carton (10 x 5 x 4.5 cm)
stacked on it, a red carton (11 x 7 x 4 cm) touching the brown one's far end, and a purple carton (9 x 5.5 x 4.5 cm)
turned 45 degrees in the tote's left corner. A yellow staging pad (8 x 12 cm, 5 mm thick) lies on the midline between
the arms, 15 cm in front of the bases. The lane, rails, tote and pad are fixed; the parcels are loose. Inducted, every
parcel lies flat on the lane between the rails, its long side along the lane, single file, with at least 10 cm of
clear lane between neighbours.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The induction lane: a 10 mm slab along y with a 10 mm x 15 mm rail on each long edge.
LANE_X = 0.30  # centre line, in front of the arm bases
LANE_DEPTH, LANE_T = 0.16, 0.010
LANE_Y = (-0.52, 0.38)  # its two ends (the belt axis is world y)
LANE_TOP = TOP + LANE_T
RAIL_T, RAIL_H = 0.010, 0.015
LANE_OUT_X = (LANE_X - LANE_DEPTH / 2, LANE_X + LANE_DEPTH / 2)  # the slab's outline: 0.22 .. 0.38
LANE_IN_X = (LANE_OUT_X[0] + RAIL_T, LANE_OUT_X[1] - RAIL_T)  # between the rails: 0.23 .. 0.37 (14 cm)
_LANE_CY, _LANE_LEN = (LANE_Y[0] + LANE_Y[1]) / 2, LANE_Y[1] - LANE_Y[0]
LANE_GREY, RAIL_BLACK = (0.36, 0.37, 0.40), (0.06, 0.06, 0.07)
LANE = [
    Box("induction_lane", pos=(LANE_X, _LANE_CY, TOP + LANE_T / 2), size=(LANE_DEPTH, _LANE_LEN, LANE_T),
        color=LANE_GREY),
    Box("rail_near", pos=(LANE_OUT_X[0] + RAIL_T / 2, _LANE_CY, LANE_TOP + RAIL_H / 2), size=(RAIL_T, _LANE_LEN, RAIL_H),
        color=RAIL_BLACK),
    Box("rail_far", pos=(LANE_OUT_X[1] - RAIL_T / 2, _LANE_CY, LANE_TOP + RAIL_H / 2), size=(RAIL_T, _LANE_LEN, RAIL_H),
        color=RAIL_BLACK),
]

# The source tote: a 4 mm floor and four 10 mm walls, 6 cm tall, in front of the left arm beyond the lane's end.
TOTE_IN_X, TOTE_IN_Y = (0.14, 0.38), (0.40, 0.635)
WALL_T, WALL_H, FLOOR_T = 0.010, 0.06, 0.004
FLOOR = TOP + FLOOR_T
TOTE_BLUE = (0.10, 0.30, 0.85)
_TX, _TY = (TOTE_IN_X[0] + TOTE_IN_X[1]) / 2, (TOTE_IN_Y[0] + TOTE_IN_Y[1]) / 2
_TW, _TD = TOTE_IN_X[1] - TOTE_IN_X[0], TOTE_IN_Y[1] - TOTE_IN_Y[0]
_WZ = TOP + WALL_H / 2
TOTE = [
    Box("tote_floor", pos=(_TX, _TY, TOP + FLOOR_T / 2), size=(_TW, _TD, FLOOR_T), color=TOTE_BLUE),
    Box("tote_wall_near", pos=(TOTE_IN_X[0] - WALL_T / 2, _TY, _WZ), size=(WALL_T, _TD + 2 * WALL_T, WALL_H),
        color=TOTE_BLUE),
    Box("tote_wall_far", pos=(TOTE_IN_X[1] + WALL_T / 2, _TY, _WZ), size=(WALL_T, _TD + 2 * WALL_T, WALL_H),
        color=TOTE_BLUE),
    Box("tote_wall_left", pos=(_TX, TOTE_IN_Y[1] + WALL_T / 2, _WZ), size=(_TW, WALL_T, WALL_H), color=TOTE_BLUE),
    Box("tote_wall_right", pos=(_TX, TOTE_IN_Y[0] - WALL_T / 2, _WZ), size=(_TW, WALL_T, WALL_H), color=TOTE_BLUE),
]

# The staging pad on the midline, where the left arm sets parcels down for the right arm.
PAD_X, PAD_Y, PAD_SIZE = 0.15, 0.0, (0.08, 0.12, 0.005)
PAD_TOP = TOP + PAD_SIZE[2]
PAD = Box("staging_pad", pos=(PAD_X, PAD_Y, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=(0.92, 0.74, 0.08))

# The parcels: size is (long side, short side, height); the long side lies along the box's own x axis.
BROWN_SIZE, ORANGE_SIZE, RED_SIZE, PURPLE_SIZE = (0.12, 0.06, 0.04), (0.10, 0.05, 0.045), (0.11, 0.07, 0.04), (
    0.09, 0.055, 0.045)
BROWN_START, BROWN_YAW = (0.205, 0.475), 0.0  # long side front to back
ORANGE_START, ORANGE_YAW = (0.21, 0.475), 0.0  # stacked on the brown carton
RED_START, RED_YAW = (0.30, 0.465), math.pi / 2  # long side across, touching the brown carton's far end
PURPLE_START, PURPLE_YAW = (0.21, 0.565), -math.pi / 4  # turned 45 degrees in the tote's left corner
BROWN = Box("brown_parcel", pos=(*BROWN_START, FLOOR + BROWN_SIZE[2] / 2 + 0.001), size=BROWN_SIZE, mass=0.25,
            color=(0.55, 0.30, 0.10), yaw=BROWN_YAW)
ORANGE = Box("orange_parcel", pos=(*ORANGE_START, FLOOR + BROWN_SIZE[2] + ORANGE_SIZE[2] / 2 + 0.002),
             size=ORANGE_SIZE, mass=0.20, color=(0.95, 0.50, 0.05), yaw=ORANGE_YAW)
RED = Box("red_parcel", pos=(*RED_START, FLOOR + RED_SIZE[2] / 2 + 0.001), size=RED_SIZE, mass=0.30,
          color=(0.80, 0.10, 0.10), yaw=RED_YAW)
PURPLE = Box("purple_parcel", pos=(*PURPLE_START, FLOOR + PURPLE_SIZE[2] / 2 + 0.001), size=PURPLE_SIZE, mass=0.15,
             color=(0.50, 0.15, 0.70), yaw=PURPLE_YAW)
PARCELS = [BROWN, ORANGE, RED, PURPLE]
PARCEL_NAMES = tuple(p.name for p in PARCELS)

OBJECTS = [*LANE, *TOTE, PAD, *PARCELS]
SUCCESS_HOLD_S = 2.5  # inducted and still for 2.5 s

# Scoring thresholds.
RAIL_TOL = 0.003  # a parcel's footprint may reach this far past a rail's inner face (it can lean on the rail)
BOTTOM_TOL = 0.005  # on the lane: its bottom within 5 mm of the lane's top
CARRIED_Z = 0.020  # a parcel whose bottom is more than 2 cm above the lane (or the parcel under it) is still carried
FLAT_DEG = 10.0  # flat: its height axis tilted at most 10 degrees from vertical, either way up
ALONG_DEG = 15.0  # its long side within 15 degrees of the lane's length
GAP_FULL, GAP_PART = 0.100, 0.050  # clear lane between neighbours: full credit from 10 cm, part credit from 5 cm
DOUBLE_FEED_GAP = 0.010  # two parcels on the lane closer than 1 cm along it (or overlapping) are a double feed
PAIR_FULL, PAIR_PART, BATCH_CREDIT = 0.25, 0.10, 0.25


def _tilt_deg(state) -> float:
    """Angle between the box's height axis and the vertical, either way up: a carton lying on either of its two
    largest faces is flat (the cartons are unmarked, so which of the two is down does not matter)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _off_lane_axis_deg(state) -> float:
    """Angle between the box's long side (its own x axis), seen from above, and the lane's length (world y)."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    ux, uy = 1 - 2 * (y * y + z * z), 2 * (x * y + z * w)
    return math.degrees(math.atan2(abs(ux), abs(uy)))


def _over_lane(state) -> bool:
    """Any part of the parcel over the lane's slab, and its bottom no lower than 5 mm below the lane's top (on the
    lane, a rail or another parcel, not on the table beside it)."""
    lo, hi = state.low, state.high
    return bool(
        float(lo[0]) < LANE_OUT_X[1] and float(hi[0]) > LANE_OUT_X[0]
        and float(lo[1]) < LANE_Y[1] and float(hi[1]) > LANE_Y[0]
        and float(lo[2]) >= LANE_TOP - BOTTOM_TOL
    )


def _footprints_overlap(a, b) -> bool:
    return bool(float(a.low[0]) < float(b.high[0]) and float(b.low[0]) < float(a.high[0])
                and float(a.low[1]) < float(b.high[1]) and float(b.low[1]) < float(a.high[1]))


def _on_lane(objects) -> list:
    """The parcels on the lane: over the slab with the bottom at most 2 cm above the lane's top, or over the slab
    with the bottom at most 2 cm above the top of a parcel on the lane whose footprint it overlaps (a stack). A
    parcel still carried higher is not on the lane."""
    over = [objects[n] for n in PARCEL_NAMES if _over_lane(objects[n])]
    on = [s for s in over if float(s.low[2]) <= LANE_TOP + CARRIED_Z]
    grew = True
    while grew:
        grew = False
        for s in over:
            if any(s is t for t in on):
                continue
            if any(_footprints_overlap(s, t) and float(s.low[2]) <= float(t.high[2]) + CARRIED_Z for t in on):
                on.append(s)
                grew = True
    return on


def _inducted(state) -> bool:
    """Flat on the lane between the rails, long side along it, wholly within its ends, and still."""
    lo, hi = state.low, state.high
    return bool(
        float(lo[0]) >= LANE_IN_X[0] - RAIL_TOL and float(hi[0]) <= LANE_IN_X[1] + RAIL_TOL
        and float(lo[1]) >= LANE_Y[0] and float(hi[1]) <= LANE_Y[1]
        and abs(float(lo[2]) - LANE_TOP) <= BOTTOM_TOL
        and _tilt_deg(state) <= FLAT_DEG
        and _off_lane_axis_deg(state) <= ALONG_DEG
        and state.resting()
    )


def _gap(a, b) -> float:
    """Clear lane between two parcels along its length (world y); negative when their extents overlap."""
    return max(float(a.low[1]), float(b.low[1])) - min(float(a.high[1]), float(b.high[1]))


def score(objects):
    """Partial credit (success scores 1.0): the parcels on the lane are taken in order along it; each neighbouring
    pair, both inducted, earns 0.25 with at least 10 cm of clear lane between them, or 0.1 with 5 to 10 cm. A
    parcel on the lane that is not inducted breaks the pairs it is in. 0.25 more when all four are inducted and all
    three gaps are at least 10 cm. 0 overall if any two parcels on the lane are less than 1 cm apart along it or
    overlap along it (side by side, stacked or touching end to end: a double feed)."""
    on = _on_lane(objects)
    for i, a in enumerate(on):
        for b in on[i + 1:]:
            if _gap(a, b) < DOUBLE_FEED_GAP:
                return 0.0
    line = sorted(on, key=lambda s: float(s.pos[1]))
    total, full = 0.0, 0
    for a, b in zip(line, line[1:]):
        if not (_inducted(a) and _inducted(b)):
            continue
        gap = _gap(a, b)
        if gap >= GAP_FULL:
            total += PAIR_FULL
            full += 1
        elif gap >= GAP_PART:
            total += PAIR_PART
    if full == len(PARCEL_NAMES) - 1 and all(_inducted(objects[n]) for n in PARCEL_NAMES):
        total += BATCH_CREDIT
    return round(total, 6)


def success(objects):
    """All four parcels inducted single file with 10 cm gaps, and still; must hold for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in PARCEL_NAMES)


# The demo: the left arm unstacks the orange carton and lays it on the lane just left of centre, sets the purple
# carton on the staging pad (squared to the lane), lays the brown carton at the lane's left end, then sets the red
# carton on the pad. The right arm takes each from the pad and lays it on the right half of the lane. Every parcel
# is gripped across its short side; the fingers close along world y at yaw 0, so a parcel lies along the lane when
# its grip is turned 90 degrees.
TRAVEL_Z = TOP + 0.15  # a carried parcel's bottom clears the tote walls and the rails by 6 cm or more
GRIP = 0.025  # TCP above a parcel's bottom: the pads cover its lower sides from 12 mm up
ORANGE_GRIP = 0.030  # higher on the stacked carton, so the fingertips stay above the purple carton beside it
DROP = 0.008  # each parcel is let go this far above where it lands
ALONG = math.pi / 2  # grip yaw that lays a parcel's long side along the lane
SLOT_Y = {"brown_parcel": 0.30, "orange_parcel": 0.07, "purple_parcel": -0.16, "red_parcel": -0.39}
LEFT_PARK = (0.22, 0.50, TRAVEL_Z)  # above the empty tote, clear of the lane and the pad
RIGHT_WAIT = (0.22, -0.25, TRAVEL_Z)  # where the right arm waits for the first parcel on the pad
RIGHT_FIRST_S = 19.5  # the left arm has left the pad (purple carton set down) by 19.2 s
RIGHT_SECOND_S = 39.5  # the left arm has left the pad (red carton set down) by 38.4 s
RIGHT_PARK = (0.16, -0.30, TRAVEL_Z)  # in front of the lane's right half, so the camera sees the line


def _lane(name, grip=GRIP):
    return (LANE_X, SLOT_Y[name], LANE_TOP + DROP + grip)


def demo():
    # The left arm turns its grip to -90 degrees and the right arm to +90 (each wrist turns toward its own side);
    # the red carton goes onto the pad long side front to back (grip yaw 0), so no grip turns half a revolution.
    pad = (PAD_X, PAD_Y, PAD_TOP + DROP + GRIP)
    left = pick_place_waypoints((*ORANGE_START, FLOOR + BROWN_SIZE[2] + ORANGE_GRIP),
                                _lane("orange_parcel", ORANGE_GRIP), travel_z=TRAVEL_Z, yaw=ORANGE_YAW,
                                place_yaw=-ALONG)
    left += pick_place_waypoints((*PURPLE_START, FLOOR + GRIP), pad, travel_z=TRAVEL_Z, yaw=PURPLE_YAW,
                                 place_yaw=-ALONG)
    left += pick_place_waypoints((*BROWN_START, FLOOR + GRIP), _lane("brown_parcel"), travel_z=TRAVEL_Z,
                                 yaw=BROWN_YAW, place_yaw=-ALONG)
    left += pick_place_waypoints((*RED_START, FLOOR + GRIP), pad, travel_z=TRAVEL_Z, yaw=RED_YAW, place_yaw=0.0)
    left.append(Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park", 0.0))

    pad_pick = (PAD_X, PAD_Y, PAD_TOP + GRIP)
    right = [Waypoint(RIGHT_FIRST_S, RIGHT_WAIT, FINGER_OPEN, "wait for the purple carton", ALONG)]
    right += pick_place_waypoints(pad_pick, _lane("purple_parcel"), travel_z=TRAVEL_Z, yaw=ALONG)
    wait = RIGHT_SECOND_S - RIGHT_FIRST_S - sum(w.duration for w in right[1:])
    right.append(Waypoint(wait, (LANE_X, SLOT_Y["purple_parcel"], TRAVEL_Z), FINGER_OPEN, "wait for the red carton",
                          ALONG))
    right += pick_place_waypoints(pad_pick, _lane("red_parcel"), travel_z=TRAVEL_Z, yaw=0.0, place_yaw=ALONG)
    right.append(Waypoint(2.5, RIGHT_PARK, FINGER_OPEN, "park", ALONG))
    return {"left": left, "right": right}
