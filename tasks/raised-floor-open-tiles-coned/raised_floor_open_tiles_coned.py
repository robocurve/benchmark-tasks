"""On the station table, between the arms: a fixed dark grey raised-floor frame, 29 x 29 cm and 3 cm tall, its centre
30.5 cm in front of the arm bases on the midline. It has four square pockets 12 cm across and 3 cm deep (the pocket floor
is the table top), separated by 2 cm webs inside a 1.5 cm rim. Each pocket holds a floor tile 7 x 7 x 3 cm (200 g),
flush with the frame's top: orange in the pocket nearer the arms on the left and in the far pocket on the right, green
in the other two. The orange tiles are marked for lifting. In front of the left arm lies a fixed blue staging pad
(18 cm front to back, 11 cm side to side, 1 cm tall), its inner edge 10.5 cm beyond the frame's left edge. In front of
the right arm stand two yellow cones (upright cylinders 4 cm across, 8 cm tall, 115 g). Done, both orange tiles lie
flat on the pad, a yellow cone stands upright in each of the two holes they leave, and the green tiles stay seated.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The raised-floor frame: four 12 cm pockets, 2 cm webs between them, a 1.5 cm rim, 3 cm tall.
FRAME_X, FRAME_Y = 0.305, 0.0
POCKET, WEB, RIM, FRAME_H = 0.12, 0.02, 0.015, 0.03
FRAME = 2 * POCKET + WEB + 2 * RIM  # 0.29 m outside
FRAME_TOP = TOP + FRAME_H
_HALF = FRAME / 2
_OFF = WEB / 2 + POCKET / 2  # pocket centres sit 7 cm from the frame's centre lines
P_NEAR_LEFT = (FRAME_X - _OFF, FRAME_Y + _OFF)
P_FAR_LEFT = (FRAME_X + _OFF, FRAME_Y + _OFF)
P_NEAR_RIGHT = (FRAME_X - _OFF, FRAME_Y - _OFF)
P_FAR_RIGHT = (FRAME_X + _OFF, FRAME_Y - _OFF)
POCKETS = {"near_left": P_NEAR_LEFT, "far_left": P_FAR_LEFT, "near_right": P_NEAR_RIGHT, "far_right": P_FAR_RIGHT}
FRAME_GREY = (0.22, 0.23, 0.25)
_FZ = TOP + FRAME_H / 2
_INNER = FRAME - 2 * RIM
FRAME_PARTS = [
    Box("frame_rim_near", pos=(FRAME_X - _HALF + RIM / 2, FRAME_Y, _FZ), size=(RIM, FRAME, FRAME_H), color=FRAME_GREY),
    Box("frame_rim_far", pos=(FRAME_X + _HALF - RIM / 2, FRAME_Y, _FZ), size=(RIM, FRAME, FRAME_H), color=FRAME_GREY),
    Box("frame_rim_left", pos=(FRAME_X, FRAME_Y + _HALF - RIM / 2, _FZ), size=(_INNER, RIM, FRAME_H), color=FRAME_GREY),
    Box("frame_rim_right", pos=(FRAME_X, FRAME_Y - _HALF + RIM / 2, _FZ), size=(_INNER, RIM, FRAME_H),
        color=FRAME_GREY),
    Box("frame_web_middle", pos=(FRAME_X, FRAME_Y, _FZ), size=(_INNER, WEB, FRAME_H), color=FRAME_GREY),
    Box("frame_web_left", pos=(FRAME_X, FRAME_Y + _OFF, _FZ), size=(WEB, POCKET, FRAME_H), color=FRAME_GREY),
    Box("frame_web_right", pos=(FRAME_X, FRAME_Y - _OFF, _FZ), size=(WEB, POCKET, FRAME_H), color=FRAME_GREY),
]
# The aisle: the frame's outline grown by 5 cm on every side. A lifted tile must not be left lying in it.
AISLE_MARGIN = 0.05
AISLE_X = (FRAME_X - _HALF - AISLE_MARGIN, FRAME_X + _HALF + AISLE_MARGIN)
AISLE_Y = (FRAME_Y - _HALF - AISLE_MARGIN, FRAME_Y + _HALF + AISLE_MARGIN)

# The tiles: 7 x 7 x 3 cm, so each pocket leaves a 2.5 cm finger slot on every side (the stand-in for a suction lifter).
TILE_SIZE, TILE_KG = (0.07, 0.07, 0.03), 0.2
TILE_H = TILE_SIZE[2]
ORANGE = (0.95, 0.42, 0.05)
GREEN = (0.10, 0.55, 0.25)
TILE_POCKET = {
    "orange_tile_1": "near_left",
    "orange_tile_2": "far_right",
    "green_tile_1": "far_left",
    "green_tile_2": "near_right",
}
MARKED = ("orange_tile_1", "orange_tile_2")
UNMARKED = ("green_tile_1", "green_tile_2")
TILES = [
    Box(name, pos=(*POCKETS[p], TOP + TILE_H / 2 + 0.001), size=TILE_SIZE, mass=TILE_KG,
        color=ORANGE if name in MARKED else GREEN)
    for name, p in TILE_POCKET.items()
]  # each starts 1 mm above the pocket floor, and settles

# The staging pad, off the aisle in front of the left arm: its inner edge 10.5 cm beyond the frame's left edge.
PAD_X, PAD_Y = 0.30, 0.305
PAD_SIZE = (0.18, 0.11, 0.01)
PAD_TOP = TOP + PAD_SIZE[2]
PAD_IN_X = (PAD_X - PAD_SIZE[0] / 2, PAD_X + PAD_SIZE[0] / 2)
PAD_IN_Y = (PAD_Y - PAD_SIZE[1] / 2, PAD_Y + PAD_SIZE[1] / 2)
PAD_BLUE = (0.15, 0.35, 0.85)
PAD = Box("staging_pad", pos=(PAD_X, PAD_Y, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=PAD_BLUE)

# The cones: upright yellow cylinders standing in for a floor cone or barrier post.
CONE_R, CONE_H, CONE_KG = 0.02, 0.08, 0.115
CONE_YELLOW = (0.95, 0.80, 0.05)
CONE_START = {"cone_1": (0.25, -0.305), "cone_2": (0.35, -0.305)}
CONE_NAMES = tuple(CONE_START)
CONES = [
    Cylinder(name, pos=(x, y, TOP + CONE_H / 2 + 0.001), radius=CONE_R, height=CONE_H, mass=CONE_KG,
             color=CONE_YELLOW)
    for name, (x, y) in CONE_START.items()
]

OBJECTS = [*FRAME_PARTS, *TILES, PAD, *CONES]
LOOSE = (*TILE_POCKET, *CONE_NAMES)
SUCCESS_HOLD_S = 2.5  # done and still for 2.5 s: the cones stand and the staged tiles stay put

# Thresholds (each used below and quoted in task.json and the README). Heights are judged from object centres; a ruler
# gets one as the height of the object's top minus half its height.
FLOOR_TOL = 0.003  # a centre within 3 mm of its height when standing on the pocket floor counts as on the floor
SEAT_TILT_DEG = 3.0  # a tile lying in a pocket is tilted at most 3 degrees
SEAT_XY_TOL = 0.008  # a seated tile's centre is within 8 mm of its pocket's centre
CONE_UPRIGHT_DEG = 10.0  # a guarding cone is tilted at most 10 degrees
STAGE_FLAT_DEG = 5.0  # a staged tile is tilted at most 5 degrees
STAGE_Z_TOL = 0.003  # a staged tile's centre is within 3 mm of its height lying on the pad (or on the other staged tile)
POINTS = 6  # 1 point each: 2 cones, 2 staged tiles, 2 green tiles seated


def _tilt_deg(state) -> float:
    """Angle between the vertical and the object's end axis, either end up (what a level on its top face reads). The
    tiles and cones are symmetric, so one stood on its other end or lying face down reads 0 degrees."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _xy_dist(state, xy) -> float:
    return math.hypot(float(state.pos[0]) - xy[0], float(state.pos[1]) - xy[1])


def _centre_in_pocket(state, pocket) -> bool:
    """Centre over the pocket's 12 cm square, at any height."""
    px, py = POCKETS[pocket]
    return abs(float(state.pos[0]) - px) <= POCKET / 2 and abs(float(state.pos[1]) - py) <= POCKET / 2


def _on_floor(state, half_h) -> bool:
    """Standing on the pocket floor: centre within 3 mm of the table top plus half the object's height."""
    return abs(float(state.pos[2]) - (TOP + half_h)) <= FLOOR_TOL


def _lies_in(state, pocket) -> bool:
    """A tile closes the pocket: centre over it, on the floor, tilted at most 3 degrees."""
    return _centre_in_pocket(state, pocket) and _on_floor(state, TILE_H / 2) and _tilt_deg(state) <= SEAT_TILT_DEG


def _seated(state, pocket) -> bool:
    """A tile lies in the pocket and its centre is within 8 mm of the pocket's centre."""
    return _lies_in(state, pocket) and _xy_dist(state, POCKETS[pocket]) <= SEAT_XY_TOL


def _guarded(objects, pocket) -> bool:
    """An upright cone stands on the pocket floor with its centre over the pocket's 12 cm square (the walls keep it
    within about 4 cm of the centre)."""
    return any(
        _tilt_deg(objects[c]) <= CONE_UPRIGHT_DEG and _on_floor(objects[c], CONE_H / 2)
        and _centre_in_pocket(objects[c], pocket)
        for c in CONE_NAMES
    )


def _closed(objects, pocket) -> bool:
    return any(_lies_in(objects[t], pocket) for t in TILE_POCKET)


def _in_aisle(state) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    return AISLE_X[0] <= x <= AISLE_X[1] and AISLE_Y[0] <= y <= AISLE_Y[1]


def _on_pad_footprint(state) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    return PAD_IN_X[0] <= x <= PAD_IN_X[1] and PAD_IN_Y[0] <= y <= PAD_IN_Y[1]


def _flat_on_pad(state) -> bool:
    return (_on_pad_footprint(state) and _tilt_deg(state) <= STAGE_FLAT_DEG
            and abs(float(state.pos[2]) - (PAD_TOP + TILE_H / 2)) <= STAGE_Z_TOL)


def _staged(objects, tile) -> bool:
    """Flat over the pad: on the pad itself, or flat on the other orange tile while that one lies flat on the pad."""
    s = objects[tile]
    if _flat_on_pad(s):
        return True
    other = objects[MARKED[1] if tile == MARKED[0] else MARKED[0]]
    return bool(
        _on_pad_footprint(s) and _tilt_deg(s) <= STAGE_FLAT_DEG and _flat_on_pad(other)
        and abs(float(s.pos[2]) - (float(other.pos[2]) + TILE_H)) <= STAGE_Z_TOL
    )


def _hazard(objects) -> bool:
    """Anything that zeroes the score."""
    if any(float(objects[n].pos[2]) < TOP for n in LOOSE):
        return True  # something knocked off the table
    if not all(_lies_in(objects[g], TILE_POCKET[g]) for g in UNMARKED):
        return True  # a wrong panel pulled or shoved out of its pocket
    if not all(_closed(objects, p) or _guarded(objects, p) for p in POCKETS):
        return True  # an open pocket with no cone in it
    if any(_in_aisle(objects[t]) and not _lies_in(objects[t], TILE_POCKET[t]) for t in MARKED):
        return True  # a lifted panel left lying in the aisle
    return False


def score(objects):
    """Partial credit in sixths (success scores 1.0). For each orange tile's pocket: 1 point if no tile's centre is
    over it and an upright cone stands in it; then 1 more if that orange tile lies flat on the pad. If at least one
    of those pockets has its cone, 1 point for each green tile still seated. 0 overall if an open pocket has no cone,
    a green tile is out of its pocket, a lifted orange tile lies in the aisle, or anything is off the table."""
    if _hazard(objects):
        return 0.0
    points, coned = 0, 0
    for tile in MARKED:
        pocket = TILE_POCKET[tile]
        if not any(_centre_in_pocket(objects[t], pocket) for t in TILE_POCKET) and _guarded(objects, pocket):
            coned += 1
            points += 1 + _staged(objects, tile)
    if coned:
        points += sum(_seated(objects[g], TILE_POCKET[g]) for g in UNMARKED)
    return round(points / POINTS, 6)


def success(objects):
    """Both holes coned, both orange tiles staged, both green tiles seated, all still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in LOOSE)


# The demo. The left arm stages the near-left orange tile, then the far-right one, which the right arm leaves on the
# table at the handover spot on the midline behind the frame; then the left arm takes cone_2 from the same spot and
# stands it in the near-left hole. The right arm lifts the far-right tile to the handover spot, brings cone_2 there,
# then stands cone_1 in the far-right hole. Every grip is straight down with the fingers closing along world y.
TRAVEL_Z = TOP + 0.14  # a carried tile's bottom clears the frame by 8.8 cm; a carried cone's clears it by 6.5 cm
TILE_GRIP = 0.022  # TCP above a tile's bottom: the pads cover its sides from 9 mm up to its top
TILE_OPEN = -0.042  # fingers opened to 84 mm for a tile: 7 mm to the tile and 8 mm to the pocket wall on each side
CONE_GRIP = 0.045  # TCP above a cone's bottom: the fingertips stay 3.2 cm up, above the frame's top
DROP = 0.006  # each object is let go this far above where it lands
HANDOVER = (0.115, 0.0)  # on the midline, 4.5 cm behind the frame's near edge: both arms reach it
STAGE_X = (PAD_X - 0.04, PAD_X + 0.04)  # the two tiles side by side on the pad, 1 cm apart
LEFT_PARK = (0.15, 0.32, TRAVEL_Z)  # beside the pad, clear of it, so the camera sees the result
RIGHT_PARK = (0.15, -0.32, TRAVEL_Z)


def _pick_place(pick, place, open_f):
    """Top-down pick and place with the fingers opened to `open_f`, carried at TRAVEL_Z."""
    (px, py, pz), (qx, qy, qz) = pick, place
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), open_f, "to the object"),
        Waypoint(1.5, (px, py, pz), open_f, "descend"),
        Waypoint(0.8, (px, py, pz), FINGER_CLOSED, "close"),
        Waypoint(1.2, (px, py, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (qx, qy, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.2, (qx, qy, qz), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (qx, qy, qz), open_f, "release"),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), open_f, "retreat"),
    ]


def demo():
    tile_in = TOP + TILE_GRIP
    tile_on_table = TOP + TILE_GRIP + DROP
    tile_on_pad = PAD_TOP + TILE_GRIP + DROP
    cone_up = TOP + CONE_GRIP
    cone_down = TOP + CONE_GRIP + DROP
    left = _pick_place((*P_NEAR_LEFT, tile_in), (STAGE_X[0], PAD_Y, tile_on_pad), TILE_OPEN)
    left += _pick_place((*HANDOVER, TOP + TILE_GRIP), (STAGE_X[1], PAD_Y, tile_on_pad), TILE_OPEN)
    left += _pick_place((*HANDOVER, cone_up), (*P_NEAR_LEFT, cone_down), FINGER_OPEN)
    left.append(Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"))
    right = _pick_place((*P_FAR_RIGHT, tile_in), (*HANDOVER, tile_on_table), TILE_OPEN)
    right += _pick_place((*CONE_START["cone_2"], cone_up), (*HANDOVER, cone_down), FINGER_OPEN)
    right += _pick_place((*CONE_START["cone_1"], cone_up), (*P_FAR_RIGHT, cone_down), FINGER_OPEN)
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
