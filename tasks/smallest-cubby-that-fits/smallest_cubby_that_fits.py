"""On the station table, in front of the left arm: a blue delivery mat with four brown cardboard parcels standing on
it, two small (4.5 x 4.5 cm, 6 cm tall, 100 g), one medium (6 x 6 cm, 6.5 cm tall, 250 g) and one large (7 cm front
to back, 11 cm side to side, 8.5 cm tall, 450 g). On the midline, 22 cm in front of the arm bases: a green handover
mat. In front of the right arm: a fixed dark grey sorting tray with four open-top compartments in a row, all 8 cm
front to back inside, centred 27 cm in front of the arm bases; from the midline outward they are 12.5 cm (large),
7.5 cm (medium), 5.5 cm and 5.5 cm (small) wide, with 5 mm walls standing 3 cm above a 4 mm floor. The mats and the
tray are fixed; the parcels are loose. Sorted, each parcel stands alone, flat on any face, on the floor of the
smallest compartment it fits: the small parcels in the small compartments, the medium parcel in the medium one, the large parcel in the
large one. The left arm cannot reach the tray, so every parcel is handed over on the green mat.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The sorting tray: four compartments in a row along y (to the robot's right of the midline), 8 cm deep front to back.
TRAY_X = 0.27
DEPTH = 0.080
WALL_T, RIM, FLOOR_T = 0.005, 0.030, 0.004  # walls 5 mm thick, standing 3 cm above the 4 mm floor
WALL_H = FLOOR_T + RIM
FLOOR = TOP + FLOOR_T
IN_X = (TRAY_X - DEPTH / 2, TRAY_X + DEPTH / 2)
Y_NEAR = -0.100  # inner face of the tray's end wall nearest the midline
LARGE_W, MEDIUM_W, SMALL_W = 0.125, 0.075, 0.055  # inside widths along y
LARGE_Y_IN = (Y_NEAR - LARGE_W, Y_NEAR)  # -0.225 to -0.100
MEDIUM_Y_IN = (LARGE_Y_IN[0] - WALL_T - MEDIUM_W, LARGE_Y_IN[0] - WALL_T)  # -0.305 to -0.230
SMALL_A_Y_IN = (MEDIUM_Y_IN[0] - WALL_T - SMALL_W, MEDIUM_Y_IN[0] - WALL_T)  # -0.365 to -0.310
SMALL_B_Y_IN = (SMALL_A_Y_IN[0] - WALL_T - SMALL_W, SMALL_A_Y_IN[0] - WALL_T)  # -0.425 to -0.370
LARGE_Y, MEDIUM_Y, SMALL_A_Y, SMALL_B_Y = (sum(r) / 2 for r in (LARGE_Y_IN, MEDIUM_Y_IN, SMALL_A_Y_IN, SMALL_B_Y_IN))
SMALL, MEDIUM, LARGE = 0, 1, 2  # size classes
COMPARTMENTS = {  # name: (inside span along y, size class)
    "large": (LARGE_Y_IN, LARGE),
    "medium": (MEDIUM_Y_IN, MEDIUM),
    "small_a": (SMALL_A_Y_IN, SMALL),
    "small_b": (SMALL_B_Y_IN, SMALL),
}
TRAY_GREY = (0.20, 0.21, 0.23)
_WZ = TOP + WALL_H / 2
_OUTER_Y = (SMALL_B_Y_IN[0] - WALL_T, Y_NEAR + WALL_T)  # the tray's full extent along y
_ENDS = {  # walls across the row, each 8 cm long front to back
    "tray_end_inner": Y_NEAR + WALL_T / 2,
    "tray_divider_1": LARGE_Y_IN[0] - WALL_T / 2,
    "tray_divider_2": MEDIUM_Y_IN[0] - WALL_T / 2,
    "tray_divider_3": SMALL_A_Y_IN[0] - WALL_T / 2,
    "tray_end_outer": SMALL_B_Y_IN[0] - WALL_T / 2,
}
TRAY = [
    *(
        Box(f"tray_floor_{name}", pos=(TRAY_X, sum(span) / 2, TOP + FLOOR_T / 2),
            size=(DEPTH, span[1] - span[0], FLOOR_T), color=TRAY_GREY)
        for name, (span, _) in COMPARTMENTS.items()
    ),
    *(Box(name, pos=(TRAY_X, y, _WZ), size=(DEPTH, WALL_T, WALL_H), color=TRAY_GREY) for name, y in _ENDS.items()),
    Box("tray_wall_near", pos=(IN_X[0] - WALL_T / 2, sum(_OUTER_Y) / 2, _WZ),
        size=(WALL_T, _OUTER_Y[1] - _OUTER_Y[0], WALL_H), color=TRAY_GREY),
    Box("tray_wall_far", pos=(IN_X[1] + WALL_T / 2, sum(_OUTER_Y) / 2, _WZ),
        size=(WALL_T, _OUTER_Y[1] - _OUTER_Y[0], WALL_H), color=TRAY_GREY),
]

# The mats: 3 mm thick, fixed.
MAT_T = 0.003
MAT_TOP = TOP + MAT_T
DELIVERY_MAT_POS, DELIVERY_MAT_SIZE = (0.26, 0.29), (0.28, 0.24)
HANDOVER_X, HANDOVER_Y, HANDOVER_SIZE = 0.22, 0.0, 0.15
HANDOVER_TOP = MAT_TOP
MATS = [
    Box("delivery_mat", pos=(*DELIVERY_MAT_POS, TOP + MAT_T / 2), size=(*DELIVERY_MAT_SIZE, MAT_T),
        color=(0.15, 0.30, 0.75)),
    Box("handover_mat", pos=(HANDOVER_X, HANDOVER_Y, TOP + MAT_T / 2), size=(HANDOVER_SIZE, HANDOVER_SIZE, MAT_T),
        color=(0.15, 0.60, 0.25)),
]

# The parcels, all the same cardboard brown: only their size tells them apart.
SMALL_SIZE, SMALL_KG = (0.045, 0.045, 0.060), 0.10
MEDIUM_SIZE, MEDIUM_KG = (0.060, 0.060, 0.065), 0.25
# 8.5 cm tall so that, on whichever face it stands, the large parcel never fits the 7.5 x 8 cm medium compartment
LARGE_SIZE, LARGE_KG = (0.070, 0.110, 0.085), 0.45  # long side along y (side to side)
PARCEL_BROWN = (0.72, 0.45, 0.20)
SMALL_1_START, SMALL_2_START = (0.18, 0.36), (0.32, 0.36)
MEDIUM_START, LARGE_START = (0.18, 0.24), (0.33, 0.24)
PARCEL_SPEC = {  # name: (size, mass, start x and y, size class)
    "small_parcel_1": (SMALL_SIZE, SMALL_KG, SMALL_1_START, SMALL),
    "small_parcel_2": (SMALL_SIZE, SMALL_KG, SMALL_2_START, SMALL),
    "medium_parcel": (MEDIUM_SIZE, MEDIUM_KG, MEDIUM_START, MEDIUM),
    "large_parcel": (LARGE_SIZE, LARGE_KG, LARGE_START, LARGE),
}
PARCELS = [
    Box(name, pos=(*xy, MAT_TOP + size[2] / 2 + 0.001), size=size, mass=kg, color=PARCEL_BROWN)
    for name, (size, kg, xy, _) in PARCEL_SPEC.items()
]  # each starts 1 mm above the mat, and settles
PARCEL_NAMES = tuple(PARCEL_SPEC)

OBJECTS = [*TRAY, *MATS, *PARCELS]
SUCCESS_HOLD_S = 2.5  # sorted and still for 2.5 s

PARCEL_CREDIT = 0.25  # per parcel seated alone in a compartment of the smallest size it fits
IN_BELOW = 0.030  # in a compartment: the parcel's bottom less than 3 cm above its floor (below the rim), or
STACK_TOL = 0.003  # resting on top of another parcel: its bottom within 3 mm of that parcel's top, and
STILL_SPEED = 0.01  # still: moving under 1 cm/s (a parcel carried in the gripper is moving)
FLOOR_TOL = 0.005  # seated: the parcel's bottom within 5 mm of the compartment floor
FLAT_DEG = 10.0  # seated: one of its faces, any face, tilted at most 10 degrees from horizontal
DROPPED_BELOW = 0.05  # fallen off the table: the parcel's bottom more than 5 cm below the table top


def _tilt_deg(state) -> float:
    """Angle between the horizontal and whichever of the parcel's faces is nearest to it: 0 standing flat on any face
    (bottom, top or side), 45 balanced on an edge."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    up = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))  # each box axis's vertical component
    return math.degrees(math.acos(min(1.0, max(abs(c) for c in up))))


def _over(state):
    """The compartment whose inner outline contains the parcel's centre (seen from above, at any height), or None."""
    x, y = float(state.pos[0]), float(state.pos[1])
    if not IN_X[0] <= x <= IN_X[1]:
        return None
    for name, ((y0, y1), _) in COMPARTMENTS.items():
        if y0 <= y <= y1:
            return name
    return None


def _compartments(parcels):
    """Which compartment each parcel is in, or None. A parcel is in a compartment when its centre is over it and its
    bottom is below the rim (less than IN_BELOW above the floor), or it rests on top of another parcel over the same
    compartment (its bottom within STACK_TOL of that parcel's top, moving under STILL_SPEED). A parcel carried higher,
    over the tray, is in none."""
    over = {name: _over(s) for name, s in parcels.items()}
    where = {}
    for name, s in parcels.items():
        comp, bottom = over[name], float(s.low[2])
        still = s.resting(max_speed=STILL_SPEED)
        stacked = still and any(
            other != name and over[other] == comp and abs(bottom - float(o.high[2])) <= STACK_TOL
            for other, o in parcels.items()
        )
        where[name] = comp if comp is not None and (bottom - FLOOR < IN_BELOW or stacked) else None
    return where


def _seated(state) -> bool:
    """Flat on the compartment floor: bottom within FLOOR_TOL of it, one face at most FLAT_DEG from level."""
    return bool(abs(float(state.low[2]) - FLOOR) <= FLOOR_TOL and _tilt_deg(state) <= FLAT_DEG)


def _dropped(state) -> bool:
    return bool(float(state.low[2]) < TOP - DROPPED_BELOW)


def score(objects):
    """Partial credit (success scores 1.0): 0.25 for each parcel seated alone in a compartment of the smallest size it
    fits. A parcel sharing a compartment with another parcel earns nothing. 0 overall if any parcel is in a compartment
    larger than the smallest it fits, or any parcel has fallen off the table. A parcel carried above the rim and not
    resting on another parcel is in no compartment, so passing over one is not penalised."""
    parcels = {name: objects[name] for name in PARCEL_NAMES}
    if any(_dropped(s) for s in parcels.values()):
        return 0.0
    where = _compartments(parcels)
    for name, comp in where.items():
        if comp is not None and COMPARTMENTS[comp][1] > PARCEL_SPEC[name][3]:
            return 0.0  # a wasted slot: the parcel fits a smaller compartment
    held = [comp for comp in where.values() if comp is not None]
    seated = sum(
        1
        for name, comp in where.items()
        if comp is not None
        and COMPARTMENTS[comp][1] == PARCEL_SPEC[name][3]
        and held.count(comp) == 1
        and _seated(parcels[name])
    )
    return round(PARCEL_CREDIT * seated, 6)


def success(objects):
    """All four parcels seated alone in their smallest compartments and still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[name].resting() for name in PARCEL_NAMES)


# The demo: the left arm carries each parcel from the delivery mat to the handover mat; the right arm takes it from
# there to its compartment. Both grip straight down with the fingers closing front to back, so the open fingers
# never reach into the neighbouring compartments along the row. The left arm turns its grip -90 degrees and the
# right arm +90 degrees (a parallel gripper grips the same either way): at +90 degrees the left arm's wrist runs out
# of range over the handover mat. The left arm fetches the next parcel while the right arm is placing the last one,
# and holds it above its start until the handover mat is clear.
LEFT_YAW, RIGHT_YAW = -math.pi / 2, math.pi / 2
TRAVEL_Z = TOP + 0.15  # a carried parcel's bottom clears the tray walls by 4.6 cm and placed parcels by 1.1 cm
GRIP_BELOW_TOP = 0.015  # the TCP 1.5 cm below a parcel's top: the pads cover its top 2.8 cm, the tips 3.2 cm above
DROP = 0.008  # each parcel is let go this far above where it lands
HANDOVER_WAIT_S = 2.5  # the left arm holds the next parcel until the right arm has carried the last one away
RIGHT_LAG_S = 0.3  # the right arm sets off for the handover mat this long after the left arm has let go there
LEFT_PARK = (0.15, 0.30, TRAVEL_Z)
RIGHT_PARK = (0.15, -0.36, TRAVEL_Z)  # also where the right arm waits for the first handover
ORDER = ("large_parcel", "medium_parcel", "small_parcel_1", "small_parcel_2")
TARGET_Y = {"large_parcel": LARGE_Y, "medium_parcel": MEDIUM_Y, "small_parcel_1": SMALL_A_Y,
            "small_parcel_2": SMALL_B_Y}
PICK_PLACE_S = 9.6  # pick_place_waypoints at speed 1


def _tcp_z(surface: float, name: str) -> float:
    """TCP height to grip the parcel standing on ``surface``."""
    return surface + PARCEL_SPEC[name][0][2] - GRIP_BELOW_TOP


def demo():
    left = []
    right = [Waypoint(PICK_PLACE_S + RIGHT_LAG_S, RIGHT_PARK, FINGER_OPEN, "wait for the first handover", RIGHT_YAW)]
    for i, name in enumerate(ORDER):
        x, y = PARCEL_SPEC[name][2]
        leg = pick_place_waypoints(
            (x, y, _tcp_z(MAT_TOP, name)), (HANDOVER_X, HANDOVER_Y, _tcp_z(HANDOVER_TOP, name) + DROP),
            travel_z=TRAVEL_Z, yaw=LEFT_YAW,
        )
        if i > 0:
            leg.insert(4, Waypoint(HANDOVER_WAIT_S, (x, y, TRAVEL_Z), FINGER_CLOSED, "wait for the handover mat",
                                   LEFT_YAW))
            right.append(Waypoint(HANDOVER_WAIT_S, right[-1].pos, FINGER_OPEN, "wait for the next handover", RIGHT_YAW))
        left += leg
        right += pick_place_waypoints(
            (HANDOVER_X, HANDOVER_Y, _tcp_z(HANDOVER_TOP, name)), (TRAY_X, TARGET_Y[name], _tcp_z(FLOOR, name) + DROP),
            travel_z=TRAVEL_Z, yaw=RIGHT_YAW,
        )
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park", LEFT_YAW))
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park", RIGHT_YAW))
    return {"left": left, "right": right}
