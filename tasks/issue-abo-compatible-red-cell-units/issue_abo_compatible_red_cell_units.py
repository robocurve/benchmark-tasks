"""A blood-bank issue bench on the station table. On the robot's left, a fixed dark grey issue shelf (a 315 x 110 mm
base plate, 10 mm thick, with three 15 mm dividers making four slots) runs front to back, its centre line 40 cm left of
the midline. Four red-cell units stand in its slots, each a 60 x 40 x 90 mm box of 300 g whose colour is its ABO group:
blue (O), white (AB), pink (B) and blue (O), 15, 22.5, 30 and 37.5 cm in front of the arm bases. On the midline, 25 cm
in front of the bases, lies a fixed grey handover pad (100 x 100 x 10 mm). On the robot's right, three fixed open trays
(inside 90 mm front to back and 120 mm side to side, 40 mm walls) stand in a row front to back, 40 cm right of the
midline: blue (patient group O) at 15 cm, yellow (patient group A) at 26 cm and pink (patient group B) at 37 cm. Each
tray must end with exactly one compatible unit standing in it (the patient's own group, or O; no group A unit is in
stock), the white AB unit stays on the shelf, and no unit is left anywhere else.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

DARK_GREY = (0.20, 0.21, 0.23)
PAD_GREY = (0.45, 0.45, 0.48)
BLUE = (0.10, 0.30, 0.85)
PINK = (0.92, 0.30, 0.62)
YELLOW = (0.95, 0.78, 0.05)
WHITE = (0.95, 0.95, 0.95)  # the usual AB label colour; it stands on the dark shelf so it shows in the top camera

# The issue shelf: a base plate with dividers between four slots, front to back in front of the left arm.
SHELF_Y = 0.40
SLOT_X = (0.15, 0.225, 0.30, 0.375)  # slot centres, 7.5 cm apart
PLATE_T = 0.010
PLATE_TOP = TOP + PLATE_T
SHELF_X0, SHELF_X1 = 0.105, 0.42  # the plate's near and far edges
SHELF_W = 0.11  # the plate's width side to side
SHELF_Y0, SHELF_Y1 = SHELF_Y - SHELF_W / 2, SHELF_Y + SHELF_W / 2
DIVIDER_T, DIVIDER_H = 0.005, 0.015
SHELF = [
    Box("shelf_plate", pos=((SHELF_X0 + SHELF_X1) / 2, SHELF_Y, TOP + PLATE_T / 2),
        size=(SHELF_X1 - SHELF_X0, SHELF_W, PLATE_T), color=DARK_GREY),
    *(
        Box(f"shelf_divider_{i + 1}", pos=((SLOT_X[i] + SLOT_X[i + 1]) / 2, SHELF_Y, PLATE_TOP + DIVIDER_H / 2),
            size=(DIVIDER_T, SHELF_W, DIVIDER_H), color=DARK_GREY)
        for i in range(3)
    ),
]

# The handover pad on the midline, which both arms reach.
PAD_X, PAD_Y = 0.25, 0.0
PAD_T = 0.010
PAD_TOP = TOP + PAD_T
PAD = Box("handover_pad", pos=(PAD_X, PAD_Y, TOP + PAD_T / 2), size=(0.10, 0.10, PAD_T), color=PAD_GREY)

# The patient trays: a 4 mm dark grey floor inside four 5 mm walls, 40 mm tall, coloured by the patient's ABO group.
TRAY_Y = -0.40
TRAY_O_X, TRAY_A_X, TRAY_B_X = 0.15, 0.26, 0.37
TRAY_X = {"O": TRAY_O_X, "A": TRAY_A_X, "B": TRAY_B_X}
TRAY_COLOR = {"O": BLUE, "A": YELLOW, "B": PINK}
TRAY_NAME = {"O": "blue_tray", "A": "yellow_tray", "B": "pink_tray"}
INNER_X, INNER_Y = 0.09, 0.12
WALL_T, WALL_H, FLOOR_T = 0.005, 0.040, 0.004
FLOOR = TOP + FLOOR_T
RIM = TOP + WALL_H


def _tray(group):
    x, name, color = TRAY_X[group], TRAY_NAME[group], TRAY_COLOR[group]
    wz = TOP + WALL_H / 2
    return [
        Box(f"{name}_floor", pos=(x, TRAY_Y, TOP + FLOOR_T / 2), size=(INNER_X, INNER_Y, FLOOR_T), color=DARK_GREY),
        Box(f"{name}_wall_near", pos=(x - INNER_X / 2 - WALL_T / 2, TRAY_Y, wz),
            size=(WALL_T, INNER_Y + 2 * WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(x + INNER_X / 2 + WALL_T / 2, TRAY_Y, wz),
            size=(WALL_T, INNER_Y + 2 * WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_left", pos=(x, TRAY_Y + INNER_Y / 2 + WALL_T / 2, wz), size=(INNER_X, WALL_T, WALL_H),
            color=color),
        Box(f"{name}_wall_right", pos=(x, TRAY_Y - INNER_Y / 2 - WALL_T / 2, wz), size=(INNER_X, WALL_T, WALL_H),
            color=color),
    ]


TRAYS = [part for group in TRAY_X for part in _tray(group)]

# The red-cell units: 60 mm front to back, 40 mm side to side, 90 mm tall; each starts 1 mm above its slot.
UNIT_SIZE, UNIT_KG = (0.06, 0.04, 0.09), 0.3
UNIT_H = UNIT_SIZE[2]
UNIT_START = {"blue_unit_1": SLOT_X[0], "white_unit": SLOT_X[1], "pink_unit": SLOT_X[2], "blue_unit_2": SLOT_X[3]}
UNIT_GROUP = {"blue_unit_1": "O", "white_unit": "AB", "pink_unit": "B", "blue_unit_2": "O"}
UNIT_COLOR = {"O": BLUE, "AB": WHITE, "B": PINK}
UNIT_NAMES = tuple(UNIT_START)
UNITS = [
    Box(name, pos=(x, SHELF_Y, PLATE_TOP + UNIT_H / 2 + 0.001), size=UNIT_SIZE, mass=UNIT_KG,
        color=UNIT_COLOR[UNIT_GROUP[name]])
    for name, x in UNIT_START.items()
]

OBJECTS = [*SHELF, PAD, *TRAYS, *UNITS]
SUCCESS_HOLD_S = 2.5  # every tray filled and every unit still for 2.5 s

# Which donor groups each patient can receive: their own group or O.
COMPATIBLE = {"O": ("O",), "A": ("O",), "B": ("B", "O")}
TRAY_CREDIT = 1 / 3  # each tray holding exactly one compatible unit, seated and upright
IN_TOL = 0.005  # a unit is over a tray when its centre is inside the tray's inside outline or within 5 mm past it
SEAT_DOWN, SEAT_UP = 0.008, 0.0005  # seated: the unit's lowest point 8 mm below to 0.5 mm above the tray floor
UPRIGHT_DEG = 15.0  # upright: the 90 mm axis within 15 degrees of vertical, either end up
LOOSE_H = 0.05  # a unit outside the shelf and trays with its lowest point under 5 cm up is loose
STACK_TOL = 0.005  # ... and so is one whose lowest point is within 5 mm above another unit's top, outlines overlapping
WHITE_LIFT = 0.02  # the white unit is in storage only while its lowest point is at most 2 cm above the plate's top


def _tilt_deg(state) -> float:
    """Angle between the unit's 90 mm axis and the vertical, either end up."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _centre_in(state, x0, x1, y0, y1) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    return bool(x0 <= x <= x1 and y0 <= y <= y1)


def _tray_of(state):
    """The patient group of the tray the unit's centre is over (at any height), or None."""
    for group, x in TRAY_X.items():
        if _centre_in(state, x - INNER_X / 2 - IN_TOL, x + INNER_X / 2 + IN_TOL,
                      TRAY_Y - INNER_Y / 2 - IN_TOL, TRAY_Y + INNER_Y / 2 + IN_TOL):
            return group
    return None


def _on_shelf(state) -> bool:
    """The unit's centre over the shelf's base plate (at any height): still in storage."""
    return _centre_in(state, SHELF_X0, SHELF_X1, SHELF_Y0, SHELF_Y1)


def _white_stored(state) -> bool:
    """The white unit over the shelf's base plate and not lifted: standing, lying or leaning in its slot."""
    return _on_shelf(state) and float(state.low[2]) <= PLATE_TOP + WHITE_LIFT


def _rests_on(state, other) -> bool:
    """The unit's lowest point within STACK_TOL above the other unit's top, their outlines overlapping from above."""
    gap = float(state.low[2]) - float(other.high[2])
    overlap = all(float(state.low[i]) < float(other.high[i]) and float(other.low[i]) < float(state.high[i])
                  for i in (0, 1))
    return bool(-STACK_TOL <= gap <= STACK_TOL and overlap)


def _loose(name, units) -> bool:
    """Outside storage and the trays, and set down: low (on the pad, the table or the floor) or resting on another
    unit. Only a unit held 5 cm or more up with nothing under it is exempt."""
    state = units[name]
    if _on_shelf(state) or _tray_of(state) is not None:
        return False
    if float(state.low[2]) < TOP + LOOSE_H:
        return True
    return any(_rests_on(state, other) for n, other in units.items() if n != name)


def _seated_upright(state) -> bool:
    """Standing on the tray floor (not held above it, not on a wall or another unit), upright and still."""
    bottom = float(state.low[2])
    return bool(
        FLOOR - SEAT_DOWN <= bottom <= FLOOR + SEAT_UP and _tilt_deg(state) <= UPRIGHT_DEG and state.resting()
    )


def score(objects):
    """Partial credit (success scores 1.0): 1/3 for each tray holding exactly one unit its patient can receive,
    seated on the tray floor, upright and still. 0 overall if the white (AB) unit is not in storage (its centre off
    the shelf's base plate, or lifted more than 2 cm above it, held or not), if any unit is over a tray whose patient
    can't receive it (pink in the blue or yellow tray), or if any unit is loose (set down outside the shelf and the
    trays, on the pad, the table, the floor or another unit)."""
    units = {name: objects[name] for name in UNIT_NAMES}
    if not _white_stored(units["white_unit"]):
        return 0.0  # the AB unit has left storage: no patient here can receive it
    if any(_loose(name, units) for name in units):
        return 0.0
    tray_of = {name: _tray_of(s) for name, s in units.items()}
    if any(g is not None and UNIT_GROUP[name] not in COMPATIBLE[g] for name, g in tray_of.items()):
        return 0.0
    credit = 0.0
    for group in TRAY_X:
        inside = [name for name, g in tray_of.items() if g == group]
        if len(inside) == 1 and _seated_upright(units[inside[0]]):
            credit += TRAY_CREDIT
    return round(credit, 6)


def success(objects):
    """Every tray holds one compatible unit, seated and upright, and every unit is still; must hold for
    SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[name].resting() for name in UNIT_NAMES)


# The demo: three issues, each a handover. The left arm lifts a unit from its slot and stands it on the pad; once it
# has risen clear, the right arm picks it from the pad and stands it in its tray. Both grip straight down across the
# unit's 40 mm side (fingers closing along world y). The carries never pass over a standing unit.
TRAVEL_Z = TOP + 0.14  # a carried unit's bottom 8.5 cm up: clear of the pad, the dividers and the tray walls
GRIP = 0.055  # TCP above a unit's bottom: the pads cover 42 to 75 mm up its side, the fingertips clear the dividers
DROP = 0.006  # each unit is let go this far above where it lands
ISSUES = (("blue_unit_1", "A"), ("pink_unit", "B"), ("blue_unit_2", "O"))
HOLD_S = 2.0  # the left arm holds each lifted unit above the shelf until the right arm has carried the last one off
LEFT_CYCLE_S = 9.6 + HOLD_S  # one pick and place of 9.6 s plus the hold
RIGHT_START_S = LEFT_CYCLE_S + 0.5  # the right arm sets off for the pad 0.5 s after the left arm has risen from it
RIGHT_WAIT = (0.26, -0.25, TRAVEL_Z)  # where the right arm waits for the first unit, clear of the pad and trays
CLEAR_Z = TOP + 0.17
LEFT_PARK = (0.20, 0.22, CLEAR_Z)  # beside the shelf, away from the pad
RIGHT_PARK = (0.22, -0.20, CLEAR_Z)  # between the pad and the trays, so the camera sees the trays


def demo():
    left = []
    for name, _ in ISSUES:
        pick = (UNIT_START[name], SHELF_Y, PLATE_TOP + GRIP)
        wps = pick_place_waypoints(pick, (PAD_X, PAD_Y, PAD_TOP + GRIP + DROP), travel_z=TRAVEL_Z)
        wps.insert(4, Waypoint(HOLD_S, (pick[0], pick[1], TRAVEL_Z), wps[3].finger, "hold until the pad is clear"))
        left += wps
    left.append(Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"))
    right = [Waypoint(RIGHT_START_S, RIGHT_WAIT, FINGER_OPEN, "wait for the first unit")]
    for i, (_, group) in enumerate(ISSUES):
        if i:
            right.append(Waypoint(HOLD_S, (TRAY_X[ISSUES[i - 1][1]], TRAY_Y, TRAVEL_Z), FINGER_OPEN,
                                  "wait for the next unit"))
        right += pick_place_waypoints(
            (PAD_X, PAD_Y, PAD_TOP + GRIP), (TRAY_X[group], TRAY_Y, FLOOR + GRIP + DROP), travel_z=TRAVEL_Z,
        )
    right.append(Waypoint(3.0, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
