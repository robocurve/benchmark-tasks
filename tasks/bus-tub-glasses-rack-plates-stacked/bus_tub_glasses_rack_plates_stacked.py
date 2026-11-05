"""On the station table, between the arms: a flat black serving tray (22 cm front to back, 36 cm side to side, 5 mm
thick), its centre 28 cm in front of the arm bases. On the tray's right half stand two blue glass tumblers (6 cm
across, 10 cm tall, 100 g each), 14.5 cm right of centre, 22 and 32 cm in front of the arm bases. On its left half
lie three orange ceramic plates, 3 cm thick, all wholly on the tray: a small one (5 cm across, 100 g) 14.5 cm left of
centre and 20 cm in front of the arm bases, then, 13.5 cm left of centre, a large one (8 cm across, 200 g) at 27.5 cm and a medium one (6.5 cm across, 150 g) at 35.5 cm. In
front of the left arm, centred 30 cm out, is a green glass rack: a 17.5 cm square base with walls and a cross divider
5 cm tall making four 8 cm square cells. In front of the right arm, centred 30 cm out, is a dark grey bus tub (inside
22 cm front to back, 16 cm side to side, walls 6 cm tall). The tray, rack and tub are fixed; the glasses and plates
are loose. Cleared, each glass stands upright in its own rack cell and the plates lie in one stack on the tub's floor,
the large plate at the bottom, the medium on it and the small on top.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The serving tray: a 5 mm sheet on the midline. The demo hands every item over at PASS_XY, on the tray's midline.
TRAY_X, TRAY_Y = 0.28, 0.0
TRAY_SIZE = (0.22, 0.36, 0.005)
TRAY_TOP = TOP + TRAY_SIZE[2]
TRAY = Box("tray", pos=(TRAY_X, TRAY_Y, TOP + TRAY_SIZE[2] / 2), size=TRAY_SIZE, color=(0.06, 0.06, 0.07))
PASS_XY = (0.22, 0.0)

# The glass rack, in front of the left arm: a 5 mm base, outer walls and a cross divider 5 cm tall, 5 mm thick,
# around four 8 x 8 cm cells.
RACK_X, RACK_Y = 0.30, 0.305
CELL, DIV_T, RACK_WALL_H, RACK_BASE_T = 0.08, 0.005, 0.05, 0.005
RACK_SPAN = 2 * CELL + 3 * DIV_T  # 17.5 cm outside
RACK_FLOOR = TOP + RACK_BASE_T
RACK_LO = (RACK_X - RACK_SPAN / 2, RACK_Y - RACK_SPAN / 2)
RACK_HI = (RACK_X + RACK_SPAN / 2, RACK_Y + RACK_SPAN / 2)
_P = (CELL + DIV_T) / 2  # cell centre from the rack's centre, along x and y
CELLS = (
    (RACK_X + _P, RACK_Y - _P),  # far, nearer the midline
    (RACK_X - _P, RACK_Y - _P),  # near, nearer the midline
    (RACK_X + _P, RACK_Y + _P),
    (RACK_X - _P, RACK_Y + _P),
)
RACK_GREEN = (0.10, 0.55, 0.20)
_RZ = RACK_FLOOR + RACK_WALL_H / 2
_WALL_OFF = (-(CELL + DIV_T), 0.0, CELL + DIV_T)
RACK = [
    Box("rack_base", pos=(RACK_X, RACK_Y, TOP + RACK_BASE_T / 2), size=(RACK_SPAN, RACK_SPAN, RACK_BASE_T),
        color=RACK_GREEN),
    *(Box(f"rack_wall_x{i}", pos=(RACK_X + off, RACK_Y, _RZ), size=(DIV_T, RACK_SPAN, RACK_WALL_H), color=RACK_GREEN)
      for i, off in enumerate(_WALL_OFF)),
    *(Box(f"rack_wall_y{i}", pos=(RACK_X, RACK_Y + off, _RZ), size=(RACK_SPAN, DIV_T, RACK_WALL_H), color=RACK_GREEN)
      for i, off in enumerate(_WALL_OFF)),
]

# The bus tub, in front of the right arm: a 4 mm floor and four 5 mm walls, 6 cm tall, around a 22 x 16 cm inside.
TUB_X, TUB_Y = 0.30, -0.305
TUB_IN = (0.22, 0.16)
TUB_WALL_T, TUB_WALL_H, TUB_FLOOR_T = 0.005, 0.06, 0.004
TUB_FLOOR = TOP + TUB_FLOOR_T
TUB_IN_X = (TUB_X - TUB_IN[0] / 2, TUB_X + TUB_IN[0] / 2)
TUB_IN_Y = (TUB_Y - TUB_IN[1] / 2, TUB_Y + TUB_IN[1] / 2)
TUB_OUT_X = (TUB_IN_X[0] - TUB_WALL_T, TUB_IN_X[1] + TUB_WALL_T)
TUB_OUT_Y = (TUB_IN_Y[0] - TUB_WALL_T, TUB_IN_Y[1] + TUB_WALL_T)
TUB_GREY = (0.25, 0.26, 0.28)
_TZ = TOP + TUB_WALL_H / 2
TUB = [
    Box("tub_floor", pos=(TUB_X, TUB_Y, TOP + TUB_FLOOR_T / 2), size=(*TUB_IN, TUB_FLOOR_T), color=TUB_GREY),
    Box("tub_wall_near", pos=(TUB_IN_X[0] - TUB_WALL_T / 2, TUB_Y, _TZ),
        size=(TUB_WALL_T, TUB_IN[1] + 2 * TUB_WALL_T, TUB_WALL_H), color=TUB_GREY),
    Box("tub_wall_far", pos=(TUB_IN_X[1] + TUB_WALL_T / 2, TUB_Y, _TZ),
        size=(TUB_WALL_T, TUB_IN[1] + 2 * TUB_WALL_T, TUB_WALL_H), color=TUB_GREY),
    Box("tub_wall_left", pos=(TUB_X, TUB_IN_Y[1] + TUB_WALL_T / 2, _TZ), size=(TUB_IN[0], TUB_WALL_T, TUB_WALL_H),
        color=TUB_GREY),
    Box("tub_wall_right", pos=(TUB_X, TUB_IN_Y[0] - TUB_WALL_T / 2, _TZ), size=(TUB_IN[0], TUB_WALL_T, TUB_WALL_H),
        color=TUB_GREY),
]

GLASS_R, GLASS_H, GLASS_KG = 0.03, 0.10, 0.10
GLASS_BLUE = (0.10, 0.45, 0.95)  # saturated: pale objects vanish from the top camera
GLASS_START = {"glass_1": (0.22, -0.145), "glass_2": (0.32, -0.145)}
GLASS_NAMES = tuple(GLASS_START)
GLASSES = [
    Cylinder(n, pos=(x, y, TRAY_TOP + GLASS_H / 2 + 0.001), radius=GLASS_R, height=GLASS_H, mass=GLASS_KG,
             color=GLASS_BLUE)
    for n, (x, y) in GLASS_START.items()
]  # each starts 1 mm above the tray, and settles

PLATE_H = 0.03
PLATE_ORANGE = (0.95, 0.42, 0.05)
PLATE_R = {"plate_large": 0.040, "plate_medium": 0.0325, "plate_small": 0.025}
PLATE_KG = {"plate_large": 0.20, "plate_medium": 0.15, "plate_small": 0.10}
PLATE_START = {"plate_large": (0.275, 0.135), "plate_medium": (0.355, 0.135), "plate_small": (0.20, 0.145)}
# every plate lies wholly on the tray (its left edge is 18 cm left of the midline); the small plate sits 1 cm further
# left than the others so the left arm picking it stays 14.6 cm from the right arm at the pass mark
PLATE_ORDER = ("plate_large", "plate_medium", "plate_small")  # the stack, bottom to top
PLATES = [
    Cylinder(n, pos=(*PLATE_START[n], TRAY_TOP + PLATE_H / 2 + 0.001), radius=PLATE_R[n], height=PLATE_H,
             mass=PLATE_KG[n], color=PLATE_ORANGE)
    for n in PLATE_ORDER
]

OBJECTS = [TRAY, *RACK, *TUB, *GLASSES, *PLATES]
SUCCESS_HOLD_S = 2.5  # cleared and still for 2.5 s: long enough to see the stack and the glasses are stable

GLASS_CREDIT, PLATE_CREDIT = 0.2, 0.2
UPRIGHT_DEG = 10.0  # a racked glass, and a stacked plate, tilted at most 10 degrees
TIPPED_DEG = 45.0  # a glass or plate tilted more than 45 degrees anywhere is knocked over
SEATED = 0.01  # a bottom within 1 cm of the surface it should rest on
STACK_OFFSET = 0.02  # a stacked plate's centre within 2 cm, sideways, of the centre of the plate under it
IN_TOL = 0.005  # a plate's centre may sit this far past the tub's inside (it can lean on a wall)


def _tilt_deg(state) -> float:
    """Angle between the object's up axis and the vertical."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _xy(state):
    return float(state.pos[0]), float(state.pos[1])


def _within(state, lo, hi, tol=0.0) -> bool:
    x, y = _xy(state)
    return bool(lo[0] - tol <= x <= hi[0] + tol and lo[1] - tol <= y <= hi[1] + tol)


def _cell_of(state):
    """The index of the rack cell whose 8 cm inside holds the object's centre, or None."""
    for i, (cx, cy) in enumerate(CELLS):
        if _within(state, (cx - CELL / 2, cy - CELL / 2), (cx + CELL / 2, cy + CELL / 2)):
            return i
    return None


def _discs_overlap(objects, big, small) -> bool:
    """Seen from above, the two plates' round outlines overlap: centres closer than the sum of their radii."""
    (ax, ay), (bx, by) = _xy(objects[big]), _xy(objects[small])
    return math.hypot(ax - bx, ay - by) < PLATE_R[big] + PLATE_R[small]


def _rests_on(objects, big, small) -> bool:
    """The larger plate rests on the smaller one, flat on it or tilted onto it with an edge lower down: their outlines
    overlap seen from above, the larger plate's top is more than 1 cm above the smaller plate's top, and its lowest
    point is at most 1 cm above the smaller plate's top. A plate held higher in the gripper doesn't count."""
    small_top = float(objects[small].high[2])
    return (float(objects[big].low[2]) <= small_top + SEATED and float(objects[big].high[2]) > small_top + SEATED
            and _discs_overlap(objects, big, small))


def _broken(objects) -> bool:
    """What breaks or chips in a dish pit: a glass in the bus tub, a plate in the glass rack, a larger plate resting on
    a smaller one, a glass or plate knocked over (tilted more than 45 degrees), or one fallen off the table (its top
    below the table surface)."""
    for g in GLASS_NAMES:
        if _within(objects[g], (TUB_OUT_X[0], TUB_OUT_Y[0]), (TUB_OUT_X[1], TUB_OUT_Y[1])):
            return True
    for p in PLATE_ORDER:
        if _within(objects[p], RACK_LO, RACK_HI):
            return True
    for i, big in enumerate(PLATE_ORDER):
        for small in PLATE_ORDER[i + 1:]:
            if _rests_on(objects, big, small):
                return True
    for n in (*GLASS_NAMES, *PLATE_ORDER):
        s = objects[n]
        if _tilt_deg(s) > TIPPED_DEG or float(s.high[2]) < TOP:
            return True
    return False


def _glass_racked(objects, name) -> bool:
    """Upright (at most 10 degrees) in a rack cell, standing on the rack's base, with no other glass in that cell."""
    g = objects[name]
    cell = _cell_of(g)
    if cell is None or _tilt_deg(g) > UPRIGHT_DEG or float(g.low[2]) > RACK_FLOOR + SEATED:
        return False
    return all(_cell_of(objects[o]) != cell for o in GLASS_NAMES if o != name)


def _plate_stacked(objects, name, below) -> bool:
    """In the tub, flat (at most 10 degrees), resting on the tub floor (below is None) or on the plate ``below``,
    centred over it within 2 cm."""
    p = objects[name]
    if not (_within(p, (TUB_IN_X[0], TUB_IN_Y[0]), (TUB_IN_X[1], TUB_IN_Y[1]), IN_TOL)
            and float(p.low[2]) >= TUB_FLOOR - 0.005 and _tilt_deg(p) <= UPRIGHT_DEG):
        return False
    if below is None:
        return float(p.low[2]) <= TUB_FLOOR + SEATED
    b = objects[below]
    off = math.hypot(_xy(p)[0] - _xy(b)[0], _xy(p)[1] - _xy(b)[1])
    return abs(float(p.low[2]) - float(b.high[2])) <= SEATED and off <= STACK_OFFSET


def _stack_levels(objects) -> int:
    """How many plates are stacked correctly, counted from the bottom (large, medium, small) up to the first error."""
    below = None
    for k, name in enumerate(PLATE_ORDER):
        if not _plate_stacked(objects, name, below):
            return k
        below = name
    return len(PLATE_ORDER)


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each glass upright in its own rack cell; 0.2 for each plate at its
    right level of the stack in the tub (large on the floor, medium on it, small on top), counted from the bottom up to
    the first error. 0 overall if a glass is in the tub, a plate is in the rack, a larger plate rests on a smaller
    one (outlines overlapping from above, the larger plate's top more than 1 cm above the smaller one's top and its
    lowest point at most 1 cm above it), or a glass or plate is knocked over or has its top below the table surface."""
    if _broken(objects):
        return 0.0
    glasses = sum(_glass_racked(objects, g) for g in GLASS_NAMES)
    return round(GLASS_CREDIT * glasses + PLATE_CREDIT * _stack_levels(objects), 6)


def success(objects):
    """Both glasses racked and the three plates stacked in order, all still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in (*GLASS_NAMES, *PLATE_ORDER))


# The demo. Each arm reaches only its own side: the right arm moves each glass from the tray to the pass mark on the
# midline and the left arm racks it; then the left arm brings the plates to the pass mark, large, medium, small, and
# the right arm stacks them in the tub. Only one arm works at the pass mark at a time. Every grip is straight down
# with the fingers closing along world y.
TRAVEL_Z = TOP + 0.15  # a carried glass's bottom clears the rack walls by 2.5 cm and a racked glass's top by 0.7 cm
GLASS_GRIP = 0.062  # TCP above a glass's bottom: the pads grip 4.9 to 8.2 cm up it, over its centre of mass at 5 cm;
# at the rack release the open fingertips (94 mm apart, wider than a cell) are still 7 mm above the walls
GLASS_RACK_CARRY_S = 2.7  # the left arm carries each glass to the rack slowly, so it doesn't swing in the grip
# (the right arm carries the second glass, which moves 10 cm along x to the pass mark, as slowly)
GLASS_SETTLE_S = 0.6  # the left arm pauses over a rack cell before lowering, so the glass hangs still and centred
PLATE_GRIP = 0.018  # TCP above a plate's bottom: the pads cover all but its lowest 5 mm
DROP = 0.008  # each object is let go this far above where it lands
HOLD_S = 0.4  # a loaded arm waits this long over its pick-up spot while the other arm leaves the pass mark
LEFT_WAIT = (RACK_X, RACK_Y, TRAVEL_Z)  # the left arm waits over the rack for the first glass
TUB_WAIT = (TUB_X, TUB_Y, TRAVEL_Z)  # the right arm waits over the tub for each plate
LEFT_PARK = (0.15, RACK_Y, TRAVEL_Z)  # both arms end in front of their bases, clear of the rack, tray and tub
RIGHT_PARK = (0.15, TUB_Y, TRAVEL_Z)


def _transfer(pick, place, hold_s=0.0):
    """A pick and place; with ``hold_s``, the arm waits with the object lifted before it carries it."""
    wps = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)
    if hold_s:
        wps.insert(4, Waypoint(hold_s, (pick[0], pick[1], TRAVEL_Z), FINGER_CLOSED, "hold until the pass mark is clear"))
    return wps


def _duration(wps) -> float:
    return sum(w.duration for w in wps)


def demo():
    pass_glass_drop = (*PASS_XY, TRAY_TOP + DROP + GLASS_GRIP)
    pass_glass_grip = (*PASS_XY, TRAY_TOP + GLASS_GRIP)
    pass_plate_drop = (*PASS_XY, TRAY_TOP + DROP + PLATE_GRIP)
    pass_plate_grip = (*PASS_XY, TRAY_TOP + PLATE_GRIP)

    right = _transfer((*GLASS_START["glass_1"], TRAY_TOP + GLASS_GRIP), pass_glass_drop)
    wps = _transfer((*GLASS_START["glass_2"], TRAY_TOP + GLASS_GRIP), pass_glass_drop, hold_s=HOLD_S)
    wps[5] = Waypoint(GLASS_RACK_CARRY_S, wps[5].pos, wps[5].finger, wps[5].label, wps[5].yaw)  # the carry
    right += wps

    left = [Waypoint(_duration(right[:8]) + 0.4, LEFT_WAIT, FINGER_OPEN, "wait for the first glass")]
    for cell in CELLS[:2]:  # the far cell first, so the second glass isn't carried over the first
        wps = _transfer(pass_glass_grip, (*cell, RACK_FLOOR + DROP + GLASS_GRIP))
        wps[4] = Waypoint(GLASS_RACK_CARRY_S, wps[4].pos, wps[4].finger, wps[4].label, wps[4].yaw)
        wps.insert(5, Waypoint(GLASS_SETTLE_S, wps[4].pos, FINGER_CLOSED, "settle over the cell", wps[4].yaw))
        left += wps
    plates_from = _duration(left)
    for k, name in enumerate(PLATE_ORDER):
        left += _transfer((*PLATE_START[name], TRAY_TOP + PLATE_GRIP), pass_plate_drop, hold_s=HOLD_S if k else 0.0)
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))

    right.append(Waypoint(1.5, TUB_WAIT, FINGER_OPEN, "to above the tub"))
    # the large plate is on the pass mark when the left arm's first plate move ends (9.6 s after plates_from)
    right.append(Waypoint(plates_from + 9.8 - _duration(right), TUB_WAIT, FINGER_OPEN, "wait for the large plate"))
    for k, name in enumerate(PLATE_ORDER):
        if k:
            right.append(Waypoint(0.4, TUB_WAIT, FINGER_OPEN, "wait for the next plate"))
        right += _transfer(pass_plate_grip, (TUB_X, TUB_Y, TUB_FLOOR + k * PLATE_H + DROP + PLATE_GRIP))
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
