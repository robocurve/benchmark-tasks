"""On the station table, in front of the left arm: an open dark grey cart tray (inside 26 cm front to back, 28 cm side
to side, walls 3 cm tall) holding five artifacts. They are a blue vase (cylinder 6 cm across, 12 cm tall), a green
figurine (box 5 x 5 cm, 10 cm tall), a yellow bowl (cylinder 7 cm across, 6 cm tall), an orange pot (cylinder 5 cm
across, 5 cm tall) and a purple bead box (4 cm cube). In front of the right arm: a brown wooden storage shelf, a board
2 cm thick, 26 cm wide and 24 cm deep, with a 2 cm back lip; its front edge (the aisle side, nearest the arms) is
20 cm in front of the arm bases. Between the arms: a black transfer pad (12 x 12 cm, 5 mm thick), 22 cm in front of
the arm bases. The tray, shelf and pad are fixed; the five artifacts are loose. Shelved, every artifact stands
upright on the shelf at least 2 cm back from the front edge and at least 2 cm from every other artifact. No artifact
stands in front of a shorter one in the same lane, so every artifact can be seen from the aisle and none has to be
lifted over another.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The shelf: a 2 cm board on the table, its front edge (aisle side) at x 0.20, with a 2 cm lip along its back.
SHELF_T = 0.02
SHELF_TOP = TOP + SHELF_T
FRONT_EDGE = 0.20
SHELF_DEPTH = 0.24
SHELF_Y = (-0.43, -0.17)  # 26 cm wide: the five footprints alone add up to 27 cm, so one row can't hold them
LIP_T, LIP_H = 0.02, 0.02
SHELF_X = (FRONT_EDGE, FRONT_EDGE + SHELF_DEPTH - LIP_T)  # the usable top, from the front edge to the lip's face
WOOD = (0.55, 0.33, 0.16)
_SHELF_CY = (SHELF_Y[0] + SHELF_Y[1]) / 2
_SHELF_W = SHELF_Y[1] - SHELF_Y[0]
SHELF = [
    Box("shelf_board", pos=(FRONT_EDGE + SHELF_DEPTH / 2, _SHELF_CY, TOP + SHELF_T / 2),
        size=(SHELF_DEPTH, _SHELF_W, SHELF_T), color=WOOD),
    Box("shelf_lip", pos=(SHELF_X[1] + LIP_T / 2, _SHELF_CY, SHELF_TOP + LIP_H / 2),
        size=(LIP_T, _SHELF_W, LIP_H), color=WOOD),
]

# The cart tray: a 5 mm floor and four 5 mm walls, 3 cm tall, around a 26 x 28 cm inside, in front of the left arm.
CART_IN_X, CART_IN_Y = (0.14, 0.40), (0.17, 0.45)
CART_FLOOR_T, CART_WALL_T, CART_WALL_H = 0.005, 0.005, 0.03
CART_FLOOR = TOP + CART_FLOOR_T
CART_GREY = (0.22, 0.23, 0.25)
_CX, _CY = (CART_IN_X[0] + CART_IN_X[1]) / 2, (CART_IN_Y[0] + CART_IN_Y[1]) / 2
_CW, _CD = CART_IN_X[1] - CART_IN_X[0], CART_IN_Y[1] - CART_IN_Y[0]
_CWZ = TOP + CART_WALL_H / 2
CART = [
    Box("cart_floor", pos=(_CX, _CY, TOP + CART_FLOOR_T / 2), size=(_CW, _CD, CART_FLOOR_T), color=CART_GREY),
    Box("cart_wall_near", pos=(CART_IN_X[0] - CART_WALL_T / 2, _CY, _CWZ),
        size=(CART_WALL_T, _CD + 2 * CART_WALL_T, CART_WALL_H), color=CART_GREY),
    Box("cart_wall_far", pos=(CART_IN_X[1] + CART_WALL_T / 2, _CY, _CWZ),
        size=(CART_WALL_T, _CD + 2 * CART_WALL_T, CART_WALL_H), color=CART_GREY),
    Box("cart_wall_left", pos=(_CX, CART_IN_Y[1] + CART_WALL_T / 2, _CWZ), size=(_CW, CART_WALL_T, CART_WALL_H),
        color=CART_GREY),
    Box("cart_wall_right", pos=(_CX, CART_IN_Y[0] - CART_WALL_T / 2, _CWZ), size=(_CW, CART_WALL_T, CART_WALL_H),
        color=CART_GREY),
]

# The transfer pad on the midline, where the left arm sets each artifact down for the right arm.
PAD_X, PAD_Y, PAD_SIZE, PAD_T = 0.22, 0.0, 0.12, 0.005
PAD_TOP = TOP + PAD_T
PAD = Box("transfer_pad", pos=(PAD_X, PAD_Y, TOP + PAD_T / 2), size=(PAD_SIZE, PAD_SIZE, PAD_T),
          color=(0.08, 0.08, 0.09))

# The artifacts: standing height H, width across W (diameter or side), all upright to start.
VASE_R, VASE_H = 0.030, 0.12
FIG_W, FIG_H = 0.05, 0.10
BOWL_R, BOWL_H = 0.035, 0.06
POT_R, POT_H = 0.025, 0.05
BEAD_W, BEAD_H = 0.04, 0.04
H = {"vase": VASE_H, "figurine": FIG_H, "bowl": BOWL_H, "pot": POT_H, "bead_box": BEAD_H}
W = {"vase": 2 * VASE_R, "figurine": FIG_W, "bowl": 2 * BOWL_R, "pot": 2 * POT_R, "bead_box": BEAD_W}
CART_START = {"vase": (0.20, 0.25), "figurine": (0.31, 0.25), "bowl": (0.185, 0.37), "pot": (0.28, 0.37),
              "bead_box": (0.36, 0.37)}
ARTIFACTS = [
    Cylinder("vase", pos=(*CART_START["vase"], CART_FLOOR + VASE_H / 2 + 0.001), radius=VASE_R, height=VASE_H,
             mass=0.30, color=(0.12, 0.30, 0.85)),
    Box("figurine", pos=(*CART_START["figurine"], CART_FLOOR + FIG_H / 2 + 0.001), size=(FIG_W, FIG_W, FIG_H),
        mass=0.25, color=(0.10, 0.62, 0.22)),
    Cylinder("bowl", pos=(*CART_START["bowl"], CART_FLOOR + BOWL_H / 2 + 0.001), radius=BOWL_R, height=BOWL_H,
             mass=0.20, color=(0.95, 0.78, 0.05)),
    Cylinder("pot", pos=(*CART_START["pot"], CART_FLOOR + POT_H / 2 + 0.001), radius=POT_R, height=POT_H,
             mass=0.15, color=(0.95, 0.42, 0.05)),
    Box("bead_box", pos=(*CART_START["bead_box"], CART_FLOOR + BEAD_H / 2 + 0.001), size=(BEAD_W, BEAD_W, BEAD_H),
        mass=0.08, color=(0.55, 0.15, 0.75)),
]  # each starts 1 mm above the cart floor, and settles
NAMES = tuple(H)

OBJECTS = [*SHELF, *CART, PAD, *ARTIFACTS]
SUCCESS_HOLD_S = 2.5  # shelved and still for 2.5 s

UPRIGHT_DEG = 15.0  # shelved: axis tilted at most 15 degrees from vertical
REST_TOL = 0.003  # set down: bottom within 3 mm of the shelf top (or, stacked, of the top of the artifact below)
SETBACK = 0.02  # footprint at least 2 cm behind the front edge
GAP_MIN = 0.02  # two shelved artifacts at least 2 cm apart
TALLER_BY = 0.005  # a screen is at least 5 mm taller (standing heights; any two artifacts differ by 1 cm or more), ...
# ... its back face nearer the front edge than the hidden artifact's centre (side by side, staggered by less than
# that, doesn't count) ...
LANE_MARGIN = 0.02  # ... and centres less than half the two widths plus 2 cm apart along the edge
SHELVED_CREDIT, EDGE_CREDIT, GAP_PENALTY = 0.2, 0.1, 0.05


def _tilt_deg(state) -> float:
    """Angle between the object's up axis and the vertical."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _over_shelf(state) -> bool:
    """Centre over the usable top."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(SHELF_X[0] <= x <= SHELF_X[1] and SHELF_Y[0] <= y <= SHELF_Y[1])


def _on_top_of(top, below) -> bool:
    """`top` rests on `below`: its bottom within 3 mm of below's top, its centre over below's footprint."""
    x, y, _ = (float(v) for v in top.pos)
    return bool(abs(float(top.low[2]) - float(below.high[2])) <= REST_TOL
                and float(below.low[0]) <= x <= float(below.high[0])
                and float(below.low[1]) <= y <= float(below.high[1]))


def _on_shelf(objects, name) -> bool:
    """Set down on the shelf, standing or lying (bottom within 3 mm of the shelf top), or stacked on an artifact
    whose centre is over the usable top. An artifact still held higher above the shelf is not on it."""
    s = objects[name]
    if not _over_shelf(s):
        return False
    if abs(float(s.low[2]) - SHELF_TOP) <= REST_TOL:
        return True
    return any(_over_shelf(objects[o]) and _on_top_of(s, objects[o]) for o in NAMES if o != name)


def _shelved(state) -> bool:
    """Upright, set down on the shelf top (not on another object), its whole footprint on the usable top."""
    lo, hi = state.low, state.high
    return bool(
        _tilt_deg(state) <= UPRIGHT_DEG
        and abs(float(lo[2]) - SHELF_TOP) <= REST_TOL
        and float(lo[0]) >= SHELF_X[0] and float(hi[0]) <= SHELF_X[1]
        and float(lo[1]) >= SHELF_Y[0] and float(hi[1]) <= SHELF_Y[1]
    )


def _set_back(state) -> bool:
    return float(state.low[0]) >= FRONT_EDGE + SETBACK


def _gap(a, b) -> float:
    """Horizontal gap between two footprints, each taken as its outline squared to the shelf edges."""
    gx = max(0.0, float(a.low[0] - b.high[0]), float(b.low[0] - a.high[0]))
    gy = max(0.0, float(a.low[1] - b.high[1]), float(b.low[1] - a.high[1]))
    return math.hypot(gx, gy)


def _screens(objects, front, back) -> bool:
    """`front` hides `back`: taller standing (by 5 mm or more), its back face nearer the front edge than back's
    centre (so more than half of `back` stands behind it), and in the same lane."""
    f, b = objects[front], objects[back]
    return bool(
        H[front] >= H[back] + TALLER_BY - 1e-9
        and float(f.high[0]) < float(b.pos[0])
        and abs(float(f.pos[1]) - float(b.pos[1])) < (W[front] + W[back]) / 2 + LANE_MARGIN
    )


def _any_hidden(objects) -> bool:
    """Any artifact on the shelf hidden behind another on the shelf (a fallen one counts at its standing height)."""
    on = [n for n in NAMES if _on_shelf(objects, n)]
    return any(_screens(objects, f, b) for f in on for b in on if f != b)


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each artifact shelved at least 2 cm back from the front edge, or
    0.1 if shelved but nearer the edge; minus 0.05 for each pair of shelved artifacts less than 2 cm apart, floored at
    0. 0 overall if any artifact on the shelf stands behind a taller one in its lane."""
    if _any_hidden(objects):
        return 0.0
    shelved = [n for n in NAMES if _shelved(objects[n])]
    credit = sum(SHELVED_CREDIT if _set_back(objects[n]) else EDGE_CREDIT for n in shelved)
    close = 0
    for i, a in enumerate(shelved):
        for b in shelved[i + 1:]:
            close += _gap(objects[a], objects[b]) < GAP_MIN
    return round(max(0.0, credit - GAP_PENALTY * close), 6)


def success(objects):
    """All five shelved, set back, spaced and none hidden; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in NAMES)


# The demo: the left arm carries each artifact from the cart to the pad; the right arm takes it from the pad to the
# shelf. Back row first (figurine, then vase, centres at x 0.375), then the front row (bowl, pot, bead box at x 0.265),
# so the right arm never carries anything over a standing artifact. The left arm and the right arm's back-row grips
# close the fingers along world y; the front-row grips close them front to back (yaw 90 degrees), so the fingers
# fit between neighbours along the row.
TRAVEL_Z = TOP + 0.165  # fingertips 1.2 cm above the vase's top on the shelf; the vase's bottom 4.5 cm above the walls
DROP = 0.008  # each artifact is let go this far above where it lands
GRIP = {"vase": 0.09, "figurine": 0.07, "bowl": 0.035, "pot": 0.03, "bead_box": 0.025}  # TCP above the bottom
SHELF_SPOT = {"figurine": (0.375, -0.33), "vase": (0.375, -0.23), "bowl": (0.265, -0.22), "pot": (0.265, -0.31),
              "bead_box": (0.265, -0.39)}
PLACE_YAW = {"figurine": 0.0, "vase": 0.0, "bowl": math.pi / 2, "pot": math.pi / 2, "bead_box": math.pi / 2}
ORDER = ("figurine", "vase", "bowl", "pot", "bead_box")
RIGHT_LAG_S = 0.3  # the right arm sets off for the pad this long after the left arm has let go and risen
PAD_CLEAR_S = 0.5  # the left arm carries the next artifact to the pad this long after the right arm has lifted
RIGHT_WAIT_XY = (0.22, -0.18)
LEFT_PARK = (0.14, 0.12, TRAVEL_Z)
RIGHT_PARK = (0.14, -0.12, TRAVEL_Z)
RIGHT_PARK_S = 3.0


def _duration(wps) -> float:
    return sum(w.duration for w in wps)


def demo():
    left, right, released = [], [], []
    for k, name in enumerate(ORDER):
        pick = (*CART_START[name], CART_FLOOR + GRIP[name])
        drop = (PAD_X, PAD_Y, PAD_TOP + GRIP[name] + DROP)
        wps = pick_place_waypoints(pick, drop, travel_z=TRAVEL_Z)
        fetch, deliver = wps[:4], wps[4:]
        left += fetch
        if k > 0:  # hold the artifact above the cart until the right arm has lifted the last one off the pad
            ready = released[-1] + RIGHT_LAG_S + 5.0 + PAD_CLEAR_S
            wait = ready - _duration(left)
            if wait > 0:
                left.append(Waypoint(wait, fetch[-1].pos, FINGER_CLOSED, "wait for the pad to clear"))
        left += deliver
        released.append(_duration(left))
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))

    for k, name in enumerate(ORDER):
        start = released[k] + RIGHT_LAG_S
        wait = start - _duration(right)
        here = right[-1].pos if right else (*RIGHT_WAIT_XY, TRAVEL_Z)
        yaw = right[-1].yaw if right else 0.0
        if wait > 0:
            right.append(Waypoint(wait, here, FINGER_OPEN, "wait for the next artifact", yaw))
        right += pick_place_waypoints(
            (PAD_X, PAD_Y, PAD_TOP + GRIP[name]), (*SHELF_SPOT[name], SHELF_TOP + GRIP[name] + DROP),
            travel_z=TRAVEL_Z, yaw=PLACE_YAW[name],
        )
    right.append(Waypoint(RIGHT_PARK_S, RIGHT_PARK, FINGER_OPEN, "park", 0.0))
    return {"left": left, "right": right}
