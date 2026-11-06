"""On the station table, 28 cm in front of the arm bases, three fixed singer markers stand in a row across the table:
red, green and blue cylinders 6 cm across and 4 cm tall, red 12 cm left of centre, green on the centre line, blue
12 cm right of centre (12 cm apart, centre to centre). In front of the left arm, a fixed open dark grey mic case
(inside 17 cm front to back, 7 cm side to side, walls 2 cm tall) holds three mic stands in a row: red, green and blue
cylinders 3 cm across, 8 cm tall, weighted to 100 g each, 14, 20 and 26 cm in front of the arm bases, 33 cm left of centre.
A fixed orange handover pad (7 x 7 cm, 1 mm thick) lies on the centre line, 13 cm in front of the arm bases. Done,
each mic stand stands upright on the table within 6 cm of the singer of its own colour, nearer that singer than
either other one, and every two stands are at least 3 times as far apart as the larger of their two distances to
their own singers (the 3:1 rule).
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

COLOURS = ("red", "green", "blue")
RGB = {"red": (0.80, 0.12, 0.10), "green": (0.10, 0.60, 0.20), "blue": (0.12, 0.30, 0.85)}

# The singers: fixed markers in a row across the table, 12 cm apart.
SINGER_R, SINGER_H = 0.03, 0.04
SINGER_X = 0.28
SINGER_Y = {"red": 0.12, "green": 0.0, "blue": -0.12}
SINGERS = [
    Cylinder(f"singer_{c}", pos=(SINGER_X, SINGER_Y[c], TOP + SINGER_H / 2), radius=SINGER_R, height=SINGER_H,
             color=RGB[c])
    for c in COLOURS
]

# The mic case: a 5 mm floor and four 5 mm walls, 2 cm tall, around a 17 x 7 cm inside, in front of the left arm.
CASE_X, CASE_Y = 0.20, 0.33
CASE_INSIDE = (0.17, 0.07)
CASE_WALL_T, CASE_WALL_H, CASE_FLOOR_T = 0.005, 0.02, 0.005
CASE_FLOOR = TOP + CASE_FLOOR_T
CASE_GREY = (0.20, 0.21, 0.23)
_CX = (CASE_X - CASE_INSIDE[0] / 2, CASE_X + CASE_INSIDE[0] / 2)
_CY = (CASE_Y - CASE_INSIDE[1] / 2, CASE_Y + CASE_INSIDE[1] / 2)
_CWZ = TOP + CASE_WALL_H / 2
CASE = [
    Box("case_floor", pos=(CASE_X, CASE_Y, TOP + CASE_FLOOR_T / 2), size=(*CASE_INSIDE, CASE_FLOOR_T),
        color=CASE_GREY),
    Box("case_wall_near", pos=(_CX[0] - CASE_WALL_T / 2, CASE_Y, _CWZ),
        size=(CASE_WALL_T, CASE_INSIDE[1] + 2 * CASE_WALL_T, CASE_WALL_H), color=CASE_GREY),
    Box("case_wall_far", pos=(_CX[1] + CASE_WALL_T / 2, CASE_Y, _CWZ),
        size=(CASE_WALL_T, CASE_INSIDE[1] + 2 * CASE_WALL_T, CASE_WALL_H), color=CASE_GREY),
    Box("case_wall_left", pos=(CASE_X, _CY[1] + CASE_WALL_T / 2, _CWZ), size=(CASE_INSIDE[0], CASE_WALL_T, CASE_WALL_H),
        color=CASE_GREY),
    Box("case_wall_right", pos=(CASE_X, _CY[0] - CASE_WALL_T / 2, _CWZ),
        size=(CASE_INSIDE[0], CASE_WALL_T, CASE_WALL_H), color=CASE_GREY),
]

# The handover pad: a thin orange mat on the centre line, where both arms reach.
PAD_X, PAD_Y = 0.13, 0.0
PAD_SIZE = (0.07, 0.07, 0.001)
PAD = Box("handover_pad", pos=(PAD_X, PAD_Y, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=(0.95, 0.50, 0.05))

# The mic stands, in a row in the case, 6 cm apart.
MIC_R, MIC_H, MIC_KG = 0.015, 0.08, 0.1
MIC_START = {"red": (0.14, CASE_Y), "green": (0.20, CASE_Y), "blue": (0.26, CASE_Y)}
MICS = [
    Cylinder(f"mic_{c}", pos=(*MIC_START[c], CASE_FLOOR + MIC_H / 2 + 0.001), radius=MIC_R, height=MIC_H,
             mass=MIC_KG, color=RGB[c])
    for c in COLOURS
]  # each starts 1 mm above the case floor, and settles

OBJECTS = [*SINGERS, *CASE, PAD, *MICS]
SUCCESS_HOLD_S = 2.5  # all three placed and still for 2.5 s

MIC_MAX_D = 0.060  # a mic's centre at most 6 cm from its own singer's centre
RULE_RATIO = 3.0  # every two mics at least 3 times the larger of their two mic-to-singer distances apart
UPRIGHT_DEG = 10.0  # a stand's axis at most 10 degrees from vertical
ON_TABLE_TOL = 0.003  # standing on the table: the stand's bottom within 3 mm of the table top
LYING_Z = 0.05  # a stand tipped past UPRIGHT_DEG with its lowest point under 5 cm above the table is knocked over
CARRIED_Z = 0.082  # a stand whose bottom is more than 8.2 cm above the table is being carried, not a live mic
# (above the 8 cm top of a standing stand, so a stand stacked on a placed one is still live)
MIC_CREDIT = 0.10  # each mic placed by its own singer
PAIRS = ((("red", "green"), 0.30), (("green", "blue"), 0.30), (("red", "blue"), 0.10))  # both mics of a pair placed


def _tilt_deg(state) -> float:
    """Angle between the stand's axis and the vertical (either end up: the stand-in is a plain cylinder)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _flat_dist(a, b) -> float:
    """Distance between two centres seen from above."""
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _singer_dist(objects, mic: str, singer: str) -> float:
    return _flat_dist(objects[f"mic_{mic}"].pos, objects[f"singer_{singer}"].pos)


def _knocked_over(state) -> bool:
    """Fallen off the table (bottom more than ON_TABLE_TOL below the table top), or tipped past UPRIGHT_DEG with its
    lowest point under LYING_Z above the table top (lying or leaning on the table, the case, a marker or the pad).
    A stand tilted in a gripper higher up than that is not counted."""
    low = float(state.low[2])
    return low < TOP - ON_TABLE_TOL or (_tilt_deg(state) > UPRIGHT_DEG and low < TOP + LYING_Z)


def _over(state, cx: float, cy: float, half_x: float, half_y: float) -> bool:
    """The centre lies over a rectangle seen from above, edges included."""
    return abs(float(state.pos[0]) - cx) <= half_x and abs(float(state.pos[1]) - cy) <= half_y


def _stowed(state) -> bool:
    """Centre over the inside of the case or over the handover pad: not set up yet, so not a live mic."""
    return (_over(state, CASE_X, CASE_Y, CASE_INSIDE[0] / 2, CASE_INSIDE[1] / 2)
            or _over(state, PAD_X, PAD_Y, PAD_SIZE[0] / 2, PAD_SIZE[1] / 2))


def _carried(state) -> bool:
    """Bottom more than CARRIED_Z above the table top: lifted higher than a stand stacked on a standing stand could
    rest, so it is in a gripper on its way somewhere, not a live mic."""
    return float(state.low[2]) > TOP + CARRIED_Z


def _placed(objects, c: str) -> bool:
    """Upright, standing on the table, within MIC_MAX_D of its own singer and nearer it than either other singer."""
    m = objects[f"mic_{c}"]
    d = _singer_dist(objects, c, c)
    return bool(
        _tilt_deg(m) <= UPRIGHT_DEG
        and abs(float(m.low[2]) - TOP) <= ON_TABLE_TOL
        and d <= MIC_MAX_D
        and all(d < _singer_dist(objects, c, o) for o in COLOURS if o != c)
    )


def _breaks_rule(objects, placed: dict) -> bool:
    """The 3:1 rule, against every live stand: some stand that is neither stowed nor carried (placed or not, standing,
    stacked or lying) has its centre closer to a placed mic than RULE_RATIO times that placed mic's distance to its
    own singer. For two placed mics this is: closer together than RULE_RATIO times the larger of their two
    mic-to-singer distances."""
    for p in COLOURS:
        if not placed[p]:
            continue
        need = RULE_RATIO * _singer_dist(objects, p, p)
        for s in COLOURS:
            other = objects[f"mic_{s}"]
            live = not _stowed(other) and not _carried(other)
            if s != p and live and _flat_dist(objects[f"mic_{p}"].pos, other.pos) < need:
                return True
    return False


def score(objects):
    """Partial credit (success scores 1.0): 0.1 for each mic placed by its own singer; for each pair of placed mics,
    0.3 (red-green, green-blue) or 0.1 (red-blue). 0 overall if any live stand (centre not over the case's inside or
    the pad, and bottom at most 8.2 cm above the table top; placed or not) is closer to a placed mic than 3 times
    that mic's distance to its own singer, if any stand has fallen off the table, or if any stand is tipped more than
    10 degrees with its lowest point under 5 cm above the table top. A stand with its bottom more than 8.2 cm up is
    being carried: it earns nothing and zeroes nothing."""
    if any(_knocked_over(objects[f"mic_{c}"]) for c in COLOURS):
        return 0.0
    placed = {c: _placed(objects, c) for c in COLOURS}
    if _breaks_rule(objects, placed):
        return 0.0  # a live mic too close to a placed one comb-filters when mixed: the take is lost
    total = MIC_CREDIT * sum(placed.values())
    total += sum(credit for (a, b), credit in PAIRS if placed[a] and placed[b])
    return round(total, 6)


def success(objects):
    """All three mics placed, every pair keeping the 3:1 rule, the stands still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[f"mic_{c}"].resting() for c in COLOURS)


# The demo's layout: each mic 5 cm from its own singer, red and blue on the outer sides, green in front of its singer.
# Red-green and green-blue are each sqrt(5^2 + 17^2) = 17.7 cm apart against 3 x 5 = 15 cm; red-blue 34 cm.
PLACE_D = 0.05  # 5 mm clear of the singer marker (3 cm marker radius plus 1.5 cm stand radius is 4.5 cm)
RED_PLACE = (SINGER_X, SINGER_Y["red"] + PLACE_D)
GREEN_PLACE = (SINGER_X - PLACE_D, SINGER_Y["green"])
BLUE_PLACE = (SINGER_X, SINGER_Y["blue"] - PLACE_D)

TRAVEL_Z = TOP + 0.15  # a carried stand's bottom 8.5 cm up: clear of the markers, the case and the placed stands
GRIP = 0.065  # TCP above a stand's bottom: the pads cover its top 2.8 cm, the fingertips 5.2 cm up, above the walls
DROP = 0.007  # each stand is let go this far above where it lands
SIDEWAYS = math.pi / 2  # outer stands set down with the fingers closing front to back, away from the singer
LEFT_PARK = (CASE_X, CASE_Y, TRAVEL_Z)  # above the emptied case, clear of the right arm
RIGHT_PARK = (CASE_X, -CASE_Y, TRAVEL_Z)  # the mirror spot in front of the right arm


def demo():
    table_z = TOP + GRIP + DROP
    pad_top = TOP + PAD_SIZE[2]
    left = pick_place_waypoints((*MIC_START["red"], CASE_FLOOR + GRIP), (*RED_PLACE, table_z), travel_z=TRAVEL_Z,
                                place_yaw=SIDEWAYS)
    left += pick_place_waypoints((*MIC_START["green"], CASE_FLOOR + GRIP), (*GREEN_PLACE, table_z),
                                 travel_z=TRAVEL_Z)
    left += pick_place_waypoints((*MIC_START["blue"], CASE_FLOOR + GRIP), (PAD_X, PAD_Y, pad_top + GRIP + DROP),
                                 travel_z=TRAVEL_Z)
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))
    left_s = sum(w.duration for w in left)
    right = [Waypoint(left_s + 0.5, RIGHT_PARK, FINGER_OPEN, "wait for the left arm to park")]
    right += pick_place_waypoints((PAD_X, PAD_Y, pad_top + GRIP), (*BLUE_PLACE, table_z), travel_z=TRAVEL_Z,
                                  place_yaw=SIDEWAYS)
    right.append(Waypoint(2.5, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
