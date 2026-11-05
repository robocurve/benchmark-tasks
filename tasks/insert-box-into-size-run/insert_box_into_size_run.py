"""On the station table, in front of the right arm: a grey shelf channel fixed to the table, running left to right.
Its floor strip is 22 cm long, 5 cm deep and 5 mm thick, with a back stop along its far edge and an end stop at each
end, all rising 3 cm above the strip. The channel's inside runs from 6 cm to 28 cm right of the midline between the arm
bases; its back stop's face is 28.1 cm in front of the arm bases. Three identical cardboard boxes (4 cm front to back,
6 cm left to right, 9 cm tall, 150 g) make up one style's size run, marked by colour: red is small, blue is medium,
green is large. The red box stands in the channel against the left end stop and the back stop; the green box stands
right next to it. The blue box, a return, stands on an orange mat (10 x 10 cm) in front of the left arm. Done, the
row reads red, blue, green from the left end, every box upright, closed up and against the back stop.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The boxes: identical, standing upright, 6 cm side along the row.
BOX_D, BOX_W, BOX_H, BOX_KG = 0.040, 0.060, 0.090, 0.15  # x (front to back), y (along the row), z
RED = (0.80, 0.12, 0.10)
BLUE = (0.12, 0.30, 0.85)
GREEN = (0.10, 0.60, 0.20)

# The channel: a 5 mm floor strip, a 1 cm back stop along its far edge and 1 cm end stops, 3 cm above the strip.
FLOOR_T, STOP_H, STOP_T = 0.005, 0.030, 0.010
FLOOR = TOP + FLOOR_T
X_C = 0.26  # the boxes' centres, front to back, when they stand against the back stop
BACK_X = X_C + BOX_D / 2 + 0.001  # the back stop's face
FRONT_X = BACK_X - 0.050  # the strip's front edge
Y_L, Y_R = -0.060, -0.280  # the left and right end stops' faces
CHANNEL_GREY = (0.30, 0.31, 0.33)
_SZ = TOP + (FLOOR_T + STOP_H) / 2
_MID_X, _MID_Y, _LEN = (FRONT_X + BACK_X) / 2, (Y_L + Y_R) / 2, Y_L - Y_R
CHANNEL = [
    Box("channel_floor", pos=(_MID_X, _MID_Y, TOP + FLOOR_T / 2), size=(BACK_X - FRONT_X, _LEN, FLOOR_T),
        color=CHANNEL_GREY),
    Box("back_stop", pos=(BACK_X + STOP_T / 2, _MID_Y, _SZ), size=(STOP_T, _LEN + 2 * STOP_T, FLOOR_T + STOP_H),
        color=CHANNEL_GREY),
    Box("left_end_stop", pos=(_MID_X, Y_L + STOP_T / 2, _SZ), size=(BACK_X - FRONT_X, STOP_T, FLOOR_T + STOP_H),
        color=CHANNEL_GREY),
    Box("right_end_stop", pos=(_MID_X, Y_R - STOP_T / 2, _SZ), size=(BACK_X - FRONT_X, STOP_T, FLOOR_T + STOP_H),
        color=CHANNEL_GREY),
]

# The return mat in front of the left arm, out of the right arm's reach.
MAT_T = 0.003
MAT_TOP = TOP + MAT_T
BLUE_START = (0.26, 0.25)
MAT = Box("return_mat", pos=(*BLUE_START, TOP + MAT_T / 2), size=(0.10, 0.10, MAT_T), color=(0.95, 0.50, 0.08))

RED_Y = Y_L - 0.001 - BOX_W / 2  # 1 mm from the left end stop
GREEN_START_Y = RED_Y - BOX_W - 0.001  # 1 mm from the red box
_BOX_Z = FLOOR + BOX_H / 2 + 0.001  # each starts 1 mm above what it stands on, and settles
SMALL = Box("red_box", pos=(X_C, RED_Y, _BOX_Z), size=(BOX_D, BOX_W, BOX_H), mass=BOX_KG, color=RED)
LARGE = Box("green_box", pos=(X_C, GREEN_START_Y, _BOX_Z), size=(BOX_D, BOX_W, BOX_H), mass=BOX_KG, color=GREEN)
MEDIUM = Box("blue_box", pos=(*BLUE_START, MAT_TOP + BOX_H / 2 + 0.001), size=(BOX_D, BOX_W, BOX_H), mass=BOX_KG,
             color=BLUE)

OBJECTS = [*CHANNEL, MAT, SMALL, MEDIUM, LARGE]
BOX_NAMES = ("red_box", "blue_box", "green_box")  # the size run, left to right
SUCCESS_HOLD_S = 2.5  # the finished row still for 2.5 s

IN_TOL = 0.005  # a box's centre may sit this far past the strip's outline
FLOOR_TOL = 0.005  # standing on the strip: its bottom within 5 mm of the strip's top
OFF_TABLE = 0.01  # a bottom more than 1 cm below the table top: off the table
UPRIGHT_DEG = 10.0  # tilted more than this: knocked over
GAP_MAX = 0.005  # closed up: every gap in the row, and to the left end stop, at most 5 mm
BACK_TOL = 0.005  # against the back stop: both bottom corners of the back face at most 5 mm from it
SLOT_MIN = BOX_W  # a slot the blue box fits: red and green at least its width apart
SLOT_CREDIT, ORDER_CREDIT, CLOSED_CREDIT, BACK_CREDIT = 0.1, 0.5, 0.2, 0.2


def _tilt_deg(state) -> float:
    """Angle between the box's up axis and the vertical."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _over_channel(state) -> bool:
    """Centre over the channel's floor strip (or just past its outline), at any height."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(FRONT_X - IN_TOL <= x <= BACK_X + IN_TOL and Y_R - IN_TOL <= y <= Y_L + IN_TOL)


def _in_channel(state) -> bool:
    """Over the strip and standing on it, not on another box or held above it."""
    return _over_channel(state) and abs(float(state.low[2]) - FLOOR) <= FLOOR_TOL


def _knocked(state) -> bool:
    """Tilted past UPRIGHT_DEG, or off the table."""
    return _tilt_deg(state) > UPRIGHT_DEG or float(state.low[2]) < TOP - OFF_TABLE


def _closed(red, blue, green) -> bool:
    """Red within GAP_MAX of the left end stop, and both gaps between neighbours GAP_MAX or less."""
    gaps = (Y_L - float(red.high[1]), float(red.low[1]) - float(blue.high[1]), float(blue.low[1]) - float(green.high[1]))
    return all(g <= GAP_MAX for g in gaps)


def _rotate(quat_xyzw, v):
    """Rotate the vector v by the unit quaternion (x, y, z, w)."""
    x, y, z, w = (float(c) for c in quat_xyzw)
    vx, vy, vz = v
    # v' = v + 2w (q x v) + 2 q x (q x v), with q the vector part
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ddx, ddy, ddz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ddx), vy + 2 * (w * cy + ddy), vz + 2 * (w * cz + ddz))


def _against_back(state) -> bool:
    """Both bottom corners of the box's back face within BACK_TOL of the back stop's face. The back face is the 6 cm x
    9 cm face (one of the box's two faces normal to its 4 cm depth) turned more towards the stop; its bottom corners
    are found from the box's pose, so a box turned about the vertical, or leaning, is measured where it really is."""
    px = float(state.pos[0])
    side = 1.0 if _rotate(state.quat_xyzw, (1.0, 0.0, 0.0))[0] >= 0 else -1.0
    gaps = []
    for along in (BOX_W / 2, -BOX_W / 2):
        cx = px + _rotate(state.quat_xyzw, (side * BOX_D / 2, along, -BOX_H / 2))[0]
        gaps.append(BACK_X - cx)
    return all(g <= BACK_TOL for g in gaps)


def score(objects):
    """Partial credit (success scores 1.0): 0.1 for a slot between red and green at least the blue box's width (or
    the blue box in it); 0.5 for the blue box standing in the channel between red and green; with it, 0.2 for the row
    closed up (every gap 5 mm or less) and 0.2 for both bottom corners of each box's back face within 5 mm of the back
    stop. 0 overall if a box is knocked over or off the table, red or green leaves the channel, green is left of red, or the blue box is over the
    channel anywhere but standing between red and green."""
    red, blue, green = (objects[n] for n in BOX_NAMES)
    if any(_knocked(b) for b in (red, blue, green)):
        return 0.0
    if not (_in_channel(red) and _in_channel(green)) or float(green.pos[1]) >= float(red.pos[1]):
        return 0.0
    blue_in = _in_channel(blue) and float(green.pos[1]) < float(blue.pos[1]) < float(red.pos[1])
    if _over_channel(blue) and not blue_in:
        return 0.0  # parked at the end of the run, or on top of the row
    credit = 0.0
    if blue_in or float(red.low[1]) - float(green.high[1]) >= SLOT_MIN:
        credit += SLOT_CREDIT
    if blue_in:
        credit += ORDER_CREDIT
        if _closed(red, blue, green):
            credit += CLOSED_CREDIT
        if all(_against_back(b) for b in (red, blue, green)):
            credit += BACK_CREDIT
    return round(credit, 6)


def success(objects):
    """The row red, blue, green, closed up and against the back stop, all three still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in BOX_NAMES)


# The demo. The right arm lifts the green box 6.4 cm to the right, opening a 6.5 cm slot. Meanwhile the left arm
# carries the blue box from the mat to the table on the midline, where both arms reach. The right arm then lowers it
# into the slot. The right arm grips across the boxes' 4 cm depth (yaw 90 degrees), so its fingers stand in front of
# and behind a box, above the back stop, never between neighbours; the left arm grips across the 6 cm width.
GRIP = 0.057  # TCP above a box's bottom: fingertips 4.4 cm up, 1.4 cm above the stops; the palm 1.7 cm over the top
TRAVEL_Z = TOP + 0.165  # a carried box's bottom clears the standing boxes' tops by 1.3 cm
DROP = 0.005  # each box is let go this far above where it lands
ROW_YAW = math.pi / 2  # fingers close front to back
# A box lowered straight down 1 mm from the back stop catches the stop's top edge on any small error and tips
# forward when let go. So the right arm lowers each box 8 mm in front of its goal, with its bottom DROP above the
# strip (below the stop's top), then slides it back against the stop and lets go there.
# The slide aims the back face 1.5 mm past the stop's face, so the stop, not the arm's few millimetres of tracking
# error on a short move, sets where the box ends: it is pressed flat against the stop when let go.
SLIDE = 0.010
PLACE_X = X_C + 0.0025
DEMO_GAP = 0.0025  # the gap the demo leaves on each side of the blue box
BLUE_GOAL_Y = RED_Y - BOX_W - DEMO_GAP
GREEN_GOAL_Y = BLUE_GOAL_Y - BOX_W - DEMO_GAP
HANDOVER = (0.20, 0.0)  # on the midline, where both arms reach
LEFT_PARK = (0.15, 0.40, TRAVEL_Z)
RIGHT_PARK = (0.15, -0.40, TRAVEL_Z)
RIGHT_WAIT_S = 1.0  # after moving the green box: the left arm has let go of the blue one and is leaving


def _row_place(pick, y):
    """Right arm: pick a box at ``pick``, lower it in front of its slot at ``y``, slide it back against the back stop,
    let go, wait with the fingers open, then rise straight up."""
    z = FLOOR + GRIP + DROP
    wps = pick_place_waypoints(pick, (PLACE_X - SLIDE, y, z), travel_z=TRAVEL_Z, yaw=ROW_YAW)
    i = next(k for k, w in enumerate(wps) if w.label == "release")
    return wps[:i] + [
        Waypoint(0.8, (PLACE_X, y, z), FINGER_CLOSED, "slide to the back stop", ROW_YAW),
        Waypoint(0.6, (PLACE_X, y, z), FINGER_OPEN, "release", ROW_YAW),
        Waypoint(0.4, (PLACE_X, y, z), FINGER_OPEN, "let it settle", ROW_YAW),
        Waypoint(1.0, (PLACE_X, y, TRAVEL_Z), FINGER_OPEN, "retreat", ROW_YAW),
    ]


def demo():
    left = pick_place_waypoints(
        (*BLUE_START, MAT_TOP + 0.001 + GRIP), (*HANDOVER, TOP + GRIP + DROP), travel_z=TRAVEL_Z,
    )
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))
    right = _row_place((X_C, GREEN_START_Y, FLOOR + 0.001 + GRIP), GREEN_GOAL_Y)
    right.append(Waypoint(RIGHT_WAIT_S, (PLACE_X, GREEN_GOAL_Y, TRAVEL_Z), FINGER_OPEN, "wait for the handover",
                          ROW_YAW))
    right += _row_place((*HANDOVER, TOP + GRIP), BLUE_GOAL_Y)
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park", 0.0))
    return {"left": left, "right": right}
