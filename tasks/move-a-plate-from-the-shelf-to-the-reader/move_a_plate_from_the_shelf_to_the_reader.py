"""On the station table: a low wooden shelf in front of the left arm, and a flat grey plate reader beside it, with a
top opening and a door that runs front to back. The door starts parked under a hood at the reader's back, and it is
pulled towards the arms to shut it.

A sample plate with a lid sits on the shelf. One corner of the plate is orange, and the reader has an orange corner
where that corner belongs. As it sits on the shelf the plate is the wrong way round for the reader, so a straight move
puts it in backwards: it has to be turned half a turn on the way. The lid comes off and goes on the table, because
this reader reads open plates. The door has a small window over the orange corner.

Run it:  inspect-robots run --embodiment yam_arena -E task=move_a_plate_from_the_shelf_to_the_reader.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, on_reset, on_step, tool  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Take the plate from the shelf. Take off its lid and put the lid on the table. Set the plate in the reader with "
    "its orange corner at the reader's orange corner, then shut the door."
)

WOOD = (0.62, 0.47, 0.30)
WOOD_DARK = (0.46, 0.34, 0.21)
CASE = (0.70, 0.72, 0.74)
CASE_DARK = (0.34, 0.36, 0.39)
BAY = (0.22, 0.24, 0.27)
WALL_PALE = (0.80, 0.82, 0.84)
SCREEN = (0.10, 0.14, 0.18)
WHITE = (0.93, 0.93, 0.95)
LID_CLEAR = (0.86, 0.91, 0.94)
WELLS = (0.74, 0.78, 0.86)
ORANGE = (0.96, 0.48, 0.04)
DARK = (0.25, 0.27, 0.30)

PROUD = 0.0002  # decals stand just off the surface, so they don't flicker against it
REST_GAP = 0.0005

# -- the plate and its lid: the standard footprint at the skirt, long side along y, so the fingers close along x -------
PLATE = (0.0855, 0.1276, 0.020)  # the footprint: too tight to grip, at 4.25 mm each side of the 94 mm opening
SKIRT_H = 0.005  # the skirt runs round the bottom at the full footprint. Pad tips stop above it
GRIP_W = 0.070  # the body above the skirt: 12 mm of clearance each side of the opening
LID = (GRIP_W, PLATE[1], 0.008)  # on the body, so a grip on the sides still has 12 mm with the lid on
TAB = (0.050, 0.020, 0.030)
MARK = 0.040  # the orange corner, seen from above: on the body's top and the skirt beside it
MARK_XY = (-(PLATE[0] - MARK) / 2, -(PLATE[1] - MARK) / 2)  # nearest the arms and to the right, on the shelf
MARK_SKIRT_W = (PLATE[0] - GRIP_W) / 2
MARK_TOP_W = MARK - MARK_SKIRT_W

# -- the shelf: a board on two legs with a back panel. Only the panel stands taller than the board ----------------------
SHELF_XY = (0.315, 0.31)
SHELF = (0.19, 0.22, 0.015)  # deep, so the back panel clears the open fingers and the wrist
SHELF_TOP = TOP + 0.060
BACK_PANEL = (0.012, 0.22, 0.160)
LEG_Y = 0.015
PLATE_XY = (0.30, 0.31)
PLATE_Z = SHELF_TOP + REST_GAP + PLATE[2] / 2
PLATE_TOP = PLATE_Z + PLATE[2] / 2
LID_Z = PLATE_TOP + REST_GAP + LID[2] / 2
LID_TOP = LID_Z + LID[2] / 2
LID_SET_XY = (0.12, 0.13)  # where the demo puts the lid: anywhere on the table or shelf counts


def _shelf() -> list[Box]:
    x, y = SHELF_XY
    deep, wide, thick = SHELF
    leg_h = SHELF_TOP - thick - TOP
    panel_x = x + deep / 2 + BACK_PANEL[0] / 2
    return [
        Box("shelf_board", pos=(x, y, SHELF_TOP - thick / 2), size=SHELF, color=WOOD),
        Box("shelf_leg_r", pos=(x, y - wide / 2 + LEG_Y / 2, TOP + leg_h / 2), size=(deep, LEG_Y, leg_h),
            color=WOOD_DARK),
        Box("shelf_leg_l", pos=(x, y + wide / 2 - LEG_Y / 2, TOP + leg_h / 2), size=(deep, LEG_Y, leg_h),
            color=WOOD_DARK),
        Box("shelf_back", pos=(panel_x, y, TOP + BACK_PANEL[2] / 2), size=BACK_PANEL, color=WOOD_DARK),
    ]


SAMPLE_PLATE = tool(
    "plate",
    pos=(*PLATE_XY, PLATE_Z),
    boxes=[
        ((0.0, 0.0, (SKIRT_H - PLATE[2]) / 2), (PLATE[0], PLATE[1], SKIRT_H), WHITE),
        ((0.0, 0.0, SKIRT_H / 2), (GRIP_W, PLATE[1], PLATE[2] - SKIRT_H), WHITE),
        ((0.0, 0.0, PLATE[2] / 2 + PROUD - 0.0005), (GRIP_W - 0.012, PLATE[1] - 0.012, 0.001), WELLS),
        ((-(GRIP_W - MARK_TOP_W) / 2, MARK_XY[1], PLATE[2] / 2 + 2 * PROUD - 0.0005), (MARK_TOP_W, MARK, 0.001), ORANGE),
        ((-(GRIP_W + MARK_SKIRT_W) / 2, MARK_XY[1], SKIRT_H - PLATE[2] / 2 + PROUD - 0.0005),
         (MARK_SKIRT_W, MARK, 0.001), ORANGE),
    ],
    mass=0.25,
    points=(("orange_corner", (-PLATE[0] / 2, -PLATE[1] / 2, 0.0)),),
)

SAMPLE_LID = tool(
    "lid",
    pos=(*PLATE_XY, LID_Z),
    boxes=[
        ((0.0, 0.0, 0.0), LID, LID_CLEAR),
        ((0.0, 0.0, LID[2] / 2 + TAB[2] / 2), TAB, DARK),
    ],
    mass=0.05,
)

# -- the reader, in its own frame: origin at the middle of the opening. x runs along the door's travel ------------------
READER_XY = (0.275, 0.09)
BODY_H = 0.034  # low, so the door sits close over the plate and the window looks straight at its corner
FLOOR_T = 0.006
OPENING = (0.170, 0.180)  # wide enough for open fingers either side of the plate, with 2.8 cm to spare
RIM = 0.012
BODY_W = OPENING[0] + 2 * RIM  # the opening and its rims, along x
LEFT_END = OPENING[1] / 2 + RIM
NEST = (0.098, 0.140)  # inside the low walls: 6 mm round the plate. The pad tips stay above them
NEST_WALL = 0.003
NEST_H = 0.005
CORNER_XY = (NEST[0] / 2, NEST[1] / 2)  # the reader's orange corner: back and left, as seen from the arms
CORNER_MARK = 0.036
CORNER_MARK_XY = (0.064, 0.072)  # round the corner, beside a seated plate, and out of sight through the door's window

DOOR_W = BODY_W
DOOR_L = 2 * LEFT_END
DOOR_T = 0.006
DOOR_GAP = 0.001
DOOR_TRAVEL = 0.188
DOOR_Z = BODY_H / 2 + DOOR_GAP + DOOR_T / 2
DOOR_SHUT_WITHIN = 0.010
FRONT_END = -BODY_W / 2
REAR_END = 0.290  # a deck for the open door to sit on
HOOD_X = 0.175  # the hood over the door's parked end starts here, leaving room for fingers behind the handle
HOOD_H = 0.080  # its top above the table
HANDLE = (0.12, 0.015, 0.030)  # width along y, thickness along x, height
WINDOW = (0.0030, 0.0330, 0.0265, 0.0565)  # x0, x1, y0, y1: seen through it, over the corner of a plate loaded right

FLOOR_Z = TOP + FLOOR_T  # the top of the reader's floor


def _reader_boxes() -> list:
    z_mid = -BODY_H / 2 + FLOOR_T / 2
    long_len = REAR_END - FRONT_END
    long_x = (REAR_END + FRONT_END) / 2
    side_y = (OPENING[1] + RIM) / 2
    deck_len = REAR_END - OPENING[0] / 2
    hood_low = BODY_H / 2 + DOOR_GAP + DOOR_T + DOOR_GAP  # the door runs under it
    hood_high = HOOD_H - BODY_H / 2
    hood_len = REAR_END - HOOD_X
    hood_x = (REAR_END + HOOD_X) / 2
    (nx, ny), wall, high = NEST, NEST_WALL, -BODY_H / 2 + FLOOR_T + NEST_H / 2
    across = nx + 2 * wall
    return [
        ((long_x, 0.0, z_mid), (long_len, 2 * LEFT_END, FLOOR_T), BAY),
        ((long_x, -side_y, 0.0), (long_len, RIM, BODY_H), CASE),
        ((long_x, side_y, 0.0), (long_len, RIM, BODY_H), CASE),
        ((-(OPENING[0] + RIM) / 2, 0.0, 0.0), (RIM, OPENING[1], BODY_H), CASE),
        ((OPENING[0] / 2 + deck_len / 2, 0.0, 0.0), (deck_len, OPENING[1], BODY_H), CASE),
        ((hood_x, 0.0, (hood_low + hood_high) / 2), (hood_len, 2 * LEFT_END, hood_high - hood_low), CASE),
        ((hood_x, 0.0, hood_high + PROUD - 0.0005), (hood_len - 0.024, 2 * LEFT_END - 0.04, 0.001), SCREEN),
        ((*CORNER_MARK_XY, -BODY_H / 2 + FLOOR_T + PROUD - 0.0005), (CORNER_MARK, CORNER_MARK, 0.001), ORANGE),
        ((nx / 2 + wall / 2, 0.0, high), (wall, ny, NEST_H), WALL_PALE),
        ((-nx / 2 - wall / 2, 0.0, high), (wall, ny, NEST_H), WALL_PALE),
        ((0.0, ny / 2 + wall / 2, high), (across, wall, NEST_H), WALL_PALE),
        ((0.0, -ny / 2 - wall / 2, high), (across, wall, NEST_H), WALL_PALE),
    ]


def _door() -> Slider:
    """Five boxes, fixed together, round a small window, with the handle on the end nearest the arms. The joint's
    frame is the middle of that end's box."""
    x0, x1, y0, y1 = WINDOW
    half_w, half_l, t = DOOR_W / 2, DOOR_L / 2, DOOR_T
    front_size = (x0 + half_w, DOOR_L, t)
    mid = (-half_w + x0) / 2
    across = x1 - x0
    parts = (
        (((x1 + half_w) / 2 - mid, 0.0, 0.0), (half_w - x1, DOOR_L, t), CASE_DARK),
        (((x0 + x1) / 2 - mid, (y1 + half_l) / 2, 0.0), (across, half_l - y1, t), CASE_DARK),
        (((x0 + x1) / 2 - mid, (y0 - half_l) / 2, 0.0), (across, y0 + half_l, t), CASE_DARK),
        ((-half_w + HANDLE[1] / 2 - mid, 0.0, t / 2 + HANDLE[2] / 2), (HANDLE[1], HANDLE[0], HANDLE[2]), DARK),
    )
    # No spring: the door stays where it is pulled. Open is 0, and shutting takes it negative along x.
    return Slider("door", rest=(mid + DOOR_TRAVEL, 0.0, DOOR_Z), size=front_size, axis="X", lower=-DOOR_TRAVEL,
                  upper=0.0, color=CASE_DARK, rate=0.0, damping=8.0, mass=0.10, parts=parts)


READER = tool(
    "reader",
    pos=(*READER_XY, TOP + BODY_H / 2),
    boxes=_reader_boxes(),
    sliders=[_door()],
    mass=3.0,  # heavy enough to stay put when the door hits its stop; taped down on a real bench
)

OBJECTS = [*_shelf(), READER, SAMPLE_PLATE, SAMPLE_LID]


# -- what happened: a plate of samples tipped on the way can't be undone by setting it straight ------------------------
TIP_LIMIT_DEG = 30.0
TIPPED = {"yes": False}


@on_reset
def _reset(states) -> None:
    TIPPED["yes"] = False


@on_step
def _watch(states) -> None:
    if states["plate"].tilt_deg() > TIP_LIMIT_DEG:
        TIPPED["yes"] = True


# -- scoring: the end state, and the one latch above ---------------------------------------------------------------
FULL = 1.0
DOOR_OPEN = 0.7  # right in every way, not shut
WRONG_WAY_OPEN = 0.3  # a person sees it before anything is read
WRONG_WAY_SHUT = 0.2  # shut and ready to read, every result against the wrong well
LID_ON = 0.2
LID_ON_WRONG_WAY = 0.1
LID_ONLY = 0.1  # the first step done, the plate still on the shelf

FLAT_DEG = 10.0
ON_FLOOR = 0.005  # the height of the low walls
WALL_GIVE = 0.002
CORNER_WITHIN = 0.025
ON_SURFACE = 0.010
READER_MOVED = 0.030
READER_ENDS = ((FRONT_END, 0.0, 0.0), (REAR_END, 0.0, 0.0))


def _in_reader_frame(reader, point) -> np.ndarray:
    return reader.rotation().T @ (np.asarray(point, dtype=float) - reader.pos)


def reader_in_place(reader) -> bool:
    """Both ends of the reader within 3 cm of where they started, so it hasn't been shoved, turned or tipped."""
    home = np.array(READER.pos)
    return all(np.linalg.norm(reader.to_world(end) - (home + np.array(end))) <= READER_MOVED for end in READER_ENDS)


def plate_in(plate, reader) -> bool:
    """All four corners inside the low walls, flat, and down on the reader's floor, not resting on a wall."""
    half = np.array(PLATE[:2]) / 2
    limit = np.array(NEST) / 2 + WALL_GIVE
    corners = [_in_reader_frame(reader, plate.to_world((sx * half[0], sy * half[1], 0.0)))
               for sx in (-1, 1) for sy in (-1, 1)]
    inside = all(abs(c[0]) <= limit[0] and abs(c[1]) <= limit[1] for c in corners)
    return bool(inside and plate.low[2] < reader.to_world((0, 0, -BODY_H / 2))[2] + FLOOR_T + ON_FLOOR
                and plate.tilt_deg() <= FLAT_DEG)


def corner_right(plate, reader) -> bool:
    """The plate's orange corner is within 2.5 cm of the reader's, seen from above."""
    x, y, _ = _in_reader_frame(reader, plate.point("orange_corner"))
    return bool(math.hypot(x - CORNER_XY[0], y - CORNER_XY[1]) <= CORNER_WITHIN)


def over_shelf(state) -> bool:
    x, y = state.pos[:2]
    return bool(abs(x - SHELF_XY[0]) <= SHELF[0] / 2 and abs(y - SHELF_XY[1]) <= SHELF[1] / 2)


def plate_on_shelf(plate) -> bool:
    return bool(over_shelf(plate) and abs(plate.low[2] - SHELF_TOP) <= ON_SURFACE and plate.tilt_deg() <= FLAT_DEG)


def lid_clear(plate, lid) -> bool:
    """Seen from above, no part of the lid is over the plate."""
    return bool(lid.low[0] >= plate.high[0] or lid.high[0] <= plate.low[0]
                or lid.low[1] >= plate.high[1] or lid.high[1] <= plate.low[1])


def lid_touches_reader(lid, reader) -> bool:
    """Seen from above, any part of the lid is over the reader."""
    x, y = reader.pos[:2]
    return bool(lid.high[0] > x + FRONT_END and lid.low[0] < x + REAR_END
                and lid.high[1] > y - LEFT_END and lid.low[1] < y + LEFT_END)


def lid_off(lid, plate, reader) -> bool:
    """Down on the table or the shelf, with none of it over the plate or the reader."""
    on_table = abs(lid.low[2] - TOP) <= ON_SURFACE
    on_shelf = over_shelf(lid) and abs(lid.low[2] - SHELF_TOP) <= ON_SURFACE
    return bool((on_table or on_shelf) and lid_clear(plate, lid) and not lid_touches_reader(lid, reader))


def door_shut(reader) -> bool:
    return -reader.sliders["door"] >= DOOR_TRAVEL - DOOR_SHUT_WITHIN


def grade(objects, tipped: bool) -> float:
    """The score as the outcome table states it."""
    plate, lid, reader = objects["plate"], objects["lid"], objects["reader"]
    if tipped or not reader_in_place(reader):
        return 0.0
    off = lid_off(lid, plate, reader)
    if not plate_in(plate, reader):
        return LID_ONLY if off and plate_on_shelf(plate) else 0.0
    right = corner_right(plate, reader)
    if not off:
        return LID_ON if right else LID_ON_WRONG_WAY
    if right:
        return FULL if door_shut(reader) else DOOR_OPEN
    return WRONG_WAY_SHUT if door_shut(reader) else WRONG_WAY_OPEN


def success(objects) -> bool:
    return grade(objects, TIPPED["yes"]) >= 1.0 and all(objects[n].resting(max_speed=0.02, max_spin=1.0)
                                                        for n in ("plate", "lid"))


def score(objects) -> float:
    return grade(objects, TIPPED["yes"])


def distance(objects) -> float:
    """Metres from the plate to the middle of the reader's opening, plus what's left of the door's travel."""
    reader = objects["reader"]
    apart = float(np.linalg.norm(objects["plate"].pos[:2] - reader.pos[:2]))
    return apart + max(DOOR_TRAVEL + reader.sliders["door"], 0.0)


# -- the scripted demo: the left arm lifts the lid off, turns the plate half a turn in the air, sets it in, then hooks
#    the door's handle and pulls it shut -----------------------------------------------------------------------------
PAD_ABOVE_BASE = 0.010  # pad tips this far above the plate's underside: over the skirt, and over the low walls
TAB_GRIP_Z = LID_TOP + 0.018  # pad tips 5 mm above the lid, on the tab
LID_SET_Z = TOP + 0.0025 + LID[2] + 0.018
SIDE_GRIP_Z = SHELF_TOP + PAD_TIP_PAST_TCP + PAD_ABOVE_BASE
PLATE_SET_Z = FLOOR_Z + 0.0065 + PAD_TIP_PAST_TCP + PAD_ABOVE_BASE  # the grip sags about 5 mm in the carry
TRAVEL_Z = TOP + 0.170
QUARTER = math.pi / 2  # only this way round: the other way runs joint 6 into its stop
FINGER_HALF = 0.0065  # half the closed fingers' thickness along their closing axis, near the tips
OVERSHOOT = 0.006  # commanded past the stop, so the door still shuts if the arm falls a little short
HANDLE_SHUT_X = READER_XY[0] + FRONT_END + HANDLE[1]  # the handle's far face, the one the fingers pull on
HANDLE_OPEN_X = HANDLE_SHUT_X + DOOR_TRAVEL
PULL_Z = TOP + BODY_H + DOOR_GAP + DOOR_T + 0.005 + PAD_TIP_PAST_TCP  # finger tips 5 mm above the door, on the handle
PULL_START_X = HANDLE_OPEN_X + 0.030
PULL_HOVER_Z = TOP + 0.105  # the wrist can't reach this far out any higher. Finger tips clear the handle by 2 cm
PULL_END_X = HANDLE_SHUT_X + FINGER_HALF - OVERSHOOT
REST_XY = (0.22, 0.25)
TRIM = {"tab": (0.0, 0.0), "plate": (0.0, 0.0), "nest": (0.018, -0.0045), "pull": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    tx, ty = PLATE_XY[0] + TRIM["tab"][0], PLATE_XY[1] + TRIM["tab"][1]
    lx, ly = LID_SET_XY
    px, py = PLATE_XY[0] + TRIM["plate"][0], PLATE_XY[1] + TRIM["plate"][1]
    nx, ny = READER_XY[0] + TRIM["nest"][0], READER_XY[1] + TRIM["nest"][1]
    mx, my = (px + nx) / 2, (py + ny) / 2
    hy = READER_XY[1] + TRIM["pull"][1]
    start, end = PULL_START_X + TRIM["pull"][0], PULL_END_X + TRIM["pull"][0]
    return [
        Waypoint(3.0, (tx, ty, TRAVEL_Z), FINGER_OPEN, "reach over the lid"),
        Waypoint(1.0, (tx, ty, TRAVEL_Z), FINGER_OPEN, "settle"),
        Waypoint(2.0, (tx, ty, TAB_GRIP_Z), FINGER_OPEN, "descend on the tab"),
        Waypoint(0.5, (tx, ty, TAB_GRIP_Z), FINGER_OPEN, "settle"),
        Waypoint(1.2, (tx, ty, TAB_GRIP_Z), FINGER_CLOSED, "close on the tab"),
        Waypoint(0.5, (tx, ty, TAB_GRIP_Z), FINGER_CLOSED, "hold"),
        Waypoint(2.0, (tx, ty, TRAVEL_Z), FINGER_CLOSED, "lift the lid off"),
        Waypoint(3.0, (lx, ly, TRAVEL_Z), FINGER_CLOSED, "carry it over the table"),
        Waypoint(2.0, (lx, ly, LID_SET_Z), FINGER_CLOSED, "lower it"),
        Waypoint(0.5, (lx, ly, LID_SET_Z), FINGER_CLOSED, "settle"),
        Waypoint(1.0, (lx, ly, LID_SET_Z), FINGER_OPEN, "let go"),
        Waypoint(1.5, (lx, ly, TRAVEL_Z), FINGER_OPEN, "up"),
        Waypoint(3.0, (px, py, TRAVEL_Z), FINGER_OPEN, "back over the plate", QUARTER),
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "settle", QUARTER),
        Waypoint(2.5, (px, py, SIDE_GRIP_Z), FINGER_OPEN, "descend round the plate", QUARTER),
        Waypoint(0.5, (px, py, SIDE_GRIP_Z), FINGER_OPEN, "settle", QUARTER),
        Waypoint(1.2, (px, py, SIDE_GRIP_Z), FINGER_CLOSED, "close on its long sides", QUARTER),
        Waypoint(0.5, (px, py, SIDE_GRIP_Z), FINGER_CLOSED, "hold", QUARTER),
        Waypoint(2.0, (px, py, TRAVEL_Z), FINGER_CLOSED, "lift", QUARTER),
        Waypoint(2.5, (mx, my, TRAVEL_Z), FINGER_CLOSED, "carry, turning", 0.0),
        Waypoint(2.5, (nx, ny, TRAVEL_Z), FINGER_CLOSED, "carry, finishing the half turn", -QUARTER),
        Waypoint(1.5, (nx, ny, TRAVEL_Z), FINGER_CLOSED, "settle", -QUARTER),
        Waypoint(2.5, (nx, ny, PLATE_SET_Z), FINGER_CLOSED, "lower into the reader", -QUARTER),
        Waypoint(0.5, (nx, ny, PLATE_SET_Z), FINGER_CLOSED, "settle", -QUARTER),
        Waypoint(1.0, (nx, ny, PLATE_SET_Z), FINGER_OPEN, "let go", -QUARTER),
        Waypoint(2.0, (nx, ny, TRAVEL_Z), FINGER_OPEN, "up", -QUARTER),
        Waypoint(2.5, (start, hy, PULL_HOVER_Z), FINGER_CLOSED, "over the door, fingers closing", -QUARTER),
        Waypoint(1.0, (start, hy, PULL_HOVER_Z), FINGER_CLOSED, "settle", -QUARTER),
        Waypoint(2.0, (start, hy, PULL_Z), FINGER_CLOSED, "down behind the handle", -QUARTER),
        Waypoint(0.5, (start, hy, PULL_Z), FINGER_CLOSED, "settle", -QUARTER),
        Waypoint(5.0, (end, hy, PULL_Z), FINGER_CLOSED, "pull the door shut", -QUARTER),
        Waypoint(0.8, (end, hy, PULL_Z), FINGER_CLOSED, "hold", -QUARTER),
        Waypoint(1.5, (end, hy, TRAVEL_Z), FINGER_CLOSED, "lift off", -QUARTER),
        Waypoint(2.5, (*REST_XY, TRAVEL_Z), FINGER_CLOSED, "clear of the reader", 0.0),
    ]


def demo():
    """The left arm does it all: the lid to the table, the plate into the reader, turned half a turn in the air, and
    the door pulled shut by its handle. The right arm holds ready. The fingers pass the plate's body with 12 mm to
    spare each side."""
    return {"left": _left_plan()}
