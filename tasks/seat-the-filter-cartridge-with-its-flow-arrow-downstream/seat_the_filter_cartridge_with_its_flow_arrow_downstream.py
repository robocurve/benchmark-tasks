"""On the station table, in front of the left arm: a white filter cartridge, a square block 7 cm across and 6 cm tall,
standing on a dark plate. A black arrow is raised on its top, pointing away from the arms. Further out and to the
right, a white open-top filter housing, taped down: a square cup 8.5 cm across inside and 4 cm tall, with a grey floor.
A yellow pipe runs up to each side of the housing; the one on the robot's right carries a green tag. Beyond the
housing, a white plate lies flat on the table along the line of the two pipes, marked IN at its right-hand end and OUT
at its left-hand end, with a black arrow between them.

Run it:  inspect-robots run --embodiment yam_arena -E task=seat_the_filter_cartridge_with_its_flow_arrow_downstream.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import DARK, GREEN, GREY, WHITE, YELLOW, Clock, distance_xy, over, tilt_deg  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Fit the new cartridge into the filter housing with the arrow on its top pointing the way the water flows: from "
    "the inlet pipe towards the outlet pipe."
)

CARTRIDGE_WHITE = (0.97, 0.97, 0.96)
BLACK = (0.06, 0.06, 0.07)

# -- the housing and its pipes: the water runs from the inlet (robot's right) to the outlet (robot's left), along +y ---
HOUSING_XY = (0.28, 0.12)
HOUSING_INNER = 0.085  # m: square opening, 7.5 mm round the cartridge on each side
HOUSING_WALL = 0.006  # m
HOUSING_OUTER = HOUSING_INNER + 2 * HOUSING_WALL
HOUSING_H = 0.040  # m: the walls, from the table
HOUSING_FLOOR = 0.004  # m: a floor plate between the walls
FLOOR_Z = TOP + HOUSING_FLOOR
RIM_Z = TOP + HOUSING_H

PIPE = (0.030, 0.080, 0.030)  # x, y, z (m): a stub of pipe lying on the table, square in section
INLET_XY = (0.28, 0.03)
OUTLET_XY = (0.28, 0.21)
IN_TAG = (0.024, 0.040, 0.002)  # m: a green patch on the inlet's top
FLOW_YAW_DEG = math.degrees(math.atan2(OUTLET_XY[1] - INLET_XY[1], OUTLET_XY[0] - INLET_XY[0]))  # 90: along +y

# -- arrows: a shaft, then a head narrowing in three steps to the point (m: along the arrow, across it) ----------------
ARROW_T = 0.002  # m: raised this far above the surface it is on
SHAFT = (0.030, 0.018)  # on the cartridge: 3 px wide in the 320 x 176 top-camera image, seated or not
HEAD = ((0.009, 0.050), (0.009, 0.034), (0.009, 0.018))  # a 5 cm-wide wedge, 9 px across in that image
PLATE_SHAFT = (0.040, 0.022)  # on the plate, which is further from the top camera, so a size larger
PLATE_HEAD = ((0.012, 0.060), (0.012, 0.042), (0.012, 0.024))


def arrow_parts(shaft, head) -> list[tuple[float, float, float]]:
    """(centre along the arrow from its tail, length along it, width across it) for each box, tail to point."""
    parts, along = [(shaft[0] / 2, shaft[0], shaft[1])], shaft[0]
    for length, width in head:
        parts.append((along + length / 2, length, width))
        along += length
    return parts


def arrow_length(shaft, head) -> float:
    return shaft[0] + sum(length for length, _ in head)


# -- the stand and the cartridge (tool frame: origin at the block's centre, the arrow along +x) ------------------------
STAND_XY = (0.17, 0.20)
STAND = (0.090, 0.090, 0.006)  # m: a flat plate
STAND_TOP = TOP + STAND[2]

CART = (0.070, 0.070, 0.060)  # x, y, z (m)
CART_MASS = 0.10  # kg
ARROW_LEN = arrow_length(SHAFT, HEAD)
ARROW_TAIL_X = -ARROW_LEN / 2  # centred on the top
ARROW_TIP_X = ARROW_LEN / 2
ARROW_Z = CART[2] / 2 + ARROW_T / 2
ARROW_TOP = CART[2] / 2 + ARROW_T

CARTRIDGE = tool(
    "cartridge",
    pos=(*STAND_XY, STAND_TOP + CART[2] / 2),
    boxes=[((0.0, 0.0, 0.0), CART, CARTRIDGE_WHITE),
           *(((ARROW_TAIL_X + c, 0.0, ARROW_Z), (length, width, ARROW_T), BLACK)
             for c, length, width in arrow_parts(SHAFT, HEAD))],
    mass=CART_MASS,  # one rigid body: the arrow turns with the block; no sliders
    points=[("arrow_tip", (ARROW_TIP_X, 0.0, ARROW_TOP)), ("arrow_tail", (ARROW_TAIL_X, 0.0, ARROW_TOP))],
)

# -- the label plate beyond the housing: IN at the inlet's end, OUT at the outlet's, an arrow between ------------------
# The letters lie flat and read from the arms' side, as the top camera sees them: their tops away from the arms (+x),
# left to right towards the robot's right (-y). The plate runs the length of the two pipes.
LABEL_X = 0.370
LABEL_Y = (INLET_XY[1] - PIPE[1] / 2 - 0.005, OUTLET_XY[1] + PIPE[1] / 2 + 0.005)  # m: its two ends
LABEL = (0.075, LABEL_Y[1] - LABEL_Y[0], 0.004)
LABEL_TOP = TOP + LABEL[2]
LETTER_H = 0.040  # m, along x
LETTER_W = 0.030  # m, along y; an I is one stroke wide
STROKE = 0.010  # m
LETTER_GAP = 0.008  # m
LABEL_MARGIN = 0.008  # m: from the words to the plate's ends


def _strokes(ch: str) -> list[tuple[float, float, float, float]]:
    """A letter's strokes as centre lines (u0, v0, u1, v1): u across it in reading order, v up it."""
    left, right, low, high = STROKE / 2, LETTER_W - STROKE / 2, STROKE / 2, LETTER_H - STROKE / 2
    return {
        "I": [(STROKE / 2, low, STROKE / 2, high)],
        "N": [(left, low, left, high), (right, low, right, high), (left, high, right, low)],
        "O": [(left, low, left, high), (right, low, right, high), (left, low, right, low), (left, high, right, high)],
        "U": [(left, low, left, high), (right, low, right, high), (left, low, right, low)],
        "T": [(left, high, right, high), (LETTER_W / 2, low, LETTER_W / 2, high)],
    }[ch]


def letter_width(ch: str) -> float:
    return STROKE if ch == "I" else LETTER_W


def word_width(word: str) -> float:
    return sum(letter_width(ch) for ch in word) + LETTER_GAP * (len(word) - 1)


def _word(name: str, word: str, left_y: float) -> list[Box]:
    """Flat black strokes spelling ``word``, its first letter's left edge at ``left_y``."""
    boxes, y0, x0, z = [], left_y, LABEL_X - LETTER_H / 2, LABEL_TOP + ARROW_T / 2
    for ch in word:
        for u0, v0, u1, v1 in _strokes(ch):
            (xa, ya), (xb, yb) = (x0 + v0, y0 - u0), (x0 + v1, y0 - u1)
            centre = ((xa + xb) / 2, (ya + yb) / 2, z)
            if ya == yb:  # up the letter
                size, yaw = (abs(xb - xa) + STROKE, STROKE, ARROW_T), 0.0
            elif xa == xb:  # across it
                size, yaw = (STROKE, abs(yb - ya) + STROKE, ARROW_T), 0.0
            else:  # a diagonal, turned about the vertical
                size, yaw = (math.hypot(xb - xa, yb - ya), STROKE, ARROW_T), math.atan2(yb - ya, xb - xa)
            boxes.append(Box(f"{name}_{len(boxes)}", pos=centre, size=size, color=BLACK, yaw=yaw))
        y0 -= letter_width(ch) + LETTER_GAP
    return boxes


IN_LEFT_Y = LABEL_Y[0] + LABEL_MARGIN + word_width("IN")  # IN at the inlet's end
OUT_LEFT_Y = LABEL_Y[1] - LABEL_MARGIN  # OUT at the outlet's end
PLATE_ARROW_LEN = arrow_length(PLATE_SHAFT, PLATE_HEAD)
PLATE_ARROW_TAIL_Y = (IN_LEFT_Y + OUT_LEFT_Y - word_width("OUT")) / 2 - PLATE_ARROW_LEN / 2  # centred between the words


def _label() -> list[Box]:
    plate = Box("label_plate", pos=(LABEL_X, sum(LABEL_Y) / 2, TOP + LABEL[2] / 2), size=LABEL, color=WHITE)
    arrow = [Box(f"label_arrow_{i}", pos=(LABEL_X, PLATE_ARROW_TAIL_Y + c, LABEL_TOP + ARROW_T / 2),
                 size=(width, length, ARROW_T), color=BLACK)
             for i, (c, length, width) in enumerate(arrow_parts(PLATE_SHAFT, PLATE_HEAD))]
    return [plate, *arrow, *_word("label_in", "IN", IN_LEFT_Y), *_word("label_out", "OUT", OUT_LEFT_Y)]


def _housing() -> list[Box]:
    """Four fixed walls round a square opening, and the floor plate between them."""
    x, y = HOUSING_XY
    off = HOUSING_INNER / 2 + HOUSING_WALL / 2
    z = TOP + HOUSING_H / 2
    return [
        Box("housing_far", pos=(x + off, y, z), size=(HOUSING_WALL, HOUSING_OUTER, HOUSING_H), color=WHITE),
        Box("housing_near", pos=(x - off, y, z), size=(HOUSING_WALL, HOUSING_OUTER, HOUSING_H), color=WHITE),
        Box("housing_left", pos=(x, y + off, z), size=(HOUSING_INNER, HOUSING_WALL, HOUSING_H), color=WHITE),
        Box("housing_right", pos=(x, y - off, z), size=(HOUSING_INNER, HOUSING_WALL, HOUSING_H), color=WHITE),
        Box("housing_floor", pos=(x, y, TOP + HOUSING_FLOOR / 2), size=(HOUSING_INNER, HOUSING_INNER, HOUSING_FLOOR),
            color=GREY),
    ]


OBJECTS = [
    *_housing(),
    Box("inlet_pipe", pos=(*INLET_XY, TOP + PIPE[2] / 2), size=PIPE, color=YELLOW),
    Box("in_tag", pos=(*INLET_XY, TOP + PIPE[2] + IN_TAG[2] / 2), size=IN_TAG, color=GREEN),
    Box("outlet_pipe", pos=(*OUTLET_XY, TOP + PIPE[2] / 2), size=PIPE, color=YELLOW),
    *_label(),
    Box("stand", pos=(*STAND_XY, TOP + STAND[2] / 2), size=STAND, color=DARK),
    CARTRIDGE,
]

# -- reading the cartridge ---------------------------------------------------------------------------------------------
SEATED_XY = 0.012  # m: its centre this close to the housing's centre, seen from above (it can be 10.6 mm off at most)
SEATED_UP = 0.005  # m: its bottom this close to the housing's floor, so it counts only once it is let go
UPRIGHT_DEG = 10.0  # degrees: it leans less than this
ARROW_WITHIN_DEG = 20.0  # degrees: the arrow this close to a direction points that way
STANDING_UP = 0.010  # m: standing on the table or the stand, its bottom no higher than this above the stand's top
LIFTED_UP = 0.010  # m: off the stand, for the record


def arrow_yaw_deg(cart: ToolState) -> float:
    """Which way the arrow points seen from above, in degrees anticlockwise from +x (away from the arms)."""
    d = cart.rotation()[:, 0]
    return float(math.degrees(math.atan2(d[1], d[0])))


def off_flow_deg(cart: ToolState) -> float:
    """Degrees between the arrow and the way the water runs, inlet to outlet: 0 to 180."""
    return abs((arrow_yaw_deg(cart) - FLOW_YAW_DEG + 180.0) % 360.0 - 180.0)


def upright(cart: ToolState) -> bool:
    return tilt_deg(cart) < UPRIGHT_DEG


def downstream(cart: ToolState) -> bool:
    """Upright with the arrow within 20 degrees of inlet-to-outlet."""
    return upright(cart) and off_flow_deg(cart) <= ARROW_WITHIN_DEG


def backwards(cart: ToolState) -> bool:
    return upright(cart) and off_flow_deg(cart) >= 180.0 - ARROW_WITHIN_DEG


def seated(cart: ToolState) -> bool:
    """Down in the housing: centred within 1.2 cm, bottom within 5 mm of the floor, leaning under 10 degrees."""
    return bool(upright(cart) and distance_xy(cart, HOUSING_XY) <= SEATED_XY
                and abs(cart.low[2] - FLOOR_Z) <= SEATED_UP)


def standing_on_bench(cart: ToolState) -> bool:
    """Upright on the table, the stand or the plate: not in or over the housing, held up, on a pipe or fallen off."""
    in_housing = over(cart, HOUSING_XY, (HOUSING_OUTER / 2, HOUSING_OUTER / 2))
    return bool(upright(cart) and not in_housing and TOP - 0.005 <= cart.low[2] <= STAND_TOP + STANDING_UP)


def off_table(cart: ToolState) -> bool:
    return bool(cart.pos[2] < TOP)


# -- a record of the trial: nothing here is scored, and nothing latches ------------------------------------------------
CLOCK = Clock()
WHEN = {"lifted": None, "turned": None, "seated": None}  # seconds after the reset each first happened


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    for key in WHEN:
        WHEN[key] = None


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    cart = states["cartridge"]
    if WHEN["lifted"] is None and cart.low[2] > STAND_TOP + LIFTED_UP:
        WHEN["lifted"] = CLOCK.t
    if WHEN["turned"] is None and downstream(cart):
        WHEN["turned"] = CLOCK.t
    if WHEN["seated"] is None and seated(cart):
        WHEN["seated"] = CLOCK.t


# -- scoring: the end state --------------------------------------------------------------------------------------------
FITTED = 1.0
TURNED_NOT_SEATED = 0.3
SEATED_SIDEWAYS = 0.2  # the arrow along x, or at any angle that is neither downstream nor backwards
BACKWARDS = 0.0


def grade(objects) -> float:
    """The score as the outcome table states it."""
    cart = objects["cartridge"]
    if off_table(cart):
        return 0.0
    if seated(cart):
        if downstream(cart):
            return FITTED
        return BACKWARDS if backwards(cart) else SEATED_SIDEWAYS
    if downstream(cart) and standing_on_bench(cart):
        return TURNED_NOT_SEATED
    return 0.0


def success(objects) -> bool:
    return grade(objects) >= 1.0 and objects["cartridge"].resting(max_speed=0.02, max_spin=1.0)


def score(objects) -> float:
    return grade(objects)


def distance(objects) -> float:
    """Metres from the cartridge's centre to the housing's, plus its bottom's height over the floor, plus the arc the
    arrow's point still has to swing through to point downstream."""
    cart = objects["cartridge"]
    return (distance_xy(cart, HOUSING_XY) + max(float(cart.low[2]) - FLOOR_Z, 0.0)
            + ARROW_TIP_X * math.radians(off_flow_deg(cart)))


# -- the scripted demo: the left arm lifts the cartridge, turns it a quarter in the hand and lowers it in --------------
GRIP_ABOVE_BOTTOM = 0.054  # m: the TCP above the cartridge's bottom, so the pads close on its top 1.9 cm
PICK_Z = STAND_TOP + GRIP_ABOVE_BOTTOM
PLACE_GAP = 0.007  # m: let go with its bottom this far above the floor, so it counts as seated only once dropped
PLACE_Z = FLOOR_Z + PLACE_GAP + GRIP_ABOVE_BOTTOM  # pad tips 12 mm above the rim, so the fingers never enter the cup
TRAVEL_Z = TOP + 0.150  # carried with its bottom 5.6 cm over the rim
# Gripped across y and turned to +90 degrees, joint6 would pass its 120-degree limit. Gripped across x (on the faces at
# the arrow's two ends) and set down with the fingers across y, the same anticlockwise quarter turn stays inside it.
PICK_YAW = -math.pi / 2
PLACE_YAW = 0.0
PARK = (0.20, 0.26, TRAVEL_Z)  # clear of the housing, on the left arm's own side, so the top camera sees the arrow
TRIM = {"pick": (0.0, 0.0), "place": (0.0, 0.0)}  # m: demo-only aim offsets, for calibration; never scored


def demo():
    """Left arm: take the cartridge off the stand, turn it a quarter anticlockwise on the way, lower it into the
    housing with the arrow towards the outlet, let go and move aside."""
    px, py = TRIM["pick"]
    qx, qy = TRIM["place"]
    plan = pick_place_waypoints((STAND_XY[0] + px, STAND_XY[1] + py, PICK_Z),
                                (HOUSING_XY[0] + qx, HOUSING_XY[1] + qy, PLACE_Z),
                                travel_z=TRAVEL_Z, yaw=PICK_YAW, place_yaw=PLACE_YAW, speed=0.7)
    plan.append(Waypoint(1.5, PARK, FINGER_OPEN, "move aside", PLACE_YAW))
    return {"left": plan}
