"""On the station table, in front of the left arm: a white filter cartridge, a square block 7 cm across and 6 cm tall,
standing on a dark plate. A black arrow is raised on its top, pointing away from the arms. Further out and to the
right, a white open-top filter housing, taped down: a square cup 8 cm across inside and 4 cm tall, with a grey floor.
A yellow pipe runs up to each side of the housing; the one on the robot's right carries a green tag reading IN.

Run it:  inspect-robots run --embodiment yam_arena -E task=seat_the_filter_cartridge_with_its_flow_arrow_downstream.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import DARK, GREEN, GREY, WHITE, YELLOW, Clock, distance_xy, tilt_deg  # noqa: E402
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
ARROW_BLACK = (0.06, 0.06, 0.07)

# -- the housing and its pipes: the water runs from the inlet (robot's right) to the outlet (robot's left), along +y ---
HOUSING_XY = (0.28, 0.12)
HOUSING_INNER = 0.080  # square opening (m)
HOUSING_WALL = 0.006  # m
HOUSING_H = 0.040  # m: the walls, from the table
HOUSING_FLOOR = 0.004  # m: a floor plate between the walls
FLOOR_Z = TOP + HOUSING_FLOOR
RIM_Z = TOP + HOUSING_H

PIPE = (0.030, 0.080, 0.030)  # x, y, z (m): a stub of pipe lying on the table, square in section
INLET_XY = (0.28, 0.03)
OUTLET_XY = (0.28, 0.21)
IN_TAG = (0.024, 0.040, 0.002)  # m: a green patch on the inlet's top
FLOW_YAW_DEG = math.degrees(math.atan2(OUTLET_XY[1] - INLET_XY[1], OUTLET_XY[0] - INLET_XY[0]))  # 90: along +y

# -- the stand and the cartridge (tool frame: origin at the block's centre, the arrow along +x) ------------------------
STAND_XY = (0.18, 0.18)
STAND = (0.090, 0.090, 0.006)  # m: a flat plate
STAND_TOP = TOP + STAND[2]

CART = (0.070, 0.070, 0.060)  # x, y, z (m)
CART_MASS = 0.10  # kg
ARROW_T = 0.002  # m: raised this far above the top
SHAFT = (0.040, 0.008)  # m: length along the arrow, width across it
HEAD = ((0.007, 0.030), (0.007, 0.020), (0.007, 0.010))  # stepped, from the shaft's end to the point: length, width
ARROW_LEN = SHAFT[0] + sum(length for length, _ in HEAD)
ARROW_TAIL_X = -ARROW_LEN / 2  # centred on the top
ARROW_TIP_X = ARROW_LEN / 2
ARROW_Z = CART[2] / 2 + ARROW_T / 2
ARROW_TOP = CART[2] / 2 + ARROW_T


def _arrow() -> list:
    """The arrow as raised boxes on the cartridge's top: a shaft, then a head that narrows in steps to the point."""
    x = ARROW_TAIL_X
    boxes = [((x + SHAFT[0] / 2, 0.0, ARROW_Z), (SHAFT[0], SHAFT[1], ARROW_T), ARROW_BLACK)]
    x += SHAFT[0]
    for length, width in HEAD:
        boxes.append(((x + length / 2, 0.0, ARROW_Z), (length, width, ARROW_T), ARROW_BLACK))
        x += length
    return boxes


CARTRIDGE = tool(
    "cartridge",
    pos=(*STAND_XY, STAND_TOP + CART[2] / 2),
    boxes=[((0.0, 0.0, 0.0), CART, CARTRIDGE_WHITE), *_arrow()],
    mass=CART_MASS,  # one rigid body: the arrow turns with the block; no sliders
    points=[("arrow_tip", (ARROW_TIP_X, 0.0, ARROW_TOP)), ("arrow_tail", (ARROW_TAIL_X, 0.0, ARROW_TOP))],
)


def _housing() -> list[Box]:
    """Four fixed walls round a square opening, and the floor plate between them."""
    x, y = HOUSING_XY
    off = HOUSING_INNER / 2 + HOUSING_WALL / 2
    z = TOP + HOUSING_H / 2
    long = HOUSING_INNER + 2 * HOUSING_WALL
    return [
        Box("housing_far", pos=(x + off, y, z), size=(HOUSING_WALL, long, HOUSING_H), color=WHITE),
        Box("housing_near", pos=(x - off, y, z), size=(HOUSING_WALL, long, HOUSING_H), color=WHITE),
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
    Box("stand", pos=(*STAND_XY, TOP + STAND[2] / 2), size=STAND, color=DARK),
    CARTRIDGE,
]

# -- reading the cartridge ---------------------------------------------------------------------------------------------
SEATED_XY = 0.010  # m: its centre this close to the housing's centre, seen from above
SEATED_UP = 0.010  # m: its bottom this close to the housing's floor
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
    """Down in the housing: centred within 1 cm, bottom within 1 cm of the floor, leaning under 10 degrees."""
    return bool(upright(cart) and distance_xy(cart, HOUSING_XY) <= SEATED_XY
                and abs(cart.low[2] - FLOOR_Z) <= SEATED_UP)


def standing_on_bench(cart: ToolState) -> bool:
    """Upright on the table or the stand, not held up, propped on a pipe or fallen off."""
    return bool(upright(cart) and TOP - 0.005 <= cart.low[2] <= STAND_TOP + STANDING_UP)


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
PLACE_GAP = 0.003  # m: let go with its bottom this far above the housing's floor
PLACE_Z = FLOOR_Z + PLACE_GAP + GRIP_ABOVE_BOTTOM  # pad tips 8 mm above the rim, so the fingers never enter the cup
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
