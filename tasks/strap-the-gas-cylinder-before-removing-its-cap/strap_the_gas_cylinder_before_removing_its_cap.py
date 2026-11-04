"""On the station table, between the arms and left of the midline: a dark cradle, taped down, holding a green gas
cylinder upright with a grey cap resting on its top. The cradle has a back post and two side posts, open towards the
arms, and across its open front a yellow strap bar that slides sideways, with a raised push tab on its free end. The
strap bar starts drawn out to the robot's right, its tab reaching 4 cm past the table's midline, and slides to the left
until it stops. A yellow tray stands to the left of the cradle, a little nearer the arms.

Run it:  inspect-robots run --embodiment yam_arena -E task=strap_the_gas_cylinder_before_removing_its_cap.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import DARK, GREEN, GREY, YELLOW, Clock, distance_xy, over, pushed, tilt_deg  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Secure the cylinder: push the strap bar across the cradle until it stops, then take the cap off the top of the "
    "cylinder and put it in the tray."
)

# -- the cradle, in its own frame: origin at the plate's centre, x towards the back post, y along the strap's travel --
CRADLE_XY = (0.22, 0.14)
PLATE = (0.12, 0.12, 0.010)
POST_T = 0.020
POST_H = 0.100
SIDE_POST_L = 0.090  # along x: the side posts start 3 cm behind the plate's front edge, clear of the bar's channel
SIDE_POST_X = PLATE[0] / 2 - SIDE_POST_L / 2
SIDE_POST_Y = PLATE[1] / 2 - POST_T / 2
POST_FRONT_X = SIDE_POST_X - SIDE_POST_L / 2  # the side posts' front faces
BACK_POST_X = PLATE[0] / 2 - POST_T / 2
BACK_POST_W = PLATE[1] - 2 * POST_T  # between the side posts
POST_Z = PLATE[2] / 2 + POST_H / 2
OPENING = PLATE[1] - 2 * POST_T  # between the side posts' inner faces

BAR = (0.015, 0.100, 0.020)
BAR_X = -PLATE[0] / 2 + BAR[0] / 2  # its front face flush with the plate's front edge
BAR_ABOVE_PLATE = 0.070  # its centre line, above the plate's top
BAR_Z = PLATE[2] / 2 + BAR_ABOVE_PLATE
STRAP_TRAVEL = 0.080
BAR_ACROSS_Y = 0.0  # against its stop it is centred on the cradle, covering the whole opening
BAR_OPEN_Y = BAR_ACROSS_Y - STRAP_TRAVEL  # at rest it is drawn out towards the midline
STRAPPED_AT = 0.7  # of its travel
# The push tab on the bar's free end: what the right arm's fingers push on. Raised so their tips pass 2 cm over the
# ledge the bar rides on (a real bench; see TAB_RAISE), and 3 cm tall so they cannot slip over it.
TAB = (0.015, 0.050, 0.030)
TAB_RAISE = 0.015  # its underside above the bar's underside
TAB_Y = -BAR[1] / 2 - TAB[1] / 2  # in the bar's frame
TAB_DZ = TAB_RAISE + TAB[2] / 2 - BAR[2] / 2

CYL_R, CYL_H, CYL_MASS = 0.030, 0.120, 0.8
CAP_R, CAP_H, CAP_MASS = 0.025, 0.020, 0.04

CRADLE = tool(
    "cradle",
    pos=(*CRADLE_XY, TOP + PLATE[2] / 2),
    boxes=[
        ((0.0, 0.0, 0.0), PLATE, DARK),
        ((BACK_POST_X, 0.0, POST_Z), (POST_T, BACK_POST_W, POST_H), DARK),
        ((SIDE_POST_X, SIDE_POST_Y, POST_Z), (SIDE_POST_L, POST_T, POST_H), DARK),
        ((SIDE_POST_X, -SIDE_POST_Y, POST_Z), (SIDE_POST_L, POST_T, POST_H), DARK),
    ],
    sliders=[
        # No spring: the strap bar stays where it is pushed, and its joint limit is its stop. The end state is scored.
        Slider("strap", rest=(BAR_X, BAR_OPEN_Y, BAR_Z), size=BAR, axis="Y", lower=0.0, upper=STRAP_TRAVEL,
               color=YELLOW, rate=0.0, damping=8.0, mass=0.05, parts=(((0.0, TAB_Y, TAB_DZ), TAB, YELLOW),)),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)

# -- the cylinder, its cap and the tray -------------------------------------------------------------------------------
PLATE_TOP = TOP + PLATE[2]
CYL_Z = PLATE_TOP + CYL_H / 2
CYL_TOP = PLATE_TOP + CYL_H
CAP_Z = CYL_TOP + CAP_H / 2
TRAY_XY = (0.16, 0.26)
TRAY_INNER = 0.080
TRAY_H = 0.020
TRAY_WALL = 0.003


def _collar(name: str, x: float, y: float, inner: float, height: float, wall: float, color) -> list[Box]:
    off = inner / 2 + wall / 2
    z = TOP + height / 2
    long = inner + 2 * wall
    return [
        Box(f"{name}_f", pos=(x + off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_b", pos=(x - off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_l", pos=(x, y + off, z), size=(inner, wall, height), color=color),
        Box(f"{name}_r", pos=(x, y - off, z), size=(inner, wall, height), color=color),
    ]


OBJECTS = [
    *_collar("tray", *TRAY_XY, TRAY_INNER, TRAY_H, TRAY_WALL, YELLOW),
    Cylinder("cylinder", pos=(*CRADLE_XY, CYL_Z), radius=CYL_R, height=CYL_H, mass=CYL_MASS, color=GREEN),
    Cylinder("cap", pos=(*CRADLE_XY, CAP_Z), radius=CAP_R, height=CAP_H, mass=CAP_MASS, color=GREY),
    CRADLE,
]

# -- scoring: the end state, plus two things that latch on the way: the cap coming off unsecured, and a tip -----------
TIPPED_DEG = 20.0
CAP_OFF_XY = 0.020  # the cap's centre this far off the cylinder's axis, measured at the cylinder's top
CAP_OFF_UP = 0.010  # or its underside this far above the cylinder's top
IN_CRADLE_XY = 0.030  # the cylinder's axis within this of the plate's centre, in any direction
IN_CRADLE_UP = 0.015  # and its base within this of the plate
IN_TRAY_UP = 0.015
STRAPPED_ONLY = 0.5
CAP_OFF_LOOSE = 0.5

CLOCK = Clock()
LATCHED = {"unsecured": False, "tipped": False}
WHEN = {"strapped": None, "cap_off": None}  # seconds after the reset each first happened, for the record


def axes(state) -> np.ndarray:
    """A body's rotation matrix from its quaternion (xyzw): its columns are the body's axes in the world."""
    x, y, z, w = state.quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def strapped(cradle: ToolState) -> bool:
    return pushed(cradle, "strap", STRAP_TRAVEL, STRAPPED_AT)


def in_cradle(cylinder, cradle: ToolState) -> bool:
    """Standing on the plate between the posts, in the cradle's own frame, so shoving the cradle changes nothing."""
    x, y, z = cradle.rotation().T @ (cylinder.pos - cradle.pos)
    return bool(np.hypot(x, y) <= IN_CRADLE_XY and z - CYL_H / 2 < PLATE[2] / 2 + IN_CRADLE_UP
                and tilt_deg(cylinder) <= TIPPED_DEG)


def secured(cradle: ToolState, cylinder) -> bool:
    return strapped(cradle) and in_cradle(cylinder, cradle)


def cap_off(cap, cylinder) -> bool:
    """Measured in the cylinder's frame from the centre of its top, so a capped cylinder leaning on the bar is still
    capped: the cap's centre off the axis, or its underside lifted clear."""
    frame = axes(cylinder)
    local = frame.T @ (cap.pos - (cylinder.pos + frame @ np.array([0.0, 0.0, CYL_H / 2])))
    return bool(np.hypot(local[0], local[1]) > CAP_OFF_XY or local[2] - CAP_H / 2 > CAP_OFF_UP)


def cap_in_tray(cap) -> bool:
    """Inside the collar with its underside on or near the table, however it lies."""
    return bool(over(cap, TRAY_XY, (TRAY_INNER / 2, TRAY_INNER / 2)) and cap.low[2] < TOP + IN_TRAY_UP)


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    LATCHED["unsecured"] = LATCHED["tipped"] = False
    WHEN["strapped"] = WHEN["cap_off"] = None


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    cradle, cylinder, cap = states["cradle"], states["cylinder"], states["cap"]
    if tilt_deg(cylinder) > TIPPED_DEG:
        LATCHED["tipped"] = True
    if WHEN["strapped"] is None and strapped(cradle):
        WHEN["strapped"] = CLOCK.t
    if cap_off(cap, cylinder):
        if WHEN["cap_off"] is None:
            WHEN["cap_off"] = CLOCK.t
        if not secured(cradle, cylinder):
            LATCHED["unsecured"] = True


def grade(objects, unsecured: bool, tipped: bool) -> float:
    """The score as the outcome table states it."""
    cradle, cylinder, cap = objects["cradle"], objects["cylinder"], objects["cap"]
    if tipped or unsecured or tilt_deg(cylinder) > TIPPED_DEG:
        return 0.0
    if not secured(cradle, cylinder):
        return 0.0  # the strap open at the end, or the cylinder out of the cradle: nothing is secured
    if not cap_off(cap, cylinder):
        return STRAPPED_ONLY
    return 1.0 if cap_in_tray(cap) else CAP_OFF_LOOSE


def success(objects) -> bool:
    still = all(objects[n].resting(max_speed=0.02, max_spin=1.0) for n in ("cylinder", "cap"))
    return grade(objects, LATCHED["unsecured"], LATCHED["tipped"]) >= 1.0 and still


def score(objects) -> float:
    return grade(objects, LATCHED["unsecured"], LATCHED["tipped"])


def distance(objects) -> float:
    """What is left of the strap's travel, plus metres from the cap to the tray."""
    left = max(STRAP_TRAVEL - objects["cradle"].sliders["strap"], 0.0)
    return left + distance_xy(objects["cap"], TRAY_XY)


# -- the scripted demo: the right arm pushes the strap across; the left arm then lifts the cap off into the tray ------
HOVER_Z = TOP + 0.185
PUSH_X = CRADLE_XY[0] + BAR_X  # over the bar's centre line
BAR_BOTTOM_Z = TOP + PLATE[2] / 2 + BAR_Z - BAR[2] / 2
LEDGE_TOP_Z = BAR_BOTTOM_Z  # the ledge the bar rides on (a real bench; not modelled) ends at the bar's underside
TAB_BOTTOM_Z = BAR_BOTTOM_Z + TAB_RAISE
PUSH_Z = TAB_BOTTOM_Z + 0.005 + PAD_TIP_PAST_TCP  # pad tips 5 mm above the tab's underside, TCP 1.2 cm under its top
FINGER_Y = 0.0065  # half the closed fingers' width along y, near the tips
FINGER_X = 0.020  # half the push face's width across the push: the fingers widen above the pads
OVERSHOOT = 0.006  # commanded past the stop, so the bar still gets there if the arm falls a little short
BAR_END_OPEN_Y = CRADLE_XY[1] + BAR_OPEN_Y - BAR[1] / 2  # the open bar's free end, 1 cm left of the midline
BAR_END_ACROSS_Y = BAR_END_OPEN_Y + STRAP_TRAVEL
TAB_END_OPEN_Y = BAR_END_OPEN_Y - TAB[1]  # the tab's free end, 4 cm right of the midline
TAB_END_ACROSS_Y = TAB_END_OPEN_Y + STRAP_TRAVEL
PUSH_START_Y = TAB_END_OPEN_Y - 0.030
PUSH_END_Y = TAB_END_ACROSS_Y - FINGER_Y + OVERSHOOT
PARK_Y = -0.10  # back on the right arm's own side
GRIP_Z = CAP_Z + PAD_TIP_PAST_TCP  # pad tips at the cap's mid-height, 1 cm above the wider cylinder's top
TRAVEL_Z = TOP + 0.185
PLACE_Z = TOP + TRAY_H + 0.002 + PAD_TIP_PAST_TCP  # pad tips 2 mm over the tray's rim; the cap drops 1.2 cm
TRIM = {"push": (0.0, 0.0), "pick": (0.0, 0.0), "place": (0.0, 0.0)}


def _right_plan() -> list[Waypoint]:
    tx, ty = TRIM["push"]
    x, start, end = PUSH_X + tx, PUSH_START_Y + ty, PUSH_END_Y + ty
    return [
        Waypoint(1.5, (x, start, HOVER_Z), FINGER_CLOSED, "over the tab's free end"),
        Waypoint(1.2, (x, start, PUSH_Z), FINGER_CLOSED, "down behind it"),
        Waypoint(0.3, (x, start, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(3.0, (x, end, PUSH_Z), FINGER_CLOSED, "push the strap across"),
        Waypoint(0.5, (x, end, PUSH_Z), FINGER_CLOSED, "hold against the stop"),
        # Back to its own side at push height before rising: past the midline the arm stays at bar height.
        Waypoint(1.5, (x, start, PUSH_Z), FINGER_CLOSED, "back off along the tab's line"),
        Waypoint(1.2, (x, start, HOVER_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.5, (x, PARK_Y, HOVER_Z), FINGER_CLOSED, "park on its own side"),
    ]


def _left_plan(start_after: float) -> list[Waypoint]:
    px, py = TRIM["pick"]
    qx, qy = TRIM["place"]
    wait = Waypoint(start_after, (*TRAY_XY, HOVER_Z), FINGER_OPEN, "ease over to the tray, clear of the right arm")
    return [wait, *pick_place_waypoints((CRADLE_XY[0] + px, CRADLE_XY[1] + py, GRIP_Z),
                                        (TRAY_XY[0] + qx, TRAY_XY[1] + qy, PLACE_Z), travel_z=TRAVEL_Z, speed=0.7)]


def demo():
    """Right arm: push the strap bar across to its stop by its tab, back off, lift and park. Left arm: ease over to
    the tray meanwhile, then once the right arm has parked, lift the cap off the cylinder and set it in the tray."""
    right = _right_plan()
    return {"left": _left_plan(sum(w.duration for w in right)), "right": right}
