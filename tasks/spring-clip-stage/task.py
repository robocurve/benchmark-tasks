"""On the station table: an inspection stage (a 10 x 10 cm aluminium block, 2 cm tall, with a 4 x 4 cm dark target
square in the middle of its top) and a steel spring clip that holds samples down on it. The clip pivots on a post just
off the stage's near edge: its jaw reaches over the target square's near edge, and a rubber thumb tab sticks out on
the other side of the pivot, over the table. A torsion spring holds the jaw down; pressing the tab lifts the jaw.
A chip carrier (a 36 x 36 x 8 mm aluminium plate with a 10 x 10 mm silicon chip on top and a red pin-1 dot at one
corner) lies on the table to the left, a quarter turn away from the orientation the stage's red corner mark asks for.

The task: clamp the carrier under the clip, centred on the target square, pin 1 at the red mark. One hand has to hold the clip open while
the other slides the carrier in underneath; the jaw then comes down on the carrier's margin, clear of the chip.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

ALU = (0.72, 0.73, 0.75)
DARK = (0.12, 0.12, 0.14)
STEEL = (0.80, 0.81, 0.83)
RUBBER = (0.10, 0.10, 0.10)
SILICON = (0.22, 0.25, 0.38)
RED = (0.85, 0.12, 0.12)

# -- the stage and its clip -------------------------------------------------------------------------------------------
# Part frame: origin at the stage's centre, on the table top. The clip lies along the part's x axis with its pivot at
# -x: the thumb tab points back towards the robots, where the right arm presses it, and the jaw reaches in from the
# near side, so the left arm works on the far side of the stage and the two wrists stay apart.
SX, SY = 0.30, -0.04  # stage centre on the table
STAGE_YAW = 0.0
STAGE = 0.10  # stage block 10 x 10 cm
STAGE_H = 0.020  # stage top 2 cm above the table
TARGET = 0.040  # dark target square, 4 x 4 cm, flush with the stage top
ST = TOP + STAGE_H  # stage top height (world)

PIVOT_X = -0.075  # pivot 2.5 cm off the stage's near edge
PIVOT_Z = STAGE_H + 0.002  # pivot height above the table; the 3 mm bar then clears the stage top by 0.5 mm
BAR_T = 0.003
JAW_TIP_X = -0.010  # the jaw reaches 1 cm short of the stage centre
TAB = (0.060, 0.030)  # thumb tab: 6 cm from the pivot outwards, 3 cm wide

_ring = (TARGET + (STAGE - TARGET) / 2) / 2  # centre offset of the four blocks around the target square
_side = (STAGE - TARGET) / 2
STAGE_BOXES = [
    ((0.0, 0.0, STAGE_H / 2), (TARGET, TARGET, STAGE_H), DARK),
    ((_ring, 0.0, STAGE_H / 2), (_side, STAGE, STAGE_H), ALU),
    ((-_ring, 0.0, STAGE_H / 2), (_side, STAGE, STAGE_H), ALU),
    ((0.0, _ring, STAGE_H / 2), (TARGET, _side, STAGE_H), ALU),
    ((0.0, -_ring, STAGE_H / 2), (TARGET, _side, STAGE_H), ALU),
    ((PIVOT_X, 0.0, (PIVOT_Z - 0.0035) / 2), (0.012, 0.020, PIVOT_Z - 0.0035), STEEL),  # pivot post
    # pin-1 mark: a red 5 mm sticker (0.4 mm thick) beside the target square's far left corner
    ((0.026, 0.026, STAGE_H + 0.0002), (0.005, 0.005, 0.0004), RED),
]

CLIP = Joint(
    "clip", kind="hinge", axis="Y", anchor=(PIVOT_X, 0.0, PIVOT_Z),
    # angle 0: jaw resting on the bare stage; negative angles lift the jaw (and push the tab down)
    lower=-0.35, upper=0.0, initial=0.0,
    stiffness=1.0, damping=0.01, target=0.0,  # torsion spring: 0.13 N m on an 8 mm carrier, 0.35 N m held open
    boxes=[
        (((PIVOT_X + JAW_TIP_X) / 2, 0.0, PIVOT_Z), (JAW_TIP_X - PIVOT_X, 0.010, BAR_T), STEEL),  # jaw
        ((PIVOT_X - TAB[0] / 2, 0.0, PIVOT_Z), (TAB[0], TAB[1], BAR_T), RUBBER),  # thumb tab
    ],
)

STAGE_PART = part("stage", pos=(SX, SY, TOP), fixed=True, yaw=STAGE_YAW, boxes=STAGE_BOXES, joints=[CLIP])

# -- the chip carrier -------------------------------------------------------------------------------------------------
CARRIER = 0.036
CARRIER_H = 0.008
CHIP = (0.010, 0.010, 0.0007)
PIN1 = (0.0125, 0.0125)  # the carrier's red pin-1 dot, in the carrier frame (its +x +y corner)
CARRIER_START = (0.20, 0.17)
CARRIER_START_YAW = -math.pi / 2  # turned a quarter turn clockwise: the dot starts at the far right corner
CARRIER_PART = part(
    "carrier", pos=(*CARRIER_START, TOP + CARRIER_H / 2), mass=0.03, friction=1.0, yaw=CARRIER_START_YAW,
    boxes=[
        ((0.0, 0.0, 0.0), (CARRIER, CARRIER, CARRIER_H), ALU),
        ((0.0, 0.0, CARRIER_H / 2 + CHIP[2] / 2), CHIP, SILICON),
        ((*PIN1, CARRIER_H / 2 + 0.00015), (0.004, 0.004, 0.0003), RED),  # pin-1 dot
    ],
)

OBJECTS = [STAGE_PART, CARRIER_PART]

# -- scoring ----------------------------------------------------------------------------------------------------------
CENTRE_TOL = 0.003  # m, carrier centre to target-square centre
YAW_TOL_DEG = 10.0  # the carrier's yaw from the pin-1 orientation (dot at the stage's red mark), full turn
FLAT_TOL_DEG = 5.0
# The jaw rests on an 8 mm carrier at about -0.13 rad; on the bare stage at 0; held fully open, -0.35.
CLAMPED = (-0.20, -0.08)
SUCCESS_HOLD_S = 1.0


def _yaw_err_deg(state) -> float:
    """Degrees between the carrier's yaw and pin-1 orientation (yaw 0: its dot at the stage's red mark corner)."""
    r = state.rotation()
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    return abs((yaw + 180.0) % 360.0 - 180.0)


def _on_stage(c) -> bool:
    dx, dy = c.pos[0] - SX, c.pos[1] - SY
    resting_on_top = abs(c.pos[2] - (ST + CARRIER_H / 2)) < 0.002
    return abs(dx) < STAGE / 2 and abs(dy) < STAGE / 2 and resting_on_top and c.tilt_deg() < FLAT_TOL_DEG


def _centred(c) -> bool:
    return math.hypot(c.pos[0] - SX, c.pos[1] - SY) <= CENTRE_TOL and _yaw_err_deg(c) <= YAW_TOL_DEG


def _clamped(objects) -> bool:
    lo, hi = CLAMPED
    return lo <= objects["stage"].joints["clip"] <= hi


def score(objects):
    """0.25: the carrier lies flat (chip up, within 5 degrees) on the stage top. 0.5: also centred on the target square
    within 3 mm and in pin-1 orientation (its red dot at the corner by the stage's red mark, within 10 degrees). 1.0: also
    clamped: the clip has been let go and its jaw rests on the carrier (clip angle -0.20 to -0.08 rad). Otherwise 0."""
    c = objects["carrier"]
    if not _on_stage(c):
        return 0.0
    if not _centred(c):
        return 0.25
    if not _clamped(objects):
        return 0.5
    return 1.0


def success(objects):
    return score(objects) == 1.0 and objects["carrier"].resting()


def distance(objects):
    """Horizontal distance (m) from the carrier to the target square's centre."""
    c = objects["carrier"]
    return float(math.hypot(c.pos[0] - SX, c.pos[1] - SY))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Right arm: closed fingers press the thumb tab down about 1.7 cm, to its stop (the jaw tip lifts about 2.2 cm) and hold it.
# Left arm: picks the carrier by its sides, turns it a quarter turn on the way (pin 1 to the red mark), sets it on the
# stage 1.6 cm beyond the target, slides it in under the
# lifted jaw to the target, lets go and leaves. Then the right arm lets the tab up and the spring closes the jaw.
TIP_ABOVE_TCP = 0.013  # the TCP is 13 mm above the pad tips
TRAVEL_Z = TOP + 0.10
FINGER_WIDE = -0.03  # 6 cm opening for the 3.6 cm carrier

_AX = (math.cos(STAGE_YAW), math.sin(STAGE_YAW))  # the part's +x in the world


def _along(d):
    """World (x, y) of the point ``d`` metres along the clip's axis from the stage centre."""
    return (SX + d * _AX[0], SY + d * _AX[1])


TAB_PRESS_XY = _along(PIVOT_X - 0.050)  # 5 cm out from the pivot
TAB_TOP = TOP + PIVOT_Z + BAR_T / 2
PRESS_DEPTH = 0.020
PRESS_YAW = 0.0

CARRIER_GRIP_Z = TIP_ABOVE_TCP + 0.0015  # pad tips 1.5 mm above the carrier's underside
SLIDE_FROM = _along(0.016)  # set down 1.6 cm beyond the target, then slid in towards the jaw
GRIP_YAW = STAGE_YAW  # the fingers close across the clip's axis, on the carrier's sides, clear of the jaw
PICK_YAW = GRIP_YAW + CARRIER_START_YAW  # turning from this to GRIP_YAW turns the carrier into pin-1 orientation
VIA = (0.34, 0.06)  # the left arm comes round the far side, away from the right arm's wrist


# For the offline plan check (common/plancheck.py): world-frame boxes the arms must not hit.
PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    ("stage", (SX, SY, TOP + STAGE_H / 2), (STAGE, STAGE, STAGE_H)),
    ("post", (*_along(PIVOT_X), TOP + PIVOT_Z / 2), (0.016, 0.016, PIVOT_Z)),
]


def _duration(wps):
    return sum(w.duration for w in wps)


def _right_plan(t_release: float):
    x, y = TAB_PRESS_XY
    top = TAB_TOP + TIP_ABOVE_TCP
    wps = [
        Waypoint(2.0, (x, y, TOP + 0.08), FINGER_CLOSED, "above the tab", PRESS_YAW),
        Waypoint(1.5, (x, y, top + 0.003), FINGER_CLOSED, "onto the tab", PRESS_YAW),
        Waypoint(2.0, (x, y, top - PRESS_DEPTH), FINGER_CLOSED, "press: jaw opens", PRESS_YAW),
    ]
    hold = t_release - _duration(wps)
    wps.append(Waypoint(hold, (x, y, top - PRESS_DEPTH), FINGER_CLOSED, "hold the clip open", PRESS_YAW))
    wps += [
        Waypoint(2.0, (x, y, top + 0.004), FINGER_CLOSED, "let the tab up: jaw closes", PRESS_YAW),
        Waypoint(1.5, (x, y, TOP + 0.08), FINGER_CLOSED, "away", PRESS_YAW),
        Waypoint(2.0, (0.15, -0.25, TOP + 0.10), FINGER_OPEN, "park"),
    ]
    return wps


def _left_plan(t_start: float):
    sx, sy = CARRIER_START
    y = GRIP_YAW
    pick_z = TOP + CARRIER_GRIP_Z
    on_stage = ST + CARRIER_GRIP_Z + 0.001  # carrier 1 mm above the stage top while it slides
    return [
        Waypoint(t_start, (sx, sy, TRAVEL_Z), FINGER_WIDE, "above the carrier", PICK_YAW),
        Waypoint(1.5, (sx, sy, pick_z), FINGER_WIDE, "descend", PICK_YAW),
        Waypoint(0.8, (sx, sy, pick_z), FINGER_CLOSED, "grip the carrier's sides", PICK_YAW),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift", PICK_YAW),
        Waypoint(2.0, (*VIA, TRAVEL_Z), FINGER_CLOSED, "round the far side, turning pin 1 to the mark", y),
        Waypoint(1.5, (*SLIDE_FROM, TRAVEL_Z), FINGER_CLOSED, "above the stage", y),
        Waypoint(1.5, (*SLIDE_FROM, on_stage), FINGER_CLOSED, "lower beside the jaw", y),
        Waypoint(2.5, (SX, SY, on_stage), FINGER_CLOSED, "slide in under the jaw", y),
        Waypoint(0.8, (SX, SY, on_stage - 0.001), FINGER_CLOSED, "set down", y),
        Waypoint(0.8, (SX, SY, on_stage - 0.001), FINGER_WIDE, "let go", y),
        Waypoint(1.2, (SX, SY, TRAVEL_Z), FINGER_WIDE, "up", y),
        Waypoint(1.5, (*VIA, TRAVEL_Z), FINGER_WIDE, "back round the far side", y),
        Waypoint(1.5, (0.15, 0.25, TRAVEL_Z), FINGER_OPEN, "park"),
    ]


def demo():
    left = _left_plan(t_start=1.5)
    t_left_clear = _duration(left)  # the left arm is parked, out of the right arm's way
    right = _right_plan(t_release=t_left_clear + 0.3)
    t_end = max(_duration(left), _duration(right)) + 2.0
    left.append(Waypoint(t_end - _duration(left), left[-1].pos, FINGER_OPEN, "hold"))
    right.append(Waypoint(t_end - _duration(right), right[-1].pos, FINGER_OPEN, "hold"))
    return {"left": left, "right": right}
