"""On the station table: a benchtop variac (variable autotransformer) that powers a tube furnace (not shown). The
variac is a fixed 12 x 18 x 10 cm dark grey box. Its lid carries a white dial with a black knob in the middle, and a
red-and-black power rocker to the left of the dial. A red pointer on the knob's side sweeps the dial: 0 V with the
pointer straight towards the robots, rising clockwise (seen from above) to 140 V at a three-quarter turn. Black dots mark
every 10 V, larger ones every 20 V. The rocker is off: its red ON end is up.

The task: switch the variac on at 0 V, then bring it up to 60 V, no faster than 10 V per second and never above 65 V.
That is the procedure that protects the furnace's heating element (switching on at a high setting slams full voltage
onto a cold element). The knob turns 270 degrees in all, more than a wrist turns in one grip, so it takes a regrip.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BODY = (0.32, 0.38, 0.40)
GREY = (0.22, 0.23, 0.25)
WHITE = (0.94, 0.94, 0.92)
BLACK = (0.06, 0.06, 0.06)
RED = (0.85, 0.12, 0.12)

# -- the variac -------------------------------------------------------------------------------------------------------
VX, VY = 0.30, -0.03  # variac centre on the table
BOX = (0.12, 0.18, 0.10)
DIAL_Y = -0.03  # dial centre, part frame (the right half of the lid)
SWITCH_Y = 0.055  # rocker pivot, part frame (the left end of the lid)
PLATE_Z = BOX[2] + 0.0005  # dial plate: a 1 mm white disc on the lid
PLATE_R = 0.052
DOT_R = 0.042  # the scale dots sit on this circle
V_MAX = 140.0
SWEEP = 1.5 * math.pi  # 0 V to 140 V is a three-quarter turn, clockwise seen from above
ZERO_DIR = math.pi  # the pointer at 0 V points along -x, towards the robots


def volts_to_angle(v: float) -> float:
    """Knob joint angle (rad, right-handed about +z) for a dial reading: clockwise is negative."""
    return -v / V_MAX * SWEEP


def angle_to_volts(a: float) -> float:
    return -a / SWEEP * V_MAX


def _dot(v, r):
    phi = ZERO_DIR + volts_to_angle(v)
    return ((DOT_R * math.cos(phi), DIAL_Y + DOT_R * math.sin(phi), BOX[2] + 0.0012), r, 0.0004, BLACK)


SCALE = [_dot(v, 0.0024 if v % 20 == 0 else 0.0013) for v in range(0, 150, 10)]

KNOB_Z = BOX[2] + 0.002  # the knob sits 1 mm clear of the dial plate
KNOB = (0.018, 0.025)  # radius, height
# Fluted grip: twelve 5 mm ribs round the knob's edge, which the fingers bite on.
FLUTES = [((KNOB[0] * math.cos(i * math.pi / 6), DIAL_Y + KNOB[0] * math.sin(i * math.pi / 6), KNOB_Z + KNOB[1] / 2),
           0.0025, KNOB[1], BLACK) for i in range(12)]
KNOB_JOINT = Joint(
    "knob", kind="hinge", axis="Z", anchor=(0.0, DIAL_Y, KNOB_Z),
    lower=volts_to_angle(V_MAX), upper=0.0, initial=0.0,
    stiffness=0.0, damping=0.3, target=0.0,  # a firm knob: viscous drag, so it stays where it is left
    cylinders=[((0.0, DIAL_Y, KNOB_Z + KNOB[1] / 2), KNOB[0], KNOB[1], BLACK), *FLUTES],
    boxes=[
        ((-(KNOB[0] + 0.016) / 2 - 0.0005, DIAL_Y, KNOB_Z + 0.001), (0.016 + KNOB[0], 0.003, 0.0015), RED),  # pointer
        ((-KNOB[0] / 2, DIAL_Y, KNOB_Z + KNOB[1] + 0.0004), (KNOB[0], 0.003, 0.0008), WHITE),  # line on the knob top
    ],
)

# Power rocker: a 2.6 x 1.6 cm paddle on a pivot across the lid (about y), 4 mm above it, red ON half towards +x
# (away from the robots), black OFF half towards the robots. Off: the red end tilted up 0.25 rad. Pressing the red end
# down tips it to +0.25 rad: on. The paddle's mass is centred on the pivot, so it stays where it is pushed.
ROCKER_PIVOT = (0.0, SWITCH_Y, BOX[2] + 0.004)
ROCKER = Joint(
    "power", kind="hinge", axis="Y", anchor=ROCKER_PIVOT, lower=-0.25, upper=0.25, initial=-0.25,
    stiffness=0.0, damping=0.02, target=0.0,
    boxes=[
        ((0.0065, SWITCH_Y, ROCKER_PIVOT[2]), (0.013, 0.016, 0.004), RED),
        ((-0.0065, SWITCH_Y, ROCKER_PIVOT[2]), (0.013, 0.016, 0.004), BLACK),
    ],
)
ON_ANGLE = 0.10  # the rocker counts as on past this (rad)
VARIAC = part(
    "variac", pos=(VX, VY, TOP), fixed=True, friction=1.0,
    boxes=[((0.0, 0.0, BOX[2] / 2), BOX, GREY)],
    cylinders=[((0.0, DIAL_Y, PLATE_Z), PLATE_R, 0.001, WHITE), *SCALE],
    joints=[KNOB_JOINT, ROCKER],
)

OBJECTS = [VARIAC]

# -- scoring: watches the whole trial ---------------------------------------------------------------------------------
TARGET_V = 60.0
LIMIT_V = 65.0  # never above (the scorer allows 1 V for reading the dial)
RAMP_V_PER_S = 10.0  # never faster (the scorer allows 2 V/s, measured over any 1 s)
SWITCH_ON_MAX_V = 5.0  # switch on only at (nearly) 0 V
SUCCESS_HOLD_S = 1.0

_trace: list = []  # (time s, volts) at every control step
_switch = {"on_at_v": None}  # the dial reading when the rocker first went on


@on_reset
def _clear(env):
    _trace.clear()
    _switch["on_at_v"] = None


@on_step
def _record(env):
    import labkit

    base = env.unwrapped
    dt = float(getattr(base, "step_dt", 0.05))
    s = labkit._part_state(env, labkit._SPECS["variac"])
    t = _trace[-1][0] + dt if _trace else 0.0
    v = angle_to_volts(s.joints["knob"])
    _trace.append((t, v))
    if _switch["on_at_v"] is None and s.joints["power"] > ON_ANGLE:
        _switch["on_at_v"] = v


def _violations() -> tuple[float, float]:
    """(highest reading, fastest rise over any 1 s) over the trial so far."""
    if not _trace:
        return 0.0, 0.0
    peak = max(v for _, v in _trace)
    rise, j = 0.0, 0
    for i, (t, v) in enumerate(_trace):
        while _trace[j][0] < t - 1.0:
            j += 1
        rise = max(rise, v - min(w for _, w in _trace[j:i + 1]))
    return peak, rise


def score(objects):
    """0 if the rocker was switched on with the dial above 5 V, if the reading ever went above 66 V or rose more than
    12 V within any 1 s (the limits plus reading margin), or if it ends below 20 V. Otherwise, with the rocker on at the
    end: 1.0 within 3 V of 60 V, 0.5 within 8 V, 0.25 anything else. With the rocker off at the end: 0.25 if the dial is
    within 8 V of 60 V, else 0."""
    v = angle_to_volts(objects["variac"].joints["knob"])
    on = objects["variac"].joints["power"] > ON_ANGLE
    peak, rise = _violations()
    hot_switch = _switch["on_at_v"] is not None and _switch["on_at_v"] > SWITCH_ON_MAX_V
    if hot_switch or peak > LIMIT_V + 1.0 or rise > RAMP_V_PER_S + 2.0 or v < 20.0:
        return 0.0
    err = abs(v - TARGET_V)
    if not on:
        return 0.25 if err <= 8.0 else 0.0
    if err <= 3.0:
        return 1.0
    return 0.5 if err <= 8.0 else 0.25


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """Volts from the target."""
    return abs(angle_to_volts(objects["variac"].joints["knob"]) - TARGET_V)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: presses the rocker's red end down with closed fingers (switching on at 0 V), then parks.
# Right arm: grips the knob straight down, turns it clockwise a quarter turn over 8 s (about 47 V, at most 9 V/s with
# the smoothed motion), lets go, turns the wrist back in the air, grips again and turns the rest of the way over 4 s.
# The regrip loses about 1 V, so the second turn aims 1.5 V past 60 V.
TIP_ABOVE_TCP = 0.013
GRIP_Z = TOP + KNOB_Z + 0.006 + TIP_ABOVE_TCP  # pad tips 5 mm above the knob's underside
HOVER_Z = TOP + KNOB_Z + KNOB[1] + 0.05
FINGER_WIDE = -0.03  # 6 cm opening for the 3.6 cm knob
FINGER_GRIP = -0.004  # closed on the knob
TWIST_START = math.pi / 2  # wrist yaw at each grip (90 deg): both turns stay within the wrist's reach
FIRST = math.radians(-90.0)
AIM_V = TARGET_V + 1.5
SECOND = volts_to_angle(AIM_V) - FIRST
KNOB_XY = (VX, VY + DIAL_Y)
PRESS_XY = (VX + 0.013, VY + SWITCH_Y)  # the red end of the rocker
PRESS_Z = TOP + BOX[2] - 0.001 + TIP_ABOVE_TCP  # pad tips 1 mm into the lid: the rocker bottoms out first
LEFT_PARK = (0.15, 0.25, TOP + 0.14)
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)

PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    ("variac", (VX, VY, TOP + BOX[2] / 2), BOX),
]


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    px, py = PRESS_XY
    left = [
        Waypoint(2.0, (px, py, TOP + BOX[2] + 0.06), FINGER_CLOSED, "above the rocker's red end"),
        Waypoint(1.2, (px, py, PRESS_Z + 0.008), FINGER_CLOSED, "onto it"),
        Waypoint(1.0, (px, py, PRESS_Z), FINGER_CLOSED, "press: on, at 0 V"),
        Waypoint(1.0, (px, py, TOP + BOX[2] + 0.06), FINGER_CLOSED, "up"),
        Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"),
    ]
    t_on = _dur(left)
    k = KNOB_XY
    y0, y1 = TWIST_START, TWIST_START + FIRST
    right = [
        Waypoint(t_on, RIGHT_PARK, FINGER_WIDE, "wait for the power to be on", y0),
        Waypoint(2.0, (*k, HOVER_Z), FINGER_WIDE, "above the knob", y0),
        Waypoint(1.5, (*k, GRIP_Z), FINGER_WIDE, "down round the knob", y0),
        Waypoint(0.8, (*k, GRIP_Z), FINGER_GRIP, "grip", y0),
        Waypoint(8.0, (*k, GRIP_Z), FINGER_GRIP, "turn a quarter turn clockwise, slowly", y1),
        Waypoint(0.8, (*k, GRIP_Z), FINGER_WIDE, "let go", y1),
        Waypoint(1.0, (*k, HOVER_Z), FINGER_WIDE, "up", y1),
        Waypoint(1.5, (*k, HOVER_Z), FINGER_WIDE, "turn the wrist back", y0),
        Waypoint(1.0, (*k, GRIP_Z), FINGER_WIDE, "down", y0),
        Waypoint(0.8, (*k, GRIP_Z), FINGER_GRIP, "grip", y0),
        Waypoint(4.0, (*k, GRIP_Z), FINGER_GRIP, "turn the rest of the way to 60 V", y0 + SECOND),
        Waypoint(0.8, (*k, GRIP_Z), FINGER_WIDE, "let go", y0 + SECOND),
        Waypoint(1.2, (*k, HOVER_Z), FINGER_WIDE, "up", y0 + SECOND),
        Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    t_end = max(_dur(left), _dur(right)) + 2.0
    left.append(Waypoint(t_end - _dur(left), LEFT_PARK, FINGER_OPEN, "hold"))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "hold"))
    return {"left": left, "right": right}
