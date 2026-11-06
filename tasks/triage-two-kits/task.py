"""On the station table: three casualty stand-ins lie side by side across the far half of the table, heads away
from the robot. Each is a grey-blue body block (160 x 80 x 35 mm) with a round head at its far end and a square
triage tag (50 x 50 mm) on its chest. From the robot's left to its right the tags are green, red and yellow. In
front of each casualty, on the robot's side, is a white treatment pad (130 x 110 mm). Two identical navy trauma
kits (100 x 50 x 45 mm boxes) stand on the bare table near the front, just left of the midline, on the green
casualty's side. The casualties, tags and pads are fixed; the two kits are loose.

The goal: triage by tag colour (START: red = immediate, yellow = delayed, green = minor). With two kits for three
casualties, one kit goes on the red casualty's pad and one on the yellow casualty's pad; the green casualty, whose
pad is the nearest to the kits, gets none.
"""

import itertools
import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
REST_GAP = 0.001  # loose objects spawn this far above their support and settle onto it

# -- trauma kits (loose): compact belt-pouch trauma kits on a rigid insert, 150 g ----------------------------------
KIT_SIZE = (0.100, 0.050, 0.045)  # long side along x; the fingers close across the 50 mm side (world y, yaw 0)
KIT_MASS = 0.15
KIT_COLOR = (0.10, 0.17, 0.40)  # navy nylon: not a tag colour, and dark on the white pads
KIT_X = 0.16
KIT_Y = {"kit_outer": 0.18, "kit_inner": 0.07}  # both left of the midline, nearest the green pad
KIT_NAMES = tuple(KIT_Y)

# -- casualties (fixed), x forward from the arms, y to the robot's left ---------------------------------------------
TAG_COLORS = {
    "red": (0.85, 0.10, 0.10),
    "yellow": (0.96, 0.80, 0.10),
    "green": (0.10, 0.62, 0.22),
}
# The fixed tag mix, robot's left to right: green, red, yellow (see the README for why this one).
CASUALTY_Y = {"green": 0.22, "red": 0.0, "yellow": -0.22}
BODY_X0, BODY_X1 = 0.39, 0.55
BODY_W = 0.080
BODY_H = 0.035
BODY_COLOR = (0.42, 0.46, 0.52)  # grey-blue clothing
HEAD_R = 0.030
HEAD_H = 0.035
HEAD_X = BODY_X1 + 0.005 + HEAD_R  # 5 mm gap after the body
HEAD_COLOR = (0.84, 0.69, 0.57)
TAG_SIZE = (0.050, 0.050, 0.004)
TAG_X = 0.47  # on the chest, the upper half of the body block

# -- treatment pads (fixed): white foam mats in front of each casualty ----------------------------------------------
PAD_X0, PAD_X1 = 0.245, 0.375  # 130 mm, 15 mm short of the body
PAD_HALF_W = 0.055  # 110 mm wide
PAD_T = 0.003
PAD_TOP = TOP + PAD_T
PAD_COLOR = (0.95, 0.95, 0.93)
PAD_X = (PAD_X0 + PAD_X1) / 2


def pad_bounds(colour):
    y = CASUALTY_Y[colour]
    return PAD_X0, PAD_X1, y - PAD_HALF_W, y + PAD_HALF_W


def _box(name, x0, x1, y0, y1, z0, z1, color, mass=None):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0),
               color=color, mass=mass)


CASUALTIES = []
for _c, _y in CASUALTY_Y.items():
    CASUALTIES += [
        _box(f"body_{_c}", BODY_X0, BODY_X1, _y - BODY_W / 2, _y + BODY_W / 2, TOP, TOP + BODY_H, BODY_COLOR),
        Cylinder(f"head_{_c}", pos=(HEAD_X, _y, TOP + HEAD_H / 2), radius=HEAD_R, height=HEAD_H, color=HEAD_COLOR),
        _box(f"tag_{_c}", TAG_X - TAG_SIZE[0] / 2, TAG_X + TAG_SIZE[0] / 2, _y - TAG_SIZE[1] / 2, _y + TAG_SIZE[1] / 2,
             TOP + BODY_H, TOP + BODY_H + TAG_SIZE[2], TAG_COLORS[_c]),
        _box(f"pad_{_c}", *pad_bounds(_c), TOP, PAD_TOP, PAD_COLOR),
    ]

KITS = [
    Box(n, pos=(KIT_X, y, TOP + REST_GAP + KIT_SIZE[2] / 2), size=KIT_SIZE, mass=KIT_MASS, color=KIT_COLOR)
    for n, y in KIT_Y.items()
]
OBJECTS = [*CASUALTIES, *KITS]

# -- scoring --------------------------------------------------------------------------------------------------------
ON_PAD_TOL = 0.015  # a kit's lowest point at most 15 mm above the pad top: lying on it, in any orientation
CREDIT = {"red": 0.6, "yellow": 0.4}  # once per pad: a second kit on the same pad adds nothing
GREEN_PENALTY = 0.4  # per kit on the green pad: a kit spent on a minor casualty while a worse one goes without


def pad_of(state):
    """The colour of the pad a kit lies on, or None. On a pad: the kit's centre (seen from above) is over the pad
    and its lowest point is at most 15 mm above the pad top."""
    x, y, _ = (float(v) for v in state.pos)
    if float(state.low[2]) > PAD_TOP + ON_PAD_TOL:
        return None
    for colour in CASUALTY_Y:
        x0, x1, y0, y1 = pad_bounds(colour)
        if x0 <= x <= x1 and y0 <= y <= y1:
            return colour
    return None


def placements(objects) -> tuple:
    """The pad colour (or None) of each kit, in KIT_NAMES order."""
    return tuple(pad_of(objects[n]) for n in KIT_NAMES)


def score(objects):
    """0.6 if any kit is on the red pad, plus 0.4 if any kit is on the yellow pad, minus 0.4 per kit on the green
    pad, never below 0. A kit anywhere else (still at the start, on the table, on a casualty, held, on the floor)
    earns nothing."""
    where = placements(objects)
    total = sum(v for c, v in CREDIT.items() if c in where) - GREEN_PENALTY * where.count("green")
    return max(0.0, round(total, 9))


def success(objects):
    """One kit on the red pad, the other on the yellow pad, both still."""
    return sorted(placements(objects), key=str) == ["red", "yellow"] and all(objects[n].resting() for n in KIT_NAMES)


def _gap(value, lo, hi) -> float:
    return max(lo - value, 0.0, value - hi)


def distance(objects):
    """Horizontal distance (m) from the kits to the red and yellow pads' footprints, summed, under the better of the
    two ways to pair kits with pads."""

    def d(name, colour):
        x0, x1, y0, y1 = pad_bounds(colour)
        p = objects[name].pos
        return math.hypot(_gap(float(p[0]), x0, x1), _gap(float(p[1]), y0, y1))

    return min(d(a, "red") + d(b, "yellow") for a, b in itertools.permutations(KIT_NAMES))


# -- demo -----------------------------------------------------------------------------------------------------------
GRIP_ABOVE_BOTTOM = 0.028  # TCP 28 mm above a kit's bottom: pads from 15 to 42 mm on the 45 mm kit
RELEASE_ABOVE = 0.003  # set a kit down 3 mm above the pad
GRIP_SHIFT_Y = 0.006  # pads hold an object 12-20 mm toward -y (seen in sim); aim the place this much to +y
TRAVEL_Z = TOP + 0.11  # carried kit's bottom 82 mm over the table, 43 mm over the casualties' tags
PICK_Z = TOP + REST_GAP + GRIP_ABOVE_BOTTOM
PLACE_Z = PAD_TOP + RELEASE_ABOVE + GRIP_ABOVE_BOTTOM
DEMO_ROUTE = {"left": ("kit_outer", "red"), "right": ("kit_inner", "yellow")}
WAIT_XY = (0.14, -0.38)  # the right arm waits here, out past the yellow casualty, while the left arm works
PARK_XY = {"left": (0.13, 0.42), "right": (0.13, -0.42)}  # outboard of everything scored, for the last frame
PARK_S = 1.5
HOLD_S = 2.0


def _deliver(side):
    kit, colour = DEMO_ROUTE[side]
    pick = (KIT_X, KIT_Y[kit], PICK_Z)
    place = (PAD_X, CASUALTY_Y[colour] + GRIP_SHIFT_Y, PLACE_Z)
    return pick_place_waypoints(pick, place, travel_z=TRAVEL_Z, yaw=0.0)


def demo():
    """Left arm: the outer kit onto the red (centre) pad, then it parks outboard. Meanwhile the right arm waits
    outboard of the yellow casualty; once the left arm has parked it reaches just past the midline for the inner kit,
    takes it onto the yellow pad and parks. The arms never work near the midline at the same time. Both then hold still for 2 s. Each release is
    followed by a straight 7.6 cm rise (the retreat) before any sideways move."""
    left = _deliver("left")
    left.append(Waypoint(PARK_S, (*PARK_XY["left"], TRAVEL_Z), FINGER_OPEN, "park"))
    left_done = sum(w.duration for w in left)
    right = [
        Waypoint(1.5, (*WAIT_XY, TRAVEL_Z), FINGER_OPEN, "to wait"),
        Waypoint(left_done - 1.5, (*WAIT_XY, TRAVEL_Z), FINGER_OPEN, "wait for the left arm"),
        *_deliver("right"),
        Waypoint(PARK_S, (*PARK_XY["right"], TRAVEL_Z), FINGER_OPEN, "park"),
        Waypoint(HOLD_S, (*PARK_XY["right"], TRAVEL_Z), FINGER_OPEN, "hold still"),
    ]
    end = sum(w.duration for w in right)
    left.append(Waypoint(end - left_done, (*PARK_XY["left"], TRAVEL_Z), FINGER_OPEN, "hold still"))
    return {"left": left, "right": right}
