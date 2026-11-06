"""Install the correct split guide around the connected black cable.

The two U-shaped inserts differ in the width of the opening. Lower one onto
the cable at the grey receiver. Slide the orange keeper toward the receiver
until it reaches the green mark. Withdraw, then press the yellow release.
The blue sleeve must pass through the guide as the drawer opens by itself.
Leave the two plugs connected throughout.
"""

import math
from dataclasses import dataclass
from itertools import pairwise

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X, Z = 0.255, TOP + 0.055
TRAVEL = 0.050
SLEEVE_Y = -0.024
SLEEVE_LENGTH = 0.012
GUIDE_DEPTH = 0.022
START_LIMIT = 0.002
PASS_TRAVEL = 0.048
PROOF_SECONDS = 3.0
HOLD_SECONDS = 2.0
SUCCESS_HOLD_S = 0.05
GUIDES = ("wide_guide", "narrow_guide")
DRAWER_POS = (X, 0.153, TOP + 0.020)
CABLE_POS = (X, 0.0, Z)
RELEASE_POS = (0.245, -0.311, TOP + 0.059)
KEEPER_POS = (X - 0.020, -0.053, Z + 0.020)
KEEPER_TRAVEL = 0.053
INSTRUCTION = (
    "Fit a split guide around the cable at the panel opening and lock it with the orange keeper. "
    "Keep both plugs seated throughout. Withdraw both hands, then press the yellow release and "
    "let the drawer open to the marked line on its own."
)

GREY = (0.55, 0.60, 0.63)
DARK = (0.075, 0.08, 0.09)
GREEN = (0.10, 0.69, 0.32)
ORANGE = (0.93, 0.35, 0.09)
YELLOW = (0.99, 0.78, 0.14)
BLUE = (0.07, 0.38, 0.87)
WHITE = (0.89, 0.88, 0.81)


@dataclass(frozen=True)
class Solid:
    name: str
    pos: tuple
    size: tuple
    color: tuple
    cylinder: bool = False


@dataclass(frozen=True)
class Joint:
    kind: str
    anchor: tuple
    parent: str | None = None
    axis: str = "Y"
    limits: tuple = (0.0, 0.0)
    spring: tuple | None = None


OBJECTS = []
SHAPES = {}
JOINTS = {}


def box(name, pos, size, color=GREY):
    return Solid(name, pos, size, color)


def tube(name, pos, radius, length, color):
    return Solid(name, pos, (radius, length), color, cylinder=True)


def body(name, pos, parts, *, mass=None, yaw=0.0, joints=()):
    OBJECTS.append(
        Box(
            name,
            tuple(map(float, pos)),
            color=parts[0].color,
            mass=mass,
            yaw=yaw,
            size=(0.01,) * 3,
        )
    )
    SHAPES[name] = tuple(parts)
    JOINTS[name] = tuple(joints)


def fixture(name, pos, size, color=GREY):
    body(name, pos, [box("block", (0, 0, 0), size, color)])


# The cabinet receiver closes the open side of the U insert; the keeper traps
# the left shoulder vertically. Its rails and ledges are separate colliders.
fixture("receiver_sill", (X, 0, Z - 0.015), (0.061, 0.033, 0.010))
for side in (-1, 1):
    label = "near" if side < 0 else "far"
    fixture(
        f"receiver_side_{label}",
        (X + side * 0.0275, 0, Z - 0.003),
        (0.009, 0.034, 0.023),
    )
    for end in (-1, 1):
        fixture(
            f"receiver_end_{label}_{end + 1}",
            (X + side * 0.018, end * 0.015, Z - 0.004),
            (0.010, 0.004, 0.018),
        )
fixture("panel_web", (X + 0.058, 0, TOP + 0.026), (0.066, 0.010, 0.052))
fixture("receiver_foot", (X, 0, TOP + 0.020), (0.061, 0.035, 0.040))
fixture("keeper_mark", (X - 0.035, 0, Z + 0.010), (0.008, 0.009, 0.006), GREEN)

GUIDE_START = {
    "wide_guide": (0.247, 0.285, TOP + 0.010),
    "narrow_guide": (0.365, 0.205, TOP + 0.010),
}
BORES = {"wide_guide": 0.016, "narrow_guide": 0.010}
for name, gap in BORES.items():
    width = (0.044 - gap) / 2
    side_x = (gap + width) / 2
    body(
        name,
        GUIDE_START[name],
        [
            box("left_cheek", (-side_x, 0, -0.001), (width, GUIDE_DEPTH, 0.018), WHITE),
            box("right_cheek", (side_x, 0, -0.001), (width, GUIDE_DEPTH, 0.018), WHITE),
            box("bridge", (0, 0, 0.012), (0.044, GUIDE_DEPTH, 0.008), WHITE),
            box(
                "handle",
                (0, 0, 0.030),
                (0.024, 0.018, 0.028),
                BLUE if name == "wide_guide" else DARK,
            ),
        ],
        mass=0.036,
    )

body(
    "keeper",
    KEEPER_POS,
    [
        box("bar", (0, 0, 0), (0.012, 0.022, 0.007), ORANGE),
        box("handle", (-0.011, 0, 0.013), (0.026, 0.018, 0.022), ORANGE),
    ],
    mass=0.035,
    joints=(Joint("slide", KEEPER_POS, limits=(0.0, KEEPER_TRAVEL)),),
)
body(
    "release",
    RELEASE_POS,
    [box("button", (0, 0, 0), (0.052, 0.045, 0.011), YELLOW)],
    mass=0.025,
    joints=(
        Joint(
            "slide",
            RELEASE_POS,
            axis="Z",
            limits=(-0.010, 0.0),
            spring=(480.0, 7.0, 12.0),
        ),
    ),
)
fixture(
    "release_post", (RELEASE_POS[0], RELEASE_POS[1], TOP + 0.020), (0.038, 0.038, 0.040)
)

body(
    "drawer",
    DRAWER_POS,
    [
        box("base", (0, 0, 0), (0.065, 0.055, 0.027), GREY),
        box("socket", (0, -0.017, Z - DRAWER_POS[2]), (0.027, 0.020, 0.021), GREY),
        box("pointer", (0.035, 0.020, 0.010), (0.009, 0.007, 0.008), WHITE),
    ],
    mass=0.140,
    joints=(Joint("slide", DRAWER_POS, limits=(0.0, TRAVEL)),),
)
fixture("drawer_rail", (X, 0.174, TOP + 0.003), (0.069, 0.113, 0.006), DARK)
fixture(
    "travel_mark",
    (X + 0.039, DRAWER_POS[1] + TRAVEL + 0.020, TOP + 0.015),
    (0.013, 0.005, 0.005),
    GREEN,
)

FIXED_PLUG_POS = (0.127, -0.080, Z)
MOVING_PLUG_POS = (X, 0.117, Z)
fixture(
    "fixed_socket_post", (FIXED_PLUG_POS[0], -0.055, TOP + 0.027), (0.033, 0.030, 0.054)
)
fixture("fixed_socket", (FIXED_PLUG_POS[0], -0.059, Z), (0.027, 0.025, 0.021))
for name, pos, parent in (
    ("fixed_plug", FIXED_PLUG_POS, None),
    ("moving_plug", MOVING_PLUG_POS, "drawer"),
):
    body(
        name,
        pos,
        [
            box("plug", (0, 0, 0), (0.023, 0.015, 0.018), DARK),
            box("witness", (0, 0.005, 0.009), (0.021, 0.002, 0.001), ORANGE),
        ],
        mass=0.021,
        joints=(
            Joint(
                "slide",
                pos,
                parent,
                limits=(-0.040, 0.002),
                spring=(1800.0, 12.0, 10.0),
            ),
        ),
    )

# Only the short straight segment that runs through the guide is rigid. The
# loop supplies slack through a series of freely rotating physical links.
body(
    "cable_run",
    CABLE_POS,
    [
        tube("jacket", (0, 0.012, 0), 0.004, 0.194, DARK),
        tube("sleeve", (0, SLEEVE_Y, 0), 0.006, SLEEVE_LENGTH, BLUE),
    ],
    mass=0.020,
    joints=(Joint("fixed", (X, 0.109, Z), "moving_plug"),),
)
loop_radius = (X - FIXED_PLUG_POS[0]) / 2
arc_x = X - loop_radius
arc_y = -0.162
points = [(FIXED_PLUG_POS[0], -0.087, Z), (FIXED_PLUG_POS[0], -0.124, Z)]
points += [
    (arc_x + loop_radius * math.cos(a), arc_y - loop_radius * math.sin(a), Z)
    for a in np.linspace(math.pi, 0, 9)
]
points += [(X, -0.123, Z), (X, -0.085, Z)]
LINKS = []
for i, (a, b) in enumerate(pairwise(points)):
    name = f"loop_{i}"
    delta = np.subtract(b, a)
    yaw = math.atan2(-delta[0], delta[1])
    joints = [Joint("ball", a, LINKS[-1] if LINKS else "fixed_plug")]
    if i == len(points) - 2:
        joints.append(Joint("ball", b, "cable_run"))
    body(
        name,
        tuple((np.array(a) + b) / 2),
        [tube("jacket", (0, 0, 0), 0.004, float(np.linalg.norm(delta)) + 0.0004, DARK)],
        mass=0.003,
        yaw=yaw,
        joints=joints,
    )
    LINKS.append(name)
fixture("loop_tray", (0.192, -0.158, Z - 0.009), (0.161, 0.158, 0.010), GREY)
fixture("tray_pedestal", (0.192, -0.158, TOP + 0.020), (0.158, 0.155, 0.040), GREY)


@dataclass(frozen=True)
class Sample:
    guide: str | None = None
    keeper_closed: bool = False
    hands_clear: bool = False
    press: bool = False
    travel: float = 0.0
    sleeve_clear: bool = False
    plug_gap: float = 0.0
    damaged: bool = False
    assisted: bool = False


class Trial:
    def __init__(self):
        self.start = None
        self.eligible_guide = None
        self.reached = None
        self.invalid = False
        self.score = 0.0

    def step(self, t, sample):
        installed = sample.guide is not None and sample.keeper_closed
        if self.start is None and sample.press:
            self.start = t
            if installed and sample.hands_clear and abs(sample.travel) <= START_LIMIT:
                self.eligible_guide = sample.guide
        proving = self.start is not None
        lost_guide = self.eligible_guide is not None and (
            not installed or sample.guide != self.eligible_guide
        )
        self.invalid |= bool(
            sample.damaged
            or sample.plug_gap > 0.002
            or lost_guide
            or (proving and sample.assisted)
        )
        passed = (
            proving
            and installed
            and sample.guide == self.eligible_guide
            and sample.hands_clear
            and sample.travel >= PASS_TRAVEL
            and sample.sleeve_clear
        )
        if passed:
            if self.reached is None and t - self.start <= PROOF_SECONDS + 1e-9:
                self.reached = t
        else:
            self.reached = None
        if self.invalid or not (installed and sample.hands_clear):
            self.score = 0.0
        elif self.reached is not None and t - self.reached >= HOLD_SECONDS - 1e-9:
            self.score = 1.0
        else:
            self.score = 0.5
        return self.score


TRIAL = Trial()


def score(objects):
    return TRIAL.score


def success(objects):
    return TRIAL.score == 1.0


def demo(guide="wide_guide"):
    """Both arms use normal joint targets; pass the narrow guide for the control."""
    hover = TOP + 0.143
    px, py, pz = GUIDE_START[guide]
    grip_z = 0.028 + PAD_TIP_PAST_TCP
    left = pick_place_waypoints(
        (px, py, pz + grip_z), (X, 0, Z + grip_z), travel_z=hover
    )
    left.append(Waypoint(2.0, (0.23, 0.29, hover), FINGER_OPEN, "withdraw from drawer"))
    kx, ky, kz = KEEPER_POS
    kx -= 0.011
    kz += 0.013 + PAD_TIP_PAST_TCP
    bx, by, bz = RELEASE_POS
    bz += 0.0055 + PAD_TIP_PAST_TCP
    right = [
        Waypoint(
            sum(w.duration for w in left),
            (kx, ky - 0.022, hover),
            FINGER_OPEN,
            "wait for insert",
        ),
        Waypoint(1.0, (kx, ky, hover), FINGER_OPEN, "above keeper"),
        Waypoint(1.3, (kx, ky, kz), FINGER_OPEN, "approach keeper"),
        Waypoint(0.8, (kx, ky, kz), FINGER_CLOSED, "grasp keeper"),
        Waypoint(2.2, (kx, 0.001, kz), FINGER_CLOSED, "close keeper"),
        Waypoint(0.8, (kx, 0.001, kz), FINGER_OPEN, "release keeper"),
        Waypoint(1.1, (kx, 0.001, hover), FINGER_OPEN, "lift clear"),
        Waypoint(1.0, (bx, by, hover), FINGER_CLOSED, "above release"),
        Waypoint(1.4, (bx, by, bz - 0.019), FINGER_CLOSED, "press release"),
        Waypoint(0.7, (bx, by, hover), FINGER_OPEN, "withdraw"),
        Waypoint(5.0, (bx, by, hover), FINGER_OPEN, "observe travel"),
    ]
    return {"left": left, "right": right}
