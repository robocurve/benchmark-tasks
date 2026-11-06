"""On the station table, centred at x 0.29 m, y 0: an open corrugated shipping carton (a regular slotted carton,
kraft board, 9 x 7 x 2 inch class), 180 mm front to back, 220 mm wide and 50 mm tall, fixed to the table. Its four
top flaps all stand open, leaning 10 degrees outward. The two end flaps (minor flaps, 172 x 90 x 3 mm, 20 g) are hinged
on the left and right walls and spring back open when let go, as new board does. The two long flaps (major flaps,
220 x 89.5 x 3 mm, 60 g, darker board) are hinged on the front and back walls; they stay where they are put and meet
in the middle when closed. Nothing else is on the table.

The goal: close the carton so it can be taped: both end flaps folded down flat and held under the two long flaps,
both long flaps folded down flat on top of them.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robocurve_ext as rx  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- carton body (fixed), kraft ----------------------------------------------------------------------------------------
CX, CY = 0.29, 0.0
LX, LY, H, WALL = 0.180, 0.220, 0.050, 0.004
X0, X1 = CX - LX / 2, CX + LX / 2  # front and back outer faces, 0.200 and 0.380
Y0, Y1 = CY - LY / 2, CY + LY / 2  # right and left outer faces, -0.110 and +0.110
WALL_TOP = TOP + H
KRAFT = (0.62, 0.48, 0.32)
BODY = [
    Box("carton_floor", pos=(CX, CY, TOP + WALL / 2), size=(LX, LY, WALL), color=KRAFT),
    Box("carton_front", pos=(X0 + WALL / 2, CY, TOP + H / 2), size=(WALL, LY, H), color=KRAFT),
    Box("carton_back", pos=(X1 - WALL / 2, CY, TOP + H / 2), size=(WALL, LY, H), color=KRAFT),
    Box("carton_right", pos=(CX, Y0 + WALL / 2, TOP + H / 2), size=(LX - 2 * WALL, WALL, H), color=KRAFT),
    Box("carton_left", pos=(CX, Y1 - WALL / 2, TOP + H / 2), size=(LX - 2 * WALL, WALL, H), color=KRAFT),
]

# -- flaps (loose boxes on hinges); every flap's zero pose is closed -----------------------------------------------------
T = 0.003  # board thickness
GAP = 0.0005
FLAP_D = 0.090  # every flap's depth from its crease (half the carton's 180 mm width, as on a real slotted carton)
MINOR = (LX - 2 * WALL, FLAP_D, T)
MAJOR = (LY, FLAP_D - 0.0005, T)  # the two majors meet in the middle with a 1 mm gap
MINOR_MASS, MAJOR_MASS = 0.020, 0.060
MINOR_COLOR, MAJOR_COLOR = (0.66, 0.52, 0.36), (0.55, 0.40, 0.26)
MINOR_Z = WALL_TOP + GAP + T / 2  # closed minors lie on the side walls
MAJOR_Z = MINOR_Z + T / 2 + GAP + T / 2  # closed majors lie on the closed minors
OPEN = 100.0  # every flap starts standing, leaning 10 deg outward on its stop
# A minor's spring (target: open) beats its own weight at every angle, so a released minor springs back open;
# a closed major's weight on it beats the spring, so a major holds the minors down (numbers in the README).
MINOR_K, DAMP = 1.3e-4, 3e-5  # N*m/deg, N*m*s/deg

FLAPS = [
    # the minors are listed first; each major's zero pose lies on them
    Box("minor_left", pos=(CX, Y1 - FLAP_D / 2, MINOR_Z), size=MINOR, mass=MINOR_MASS, color=MINOR_COLOR),
    Box("minor_right", pos=(CX, Y0 + FLAP_D / 2, MINOR_Z), size=MINOR, mass=MINOR_MASS, color=MINOR_COLOR),
    Box("major_near", pos=(X0 + MAJOR[1] / 2, CY, MAJOR_Z), size=(MAJOR[1], MAJOR[0], T), mass=MAJOR_MASS,
        color=MAJOR_COLOR),
    Box("major_far", pos=(X1 - MAJOR[1] / 2, CY, MAJOR_Z), size=(MAJOR[1], MAJOR[0], T), mass=MAJOR_MASS,
        color=MAJOR_COLOR),
]
OBJECTS = [*BODY, *FLAPS]
FLAP_NAMES = ("minor_left", "minor_right", "major_near", "major_far")
MINORS, MAJORS = FLAP_NAMES[:2], FLAP_NAMES[2:]

# Hinges on each wall's top outer edge. Opening is the sign that lifts the free edge: about +x a flap reaching -y
# from its crease lifts at negative angles, about +y a flap reaching +x lifts at negative angles.
JOINTS = {
    "minor_left": rx.Revolute(anchor=(0.0, FLAP_D / 2, 0.0), axis="X", lower=-OPEN, upper=2.0, initial=-OPEN,
                              target=-OPEN, stiffness=MINOR_K, damping=DAMP),
    "minor_right": rx.Revolute(anchor=(0.0, -FLAP_D / 2, 0.0), axis="X", lower=-2.0, upper=OPEN, initial=OPEN,
                               target=OPEN, stiffness=MINOR_K, damping=DAMP),
    "major_near": rx.Revolute(anchor=(-MAJOR[1] / 2, 0.0, 0.0), axis="Y", lower=-OPEN, upper=5.0, initial=-OPEN,
                              damping=DAMP),
    "major_far": rx.Revolute(anchor=(MAJOR[1] / 2, 0.0, 0.0), axis="Y", lower=-5.0, upper=OPEN, initial=OPEN,
                             damping=DAMP),
}
rx.install(JOINTS)

CLOSED_DEG = 10.0


# -- scoring -------------------------------------------------------------------------------------------------------------


def flap_angle_deg(state) -> float:
    """Rotation from the closed (zero) pose, which has the identity rotation."""
    w = abs(float(state.quat_xyzw[3]))
    return math.degrees(2.0 * math.acos(min(1.0, w)))


def closed(state) -> bool:
    return flap_angle_deg(state) < CLOSED_DEG


def _minor_under(objects, name) -> bool:
    return closed(objects[name]) and all(objects[name].pos[2] < objects[m].pos[2] for m in MAJORS)


def flap_credits(objects) -> dict:
    """0.25 per flap: a closed minor below both majors; a closed major with both minors closed below it."""
    minors_ok = all(_minor_under(objects, n) for n in MINORS)
    out = {n: 0.25 if _minor_under(objects, n) else 0.0 for n in MINORS}
    for m in MAJORS:
        out[m] = 0.25 if closed(objects[m]) and minors_ok else 0.0
    return out


def score(objects) -> float:
    return float(sum(flap_credits(objects).values()))


def success(objects) -> bool:
    return score(objects) == 1.0 and all(objects[n].resting() for n in FLAP_NAMES)


def distance(objects) -> float:
    return float(sum(math.radians(flap_angle_deg(objects[n])) for n in FLAP_NAMES))


# -- scripted demo (both arms on one clock; it never reads object poses) -------------------------------------------------
# Pushes and presses use the closed gripper; opened, its two pads stand 47 mm either side of the TCP (FINGER_OPEN).
FINGER_HALF = abs(FINGER_OPEN)  # 47 mm
PUSH_Z = TOP + 0.100  # TCP height for pushing a standing flap past vertical
PRESS_Z = MINOR_Z + T / 2 - 0.001 + 0.013  # pad tips 1 mm into a closed minor's top (pad tips are 13 mm below the TCP)
MAJOR_PUSH_Z = PUSH_Z  # lower, the finger rail (68 mm long, 40 mm above the TCP) would touch the standing flap
HIGH = TOP + 0.120
MINOR_X = {"right": 0.240, "left": 0.350}  # each arm works one minor, the right in front, the left behind
PRESS_Y = 0.070  # |TCP y| pressing a minor: the closed pads 40 mm in from the crease
PARK = {"left": (0.20, 0.40, HIGH), "right": (0.20, -0.40, HIGH)}


def demo():
    xr, xl = MINOR_X["right"], MINOR_X["left"]
    c, o = FINGER_CLOSED, FINGER_OPEN
    right = [
        Waypoint(2.5, (xr, -0.190, PUSH_Z), c, "outside the right end flap"),
        Waypoint(2.0, (xr, -0.115, PUSH_Z), c, "push it past vertical"),
        Waypoint(1.5, (xr, -PRESS_Y, PRESS_Z), c, "press it flat"),
        Waypoint(1.0, (xr, -PRESS_Y, PRESS_Z), c, "hold"),
        # slide to the middle while opening: one pad stays on the right flap (y -0.070 -> -0.047) and the other
        # crosses to the left flap (y +0.047), so one gripper holds both end flaps
        Waypoint(2.0, (xr, 0.0, PRESS_Z), o, "hold both end flaps"),
        Waypoint(1.0, (xr, 0.0, PRESS_Z), o, "hold"),  # left arm rises
        Waypoint(3.0, (xr, 0.0, PRESS_Z), o, "hold"),  # left arm goes round the back corner, behind the far flap
        Waypoint(1.0, (xr, 0.0, PRESS_Z), o, "hold"),
        Waypoint(2.0, (xr, 0.0, PRESS_Z), o, "hold"),  # left arm pushes the far flap shut
        Waypoint(1.5, (xr, 0.0, PRESS_Z), o, "hold"),
        Waypoint(1.0, (xr, 0.0, PRESS_Z + 0.030), o, "rise"),
        # round the front corner, never through the standing near flap
        Waypoint(1.0, (xr, -0.190, PUSH_Z), c, "out past the right side"),
        Waypoint(1.0, (0.150, -0.190, PUSH_Z), c, "round the front corner"),
        Waypoint(1.0, (0.150, -0.060, PUSH_Z), c, "in front of the near flap"),
        Waypoint(1.0, (0.150, -0.060, MAJOR_PUSH_Z), c, "settle"),
        # just past vertical (the flap falls shut on its own), then out sideways before it has rested 1 s, so the
        # last frame shows the closed top
        Waypoint(1.5, (0.215, -0.060, MAJOR_PUSH_Z), c, "push the near flap past vertical"),
        Waypoint(0.6, (0.200, -0.200, HIGH), c, "out to the side"),
        Waypoint(2.4, PARK["right"], o, "park"),
        Waypoint(2.0, PARK["right"], o, "hold"),
    ]
    left = [
        Waypoint(2.5, (xl, 0.190, PUSH_Z), c, "outside the left end flap"),
        Waypoint(2.0, (xl, 0.115, PUSH_Z), c, "push it past vertical"),
        Waypoint(1.5, (xl, PRESS_Y, PRESS_Z), c, "press it flat"),
        Waypoint(1.0, (xl, PRESS_Y, PRESS_Z), c, "hold"),
        Waypoint(2.0, (xl, PRESS_Y, PRESS_Z), c, "hold"),  # right gripper takes both end flaps
        # off outward (+y), away from the right arm's elbow over the middle, then round the back corner
        Waypoint(1.0, (xl, 0.125, PRESS_Z + 0.012), c, "slide off outward"),
        Waypoint(1.0, (xl, 0.190, PUSH_Z), c, "out past the left side"),
        Waypoint(1.0, (0.430, 0.190, PUSH_Z), c, "round the back corner"),
        Waypoint(1.0, (0.430, 0.100, PUSH_Z), c, "behind the far flap"),
        Waypoint(1.0, (0.430, 0.100, MAJOR_PUSH_Z), c, "settle"),
        Waypoint(2.0, (0.350, 0.100, MAJOR_PUSH_Z), c, "push the far flap past vertical"),
        Waypoint(1.5, (0.350, 0.100, HIGH), c, "up"),
        Waypoint(1.0, (0.250, 0.250, HIGH), o, "away"),
        Waypoint(2.0, PARK["left"], o, "park"),
        Waypoint(1.0, PARK["left"], o, "hold"),
        Waypoint(2.0, PARK["left"], o, "hold"),
        Waypoint(1.0, PARK["left"], o, "hold"),
        Waypoint(2.5, PARK["left"], o, "hold"),
        Waypoint(2.0, PARK["left"], o, "hold"),
    ]
    return {"left": left, "right": right}
