"""On the station table: a grey deck (26 x 32.5 x 6 cm) carries three sliding bars, red, yellow and orange, each with a
raised handle, and a green mat. Beside it a bay with a sliding blue door as its roof holds a white fuse carrier; the door has a 3.5 x 3.5 cm window
through which the carrier's white top shows (the 4.5 cm carrier cannot pass it). The bars
block each other and the last one blocks the door: the red bar's tip stands in the yellow bar's path, the yellow bar's end
stands in the orange bar's path, and the orange bar's tip stands in the door's path. A bar that has moved also stops the
one before it from sliding back. The carrier is loose; everything else is one fixed assembly with four sliding joints.
The door's north guide rail (an inverted L, brown) stands on the table behind the bay and its lip reaches 1 mm over the plate's
north edge, so the door can only slide.

Requires -E object_jitter=0: the demo and the carrier's start are placed to the millimetre.

Run it:  inspect-robots run --embodiment yam_arena -E task=switchgear_interlock_chain.py -E object_jitter=0 \\
           --instruction "Take the white fuse carrier out of the locked bay and set it on the green mat. The bay door
           is held by a chain of interlock bars. See how they block each other and slide them in order." \\
           --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from belimits import Rect, bottom, inside, is_airborne, tilt_deg
from mechkit import Joint, Mech, box
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
SUCCESS_HOLD_S = 4.0  # the episode ends about 1.6 s after the plan's last move, with the gripper open and clear of the carrier
TOP = TABLE_TOP[SCENE]

GREY, DARK = (0.50, 0.50, 0.53), (0.28, 0.28, 0.31)
RED, YELLOW, ORANGE = (0.82, 0.10, 0.10), (0.95, 0.80, 0.05), (0.95, 0.45, 0.05)
BLUE, GREEN, WHITE = (0.15, 0.30, 0.85), (0.10, 0.62, 0.25), (0.96, 0.96, 0.94)
RAIL = (0.45, 0.27, 0.12)  # wood brown, darker and redder than the oak table

DECK_H = 0.06  # the deck's top above the table
CLEAR = 0.001  # every moving part rides 1 mm above what it slides on
BAR_Z0, BAR_Z1 = DECK_H + CLEAR, DECK_H + 0.02  # bars: z 0.061 .. 0.08
HANDLE_Z1 = DECK_H + 0.07  # handles: up to 0.13

# The mat: 12 x 8.5 x 0.2 cm on the deck's west end beside the bay, clear of every bar and handle (4 mm from bar B's
# handle) and 4 mm from the path of the door's lug.
MAT_XY, MAT_SIZE = (0.19, 0.0665), (0.12, 0.085)
MAT_TOP = DECK_H + 0.002
MAT = Rect((MAT_XY[0] - MAT_SIZE[0] / 2, MAT_XY[0] + MAT_SIZE[0] / 2), (MAT_XY[1] - MAT_SIZE[1] / 2, MAT_XY[1] + MAT_SIZE[1] / 2))


def _slab(x, y, z, color):
    """A box from (x0, x1), (y0, y1), (z0, z1) in the Mech frame (x, y station; z above the table top)."""
    return box(((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2), (x[1] - x[0], y[1] - y[0], z[1] - z[0]), color)


def _piece(origin, parts):
    """Shapes of a moving piece, given as absolute slabs, re-expressed about the piece's own origin."""
    out = []
    for x, y, z, color in parts:
        s = _slab(x, y, z, color)
        out.append(box(tuple(np.array(s.at) - origin), s.size, color))
    return tuple(out)


A_O = np.array([0.16, -0.0875, 0.0705])
B_O = np.array([0.237, 0.0, 0.0705])
C_O = np.array([0.29, 0.0685, 0.0705])
D_O = np.array([0.305, 0.185, 0.066])

BARS = {
    # Bar A, red (supply): slides along y, tip at y +0.015 stands 5 mm short of bar B's end.
    "bar_a": Joint(
        "bar_a", "slide", axis="y", at=tuple(A_O), low=-0.04, high=0.0, damping=6.0, mass=0.05,
        shapes=_piece(A_O, [((0.15, 0.17), (-0.19, 0.015), (BAR_Z0, BAR_Z1), RED),
                            ((0.14, 0.18), (-0.16, -0.13), (BAR_Z1, HANDLE_Z1), RED)]),
    ),
    # Bar B, yellow (earth): slides along x, its -x end 5 mm short of bar A's tip; bar C's tail stands 5 mm beside its +x end.
    "bar_b": Joint(
        "bar_b", "slide", axis="x", at=tuple(B_O), low=-0.04, high=0.0, damping=6.0, mass=0.05,
        shapes=_piece(B_O, [((0.175, 0.299), (-0.01, 0.01), (BAR_Z0, BAR_Z1), YELLOW),
                            ((0.23, 0.26), (-0.02, 0.02), (BAR_Z1, HANDLE_Z1), YELLOW)]),
    ),
    # Bar C, orange (door bolt): slides along y, its tail 5 mm from bar B's side, its tip 5 mm short of the door's lug
    # and in its path.
    "bar_c": Joint(
        "bar_c", "slide", axis="y", at=tuple(C_O), low=-0.04, high=0.0, damping=6.0, mass=0.05,
        shapes=_piece(C_O, [((0.28, 0.30), (0.015, 0.122), (BAR_Z0, BAR_Z1), ORANGE),
                            ((0.27, 0.31), (0.06, 0.09), (BAR_Z1, HANDLE_Z1), ORANGE)]),
    ),
    # The door, blue: a roof plate on the bay's walls with a 3.5 x 3.5 cm window (x 0.2875..0.3225, y 0.1675..0.2025) centred
    # on the carrier (four slabs round a hole: mechkit has no transparency), a handle and a lug that stands 5 mm short of bar C's tip.
    "door": Joint(
        "door", "slide", axis="x", at=tuple(D_O), low=-0.13, high=0.0, damping=6.0, mass=0.05, friction=0.2,
        shapes=_piece(D_O, [((0.235, 0.375), (0.125, 0.1675), (DECK_H + CLEAR, DECK_H + 0.011), BLUE),  # roof plate: south strip
                            ((0.235, 0.375), (0.2025, 0.245), (DECK_H + CLEAR, DECK_H + 0.011), BLUE),  # north strip
                            ((0.235, 0.2875), (0.1675, 0.2025), (DECK_H + CLEAR, DECK_H + 0.011), BLUE),  # west strip
                            ((0.3225, 0.375), (0.1675, 0.2025), (DECK_H + CLEAR, DECK_H + 0.011), BLUE),  # east strip
                            ((0.245, 0.275), (0.16, 0.20), (DECK_H + 0.011, DECK_H + 0.06), BLUE),
                            ((0.305, 0.325), (0.113, 0.125), (DECK_H + CLEAR, DECK_H + 0.021), BLUE)]),
    ),
}

SWITCHGEAR = Mech(
    "switchgear", pos=(0.0, 0.0, TOP), mass=None, friction=0.8,
    shapes=(
        _slab((0.12, 0.38), (-0.20, 0.125), (0.0, DECK_H), GREY),  # the deck
        _slab((0.23, 0.24), (0.12, 0.25), (0.0, DECK_H), DARK),  # bay walls, 1 cm thick, 6 cm tall
        _slab((0.37, 0.38), (0.12, 0.25), (0.0, DECK_H), DARK),
        _slab((0.23, 0.38), (0.12, 0.13), (0.0, DECK_H), DARK),
        _slab((0.23, 0.38), (0.24, 0.25), (0.0, DECK_H), DARK),
        _slab((MAT.x0, MAT.x1), (MAT.y0, MAT.y1), (DECK_H, MAT_TOP), GREEN),  # the mat
        _slab((0.10, 0.38), (0.25, 0.26), (0.0, 0.082), RAIL),  # the door's north guide rail: upright on the table, 1 cm thick
        _slab((0.10, 0.38), (0.24, 0.25), (0.072, 0.082), RAIL),  # its lip, 1 mm over the plate's north edge
    ),
    joints=tuple(BARS.values()),
)

CARRIER_POS = (0.305, 0.185, TOP + 0.025)
CARRIER = Box("fuse_carrier", pos=CARRIER_POS, size=(0.045, 0.045, 0.05), mass=0.06, color=WHITE)
OBJECTS = [SWITCHGEAR, CARRIER]

# Thresholds (m of bar travel; the tape reads the distance a handle has moved from its start).
MOVED_A = 0.024  # bar A: at least 2.4 cm (it frees bar B at 2.5 cm, so the tape reads just under the freeing travel)
MOVED_B = 0.018  # bar B: at least 1.8 cm (it frees bar C at 1.9 cm)
MOVED_C = 0.008  # bar C: at least 0.8 cm (it frees the door at 0.9 cm; a blocked B or C can move only 5 mm)
_MOVED = {"bar_a": MOVED_A, "bar_b": MOVED_B, "bar_c": MOVED_C}
DOOR_OPEN = 0.10  # door: at least 10 cm
FLAT_TOL = 0.01  # carrier bottom within 1 cm of the deck top
MAX_TILT = 10.0  # degrees


def _moved(joints, name):
    return joints[name] <= -_MOVED[name]


def _carrier_on_mat(c) -> bool:
    """Footprint wholly inside the mat, bottom within 1 cm of the deck top, tilted at most 10 degrees. A carrier whose
    bottom is more than 1 cm above both the deck and the mat is still held or in flight and never counts."""
    if is_airborne(c, CARRIER, [TOP + DECK_H, TOP + MAT_TOP], FLAT_TOL):
        return False
    return bool(
        inside(c, CARRIER, MAT)
        and abs(bottom(c, CARRIER) - (TOP + DECK_H)) <= FLAT_TOL
        and tilt_deg(c) <= MAX_TILT
    )


def score(objects):
    """Bands in order: 0.2 bar A moved 2.4 cm; 0.4 and bar B 1.8 cm; 0.6 and bar C 0.8 cm; 0.8 and the door 10 cm; 1.0 the
    carrier on the mat after bars A, B and C were moved that far. Bands 0.2 to 0.8 each need the ones before it; 1.0 does
    not need the door to stay open once the carrier is out."""
    j = objects["switchgear"].joints
    a = _moved(j, "bar_a")
    b = a and _moved(j, "bar_b")
    c = b and _moved(j, "bar_c")
    d = c and j["door"] <= -DOOR_OPEN
    if c and _carrier_on_mat(objects["fuse_carrier"]):
        return 1.0
    return 0.8 if d else 0.6 if c else 0.4 if b else 0.2 if a else 0.0


def success(objects):
    """The carrier on the mat, bars A, B and C moved in order, and the carrier still; held for SUCCESS_HOLD_S (4 s)."""
    return score(objects) == 1.0 and objects["fuse_carrier"].resting()


# The robot keeps out too: no part of either arm goes below the bars' top (plus 5 mm) over the part of the deck that
# holds the bars and handles (x up to 0.35 and y up to 0.12, a little inside the deck's edges at 0.38 and 0.125).
# The fingertips reach the handles from above (pad tips 9.2 cm up), so the demo never reaches in among the bars.
ARM_KEEP_OUT = [
    {"name": "interlock deck below the bar tops", "x": (0.11, 0.35), "y": (-0.27, 0.12), "z_top": TOP + DECK_H + 0.025},
]

# -- the demo ---------------------------------------------------------------------------------------------------------

HANDLE_Z = TOP + 0.105  # TCP height at a bar handle: pad tips 9.2 cm up, 1.2 cm over the bar tops, 3.8 cm up the handle
HOVER = TOP + 0.17
PEAK = 0.15  # m/s: with smoothstep the segment's peak is 1.5 times its mean; the tool cap is 0.20


class _Path:
    """Builds a waypoint list from the arm's ready pose, sizing each move so the tool's peak speed stays under PEAK."""

    def __init__(self, arm):
        self.pos = np.array(ik.tcp_pose(np.array(READY, float), np.array(BASES[arm]))[0], float)
        self.finger = FINGER_OPEN
        self.yaw = 0.0
        self.points = []

    def go(self, pos, label, yaw=None, finger=None, hold=0.0, min_s=0.8, peak=PEAK):
        pos = np.asarray(pos, float)
        yaw = self.yaw if yaw is None else yaw
        finger = self.finger if finger is None else finger
        turn = abs(yaw - self.yaw) / (np.pi / 2) * 2.5  # a quarter turn takes at least 2.5 s
        seconds = max(min_s, 1.5 * float(np.linalg.norm(pos - self.pos)) / peak, turn, hold)
        self.points.append(Waypoint(seconds, tuple(float(v) for v in pos), finger, label, yaw))
        self.pos, self.yaw, self.finger = pos, yaw, finger
        return self

    def at(self, label, seconds):
        return self.go(self.pos, label, hold=seconds)

    def total(self):
        return sum(w.duration for w in self.points)


def _slide(path, start, end, label, handle_z, yaw, seconds, finger_open=FINGER_OPEN):
    """Over a handle, down, close on it, slide it, open, back up."""
    over = (start[0], start[1], HOVER)
    path.go(over, f"to {label}", yaw=yaw, finger=finger_open)
    path.go((start[0], start[1], handle_z), f"down on {label}", finger=finger_open)
    path.go(path.pos, f"close on {label}", finger=FINGER_CLOSED, min_s=0.8)
    path.go((end[0], end[1], handle_z), f"slide {label}", finger=FINGER_CLOSED, min_s=seconds)
    path.go(path.pos, f"let go of {label}", finger=finger_open, min_s=0.8)
    path.go((end[0], end[1], HOVER), f"up from {label}", finger=finger_open)


def demo():
    """The right arm slides bar A, then bar B, and parks on its own side. The left arm, waiting over the bay, slides bar
    C, then the door, then lifts the carrier out and sets it on the mat beside the open door."""
    right = _Path("right")
    _slide(right, (0.16, -0.145), (0.16, -0.185), "bar A", HANDLE_Z, 0.0, 3.0)
    _slide(right, (0.245, 0.0), (0.205, 0.0), "bar B", HANDLE_Z, np.pi / 2, 3.0)
    right.go((0.15, -0.27, HOVER), "park", yaw=0.0)

    left = _Path("left")
    left.go((0.30, 0.20, TOP + 0.22), "wait over the bay", yaw=0.0, min_s=4.0)
    wait_s = right.total() - left.total()
    if wait_s > 0:
        left.at("wait for the right arm", wait_s)
    _slide(left, (0.29, 0.075), (0.29, 0.035), "bar C", HANDLE_Z, 0.0, 3.0)
    _slide(left, (0.26, 0.18), (0.14, 0.18), "the door", TOP + 0.10, -np.pi / 2, 6.0)

    carrier_open = -0.0325  # 6.5 cm between the pads, for a 4.5 cm carrier
    left.go((0.305, 0.185, TOP + 0.12), "over the carrier", yaw=0.0, finger=carrier_open)
    left.go((0.305, 0.185, TOP + 0.035), "down on the carrier", finger=carrier_open)
    left.go(left.pos, "close on the carrier", finger=FINGER_CLOSED, min_s=0.8)
    carry_z = TOP + 0.125  # carrier bottom 9 cm up: 1 cm over the bar bodies and the door's lug, 1.9 cm over the plate
    left.go((0.305, 0.185, carry_z), "lift", finger=FINGER_CLOSED)
    left.go((0.22, 0.185, carry_z), "west over the open door", finger=FINGER_CLOSED)
    left.go((MAT_XY[0], MAT_XY[1], carry_z), "toward the mat", finger=FINGER_CLOSED)
    left.go((MAT_XY[0], MAT_XY[1], TOP + 0.102), "lower onto the mat", finger=FINGER_CLOSED)
    # Open only to the carrier width: full open puts the -y finger about 7 mm into bar B's moved handle.
    left.go(left.pos, "release", finger=carrier_open, min_s=0.8)
    # Rise to HOVER (finger tips 2.7 cm over bar B's moved handle) still at the carrier width; open fully only on the way
    # away.
    left.go((MAT_XY[0], MAT_XY[1], HOVER), "clear of the carrier", finger=carrier_open)
    left.go((0.28, 0.20, HOVER), "away", finger=FINGER_OPEN)
    return {"left": left.points, "right": right.points}
