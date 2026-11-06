"""On the station table: three round steel tins stand in a row across the table, 30 cm out from the arms, one in
front of the left arm, one between the arms and one in front of the right arm. The tins are 40, 60 and 80 mm in
diameter and all 5 cm tall, with closed flat tops, and they are taped down so they do not move.

On top of each tin lies a flat round lid. The three lids are one colour and one thickness (2.5 cm) and differ only
in diameter: 40, 60 and 80 mm. Every lid is on a tin it does not fit: a lid fits the tin of its own diameter.
There is no fourth tin. The rest of the table is clear.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- tins: straight-sided steel tins with closed tops, fixed (taped down) -------------------------------------

TIN_H = 0.050
TIN_TOP = TOP + TIN_H
TIN_X = 0.30
STEEL = (0.74, 0.75, 0.77)
# name -> (radius, y). Left to right as seen from the arms: the 40 mm tin, the 60 mm tin, the 80 mm tin.
TIN_SPEC = {
    "tin_left": (0.020, 0.13),
    "tin_middle": (0.030, 0.00),
    "tin_right": (0.040, -0.13),
}
TINS = {
    name: Cylinder(name, pos=(TIN_X, y, TOP + TIN_H / 2), radius=r, height=TIN_H, color=STEEL)
    for name, (r, y) in TIN_SPEC.items()
}

# -- lids: flat discs of one colour, loose. Each starts on a tin of another diameter (a three-way cycle) --------

LID_H = 0.025
LID_BLUE = (0.16, 0.30, 0.62)
SPAWN_GAP = 0.001  # lids spawn this far above their tin and settle onto it
# name -> (radius, mass kg, the tin it starts on). Masses are hardwood (about 720 kg/m^3) at these sizes.
LID_SPEC = {
    "lid_a": (0.040, 0.090, "tin_left"),  # 80 mm lid on the 40 mm tin: overhangs it by 2 cm all round
    "lid_b": (0.020, 0.023, "tin_middle"),  # 40 mm lid on the 60 mm tin: leaves a 1 cm ring of the tin top bare
    "lid_c": (0.030, 0.051, "tin_right"),  # 60 mm lid on the 80 mm tin
}
LIDS = {
    name: Cylinder(
        name,
        pos=(TIN_X, TIN_SPEC[tin][1], TIN_TOP + SPAWN_GAP + LID_H / 2),
        radius=r,
        height=LID_H,
        mass=m,
        color=LID_BLUE,
    )
    for name, (r, m, tin) in LID_SPEC.items()
}
# The lid each tin takes: the one of its own diameter (checked in test_task.py).
MATCH = {"tin_left": "lid_b", "tin_middle": "lid_c", "tin_right": "lid_a"}

OBJECTS = [*TINS.values(), *LIDS.values()]

# -- scoring ---------------------------------------------------------------------------------------------------

# A lid counts as on its tin when it visibly covers it: its centre within 1 cm of the tin's axis, so its edge
# overhangs the tin's side by at most 1 cm anywhere (a ruler checks this). A 1 cm offset is below what the top
# camera resolves (about 2 cm), so a lid within it looks square on its tin; it is half the radius of the smallest
# tin, so the lid rests flat with its centre well inside the rim, and at least two-thirds of every tin top (68% of
# the 40 mm tin's, 84% of the 80 mm tin's) is under its lid.
CENTRE_TOL = 0.010
SEAT_TOL = 0.003  # lid underside within 3 mm of the tin top (contact and solver slop)
MAX_TILT_DEG = 5.0
STACK_WINDOW = 3 * LID_H  # another lid over a tin and up to 7.5 cm above its top counts as "also on that tin"
PER_TIN = 1.0 / 3.0


def centre_tol(tin: str) -> float:
    return CENTRE_TOL


def tin_axis(tin: str) -> np.ndarray:
    return np.array([TIN_X, TIN_SPEC[tin][1]])


def tilt_deg(state) -> float:
    """Angle between the lid's axis and the vertical, either face up (deg)."""
    x, y, _, _ = state.quat_xyzw
    cos = abs(1.0 - 2.0 * (x * x + y * y))
    return math.degrees(math.acos(min(1.0, cos)))


def seated(state, tin: str) -> bool:
    """Lying flat and still on the tin's top: centre within the tolerance of the axis, underside on the top,
    tilted under 5 deg."""
    off = float(np.linalg.norm(state.pos[:2] - tin_axis(tin)))
    return bool(
        off <= centre_tol(tin)
        and abs(state.low[2] - TIN_TOP) <= SEAT_TOL
        and tilt_deg(state) < MAX_TILT_DEG
        and state.resting()
    )


def over(state, lid: str, tin: str) -> bool:
    """The lid overlaps the tin's outline and lies on or above its top (within the stack window): it is on that
    tin, or stacked on whatever is."""
    reach = TIN_SPEC[tin][0] + LID_SPEC[lid][0]
    near = float(np.linalg.norm(state.pos[:2] - tin_axis(tin))) < reach
    return bool(near and TIN_TOP - SEAT_TOL <= state.low[2] <= TIN_TOP + STACK_WINDOW)


def tin_score(objects, tin: str) -> float:
    """1/3 when the tin's own lid is seated on it and no other lid is on it or stacked over it; 0 otherwise."""
    own = MATCH[tin]
    if not seated(objects[own], tin):
        return 0.0
    if any(over(objects[lid], lid, tin) for lid in LIDS if lid != own):
        return 0.0
    return PER_TIN


def score(objects):
    """Sum over the three tins, 0..1."""
    return float(min(1.0, sum(tin_score(objects, tin) for tin in TINS)))


def success(objects):
    """Every tin carries its own lid, seated and alone; must hold for 1 s."""
    return all(tin_score(objects, tin) > 0.0 for tin in TINS)


def goal_pos(lid: str) -> np.ndarray:
    tin = next(t for t, l in MATCH.items() if l == lid)
    return np.array([*tin_axis(tin), TIN_TOP + LID_H / 2])


def distance(objects):
    """Metres, summed over the lids: each lid's centre to where it lies centred on its own tin."""
    return float(sum(np.linalg.norm(objects[lid].pos - goal_pos(lid)) for lid in LIDS))


# -- demo ------------------------------------------------------------------------------------------------------
#
# The arms' wrists bulge sideways, so only one arm may work at the middle tin or the parking spot at a time while
# the other waits at its rest point; both may work at their own end tins at once. At the start both arms lift the
# lid off the tin in front of them. The right arm takes its lid (60 mm) to its rest point and holds it. The left
# arm parks the 80 mm lid on the table between the arms, moves the 40 mm lid from the middle tin onto the 40 mm
# tin, and goes to its rest point. The right arm then sets the 60 mm lid on the middle tin and takes the 80 mm
# lid from the parking spot to the 80 mm tin. Grips are straight down at yaw 0 (fingers closing along y); a disc
# needs no turn.

TRAVEL_Z = TOP + 0.13  # TCP carry height; the carried lid's underside is then 3.5 cm above the lids on the tins
GRIP_ABOVE_BOTTOM = 0.020  # TCP this far above the lid's underside: pad tips (13 mm below the TCP) 7 mm above it
RELEASE_GAP = 0.003  # a lid is let go this far above what it lands on
APPROACH_CLEARANCE = 0.007  # pads this far off each side of a lid on the way down; it also bounds the grip offset
# Closing pads do not centre the lid: one finger leads and the other stays near its open stop, so the lid ends up
# to -y of the TCP by up to APPROACH_CLEARANCE (seen in sim on other tasks, always toward -y). Aim every place half
# of that to +y, so the lid lands within +-3.5 mm of the target whether the offset is 0 or the whole 7 mm.
GRIP_SHIFT_Y = APPROACH_CLEARANCE / 2
LIFT_AFTER_RELEASE = 0.03  # rise this far straight up before any sideways move
PARK_XY = (0.17, 0.0)  # the table spot between the arms where the 80 mm lid waits
LEFT_REST = (0.20, 0.24, TRAVEL_Z)  # out of the right arm's way at the end
RIGHT_REST = (0.20, -0.24, TRAVEL_Z)
FINAL_HOLD_S = 1.5


def approach_finger(lid: str) -> float:
    """Finger target that leaves APPROACH_CLEARANCE between each pad and the lid (fully open for the 80 mm lid)."""
    return max(FINGER_OPEN, -(LID_SPEC[lid][0] + APPROACH_CLEARANCE))


class _Track:
    """A waypoint list with its running end time."""

    def __init__(self, start_pos, finger=FINGER_OPEN):
        self.wps, self.t, self.pos, self.finger = [], 0.0, start_pos, finger

    def go(self, duration, pos=None, finger=None, label=""):
        if duration <= 0:
            return
        self.pos = self.pos if pos is None else pos
        self.finger = self.finger if finger is None else finger
        self.wps.append(Waypoint(duration, self.pos, self.finger, label))
        self.t += duration

    def wait_until(self, t, label="wait"):
        self.go(t - self.t, label=label)


def pick(track: _Track, lid: str, xy, support_top: float):
    """From anywhere at travel height: over the lid, down, close, straight up. Returns when the lid is lifted."""
    x, y = xy
    open_ = approach_finger(lid)
    grasp_z = support_top + GRIP_ABOVE_BOTTOM
    track.go(1.5, (x, y, TRAVEL_Z), open_, f"over {lid}")
    track.go(0.4, label="settle")
    track.go(1.4, (x, y, grasp_z), label="descend")
    track.go(0.8, finger=FINGER_CLOSED, label="close")
    track.go(1.2, (x, y, TRAVEL_Z), label="lift")


def place(track: _Track, lid: str, xy, support_top: float):
    """From travel height holding ``lid``: carry over ``xy``, lower, let go, straight up 3 cm, then to travel."""
    x, y = xy[0], xy[1] + GRIP_SHIFT_Y
    release_z = support_top + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    track.go(2.0, (x, y, TRAVEL_Z), label=f"carry {lid}")
    track.go(2.0, (x, y, release_z), label="lower")
    track.go(0.5, label="settle")
    track.go(0.6, finger=approach_finger(lid), label="release")
    track.go(0.6, (x, y, release_z + LIFT_AFTER_RELEASE), label="lift off")
    track.go(0.8, (x, y, TRAVEL_Z), label="up")


def demo():
    """Four lid moves, two per arm, with the 80 mm lid parked on the table in between (see the comment above)."""
    left, right = _Track(None), _Track(None)
    xy = {tin: (TIN_X, TIN_SPEC[tin][1]) for tin in TINS}

    # Both at once, each at its own end tin: lift the lid off it.
    pick(left, "lid_a", xy["tin_left"], TIN_TOP)
    pick(right, "lid_c", xy["tin_right"], TIN_TOP)
    right.go(1.5, RIGHT_REST, label="to rest, holding the lid")
    # Left, with the right arm out of the way: park the 80 mm lid, then the 40 mm lid onto the 40 mm tin.
    left.wait_until(right.t, "wait for the right arm")
    place(left, "lid_a", PARK_XY, TOP)
    pick(left, "lid_b", xy["tin_middle"], TIN_TOP)
    place(left, "lid_b", xy["tin_left"], TIN_TOP)
    left.go(1.5, LEFT_REST, label="rest")
    # Right, with the left arm out of the way: the 60 mm lid onto the middle tin, the 80 mm lid onto the 80 mm tin.
    right.wait_until(left.t, "hold the lid until the left arm is out of the way")
    place(right, "lid_c", xy["tin_middle"], TIN_TOP)
    pick(right, "lid_a", PARK_XY, TOP)  # the lid lies within +-3.5 mm of PARK_XY
    place(right, "lid_a", xy["tin_right"], TIN_TOP)
    right.go(1.5, RIGHT_REST, label="rest")

    end = max(left.t, right.t) + FINAL_HOLD_S
    left.wait_until(end, "hold")
    right.wait_until(end, "hold")
    return {"left": left.wps, "right": right.wps}
