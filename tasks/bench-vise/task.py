"""On the station table: a small drill-press vise, fixed to the table, and a 3D-printed part standing upright beside it.

The vise's jaws close along y: the fixed jaw is on the left (+y), the moving jaw on the right, driven by a screw under
the bed that ends in a black ribbed knob at the vise's right end. Turning the knob clockwise as seen from its end (its
top moving away from the robots) closes the jaw, 16 mm per turn. The jaws start 30 mm apart.

The part is an orange printed block, 20 x 20 x 50 mm, with a black line round it 20 mm above its bottom: the clamp
height. Clamping it means holding it upright between the jaws with the line level with the jaw tops while the jaw is
closed onto it. Let go early and it drops 12 mm onto the vise's bed, too deep, so one hand holds the part while the other
turns the knob.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CAST = (0.20, 0.30, 0.42)  # painted cast iron
STEEL = (0.70, 0.71, 0.73)
BLACK = (0.06, 0.06, 0.07)
WHITE = (0.97, 0.97, 0.97)
ORANGE = (0.95, 0.45, 0.10)  # PLA

# -- the vise (fixed). Frame: origin on the table under the centre of a clamped part; x away from the robots, y left. ---
VX, VY = 0.32, 0.05
JAW_W = 0.060  # jaw width along x
JAW_T = 0.020  # jaw thickness along y
B = 0.075  # bed: the top of the body, between the jaws
J = 0.107  # jaw tops
FIXED_FACE = 0.010  # the fixed jaw's face (y); a clamped 20 mm part is centred on y = 0
GAP0 = 0.030  # jaw opening at the start
BODY_Y = (-0.075, 0.035)  # body from its right end (knob side) to behind the fixed jaw
SCREW_Z = 0.065  # screw axis, under the bed
LEAD = 0.016  # jaw travel per knob turn (m)
KNOB_R, KNOB_W = 0.018, 0.020
RIB_R = 0.0022
SHAFT_L = 0.008  # bare shaft between the body's end and the knob
KNOB_Y = BODY_Y[0] - SHAFT_L - KNOB_W / 2  # knob centre
JAW_LO, JAW_HI = -0.012, GAP0 - 0.001  # moving-jaw travel (m, + closes): 42 mm open to 1 mm
JAW_K, JAW_C = 10000.0, 100.0  # the screw's drive on the jaw: stiffness N/m, damping N s/m
SLIP = 0.003  # the screw winds the jaw's target at most 3 mm past where the jaw is: clamping force <= 30 N
KNOB_LIM = 8 * math.pi

_MOVING_FACE0 = FIXED_FACE - GAP0


def _ribs(cy, r, w, n=12):
    return [((r * math.cos(2 * math.pi * i / n), cy, SCREW_Z + r * math.sin(2 * math.pi * i / n)), RIB_R, w, BLACK,
             "Y") for i in range(n)]


JAW = Joint(
    "jaw", kind="slide", axis="Y", anchor=(0.0, _MOVING_FACE0 - JAW_T / 2, B), lower=JAW_LO, upper=JAW_HI,
    initial=0.0, stiffness=JAW_K, damping=JAW_C, target=0.0, mass=0.15,
    boxes=[
        ((0.0, _MOVING_FACE0 - JAW_T / 2 - 0.0015, (B + J) / 2), (JAW_W, JAW_T - 0.003, J - B), CAST),
        ((0.0, _MOVING_FACE0 - 0.0015, (B + J) / 2), (JAW_W, 0.003, J - B), STEEL),  # hardened jaw plate
    ],
)
KNOB = Joint(
    "knob", kind="hinge", axis="Y", anchor=(0.0, KNOB_Y, SCREW_Z), lower=-KNOB_LIM, upper=KNOB_LIM, initial=0.0,
    stiffness=0.0, damping=0.3, target=0.0, mass=0.05,
    cylinders=[
        ((0.0, BODY_Y[0] - SHAFT_L / 2 - 0.001, SCREW_Z), 0.005, SHAFT_L + 0.002, STEEL, "Y"),
        ((0.0, KNOB_Y, SCREW_Z), KNOB_R, KNOB_W, BLACK, "Y"),
        *_ribs(KNOB_Y, KNOB_R, KNOB_W),
    ],
    boxes=[((0.0, KNOB_Y - KNOB_W / 2 - 0.0004, SCREW_Z + KNOB_R / 2), (0.003, 0.0008, KNOB_R), WHITE)],  # index line
)
VISE = part(
    "vise", pos=(VX, VY, TOP), fixed=True, friction=0.8,
    boxes=[
        ((0.0, sum(BODY_Y) / 2, 0.005), (0.090, BODY_Y[1] - BODY_Y[0], 0.010), CAST),  # base with mounting flanges
        ((0.0, sum(BODY_Y) / 2, (0.010 + B) / 2), (JAW_W, BODY_Y[1] - BODY_Y[0], B - 0.010), CAST),  # body and bed
        ((0.0, FIXED_FACE + 0.0015, (B + J) / 2), (JAW_W, 0.003, J - B), STEEL),  # fixed jaw plate
        ((0.0, (FIXED_FACE + 0.003 + BODY_Y[1]) / 2, (B + J) / 2), (JAW_W, BODY_Y[1] - FIXED_FACE - 0.003, J - B), CAST),
    ],
    joints=[JAW, KNOB],
)

# -- the printed part (loose), standing upright on the table ----------------------------------------------------------
PART_W, PART_H = 0.020, 0.050
MARK_H, MARK_T = 0.020, 0.001  # the line: 1 mm wide, centred 20 mm above the bottom
PART_XY = (0.22, 0.20)
PART = part(
    "part", pos=(*PART_XY, TOP), mass=0.015, friction=0.8,
    boxes=[
        ((0.0, 0.0, (MARK_H - MARK_T / 2) / 2), (PART_W, PART_W, MARK_H - MARK_T / 2), ORANGE),
        ((0.0, 0.0, MARK_H), (PART_W, PART_W, MARK_T), BLACK),
        ((0.0, 0.0, (MARK_H + MARK_T / 2 + PART_H) / 2), (PART_W, PART_W, PART_H - MARK_H - MARK_T / 2), ORANGE),
    ],
)

OBJECTS = [VISE, PART]


# -- the screw: the jaw follows the knob ------------------------------------------------------------------------------
def screw_target(prev: float, d_knob: float, jaw: float) -> float:
    """The moving jaw's drive target after the knob turns by ``d_knob`` (rad). Closing winds the target on by the lead,
    but never more than ``SLIP`` past the jaw itself, so a jaw stopped by the part presses on it with at most
    JAW_K * SLIP; opening unwinds it at once."""
    s = prev + LEAD * d_knob / (2 * math.pi)
    if d_knob > 0:
        s = min(s, max(prev, jaw + SLIP))
    return float(np.clip(s, JAW_LO, JAW_HI))


_screw = {"knob": None, "target": 0.0}


@on_reset
def _reset_screw(env):
    _screw["knob"], _screw["target"] = None, 0.0


@on_step
def _drive(env):
    import torch

    import labkit

    s = labkit._part_state(env, labkit._SPECS["vise"])
    th, q = s.joints["knob"], s.joints["jaw"]
    if _screw["knob"] is None:
        _screw["knob"] = th
    _screw["target"] = screw_target(_screw["target"], th - _screw["knob"], q)
    _screw["knob"] = th
    asset = env.unwrapped.scene["vise"]
    ids = [list(asset.joint_names).index(JAW.joint_name)]
    asset.set_joint_position_target(torch.tensor([[_screw["target"]]], device=asset.device), joint_ids=ids)


# -- scoring ----------------------------------------------------------------------------------------------------------
GAP_TOL = 0.0005  # clamped: jaw gap within 0.5 mm of the part's width
PRELOAD = 0.0005  # and the screw wound at least 0.5 mm past the jaw (5 N of clamping force)
NEAR_TOL, MARK_TOL = 0.005, 0.002
TILT_TOL = 5.0
SUCCESS_HOLD_S = 10.0  # end a trial early only once the part has stayed clamped for 10 s (time to let go of it)


def jaw_gap(j: dict) -> float:
    return GAP0 - j["jaw"]


def _part_in_vise(p, j: dict):
    """(between the jaws, mark height above the jaw tops in m). Between: the part's bottom centre below the jaw tops,
    within the jaws' width and between their faces."""
    bottom = p.to_world((0.0, 0.0, 0.0)) - np.array([VX, VY, TOP])
    mark = p.to_world((0.0, 0.0, MARK_H))[2] - (TOP + J)
    moving_face = FIXED_FACE - jaw_gap(j)
    between = bottom[2] < J and abs(bottom[0]) <= JAW_W / 2 and moving_face <= bottom[1] <= FIXED_FACE
    return between, float(mark)


def clamped(j: dict) -> bool:
    return jaw_gap(j) <= PART_W + GAP_TOL and _screw["target"] - j["jaw"] >= PRELOAD


def score(objects):
    """0 unless the part is between the jaws (its bottom centre below the jaw tops, inside the jaws' 60 mm width and
    between their faces). 0.25 between the jaws but not clamped. Clamped: the jaw gap within 0.5 mm of the part's 20 mm
    width and the vise tightened on it (at least 5 N). 0.5 clamped, but the line more than 5 mm from the jaw tops'
    level or the part tilted more than 5 degrees; 0.75 clamped upright with the line within 5 mm of the jaw tops' level;
    1.0 within 2 mm."""
    v, p = objects["vise"], objects["part"]
    between, mark = _part_in_vise(p, v.joints)
    if not between:
        return 0.0
    if not clamped(v.joints):
        return 0.25
    if p.tilt_deg() > TILT_TOL or abs(mark) > NEAR_TOL:
        return 0.5
    return 1.0 if abs(mark) <= MARK_TOL else 0.75


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """Line height off the jaw tops (m) plus the jaw gap's excess over the part's width."""
    _, mark = _part_in_vise(objects["part"], objects["vise"].joints)
    return abs(mark) + max(0.0, jaw_gap(objects["vise"].joints) - PART_W)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: picks the part off the table by its top (fingers closing along x, pad tips 28 mm above its bottom), carries
# it over the vise, lowers it between the open jaws until its line is level with the jaw tops, 0.5 mm off the fixed jaw,
# and holds it there; once the vise is tight it lets go and parks. Right arm: meanwhile levels its gripper beside the
# knob, pointing along +y, and rolls it to -150 degrees; when the part is held it grips the knob's rim and turns it
# clockwise 150 degrees (wrist roll -150 to 0), lets go, rolls back in place, grips again and turns another 150 degrees:
# 300 degrees, 13.3 mm of screw, against 10 mm to close on the part. The task file lets a Waypoint's ``yaw`` carry a full
# gripper rotation (``Orient``).


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_down_grip = _demo.grip_rotation
if not getattr(_down_grip, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _down_grip(yaw)

    _grip_rotation._orient = True
    _demo.grip_rotation = _grip_rotation

SIDE = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.pi / 2) @ _down_grip(0.0)  # level, pointing +y


def side(roll_deg: float) -> Orient:
    """Level grip pointing +y, rolled about +y (positive turns the knob's top away from the robots: closing)."""
    return Orient(axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(roll_deg)) @ SIDE)


def W(x, y, z):
    """Local vise coordinates to world."""
    return (VX + x, VY + y, TOP + z)


TIP_ABOVE_TCP = 0.013
LEFT_PARK = (0.15, 0.25, TOP + 0.14)
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)
PART_YAW = -math.pi / 2  # fingers close along x, on the part's x faces
PART_WIDE = -0.020  # pads 40 mm apart round the 20 mm part
PART_RELEASE = -0.012
GRIP_UP = 0.028  # pad tips 28 mm above the part's bottom: 8 mm above the jaw tops when it is held at the line
PICK_Z = TOP + GRIP_UP + TIP_ABOVE_TCP
PICK_HOVER = TOP + PART_H + 0.05
CARRY_Z = TOP + J + 0.010 + GRIP_UP + TIP_ABOVE_TCP  # the part's bottom 10 mm above the jaw tops
HOLD_Y = -0.0005  # 0.5 mm off the fixed jaw
ENTRY_DY = 0.004  # lowered in 4.5 mm clear of the fixed jaw, then moved across to it
HOLD = W(0.0, HOLD_Y, J - MARK_H + GRIP_UP + TIP_ABOVE_TCP)
KNOB_WIDE = -0.027  # pads 54 mm apart round the 40 mm ribbed knob
KNOB_GRIP = W(0.0, KNOB_Y - KNOB_W / 2 + 0.009 - TIP_ABOVE_TCP, SCREW_Z)  # pad tips 9 mm onto the knob
KNOB_OUT = (KNOB_GRIP[0], KNOB_GRIP[1] - 0.04, KNOB_GRIP[2])
KNOB_OUT_UP = (KNOB_OUT[0], KNOB_OUT[1], KNOB_OUT[2] + 0.04)
ROLL0, TURN = -150.0, 150.0  # each grip turns the knob from wrist roll -150 to 0 degrees
TURN_STEPS = 6

PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    ("vise base", W(0.0, sum(BODY_Y) / 2, 0.005), (0.090, BODY_Y[1] - BODY_Y[0], 0.010)),
    ("vise body", W(0.0, sum(BODY_Y) / 2, (0.010 + B) / 2), (JAW_W, BODY_Y[1] - BODY_Y[0], B - 0.010)),
    ("fixed jaw", W(0.0, (FIXED_FACE + BODY_Y[1]) / 2, (B + J) / 2), (JAW_W, BODY_Y[1] - FIXED_FACE, J - B)),
    ("moving jaw (its travel)", W(0.0, (_MOVING_FACE0 - JAW_T + FIXED_FACE - PART_W) / 2, (B + J) / 2),
     (JAW_W, JAW_T + GAP0 - PART_W, J - B)),
]


def _dur(wps):
    return sum(w.duration for w in wps)


def _turn(wps, label):
    """Grip the knob at wrist roll ROLL0 and roll it TURN degrees clockwise (closing), then let go."""
    wps += [
        Waypoint(1.2, KNOB_GRIP, KNOB_WIDE, "onto the knob", side(ROLL0)),
        Waypoint(0.8, KNOB_GRIP, FINGER_CLOSED, "grip the knob's rim", side(ROLL0)),
    ]
    for k in range(1, TURN_STEPS + 1):
        wps.append(Waypoint(1.0, KNOB_GRIP, FINGER_CLOSED, label, side(ROLL0 + TURN * k / TURN_STEPS)))
    wps += [
        Waypoint(0.5, KNOB_GRIP, FINGER_CLOSED, "settle", side(ROLL0 + TURN)),
        Waypoint(0.6, KNOB_GRIP, KNOB_WIDE, "let go", side(ROLL0 + TURN)),
    ]


def demo():
    px, py = PART_XY
    left = [
        Waypoint(2.5, (px, py, PICK_HOVER), FINGER_OPEN, "above the part", PART_YAW),
        Waypoint(1.2, (px, py, PICK_Z), PART_WIDE, "fingers either side of its top", PART_YAW),
        Waypoint(0.8, (px, py, PICK_Z), FINGER_CLOSED, "grip", PART_YAW),
        Waypoint(1.5, (px, py, CARRY_Z), FINGER_CLOSED, "lift", PART_YAW),
        Waypoint(2.0, (HOLD[0], HOLD[1] - ENTRY_DY, CARRY_Z), FINGER_CLOSED, "over the open jaws", PART_YAW),
        Waypoint(1.5, (HOLD[0], HOLD[1] - ENTRY_DY, HOLD[2]), FINGER_CLOSED, "lower between the jaws: line level with the jaw tops", PART_YAW),
        Waypoint(0.8, HOLD, FINGER_CLOSED, "to the fixed jaw", PART_YAW),
        Waypoint(0.8, HOLD, FINGER_CLOSED, "steady", PART_YAW),
    ]
    t_held = _dur(left)
    right = [
        Waypoint(2.5, KNOB_OUT_UP, KNOB_WIDE, "above, beside the knob"),
        Waypoint(1.5, KNOB_OUT, KNOB_WIDE, "level the gripper", side(0.0)),
        Waypoint(1.2, KNOB_OUT, KNOB_WIDE, "roll", side(ROLL0 / 2)),
        Waypoint(1.2, KNOB_OUT, KNOB_WIDE, "roll", side(ROLL0)),
    ]
    right.append(Waypoint(max(0.3, t_held - _dur(right)), KNOB_OUT, KNOB_WIDE, "wait for the part to be held",
                          side(ROLL0)))
    _turn(right, "close the jaw: turn the knob, top away")
    right += [
        Waypoint(1.0, KNOB_GRIP, KNOB_WIDE, "roll back", side(ROLL0 + TURN / 2)),
        Waypoint(1.0, KNOB_GRIP, KNOB_WIDE, "roll back", side(ROLL0)),
    ]
    _turn(right, "tighten: turn the knob, top away")
    t_tight = _dur(right)
    right += [
        Waypoint(1.0, KNOB_OUT, KNOB_WIDE, "back off", side(ROLL0 + TURN)),
        Waypoint(1.5, KNOB_OUT_UP, KNOB_WIDE, "point down again"),
        Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    left.append(Waypoint(t_tight - _dur(left), HOLD, FINGER_CLOSED, "hold the part at the line", PART_YAW))
    left += [
        Waypoint(0.6, HOLD, PART_RELEASE, "let go", PART_YAW),
        Waypoint(1.0, (HOLD[0], HOLD[1], CARRY_Z), PART_RELEASE, "up", PART_YAW),
        Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"),
    ]
    t_end = max(_dur(left), _dur(right)) + 2.0
    left.append(Waypoint(t_end - _dur(left), LEFT_PARK, FINGER_OPEN, "hold"))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "hold"))
    return {"left": left, "right": right}
