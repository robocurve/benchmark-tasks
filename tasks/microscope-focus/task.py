"""On the station table: an upright light microscope (modelled on a research stand: mechanical stage with a spring slide
holder and x/y drive knobs, focus knobs on both sides) and a glass slide on a rest beside it. The stage is pulled
towards the robots, away from the objective, for loading, and sits 12 mm below focus.

The task, in the order a microscopist works:
1. Load the slide: hold the slide holder's spring arm open and lay the slide in the holder against its stops, label to
   the left; let the arm close over the slide's right end. (Two hands: the arm springs shut when let go.)
2. Bring the specimen under the objective with the stage's x and y drive knobs.
3. Focus: raise the stage with a focus knob until the slide's top is at the objective's working distance, never letting
   it come within 0.5 mm of the objective.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import VISUAL_ONLY, Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STAND = (0.90, 0.90, 0.88)
DARK = (0.16, 0.17, 0.19)
GREY = (0.35, 0.36, 0.38)
BLACK = (0.06, 0.06, 0.07)
STEEL = (0.70, 0.71, 0.73)
GLASS = (0.80, 0.90, 0.95)
SPECIMEN = (0.55, 0.20, 0.45)
WHITE = (0.97, 0.97, 0.97)
RED = (0.85, 0.12, 0.12)
LIGHT = (1.0, 0.98, 0.80)  # the illuminator's spot of light
VISUAL_ONLY.add(LIGHT)
SPOT_R = 0.012  # the light spot: 24 mm across, on the objective's axis, so it shows round the objective from above

# -- the microscope (fixed). Frame: origin on the table under the stand, x away from the robots, y to their left. -----
MX, MY = 0.34, 0.06
BASE = (0.26, 0.16, 0.03)
PILLAR = (0.07, 0.11, 0.185)
PILLAR_X = 0.085
HEAD = (0.18, 0.09, 0.05)  # the arm over the stage
HEAD_TOP = 0.215
STAGE_X = -0.03  # objective axis (local x); the stage is centred under it when its carriages are at zero
OBJ_R, OBJ_TIP = 0.012, 0.120  # the 10x objective: 24 mm across, front lens 12 cm above the table
WD = 0.0052  # working distance: in focus when the slide's top is 5.2 mm below the front lens
START_OFF = 0.012  # the stage starts 12 mm below focus
SLIDE_W, SLIDE_L, SLIDE_T = 0.026, 0.076, 0.002  # x by y by thickness (drawn 2 mm thick; a real slide is 1 mm)
SPEC_Y = -0.010  # the specimen sits 10 mm right of the slide's centre; the label is on the left end
PLATE_TOP0 = OBJ_TIP - WD - START_OFF - SLIDE_T  # top of the stage (x carriage) at the start
STAGE_MAX = START_OFF + WD - 0.0002  # the focus stops just short of the slide touching the objective

# Focus drive housing under the stage, with one knob shaft through it along y.
HOUSING = (0.08, 0.20, 0.052)
FKNOB_X, FKNOB_Z = -0.04, 0.07
FKNOB_R, FKNOB_W = 0.021, 0.020
FKNOB_Y0 = HOUSING[1] / 2
F_LEAD = 0.024  # focus: m of stage travel per turn
F_MIN = -2 * math.pi * STAGE_MAX / F_LEAD

# Stage travel: the x carriage starts 85 mm towards the robots (loading position), the y carriage centred.
QX0, QY0 = -0.085, 0.0
QX_FOCUS, QY_FOCUS = 0.0, -SPEC_Y  # carriage positions that put the specimen on the objective axis
X_LEAD, Y_LEAD = 0.170, 0.040  # stage drive: m per knob turn (turning a knob clockwise seen from above moves the stage
#                                away from the robots for x, to the left for y)
XKNOB = (STAGE_X - 0.075, 0.105)  # x (inner) and y (outer) drive knobs on a bracket out to the stage's left (local xy)
YKNOB = (STAGE_X - 0.075, 0.170)
DKNOB_R, DKNOB_H = 0.017, 0.016
DKNOB_TOP = PLATE_TOP0 - 0.006
LIM = 0.10

# Slide holder on the x carriage: the slide lies with its centre at the holder centre (STAGE_X, 0) against two back
# posts and a left stop; a spring lever pivoting behind its right end presses a jaw on that end, against the stop. Pulling the lever's
# handle (out to the right) back swings the jaw off the slide's end.
STOP_IN = SLIDE_L / 2 + 0.0005  # the left stop's inner face (y): a loaded slide is pushed against it
PUSHED_DY = STOP_IN - SLIDE_L / 2  # so a loaded slide's centre sits this far left of the holder centre
PIVOT = (STAGE_X + SLIDE_W / 2 + 0.012, STOP_IN - SLIDE_L + 0.0008 - 0.003)  # the closed jaw's face presses 0.8 mm
ARM_L = 0.044  # the clamp jaw, across the slide's right end
HANDLE_L = 0.090  # the lever's handle, out to the right, with the post at its end
ARM_Z = PLATE_TOP0 + 0.0003  # the jaw's underside, just clear of the plate: it pushes the slide's right end
POST_R, POST_H = 0.005, 0.020
ARM_OPEN = 1.2


def _ribs(cx, cy, z0, r, h, n=12):
    return [((cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n), z0 + h / 2), 0.0025, h,
             BLACK) for i in range(n)]


FOCUS = Joint(
    "focus", kind="hinge", axis="Y", anchor=(FKNOB_X, 0.0, FKNOB_Z), lower=F_MIN - 0.3, upper=0.3, initial=0.0,
    stiffness=0.0, damping=0.3, target=0.0, mass=0.05,
    cylinders=[
        ((FKNOB_X, 0.0, FKNOB_Z), 0.006, 2 * FKNOB_Y0 + 2 * FKNOB_W - 0.002, STEEL, "Y"),
        ((FKNOB_X, FKNOB_Y0 + FKNOB_W / 2, FKNOB_Z), FKNOB_R, FKNOB_W, BLACK, "Y"),
        ((FKNOB_X, -(FKNOB_Y0 + FKNOB_W / 2), FKNOB_Z), FKNOB_R, FKNOB_W, BLACK, "Y"),
        *[((FKNOB_X + FKNOB_R * math.cos(i * math.pi / 6), s * (FKNOB_Y0 + FKNOB_W / 2),
            FKNOB_Z + FKNOB_R * math.sin(i * math.pi / 6)), 0.0025, FKNOB_W, BLACK, "Y")
          for s in (1, -1) for i in range(12)],
    ],
    boxes=[((FKNOB_X, s * (FKNOB_Y0 + FKNOB_W + 0.0005), FKNOB_Z + FKNOB_R / 2), (0.003, 0.001, FKNOB_R), WHITE)
           for s in (1, -1)],
)
STAGE = Joint(  # the focus slide: carries the stage, the drive bracket and knobs
    "stage", kind="slide", axis="Z", anchor=(STAGE_X, 0.0, PLATE_TOP0), lower=0.0, upper=STAGE_MAX, initial=0.0,
    stiffness=20000.0, damping=400.0, target=0.0, mass=0.05,
    boxes=[
        ((STAGE_X + 0.075, 0.0, PLATE_TOP0 - 0.03), (0.03, 0.07, 0.03), DARK),  # carrier, into the pillar
        ((STAGE_X, 0.0, PLATE_TOP0 - 0.0135), (0.12, 0.15, 0.005), DARK),  # fixed lower plate
        ((STAGE_X - 0.075, 0.1275, DKNOB_TOP - DKNOB_H - 0.006), (0.03, 0.105, 0.012), DARK),  # drive bracket, from under the stage
    ],
    # The illuminator's light, from below through the stage, shows as a spot on the slide on the objective's axis. It
    # rises with the stage (the condenser does) and is only drawn: nothing collides with it.
    # It is drawn just over the glass and under the specimen square's top, so the square shows on the light.
    cylinders=[((STAGE_X, 0.0, PLATE_TOP0 + SLIDE_T + 0.0001), SPOT_R, 0.0001, LIGHT)],
)
STAGE_Y = Joint(
    "stage_y", kind="slide", axis="Y", anchor=(STAGE_X, 0.0, PLATE_TOP0), lower=-LIM, upper=LIM, initial=QY0,
    stiffness=20000.0, damping=400.0, target=QY0, mass=0.05, parent="stage",
    boxes=[((STAGE_X, 0.0, PLATE_TOP0 - 0.0085), (0.11, 0.14, 0.005), GREY)],
)
STAGE_X_J = Joint(
    "stage_x", kind="slide", axis="X", anchor=(STAGE_X, 0.0, PLATE_TOP0), lower=-LIM, upper=LIM, initial=QX0,
    stiffness=20000.0, damping=400.0, target=QX0, mass=0.05, parent="stage_y",
    boxes=[
        ((STAGE_X, 0.0, PLATE_TOP0 - 0.003), (0.10, 0.13, 0.006), DARK),  # top plate
        *[((STAGE_X + SLIDE_W / 2 + 0.004, y, PLATE_TOP0 + 0.0004), (0.004, 0.006, 0.0008), STEEL)
          for y in (-0.020, 0.005)],  # low back posts, 2 mm behind the slide's back edge
        ((STAGE_X, STOP_IN + 0.002, PLATE_TOP0 + 0.0015), (0.012, 0.004, 0.003), STEEL),  # left stop
        ((STAGE_X - 0.02, 0.0, PLATE_TOP0 + 0.0002), (0.002, 0.03, 0.0004), WHITE),  # holder index line
    ],
)
HOLDER = Joint(
    "holder", kind="hinge", axis="Z", anchor=(*PIVOT, ARM_Z), lower=0.0, upper=ARM_OPEN, initial=0.0,
    stiffness=0.25, damping=0.03, target=0.0, mass=0.02, parent="stage_x",
    boxes=[((PIVOT[0] - ARM_L / 2, PIVOT[1], ARM_Z + 0.00135), (ARM_L, 0.006, 0.0027), STEEL),  # jaw
           ((PIVOT[0], PIVOT[1] - HANDLE_L / 2, ARM_Z + 0.0015), (0.006, HANDLE_L, 0.003), STEEL)],  # handle
    cylinders=[((PIVOT[0], PIVOT[1], ARM_Z + 0.0015), 0.005, 0.003, STEEL),
               ((PIVOT[0], PIVOT[1] - HANDLE_L + 0.003, ARM_Z + 0.003 + POST_H / 2), POST_R, POST_H, BLACK)],
)


def _drive_knob(name, xy):
    cx, cy = xy
    return Joint(
        name, kind="hinge", axis="Z", anchor=(cx, cy, DKNOB_TOP - DKNOB_H), lower=-2 * math.pi, upper=2 * math.pi,
        initial=0.0, stiffness=0.0, damping=0.1, target=0.0, mass=0.02, parent="stage",
        cylinders=[((cx, cy, DKNOB_TOP - DKNOB_H / 2), DKNOB_R, DKNOB_H, BLACK),
                   *_ribs(cx, cy, DKNOB_TOP - DKNOB_H, DKNOB_R, DKNOB_H)],
        boxes=[((cx - DKNOB_R / 2, cy, DKNOB_TOP + 0.0004), (DKNOB_R, 0.002, 0.0008), WHITE)],
    )


XDRIVE, YDRIVE = _drive_knob("xdrive", XKNOB), _drive_knob("ydrive", YKNOB)

MICROSCOPE = part(
    "microscope", pos=(MX, MY, TOP), fixed=True, friction=1.0,
    boxes=[
        ((0.0, 0.0, BASE[2] / 2), BASE, STAND),
        ((PILLAR_X, 0.0, BASE[2] + PILLAR[2] / 2), PILLAR, STAND),
        ((PILLAR_X + PILLAR[0] / 2 - HEAD[0] / 2, 0.0, HEAD_TOP - HEAD[2] / 2), HEAD, STAND),
        ((FKNOB_X, 0.0, 0.03 + HOUSING[2] / 2), HOUSING, STAND),
    ],
    cylinders=[
        ((STAGE_X, 0.0, HEAD_TOP - HEAD[2] - 0.006), 0.032, 0.012, STEEL),  # nosepiece
        ((STAGE_X, 0.0, (OBJ_TIP + HEAD_TOP - HEAD[2] - 0.012) / 2), OBJ_R, HEAD_TOP - HEAD[2] - 0.012 - OBJ_TIP, STEEL),
        ((STAGE_X, 0.0, OBJ_TIP + 0.004), OBJ_R + 0.0005, 0.002, RED),  # the objective's colour ring
    ],
    joints=[FOCUS, STAGE, STAGE_Y, STAGE_X_J, HOLDER, XDRIVE, YDRIVE],
)

# -- the slide (loose) on its rest -----------------------------------------------------------------------------------
REST_XY = (0.18, 0.26)
REST = (0.016, 0.060, 0.030)  # narrower than the slide, so fingers can close on the slide's edges
SLIDE_REST = part("slide_rest", pos=(*REST_XY, TOP), fixed=True,
                  boxes=[((0.0, 0.0, REST[2] / 2), REST, GREY)])
SLIDE = part(
    "slide", pos=(*REST_XY, TOP + REST[2]), mass=0.006, friction=0.6,
    boxes=[
        ((0.0, 0.0, SLIDE_T / 2), (SLIDE_W, SLIDE_L, SLIDE_T), GLASS),
        ((0.0, SPEC_Y, SLIDE_T + 0.0002), (0.010, 0.010, 0.0004), SPECIMEN),
        ((0.0, 0.025, SLIDE_T + 0.0002), (0.024, 0.020, 0.0004), WHITE),  # label
    ],
)

OBJECTS = [MICROSCOPE, SLIDE_REST, SLIDE]


# -- drives: the stage follows its knobs ------------------------------------------------------------------------------
def carriages(j: dict) -> tuple[float, float, float]:
    """(x, y, z) stage positions the knobs ask for: z from the focus knob, x and y from the drive knobs."""
    qz = float(np.clip(-F_LEAD * j["focus"] / (2 * math.pi), 0.0, STAGE_MAX))
    qx = float(np.clip(QX0 - X_LEAD * j["xdrive"] / (2 * math.pi), -LIM, LIM))
    qy = float(np.clip(QY0 - Y_LEAD * j["ydrive"] / (2 * math.pi), -LIM, LIM))
    return qx, qy, qz


def stage_gap(qz: float) -> float:
    """Clearance (m) from a loaded slide's top to the objective's front lens."""
    return OBJ_TIP - (PLATE_TOP0 + SLIDE_T + qz)


_trial = {"min_gap": None}


@on_reset
def _clear(env):
    _trial["min_gap"] = None


@on_step
def _drive(env):
    import torch

    import labkit

    s = labkit._part_state(env, labkit._SPECS["microscope"])
    sl = labkit._part_state(env, labkit._SPECS["slide"])
    g = stage_gap(s.joints["stage"])
    if _loaded(s.joints, sl):
        _trial["min_gap"] = g if _trial["min_gap"] is None else min(_trial["min_gap"], g)
    asset = env.unwrapped.scene["microscope"]
    names = list(asset.joint_names)
    ids = [names.index(j.joint_name) for j in (STAGE_X_J, STAGE_Y, STAGE)]
    qx, qy, qz = carriages(s.joints)
    asset.set_joint_position_target(torch.tensor([[qx, qy, qz]], device=asset.device), joint_ids=ids)


# -- scoring ----------------------------------------------------------------------------------------------------------
OBJ_AXIS = (MX + STAGE_X, MY)
HOLD_TOL, HOLD_Z, FLAT_DEG, CLOSED = 0.003, 0.001, 3.0, 0.10
CENTRE_TOL = 0.0015
CRASH_GAP = 0.0005
FOCUS_TOL, NEAR_TOL = 0.0005, 0.002


def holder_centre(j: dict) -> np.ndarray:
    """World position of the holder centre (the middle of a loaded slide's underside)."""
    return np.array([MX + STAGE_X + j["stage_x"], MY + PUSHED_DY + j["stage_y"], TOP + PLATE_TOP0 + j["stage"]])


def _loaded(j: dict, slide) -> bool:
    d = slide.pos - holder_centre(j)
    return (math.hypot(d[0], d[1]) <= HOLD_TOL and abs(d[2]) <= HOLD_Z and slide.tilt_deg() <= FLAT_DEG
            and abs(slide.to_world((0, 1, 0))[1] - slide.pos[1]) > 0.99 and j["holder"] <= CLOSED)


def specimen_offset(slide) -> float:
    p = slide.to_world((0.0, SPEC_Y, 0.0))
    return math.hypot(p[0] - OBJ_AXIS[0], p[1] - OBJ_AXIS[1])


def focus_error(slide) -> float:
    top = slide.to_world((0.0, 0.0, SLIDE_T))[2]
    return abs((TOP + OBJ_TIP - top) - WD)


def score(objects):
    """0 unless the slide is loaded: flat in the holder within 2 mm of its centre, label to the left, holder arm closed
    over it. 0 also if, once loaded, the slide ever came within 0.5 mm of the objective. 0.25 loaded; 0.5 loaded and
    the specimen within 1.5 mm of the objective's axis; 0.75 also within 2 mm of focus (slide top 5.2 mm below the front
    lens); 1.0 within 0.5 mm of focus."""
    m, sl = objects["microscope"], objects["slide"]
    if not _loaded(m.joints, sl):
        return 0.0
    if _trial["min_gap"] is not None and _trial["min_gap"] < CRASH_GAP:
        return 0.0
    if specimen_offset(sl) > CENTRE_TOL:
        return 0.25
    e = focus_error(sl)
    return 1.0 if e <= FOCUS_TOL else 0.75 if e <= NEAR_TOL else 0.5


SUCCESS_HOLD_S = 15.0  # end a trial early only once it has stayed in focus for 15 s (time to let go of the knob and park)


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return specimen_offset(objects["slide"]) + focus_error(objects["slide"])


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: picks the slide off its rest by the label end and lays it in the holder while the right arm holds the
# holder's arm open by its post; then parks. Right arm: lets the holder arm close, turns the y knob a quarter turn and
# the x knob half a turn (two quarter turns) clockwise, then grips the right focus knob from the side and turns it half
# a turn, top towards the robots. The task file lets a Waypoint's ``yaw`` carry a full gripper rotation (``Orient``).


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


def side(roll: float) -> Orient:
    return Orient(axis_angle_matrix(np.array([0.0, 1.0, 0.0]), roll) @ SIDE)


class Servo(Orient):
    """A Waypoint ``yaw`` whose gripper rotation turns a knob until the knob reaches ``target`` (closed loop): an
    ``on_step`` hook moves the commanded wrist angle by the remaining knob error, a little each step, between ``t0``
    and ``t1`` (seconds from the start of the demo). ``rot_of`` maps the wrist angle to a gripper rotation."""

    def __new__(cls, joint, target, start, rot_of):
        obj = float.__new__(cls, 0.0)
        obj.joint, obj.target, obj.start, obj.rot_of = joint, target, start, rot_of
        obj.cmd, obj.t0, obj.t1, obj.a0 = start, 0.0, 0.0, None
        return obj

    @property
    def rot(self):
        return self.rot_of(self.cmd)


_SERVOS: list = []
_clock = {"t": 0.0}
SERVO_GAIN, SERVO_MAX = 0.2, 0.02  # rad of wrist per step per rad of knob error; at most 0.02 rad (0.4 rad/s)


@on_reset
def _servo_reset(env):
    _clock["t"] = 0.0
    for sv in _SERVOS:
        sv.cmd, sv.a0 = sv.start, None


@on_step
def _servo_step(env):
    import labkit

    _clock["t"] += float(getattr(env.unwrapped, "step_dt", 0.05))
    active = [sv for sv in _SERVOS if sv.t0 <= _clock["t"] <= sv.t1]
    if not active:
        return
    j = labkit._part_state(env, labkit._SPECS["microscope"]).joints
    for sv in active:
        err = sv.target - j[sv.joint]
        sv.cmd += float(np.clip(SERVO_GAIN * err, -SERVO_MAX, SERVO_MAX))


def _servo(wps, duration, xyz, finger, label, joint, target, start, rot_of):
    """Append a closed-loop knob turn of ``duration`` seconds to ``wps``: a 0.05 s step that lands on the servo's
    wrist angle (equal to ``start``, the previous waypoint's), then a hold on the same servo, so the commanded wrist
    angle is used as it is, not interpolated."""
    sv = Servo(joint, target, start, rot_of)
    wps.append(Waypoint(0.05, xyz, finger, label, sv))
    sv.t0 = sum(w.duration for w in wps)
    sv.t1 = sv.t0 + duration
    _SERVOS.append(sv)
    wps.append(Waypoint(duration, xyz, finger, label, sv))
    return sv


def _down(yaw):
    return _down_grip(yaw)


TIP_ABOVE_TCP = 0.013
LEFT_PARK = (0.15, 0.25, TOP + 0.14)
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)
RIGHT_ASIDE = (0.18, -0.16, TOP + 0.12)
OPEN_TH = 0.95  # how far the holder lever is pulled (rad): enough to swing the jaw off the slide's end


def W(x, y, z):
    """Local microscope coordinates to world."""
    return (MX + x, MY + y, TOP + z)


def _push_at(theta: float):
    """TCP xy for closed fingers pushing the post back, on its front side, when the lever is open by theta."""
    px, py = _post_at(theta)
    return px - (POST_R + 0.010) * math.cos(theta), py - (POST_R + 0.010) * math.sin(theta)


def _post_at(theta: float, qx=QX0, qy=QY0):
    """World xy of the holder arm's post when the arm is open by theta."""
    r = HANDLE_L - 0.003
    px, py = PIVOT[0] + r * math.sin(theta), PIVOT[1] - r * math.cos(theta)
    return MX + px + qx, MY + py + qy


POST_GRIP_Z = TOP + ARM_Z + 0.003 + POST_H * 0.55 + TIP_ABOVE_TCP
SLIDE_GRIP_DY = 0.031  # grip the slide 31 mm left of its centre (on the label end)
REST_GRIP_Z = TOP + REST[2] - 0.003 + TIP_ABOVE_TCP  # pad tips 3 mm below the slide, beside the narrow rest
PLACE = (MX + STAGE_X + QX0 - 0.001, MY + QY0 - 0.002)  # 1 mm in front of and 2 mm right of where it ends
PLACE_Z = TOP + PLATE_TOP0 + 0.0005 + TIP_ABOVE_TCP  # pad tips 0.5 mm above the plate: the slide drops 3.5 mm
CARRY_Z = TOP + PLATE_TOP0 + 0.05
DK_GRIP_Z = TOP + DKNOB_TOP - DKNOB_H / 2 + TIP_ABOVE_TCP
DK_HOVER = DK_GRIP_Z + 0.05
F_GRIP = W(FKNOB_X, -(FKNOB_Y0 + FKNOB_W) + 0.009 - TIP_ABOVE_TCP, FKNOB_Z)
F_OUT = (F_GRIP[0], F_GRIP[1] - 0.04, F_GRIP[2])
F_OUT_UP = (F_GRIP[0], F_GRIP[1] - 0.04, F_GRIP[2] + 0.04)
F_TURN = -2 * math.pi * START_OFF / F_LEAD
F_TARGET = F_TURN  # focus knob angle for the working distance
X_TARGET = -2 * math.pi * (QX_FOCUS - QX0) / X_LEAD  # drive knob angles that centre the specimen
Y_TARGET = -2 * math.pi * (QY_FOCUS - PUSHED_DY - QY0) / Y_LEAD
WIDE = -0.025  # fingers part-open round the post or the slide
SLIDE_RELEASE = -0.016  # open just enough to free the slide, clear of the objective
FOCUS_WIDE = -0.03  # part-open round the 47 mm focus knob, clear of the table
KNOB_WIDE = -0.030  # part-open round a 39 mm ribbed drive knob
KNOB_GRIP = -0.004  # closed on a drive knob
KNOB_RELEASE = -0.022  # opened just off the knob, clear of its neighbour

PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    ("base", W(0.0, 0.0, BASE[2] / 2), BASE),
    ("housing", W(FKNOB_X, 0.0, 0.03 + HOUSING[2] / 2), HOUSING),
    ("pillar", W(PILLAR_X, 0.0, BASE[2] + PILLAR[2] / 2), PILLAR),
    ("objective", W(STAGE_X, 0.0, (OBJ_TIP + HEAD_TOP - HEAD[2]) / 2), (0.024, 0.024, HEAD_TOP - HEAD[2] - OBJ_TIP)),
    ("head", W(PILLAR_X + PILLAR[0] / 2 - HEAD[0] / 2, 0.0, HEAD_TOP - HEAD[2] / 2), HEAD),
    ("stage at load", W(STAGE_X + QX0, QY0, PLATE_TOP0 - 0.003), (0.10, 0.13, 0.006)),
    ("slide rest", (*REST_XY, TOP + REST[2] / 2), REST),
]


def _dur(wps):
    return sum(w.duration for w in wps)


def _knob_turn(wps, xy, label, joint, target, servo):
    """A clockwise turn of a top-down drive knob: grip at yaw 90 degrees (fingers closing along x) and turn; an open
    quarter turn to yaw 0, or (``servo``) a closed-loop turn that stops at the knob angle ``target``. Then let go and
    lift."""
    z = DK_GRIP_Z
    q = math.pi / 2
    wps += [
        Waypoint(1.5, (*xy, DK_HOVER), KNOB_WIDE, f"above the {label}", q),
        Waypoint(1.0, (*xy, z), KNOB_WIDE, f"round the {label}", q),
        Waypoint(0.8, (*xy, z), KNOB_GRIP, "grip", q),
    ]
    if servo:
        sv = _servo(wps, 5.0, (*xy, z), KNOB_GRIP, f"turn the {label} clockwise to its mark", joint, target, q, _down)
        end = sv
    else:
        wps.append(Waypoint(4.0, (*xy, z), KNOB_GRIP, f"turn the {label} a quarter turn clockwise", 0.0))
        end = 0.0
    wps += [
        Waypoint(0.6, (*xy, z), KNOB_RELEASE, "let go", end),
        Waypoint(0.8, (*xy, DK_HOVER), KNOB_RELEASE, "up", end),
        Waypoint(0.8, (*xy, DK_HOVER), KNOB_RELEASE, "wrist back", q),
    ]


def demo():
    _SERVOS.clear()
    sx, sy = REST_XY
    gx, gy = PLACE
    # Right arm: pushes the lever's post back with closed fingers (the spring holds it against them), holds it while
    # the left arm lays the slide in, then eases off so the jaw closes, and later focuses.
    right = [
        Waypoint(2.5, (*_push_at(0.0), POST_GRIP_Z + 0.04), FINGER_CLOSED, "above the front of the lever post"),
        Waypoint(1.0, (*_push_at(0.0), POST_GRIP_Z), FINGER_CLOSED, "down in front of the post"),
    ]
    for th in (0.3, 0.6, OPEN_TH):
        right.append(Waypoint(0.8, (*_push_at(th), POST_GRIP_Z), FINGER_CLOSED, "push the lever back", th))
    t_open = _dur(right)
    left = [
        Waypoint(2.5, (sx, sy + SLIDE_GRIP_DY, REST_GRIP_Z + 0.05), FINGER_OPEN, "above the slide's label", -math.pi / 2),
        Waypoint(1.2, (sx, sy + SLIDE_GRIP_DY, REST_GRIP_Z), WIDE, "fingers either side of the slide", -math.pi / 2),
        Waypoint(0.8, (sx, sy + SLIDE_GRIP_DY, REST_GRIP_Z), FINGER_CLOSED, "grip its edges", -math.pi / 2),
        Waypoint(1.0, (sx, sy + SLIDE_GRIP_DY, CARRY_Z), FINGER_CLOSED, "lift", -math.pi / 2),
    ]
    left.append(Waypoint(max(0.3, t_open - _dur(left)), (sx, sy + SLIDE_GRIP_DY, CARRY_Z), FINGER_CLOSED,
                         "wait for the holder to open", -math.pi / 2))
    left += [
        Waypoint(2.0, (gx, gy + SLIDE_GRIP_DY, CARRY_Z), FINGER_CLOSED, "over the holder", -math.pi / 2),
        Waypoint(1.5, (gx, gy + SLIDE_GRIP_DY, PLACE_Z), FINGER_CLOSED, "lower into the holder", -math.pi / 2),
        Waypoint(0.6, (gx, gy + SLIDE_GRIP_DY, PLACE_Z), SLIDE_RELEASE, "let go", -math.pi / 2),
        Waypoint(1.0, (gx, gy + SLIDE_GRIP_DY, CARRY_Z), SLIDE_RELEASE, "up", -math.pi / 2),
        Waypoint(1.5, (*W(*YKNOB, 0.0)[:2], DK_HOVER + 0.03), KNOB_WIDE, "towards the drive knobs"),
    ]
    t_left_clear = _dur(left) - 1.0
    right.append(Waypoint(max(0.3, t_left_clear - _dur(right)), (*_push_at(OPEN_TH), POST_GRIP_Z), FINGER_CLOSED,
                          "hold the lever back until the left arm is clear", OPEN_TH))
    for th in (0.6, 0.3, 0.0):
        right.append(Waypoint(0.8, (*_push_at(th), POST_GRIP_Z), FINGER_CLOSED, "ease off: the jaw closes", th))
    right += [
        Waypoint(0.6, (*_push_at(-0.15), POST_GRIP_Z), FINGER_CLOSED, "clear of the post"),
        Waypoint(0.8, (*_push_at(-0.15), POST_GRIP_Z + 0.04), FINGER_CLOSED, "up"),
        Waypoint(1.5, RIGHT_ASIDE, FINGER_OPEN, "aside"),
    ]
    # Left arm: the stage drives, once the jaw has closed over the slide.
    left.append(Waypoint(max(0.3, _dur(right) - _dur(left)), (*W(*YKNOB, 0.0)[:2], DK_HOVER + 0.03), KNOB_WIDE,
                         "wait for the jaw to close"))
    _knob_turn(left, W(*YKNOB, 0.0)[:2], "y knob", "ydrive", Y_TARGET, servo=True)
    _knob_turn(left, W(*XKNOB, 0.0)[:2], "x knob", "xdrive", X_TARGET, servo=False)
    _knob_turn(left, W(*XKNOB, 0.0)[:2], "x knob", "xdrive", X_TARGET, servo=True)
    left.append(Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"))
    right.append(Waypoint(max(0.3, _dur(left) - 2.0 - _dur(right)), RIGHT_ASIDE, FINGER_OPEN,
                          "wait while the left arm drives the stage"))
    right += [
        Waypoint(2.0, F_OUT_UP, KNOB_WIDE, "above, beside the right focus knob"),
        Waypoint(1.5, F_OUT, FOCUS_WIDE, "level the gripper", side(0.0)),
        Waypoint(1.5, F_GRIP, FOCUS_WIDE, "onto the focus knob", side(0.0)),
        Waypoint(0.8, F_GRIP, FINGER_CLOSED, "grip the rim", side(0.0)),
    ]
    for k in range(1, 6):  # most of the way open loop, in 1.2 s steps of 0.18 turn
        right.append(Waypoint(1.2, F_GRIP, FINGER_CLOSED, "focus: turn, top towards the robots",
                              side(0.9 * F_TARGET * k / 5)))
    fsv = _servo(right, 5.0, F_GRIP, FINGER_CLOSED, "focus: finish the turn until in focus", "focus",
                 F_TARGET, 0.9 * F_TARGET, lambda r: side(r).rot)
    right += [
        Waypoint(0.6, F_GRIP, FINGER_CLOSED, "settle", fsv),
        Waypoint(0.8, F_GRIP, FOCUS_WIDE, "let go", fsv),
        Waypoint(1.0, F_OUT, FOCUS_WIDE, "back off", fsv),
        Waypoint(1.2, F_OUT, FOCUS_WIDE, "unroll", side(F_TURN / 2)),
        Waypoint(1.2, F_OUT, FOCUS_WIDE, "unroll", side(0.0)),
        Waypoint(1.5, F_OUT_UP, KNOB_WIDE, "point down again"),
        Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"),
        Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "hold"),
    ]
    left.append(Waypoint(max(0.5, _dur(right) - _dur(left)), LEFT_PARK, FINGER_OPEN, "rest"))
    return {"left": left, "right": right}
