"""On the station table: the load side of a motor starter. A light grey backplate carries a black contactor high up and a
grey terminal strip at the table; three stiff 2 AWG motor leads (black T1, red T2, blue T3, 10.5 mm over the insulation,
40 mm apart) run straight up between them. On a dark tray to the right lies a yellow clamp meter with a black sprung jaw
(26 mm window) and a black trigger lever on its side. The meter is held in one gripper while the other squeezes the
trigger against the neck to open the jaw; the open jaw goes round the red middle lead, the trigger is let go so the jaw
shuts round that lead alone, the meter is kept still 3 s, then opened, drawn off and laid back on its tray.

Requires -E object_jitter=0: the tray position decides the scored end pose.

Run it:  inspect-robots run --embodiment yam_arena -E task=shut_a_clamp_meter_round_one_motor_lead.py \\
           -E object_jitter=0 --instruction "Take the red middle lead's current: shut the clamp meter's jaw round it
           for 3 s, then put the meter back on its tray." --scorer task_score ...
"""

import dataclasses
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import closedloop  # noqa: E402, F401
import mechkit  # noqa: E402
from belimits import tilt_deg  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, arc, box, cyl, extrude, finger_clearance, finger_points, hull, mass_props, on_reset  # noqa: E402

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips the meter: let it open the gripper and park clear before the episode ends

YELLOW, BLACK, DARK, MID, LIGHT, WHITE = (0.98, 0.78, 0.05), (0.04, 0.04, 0.05), (0.20, 0.21, 0.23), (0.45, 0.46, 0.48), (0.80, 0.81, 0.83), (0.96, 0.96, 0.94)
RED, BLUE = (0.90, 0.05, 0.05), (0.05, 0.25, 0.90)


def _look(shape):
    """A look-only part: shown, never collides."""
    return dataclasses.replace(shape, collide=False)


def _hidden(shape):
    """A collider that is not drawn (the battery block inside the handle)."""
    return dataclasses.replace(shape, visible=False)


# -- the starter: backplate, contactor, terminal strip and the three motor leads (all fixed) ---------------------------------
LEAD_X, LEAD_R, PITCH = 0.340, 0.00525, 0.040  # 2 AWG THHN, 10.5 mm over the insulation, 40 mm between centres
LEAD_Y = {"T1": PITCH, "T2": 0.0, "T3": -PITCH}
LEAD_COLOR = {"T1": BLACK, "T2": RED, "T3": BLUE}
STRIP_TOP, CONT_Z = 0.060, (0.360, 0.440)  # heights above the table: the leads run free between them (30 cm)
RUN_Z = (0.090, 0.330)  # the jaw must sit on this part of the free run, 3 cm clear of the strip and the contactor
STARTER = Mech("starter", pos=(0.0, 0.0, TOP), mass=None, friction=0.5, shapes=[
    box((0.393, 0.0, 0.230), (0.006, 0.300, 0.460), LIGHT),  # backplate
    box((0.360, 0.0, (CONT_Z[0] + CONT_Z[1]) / 2), (0.060, 0.140, CONT_Z[1] - CONT_Z[0]), BLACK),  # contactor
    box((0.355, 0.0, STRIP_TOP / 2), (0.070, 0.140, STRIP_TOP), MID),  # terminal strip
    *[cyl((LEAD_X, y, (STRIP_TOP + CONT_Z[0]) / 2), LEAD_R, CONT_Z[0] - STRIP_TOP, LEAD_COLOR[n]) for n, y in LEAD_Y.items()],
    *[_look(box((LEAD_X, y, STRIP_TOP + 0.001), (0.022, 0.022, 0.002), WHITE)) for y in LEAD_Y.values()],  # lugs
    *[_look(cyl((0.329, y, 0.400), 0.006, 0.003, WHITE, axis="x")) for y in LEAD_Y.values()],  # contactor screw heads
])

# -- the tray ------------------------------------------------------------------------------------------------------------
TRAY_C, TRAY_SIZE = (0.250, -0.150), (0.240, 0.090, 0.006)
TRAY = Box("tray", pos=(TRAY_C[0], TRAY_C[1], TOP + TRAY_SIZE[2] / 2), size=TRAY_SIZE, color=DARK)

# -- the clamp meter (free) ------------------------------------------------------------------------------------------------
# Frame: origin O at the centre of the jaw window, on the jaw's mid-plane; x toward the jaw tip, y to the left, z up.
R_IN, R_OUT, JAW_H = 0.013, 0.0195, 0.024  # 26 mm window, jaw section 6.5 x 24 mm, head 39 mm across
SPLIT = np.radians(30.0)  # the two jaw tips meet 30 degrees left of the nose
BACK = np.radians(-120.0)  # the moving half runs from here to the tips
PIVOT = (-0.010, -0.022)  # jaw hinge, at the base of the moving half
OPEN_STOP = 0.70  # rad: the trigger is flush with the neck here and the tips are about 20 mm apart
NECK_HALF, BODY_HALF, THICK = 0.023, 0.033, 0.040
NECK_X, BODY_X = (-0.014, -0.070), (-0.080, -0.195)
_OUTLINE = [(NECK_X[0], -NECK_HALF), (NECK_X[0], NECK_HALF), (NECK_X[1], NECK_HALF), (BODY_X[0], BODY_HALF),
            (BODY_X[1], BODY_HALF), (BODY_X[1], -BODY_HALF), (BODY_X[0], -BODY_HALF), (NECK_X[1], -NECK_HALF)]
PAD_R, LEVER = 0.006, 0.032  # trigger pad radius; pad centre this far behind the pivot


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Trigger pad centre in the pivot frame: flush with the neck's right face at the open stop, swung out when the jaw is shut.
_PAD_OPEN = np.array([-LEVER, -NECK_HALF + PAD_R - PIVOT[1], 0.0])
_PAD_SHUT = _rz(OPEN_STOP) @ _PAD_OPEN
_n = np.array([-_PAD_SHUT[1], _PAD_SHUT[0], 0.0]) / np.linalg.norm(_PAD_SHUT[:2]) * 0.003
_LEVER_PTS = [tuple(p + sn * _n + np.array([0.0, 0.0, z])) for p in (np.zeros(3), _PAD_SHUT) for sn in (-1, 1) for z in (-0.010, 0.010)]
PAD_SHUT_O = np.array([PIVOT[0], PIVOT[1], 0.0]) + _PAD_SHUT  # the pad centre in the meter frame, jaw shut

_BASE_SHAPES = [
    extrude((0.0, 0.0, 0.0), _OUTLINE, THICK, YELLOW, axis="z"),
    arc((0.0, 0.0, 0.0), (R_IN + R_OUT) / 2, SPLIT, 2 * np.pi + BACK, BLACK, section=(R_OUT - R_IN, JAW_H), axis="z"),
    # two 9 V-size battery stacks' worth of mass in the handle, as real meters carry their cells there
    *[_hidden(box((-0.158, 0.0, 0.0), (0.070, 0.060, 0.036), YELLOW)) for _ in range(4)],
    _look(box((-0.110, 0.0, THICK / 2 + 0.001), (0.050, 0.040, 0.002), DARK)),  # display
    _look(cyl((-0.165, 0.0, THICK / 2 + 0.002), 0.015, 0.004, BLACK)),  # range dial
]
_JAW_SHAPES = [
    arc((-PIVOT[0], -PIVOT[1], 0.0), (R_IN + R_OUT) / 2, BACK, SPLIT, BLACK, section=(R_OUT - R_IN, JAW_H), axis="z"),
    hull((0.0, 0.0, 0.0), _LEVER_PTS, BLACK),
    cyl(tuple(_PAD_SHUT), PAD_R, JAW_H, BLACK, sides=12),
]
BASE_MASS, JAW_MASS = 0.215, 0.025
COM_X = float((BASE_MASS * mass_props(_BASE_SHAPES)[1][0] + JAW_MASS * (PIVOT[0] + mass_props(_JAW_SHAPES)[1][0])) / (BASE_MASS + JAW_MASS))
REST_Z = TRAY_SIZE[2] + THICK / 2  # O this far above the table when the meter lies on its tray
METER_START = (0.337, TRAY_C[1], TOP + REST_Z)
# Jaw spring 0.15 N m/rad from the shut stop (2.1 N at the trigger pad when fully squeezed), lightly damped: it shuts in
# about 0.2 s. The hinge is vertical, so gravity never loads it.
METER = Mech("clamp_meter", pos=METER_START, mass=BASE_MASS, friction=0.6, shapes=_BASE_SHAPES, joints=[
    Joint("jaw", "hinge", axis="z", at=(PIVOT[0], PIVOT[1], 0.0), low=-OPEN_STOP, high=0.0, spring=0.15, damping=0.006,
          mass=JAW_MASS, friction=0.6, shapes=_JAW_SHAPES)])

OBJECTS = [STARTER, TRAY, METER]

# -- scoring -----------------------------------------------------------------------------------------------------------------
SHUT_Q, OPEN_Q = 0.05, 0.30  # rad: the jaw counts as shut at or under 0.05, as opened at or over 0.30
IN_R, IN_TILT = 0.009, 20.0  # window centre within 9 mm of a lead's axis, window axis within 20 degrees of the lead
HOLD_STEPS, HOLD_MOVE, HOLD_TURN = 60, 0.005, 3.0  # 3.0 s at 20 Hz; the meter may move 5 mm and turn 3 degrees in it
TRIGGER_CLEAR, RELEASE_CLEAR = 0.020, 0.030
DROP_V = 0.4  # m/s downward: the meter fell
TRAY_XY, TRAY_Z, TRAY_TILT, TRAY_YAW = 0.020, 0.006, 10.0, 20.0
STILL_V, STILL_STEPS = 0.01, 20
METER_LO, METER_HI = (BODY_X[1], -0.047, -THICK / 2), (R_OUT + 0.012, BODY_HALF, THICK / 2)  # the whole meter, its frame
TRIG_LO, TRIG_HI = (-0.046, -0.047, -JAW_H / 2), (-0.008, -NECK_HALF, JAW_H / 2)  # the trigger lever, jaw shut
OFFLINE_FINGERS_CLEAR = True  # what the finger tests return with no simulator (the checker flips it for a held meter)


class _Latch:
    def __init__(self):
        self.clear()

    def clear(self):
        self.taken = self.wrong = self.fell = self.threaded = False
        self.anchor, self.count, self.still = None, 0, 0
        self.seen = None


LATCH = _Latch()


@on_reset
def _reset_latch(env=None):
    LATCH.clear()


def jaw_open(m) -> float:
    """How far the jaw is open (rad, 0 shut)."""
    return -float(m.joints.get("jaw", 0.0))


def lead_in_window(m, name: str) -> bool:
    """The window centre is within 9 mm of this lead's axis, on its free run, with the window axis within 20 degrees of it."""
    o = np.asarray(m.origin, float)
    return bool(np.hypot(o[0] - LEAD_X, o[1] - LEAD_Y[name]) <= IN_R and TOP + RUN_Z[0] <= o[2] <= TOP + RUN_Z[1]
                and tilt_deg(m) <= IN_TILT)


def _fingers(m, lo, hi, clearance) -> bool:
    """Every finger point of both grippers is at least `clearance` from this box of the meter. Offline there are none."""
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    try:
        return finger_clearance(finger_points(), m, lo, hi) >= clearance
    except Exception:  # noqa: BLE001  an unknown grip never counts as clear
        return False


def _yaw_deg(m) -> float:
    r = m.rotation()
    return float(np.degrees(np.arctan2(r[1, 0], r[0, 0])))


def _update(objects):
    m = objects["clamp_meter"]
    if LATCH.seen is objects:  # success() and score() are both called on one dict each step: count the step once
        return m
    LATCH.seen = objects
    o = np.asarray(m.origin, float)
    v = float(np.linalg.norm(m.lin_vel))
    q = jaw_open(m)
    shut = q <= SHUT_Q
    if float(m.lin_vel[2]) <= -DROP_V:
        LATCH.fell = True
    if shut and (lead_in_window(m, "T1") or lead_in_window(m, "T3")):
        LATCH.wrong = True
    if q >= OPEN_Q and lead_in_window(m, "T2"):
        LATCH.threaded = True
    if shut and lead_in_window(m, "T2") and _fingers(m, TRIG_LO, TRIG_HI, TRIGGER_CLEAR):
        rot = m.rotation()
        if LATCH.anchor is not None:
            turn = np.degrees(np.arccos(np.clip((np.trace(LATCH.anchor[1].T @ rot) - 1.0) / 2.0, -1.0, 1.0)))
            if np.linalg.norm(o - LATCH.anchor[0]) > HOLD_MOVE or turn > HOLD_TURN:
                LATCH.anchor = None
        if LATCH.anchor is None:
            LATCH.anchor, LATCH.count = (o.copy(), rot.copy()), 0
        LATCH.count += 1
        if LATCH.count >= HOLD_STEPS:
            LATCH.taken = True
    else:
        LATCH.anchor, LATCH.count = None, 0
    LATCH.still = LATCH.still + 1 if v < STILL_V else 0
    return m


def on_tray(m) -> bool:
    """Flat on its tray: centre within 20 mm of the start spot, resting (within 6 mm of the rest height, so a meter held
    in the air is not placed), tilt 10 degrees or less, yaw 20 degrees or less."""
    o = np.asarray(m.origin, float)
    return bool(np.hypot(o[0] - METER_START[0], o[1] - METER_START[1]) <= TRAY_XY and abs(o[2] - METER_START[2]) <= TRAY_Z
                and tilt_deg(m) <= TRAY_TILT and abs(_yaw_deg(m)) <= TRAY_YAW)


def score(objects):
    """0 if the jaw was ever shut round T1 or T3, the meter ever fell (0.4 m/s downward), or a gripper is within 3 cm of
    the meter at the end. Else 1.0: reading taken on T2 (jaw shut round it, trigger untouched, meter still, 3.0 s), then
    the meter flat on its tray, jaw shut, still 1 s. 0.6: reading taken, meter left anywhere else. 0.3: T2 was inside the
    opened jaw at some time but no reading was taken. Else 0."""
    m = _update(objects)
    if LATCH.wrong or LATCH.fell:
        return 0.0
    if not _fingers(m, METER_LO, METER_HI, RELEASE_CLEAR):
        return 0.0
    if LATCH.taken:
        if on_tray(m) and jaw_open(m) <= SHUT_Q and LATCH.still >= STILL_STEPS:
            return 1.0
        return 0.6
    return 0.3 if LATCH.threaded else 0.0


def success(objects):
    return score(objects) == 1.0


# The lugs are the live parts of a starter: both hands stay off the contactor and the terminal strip and work on the
# insulated run between them. One column covers both and the leads (1.5 cm in front of the leads' faces).
ARM_KEEP_OUT = [
    {"name": "starter: contactor, leads and terminal strip", "x": (0.320, 0.400), "y": (-0.080, 0.080), "z_top": TOP + CONT_Z[1] + 0.02},
]

# -- the scripted demo (both arms, closed loop) -------------------------------------------------------------------------------
demo = placeholder_demo

MM = FINGER_OPEN / 0.094  # finger command per metre of pad gap
X_GRIP = -0.160  # the right gripper holds the handle here (the centre of mass is 3.6 cm in front of it)
R_PITCH = 0.30  # the right gripper leans back toward the robot too, which keeps the two wrists apart
X_TRIG = -0.040  # the left gripper straddles the neck and the trigger pad here
GRIP_UP, TRIG_DOWN = 0.005, 0.004  # tool points: right 5 mm above the mid-plane, left 4 mm below it
WORK_Z = 0.110  # the jaw's mid-plane on the leads, above the table
ENTER_D = 0.003  # the meter comes in 3 mm left of T2's axis, where the open tips straddle the lead evenly
ENTER_X = LEAD_X - 0.048  # window centre here before threading: the tips are 2.8 cm short of the leads
L_PITCH = 0.30  # the left gripper leans back toward the robot, which keeps its wrist off the contactor
OPEN_GAP, SHUT_GAP = 0.080, 0.043


def demo_script(ctx):
    """The right arm lifts the meter by its handle and holds it level in front of the leads; the left arm squeezes the
    trigger against the neck until the jaw is open; both feed the open jaw round the red lead; the left lets go so the
    jaw shuts, and leaves; the right keeps the meter still 3.5 s; the left reopens the jaw, both draw the meter off, and
    the right lays it back on its tray."""
    L, R = ctx.left, ctx.right
    live = ctx.states() is not ctx.states()  # an offline replay hands back the same start poses every time
    rot_l = grip_rot(pitch=L_PITCH)
    rot_r = grip_rot(pitch=R_PITCH)
    nom = {"o": np.array(METER_START, float), "q": 0.0}  # where the plan has the meter (used offline)
    trim = {"t": np.zeros(3), "yaw": 0.0}

    def meter():
        if live:
            st = ctx.obj("clamp_meter")
            return np.asarray(st.origin, float), st.rotation(), jaw_open(st)
        return nom["o"].copy(), np.eye(3), nom["q"]

    def hold_at(o, fb=True):
        """Command the right gripper so the window centre goes to o, trimmed by the meter's live pose."""
        if live and fb:
            oa, ra, _ = meter()
            e = np.asarray(o, float) - oa
            trim["t"] = trim["t"] + 0.08 * e
            n = float(np.linalg.norm(trim["t"]))
            if n > 0.045:
                trim["t"] *= 0.045 / n
            trim["yaw"] = float(np.clip(trim["yaw"] - 0.05 * np.arctan2(ra[1, 0], ra[0, 0]), -0.2, 0.2))
        rz = _rz(trim["yaw"])
        R._set_cartesian(np.asarray(o, float) + rz @ np.array([X_GRIP, 0.0, GRIP_UP]) + trim["t"], rz @ rot_r, 0.050 * MM)
        nom["o"] = np.asarray(o, float).copy()

    def neck_at(gap):
        """Command the left gripper over the neck, following the meter: its left pad stays 3 mm off the neck's left
        face until the right pad has pushed the trigger in, so the squeeze is pad against pad."""
        o, r, _ = meter()
        cy = min(0.0, NECK_HALF + 0.003 - gap / 2)
        L._set_cartesian(o + r @ np.array([X_TRIG, cy, -TRIG_DOWN]), rot_l, gap * MM)

    def carry(o1, dur, gap=None, fb=True):
        o0 = nom["o"].copy()
        n = max(1, round(dur / ctx.dt))
        for i in range(1, n + 1):
            hold_at(o0 + closedloop.ik.smoothstep(i / n) * (np.asarray(o1, float) - o0), fb)
            if gap is not None:
                neck_at(gap)
            yield

    def squeeze():
        gap = OPEN_GAP
        while gap > SHUT_GAP:
            gap -= 0.0005
            hold_at(nom["o"])
            neck_at(gap)
            yield
            q = meter()[2]
            if live and q >= OPEN_STOP - 0.06:
                break
            nom["q"] = float(np.clip((OPEN_GAP - 0.012 - gap) / 0.022, 0.0, 1.0)) * OPEN_STOP
        for _ in range(6):
            hold_at(nom["o"])
            neck_at(gap)
            yield
        return gap

    def let_go(gap):
        while gap < OPEN_GAP:
            gap = min(OPEN_GAP, gap + 0.001)
            hold_at(nom["o"])
            neck_at(gap)
            yield
        nom["q"] = 0.0

    def to_neck():
        o, r, _ = meter()
        p = o + r @ np.array([X_TRIG, NECK_HALF + 0.003 - OPEN_GAP / 2, -TRIG_DOWN])
        yield from L.move((p[0] - 0.02, 0.120, TOP + 0.240), 2.0, OPEN_GAP * MM, rot=rot_l)
        yield from L.move(p + np.array([-0.02, 0.0, 0.110]), 1.5, OPEN_GAP * MM, rot=rot_l)
        yield from L.trim_to_measured(p + np.array([-0.02, 0.0, 0.110]), max_duration=0.6)
        yield from L.move(p, 2.0, OPEN_GAP * MM, rot=rot_l)
        yield from L.trim_to_measured(p, tol=0.002, max_duration=0.8)

    def leave_neck():
        yield from L.move(L.pos + np.array([-0.02, 0.0, 0.110]), 1.5, OPEN_GAP * MM, rot=rot_l)
        yield from L.move((L.pos[0], 0.150, TOP + 0.240), 1.5, OPEN_GAP * MM, rot=rot_l)

    # 1. the right arm picks the meter off its tray by the handle
    g = np.array(METER_START) + np.array([X_GRIP, 0.0, GRIP_UP])
    yield from R.move(g + np.array([0.0, 0.0, 0.120]), 2.0, 0.084 * MM, rot=rot_r)
    yield from R.trim_to_measured(g + np.array([0.0, 0.0, 0.120]), max_duration=0.6)
    yield from R.move(g, 1.8, 0.084 * MM, rot=rot_r)
    yield from R.trim_to_measured(g, tol=0.002, max_duration=0.8)
    yield from R.grip(0.050 * MM, 0.8)
    yield from R.hold(0.3)
    if not R.holding(0.060):  # pads did not stop on the 66 mm handle: open, go 3 mm lower, try once more
        yield from R.grip(0.084 * MM, 0.5)
        yield from R.move(g + np.array([0.0, 0.0, -0.003]), 0.5, 0.084 * MM, rot=rot_r)
        yield from R.grip(0.050 * MM, 0.8)
        yield from R.hold(0.3)
        if not R.holding(0.060):
            yield from R.home(3.0)
            return
    # 2. lift it level and bring the shut jaw in front of the red lead
    enter = np.array([ENTER_X, ENTER_D, TOP + WORK_Z])
    yield from carry((METER_START[0] - 0.040, METER_START[1], TOP + WORK_Z), 2.5)
    yield from carry(enter, 3.5)
    yield from carry(enter, 1.5)
    # 3. the left arm comes down over the neck and squeezes the trigger until the jaw is open
    yield from to_neck()
    gap = yield from squeeze()
    # 4. thread: feed the open jaw straight in until the red lead is in the window, then centre it
    yield from carry((LEAD_X, ENTER_D, TOP + WORK_Z), 4.0, gap)
    yield from carry((LEAD_X, 0.0, TOP + WORK_Z), 1.0, gap)
    # 5. shut: let the trigger go, take the left hand away, keep the meter still for the reading
    yield from let_go(gap)
    yield from leave_neck()
    yield from carry((LEAD_X, 0.0, TOP + WORK_Z), 3.6, fb=False)
    # 6. draw off: reopen the jaw, back out the way it came, let the jaw shut in the air
    yield from to_neck()
    gap = yield from squeeze()
    yield from carry((LEAD_X, ENTER_D, TOP + WORK_Z), 1.0, gap)
    yield from carry(enter, 4.0, gap)
    yield from let_go(gap)
    yield from leave_neck()

    # 7. the right arm lays the meter back on its tray, lets go and parks; the left arm parks
    def put_back():
        yield from carry((METER_START[0] - 0.040, METER_START[1], TOP + WORK_Z), 3.5)
        yield from carry((METER_START[0], METER_START[1], TOP + REST_Z + 0.030), 2.5)
        yield from carry((METER_START[0], METER_START[1], TOP + REST_Z + 0.003), 2.0)
        yield from R.hold(0.4)
        yield from R.grip(0.084 * MM, 0.8)
        yield from R.hold(0.3)
        yield from R.move(R.pos + np.array([0.0, 0.0, 0.120]), 1.5, 0.084 * MM, rot=R.rot.copy())
        yield from R.home(2.5)

    yield from ctx.both(L.home(3.0), put_back())
    yield from ctx.wait(1.5)
