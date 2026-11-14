"""An 8-inch adjustable wrench set on an M5 nylon 6/6 hex nut (8 mm across flats) tightened to its recommended 0.76 N*m,
on a stud of a steel equipment mounting plate. The wrench's jaw has been left 1 mm wider than the nut. The job is to
loosen the nut. Nylon nuts are used where a fastener must insulate or not corrode; their recommended torque is close
to their failure torque (0.93 N*m for M5), so a loose jaw bearing on the corners damages them at service load.

What a trained mechanic does first is roll the knurled worm (thumbwheel) in the wrench's head until the movable jaw
closes snug on the nut's flats: a jaw with play lets the wrench turn on the nut until it bears on the corners, which
rounds them off and the wrench slips. Then they pull so that the nut presses into the FIXED jaw: on this wrench, as it
lies on the nut, that is a counterclockwise pull (seen from above), which is also the way a right-hand nut loosens.
Pulling the other way loads the movable jaw's tip, which levers the jaw open in its slot so that the wrench springs
off the nut. Last, they take the wrench off the nut and lay it down.

The sim: the wrench is a chain of joints on the base plate (lift, turn about the nut's axis, two slides), so it can be
lifted off the nut, moved and laid flat again, but it can't tip or be flipped over. The movable jaw is driven by the
worm (2 mm of jaw per turn of the worm). The thread is a breakaway model: the nut turns with the wrench only past a
breakaway torque, and the wrench turns freely through the jaws' play first.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props and the moving parts; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Loosen the nut on the base plate with the adjustable wrench, then take the wrench off it and set it down."

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
NX, NY = 0.30, 0.0  # the stud's axis
PLATE_C, PLATE = (0.27, 0.04), (0.26, 0.30, 0.008)  # the steel base plate: center (x, y) and size
WASHER_T, WASHER_R = 0.001, 0.005  # an M5 flat washer (DIN 125), 10 mm across, 1 mm thick
AF = 0.008  # m: the nut's width across flats (an M5 nylon 6/6 hex nut, DIN 934 size)
NUT_H = 0.004  # m: the nut's thickness
NUT_R = AF / math.sqrt(3)  # its corner radius (9.24 mm across corners)
NZ = PLATE[2] + WASHER_T + NUT_H / 2  # the nut's mid-plane above the table
JAW_T = 0.010  # the wrench's head and jaws are 10 mm thick
WZ = PLATE[2] + WASHER_T + JAW_T / 2  # the wrench's mid-plane: its jaws rest on the washer, round the 4 mm nut
STUD_R, STUD_OUT = 0.0025, 0.004  # the M5 stainless stud's radius and how far it stands above the nut

# --- The wrench (8 inches, 205 mm). Its frame: origin on the nut's axis at the wrench's mid-plane; x along the jaw's
# travel (the fixed jaw on +x, the movable jaw and the handle toward -x), y toward the jaw opening (the throat on -y).
OPEN0 = AF + 0.001  # m: the jaw opening at the start, 1 mm wider than the nut (it turns 16.9 deg before bearing)
OPEN_MIN, OPEN_MAX = 0.0005, 0.027  # the jaw's travel (closed to 27 mm)
LEAD = 0.002  # m of jaw travel per turn of the worm
WHEEL_R, WHEEL_L = 0.008, 0.011  # the worm: 16 mm across, 11 mm long, its top 3 mm above the head's faces
WHEEL_AT = (-0.038, -0.004)  # its center in the wrench's frame (axis along x)
HANDLE_Y, HANDLE_W, HANDLE_T = -0.016, 0.018, 0.007  # the handle's centerline (y), width and thickness
HANDLE_X = (-0.180, -0.050)  # its extent along x
GRIP_X = -0.130  # a point on the handle's grip, 130 mm from the nut's axis (the tests hold the wrench here)
TURN0 = math.radians(-120)  # the wrench's start: its x axis 120 degrees clockwise of world x (handle toward 60 deg)
NUT0 = math.radians(30)  # the nut's start angle, flats square to the wrench's jaws
WRENCH_MASS = 0.25  # kg (a real 8-inch adjustable wrench is 0.2 to 0.3 kg)

# --- The thread and the jaws' play (see Model).
K_THREAD = 12.7  # N*m/rad: the wrench's spring against a nut that has not broken loose (the tool and nut flexing)
BREAK = 0.06  # rad: the flex at breakaway, so the nut breaks loose at K_THREAD * BREAK = 0.76 N*m, the recommended
#               tightening torque of an M5 nylon 6/6 nut (0.73-0.79 N*m; it fails at 0.93 N*m), see fidelity
D_THREAD = 0.25  # N*m*s/rad: the thread's drag once it turns
TIGHT_STOP = math.radians(10)  # the nut can be forced at most 10 degrees tighter than it starts
SEAT_TOL = 0.002  # m: the wrench is down on the nut (lift joint within 2 mm of its rest)
CENTER_TOL = 0.003  # m: and its jaws centered on the nut's axis within 3 mm
LIFT_FREE = 0.016  # m: lifted this far the head clears the stud's top and can be moved off the nut
LIFT_MAX = 0.06
CLEAR = 0.03  # m: the wrench is off the nut once its jaw center is 3 cm from the nut's axis (the head is 4.4 cm long)

# --- Scoring thresholds.
PLAY_MAX = 0.0004  # m: snug = jaw within 0.4 mm of the flats; 0.4 mm lets the nut turn 5.4 degrees in the jaws
#                    (see play_angle); a new 150 mm wrench's jaw clearance is at most 0.25 mm (ISO 6787)
ROUND_DEG = 2.0  # degrees: the nut turned this far with the jaw loose = corners rounded (it bears only on them)
SPIN_TURN_DEG = 30.0  # degrees: the wrench spun this far on a nut that turns freely in its jaws = corners rounded
CW_DEG = 2.0  # degrees: the nut forced this far clockwise (tighter) = the pull loaded the movable jaw
# PLAY_MAX with ROUND_DEG, and CW_DEG, are operational proxies: the sim's nut can't deform, so the scorer counts the
# pull that would round the corners or spring the jaw. A real trial reads the jaw's play and the nut's turn from
# overhead video in every frame, checks the nylon nut's corners in before and after photos (see the README).
BROKEN_DEG = 5.0  # degrees: the nut has broken loose (more than any flex or play: it really turned on its thread)
LOOSE_DEG = 60.0  # degrees: one flat of the hex, a full swing before the wrench is reset on the next flats
SPRUNG_GAP = 0.003  # m: how far a movable jaw loaded at its tip springs open in its slot
HANDS_OFF = 0.05  # m: a gripper holds the wrench with its tool point within 5 cm of the handle or head
HOLD_OPEN = 0.6  # and its fingers under 60 % open (an open hand resting near the wrench is not holding it)
GRIPPERS = ("left_gripper", "right_gripper")
KEY = "base"  # the mechanism's name

PAINT = kit.Look((0.20, 0.30, 0.42), 0.6, metallic=0.3)  # the machine base's blue-gray paint
BOLT = kit.Look((0.30, 0.30, 0.32), 0.4, metallic=0.7)
ZINC = kit.Look((0.78, 0.78, 0.74), 0.35, metallic=0.8)
NYLON = kit.Look((0.92, 0.90, 0.84), 0.55)  # natural (undyed) nylon 6/6, off-white
STEEL = kit.Look((0.55, 0.56, 0.58), 0.4, metallic=0.8)
CHROME = kit.Look((0.80, 0.81, 0.83), 0.2, metallic=0.95)
KNURL = kit.Look((0.40, 0.41, 0.43), 0.5, metallic=0.8)
DARK = kit.Look((0.08, 0.08, 0.09), 0.6)
GRIP = kit.Look((0.75, 0.10, 0.08), 0.6)  # a red dipped handle grip


def play_angle(play: float) -> float:
    """How far (rad) a hex nut 8 mm across flats can turn between parallel jaws `play` wider than it before both jaws
    bear (on its corners): the hex is 2 * NUT_R * cos(30 deg - phi) wide at a turn phi. inf once it spins freely
    (play over 1.24 mm, the across-corners width less the flats)."""
    if play <= 0:
        return 0.0
    c = (AF + play) / (2 * NUT_R)
    return math.inf if c >= 1.0 else math.radians(30.0) - math.acos(c)


# --- The model: one record per trial, stepped once per control step from the joints (by link name).
class Model:
    def __init__(self):
        self.episode = kit.episode()
        self.nut0 = None  # the nut's angle at the start (rad)
        self.nut = None  # the nut's angle now (ccw from above +; loosening is +)
        self.coupled = False  # the wrench is seated on the nut and they turn together (past breakaway)
        self.offset = 0.0  # wrench turn minus nut angle when they coupled
        self.turn_target = None  # where the thread spring pulls the wrench (rad)
        self.open = OPEN0  # the jaw opening (m)
        self.play = OPEN0 - AF
        self.over = True  # the wrench's jaws are seated on the nut (down and centered)
        self.engaged = True  # seated, with the jaw open at least as wide as the nut
        self.cw = 0.0  # rad: total clockwise (tightening) turn forced on the nut
        self.loose_turn = 0.0  # rad: nut turn made with jaw play over PLAY_MAX
        self.spin_turn = 0.0  # rad: wrench turn on a nut that spins freely in its jaws
        self.max_play_turning = 0.0  # m: the largest play the nut was turned with (for the report)
        self.rounded = False  # irreversible: the nut's corners were rounded (the wrench then slips on it)
        self.sprung = False  # irreversible: the movable jaw was loaded at its tip and sprang open
        self.wheel0 = None
        self.lock_turn = None  # the wrench lying down off the nut: held where it was set down
        self.lock_xy = None  # the wrench down (on the nut or the plate): its slides held
        self.last_turn = None
        self.steps = 0


_M = Model()


def model() -> Model:
    global _M
    if _M.episode != kit.episode():  # the simulator reset the scene: a new trial
        _M = Model()
    return _M


def model_step(J: dict) -> Model:
    """Advance the model by one control step. J: joint positions by link name ("nut", "lift", "wr_turn", "wr_x",
    "wrench", "jaw", "wheel"). Called by the nut's law in the sim (the first link), by the tests offline."""
    m = model()
    m.steps += 1
    if m.nut0 is None:
        m.nut0 = m.nut = J["nut"]
        m.wheel0 = J["wheel"]
        m.last_turn = J["wr_turn"]
    lift, turn, x, y = J["lift"], J["wr_turn"], J["wr_x"], J["wrench"]
    off = math.hypot(x, y)
    m.over = lift <= SEAT_TOL and off <= CENTER_TOL
    gap = SPRUNG_GAP if m.sprung else 0.0
    lo = AF if m.over else OPEN_MIN
    want = OPEN0 - (J["wheel"] - m.wheel0) * LEAD / (2 * math.pi) + gap
    m.open = min(max(want, lo), OPEN_MAX)
    m.play = m.open - AF
    m.engaged = m.over and m.play >= -0.0002
    phi = play_angle(m.play)
    dturn = turn - m.last_turn
    m.last_turn = turn
    if m.engaged and not m.rounded and phi != math.inf:
        if not m.coupled:
            m.coupled, m.offset = True, turn - m.nut
        band = phi + BREAK
        # The jaw closing narrows the band: the wrench was already lying wherever it sat in the old play, so the jaw
        # squares the wrench onto the flats (the offset moves) instead of turning the nut. Only the wrench's own turn
        # this step, beyond the band, turns the nut.
        rel_before = (turn - dturn) - (m.nut + m.offset)
        if rel_before > band:
            m.offset += rel_before - band
        elif rel_before < -band:
            m.offset += rel_before + band
        rel = turn - (m.nut + m.offset)
        new = m.nut
        if rel > band:
            new = m.nut + rel - band
        elif rel < -band:
            new = m.nut + rel + band
        new = min(max(new, m.nut0 - TIGHT_STOP), m.nut0 + 4 * math.pi)
        d = new - m.nut
        if d < 0:
            m.cw += -d
        if abs(d) > 0:
            m.max_play_turning = max(m.max_play_turning, m.play)
        if m.play > PLAY_MAX:
            m.loose_turn += abs(d)
        m.nut = new
        m.turn_target = m.nut + m.offset + min(max(turn - (m.nut + m.offset), -phi), phi)
        if math.degrees(m.cw) >= CW_DEG:
            m.sprung = True  # the pull loaded the movable jaw's tip: it springs open and the wrench slips off the flats
        if math.degrees(m.loose_turn) >= ROUND_DEG:
            m.rounded = True
    else:
        m.coupled = False
        m.turn_target = None
        if m.engaged and phi == math.inf:  # the nut spins inside jaws wider than its corners: the wrench skids on it
            m.spin_turn += abs(dturn)
            if math.degrees(m.spin_turn) >= SPIN_TURN_DEG:
                m.rounded = True
    down = lift < LIFT_FREE
    if down:
        if m.lock_xy is None:
            m.lock_xy = (x, y)
    else:
        m.lock_xy = None
    if lift <= SEAT_TOL and not m.over:  # lying on the plate off the nut: it stays where it was set down
        if m.lock_turn is None:
            m.lock_turn = turn
    else:
        m.lock_turn = None
    return m


def wheel_limits(m: Model) -> tuple:
    """The worm's travel: it stops where the jaw closes on the nut's flats (or fully, off the nut) and fully open."""
    gap = SPRUNG_GAP if m.sprung else 0.0
    lo_open = AF if m.over else OPEN_MIN
    per = 2 * math.pi / LEAD
    w0 = m.wheel0 if m.wheel0 is not None else 0.0
    return (w0 + (OPEN0 + gap - OPEN_MAX) * per, w0 + (OPEN0 + gap - lo_open) * per)


class NutLaw:
    """The nut, turned by the model (kinematically: it follows the thread's position)."""

    kinematic, rate, hold, sets_limits = True, 0.5, 0.01, False

    def drive(self, key, q, qd, J, mem):
        m = model_step(J)
        mem["target"] = m.nut
        return m.nut, self.hold, 0.0, 0.0


class LiftLaw:
    """The wrench's height: its weight holds it down; off-center near the nut it rests on the nut and stud."""

    sets_limits = True

    def drive(self, key, q, qd, J, mem):
        off = math.hypot(J["wr_x"], J["wrench"])
        lo = 0.0 if (off <= CENTER_TOL or off >= CLEAR) else LIFT_FREE
        mem["limits"] = (lo, LIFT_MAX)
        return q, 0.0, 0.0, 0.5


class TurnLaw:
    """The wrench's turn about the nut's axis: on the nut, the thread spring (breakaway); lying off the nut, held;
    lifted (or skidding on a rounded nut), free."""

    sets_limits = True

    def drive(self, key, q, qd, J, mem):
        m = model()
        mem["limits"] = None
        if m.coupled and m.turn_target is not None:
            return m.turn_target, K_THREAD, 0.0, D_THREAD
        if m.lock_turn is not None:
            mem["limits"] = (m.lock_turn - 0.004, m.lock_turn + 0.004)
            return q, 0.0, 0.0, 0.05
        return q, 0.0, 0.0, 0.01


class SlideLaw:
    """The wrench's slides (in its turned frame): held while it is down (around the nut, or lying on the plate)."""

    sets_limits = True

    def __init__(self, k, lower, upper):
        self.k, self.lower, self.upper = k, lower, upper

    def drive(self, key, q, qd, J, mem):
        m = model()
        mem["limits"] = None
        if m.lock_xy is not None:
            at = m.lock_xy[self.k]
            mem["limits"] = (max(self.lower, at - 0.0005), min(self.upper, at + 0.0005))
        return q, 0.0, 0.0, 2.0


class JawLaw:
    """The movable jaw, set by the worm (kinematic): its face at x = AF/2 - opening in the wrench's frame."""

    kinematic, rate, hold, sets_limits = True, 0.003, 0.01, False

    def drive(self, key, q, qd, J, mem):
        t = AF / 2 - model().open
        mem["target"] = t
        return t, self.hold, 0.0, 0.0


class WheelLaw:
    """The worm: free to roll (a little drag), stopped where the jaw closes on the nut."""

    sets_limits = True

    def drive(self, key, q, qd, J, mem):
        mem["limits"] = wheel_limits(model())
        return q, 0.0, 0.0, 0.0004


def _w(c):
    """A point in world xy (and height above the table), in the base's frame (origin on the stud's axis)."""
    return (c[0] - NX, c[1] - NY, c[2])


def _wrench_parts():
    """The wrench body's meshes and colliders, in its frame (the movable jaw and the worm are links of their own)."""
    z = 0.0
    fixed = ((AF / 2 + 0.006, 0.0005, z), (0.012, 0.027, JAW_T))  # the fixed jaw: face on the nut's flat at x = AF/2
    spine = ((-0.0155, -0.0195, z), (0.074, 0.013, JAW_T))  # the head's back, behind the throat (y -0.026 to -0.013)
    cheek = ((-0.041, 0.0085, z), (0.022, 0.007, JAW_T - 0.002))  # the outer cheek beside the worm window
    end_a = ((-0.0485, -0.004, z), (0.007, 0.018, JAW_T - 0.002))  # the worm window's end walls
    end_b = ((-0.0300, -0.004, z), (0.003, 0.018, JAW_T - 0.002))
    neck = ((-0.056, (HANDLE_Y - 0.004) / 2 - 0.004, z), (0.014, 0.030, JAW_T - 0.002))
    hl = HANDLE_X[1] - HANDLE_X[0]
    handle = (((HANDLE_X[0] + HANDLE_X[1]) / 2, HANDLE_Y, z), (hl, HANDLE_W, HANDLE_T))
    meshes = [kit.box(*fixed, CHROME), kit.box(*spine, CHROME), kit.box(*cheek, CHROME), kit.box(*end_a, CHROME),
              kit.box(*end_b, CHROME), kit.box(*neck, CHROME), kit.box(*handle, CHROME)]
    meshes.append(kit.box((HANDLE_X[0] + 0.045, HANDLE_Y, z), (0.09, HANDLE_W + 0.002, HANDLE_T + 0.002), GRIP))
    meshes.append(kit.cylinder((HANDLE_X[0] + 0.009, HANDLE_Y, z), 0.0045, HANDLE_T + 0.004, DARK, n=20))  # hang hole
    meshes.append(kit.box((0.0, -0.022, JAW_T / 2 + 0.0002), (0.030, 0.004, 0.0004), DARK))  # the size scale's mark
    colliders = [kit.Collider(c, size=s) for c, s in (fixed, spine, cheek, end_a, end_b, neck, handle)]
    return meshes, colliders


def _base():
    meshes, colliders = [], []
    plate_c = (PLATE_C[0], PLATE_C[1], PLATE[2] / 2)
    meshes.append(kit.box(_w(plate_c), PLATE, PAINT))
    colliders.append(kit.Collider(_w(plate_c), size=PLATE))
    for dx in (-1, 1):  # the plate's four corner bolts
        for dy in (-1, 1):
            c = (PLATE_C[0] + dx * (PLATE[0] / 2 - 0.015), PLATE_C[1] + dy * (PLATE[1] / 2 - 0.015), PLATE[2] + 0.003)
            meshes.append(kit.cylinder(_w(c), 0.0085, 0.006, BOLT, n=6))
    meshes.append(kit.cylinder((0, 0, PLATE[2] + WASHER_T / 2), WASHER_R, WASHER_T, ZINC, n=40))
    stud_top = PLATE[2] + WASHER_T + NUT_H + STUD_OUT
    meshes.append(kit.cylinder((0, 0, (PLATE[2] + stud_top) / 2), STUD_R, stud_top - PLATE[2], STEEL, n=24))
    for k in range(3):  # thread crests on the stud's end
        meshes.append(kit.cylinder((0, 0, stud_top - 0.0008 - k * 0.00175), STUD_R + 0.0003, 0.0005, BOLT, n=24))

    # --- the nut (link frame: its center on the axis), turned only by the model
    nut = kit.Link("nut", "revolute", "Z", (0.0, 0.0, NZ), lower=NUT0 - TIGHT_STOP - 0.01, upper=NUT0 + 4 * math.pi,
                   start=NUT0, mass=0.02, damping=0.0, floating=True, law=NutLaw(),
                   meshes=(kit.cylinder((0, 0, 0), NUT_R - 0.0002, NUT_H - 0.0006, NYLON, n=6),
                           kit.cylinder((0, 0, 0), NUT_R * 0.93, NUT_H, NYLON, n=24),  # the chamfered faces
                           kit.box((0.0034, 0, NUT_H / 2 + 0.0002), (0.0016, 0.0006, 0.0004), DARK)))  # the paint line

    # --- the wrench: lift (Z) -> turn about the nut's axis -> slide x -> slide y (the wrench body) in its own frame
    pin = (0.0, 0.0, WZ)
    reach = 0.10
    wm, wc = _wrench_parts()
    lift = kit.Link("lift", "prismatic", "Z", pin, lower=0.0, upper=LIFT_MAX, start=0.0,
                    mass=0.005, damping=0.0, law=LiftLaw())
    turn = kit.Link("wr_turn", "revolute", "Z", pin, lower=-2 * math.pi, upper=2 * math.pi, start=TURN0, mass=0.005,
                    damping=0.0, parent="lift", law=TurnLaw())
    sx = kit.Link("wr_x", "prismatic", "X", pin, lower=-reach, upper=reach, start=0.0, mass=0.005, damping=0.0,
                  parent="wr_turn", law=SlideLaw(0, -reach, reach))
    body = kit.Link("wrench", "prismatic", "Y", pin, lower=-reach, upper=reach, start=0.0, mass=WRENCH_MASS,
                    meshes=tuple(wm), colliders=tuple(wc), damping=0.0, parent="wr_x", law=SlideLaw(1, -reach, reach))
    # the movable jaw: its face at x = q in the wrench's frame (q = AF/2 - opening)
    jaw_m = [kit.box((-0.0055, -0.001, 0), (0.011, 0.024, JAW_T), CHROME),
             kit.box((-0.020, -0.004, -0.0025), (0.020, 0.010, 0.004), STEEL)]  # its rack, under the worm
    jaw = kit.Link("jaw", "prismatic", "X", pin, lower=AF / 2 - OPEN_MAX - 0.0005, upper=AF / 2 - OPEN_MIN + 0.0005,
                   start=AF / 2 - OPEN0, mass=0.03, meshes=tuple(jaw_m),
                   colliders=(kit.Collider((-0.0055, -0.001, 0), size=(0.011, 0.024, JAW_T)),), damping=0.0,
                   floating=True, parent="wrench", law=JawLaw())
    # the worm: knurled, its axis along the wrench's x
    wa = (WHEEL_AT[0], WHEEL_AT[1], WZ)
    wheel_m = [kit.cylinder((0, 0, 0), WHEEL_R, WHEEL_L, KNURL, axis="x", n=28)]
    for k in range(5):
        wheel_m.append(kit.cylinder((-WHEEL_L / 2 + 0.0015 + k * 0.002, 0, 0), WHEEL_R + 0.0003, 0.0006, DARK,
                                    axis="x", n=28))
    wheel_m.append(kit.box((0, 0, WHEEL_R + 0.0002), (WHEEL_L + 0.001, 0.0016, 0.0006), DARK))  # a stripe that shows it roll
    wheel = kit.Link("wheel", "revolute", "X", wa, lower=-60.0, upper=60.0, start=0.0, mass=0.02,
                     meshes=tuple(wheel_m),
                     colliders=(kit.Collider((0, 0, 0), radius=WHEEL_R, height=WHEEL_L, axis="X"),),
                     damping=0.0, floating=True, parent="wrench", law=WheelLaw())

    return kit.Mechanism(KEY, pos=(NX, NY, TOP), size=(PLATE[0] + 0.06, PLATE[1] + 0.06, 0.04),
                         meshes=tuple(meshes), colliders=tuple(colliders),
                         links=(nut, lift, turn, sx, body, jaw, wheel))


OBJECTS = [_base()]


# --- Geometry helpers (world frame, from the joints).
def _joints(objects) -> dict:
    return {n: kit.joint(objects, f"{KEY}/{n}").pos for n in ("nut", "lift", "wr_turn", "wr_x", "wrench", "jaw",
                                                               "wheel")}


def wrench_point(J, local):
    """A point given in the wrench's frame (x, y), in world (x, y), from the joints."""
    t = J["wr_turn"]
    c, s = math.cos(t), math.sin(t)
    lx, ly = local[0] + J["wr_x"], local[1] + J["wrench"]
    return np.array([NX + c * lx - s * ly, NY + s * lx + c * ly])


def _held(objects, J) -> bool:
    """Any gripper with its fingers under 60 % open and its tool point within 5 cm of the wrench (its handle's
    centerline, or its head)."""
    a, b = wrench_point(J, (HANDLE_X[0], HANDLE_Y)), wrench_point(J, (0.0, 0.0))
    z = TOP + WZ + J["lift"]
    for g in GRIPPERS:
        if g not in objects or kit.opening(objects, g) >= HOLD_OPEN:
            continue
        p = np.asarray(objects[g].pos, float)
        u = np.clip(np.dot(p[:2] - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
        d = math.hypot(np.linalg.norm(p[:2] - (a + u * (b - a))), p[2] - z)
        if d < HANDS_OFF:
            return True
    return False


def set_down(J) -> bool:
    """The wrench is off the nut (its jaws 3 cm or more from the nut's axis) and lying on the plate."""
    return math.hypot(J["wr_x"], J["wrench"]) >= CLEAR and J["lift"] <= SEAT_TOL


def loosened_deg(m: Model) -> float:
    return 0.0 if m.nut0 is None else math.degrees(m.nut - m.nut0)


def _job(objects):
    m = model()
    if m.nut0 is None:
        return 0.0
    if m.rounded or m.sprung:
        return 0.0  # irreversible: the nut was turned with a loose jaw, or the pull loaded the movable jaw
    lo = loosened_deg(m)
    if lo < BROKEN_DEG:
        return 0.0  # the nut never broke loose
    J = _joints(objects)
    if _held(objects, J):
        return 0.0  # nothing counts while the wrench is still in hand
    if lo < LOOSE_DEG:
        return 0.25  # broken loose with a snug jaw and the right pull, but less than one flat
    if not set_down(J):
        return 0.5  # loosened a full flat, but the wrench left on the nut (or not laid down)
    return 1.0


def success(objects):
    """The nut loosened at least 60 degrees with a snug jaw and the pull on the fixed jaw, the wrench off the nut and
    laid down with nobody holding it."""
    return _job(objects) == 1.0


def score(objects):
    """Doing nothing scores 0. 0 if the nut was ever turned 2 degrees or more with the jaw over 0.4 mm wider than its
    flats (the proxy for rounding its corners; irreversible), or forced 2 degrees or more clockwise (the proxy for
    loading the movable jaw, which springs open; irreversible), or if it never turned 5 degrees looser, or if a
    gripper still holds the wrench at the end (fingers under 60 % open, tool point within 5 cm of it). Otherwise 0.25
    if the nut turned 5 to 60 degrees looser; 0.5 if 60 degrees or more but the wrench is not off the nut and laid
    down; 1.0 with it laid down off the nut (its jaws 3 cm or more from the nut's axis).
    Possible scores: 0, 0.25, 0.5, 1."""
    return _job(objects)


# --- The scripted demo.
PULL_X = -0.150  # where the demo grips the handle to pull it: 150 mm from the nut's axis, near the grip's end
GRIP_Z = TOP + WZ - 0.002 + PAD_TIP_PAST_TCP  # TCP height with the pad tips 2 mm below the handle's mid-plane
FIXED_PAD = 0.0445  # m: the YAM's fingers close on the pad that stays put, its face this far from the tool point
GRIP_CLEAR = 0.017  # m: the hand comes down with that pad this far clear of the handle; the other finger closes it
STROKE = math.radians(30)  # each pull swings the handle this far past where the hand took hold of it
LEAD = math.radians(4)  # pulled this much further: the arm's servos give about 3 to 4 degrees under 0.76 N*m
SETTLE = math.radians(2) + BREAK  # the wrench ends this far short of the pull: the arm's give, then the thread's flex
N_PULLS = 3
ROLL_Z = TOP + WZ + WHEEL_R - 0.0025 + PAD_TIP_PAST_TCP  # pad tips 2.5 mm below the worm's top


def _at(turn, local, dx=0.0, dy=0.0):
    J = {"wr_turn": turn, "wr_x": 0.0, "wrench": 0.0}
    p = wrench_point(J, local)
    return float(p[0] + dx), float(p[1] + dy)


def _grip_xy(turn):
    """The tool point for a grip on the handle at PULL_X, the wrench turned `turn`, the hand's yaw = turn. The pad that
    stays put is on the wrench's -y side (the side the handle moves toward in a loosening pull), GRIP_CLEAR from it."""
    shift = FIXED_PAD - HANDLE_W / 2 - GRIP_CLEAR  # handle centerline from the tool point, toward that pad
    return _at(turn, (PULL_X, HANDLE_Y + shift))


def demo():
    """Left arm (the right arm stays home): roll the worm with the closed fingertips, six strokes across its top
    toward the throat side (each closes the jaw about 0.8 mm; the worm stops once the jaw meets the flats). Then three
    pulls, as a person ratchets a wrench: grip the handle across its width near the end, pull it about 30 degrees
    counterclockwise (seen from above) round the nut's axis, let go, take a new hold where the handle now lies. Last,
    grip it, lift the wrench 3.5 cm off the nut, carry it 6 cm forward, lay it down and let go. Home."""
    from yam_arena.demo import Waypoint

    # the worm: strokes along the wrench's -y (top of the worm moving toward the throat)
    sweep = (math.cos(TURN0 - math.pi / 2), math.sin(TURN0 - math.pi / 2))  # the wrench's -y in world
    wx, wy = _at(TURN0, WHEEL_AT)
    yaw_r = TURN0 + math.pi / 2  # the closed fingers' long side along the stroke
    a = (wx - 0.022 * sweep[0], wy - 0.022 * sweep[1])
    b = (wx + 0.020 * sweep[0], wy + 0.020 * sweep[1])
    strokes = [Waypoint(2.0, (a[0], a[1], ROLL_Z + 0.04), FINGER_CLOSED, "over the worm", yaw_r)]
    for k in range(6):
        strokes += [Waypoint(0.8, (a[0], a[1], ROLL_Z), FINGER_CLOSED, f"stroke {k + 1}: down", yaw_r),
                    Waypoint(1.4, (b[0], b[1], ROLL_Z), FINGER_CLOSED, f"stroke {k + 1}: roll the worm", yaw_r),
                    Waypoint(0.6, (b[0], b[1], ROLL_Z + 0.02), FINGER_CLOSED, f"stroke {k + 1}: up", yaw_r),
                    Waypoint(0.8, (a[0], a[1], ROLL_Z + 0.02), FINGER_CLOSED, f"stroke {k + 1}: back", yaw_r)]
    strokes.append(Waypoint(0.8, (a[0], a[1], ROLL_Z + 0.05), FINGER_CLOSED, "up off the worm", yaw_r))

    def at(turn, dur, finger, label, dz=0.0):
        return Waypoint(dur, (*_grip_xy(turn), GRIP_Z + dz), finger, label, turn)

    left = list(strokes)
    turn = TURN0  # where the wrench lies
    for k in range(1, N_PULLS + 1):
        pull = turn + STROKE + LEAD
        left += [at(turn, 2.0 if k == 1 else 1.2, FINGER_OPEN, f"pull {k}: over the handle", 0.05),
                 at(turn, 0.9, FINGER_OPEN, f"pull {k}: round the handle"),
                 at(turn, 0.9, FINGER_CLOSED, f"pull {k}: grip it")]
        n = 4
        for i in range(1, n + 1):
            left.append(at(turn + (pull - turn) * i / n, 0.6, FINGER_CLOSED, f"pull {k}: pull round ({i}/{n})"))
        left += [at(pull, 0.3, FINGER_CLOSED, f"pull {k}: hold"),
                 at(pull, 0.6, FINGER_OPEN, f"pull {k}: let go"),
                 at(pull, 0.6, FINGER_OPEN, f"pull {k}: up", 0.05)]
        turn = pull - SETTLE
    gx, gy = _grip_xy(turn)
    left += [at(turn, 1.2, FINGER_OPEN, "over the handle", 0.05),
             at(turn, 0.9, FINGER_OPEN, "round the handle"),
             at(turn, 0.8, FINGER_CLOSED, "grip it"),
             at(turn, 1.2, FINGER_CLOSED, "lift the wrench off the nut", 0.035),
             Waypoint(1.5, (gx + 0.06, gy, GRIP_Z + 0.035), FINGER_CLOSED, "carry it clear", turn),
             Waypoint(1.0, (gx + 0.06, gy, GRIP_Z + 0.001), FINGER_CLOSED, "lay it down", turn),
             Waypoint(0.7, (gx + 0.06, gy, GRIP_Z + 0.001), FINGER_OPEN, "let go", turn),
             Waypoint(1.0, (gx + 0.06, gy, GRIP_Z + 0.08), FINGER_OPEN, "up", turn),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": [kit.Home(2.0, label="stay home")]}
