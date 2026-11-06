"""On the station table: a 0 to 100 mm digital sliding caliper, clamped flat on a black holder block with its jaws
pointing at the robots and overhanging the block's front edge, and a short turned aluminium pin standing on end on the
bench to its left.

The caliper's steel beam runs along y. The fixed jaw is at its left end (+y); the sliding jaw rides on a slider whose
dark housing carries a live display (the jaw gap in mm, to 0.01 mm) and, at its right end, a black thumb rest. Pushing
the thumb rest to the left closes the jaws. They start 40 mm apart, and the display reads 40.00.

Measuring the pin means holding it upright between the jaws, its side against the fixed jaw, and closing the sliding
jaw until both jaws just touch it. The jaws are 5 cm above the bench with nothing under them, so a pin let go falls
through: one hand holds the pin while the other closes the slider. The display reads the slider, not the jaw tip:
pressing hard flexes the sliding jaw (0.1 mm per newton) and the display reads low, as on a real caliper.

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

STEEL = (0.74, 0.75, 0.77)  # hardened stainless
HOUSING_C = (0.20, 0.21, 0.23)
SCREEN = (0.55, 0.60, 0.52)  # grey-green LCD
DIGIT = (0.05, 0.05, 0.06)  # dark LCD segments
BLACK = (0.06, 0.06, 0.07)
HOLDER_C = (0.12, 0.12, 0.13)
BUTTON = (0.75, 0.15, 0.12)
ALU = (0.80, 0.82, 0.85)

# -- the caliper (fixed). Frame: origin on the table under the beam's front edge (the jaw side), level with the fixed
#    jaw's measuring face; x away from the robots, y to their left. -------------------------------------------------
CX, CY = 0.296, 0.085
HB = 0.050  # the beam's underside, above the table (the top of the holder block)
T = 0.0035  # beam and jaw thickness
BEAM_W = 0.016
BEAM_Y = (-0.190, 0.012)  # beam ends: the right end to the head behind the fixed jaw
JAW_L = 0.045  # the jaws reach 45 mm out from the beam's front edge, towards the robots
JAW_T = 0.008  # jaw plate thickness along y
GAP0 = 0.040  # jaw gap at the start
GAP_MAX = 0.099
MF0 = -GAP0  # the sliding jaw's measuring face at the start (local y)
HOUSING_X = (-0.002, 0.036)
HOUSING_H = 0.009
HOUSING_Y = (-0.079, -0.002)  # from the sliding jaw's face
REST_OFF, REST_T, REST_X, REST_H = 0.085, 0.006, (0.0, 0.020), 0.016  # thumb rest: its pushed face 85 mm behind the jaw
HOLDER_X, HOLDER_Y = (0.0, 0.030), (-0.185, 0.010)
SLIDE_DAMP, SLIDE_FRICTION = 3.0, 0.1  # N s/m; light friction: the slider stays where it is left
FLEX_K, FLEX_C, FLEX_MAX = 10000.0, 20.0, 0.0005  # the sliding jaw's flex: 0.1 mm per N, up to 0.5 mm (5 N)
Z_TOP = HB + T  # the beam's top face
HOUSING_TOP = Z_TOP + HOUSING_H

FLEX = Joint(  # the sliding jaw itself, sprung against the slider: pressed on the part, it gives way a little
    "jaw", kind="slide", axis="Y", anchor=(-JAW_L / 2, MF0 - JAW_T / 2, HB + T / 2), lower=-FLEX_MAX, upper=0.0,
    initial=0.0, stiffness=FLEX_K, damping=FLEX_C, target=0.0, mass=0.02, parent="slider",
    boxes=[((-JAW_L / 2, MF0 - JAW_T / 2, HB + T / 2), (JAW_L, JAW_T, T), STEEL)],
)

# Live display on the slider: four 7-segment digits 14 mm tall (their tops away from the robots, so they read upright
# from the robots' side), a decimal point after the second. Each segment is a box on its own sprung slide riding on the
# slider, driven up flush with the screen when lit and down into the housing when not.
DISP_X, DISP_Y = 0.017, MF0 - 0.042  # screen centre (local, slider at its start)
SCREEN_SIZE = (0.020, 0.050, 0.0005)
FACE_Z = HOUSING_TOP + SCREEN_SIZE[2]
DIG_H, DIG_W, SEG_T, DIG_PITCH = 0.014, 0.008, 0.0016, 0.0105
SEG_SUNK = -0.003
SEG_Z = FACE_Z + 0.0004 - 0.0005  # centre of a lit segment: its top 0.4 mm proud of the screen
SEGS = "abcdefg"
GLYPHS = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg", "6": "acdefg",
          "7": "abc", "8": "abcdefg", "9": "abcdfg", "-": "g", " ": ""}


def _digit_y(i: int) -> float:
    return DISP_Y + (1.5 - i) * DIG_PITCH  # digit 0 (tens) on the left


def _segment(i: int, s: str):
    """(centre xy, size xy) of segment s of digit i, in the caliper frame with the slider at its start."""
    xc, yc = DISP_X, _digit_y(i)
    hl, vl = DIG_W - SEG_T - 0.0006, DIG_H / 2 - SEG_T / 2 - 0.0006
    side = DIG_W / 2 - SEG_T / 2
    return {
        "a": ((xc + DIG_H / 2 - SEG_T / 2, yc), (SEG_T, hl)),
        "b": ((xc + DIG_H / 4, yc - side), (vl, SEG_T)),
        "c": ((xc - DIG_H / 4, yc - side), (vl, SEG_T)),
        "d": ((xc - DIG_H / 2 + SEG_T / 2, yc), (SEG_T, hl)),
        "e": ((xc - DIG_H / 4, yc + side), (vl, SEG_T)),
        "f": ((xc + DIG_H / 4, yc + side), (vl, SEG_T)),
        "g": ((xc, yc), (SEG_T, hl)),
    }[s]


def display_text(mm: float) -> str:
    """The four characters the display shows for a gap (mm): tens (blank under 10), units, tenths, hundredths."""
    v = min(max(round(float(mm), 2), 0.0), 99.99)
    s = f"{v:5.2f}"
    return s[0] + s[1] + s[3] + s[4]


def _lit(text: str) -> list[float]:
    return [0.0 if s in GLYPHS[ch] else SEG_SUNK for ch in text for s in SEGS]


# Each segment is built where it sits at the start, with the opening gap showing, so the very first rendered frames
# already read it (the renderer takes about a second to pick up joint positions set at reset). A segment joint's
# position is its offset from that start height.
_START = _lit(display_text(GAP0 * 1000))
SEGMENT_JOINTS = []
for _i in range(4):
    for _k, _s in enumerate(SEGS):
        (_cx, _cy), (_sx, _sy) = _segment(_i, _s)
        _o = _START[7 * _i + _k]
        SEGMENT_JOINTS.append(Joint(
            f"seg{_i}{_s}", kind="slide", axis="Z", anchor=(_cx, _cy, SEG_Z + _o), lower=SEG_SUNK - _o, upper=-_o,
            initial=0.0, target=0.0, stiffness=200.0, damping=2.0, mass=0.002, parent="slider",
            boxes=[((_cx, _cy, SEG_Z + _o), (_sx, _sy, 0.001), DIGIT)],
        ))


def _seg_targets(text: str) -> list[float]:
    return [q - o for q, o in zip(_lit(text), _START)]
_dot = (DISP_X - DIG_H / 2 + SEG_T / 2, (_digit_y(1) + _digit_y(2)) / 2)

SLIDER = Joint(  # the slider: housing, screen, buttons and thumb rest. Positive closes the jaws.
    "slider", kind="slide", axis="Y", anchor=(DISP_X, DISP_Y, Z_TOP), lower=GAP0 - GAP_MAX, upper=GAP0, initial=0.0,
    stiffness=0.0, damping=SLIDE_DAMP, friction=SLIDE_FRICTION, target=0.0, mass=0.05,
    boxes=[
        ((sum(HOUSING_X) / 2, MF0 + sum(HOUSING_Y) / 2, Z_TOP + HOUSING_H / 2),
         (HOUSING_X[1] - HOUSING_X[0], HOUSING_Y[1] - HOUSING_Y[0], HOUSING_H), HOUSING_C),
        ((-0.002, MF0 - 0.005, HB + (T + HOUSING_H) / 2), (0.004, 0.006, T + HOUSING_H), HOUSING_C),  # jaw root
        ((DISP_X, DISP_Y, HOUSING_TOP + SCREEN_SIZE[2] / 2), SCREEN_SIZE, SCREEN),
        ((_dot[0], _dot[1], SEG_Z), (SEG_T, SEG_T, 0.001), DIGIT),  # decimal point (always shown)
        *[((0.031, MF0 + dy, HOUSING_TOP + 0.0005), (0.006, 0.008, 0.001), BUTTON if k == 0 else BLACK)
          for k, dy in enumerate((-0.026, -0.058))],  # ZERO and ON/OFF buttons (not used)
        ((sum(REST_X) / 2, MF0 - REST_OFF + REST_T / 2, Z_TOP + REST_H / 2), (REST_X[1] - REST_X[0], REST_T, REST_H),
         BLACK),  # thumb rest
    ],
)

CALIPER = part(
    "caliper", pos=(CX, CY, TOP), fixed=True, friction=0.5,
    boxes=[
        ((sum(HOLDER_X) / 2, sum(HOLDER_Y) / 2, HB / 2), (HOLDER_X[1] - HOLDER_X[0], HOLDER_Y[1] - HOLDER_Y[0], HB),
         HOLDER_C),  # holder block
        ((BEAM_W / 2, sum(BEAM_Y) / 2, HB + T / 2), (BEAM_W, BEAM_Y[1] - BEAM_Y[0], T), STEEL),  # beam
        ((-JAW_L / 2, JAW_T / 2, HB + T / 2), (JAW_L, JAW_T, T), STEEL),  # fixed jaw
        ((BEAM_W / 2, BEAM_Y[1] - 0.006, Z_TOP + 0.0005), (BEAM_W - 0.004, 0.008, 0.001), BLACK),  # name plate
    ],
    joints=[SLIDER, FLEX, *SEGMENT_JOINTS],
)

# -- the pin (loose), standing on end on the bench -------------------------------------------------------------------
PIN_D = 0.02340  # the true diameter, 23.40 mm
PIN_R, PIN_H = PIN_D / 2, 0.040
PIN_XY = (0.19, 0.17)
PIN = part("pin", pos=(*PIN_XY, TOP), mass=0.046, friction=0.5,
           cylinders=[((0.0, 0.0, PIN_H / 2), PIN_R, PIN_H, ALU)])

OBJECTS = [CALIPER, PIN]


# -- the display: shows the slider's position as the jaw gap ---------------------------------------------------------
def reading_mm(j: dict) -> float:
    """What the display shows (mm, to 0.01): the gap the slider's position gives, as the caliper's encoder reads it.
    The jaw's flex is not seen, so a squeezed part reads low."""
    return round((GAP0 - j["slider"]) * 1000.0, 2)


def jaw_gap(j: dict) -> float:
    """The true gap between the measuring faces (m): the slider's gap plus the jaw's flex."""
    return GAP0 - j["slider"] - j["jaw"]


def jaw_force(j: dict) -> float:
    """N with which the sliding jaw presses on whatever is between the jaws, from its flex."""
    return FLEX_K * max(0.0, -j["jaw"])


_trial = {"flex": None, "text": display_text(GAP0 * 1000)}
_loose = {"dy": 0.0}
_ids: dict = {}


@on_reset
def _clear(env):
    _trial.update(flex=None, text=display_text(GAP0 * 1000))
    _loose["dy"] = 0.0
    _ids.pop("set", None)
    _push_reset()


@on_step
def _watch(env):
    import torch

    import labkit

    j = labkit._part_state(env, labkit._SPECS["caliper"]).joints
    _trial["flex"] = j["jaw"]
    f = jaw_force(j)
    if f > YIELD_F:  # the holding hand gives way to the sliding jaw's push, towards the fixed jaw
        dt = float(getattr(env.unwrapped, "step_dt", 0.05))
        _loose["dy"] = min(YIELD_MAX, _loose["dy"] + YIELD_GAIN * f * dt)
    text = display_text(reading_mm(j))
    asset = env.unwrapped.scene["caliper"]
    if "segments" not in _ids:
        names = list(asset.joint_names)
        _ids["segments"] = [names.index(s.joint_name) for s in SEGMENT_JOINTS]
    if text != _trial["text"] or "set" not in _ids:
        targets = torch.tensor([_seg_targets(text)], device=asset.device)
        asset.set_joint_position_target(targets, joint_ids=_ids["segments"])
        _trial["text"], _ids["set"] = text, True


# -- scoring ---------------------------------------------------------------------------------------------------------
JAW_MID = TOP + HB + T / 2  # the jaws' mid-plane (world z)
END_MARGIN = 0.002
UPRIGHT_MIN = math.cos(math.radians(30.0))
TIERS = ((0.05, 1.0, 0.3, 1.0), (0.20, 3.0, 2.0, 0.5))  # (mm off the true size, deg off square, N of jaw force, score)
NEAR_MM = 1.0


def _crossing(pin):
    """Where the pin's axis crosses the jaws' mid-plane, in the caliper frame, and how far along the axis from its
    bottom (m); None if the pin is not upright within 30 degrees."""
    axis = pin.rotation()[:, 2]
    if axis[2] < UPRIGHT_MIN:
        return None
    s = (JAW_MID - pin.pos[2]) / axis[2]
    p = pin.pos + s * axis
    return np.array([p[0] - CX, p[1] - CY, p[2] - TOP]), float(s)


def between_jaws(pin, j: dict) -> bool:
    """The pin is upright with its axis crossing the jaws' mid-plane between their two measuring faces, within the
    jaws' 45 mm length, and the jaws meet its side (the crossing at least 2 mm from either end of the pin)."""
    c = _crossing(pin)
    if c is None:
        return False
    p, s = c
    return (-JAW_L + END_MARGIN <= p[0] <= 0.0 and -jaw_gap(j) <= p[1] <= 0.0
            and END_MARGIN <= s <= PIN_H - END_MARGIN)


def off_square_deg(pin) -> float:
    """Angle (deg) between the pin's axis and the jaw faces: its lean towards or away from the fixed jaw."""
    return float(np.degrees(np.arcsin(min(1.0, abs(pin.rotation()[1, 2])))))


def score(objects):
    """0 unless the pin is between the jaws (see between_jaws). 0.1 if it is; 0.25 if the display also reads within
    1.00 mm of the pin's 23.40 mm; 0.5 within 0.20 mm with the pin square to the jaws within 3 degrees and the jaw
    pressing on it with at most 2 N; 1.0 within 0.05 mm, square within 1 degree, at most 0.3 N (just touching)."""
    c, pin = objects["caliper"], objects["pin"]
    j = c.joints
    if not between_jaws(pin, j):
        return 0.0
    err = abs(reading_mm(j) - PIN_D * 1000.0)
    sq, f = off_square_deg(pin), jaw_force(j)
    for mm, deg, newton, s in TIERS:
        if err <= mm + 1e-6 and sq <= deg and f <= newton:
            return s
    return 0.25 if err <= NEAR_MM + 1e-6 else 0.1


SUCCESS_HOLD_S = 6.0  # end a trial early once it has stayed measured for 6 s


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """Display error (m) once the pin is between the jaws; before that, the pin's distance from the measuring spot."""
    c, pin = objects["caliper"], objects["pin"]
    if between_jaws(pin, c.joints):
        return abs(reading_mm(c.joints) / 1000.0 - PIN_D)
    spot = np.array([CX - 0.025, CY - PIN_R, JAW_MID])
    return float(np.linalg.norm(pin.to_world((0.0, 0.0, 0.010)) - spot)) + GAP0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Left arm: picks the pin off the bench by its upper part (fingers closing along x, pad tips 14.5 mm above its bottom),
# lifts it over the jaws, lowers it between them 3 mm clear of the fixed jaw and holds it there loosely: it carries
# the pin's weight and keeps it square, but gives way sideways whenever the sliding jaw presses on the pin, moving its
# target towards the fixed jaw in proportion to the jaw's force (``_LooseHold``). The sliding jaw then seats the pin against the fixed jaw, as a person lets a part
# settle onto the fixed jaw while closing the caliper.
# Right arm: meanwhile comes down behind the thumb rest with closed fingers, leaning 30 degrees towards the fixed jaw
# (``Orient`` lets a Waypoint's ``yaw`` carry the full rotation); once the pin is held, it pushes the rest to
# the left at 3 mm/s, closed loop on the sliding jaw: the jaw meets the pin and carries it across to the fixed jaw with
# little force (it pauses whenever the jaw presses with 0.2 N before the pin is seated, letting the holding hand give
# way), and stops once the jaw flexes by 0.02 mm (0.2 N) with the pin seated on the fixed jaw. Then it backs
# off 5 mm, so the jaw rests on the pin without load, and parks. The push's waypoints read their position from the
# trial every control step (``_PushPos``).

_prev_action7 = _demo.ArmPlan.action7
if not getattr(_prev_action7, "_clock", False):
    def _action7(self, t, _prev=_prev_action7):
        _demo._plan_clock = float(t)  # the plan's time, for waypoints that move with the trial
        return _prev(self, t)

    _action7._clock = True
    _demo.ArmPlan.action7 = _action7

TIP_ABOVE_TCP = 0.013
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)
PIN_YAW = -math.pi / 2  # fingers close along x, on the pin's front and back
PIN_WIDE = -0.020  # pads 40 mm apart round the 23.4 mm pin
GRIP_UP = 0.0145  # pad tips 14.5 mm above the pin's bottom: 5 mm above the jaws when it is held
HOLD_BOTTOM = HB - 0.006  # the held pin's bottom, 6 mm below the jaws' underside
PICK_Z = TOP + GRIP_UP + TIP_ABOVE_TCP
PICK_HOVER = TOP + PIN_H + 0.05
CARRY_Z = TOP + Z_TOP + 0.010 + GRIP_UP + TIP_ABOVE_TCP  # the pin's bottom 10 mm over the jaws
HOLD_Z = TOP + HOLD_BOTTOM + GRIP_UP + TIP_ABOVE_TCP
HOLD_X = CX - 0.025  # the pin's axis 25 mm out from the beam, 20 mm in from the jaw tips
CLEAR = 0.003  # the pin is held 3 mm clear of the fixed jaw; the sliding jaw seats it
YIELD_F = 0.03  # N: the holding hand gives way while the sliding jaw presses on the pin with more than this,
YIELD_GAIN = 0.10  # m/s per N of jaw force (10 mm/s at 0.1 N: faster than the 3 mm/s push),
YIELD_MAX = CLEAR + 0.0015  # at most until it aims 1.5 mm into the fixed jaw: the pin then rests on it
HOLD = (HOLD_X, CY - PIN_R - CLEAR, HOLD_Z)
ENTRY = (HOLD_X, CY - PIN_R - 0.005, HOLD_Z)

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

# The pushing hand leans its closed fingers 30 degrees towards the fixed jaw (+y): its wrist and elbow then stay out to
# the right, clear of the left arm's elbow (pointing straight down, the elbows would need about 16 cm between the hands).
PUSH_TILT = math.radians(30.0)
PUSH_ROT = Orient(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), PUSH_TILT) @ _down_grip(0.0))
FINGER_FACE = 0.0092  # the leaning fingers meet the thumb rest's face with the TCP this far behind it (from the hulls)
FINGER_LOW = 0.0123  # and their lowest point is this far below the TCP
PUSH_X = CX + sum(REST_X) / 2
PUSH_Z = TOP + Z_TOP + 0.005 + FINGER_LOW  # fingers 5 mm above the beam
PUSH_Y0 = CY + MF0 - REST_OFF - FINGER_FACE - 0.008  # 8 mm short of the thumb rest
PUSH_NOMINAL = (GAP0 - PIN_D) + 0.008  # how far the finger goes until the jaw meets the pin (offline plan)
PUSH_EXTRA = 0.004  # in the simulator, at most 4 mm further if no touch is felt
PUSH_V = 0.003  # m/s
PUSH_WINDOW = 11.0  # s
FLEX_TOUCH = 0.00002  # 0.02 mm of jaw flex (0.2 N): the jaw has met the pin

PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    ("holder", (CX + sum(HOLDER_X) / 2, CY + sum(HOLDER_Y) / 2, TOP + HB / 2),
     (HOLDER_X[1] - HOLDER_X[0], HOLDER_Y[1] - HOLDER_Y[0], HB)),
    ("beam", (CX + BEAM_W / 2, CY + sum(BEAM_Y) / 2, TOP + HB + T / 2), (BEAM_W, BEAM_Y[1] - BEAM_Y[0], T)),
    ("fixed jaw", (CX - JAW_L / 2, CY + JAW_T / 2, TOP + HB + T / 2), (JAW_L, JAW_T, T)),
    ("sliding jaw (its travel)", (CX - JAW_L / 2, CY + (MF0 - JAW_T - PIN_D) / 2, TOP + HB + T / 2),
     (JAW_L, -PIN_D - MF0 + JAW_T, T)),
    ("housing (its travel)", (CX + sum(HOUSING_X) / 2, CY + MF0 + (HOUSING_Y[0] + HOUSING_Y[1] + GAP0 - PIN_D) / 2,
                              TOP + Z_TOP + HOUSING_H / 2),
     (HOUSING_X[1] - HOUSING_X[0], HOUSING_Y[1] - HOUSING_Y[0] + GAP0 - PIN_D, HOUSING_H)),
    ("thumb rest (at the start)", (CX + sum(REST_X) / 2, CY + MF0 - REST_OFF + REST_T / 2, TOP + Z_TOP + REST_H / 2),
     (REST_X[1] - REST_X[0], REST_T, REST_H)),
]

_push = {"t0": None, "tl": 0.0, "dy": 0.0, "mode": "advance"}


def _push_reset():
    _push.update(t0=None, tl=0.0, dy=0.0, mode="advance")


def _push_dy() -> float:
    """How far (m) the pushing finger has advanced from PUSH_Y0: at PUSH_V until the jaw is felt to touch the pin (its
    flex past FLEX_TOUCH), then held. Frozen after PUSH_WINDOW. Offline (no simulator) it advances to PUSH_NOMINAL."""
    t = float(getattr(_demo, "_plan_clock", 0.0))
    p = _push
    if p["t0"] is None or t < p["tl"] - 0.5:
        p.update(t0=t, tl=t, dy=0.0, mode="advance")
    t_eff = min(t, p["t0"] + PUSH_WINDOW)
    dt = max(0.0, t_eff - p["tl"])
    p["tl"] = max(p["tl"], t_eff)
    flex = _trial["flex"]
    if flex is None:
        cap = PUSH_NOMINAL
    else:
        cap = PUSH_NOMINAL + PUSH_EXTRA
        if flex < -FLEX_TOUCH:
            # The jaw is pressing. Seated (the holding hand can give way no further): stop. Otherwise wait for the
            # holding hand to give way before pressing on.
            if _loose["dy"] >= YIELD_MAX - 0.0003:
                p["mode"] = "hold"
            else:
                dt = 0.0
    if p["mode"] == "advance":
        p["dy"] = min(cap, p["dy"] + PUSH_V * dt)
    return p["dy"]


class _PushPos:
    """The pushing finger's TCP position, read every control step: the push, then ``dy``/``dz`` from where it ended."""

    def __init__(self, dy=0.0, dz=0.0):
        self.dy, self.dz = dy, dz

    def __array__(self, dtype=None, copy=None):
        return np.asarray((PUSH_X, PUSH_Y0 + _push_dy() + self.dy, PUSH_Z + self.dz), dtype=dtype or float)


class _LooseHold:
    """The holding hand's TCP target, read every control step: HOLD, moved towards the fixed jaw by however far the
    hand has given way to the sliding jaw's push so far (``_loose``)."""

    def __array__(self, dtype=None, copy=None):
        return np.asarray((HOLD[0], HOLD[1] + _loose["dy"], HOLD[2]), dtype=dtype or float)


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    _push_reset()
    _loose["dy"] = 0.0
    px, py = PIN_XY
    left = [
        Waypoint(2.5, (px, py, PICK_HOVER), FINGER_OPEN, "above the pin", PIN_YAW),
        Waypoint(1.2, (px, py, PICK_Z), PIN_WIDE, "fingers either side of the pin", PIN_YAW),
        Waypoint(0.8, (px, py, PICK_Z), FINGER_CLOSED, "grip", PIN_YAW),
        Waypoint(1.2, (px, py, CARRY_Z), FINGER_CLOSED, "lift", PIN_YAW),
        Waypoint(2.0, (ENTRY[0], ENTRY[1], CARRY_Z), FINGER_CLOSED, "over the open jaws", PIN_YAW),
        Waypoint(1.5, ENTRY, FINGER_CLOSED, "lower between the jaws", PIN_YAW),
        Waypoint(1.0, HOLD, FINGER_CLOSED, "near the fixed jaw, 3 mm clear", PIN_YAW),
        Waypoint(0.8, HOLD, FINGER_CLOSED, "steady", PIN_YAW),
        Waypoint(0.1, _LooseHold(), FINGER_CLOSED, "hold loosely: free to move across the jaws", PIN_YAW),
    ]
    t_held = _dur(left)
    right = [
        Waypoint(2.5, (PUSH_X, PUSH_Y0, TOP + 0.11), FINGER_CLOSED, "above, behind the thumb rest, leaning",
                 PUSH_ROT),
    ]
    right.append(Waypoint(max(0.3, t_held - _dur(right)), (PUSH_X, PUSH_Y0, TOP + 0.11), FINGER_CLOSED,
                          "wait for the pin to be held", PUSH_ROT))
    right += [
        Waypoint(1.2, (PUSH_X, PUSH_Y0, PUSH_Z), FINGER_CLOSED, "down behind the thumb rest", PUSH_ROT),
        Waypoint(0.1, _PushPos(), FINGER_CLOSED, "push the slider", PUSH_ROT),
        Waypoint(PUSH_WINDOW, _PushPos(), FINGER_CLOSED, "close the jaw until it touches the pin", PUSH_ROT),
        Waypoint(1.0, _PushPos(dy=-0.005), FINGER_CLOSED, "back off the thumb rest", PUSH_ROT),
        Waypoint(1.0, _PushPos(dy=-0.005, dz=0.04), FINGER_CLOSED, "up", PUSH_ROT),
        Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    t_end = _dur(right) + 9.0  # the pin held, measured, until the trial ends
    left.append(Waypoint(t_end - _dur(left), _LooseHold(), FINGER_CLOSED,
                         "hold the pin loosely while the sliding jaw seats it on the fixed jaw", PIN_YAW))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "rest"))
    return {"left": left, "right": right}
