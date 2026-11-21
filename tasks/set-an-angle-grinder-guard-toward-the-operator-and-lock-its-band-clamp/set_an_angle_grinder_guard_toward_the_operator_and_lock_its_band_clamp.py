"""On the station table: a cordless 115 mm angle grinder (bare tool, battery out) lying upside down on its gear head and
motor housing, spindle up, a smooth nonabrasive stand-in for a Type 27 grinding wheel on the spindle. Its tool-free
wheel guard (zinc steel, a 190 degree skirt round the wheel's edge) is turned so its closed side faces the front right
and its open side faces back along the motor housing, toward the hands of whoever picks the grinder up to grind. The
guard's cam lever is shut: the lever lies along the guard's band with a red thumb tab beside the wheel. Flipping the
lever open (35 degrees, over a sprung detent) frees the band so the guard turns about the spindle; flipping it shut
locks the band again.

Requires -E object_jitter=0 (the grinder's 3 cm and 15 degree limits are measured from its start pose).

Run it:  inspect-robots run --embodiment yam_arena -E task=set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py \\
           -E object_jitter=0 --instruction "A welder grinds weld seams flush with this angle grinder next. Get it
           ready for them, then let go of it." --scorer task_score ...
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mechkit  # noqa: E402
from belimits import tilt_deg  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, on_reset, on_step, rod, tube  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0

# Colours (sRGB).
ALU = (0.36, 0.37, 0.39)       # gear head, painted cast aluminium
BLACK = (0.07, 0.07, 0.08)     # neck, rear cap, side handle
TEAL = (0.05, 0.52, 0.55)      # motor housing
ZINC = (0.80, 0.81, 0.83)      # guard, bright zinc-plated steel
WHEEL = (0.47, 0.30, 0.18)     # nonabrasive stand-in wheel, brown PETG (fabricate.py)
STEEL = (0.62, 0.63, 0.65)     # wheel lock nut and inner flange
ORANGE = (1.00, 0.45, 0.02)    # tape stripe on the guard's closed-side centre
RED = (0.85, 0.08, 0.06)       # cam lever thumb tab

# Where the grinder lies: Mech frame origin on the spindle axis at table level, motor housing along +y.
SPINDLE_XY = (0.27, -0.09)
GRINDER_POS = (SPINDLE_XY[0], SPINDLE_XY[1], TOP + 0.0005)

# The tool body (Mech frame, metres). Gear head 62 x 72 x 58 mm; 41 mm guard neck; 58 mm motor housing.
HEAD_H = 0.058
BODY = [
    box((0.0, 0.0, HEAD_H / 2), (0.062, 0.072, HEAD_H), ALU),                               # gear head (back face down)
    cyl((0.0, 0.0, 0.065), 0.0205, 0.014, BLACK),                                          # guard neck (41 mm boss)
    cyl((0.0, 0.12, 0.029), 0.029, 0.17, TEAL, axis="y", sides=16),                       # motor housing / grip
    cyl((0.0, 0.22, 0.029), 0.027, 0.03, BLACK, axis="y", sides=16),                      # rear cap, battery mount
    rod((0.031, 0.0, 0.030), (0.141, 0.0, 0.030), 0.0115, BLACK),                          # side handle, 23 mm x 110 mm
    cyl((0.0, 0.0, 0.07475), 0.0175, 0.0045, STEEL),                                       # inner flange
    tube((0.0, 0.0, 0.080), 0.0111, 0.0575, 0.006, WHEEL),                                 # 115 x 6 x 22.23 stand-in
    cyl((0.0, 0.0, 0.087), 0.019, 0.008, STEEL),                                           # wheel lock nut
]
WHEEL_R, WHEEL_TOP = 0.0575, 0.083

# The guard (Makita 123144-0 pattern): a band round the 41 mm neck, a 190 degree plate and skirt. At joint 0 (the start)
# its closed-side centre points at -45 degrees in the Mech frame; q, its turn from the motor axis (+y, 90 degrees),
# is then -135 degrees. The collar's stop pin limits the turn to q -180 .. +48 degrees.
GUARD_START = -0.75 * math.pi           # q at joint 0 (rad)
C0 = math.radians(-45.0)                # closed-side centre at joint 0, Mech frame
HALF_SPAN = math.radians(95.0)          # 190 degree skirt
A0, A1 = C0 - HALF_SPAN, C0 + HALF_SPAN
SKIRT_RIN, SKIRT_ROUT, SKIRT_Z0, SKIRT_H = 0.062, 0.0645, 0.0745, 0.025
SKIRT_TOP = SKIRT_Z0 + SKIRT_H          # 0.0995
SKIRT_RMID = (SKIRT_RIN + SKIRT_ROUT) / 2
BAND_Z = 0.065


def _rz(a: float) -> tuple[float, float, float, float]:
    """(w, x, y, z) quaternion of a turn by a about z."""
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def _ghost(shape):
    import dataclasses

    return dataclasses.replace(shape, collide=False)


# The cam lever (Makita XAG03 manual, page 9, figures 015084 and 015085: "For tool with clamp lever type wheel guard").
# It pivots on the band's ear at r 32 mm, 140 degrees clockwise of the closed-side centre (on the open side), about an
# axis parallel to the spindle. Shut, it lies almost along the band (75 degrees clockwise of straight out); it swings
# 35 degrees anticlockwise, away from the band, to open, as the manual's figures show. The real arm is curved; here
# it is a straight 58 mm arm with a red printed thumb cap (fabricate.py lever_cap.stl, 10 x 14 x 40 mm) on its end that
# rises 17.5 mm above the wheel beside its edge. A detent spring
# makes the over-centre action: it rests shut (0) or open (LEVER_OPEN) and flips at the midpoint.
LEVER_BETA = math.radians(-140.0)
LAM_RADIAL = C0 + LEVER_BETA                     # the pivot's direction from the spindle, Mech frame (-185 degrees)
SHUT_ALPHA = math.radians(75.0)                  # shut: the lever lies 75 degrees clockwise of straight out
LAM0 = LAM_RADIAL - SHUT_ALPHA                   # lever direction when shut, Mech frame
PIVOT_R = 0.032
LEVER_LEN = 0.058
TAB_D, TAB_ALONG, TAB_W = 0.052, 0.010, 0.014    # tab centre from the pivot, its length along the lever, its width
TAB_Z0, TAB_TOP = 0.0605, 0.1005                 # Mech frame heights
LEVER_TRAVEL_DEG = 35.0
LEVER_OPEN = math.radians(LEVER_TRAVEL_DEG)      # 0.611 rad, anticlockwise (positive) from shut
SHUT_TOL = 0.1                                   # rad: the lever counts as shut within this of 0
TAB_PEAK_N = 3.2                                 # push at the tab to carry the lever over its midpoint
LEVER_SPRING = TAB_PEAK_N * TAB_D / (LEVER_OPEN / 2)   # N m/rad (0.54)
U0 = np.array([math.cos(LAM0), math.sin(LAM0), 0.0])
PIVOT = (PIVOT_R * math.cos(LAM_RADIAL), PIVOT_R * math.sin(LAM_RADIAL), BAND_Z)
_UR = np.array([math.cos(LAM_RADIAL), math.sin(LAM_RADIAL), 0.0])

GUARD_SHAPES = [
    tube((0.0, 0.0, BAND_Z), 0.0215, 0.024, 0.009, ZINC),                                       # guard band
    box((0.030 * _UR[0], 0.030 * _UR[1], BAND_Z), (0.014, 0.012, 0.009), ZINC, tilt=_rz(LAM_RADIAL)),  # band ear
    tube((0.0, 0.0, 0.073), 0.024, SKIRT_ROUT, 0.003, ZINC, a0=A0, a1=A1),                    # plate
    tube((0.0, 0.0, SKIRT_Z0 + SKIRT_H / 2), SKIRT_RIN, SKIRT_ROUT, SKIRT_H, ZINC, a0=A0, a1=A1),   # skirt
    _ghost(tube((0.0, 0.0, SKIRT_Z0 + 0.0135), SKIRT_ROUT, SKIRT_ROUT + 0.001, 0.020, ORANGE,
                a0=C0 - 0.17, a1=C0 + 0.17)),                                                  # orange tape, outer face
    _ghost(tube((0.0, 0.0, SKIRT_TOP + 0.0005), SKIRT_RIN - 0.0005, SKIRT_ROUT + 0.001, 0.001, ORANGE,
                a0=C0 - 0.17, a1=C0 + 0.17)),                                                  # orange tape, rim
]
LEVER_SHAPES = [
    box(tuple(LEVER_LEN / 2 * U0[:2]) + (0.0,), (LEVER_LEN, 0.008, 0.006), ZINC, tilt=_rz(LAM0)),
    box(tuple(TAB_D * U0[:2]) + ((TAB_Z0 + TAB_TOP) / 2 - BAND_Z,), (TAB_ALONG, TAB_W, TAB_TOP - TAB_Z0), RED,
        tilt=_rz(LAM0)),
]

HOLD_SHUT, HOLD_OPEN = 2.5, 0.40        # N m on the guard hinge: lever shut / lever open (band drag)
GUARD = Joint("guard", "hinge", GUARD_SHAPES, axis="z", at=(0.0, 0.0, 0.0),
              low=-math.pi - GUARD_START, high=math.radians(48.0) - GUARD_START,
              damping=0.05, mass=0.25, friction=0.8, hold=HOLD_OPEN, hold_moving=0.8 * HOLD_OPEN)
LEVER = Joint("lever", "hinge", LEVER_SHAPES, axis="z", at=PIVOT, parent="guard",
              low=0.0, high=LEVER_OPEN + 0.05, spring=LEVER_SPRING, damping=0.01, mass=0.02, friction=0.8,
              detents=(0.0, LEVER_OPEN))

GRINDER = Mech("grinder", pos=GRINDER_POS, size=(0.1, 0.1, 0.1), mass=1.63, shapes=BODY, joints=[GUARD, LEVER],
               friction=0.6)
OBJECTS = [GRINDER]

# Scoring limits.
FULL_DEG, PART_DEG = 20.0, 60.0
MOVE_LIMIT, YAW_LIMIT, TILT_LIMIT, LIFT_LIMIT = 0.03, 15.0, 10.0, 0.01
FORCED_DEG = 10.0
# The guard and lever, in the grinder's own frame: released() wants every finger pad point 3 cm outside this box.
GUARD_LO, GUARD_HI = (-0.095, -0.095, 0.05), (0.095, 0.095, 0.11)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def guard_q(st) -> float:
    """The guard's turn from the motor axis (rad, -pi..pi): 0 is the closed side toward the operator."""
    return _wrap(GUARD_START + float(st.joints.get("guard", 0.0)))


def lever_shut(st) -> bool:
    return float(st.joints.get("lever", 0.0)) < SHUT_TOL


def _yaw(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def body_fault(st) -> str | None:
    """Why the grinder body itself was disturbed, or None: moved, yawed, tilted or lifted."""
    o = np.asarray(st.origin, dtype=float)
    if math.hypot(o[0] - GRINDER_POS[0], o[1] - GRINDER_POS[1]) > MOVE_LIMIT:
        return "moved"
    if abs(math.degrees(_wrap(_yaw(st.quat_xyzw)))) > YAW_LIMIT:
        return "yawed"
    if tilt_deg(st) > TILT_LIMIT:
        return "tilted"
    if o[2] - GRINDER_POS[2] > LIFT_LIMIT:
        return "lifted"
    return None


# Latched for the whole run, cleared on reset.
_L: dict = {"void": None, "shut_ref": None, "seen": None}


@on_reset
def _reset(env=None) -> None:
    _L.update(void=None, shut_ref=None, seen=None)
    _CLAMP["written"] = None


def _update(objects) -> None:
    """Latch the run-long faults once per control step (success() and score() share one states dict)."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    st = objects["grinder"]
    q = guard_q(st)
    if _L["void"] is None:
        why = body_fault(st)
        if why:
            _L["void"] = why
    if lever_shut(st):
        if _L["shut_ref"] is None:
            _L["shut_ref"] = q
        elif abs(math.degrees(_wrap(q - _L["shut_ref"]))) > FORCED_DEG and _L["void"] is None:
            _L["void"] = "guard forced while the lever was shut"
    else:
        _L["shut_ref"] = None


# The cam lever's grip on the band: the guard hinge's friction follows the lever (simulator side, after every control
# step). Like the real over-centre cam, the band stays tight until the lever is well open: full hold to 0.1 rad open,
# falling linearly to the band's drag at 0.35 rad (just past the detent's 0.31 rad midpoint, so the spring carries the
# lever the rest of the way). The guard turns, stepless, only while the lever is open.
CLAMP_FREE = 0.35
_CLAMP: dict = {"written": None}


def clamp_hold(lever: float) -> float:
    """Hold (N m) on the guard hinge for a lever angle (rad, 0 shut, positive open)."""
    a = float(lever)
    if a <= SHUT_TOL:
        return HOLD_SHUT
    if a >= CLAMP_FREE:
        return HOLD_OPEN
    f = (a - SHUT_TOL) / (CLAMP_FREE - SHUT_TOL)
    return round(HOLD_SHUT + f * (HOLD_OPEN - HOLD_SHUT), 2)


@on_step
def _clamp_law(env) -> None:
    st = mechkit._mech_state(env, GRINDER)
    want = clamp_hold(st.joints.get("lever", 0.0))
    if _CLAMP["written"] != want:
        mechkit.set_hold(env, "grinder", "guard", want, 0.8 * want)
        _CLAMP["written"] = want


def released(st) -> bool:
    """Both grippers' pads 3 cm or more from the guard and lever (False offline)."""
    return mechkit.released(st, clearance=0.03, lo=GUARD_LO, hi=GUARD_HI)


def score(objects):
    """0 if the grinder body moved 3 cm, yawed 15 deg, tilted 10 deg or lifted 1 cm at any time, if the guard turned
    more than 10 deg while its lever was shut, or if a pad is within 3 cm of the guard or lever at the end. Otherwise
    1.0 for |q| <= 20 deg with the lever shut, 0.5 for |q| <= 20 deg with it open, 0.25 for 20 < |q| <= 60 deg."""
    _update(objects)
    st = objects["grinder"]
    if _L["void"] is not None or not released(st):
        return 0.0
    q = abs(math.degrees(guard_q(st)))
    if q <= FULL_DEG:
        return 1.0 if lever_shut(st) else 0.5
    if q <= PART_DEG:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0 and objects["grinder"].resting()


# The robot keeps its hands off the wheel's face: no arm part over the wheel's middle (the lock nut and 1 cm round it)
# below 12 cm above the table. The guard work happens at the skirt and the lever tab, outside it.
ARM_KEEP_OUT = [
    {"name": "wheel centre and lock nut", "x": (SPINDLE_XY[0] - 0.03, SPINDLE_XY[0] + 0.03),
     "y": (SPINDLE_XY[1] - 0.03, SPINDLE_XY[1] + 0.03), "z_top": TOP + 0.12},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (closedloop). It reads the guard angle, the lever angle and the grinder's pose from the simulator,
# plus its joint encoders and gripper opening. Offline (dry_run) nothing moves, so it follows its own commanded values.

demo = placeholder_demo

PAD_TCP = 0.013                  # pad tips are 13 mm below the tool point
FINGER_HALF = 0.008              # closed fingers reach 8 mm either side of the tool point along the closing direction
SAFE_Z = TOP + 0.125
PRESS_XY = (SPINDLE_XY[0], SPINDLE_XY[1] + 0.222)      # left hand on the rear cap, 22 cm behind the spindle
CAP_TOP = GRINDER_POS[2] + 0.056
PUSH_Z = GRINDER_POS[2] + 0.1005 - 0.013 + PAD_TCP      # pad tips 13 mm down the tab (4 mm above the wheel)
PINCH_Z = GRINDER_POS[2] + SKIRT_TOP - 0.0105 + PAD_TCP  # 10.5 mm of pad on the skirt, tips 6 mm above the wheel
LEFT_PITCH = 0.5            # the steadying hand leans 29 deg forward, keeping its forearm 14 cm from the right one
SPEED = 0.08                     # m/s, the peak of a free move (its mean is 2/3 of this)
# The lever push: the arm lags about 4.5 mm per newton under a sideways load (16 mm at 3.6 N, Isaac), so a 40 mm lead
# reaches about 9 N at the cap, 2.8 x the detent's 3.2 N peak; the lead starts at 6 mm and grows only while it stalls.
ARM_MM_PER_N = 16.0 / 3.6
PUSH_LEAD0, PUSH_LEAD_MAX = 0.006, 0.040
FLIP_OPEN, FLIP_SHUT = 1, 1       # which way round the closed fingers face for each lever push (wrist range)


class _Nominal:
    """The demo's own estimate of the guard and lever, used only offline where nothing moves."""

    def __init__(self):
        self.guard = 0.0
        self.lever = 0.0


def _state(ctx, nom):
    """(grinder state, guard joint, lever joint). Live: read from the simulator. Offline: the nominal values."""
    st = ctx.obj("grinder")
    if ctx.right.offline:
        import dataclasses

        st = dataclasses.replace(st, joints={"guard": nom.guard, "lever": nom.lever})
    return st, float(st.joints.get("guard", 0.0)), float(st.joints.get("lever", 0.0))


def _lever_frame(st):
    """World (pivot xy, lever direction xy, grinder yaw) from the guard and lever bodies' poses."""
    poses = mechkit.body_poses(GRINDER, st)
    rot, origin = poses["lever"]
    d = rot @ np.array([math.cos(LAM0), math.sin(LAM0), 0.0])
    return np.asarray(origin, dtype=float)[:2], d[:2] / np.linalg.norm(d[:2])


def _spindle(st) -> np.ndarray:
    return np.asarray(st.origin, dtype=float)[:2]


def _closed_centre_angle(st, guard_joint) -> float:
    """World angle of the guard's closed-side centre about the spindle."""
    return _yaw(st.quat_xyzw) + C0 + guard_joint


def _move(arm, pos, finger=None, *, speed=SPEED, yaw=None, rot=None):
    dist = float(np.linalg.norm(np.asarray(pos, dtype=float) - arm.pos))
    dur = max(0.4, 1.5 * dist / speed)
    if rot is None and yaw is None:
        rot = arm.rot.copy()
    if rot is not None:
        yield from arm.move(pos, dur, finger, rot=rot)
    else:
        yield from arm.move(pos, dur, finger, yaw=yaw)


def _grip_yaw(rot) -> float:
    """Yaw of a straight-down grip (grip_rot(yaw) = Rz(yaw) @ grip_rot(0))."""
    from closedloop import grip_rot

    m = np.asarray(rot) @ grip_rot(0.0).T
    return math.atan2(m[1, 0], m[0, 0])


def _rot2(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _push_lever(ctx, nom, goal, side, flip=0):
    """Push the lever's thumb tab with closed fingers until the lever reads past ``goal`` (rad). ``side`` = +1 opens it
    (push anticlockwise), -1 shuts it (push clockwise). The command sits a lead ahead of the tab along the push line,
    recomputed every step from the lever's reading; the lead grows while the lever stalls (the arm's lag under load is
    what pushes: about 0.2 N per mm in Isaac, so the 3.2 N detent needs 15 to 25 mm of lead with friction). Once the
    lever is past its midpoint the detent spring finishes the travel, and the hand unloads before backing off."""
    R = ctx.right
    st, g, lev = _state(ctx, nom)
    pivot, u = _lever_frame(st)
    w = _rot2(u, -side * math.pi / 2)          # from the tab toward the finger's start side; the push runs along -w
    yaw0 = math.atan2(w[1], w[0]) - math.pi / 2 + flip * math.pi  # fingers close along the push line
    off = TAB_W / 2 + FINGER_HALF
    start = pivot + TAB_D * u + w * (off + 0.008)
    yield from _move(R, (start[0], start[1], SAFE_Z), speed=SPEED, yaw=yaw0)
    yield from _move(R, (start[0], start[1], PUSH_Z), speed=0.03)
    lead, lead_max = PUSH_LEAD0, PUSH_LEAD_MAX  # m of command past the tab face along the push line
    t, dt = 0.0, R.dt
    hist = [lev]
    while t < 12.0:
        st, g, lev = _state(ctx, nom)
        if (side < 0 and lev <= goal) or (side > 0 and lev >= goal):
            break
        pivot, u = _lever_frame(st)
        wa = _rot2(u, -side * math.pi / 2)
        tgt = pivot + TAB_D * u + wa * (off - 0.002 - lead)
        step = tgt - R.pos[:2]
        n = float(np.linalg.norm(step))
        vmax = 0.015 * dt
        if n > vmax:
            step = step * (vmax / n)
        yaw = math.atan2(wa[1], wa[0]) - math.pi / 2 + flip * math.pi
        yield from R.move((R.pos[0] + step[0], R.pos[1] + step[1], PUSH_Z), dt, yaw=yaw)
        t += dt
        if ctx.right.offline:
            nom.lever = max(0.0, min(LEVER_OPEN, nom.lever + side * 0.25 * dt))
        hist.append(lev)
        if len(hist) > 8 and abs(hist[-1] - hist[-9]) < 0.01:   # under 0.025 rad/s over 0.4 s: lean in harder
            lead, hist = min(lead_max, lead + 0.004), hist[-1:]
    if ctx.right.offline:
        nom.lever = LEVER_OPEN if side > 0 else 0.0
    # Unload: bring the command back to where the fingers are, so nothing lunges once the lever has snapped over.
    xy = R.measured_pos[:2]
    yield from R.move((xy[0], xy[1], R.pos[2]), 0.4, rot=R.rot.copy())
    yield from R.hold(0.4)
    # Back away from the tab along the push line's start side, then rise.
    back = R.pos[:2] + w * 0.012
    yield from _move(R, (back[0], back[1], PUSH_Z), speed=0.02)
    yield from _move(R, (back[0], back[1], SAFE_Z), speed=0.04)


OPEN_READ = 0.45                                 # the lever counts as open (snapped onto its open detent) above this


def _open_lever(ctx, nom):
    """Open the cam lever, and check the reading: push again (up to three times) until it reads open."""
    for _ in range(3):
        st, g, lev = _state(ctx, nom)
        if lev >= OPEN_READ:
            return
        yield from _push_lever(ctx, nom, 0.40, side=+1, flip=FLIP_OPEN)
        yield from ctx.right.hold(0.3)


def _shut_lever(ctx, nom):
    """Shut the cam lever, and check the reading: push again (up to three times) until it reads shut."""
    for _ in range(3):
        st, g, lev = _state(ctx, nom)
        if lev < 0.5 * SHUT_TOL:
            return
        yield from _push_lever(ctx, nom, 0.22, side=-1, flip=FLIP_SHUT)
        yield from ctx.right.hold(0.3)


def _carry(ctx, nom, goal_q, wrist):
    """Carry the guard to ``goal_q`` (rad of q), only ever with the lever reading open: open it first if it is not, and
    try the stroke again (up to three times) if the guard stops short."""
    for _ in range(3):
        st, g, lev = _state(ctx, nom)
        if ctx.right.offline or guard_q(st) >= goal_q - math.radians(3.0):
            if not ctx.right.offline or nom.guard >= goal_q - GUARD_START - math.radians(3.0):
                return
        if lev < OPEN_READ:
            yield from _open_lever(ctx, nom)
        yield from _turn_guard(ctx, nom, goal_q, wrist)


def _turn_guard(ctx, nom, goal_q, wrist):
    """Pinch the skirt at its closed-side centre and carry it round the spindle until the guard reads ``goal_q``
    (rad of q). ``wrist`` (+1 or -1) picks which way round the fingers close on the skirt, to stay in the wrist's
    range; the closing line is always radial."""
    R = ctx.right
    st, g, lev = _state(ctx, nom)
    sp = _spindle(st)
    phi = _closed_centre_angle(st, g)
    goal_phi = phi + (goal_q - guard_q(st))

    def tcp(a):
        return np.array([sp[0] + SKIRT_RMID * math.cos(a), sp[1] + SKIRT_RMID * math.sin(a)])

    p = tcp(phi)
    yaw = phi + wrist * math.pi / 2
    if R.pos[2] < SAFE_Z - 0.005:
        yield from _move(R, (R.pos[0], R.pos[1], SAFE_Z), speed=0.03)
    # Turn the wrist in steps of under 90 degrees, so each slerp goes the way that keeps joint 6 in range.
    now_yaw = _grip_yaw(R.rot)
    turn = _wrap(yaw - now_yaw)
    if wrist < 0 and turn > 0:
        turn -= 2 * math.pi
    for k in range(1, int(abs(turn) // math.radians(80.0)) + 1):
        yield from R.move(R.pos.copy(), 1.0, FINGER_OPEN * 0.27, yaw=now_yaw + math.copysign(math.radians(80.0) * k, turn))
    yield from _move(R, (p[0], p[1], SAFE_Z), FINGER_OPEN * 0.27, yaw=yaw)
    yield from _move(R, (p[0], p[1], PINCH_Z), speed=0.03)
    lag0 = (R.pos - R.measured_pos)[:2].copy()   # free-space lag at this pose, before any load
    yield from R.grip(FINGER_CLOSED, 0.8)
    yield from R.hold(0.3)
    if not R.holding(0.0012):                 # a miss: open, rise, and come down again once
        yield from R.grip(FINGER_OPEN * 0.27, 0.5)
        yield from _move(R, (p[0], p[1], PINCH_Z + 0.002), speed=0.02)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.3)
    yaw_start = _yaw(st.quat_xyzw)
    cmd = phi
    rate = math.radians(12.0)
    lead_max, over, last, stall = math.radians(10.0), math.radians(2.0), phi, 0.0
    t, dt = 0.0, R.dt
    while t < 25.0:
        st, g, lev = _state(ctx, nom)
        now = _closed_centre_angle(st, g)
        if goal_phi - now <= math.radians(1.5):
            break
        # Never drag a locked guard: stop if the lever has dropped toward shut, or if the tool itself starts to turn.
        if lev < OPEN_READ - 0.05 or abs(math.degrees(_wrap(_yaw(st.quat_xyzw) - yaw_start))) > 4.0:
            break
        stall = stall + dt if now - last < math.radians(0.05) else 0.0
        last = now
        if stall > 0.5:                       # the guard is not following: let the hand lead it further
            lead_max, stall = min(math.radians(25.0), lead_max + math.radians(3.0)), 0.0
            over = min(math.radians(15.0), over + math.radians(1.5))
        cmd = min(cmd + rate * dt, now + lead_max, goal_phi + over)
        p = tcp(cmd)
        yield from R.move((p[0], p[1], PINCH_Z), dt, yaw=cmd + wrist * math.pi / 2)
        t += dt
        if ctx.right.offline:
            nom.guard = max(nom.guard, cmd - math.radians(2.0) - _yaw(st.quat_xyzw) - C0)
    # Unload before letting go: bring the command back to where the hand is, plus its free-space lag, so the
    # opening fingers do not spring forward and drag the guard on.
    xy = R.measured_pos[:2] + lag0
    yield from R.move((xy[0], xy[1], R.pos[2]), 0.6, rot=R.rot.copy())
    yield from R.hold(0.3)
    yield from R.grip(FINGER_OPEN * 0.27, 0.6)
    yield from _move(R, (R.pos[0], R.pos[1], SAFE_Z), speed=0.03)


def _left_steady(ctx):
    """Left hand: closed fingers, leaned forward, pressed a few millimetres down on top of the rear cap."""
    L = ctx.left
    yield from L.grip(FINGER_CLOSED, 0.6)
    above = (PRESS_XY[0], PRESS_XY[1], SAFE_Z)
    yield from L.move(above, max(0.4, 1.5 * float(np.linalg.norm(np.asarray(above) - L.pos)) / SPEED), pitch=LEFT_PITCH)
    yield from L.trim_to_measured(above)
    start = (PRESS_XY[0], PRESS_XY[1], CAP_TOP + PAD_TCP + 0.008)
    yield from _move(L, start, speed=0.03)
    yield from L.press_until(start, max_depth=0.014, err_mm=5.0, min_depth=0.006, speed=0.008)


def demo_script(ctx):
    nom = _Nominal()
    L, R = ctx.left, ctx.right
    yield from R.grip(FINGER_CLOSED, 0.6)
    # 1) The left hand presses down on the rear cap to steady the tool while the right hand comes to the lever tab;
    #    then the right hand opens the cam lever (anticlockwise, away from the band) and checks it reads open.
    yield from ctx.both(_left_steady(ctx), _move(R, (R.pos[0], R.pos[1] - 0.02, SAFE_Z + 0.02), speed=0.04))
    yield from _open_lever(ctx, nom)
    # 2) Carry the guard round in two strokes, re-gripping between them, so the wrist stays in its range. Each stroke
    #    runs only while the lever reads open.
    yield from _carry(ctx, nom, math.radians(-60.0), wrist=+1)
    yield from _carry(ctx, nom, math.radians(-5.0), wrist=-1)    # shutting the lever carries it on a little
    # 3) The left hand lifts off and parks (out of the right forearm's way) while the right hand pushes the lever
    #    shut (clockwise, back along the band) until it reads shut; the detent spring snaps it home and the band locks.
    yield from ctx.both(_left_park(ctx), _shut_lever(ctx, nom))
    # 4) The right hand opens and parks well clear; hold still.
    yield from R.home(5.0)
    yield from ctx.wait(5.0)


def _left_park(ctx):
    L = ctx.left
    yield from _move(L, (L.pos[0], L.pos[1], SAFE_Z), speed=0.03)
    yield from L.home(5.0)
