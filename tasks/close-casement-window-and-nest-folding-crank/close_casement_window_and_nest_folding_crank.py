"""On the station table: a painted timber window stool (room-side sill board) with a lower casement window section
behind it (white frame, 432 x 300 mm hinged sash with a pale-blue pane, black EPDM weatherstrip on the room-side
stop). The sash is hinged on its left edge (+y) and stands 9 degrees open (68 mm at the lock edge). On the stool in
front of it sits a full-size casement operator: a bronze nesting cover (136 x 41 x 38 mm) with its splined shaft
and a die-cast zinc full-flip folding crank (78 mm reach, 14 mm swivel knob), unfolded and pointing left.

Turning the crank half a turn (180 degrees) through the room side closes the sash onto the seal (geared,
-0.050 rad of sash per rad of crank). At exactly that phase the folded handle fits the cover's channel and knob well;
off phase by more than about 4 degrees it lands on the cover walls and stays proud. The crank can turn on another
20 degrees (weatherstrip compression) past the cove.

Requires -E object_jitter=0: the layout is fixed and the demo plans from it.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=close_casement_window_and_nest_folding_crank.py -E object_jitter=0 \\
           -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Crank the casement window shut with the folding handle, then fold the handle flush into its cover pocket and let go." \\
           --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402, F401
import closedloop  # noqa: E402, F401
import mechkit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import (  # noqa: E402
    Gear,
    Joint,
    Mech,
    box,
    cyl,
    on_reset,
    on_step,
    step_count,
    touch_state,
    watch_touch,
)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HZ = 20.0
SUCCESS_HOLD_S = 8.0

# -- palette (high contrast at 320x176) --------------------------------------------------------------------------------
CREAM = (0.92, 0.90, 0.86)       # painted timber stool
WHITE = (0.96, 0.96, 0.95)       # window frame and sash
PANE = (0.68, 0.84, 0.94)        # pale-blue acrylic pane
EPDM = (0.10, 0.10, 0.12)        # black weatherstrip
BRONZE = (0.20, 0.19, 0.21)      # dark bronze nesting cover
ZINC = (0.80, 0.82, 0.85)        # spline and clevis
SILVER = (0.84, 0.86, 0.89)      # satin crank arm and knob


def _qz(rad: float) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) for a turn of ``rad`` about +z."""
    return (math.cos(rad / 2.0), 0.0, 0.0, math.sin(rad / 2.0))


def _look(shape):
    return dataclasses.replace(shape, collide=False)


# -- layout (station frame, m) -----------------------------------------------------------------------------------------
# The Mech origin is the foot of the spline shaft on the table: x 0.295, y -0.030. Everything below is in that frame.
OP_X, OP_Y = 0.295, -0.030
WIN_DY = 0.030             # the window is centred on the station (y 0); the operator sits 3 cm right of centre
STOOL_TOP = 0.032          # 32 mm painted timber stool board
COVER_TOP = 0.070          # nesting cover rim (38 mm tall on the stool)
FLOOR_TOP = 0.042          # pocket floor
SPLINE_Z = 0.068           # spline joint frame
LEVER_Z = 0.070            # fold hinge (axis x in the spline frame)
ARM_R = 0.078              # crank reach: spline axis to knob axis
KNOB_R, KNOB_H, KNOB_ZL = 0.007, 0.018, 0.016   # 14 mm x 18 mm knob, centre 16 mm above the fold axis
KNOB_Z = LEVER_Z + KNOB_ZL                       # knob centre height (unfolded) above the table

# Window: sash inner face (closed) at local x 0.105 (world 0.400), 38 mm deep, 432 x 300 mm.
SASH_IN, SASH_D = 0.105, 0.038
SASH_W, SASH_Y0 = 0.432, 0.216   # sash spans y -0.216 .. +0.216, hinge on the +y edge
SASH_Z0, SASH_Z1 = 0.080, 0.380
OPEN_RAD = math.radians(9.0)     # start: 9 degrees open (68 mm at the lock edge)
GEAR_RATIO = -0.050              # sash rad per spline rad (operator worm and arm, as a coupling)

# -- scoring constants -------------------------------------------------------------------------------------------------
INDEX = math.pi                  # spline phase of the cover's cove
SASH_SEALED = -0.150             # sash <= this: lock edge within 3 mm of the weatherstrip (432 mm sash)
INDEX_TOL = math.radians(5.0)    # |spline - pi| <= 5 deg
NESTED = math.radians(175.0)     # lever >= 175 deg: arm top at or below the cover rim
STILL_RAD = math.radians(1.0) / HZ   # under 1 deg/s on every joint
END_HOLD_STEPS = 20              # 1.0 s
RELEASE_CLEARANCE = 0.030
# The operator region released() measures from (Mech frame): the cover, the spline and the crank's whole sweep.
OP_LO, OP_HI = (-0.030, -0.095, 0.030), (0.030, 0.120, 0.115)

# -- sash shapes, built closed in the hinge frame then turned open about the hinge -------------------------------------
HINGE_AT = (SASH_IN + SASH_D, SASH_Y0 + WIN_DY, 0.0)    # hinge on the outer face of the +y stile


def _rot_xy(x: float, y: float, a: float) -> tuple[float, float]:
    return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))


def _sash_box(cx: float, cy: float, cz: float, size, color):
    """A sash member given closed in the hinge frame (x back from the outer face, y from the hinge), turned open."""
    x, y = _rot_xy(cx, cy, OPEN_RAD)
    return box((float(x), float(y), float(cz)), size, color, tilt=_qz(OPEN_RAD))


_SZM = (SASH_Z0 + SASH_Z1) / 2.0
_SH = SASH_Z1 - SASH_Z0
SASH_SHAPES = (
    _sash_box(-SASH_D / 2, -0.025, _SZM, (SASH_D, 0.050, _SH), WHITE),                 # hinge stile
    _sash_box(-SASH_D / 2, -SASH_W + 0.025, _SZM, (SASH_D, 0.050, _SH), WHITE),        # lock stile
    _sash_box(-SASH_D / 2, -SASH_W / 2, SASH_Z0 + 0.030, (SASH_D, SASH_W - 0.100, 0.060), WHITE),  # bottom rail
    _sash_box(-SASH_D / 2, -SASH_W / 2, SASH_Z1 - 0.025, (SASH_D, SASH_W - 0.100, 0.050), WHITE),  # top rail
    _sash_box(-SASH_D / 2, -SASH_W / 2, (SASH_Z0 + 0.060 + SASH_Z1 - 0.050) / 2,
              (0.003, SASH_W - 0.100, _SH - 0.110), PANE),                               # 3 mm acrylic pane
)

# -- the window and operator -------------------------------------------------------------------------------------------
_FX0, _FX1 = 0.087, 0.160      # frame depth (local x)
_FXM, _FXD = (_FX0 + _FX1) / 2, _FX1 - _FX0
# room-side stop face at local x 0.087 (world 0.382); the sash closes 9 mm behind it (the seal fills the gap)

WINDOW = Mech(
    "window",
    pos=(OP_X, OP_Y, TOP),
    mass=None,
    friction=0.8,
    shapes=[
        # Painted timber stool (room-side sill board), 132 x 500 x 32 mm
        box((0.021, WIN_DY, STOOL_TOP / 2), (0.132, 0.500, STOOL_TOP), CREAM),
        # Window frame: jambs, head and frame sill, 73 mm deep
        box((_FXM, WIN_DY + 0.2325, 0.2205), (_FXD, 0.025, 0.377), WHITE),
        box((_FXM, WIN_DY - 0.2325, 0.2205), (_FXD, 0.025, 0.377), WHITE),
        box((_FXM, WIN_DY, 0.3965), (_FXD, 0.490, 0.025), WHITE),
        box((_FXM, WIN_DY, 0.053), (_FXD, 0.490, 0.042), WHITE),
        # Room-side stop lip round the opening (9 mm), its face 9 mm in front of the closed sash
        box((0.0915, WIN_DY + 0.210, 0.2295), (0.009, 0.020, 0.300), WHITE),
        box((0.0915, WIN_DY - 0.210, 0.2295), (0.009, 0.020, 0.300), WHITE),
        box((0.0915, WIN_DY, 0.372), (0.009, 0.400, 0.016), WHITE),
        box((0.0915, WIN_DY, 0.082), (0.009, 0.400, 0.010), WHITE),
        # Black EPDM D-profile weatherstrip on the stop face (look-only, 6 mm)
        _look(box((0.099, WIN_DY + 0.205, 0.2295), (0.006, 0.008, 0.290), EPDM)),
        _look(box((0.099, WIN_DY - 0.205, 0.2295), (0.006, 0.008, 0.290), EPDM)),
        _look(box((0.099, WIN_DY, 0.369), (0.006, 0.402, 0.008), EPDM)),
        _look(box((0.099, WIN_DY, 0.090), (0.006, 0.402, 0.008), EPDM)),
        # Nesting cover (bronze, 136 x 41 x 38 mm on the stool): floor, hub cap, 22 mm channel, 28 mm knob well, end
        box((0.0, 0.043, 0.037), (0.041, 0.136, 0.010), BRONZE),
        box((0.0, -0.019, 0.056), (0.041, 0.012, 0.028), BRONZE),
        box((0.01575, 0.038, 0.056), (0.0095, 0.048, 0.028), BRONZE),
        box((-0.01575, 0.038, 0.056), (0.0095, 0.048, 0.028), BRONZE),
        box((0.01725, 0.081, 0.056), (0.0065, 0.038, 0.028), BRONZE),
        box((-0.01725, 0.081, 0.056), (0.0065, 0.038, 0.028), BRONZE),
        box((0.0, 0.1055, 0.056), (0.041, 0.011, 0.028), BRONZE),
    ],
    joints=[
        # 11/32 in spline shaft and clevis: turns about z, 0 .. +200 deg (weatherstrip compression past the cove)
        Joint(
            "spline",
            "hinge",
            shapes=(
                cyl((0.0, 0.0, -0.011), 0.0085, 0.026, ZINC, sides=16),
                box((0.0, 0.0, 0.0075), (0.016, 0.018, 0.011), ZINC),
            ),
            axis="z",
            at=(0.0, 0.0, SPLINE_Z),
            low=-0.17,
            high=3.4907,
            damping=0.015,
            hold=0.12,
            hold_moving=0.10,
            mass=0.040,
            armature=5e-4,
        ),
        # Full-flip folding handle: hinge about x on the clevis, 0 (unfolded) .. pi (folded), over-centre detent spring
        Joint(
            "lever",
            "hinge",
            shapes=(
                box((0.0, 0.045, 0.0075), (0.010, 0.078, 0.009), SILVER),
                cyl((0.0, ARM_R, KNOB_ZL), KNOB_R, KNOB_H, SILVER, sides=16),
            ),
            axis="x",
            at=(0.0, 0.0, LEVER_Z - SPLINE_Z),
            parent="spline",
            low=0.0,
            high=math.pi,
            spring=0.06,
            damping=0.008,
            detents=(0.0, math.pi),
            mass=0.060,
        ),
        # Casement sash on its +y hinge, driven by the operator: rest = 9 deg open, closed on the seal at -0.157
        Joint(
            "sash",
            "hinge",
            shapes=SASH_SHAPES,
            axis="z",
            at=HINGE_AT,
            low=-0.176,
            high=0.0087,
            damping=0.005,
            mass=1.2,
            armature=1e-3,
            follows=Gear("spline", GEAR_RATIO),
        ),
    ],
)

OBJECTS = [WINDOW]

# Any finger pad on the sash or pane (the operator, not a push, must close the window).
watch_touch("window", bodies=("sash",), min_force=0.05)

# Whole-robot keep-out: the sash and pane, open or closed, up to the frame head.
ARM_KEEP_OUT = [
    {"name": "casement sash and pane", "x": (OP_X + 0.092, OP_X + 0.235), "y": (-0.222, 0.222), "z_top": TOP + 0.41},
]


# -- scorer ------------------------------------------------------------------------------------------------------------
_STATE: dict = {}


def _clear_state() -> None:
    _STATE.clear()
    _STATE.update(step_id=-1, last=None, sash_touched=False, hold={1.0: 0, 0.5: 0, 0.25: 0})


_clear_state()


def sash_sealed(j) -> bool:
    return float(j.get("sash", 0.0)) <= SASH_SEALED


def indexed(j) -> bool:
    return abs(float(j.get("spline", 0.0)) - INDEX) <= INDEX_TOL


def nested(j) -> bool:
    return float(j.get("lever", 0.0)) >= NESTED


def tier(j) -> float:
    """1.0 sealed, indexed and nested; 0.5 sealed and indexed; 0.25 sealed; else 0."""
    if not sash_sealed(j):
        return 0.0
    if not indexed(j):
        return 0.25
    return 1.0 if nested(j) else 0.5


def _advance(win, *, released_now: bool, sash_touch: bool, step_id) -> None:
    if step_id is not None:
        if step_id == _STATE["step_id"]:
            return
        _STATE["step_id"] = step_id
    j = {k: float(v) for k, v in win.joints.items()}
    if sash_touch:
        _STATE["sash_touched"] = True
    last = _STATE["last"]
    still = last is not None and all(abs(j.get(k, 0.0) - last.get(k, 0.0)) < STILL_RAD for k in ("spline", "lever", "sash"))
    _STATE["last"] = j
    t = tier(j)
    for level in (1.0, 0.5, 0.25):
        ok = released_now and still and t >= level
        _STATE["hold"][level] = _STATE["hold"][level] + 1 if ok else 0


def _current() -> float:
    if _STATE["sash_touched"]:
        return 0.0
    for level in (1.0, 0.5, 0.25):
        if _STATE["hold"][level] >= END_HOLD_STEPS:
            return level
    return 0.0


def _released(win, env=None) -> bool:
    return mechkit.released(win, clearance=RELEASE_CLEARANCE, lo=OP_LO, hi=OP_HI, environment=env)


@on_reset
def _reset(env=None) -> None:
    _clear_state()


@on_step
def _step(env) -> None:
    if env is None:
        return
    win = mechkit._mech_state(env, WINDOW)
    _advance(win, released_now=_released(win, env), sash_touch=touch_state("window").touched, step_id=step_count())


def score(objects) -> float:
    """0 if a finger pad ever touched the sash. Otherwise, held for the last 1.0 s with both hands 30 mm clear of the
    operator and every joint under 1 deg/s: 1.0 sash sealed (<= -0.150 rad), spline within 5 deg of the cove and
    handle folded >= 175 deg; 0.5 sealed and indexed; 0.25 sealed only; 0 otherwise."""
    if step_count() == 0:
        # Offline (no live on_step): each score() call advances one 20 Hz step.
        win = objects["window"]
        _advance(win, released_now=_released(win), sash_touch=touch_state("window").touched, step_id=None)
    return _current()


def success(objects) -> bool:
    return _current() >= 1.0


def distance(objects) -> float:
    j = objects["window"].joints
    return float(max(0.0, float(j.get("sash", 0.0)) - SASH_SEALED) * 10.0
                 + abs(float(j.get("spline", 0.0)) - INDEX) + max(0.0, NESTED - float(j.get("lever", 0.0))))


# -- scripted closed-loop demo -----------------------------------------------------------------------------------------
demo = placeholder_demo

CRANK_Z = TOP + 0.097          # TCP while cranking: pad tips at TOP+0.084, above the crank arm (top TOP+0.082)
HOVER_Z = TOP + 0.150
CRADLE = -0.013                # 26 mm pad gap around the 14 mm knob: pushes it round, never pinches it
WIDE = -0.022                  # 44 mm
PINCH = -0.004                 # close toward 8 mm on the 14 mm knob (firm pinch for the fold)
LEAD_MAX = 0.40                # rad: how far the pushing hand may run ahead of the measured crank angle
CRANK_RATE = 0.030 / ARM_R / HZ  # rad per step: knob speed 0.03 m/s
FREE_LEAD = 0.075              # rad: pad-to-knob play in the cradle (6 mm at 78 mm)
FOLD_RELEASE = math.radians(112.0)
HANDOFF = math.radians(70.0)   # left arm turns the crank to here (knob just right of centre), the right arm the rest
FOLD_YAW = math.pi / 2.0       # right arm pads across x
FOLD_GRIP_ZL = KNOB_ZL + 0.006 # the knob point between the pads (lever frame)
TCP_ABOVE_GRIP = 0.005


def knob_xy(q: float) -> tuple[float, float]:
    """World x, y of the knob axis at spline angle q (lever unfolded)."""
    return (OP_X - ARM_R * math.sin(q), OP_Y + ARM_R * math.cos(q))


def fold_tcp(theta: float) -> tuple[float, float, float]:
    """TCP for a straight-down pinch across x on the knob, with the handle folded theta at spline = pi."""
    yl = ARM_R * math.cos(theta) - FOLD_GRIP_ZL * math.sin(theta)
    zl = ARM_R * math.sin(theta) + FOLD_GRIP_ZL * math.cos(theta)
    return (OP_X, OP_Y - yl, TOP + LEVER_Z + zl + TCP_ABOVE_GRIP)


def _joints(ctx) -> dict:
    return dict(ctx.obj("window").joints)


def _crank(ctx, arm, q_from: float, q_to: float, yaw_of, max_steps: int):
    """Push the knob round the room-side arc from q_from to q_to with the open pads (cradle), servoing on the
    measured spline angle: the hand runs ahead of the knob by up to LEAD_MAX, never faster than 0.03 m/s."""
    phi = q_from
    for _ in range(max_steps):
        q = q_from if ctx.offline else float(_joints(ctx).get("spline", 0.0))
        if ctx.offline:
            q = max(q_from, phi - FREE_LEAD)
        if q >= q_to:
            break
        phi = min(phi + CRANK_RATE, q + LEAD_MAX, q_to + FREE_LEAD + 0.02)
        x, y = knob_xy(phi)
        yield from arm.move((x, y, CRANK_Z), 1.0 / HZ, CRADLE, yaw=yaw_of(phi))
    return phi


def _centre_on_knob(ctx, arm, yaw_of, q_offline: float, duration=0.6):
    """Centre the open cradle on the knob at the measured crank angle (no push)."""
    q = q_offline if ctx.offline else float(_joints(ctx).get("spline", 0.0))
    x, y = knob_xy(q)
    yield from arm.move((x, y, CRANK_Z), duration, CRADLE, yaw=yaw_of(q))
    return q


def demo_script(ctx):
    """1) Left arm: pads open in a 26 mm cradle round the knob, pushes the crank from 0 to 70 deg along the room-side
    arc, centres, opens and goes home. 2) Right arm: cradles the knob at 70 deg, pushes it on to 180 deg (the sash
    closes on its seal), trims the phase to within 1.5 deg of the cove from the measured spline angle, centres and
    lifts. 3) Right arm: pinches the knob across x, swings the handle up and over in the x = 0.295 m plane to 112 deg,
    past the detent's centre, opens and lets the detent spring and gravity fold it into the pocket, then goes home."""
    L, R = ctx.left, ctx.right
    yaw_l = lambda q: q - math.pi / 2.0  # noqa: E731 - pads across the arc's tangent
    yaw_r = lambda q: q - math.pi / 2.0  # noqa: E731

    # ---- 1. Left arm: first 70 degrees ---------------------------------------------------------------------------
    x0, y0 = knob_xy(0.0)
    yield from L.move((x0, y0, HOVER_Z), 5.5, WIDE, yaw=yaw_l(0.0))
    yield from L.trim_to_measured((x0, y0, HOVER_Z), max_duration=0.6)
    yield from L.move((x0, y0, CRANK_Z), 1.6, WIDE, yaw=yaw_l(0.0))
    yield from L.grip(CRADLE, 0.5)
    phi = yield from _crank(ctx, L, 0.0, HANDOFF, yaw_l, 400)
    q = yield from _centre_on_knob(ctx, L, yaw_l, phi - FREE_LEAD)
    yield from L.grip(WIDE, 0.4)
    lx, ly = knob_xy(q)
    yield from L.move((lx, ly, HOVER_Z), 1.4, WIDE, yaw=yaw_l(q))
    yield from L.home(5.5)

    # ---- 2. Right arm: the rest of the half turn, phase index-------------------------------------------------------
    q = HANDOFF if ctx.offline else float(_joints(ctx).get("spline", 0.0))
    rx, ry = knob_xy(q)
    yield from R.move((rx, ry, HOVER_Z), 5.5, WIDE, yaw=yaw_r(q))
    yield from R.trim_to_measured((rx, ry, HOVER_Z), max_duration=0.6)
    yield from R.move((rx, ry, CRANK_Z), 1.6, WIDE, yaw=yaw_r(q))
    yield from R.grip(CRADLE, 0.5)
    target = INDEX - 0.015
    phi = yield from _crank(ctx, R, q, target, yaw_r, 500)
    # Trim: centre on the knob, then nudge from whichever side is needed until within 1.5 deg of the cove.
    for _ in range(4):
        q = INDEX if ctx.offline else float(_joints(ctx).get("spline", 0.0))
        err = q - INDEX
        if abs(err) <= math.radians(1.5):
            break
        side = -1.0 if err > 0 else 1.0
        phi = q
        for _ in range(60):
            q = INDEX if ctx.offline else float(_joints(ctx).get("spline", 0.0))
            if (q - INDEX) * side >= -math.radians(0.5):
                break
            phi = phi + side * min(CRANK_RATE * 0.5, max(0.0, LEAD_MAX - abs(phi - q)))
            x, y = knob_xy(phi)
            yield from R.move((x, y, CRANK_Z), 1.0 / HZ, CRADLE, yaw=yaw_r(phi))
        yield from _centre_on_knob(ctx, R, yaw_r, INDEX, 0.5)
        yield from R.hold(0.3)
    q = yield from _centre_on_knob(ctx, R, yaw_r, INDEX, 0.5)
    yield from R.grip(WIDE, 0.4)
    x, y = knob_xy(q)
    yield from R.move((x, y, HOVER_Z), 1.4, WIDE, yaw=yaw_r(q))

    # ---- 3. Right arm: pinch across x and fold over the detent ------------------------------------------------------
    t0 = fold_tcp(0.0)
    yield from R.move((t0[0], t0[1], HOVER_Z), 1.6, WIDE, yaw=FOLD_YAW)
    yield from R.trim_to_measured((t0[0], t0[1], HOVER_Z), max_duration=0.6)
    yield from R.move(t0, 1.6, WIDE, yaw=FOLD_YAW)
    yield from R.grip(PINCH, 0.6)
    yield from R.hold(0.3)
    if not R.holding(0.010):
        yield from R.grip(WIDE, 0.4)
        yield from R.move(t0, 0.6, WIDE, yaw=FOLD_YAW)
        yield from R.grip(PINCH, 0.6)
        yield from R.hold(0.3)
    n = int(round(FOLD_RELEASE * 0.081 / 0.025 * HZ))   # 0.025 m/s along the knob's arc
    for i in range(1, n + 1):
        th = FOLD_RELEASE * (i / n)
        yield from R.move(fold_tcp(th), 1.0 / HZ, PINCH, yaw=FOLD_YAW)
    yield from R.grip(WIDE, 0.5)
    yield from R.hold(1.0)
    tr = fold_tcp(FOLD_RELEASE)
    yield from R.move((tr[0], tr[1], tr[2] + 0.012), 0.6, WIDE, yaw=FOLD_YAW)
    yield from R.move((OP_X - 0.02, OP_Y - 0.03, HOVER_Z), 1.6, WIDE, yaw=FOLD_YAW)
    # out to the right and back before home, so the turning fingers never swing onto the right jamb
    yield from R.move((0.230, -0.250, HOVER_Z), 3.0, WIDE, yaw=FOLD_YAW)
    yield from R.home(6.0)
    yield from ctx.wait(10.0)
