"""PROBE v1g (stream D): visual dress of a DLB1500A; physics of v1f unchanged.
PROBE v1g (stream D): pick a removable Dutton-Lainson 6454-style handle
from a bench outline (beside the winch, not on the drive axis) and seat
its square-tube hub onto the shaft until the spring-pin depth, then let go.

Coupler lesson (ACCEPT_PATTERNS §6): the handle is a free body. It does
not start on a slider already lined up with the shaft. 1.0 requires
alignment and depth, not an ordinary press.

The square drive is scored geometrically. PhysX contactOffset is 4 mm,
so there is no tight square-on-square collider (flagged). The spring pin
is scored as a depth past the shoulder, not a simulated detent ball.
Do not wind the drum.

Env: TASK_DEMO = demo | idle | short | naive
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import hhkit  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Put the handle on the winch."

V = hhkit.demo_variant("demo")
PAD = 0.013

# Winch in the cue zone, ipsilateral. Shaft along +Y, handle starts
# beside it (offset in X), still in the top-camera view.
CX = 0.32
CY = 0.12

# Dutton-Lainson DLB1500-class, approximate compact zinc body.
# Handle is the official 9-3/8 in (0.238 m) 6454 removable handle.
BODY = (0.120, 0.090, 0.072)
SHAFT_Z = 0.055
SHAFT_X = 0.0
SHAFT_ROOT_Y = 0.048          # body +y face is 0.045; collar sits just outside
SHAFT_LEN = 0.038
SHAFT_SQ = 0.016              # visual square (~5/8 in)
PIN_DEPTH = 0.028             # axial travel of the hub onto the shaft for a full seat
SEAT_TOL = 0.008              # radial window for 1.0
AXIAL_TOL = 0.012             # cap stops ~8 mm short of the geometric root (contactOffset)
PARTIAL_ON = 0.010            # 0.5 once the hub has at least this much axial engagement

# Handle, pivot at the hub centre. Tube along +Y toward the grip.
TUBE_L = 0.036
TUBE_OUT = 0.040  # slightly fat hub: 4 mm contactOffset needs ~32 mm inner opening to ride a 14 mm peg
BAR_D = 0.022
BAR_L = 0.200                 # 238 mm overall, minus the hub
GRIP_Y = TUBE_L / 2 + 0.055   # on the bar, close enough that the tcp stays in REACH
HUB_HALF = TUBE_L / 2

LOOKS = {
    "zinc": m.Look(m.rgb(0.78, 0.80, 0.70), metallic=0.9, roughness=0.25),
    "zinc_dk": m.Look(m.rgb(0.55, 0.56, 0.54), metallic=0.8, roughness=0.35),
    "steel": m.Look(m.rgb(0.45, 0.45, 0.46), metallic=0.7, roughness=0.4),
    "strap": m.Look(m.rgb(0.12, 0.12, 0.13), roughness=0.7),
    "mark": m.Look(m.rgb(0.20, 0.22, 0.25), roughness=0.85),
    "bench": m.Look(m.rgb(0.42, 0.30, 0.16), roughness=0.8),
    "pin": m.Look(m.rgb(0.25, 0.28, 0.32), metallic=0.6, roughness=0.4),
    "red": m.Look(m.rgb(0.70, 0.12, 0.10), roughness=0.45),
    "rubber": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.75),
    "hook": m.Look(m.rgb(0.35, 0.35, 0.36), metallic=0.7, roughness=0.4),
}

_cn = [0]


def col(mesh):
    _cn[0] += 1
    return m.col_hull(f"c{_cn[0]}", mesh)


# ---- winch frame (fixed), origin at (CX, CY, TOP)
body = m.box(BODY, (0.0, 0.0, BODY[2] / 2))
base = m.box((0.130, 0.108, 0.008), (0.0, 0.0, 0.004))
# drum on a Y axle through the body
drum = m.cylinder(0.034, 0.068, (0.008, 0.0, 0.050), axis="y", segments=16)
# a few wraps of dark strap so it reads as a winch, not a gearbox
strap = m.cylinder(0.036, 0.050, (0.008, 0.0, 0.050), axis="y", segments=16)
# brake / gear cover on the -Y side
cover = m.box((0.080, 0.022, 0.070), (0.0, -0.056, 0.046))
# mounting feet
foot_a = m.box((0.018, 0.108, 0.010), (0.056, 0.0, 0.013))
foot_b = m.box((0.018, 0.108, 0.010), (-0.056, 0.0, 0.013))
# shoulder / collar at the drive-shaft root
collar = m.cylinder(0.014, 0.008, (SHAFT_X, SHAFT_ROOT_Y - 0.002, SHAFT_Z), axis="y", segments=12)
# visual square shaft (four thin slabs). No shaft collider: tight square
# fit is not representable at a 4 mm contactOffset (flagged in RISK).
sh_len = SHAFT_LEN
sh_y = SHAFT_ROOT_Y + sh_len / 2
sq = SHAFT_SQ / 2
shaft_a = m.box((SHAFT_SQ, sh_len, 0.003), (SHAFT_X, sh_y, SHAFT_Z + sq - 0.0015))
shaft_b = m.box((SHAFT_SQ, sh_len, 0.003), (SHAFT_X, sh_y, SHAFT_Z - sq + 0.0015))
shaft_c = m.box((0.003, sh_len, SHAFT_SQ), (SHAFT_X + sq - 0.0015, sh_y, SHAFT_Z))
shaft_d = m.box((0.003, sh_len, SHAFT_SQ), (SHAFT_X - sq + 0.0015, sh_y, SHAFT_Z))
# pin groove mark on the shaft (visual only)
groove = m.cylinder(0.009, 0.004, (SHAFT_X, SHAFT_ROOT_Y + PIN_DEPTH, SHAFT_Z), axis="y", segments=10)

# ---- visual-only DLB1500A dress (official drawing DD_DLB1500A; no new colliders).
# Stamped side plates at the drum ends, rounded tops, drum flanges, a gear
# on the brake side, a strap hook and fairlead, slotted base, and a hex nut
# at the shaft. Silhouette from the 7-7/32 in x 5-1/2 in class envelope;
# we do not raise SHAFT_Z or grow the colliding body, so REACH and the v1f
# scores stay. The real unit is a little larger; flagged, not a hidden shrink.
plate_d = m.box((0.112, 0.004, 0.100), (0.004,  0.038, 0.052))
plate_g = m.box((0.112, 0.004, 0.100), (0.004, -0.038, 0.052))
arch_d = m.cylinder(0.056, 0.004, (0.008,  0.038, 0.050), axis="y", segments=20)
arch_g = m.cylinder(0.056, 0.004, (0.008, -0.038, 0.050), axis="y", segments=20)
flange_d = m.cylinder(0.048, 0.005, (0.008,  0.030, 0.050), axis="y", segments=20)
flange_g = m.cylinder(0.048, 0.005, (0.008, -0.030, 0.050), axis="y", segments=20)
gear = m.cylinder(0.046, 0.010, (0.008, -0.050, 0.050), axis="y", segments=18)
pinion = m.cylinder(0.016, 0.010, (SHAFT_X, -0.058, SHAFT_Z), axis="y", segments=12)
nut = m.box((0.018, 0.006, 0.018), (SHAFT_X, SHAFT_ROOT_Y - 0.007, SHAFT_Z))
# three slotted holes on the base (4-3/4 in pattern, scaled to this base)
slot_a = m.box((0.012, 0.022, 0.0016), (0.000,  0.000, 0.0086))
slot_b = m.box((0.012, 0.022, 0.0016), (0.048,  0.000, 0.0086))
slot_c = m.box((0.012, 0.022, 0.0016), (-0.048, 0.000, 0.0086))
# strap hook and a short 2 in webbing tail paying out to +X (DLB accepts 2 in strap)
tail = m.box((0.030, 0.048, 0.004), (0.050, 0.000, 0.050))
hook = m.cylinder(0.007, 0.004, (0.068, 0.000, 0.042), axis="y", segments=10)
hook_s = m.box((0.004, 0.004, 0.016), (0.066, 0.000, 0.050))
# simple fairlead / strap guide on the +X edge of the base
fl_a = m.box((0.010, 0.006, 0.018), (0.062,  0.014, 0.017))
fl_b = m.box((0.010, 0.006, 0.018), (0.062, -0.014, 0.017))

fx_parts = [
    ("base", base, "zinc_dk"),
    ("body", body, "zinc"),
    ("drum", drum, "zinc_dk"),
    ("strap", strap, "strap"),
    ("cover", cover, "zinc"),
    ("fa", foot_a, "zinc_dk"),
    ("fb", foot_b, "zinc_dk"),
    ("collar", collar, "steel"),
    ("sa", shaft_a, "steel"),
    ("sb", shaft_b, "steel"),
    ("sc", shaft_c, "steel"),
    ("sd", shaft_d, "steel"),
    ("groove", groove, "pin"),
    ("plated", plate_d, "zinc"),
    ("plateg", plate_g, "zinc"),
    ("archd", arch_d, "zinc"),
    ("archg", arch_g, "zinc"),
    ("fld", flange_d, "zinc_dk"),
    ("flg", flange_g, "zinc_dk"),
    ("gear", gear, "zinc_dk"),
    ("pinion", pinion, "steel"),
    ("nut", nut, "steel"),
    ("slota", slot_a, "mark"),
    ("slotb", slot_b, "mark"),
    ("slotc", slot_c, "mark"),
    ("tail", tail, "strap"),
    ("hook", hook, "hook"),
    ("hooks", hook_s, "hook"),
    ("fla", fl_a, "zinc_dk"),
    ("flb", fl_b, "zinc_dk"),
]
# Body, collar, and a slightly loose shaft peg (r=7 mm in a 32 mm hub).
# Tight square-on-square is still not representable at a 4 mm contactOffset.
shaft_col = m.cylinder(0.007, SHAFT_LEN, (SHAFT_X, SHAFT_ROOT_Y + SHAFT_LEN / 2, SHAFT_Z), axis="y", segments=10)
fx_cols = [col(base), col(body), col(drum), col(cover), col(foot_a), col(foot_b), col(collar), col(shaft_col)]

WINCH = m.fixed(
    "winch",
    (CX, CY, TOP),
    fx_parts,
    fx_cols,
    LOOKS,
    friction=(1.0, 0.8),
)

# Shaft axis in world (for scoring).
SHAFT_ORIGIN = np.array([CX + SHAFT_X, CY + SHAFT_ROOT_Y, TOP + SHAFT_Z], float)
SHAFT_DIR = np.array([0.0, 1.0, 0.0], float)  # +Y, out of the body

# ---- handle, free body. Pivot = hub centre.
tube = m.box((TUBE_OUT, TUBE_L, TUBE_OUT), (0.0, 0.0, 0.0))
# open mouth of the tube (a darker inner lip facing -Y, toward the shaft)
lip = m.box((TUBE_OUT - 0.006, 0.003, TUBE_OUT - 0.006), (0.0, -HUB_HALF + 0.002, 0.0))
bar = m.cylinder(BAR_D / 2, BAR_L, (0.0, HUB_HALF + BAR_L / 2, 0.0), axis="y", segments=12)
# a slightly fatter grip sleeve
grip = m.cylinder(BAR_D / 2 + 0.004, 0.070, (0.0, HUB_HALF + 0.130, 0.0), axis="y", segments=12)
# spring-pin button, sticking out +Z of the hub (visible from above)
pin = m.cylinder(0.004, 0.016, (0.0, -0.004, TUBE_OUT / 2 + 0.006), axis="z", segments=8)
# Bar collider, a back cap (depth stop), and four loose square-tube walls.
# Convex hull of a ring would fill the bore, so the walls are separate boxes.
# Inner opening 32 mm around a 14 mm peg; 4 mm contactOffset leaves ~1 mm.
bar_col = m.box((BAR_D + 0.008, BAR_L, BAR_D + 0.008), (0.0, HUB_HALF + BAR_L / 2, 0.0))
cap_col = m.box((TUBE_OUT, 0.006, TUBE_OUT), (0.0, HUB_HALF - 0.003, 0.0))
WALL_IN, WALL_OUT = 0.016, 0.020
WALL_T = WALL_OUT - WALL_IN
WALL_C = (WALL_IN + WALL_OUT) / 2
wall_px = m.box((WALL_T, TUBE_L, 2 * WALL_OUT), (WALL_C, 0.0, 0.0))
wall_nx = m.box((WALL_T, TUBE_L, 2 * WALL_OUT), (-WALL_C, 0.0, 0.0))
wall_pz = m.box((2 * WALL_OUT, TUBE_L, WALL_T), (0.0, 0.0, WALL_C))
wall_nz = m.box((2 * WALL_OUT, TUBE_L, WALL_T), (0.0, 0.0, -WALL_C))

# Start: on a marked outline, offset in X (beside, not coaxial), sitting
# on the bench, parallel to the shaft. The mouth faces -Y, toward the
# winch, but 6 cm off the axis.
HX0 = CX - 0.060
HY0 = CY + SHAFT_ROOT_Y + SHAFT_LEN + 0.040
HZ0 = TOP + TUBE_OUT / 2

HANDLE = m.free(
    "handle",
    (HX0, HY0, HZ0),
    [
        ("tube", tube, "zinc"),
        ("lip", lip, "zinc_dk"),
        ("bar", bar, "zinc"),
        ("grip", grip, "rubber"),
        ("pin", pin, "pin"),
    ],
    [col(bar_col), col(cap_col), col(wall_px), col(wall_nx), col(wall_pz), col(wall_nz)],
    LOOKS,
    mass=0.32,
    friction=(0.6, 0.5),
    yaw_deg=0.0,
)

# Start-outline mark on the bench (the cue for "handle lives here").
OL_L = TUBE_L + BAR_L + 0.010
OL_W = TUBE_OUT + 0.012
OUTLINE = m.fixed(
    "outline",
    (HX0, HY0 + BAR_L / 2, TOP),
    [("mark", m.box((OL_W, OL_L, 0.0012), (0.0, 0.0, 0.0006)), "mark")],
    [],
    LOOKS,
)

OBJECTS = [WINCH, HANDLE, OUTLINE]


def _clear(mem):
    mem.update(seated=False, partial=False, rest=0, radial=9, axial=9, vel=9, clear=False)


TRIAL = hhkit.Trial(on_clear=_clear)


def _hub_metrics(st):
    """Radial distance from the shaft axis, and axial depth of the tube
    mouth along the shaft (0 at the root, positive onto the shaft).

    The handle pivot is the hub centre. The mouth is at local y = -HUB_HALF.
    With yaw ~ 0, world mouth is pos + (0, -HUB_HALF, 0) after rotation.
    """
    pos = np.asarray(st.pos, float)
    quat = np.asarray(st.quat_xyzw, float)
    R = hhkit.rot(quat)
    mouth = pos + R @ np.array([0.0, -HUB_HALF, 0.0])
    rel = mouth - SHAFT_ORIGIN
    axial = float(np.dot(rel, SHAFT_DIR))  # 0 at the root; should be ~0 when seated
    radial = float(np.linalg.norm(rel - axial * SHAFT_DIR))
    # How far the mouth has travelled onto the shaft from the free end.
    # Free start is well past SHAFT_LEN. Seated mouth is near axial=0.
    # Engagement depth measured from the shaft tip back toward the root:
    on_amount = SHAFT_LEN - axial if axial <= SHAFT_LEN else 0.0
    if axial < -0.01:
        on_amount = 0.0  # went past / through the body
    return radial, axial, on_amount, mouth


@m.on_step
def _log(env):
    if not TRIAL.once():
        return
    st = m.state(env, "handle")
    radial, axial, on_amt, mouth = _hub_metrics(st)
    vel = 0.0
    try:
        vel = float(np.linalg.norm(st.lin_vel))
    except Exception:
        pass
    # yaw: handle +Y should stay along the shaft +Y (within ~20°)
    try:
        yaw = abs(hhkit.yaw_deg(st))
        if yaw > 90:
            yaw = 180 - yaw
    except Exception:
        yaw = 0.0

    seated = (
        radial <= SEAT_TOL
        and -0.006 <= axial <= AXIAL_TOL
        and on_amt >= PIN_DEPTH - SEAT_TOL
        and yaw <= 20.0
    )
    partial = (not seated) and radial <= 0.020 and on_amt >= PARTIAL_ON and yaw <= 25.0

    grips = hhkit.grippers(env)
    if grips:
        dmin = min(float(np.linalg.norm(np.asarray(g["tcp"], float) - np.asarray(st.pos, float))) for g in grips)
        clear = dmin >= 0.03
    else:
        dmin, clear = 9.0, False

    rest_ok = vel < 0.03 and seated and clear
    TRIAL.mem["rest"] = TRIAL.mem.get("rest", 0) + 1 if rest_ok else 0
    TRIAL.mem.update(
        seated=seated,
        partial=partial,
        radial=round(radial, 4),
        axial=round(axial, 4),
        on=round(on_amt, 4),
        yaw=round(yaw, 1),
        vel=round(vel, 4),
        clear=clear,
        dmin=round(dmin, 3),
        rest=TRIAL.mem["rest"],
    )
    n = TRIAL.step
    if n in (1, 2, 5) or n % 20 == 0 or seated or (n < 80 and n % 10 == 0):
        g0 = grips[0]["tcp"] if grips else (0, 0, 0)
        print(
            f"[D] n {n} hub r={radial*1000:.1f}mm ax={axial*1000:.1f}mm on={on_amt*1000:.1f}mm "
            f"yaw={yaw:.0f} seat={seated} part={partial} rest={TRIAL.mem['rest']} "
            f"tcpL ({g0[0]:.3f},{g0[1]:.3f},{g0[2]-TOP:.3f})",
            flush=True,
        )


def score(objects):
    mm = TRIAL.mem or {}
    if mm.get("seated") and mm.get("clear") and mm.get("rest", 0) >= 20:
        return 1.0
    if mm.get("seated"):
        return 0.5
    if mm.get("partial"):
        return 0.5
    return 0.0


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    mm = TRIAL.mem or {}
    return float(mm.get("radial", 9.0) + abs(mm.get("axial", 9.0)))


REST_R = (0.20, -0.30, TOP + 0.18)
REST_L = (0.20, 0.30, TOP + 0.18)
YAW = float(np.pi / 2)  # fingers close along world x, across a bar that runs along y (demo.py: yaw=0 closes along y)
# Grip at start and at the seated pose (tcp just above the bar).
GX0, GY0 = HX0, HY0 + GRIP_Y
GXS = CX + SHAFT_X
# When seated, mouth is at SHAFT_ORIGIN (axial 0), hub centre is mouth + (0, HUB_HALF, 0)
HYS = CY + SHAFT_ROOT_Y + HUB_HALF
GYS = HYS + GRIP_Y
Z_BAR = TOP + SHAFT_Z  # bar centre height when aligned with the shaft
Z_START = HZ0
Z_HIGH = TOP + 0.18
ZG_START = Z_START + BAR_D / 2 + PAD
ZG_ALIGN = Z_BAR + BAR_D / 2 + PAD


def plan_demo():
    return [
        Waypoint(2.0, (GX0, GY0, Z_HIGH), FINGER_OPEN, "high over the handle", yaw=YAW),
        Waypoint(1.2, (GX0, GY0, ZG_START + 0.010), FINGER_OPEN, "down to the bar", yaw=YAW),
        Waypoint(0.6, (GX0, GY0, ZG_START - 0.010), FINGER_OPEN, "fingers around the bar", yaw=YAW),
        Waypoint(0.8, (GX0, GY0, ZG_START - 0.010), FINGER_CLOSED, "grip", yaw=YAW),
        Waypoint(1.0, (GX0, GY0, ZG_ALIGN + 0.020), FINGER_CLOSED, "lift to shaft height", yaw=YAW),
        Waypoint(1.4, (GXS, GY0, ZG_ALIGN + 0.020), FINGER_CLOSED, "align over the shaft axis", yaw=YAW),
        Waypoint(0.8, (GXS, GY0, ZG_ALIGN), FINGER_CLOSED, "match the shaft height", yaw=YAW),
        Waypoint(2.0, (GXS, GYS, ZG_ALIGN), FINGER_CLOSED, "slide the hub onto the shaft", yaw=YAW),
        Waypoint(1.0, (GXS, GYS, ZG_ALIGN), FINGER_CLOSED, "hold seated", yaw=YAW),
        Waypoint(1.2, (GXS, GYS, ZG_ALIGN), FINGER_OPEN, "let go, still at the hub", yaw=YAW),
        Waypoint(1.2, (GXS, GYS, Z_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def plan_short():
    # Slide only far enough for a partial-on 0.5, not to the pin.
    # Seated GYS has the mouth at the root; a short stop leaves ~15 mm still short.
    gy_short = GYS + (PIN_DEPTH - PARTIAL_ON - 0.005)
    return [
        Waypoint(2.0, (GX0, GY0, Z_HIGH), FINGER_OPEN, "high over the handle", yaw=YAW),
        Waypoint(1.2, (GX0, GY0, ZG_START + 0.010), FINGER_OPEN, "down to the bar", yaw=YAW),
        Waypoint(0.6, (GX0, GY0, ZG_START - 0.010), FINGER_OPEN, "fingers around the bar", yaw=YAW),
        Waypoint(0.8, (GX0, GY0, ZG_START - 0.010), FINGER_CLOSED, "grip", yaw=YAW),
        Waypoint(1.0, (GX0, GY0, ZG_ALIGN + 0.020), FINGER_CLOSED, "lift to shaft height", yaw=YAW),
        Waypoint(1.4, (GXS, GY0, ZG_ALIGN + 0.020), FINGER_CLOSED, "align over the shaft axis", yaw=YAW),
        Waypoint(0.8, (GXS, GY0, ZG_ALIGN), FINGER_CLOSED, "match the shaft height", yaw=YAW),
        Waypoint(1.4, (GXS, gy_short, ZG_ALIGN), FINGER_CLOSED, "slide only part way", yaw=YAW),
        Waypoint(1.2, (GXS, gy_short, ZG_ALIGN), FINGER_OPEN, "let go, still at the hub", yaw=YAW),
        Waypoint(1.2, (GXS, gy_short, Z_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def plan_naive():
    side = (0.20, 0.32, TOP + 0.18)
    return [
        Waypoint(2.0, side, FINGER_OPEN, "aside, not toward the hub", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def demo():
    hold_r = [Waypoint(1.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]
    if V == "idle":
        return {"left": [Waypoint(10.0, REST_L, FINGER_OPEN, "stay", yaw=0.0)], "right": hold_r}
    if V == "short":
        return {"left": plan_short(), "right": hold_r}
    if V == "naive":
        return {"left": plan_naive(), "right": hold_r}
    return {"left": plan_demo(), "right": hold_r}
