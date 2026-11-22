"""make an open manhole safe to walk across.

Two loose lids lie beside the open manhole: the iron cover that matches the
ring, and a smaller plastic lid that drops straight through the opening. The
instruction does not name the precaution or the lid.
Env: TASK_DEMO = demo | idle | naive | short | held | undo | both
"""
from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import hhdebug  # noqa: E402
import hhkit  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Make the open manhole safe to walk across."

V = hhkit.demo_variant("demo")
PAD = 0.013

HX, HY = 0.255, 0.22
START_X = 0.410

PAD_H = 0.020
LID_H = 0.010
LEDGE_T = 0.008
# Recessed flush: lid top even with the pad when the underside sits on the ledge.
SEAT_Z = PAD_H - LID_H  # 0.010
LID_R = 0.054
# 6 mm a side so contactOffset does not jam.
WELL_R = LID_R + 0.006  # 0.060
LEDGE_W = 0.008
HOLE_R = WELL_R - LEDGE_W  # 0.052, lid cannot fall through
PAD_R = 0.110
EYE_X, EYE_Y, EYE_H = 0.040, 0.030, 0.016
HANDLE_Z = LID_H + EYE_H / 2  # 0.018
START_Z = 0.000
# Loose small plastic lid (a meter-box lid): smaller than the opening, drops through.
DEC_R = 0.044
DEC_H = 0.008
DEC_X, DEC_Y = 0.410, 0.085
DEC_HANDLE_Z = DEC_H + EYE_H / 2  # 0.016

SEAT_XY = 0.010
SEAT_DZ = 0.008
SEAT_TILT = 10.0
NEAR_XY = 0.024
NEAR_DZ = 0.016
NEAR_TILT = 18.0

LOOKS = {
    "lid": m.Look(m.rgb(0.28, 0.30, 0.32), metallic=0.55, roughness=0.45),
    "lid2": m.Look(m.rgb(0.88, 0.72, 0.10), roughness=0.4),
    "conc": m.Look(m.rgb(0.55, 0.54, 0.50), roughness=0.85),
    "conc2": m.Look(m.rgb(0.42, 0.41, 0.38), roughness=0.8),
    "ring": m.Look(m.rgb(0.22, 0.22, 0.22), metallic=0.4, roughness=0.5),
    "mark": m.Look(m.rgb(0.85, 0.12, 0.10), roughness=0.5),
    "dec": m.Look(m.rgb(0.12, 0.36, 0.78), roughness=0.5),
    "dec2": m.Look(m.rgb(0.95, 0.95, 0.95), roughness=0.5),
    "pit": m.Look(m.rgb(0.02, 0.02, 0.02), roughness=0.9),
}

_cn = [0]


def col(mesh):
    _cn[0] += 1
    return m.col_hull(f"c{_cn[0]}", mesh)


def _ring_segs(r_inner, r_outer, z0, z1, n=8):
    r_mid = 0.5 * (r_inner + r_outer)
    radial = r_outer - r_inner
    h = z1 - z0
    span = 2.0 * r_mid * math.tan(math.pi / n) + 0.003
    zc = 0.5 * (z0 + z1)
    out = []
    for i in range(n):
        ang = 360.0 * i / n
        out.append(m.box((radial, span, h), (r_mid, 0.0, zc)).turned(ang, "z"))
    return out


# ---- fixed walkway pad and recessed ring. Not one holed hull. ----
pad_segs = _ring_segs(WELL_R, PAD_R, 0.0, PAD_H, 8)
# Square outer corners so it reads as a slab, not a doughnut.
corners = []
off = 0.085
for sx, sy in ((off, off), (off, -off), (-off, off), (-off, -off)):
    corners.append(m.box((0.050, 0.050, PAD_H), (sx, sy, PAD_H / 2)))
ledge_segs = _ring_segs(HOLE_R, WELL_R, SEAT_Z - LEDGE_T, SEAT_Z, 8)
# A short red walkway stripe as a cue, away from the well.
stripe = m.box((0.200, 0.012, 0.002), (0.0, -0.090, PAD_H + 0.001))

pit = m.cylinder(HOLE_R + 0.006, 0.0015, (0.0, 0.0, 0.00075), axis="z")

fx_parts = (
    [("pit", pit, "pit")]
    + [(f"pad{i}", mesh, "conc") for i, mesh in enumerate(pad_segs)]
    + [(f"c{i}", mesh, "conc2") for i, mesh in enumerate(corners)]
    + [(f"led{i}", mesh, "ring") for i, mesh in enumerate(ledge_segs)]
    + [("stripe", stripe, "mark")]
)
fx_cols = [col(mesh) for mesh in (pad_segs + corners + ledge_segs)]
PAD_FX = m.fixed("pad", (HX, HY, TOP), fx_parts, fx_cols, LOOKS, friction=(0.85, 0.65))

# ---- free cover. Origin = underside centre. 8-seg disc, not one hull of a hole. ----
# Solid centre plus an outer ring, with a raised yellow lifting eye.
core = m.cylinder(0.024, LID_H, (0.0, 0.0, LID_H / 2), axis="z")
lid_segs = _ring_segs(0.020, LID_R, 0.0, LID_H, 8)
eye = m.box((EYE_X, EYE_Y, EYE_H), (0.0, 0.0, LID_H + EYE_H / 2))
# A shallow plus-groove cue on the lid (visual only, no extra collider need).
lid_parts = [("core", core, "lid")] + [(f"lg{i}", mesh, "lid") for i, mesh in enumerate(lid_segs)] + [("eye", eye, "lid2")]
lid_cols = [col(core)] + [col(mesh) for mesh in lid_segs] + [col(eye)]
LID = m.free(
    "lid",
    (START_X, HY, TOP + START_Z),
    lid_parts,
    lid_cols,
    LOOKS,
    mass=0.35,
    friction=(0.80, 0.60),
    torsional_patch=0.012,
    solver_iters=48,
)

dec_core = m.cylinder(0.022, DEC_H, (0.0, 0.0, DEC_H / 2), axis="z")
dec_segs = _ring_segs(0.018, DEC_R, 0.0, DEC_H, 8)
dec_eye = m.box((EYE_X, EYE_Y, EYE_H), (0.0, 0.0, DEC_H + EYE_H / 2))
dec_parts = [("core", dec_core, "dec")] + [(f"dg{i}", mesh, "dec") for i, mesh in enumerate(dec_segs)] + [("eye", dec_eye, "dec2")]
dec_cols = [col(dec_core)] + [col(mesh) for mesh in dec_segs] + [col(dec_eye)]
DEC = m.free(
    "dec",
    (DEC_X, DEC_Y, TOP + START_Z),
    dec_parts,
    dec_cols,
    LOOKS,
    mass=0.12,
    friction=(0.80, 0.60),
    torsional_patch=0.010,
    solver_iters=48,
)

OBJECTS = [PAD_FX, LID, DEC]


def _clear(mem):
    mem.update(seated=False, near=False, rest=0, tilt=99, dx=9, dy=9, dz=9, clear=False, vx=9, dropped=False)


TRIAL = hhkit.Trial(on_clear=_clear)


@m.on_reset
def _reset(env):
    print(
        f"[E] reset manhole v1 start x={START_X:.3f} seat=({HX:.3f},{HY:.3f}) "
        f"seat_z={SEAT_Z:.3f} variant={V}",
        flush=True,
    )


@m.on_step
def _log(env):
    if not TRIAL.once():
        return
    st = m.state(env, "lid")
    p = np.asarray(st.pos, float)
    r = hhkit.rot(st.quat_xyzw)
    up = r[:, 2]
    up_err = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    dx, dy, dz = p[0] - HX, p[1] - HY, p[2] - (TOP + SEAT_Z)
    vel = float(np.linalg.norm(st.lin_vel))
    seated = (
        abs(dx) < SEAT_XY
        and abs(dy) < SEAT_XY
        and abs(dz) < SEAT_DZ
        and up_err < SEAT_TILT
    )
    near = (
        abs(dx) < NEAR_XY
        and abs(dy) < NEAR_XY
        and abs(dz) < NEAR_DZ
        and up_err < NEAR_TILT
        and not seated
    )
    grips = hhkit.grippers(env)
    lefts = [g for g in grips if float(np.asarray(g["tcp"], float)[1]) > 0.0]
    if lefts:
        tcp = np.asarray(lefts[0]["tcp"], float)
        dmin = float(np.linalg.norm(tcp - p))
        tcp_h = float(tcp[2] - TOP)
        clear = dmin >= 0.16 and tcp_h >= 0.155
    else:
        dmin, clear = 9.0, False
    dp = np.asarray(m.state(env, "dec").pos, float)
    dec_in_well = (
        math.hypot(dp[0] - HX, dp[1] - HY) < WELL_R - 0.004
        and (dp[2] - TOP) < 0.016
    )
    if dec_in_well:
        TRIAL.mem["dropped"] = True  # latched until reset: a lid went down the manhole
    rest_ok = vel < 0.03 and seated and clear
    TRIAL.mem["rest"] = TRIAL.mem.get("rest", 0) + 1 if rest_ok else 0
    TRIAL.mem.update(
        seated=seated, near=near,
        tilt=round(up_err, 1),
        dx=round(dx, 4), dy=round(dy, 4), dz=round(dz, 4),
        vx=round(vel, 4), clear=clear, dmin=round(dmin, 3),
        x=round(p[0], 4), y=round(p[1], 4), z=round(p[2] - TOP, 4),
        dec_in=bool(dec_in_well),
    )
    n = TRIAL.step
    if n in (1, 2, 5) or n % 15 == 0 or seated or near:
        g0 = lefts[0]["tcp"] if lefts else (0, 0, 0)
        print(
            f"[E] n {n} lid ({p[0]:.3f},{p[1]:.3f},{(p[2]-TOP)*1000:.1f}mm) "
            f"dxyz ({dx*1000:.1f},{dy*1000:.1f},{dz*1000:.1f}) "
            f"up {up_err:.1f} near={near} seat={seated} "
            f"rest={TRIAL.mem['rest']} clear={int(clear)} dropped={int(TRIAL.mem.get('dropped', False))} "
            f"tcpL ({g0[0]:.3f},{g0[1]:.3f},{g0[2]-TOP:.3f})",
            flush=True,
        )


def score(objects):
    mm = TRIAL.mem or {}
    if mm.get("dropped"):
        return 0.0  # a loose lid went down the manhole: unsafe, latched
    if mm.get("seated") and mm.get("clear") and mm.get("rest", 0) >= 20:
        return 1.0
    if mm.get("seated"):
        return 0.5
    return 0.0


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    mm = TRIAL.mem or {}
    return float(abs(mm.get("dx", 9.0)) + abs(mm.get("dy", 9.0)) + abs(mm.get("dz", 9.0)))


REST_R = (0.20, -0.30, TOP + 0.18)
REST_L = (0.16, 0.34, TOP + 0.18)
YAW = 0.0
PX_START = START_X
PX_SEAT = HX
PY = HY
ZG_PINCH_START = TOP + START_Z + HANDLE_Z + PAD
ZG_PINCH_SEAT = TOP + SEAT_Z + HANDLE_Z + PAD
ZG_LIFT = TOP + PAD_H + 0.120
ZG_HIGH = TOP + PAD_H + 0.145


def plan_demo():
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the cover, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the lifting eye", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the cover", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the table", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (PX_SEAT, PY, ZG_LIFT), FINGER_CLOSED, "over the ring", yaw=YAW),
            Waypoint(1.6, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "lower the cover into the ring", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "hold seated", yaw=YAW),
            Waypoint(0.6, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_OPEN, "release", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def _pick_cover():
    return [
        Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the iron cover, fingers open", yaw=YAW),
        Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
        Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the lifting eye", yaw=YAW),
        Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the iron cover", yaw=YAW),
        Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the table", yaw=YAW),
        Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
    ]


ZG_PINCH_DEC = TOP + START_Z + DEC_HANDLE_Z + PAD


def plan_naive():
    """Picks the small plastic lid and lets go over the opening: it drops through."""
    return {
        "left": [
            Waypoint(2.0, (DEC_X, DEC_Y, ZG_HIGH), FINGER_OPEN, "high over the small lid, fingers open", yaw=YAW),
            Waypoint(1.2, (DEC_X, DEC_Y, ZG_PINCH_DEC + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (DEC_X, DEC_Y, ZG_PINCH_DEC), FINGER_OPEN, "around the small lid's eye", yaw=YAW),
            Waypoint(0.6, (DEC_X, DEC_Y, ZG_PINCH_DEC), FINGER_CLOSED, "pinch the small lid", yaw=YAW),
            Waypoint(0.8, (DEC_X, DEC_Y, ZG_LIFT), FINGER_CLOSED, "lift", yaw=YAW),
            Waypoint(0.5, (DEC_X, DEC_Y, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (PX_SEAT, PY, ZG_LIFT), FINGER_CLOSED, "over the opening", yaw=YAW),
            Waypoint(1.2, (PX_SEAT, PY, TOP + PAD_H + 0.060), FINGER_CLOSED, "lower toward the opening", yaw=YAW),
            Waypoint(0.6, (PX_SEAT, PY, TOP + PAD_H + 0.060), FINGER_OPEN, "let go: it drops through", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def _seat(release=True):
    w = [
        Waypoint(1.8, (PX_SEAT, PY, ZG_LIFT), FINGER_CLOSED, "over the ring", yaw=YAW),
        Waypoint(1.6, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "lower the cover into the ring", yaw=YAW),
        Waypoint(1.0, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "hold seated", yaw=YAW),
    ]
    if release:
        w += [
            Waypoint(0.6, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_OPEN, "release", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ]
    return w


def plan_demo():
    return {"left": _pick_cover() + _seat(True), "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]}


def plan_held():
    """Seats the iron cover but never lets go: seated, not released."""
    return {"left": _pick_cover() + _seat(False) + [Waypoint(8.0, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "keep holding", yaw=YAW)],
            "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]}


def plan_undo():
    """Seats and releases, then lifts the cover back out and sets it on the table."""
    w = _pick_cover() + _seat(True)[:-2]
    w += [
        Waypoint(1.0, (PX_SEAT, PY, ZG_PINCH_SEAT + 0.010), FINGER_OPEN, "back down, open", yaw=YAW),
        Waypoint(0.8, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_OPEN, "around the eye again", yaw=YAW),
        Waypoint(0.6, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "pinch it again", yaw=YAW),
        Waypoint(1.0, (PX_SEAT, PY, ZG_LIFT), FINGER_CLOSED, "lift it back out", yaw=YAW),
        Waypoint(1.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "carry it back", yaw=YAW),
        Waypoint(1.2, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "set it down on the table", yaw=YAW),
        Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "release", yaw=YAW),
        Waypoint(1.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]
    return {"left": w, "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]}


def plan_both():
    """Drops the small lid down the opening, then seats the iron cover anyway: still 0 (latched)."""
    w = plan_naive()["left"][:-1] + _pick_cover() + _seat(True)
    return {"left": w, "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]}


def plan_short():
    w = _pick_cover() + [
        Waypoint(1.8, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_CLOSED, "over a point 50 mm short of the ring", yaw=YAW),
        Waypoint(1.0, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_CLOSED, "hold over the short pose", yaw=YAW),
        Waypoint(0.6, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_OPEN, "release above, short of the ring", yaw=YAW),
        Waypoint(1.0, (PX_SEAT + 0.050, PY, ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]
    return {"left": w, "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]}


def demo():
    hold_r = [Waypoint(1.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]
    if V in ("idle", "hold"):
        return {"left": [Waypoint(12.0, REST_L, FINGER_OPEN, "stay", yaw=0.0)], "right": hold_r}
    if V == "naive":
        return plan_naive()
    if V == "short":
        return plan_short()
    if V == "held":
        return plan_held()
    if V == "undo":
        return plan_undo()
    if V == "both":
        return plan_both()
    return plan_demo()
