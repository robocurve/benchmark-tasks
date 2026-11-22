"""PROBE v1 (stream E): place an uncoupled PTO shaft into its support cradle.

Straight-down pinch-and-place. Not a sleeve (not a retry of the failed
PTO driveline shield). Env: TASK_DEMO = demo | idle | naive | short
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
INSTRUCTION = (
    "The PTO shaft is uncoupled and on the ground. Set it in the yellow cradle so it sits off the ground, then let go."
)

V = hhkit.demo_variant("demo")
PAD = 0.013

# Shaft: short guard-tube section, axis along +y. Origin at mid-body.
SH_X, SH_Y, SH_Z = 0.040, 0.140, 0.040
# Cradle U at (CX, SY). Floor top is U_FLOOR above the table.
CX, SY = 0.320, 0.22
U_FLOOR = 0.030
U_INNER = 0.070          # shaft is 40 mm; 15 mm each side, loose on purpose
U_LEN = 0.050            # along y
U_WALL = 0.010
U_H = 0.040              # wall height above the floor
START_X = 0.410
START_Z = 0.008 + SH_Z / 2   # on the 8 mm pedestal
SEAT_Z = U_FLOOR + SH_Z / 2  # 0.050
HANDLE_Z = SH_Z / 2 + 0.008  # handle centre above the tube top

LOOKS = {
    "yel": m.Look(m.rgb(0.95, 0.78, 0.08), roughness=0.45),
    "yel2": m.Look(m.rgb(0.88, 0.55, 0.08), roughness=0.5),
    "joint": m.Look(m.rgb(0.12, 0.12, 0.13), roughness=0.55),
    "steel": m.Look(m.rgb(0.45, 0.46, 0.48), metallic=0.6, roughness=0.4),
    "paint": m.Look(m.rgb(0.18, 0.38, 0.16), roughness=0.55),
    "wood": m.Look(m.rgb(0.45, 0.32, 0.18), roughness=0.8),
}

_cn = [0]


def col(mesh):
    _cn[0] += 1
    return m.col_hull(f"c{_cn[0]}", mesh)


# ---- fixed cradle on a short tongue stub ----
floor = m.box((U_INNER + 2 * U_WALL, U_LEN, 0.010), (0.0, 0.0, U_FLOOR - 0.005))
left_w = m.box((U_WALL, U_LEN, U_H), (-(U_INNER / 2 + U_WALL / 2), 0.0, U_FLOOR + U_H / 2))
right_w = m.box((U_WALL, U_LEN, U_H), ( (U_INNER / 2 + U_WALL / 2), 0.0, U_FLOOR + U_H / 2))
# tongue stub runs −x, kept off the +x start/carry path
tongue = m.box((0.160, 0.055, 0.024), (-0.070, 0.0, 0.012))
# a dark PIC stub so the tongue reads as an implement hitch, not a plank
pic = m.box((0.030, 0.040, 0.040), (-0.145, 0.0, 0.044))
cr_parts = [
    ("floor", floor, "yel"),
    ("left", left_w, "yel2"),
    ("right", right_w, "yel2"),
    ("tongue", tongue, "paint"),
    ("pic", pic, "steel"),
]
cr_cols = [col(floor), col(left_w), col(right_w), col(tongue), col(pic)]
CRADLE = m.fixed("cradle", (CX, SY, TOP), cr_parts, cr_cols, LOOKS, friction=(0.8, 0.6))

# ---- free shaft. Origin mid-body. ----
tube = m.box((SH_X, SH_Y - 0.024, SH_Z), (0.0, 0.0, 0.0))
# dark joints / yokes at each end
yoke_p = m.box((SH_X + 0.008, 0.016, SH_Z + 0.008), (0.0,  (SH_Y / 2 - 0.008), 0.0))
yoke_n = m.box((SH_X + 0.008, 0.016, SH_Z + 0.008), (0.0, -(SH_Y / 2 - 0.008), 0.0))
# top grab bar, 30 mm in y, yaw 0
handle = m.box((0.030, 0.040, 0.016), (0.0, 0.0, HANDLE_Z))
sh_parts = [
    ("tube", tube, "yel"),
    ("yoke_p", yoke_p, "joint"),
    ("yoke_n", yoke_n, "joint"),
    ("handle", handle, "yel2"),
]
sh_cols = [col(tube), col(yoke_p), col(yoke_n), col(handle)]
SHAFT = m.free(
    "shaft",
    (START_X, SY, TOP + START_Z),
    sh_parts,
    sh_cols,
    LOOKS,
    mass=0.40,
    friction=(0.80, 0.60),
    torsional_patch=0.010,
    solver_iters=48,
)

pedestal_mesh = m.box((0.050, 0.050, 0.008), (0.0, 0.0, 0.004))
PEDESTAL = m.fixed(
    "pedestal",
    (START_X, SY, TOP),
    [("block", pedestal_mesh, "wood")],
    [col(pedestal_mesh)],
    LOOKS,
    friction=(0.12, 0.08),
)
OBJECTS = [CRADLE, PEDESTAL, SHAFT]


def _clear(mem):
    mem.update(seated=False, near=False, rest=0, tilt=99, dx=9, dy=9, dz=9, clear=False, vx=9)


TRIAL = hhkit.Trial(on_clear=_clear)


@m.on_reset
def _reset(env):
    print(
        f"[E] reset ptocradle v1 start x={START_X:.3f} seat x={CX:.3f} "
        f"y={SY} seat_z={SEAT_Z:.3f} variant={V}",
        flush=True,
    )


@m.on_step
def _log(env):
    if not TRIAL.once():
        return
    st = m.state(env, "shaft")
    p = np.asarray(st.pos, float)
    r = hhkit.rot(st.quat_xyzw)
    up = r[:, 2]
    up_err = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    dx, dy, dz = p[0] - CX, p[1] - SY, p[2] - (TOP + SEAT_Z)
    vel = float(np.linalg.norm(st.lin_vel))
    seated = (
        abs(dx) < 0.018
        and abs(dy) < 0.025
        and abs(dz) < 0.010
        and up_err < 15.0
    )
    near = (
        abs(dx) < 0.035
        and abs(dy) < 0.035
        and abs(dz) < 0.020
        and up_err < 25.0
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
    rest_ok = vel < 0.03 and seated and clear
    TRIAL.mem["rest"] = TRIAL.mem.get("rest", 0) + 1 if rest_ok else 0
    TRIAL.mem.update(
        seated=seated, near=near,
        tilt=round(up_err, 1),
        dx=round(dx, 4), dy=round(dy, 4), dz=round(dz, 4),
        vx=round(vel, 4), clear=clear, dmin=round(dmin, 3),
        x=round(p[0], 4), y=round(p[1], 4), z=round(p[2] - TOP, 4),
    )
    n = TRIAL.step
    if n in (1, 2, 5) or n % 15 == 0 or seated or near:
        g0 = lefts[0]["tcp"] if lefts else (0, 0, 0)
        print(
            f"[E] n {n} sh ({p[0]:.3f},{p[1]:.3f},{(p[2]-TOP)*1000:.1f}mm) "
            f"dxyz ({dx*1000:.1f},{dy*1000:.1f},{dz*1000:.1f}) "
            f"up {up_err:.1f} near={near} seat={seated} "
            f"rest={TRIAL.mem['rest']} clear={int(clear)} "
            f"tcpL ({g0[0]:.3f},{g0[1]:.3f},{g0[2]-TOP:.3f})",
            flush=True,
        )


def score(objects):
    mm = TRIAL.mem or {}
    if mm.get("seated") and mm.get("clear") and mm.get("rest", 0) >= 20:
        return 1.0
    if mm.get("seated"):
        return 0.5
    if mm.get("near"):
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    mm = TRIAL.mem or {}
    return float(abs(mm.get("dx", 9.0)) + abs(mm.get("dy", 9.0)) + abs(mm.get("dz", 9.0)))


REST_R = (0.20, -0.30, TOP + 0.18)
REST_L = (0.20, 0.30, TOP + 0.18)
YAW = 0.0
PX_START = START_X
PX_SEAT = CX
PY = SY
ZG_PINCH_START = TOP + START_Z + HANDLE_Z + PAD  # h ≈ 0.069
ZG_PINCH_SEAT = TOP + SEAT_Z + HANDLE_Z + PAD    # h ≈ 0.091
# Carry high enough to clear the U walls (tops at 0.070) plus contactOffset.
ZG_LIFT = TOP + 0.145
ZG_HIGH = TOP + 0.160


def plan_demo():
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the shaft, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the handle", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the shaft", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the pedestal", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (PX_SEAT, PY, ZG_LIFT), FINGER_CLOSED, "over the cradle", yaw=YAW),
            Waypoint(1.4, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "lower the shaft into the cradle", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "hold seated", yaw=YAW),
            Waypoint(0.6, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_OPEN, "release", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def plan_naive():
    park = (PX_START + 0.02, PY, ZG_PINCH_START)
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the shaft, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the handle", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the shaft", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.6, (park[0], park[1], ZG_LIFT), FINGER_CLOSED, "carry it the wrong way, away from the cradle", yaw=YAW),
            Waypoint(1.0, park, FINGER_CLOSED, "set it down short of the cradle", yaw=YAW),
            Waypoint(0.6, park, FINGER_OPEN, "release", yaw=YAW),
            Waypoint(0.8, (park[0], park[1], ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def plan_short():
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the shaft, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the handle", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the shaft", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the pedestal", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_CLOSED, "over a point 50 mm short of the cradle, do not lower", yaw=YAW),
            Waypoint(1.0, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_CLOSED, "hold over the short pose", yaw=YAW),
            Waypoint(0.6, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_OPEN, "release above, short of the cradle", yaw=YAW),
            Waypoint(1.0, (PX_SEAT + 0.050, PY, ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def demo():
    hold_r = [Waypoint(1.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]
    if V in ("idle", "hold"):
        return {"left": [Waypoint(12.0, REST_L, FINGER_OPEN, "stay", yaw=0.0)], "right": hold_r}
    if V == "naive":
        return plan_naive()
    if V == "short":
        return plan_short()
    return plan_demo()
