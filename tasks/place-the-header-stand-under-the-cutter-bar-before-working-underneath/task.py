"""PROBE v1 (stream E): place a yellow header stand under a raised cutter bar
so the base sits on the table and the saddle is under the bar.

Straight-down pinch-and-place. No sleeve, no hook.
Env: TASK_DEMO = demo | idle | naive | short
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
    "The header is raised. Set the yellow stand under the cutter bar so it sits on the ground and the saddle is under the bar."
)

V = hhkit.demo_variant("demo")
PAD = 0.013

# Cutter bar: axis along +y, underside at BAR_Z. Approach from +x.
HX, SY = 0.280, 0.22
BAR_Z = 0.140          # underside; v1 ears hit the +x face (bar was at 0.130)
BAR_X, BAR_Y, BAR_H = 0.018, 0.160, 0.018
# Stand: origin mid-base. Saddle floor at STAND_H.
STAND_H = 0.100
BASE_W = 0.060          # x-width; v3 keeps the base clear of the bar
HANDLE_DX = 0.018      # v3: short +x tab; the post itself is already +x of the bar
HANDLE_Z = 0.065
START_X = 0.400
SX = 0.330               # post stays +x of the bar; fork reaches under it
SEAT_Z = 0.000
START_Z = 0.008        # on the pedestal

LOOKS = {
    "yel": m.Look(m.rgb(0.95, 0.78, 0.08), roughness=0.45),
    "yel2": m.Look(m.rgb(0.88, 0.55, 0.08), roughness=0.5),
    "steel": m.Look(m.rgb(0.22, 0.23, 0.25), metallic=0.6, roughness=0.4),
    "steel2": m.Look(m.rgb(0.35, 0.36, 0.38), metallic=0.55, roughness=0.45),
    "guard": m.Look(m.rgb(0.55, 0.56, 0.58), metallic=0.7, roughness=0.3),
    "stripe": m.Look(m.rgb(0.95, 0.78, 0.08), roughness=0.5),
    "post": m.Look(m.rgb(0.18, 0.18, 0.19), metallic=0.4, roughness=0.5),
    "wood": m.Look(m.rgb(0.45, 0.32, 0.18), roughness=0.8),
    "green": m.Look(m.rgb(0.18, 0.38, 0.16), roughness=0.55),
    "green2": m.Look(m.rgb(0.12, 0.28, 0.12), roughness=0.55),
    "reel": m.Look(m.rgb(0.95, 0.78, 0.08), roughness=0.4),
    "tine": m.Look(m.rgb(0.90, 0.70, 0.10), metallic=0.3, roughness=0.35),
    "auger": m.Look(m.rgb(0.40, 0.42, 0.44), metallic=0.65, roughness=0.35),
}

_cn = [0]


def col(mesh):
    _cn[0] += 1
    return m.col_hull(f"c{_cn[0]}", mesh)


# ---- fixed cutter-bar section, held up on two posts behind the bar (-x) ----
bar = m.box((BAR_X, BAR_Y, BAR_H), (0.0, 0.0, BAR_Z + BAR_H / 2))
# yellow stripe on the +x face so the raised bar reads at 320x176
stripe = m.box((0.003, BAR_Y * 0.9, BAR_H * 0.7), (BAR_X / 2 + 0.0015, 0.0, BAR_Z + BAR_H / 2))
# pointed guards along the -x (cutting) face; they do not hang below the underside
guard_meshes = []
for i, gy in enumerate((-0.060, -0.030, 0.000, 0.030, 0.060)):
    # triangular finger pointing -x, sitting on the bar's front face
    g = (
        m.prism(
            [(0.0, -0.007), (0.0, 0.007), (-0.028, 0.0)],
            0.0,
            0.012,
        )
        .moved((-(BAR_X / 2), gy, BAR_Z + 0.003))
    )
    guard_meshes.append((f"g{i}", g, "guard"))
# two posts behind the bar, from the table up to the bar
POST_X = -0.060          # v1 posts at -0.040 overlapped the 70 mm base at the seat
post_l = m.box((0.020, 0.020, BAR_Z + BAR_H), (POST_X,  0.070, (BAR_Z + BAR_H) / 2))
post_r = m.box((0.020, 0.020, BAR_Z + BAR_H), (POST_X, -0.070, (BAR_Z + BAR_H) / 2))
# a light cross-brace so it reads as a frame, not two sticks
brace = m.box((0.016, 0.140, 0.012), (POST_X, 0.0, 0.040))

# v3vis: make it read as a combine header at 320x176 — yellow reel,
# side sheets, and an auger trough behind the bar. All new parts sit
# -x of the bar or well above BAR_Z so they cannot meet the fork.
# Reel axis at (-0.055, 0, 0.205); 0.040 bats stay at z>=0.165, x<=0.265 world.
reel = m.cylinder(0.012, 0.170, (-0.055, 0.0, 0.205), axis="y")
reel_end_l = m.cylinder(0.018, 0.008, (-0.055,  0.089, 0.205), axis="y")
reel_end_r = m.cylinder(0.018, 0.008, (-0.055, -0.089, 0.205), axis="y")
bat_meshes = []
for i, deg in enumerate((0, 60, 120, 180, 240, 300)):
    bat = (
        m.box((0.040, 0.155, 0.004), (0.020, 0.0, 0.0))
        .turned(deg, "y")
        .moved((-0.055, 0.0, 0.205))
    )
    bat_meshes.append((f"bat{i}", bat, "tine"))
# side sheets (header walls), behind and beside the bar, not +x of it
side_l = m.box((0.11, 0.008, 0.12), (-0.045,  0.088, 0.10))
side_r = m.box((0.11, 0.008, 0.12), (-0.045, -0.088, 0.10))
# auger trough and shaft behind the cutter bar
trough = m.box((0.050, 0.150, 0.010), (-0.085, 0.0, 0.055))
trough_l = m.box((0.010, 0.150, 0.035), (-0.105, 0.0, 0.072))
trough_r = m.box((0.010, 0.150, 0.035), (-0.065, 0.0, 0.072))
auger = m.cylinder(0.016, 0.148, (-0.085, 0.0, 0.080), axis="y")
# a few flight disks so it reads as a screw, not a plain tube
flight_meshes = []
for i, fy in enumerate((-0.05, -0.025, 0.0, 0.025, 0.05)):
    fl = m.cylinder(0.028, 0.004, (-0.085, fy, 0.080), axis="y")
    flight_meshes.append((f"fl{i}", fl, "auger"))
# a short snout / divider above the trough, dark green
hood = m.box((0.08, 0.150, 0.008), (-0.070, 0.0, 0.155))

hdr_parts = [
    ("bar", bar, "steel"),
    ("stripe", stripe, "stripe"),
    ("post_l", post_l, "post"),
    ("post_r", post_r, "post"),
    ("brace", brace, "green2"),
    ("side_l", side_l, "green"),
    ("side_r", side_r, "green"),
    ("trough", trough, "steel2"),
    ("trough_l", trough_l, "steel2"),
    ("trough_r", trough_r, "steel2"),
    ("auger", auger, "auger"),
    ("hood", hood, "green2"),
    ("reel", reel, "reel"),
    ("reel_l", reel_end_l, "reel"),
    ("reel_r", reel_end_r, "reel"),
] + guard_meshes + bat_meshes + flight_meshes
hdr_cols = (
    [col(bar), col(post_l), col(post_r), col(brace), col(side_l), col(side_r),
     col(trough), col(auger), col(hood), col(reel)]
    + [col(g[1]) for g in guard_meshes]
    + [col(b[1]) for b in bat_meshes]
    + [col(f[1]) for f in flight_meshes]
)
HEADER = m.fixed("header", (HX, SY, TOP), hdr_parts, hdr_cols, LOOKS, friction=(0.8, 0.6))

# ---- free stand. Origin = mid-base. Saddle floor at z=STAND_H. Handle +x. ----
base = m.box((BASE_W, BASE_W, 0.008), (0.0, 0.0, 0.004))
post = m.box((0.020, 0.020, STAND_H - 0.008), (0.0, 0.0, 0.008 + (STAND_H - 0.008) / 2))
# post cap + a -x fork that tucks under the bar. v1/v2 had a centred
# saddle, so the post itself had to go under the bar and the arm hit it.
cap = m.box((0.024, 0.024, 0.008), (0.0, 0.0, STAND_H - 0.004))
fork = m.box((0.045, 0.044, 0.008), (-0.022, 0.0, STAND_H - 0.004))  # to x=-0.0445
ear_y = 0.019
ear_l = m.box((0.040, 0.006, 0.008), (-0.022,  ear_y, STAND_H + 0.004))
ear_r = m.box((0.040, 0.006, 0.008), (-0.022, -ear_y, STAND_H + 0.004))
# 30 mm in y so a yaw-0 pinch has a clear target
handle = m.box((0.032, 0.030, 0.016), (HANDLE_DX, 0.0, HANDLE_Z))
stand_parts = [
    ("base", base, "yel2"),
    ("post", post, "yel"),
    ("cap", cap, "yel2"),
    ("fork", fork, "yel2"),
    ("ear_l", ear_l, "yel2"),
    ("ear_r", ear_r, "yel2"),
    ("handle", handle, "yel2"),
]
stand_cols = [col(base), col(post), col(cap), col(fork), col(ear_l), col(ear_r), col(handle)]
STAND = m.free(
    "stand",
    (START_X, SY, TOP + START_Z),
    stand_parts,
    stand_cols,
    LOOKS,
    mass=0.40,
    friction=(0.85, 0.65),
    torsional_patch=0.010,
    solver_iters=48,
)

# low-friction pedestal under the START pose only (under the CG, near the post)
pedestal_mesh = m.box((0.050, 0.050, 0.008), (0.0, 0.0, 0.004))
# 10 mm gap: start base at 0.390, seat base extends to 0.28+0.035=0.315, pedestal
# at START_X extends to 0.390+0.025=0.415, no overlap with the seat.
PEDESTAL = m.fixed(
    "pedestal",
    (START_X, SY, TOP),
    [("block", pedestal_mesh, "wood")],
    [col(pedestal_mesh)],
    LOOKS,
    friction=(0.12, 0.08),
)
OBJECTS = [HEADER, PEDESTAL, STAND]


def _clear(mem):
    mem.update(seated=False, near=False, rest=0, tilt=99, dx=9, dy=9, dz=9, clear=False, vx=9)


TRIAL = hhkit.Trial(on_clear=_clear)


@m.on_reset
def _reset(env):
    print(
        f"[E] reset headerstand v3vis start x={START_X:.3f} seat x={SX:.3f} "
        f"y={SY} bar_z={BAR_Z} stand_h={STAND_H} variant={V}",
        flush=True,
    )


@m.on_step
def _log(env):
    if not TRIAL.once():
        return
    st = m.state(env, "stand")
    p = np.asarray(st.pos, float)
    r = hhkit.rot(st.quat_xyzw)
    up = r[:, 2]
    up_err = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    dx, dy, dz = p[0] - SX, p[1] - SY, p[2] - (TOP + SEAT_Z)
    vel = float(np.linalg.norm(st.lin_vel))
    seated = (
        abs(dx) < 0.015
        and abs(dy) < 0.020
        and abs(dz) < 0.010
        and up_err < 12.0
    )
    near = (
        abs(dy) < 0.030
        and abs(dz) < 0.015
        and 0.020 <= abs(dx) <= 0.055
        and dx > 0
        and up_err < 18.0
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
        dmin, tcp_h, clear = 9.0, 0.0, False
    rest_ok = vel < 0.03 and seated and clear
    TRIAL.mem["rest"] = TRIAL.mem.get("rest", 0) + 1 if rest_ok else 0
    TRIAL.mem.update(
        seated=seated, near=near,
        tilt=round(up_err, 1),
        dx=round(dx, 4), dy=round(dy, 4), dz=round(dz, 4),
        vx=round(vel, 4), clear=clear, dmin=round(dmin, 3),
        x=round(p[0], 4), y=round(p[1], 4), z=round(p[2] - TOP, 4),
        tcp_h=round(tcp_h, 3),
    )
    n = TRIAL.step
    if n in (1, 2, 5) or n % 15 == 0 or seated or near:
        g0 = lefts[0]["tcp"] if lefts else (0, 0, 0)
        print(
            f"[E] n {n} st ({p[0]:.3f},{p[1]:.3f},{(p[2]-TOP)*1000:.1f}mm) "
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
    return float(abs(mm.get("dx", 9.0)) + abs(mm.get("dy", 9.0)))


REST_R = (0.20, -0.30, TOP + 0.18)
REST_L = (0.20, 0.30, TOP + 0.18)
YAW = 0.0
# Pinch the +x handle. Lift only ~12 mm so the saddle stays under the bar.
PX_START = START_X + HANDLE_DX          # 0.418
PX_SEAT = SX + HANDLE_DX                # 0.348
PY = SY
ZG_PINCH_START = TOP + START_Z + HANDLE_Z + PAD   # h ≈ 0.086
ZG_PINCH_SEAT = TOP + SEAT_Z + HANDLE_Z + PAD     # h ≈ 0.078
LIFT = 0.012
ZG_LIFT = ZG_PINCH_START + LIFT                   # h ≈ 0.098
ZG_HIGH = TOP + 0.155


def plan_demo():
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the handle, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the handle", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the handle", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the pedestal", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (PX_SEAT, PY, ZG_LIFT), FINGER_CLOSED, "over the seat, saddle under the bar", yaw=YAW),
            Waypoint(1.4, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "lower the stand under the bar", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "hold seated", yaw=YAW),
            Waypoint(0.6, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_OPEN, "release", yaw=YAW),
            Waypoint(1.0, (PX_SEAT, PY, ZG_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def plan_naive():
    park = (PX_START + 0.015, PY, ZG_PINCH_START)
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the handle, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the handle", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the handle", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.6, (park[0], park[1], ZG_LIFT), FINGER_CLOSED, "carry it the wrong way, away from the bar", yaw=YAW),
            Waypoint(1.0, park, FINGER_CLOSED, "set it down short of the bar", yaw=YAW),
            Waypoint(0.6, park, FINGER_OPEN, "release", yaw=YAW),
            Waypoint(0.8, (park[0], park[1], ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def plan_short():
    # 50 mm short of the seat so a drop cannot luck into 1.0
    xs = PX_SEAT + 0.050
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the handle, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the handle", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the handle", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the pedestal", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (xs, PY, ZG_LIFT), FINGER_CLOSED, "over a point 50 mm short of the bar, do not lower", yaw=YAW),
            Waypoint(1.0, (xs, PY, ZG_LIFT), FINGER_CLOSED, "hold over the short pose", yaw=YAW),
            Waypoint(0.6, (xs, PY, ZG_LIFT), FINGER_OPEN, "release above, short of the bar", yaw=YAW),
            Waypoint(1.0, (xs, PY, ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
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
