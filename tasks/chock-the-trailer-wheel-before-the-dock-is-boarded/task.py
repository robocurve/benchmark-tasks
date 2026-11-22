"""PROBE v1 (stream E): place a yellow wheel chock against a trailer tire
so the wedge sits on the table and its slope faces the tread.

Straight-down pinch-and-place. No sleeve, no hook, no pins.
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
    "The wheel chock is out. Set the chock against the tire so it sits on the ground and the slope faces the tread."
)

V = hhkit.demo_variant("demo")
PAD = 0.013

# Tire: axis along +y, light-trailer size. +x face is the contact side.
TX, SY, TR = 0.18, 0.22, 0.10
TIRE_W = 0.080          # section width along y
# Chock: length along x, width along y, height z. Tip at local -x.
CHK_L, CHK_W, CHK_H = 0.100, 0.070, 0.100
# Seat: origin mid-bottom; tip at x = SX - CHK_L/2 should meet the +x tire face
# TX + TR, minus a 5 mm tuck.
TUCK = 0.000
SX = (TX + TR - TUCK) + CHK_L / 2     # 0.330
START_DX = 0.080
START_X = SX + START_DX               # 0.410
SEAT_Z = 0.000
START_Z = 0.008  # on the pedestal; v1b spawned at z=0 and popped to 13 mm / 15°

LOOKS = {
    "yel": m.Look(m.rgb(0.95, 0.78, 0.08), roughness=0.45),
    "yel2": m.Look(m.rgb(0.88, 0.55, 0.08), roughness=0.5),
    "rubber": m.Look(m.rgb(0.12, 0.12, 0.13), roughness=0.85),
    "tread": m.Look(m.rgb(0.18, 0.18, 0.19), roughness=0.9),
    "rim": m.Look(m.rgb(0.82, 0.82, 0.84), metallic=0.7, roughness=0.3),
    "hub": m.Look(m.rgb(0.45, 0.46, 0.48), metallic=0.6, roughness=0.4),
    "wood": m.Look(m.rgb(0.45, 0.32, 0.18), roughness=0.8),
    "panel": m.Look(m.rgb(0.82, 0.18, 0.14), roughness=0.45),
    "panel2": m.Look(m.rgb(0.88, 0.88, 0.86), roughness=0.5),
    "fender": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "frame": m.Look(m.rgb(0.28, 0.28, 0.30), metallic=0.5, roughness=0.45),
    "concrete": m.Look(m.rgb(0.55, 0.54, 0.50), roughness=0.9),
    "bumper": m.Look(m.rgb(0.95, 0.78, 0.08), roughness=0.5),
    "bumper2": m.Look(m.rgb(0.10, 0.10, 0.10), roughness=0.6),
}

_cn = [0]


def col(mesh):
    _cn[0] += 1
    return m.col_hull(f"c{_cn[0]}", mesh)


# ---- fixed tire ----
tire = m.cylinder(TR, TIRE_W, (0.0, 0.0, 0.0), axis="y")
# a slightly smaller ring as a rim lip, plus a hub
rim = m.cylinder(TR - 0.018, TIRE_W + 0.006, (0.0, 0.0, 0.0), axis="y")
hub = m.cylinder(0.028, TIRE_W + 0.010, (0.0, 0.0, 0.0), axis="y")
# shallow tread grooves: thin darker rings, visual only (same collider as tire)
fx_parts = [
    ("tire", tire, "rubber"),
    ("rim", rim, "rim"),
    ("hub", hub, "hub"),
]
fx_cols = [col(tire), col(rim)]
# tire sits on the table: centre at z = TR
TIRE = m.fixed("tire", (TX, SY, TOP + TR), fx_parts, fx_cols, LOOKS, friction=(1.2, 1.0))

# ---- trailer bed and dock lip (v3d). Kept off the +x chock path and
# off the left-arm workspace at y=0.22. Collision is on so a policy
# cannot pass through them; they are not scored.
# Trailer frame is the tire's: origin at (TX, SY, TOP), tire centre at (0,0,TR).
bed = m.box((0.22, 0.12, 0.070), (-0.04, -0.16, 0.055))
side = m.box((0.22, 0.008, 0.090), (-0.04, -0.098, 0.135))
rail = m.box((0.20, 0.020, 0.020), (-0.04, -0.10, 0.018))
# fender over the -y half of the tire, clear of the +x contact patch at x=+TR
fender = m.box((0.12, 0.040, 0.030), (-0.02, -0.070, 0.195))
flap = m.box((0.008, 0.055, 0.080), (-0.12, -0.030, 0.060))
# a second "side" so the cut-off bed reads as a box, not a plank
tail = m.box((0.010, 0.12, 0.090), (-0.155, -0.16, 0.135))
tr_parts = [
    ("bed", bed, "panel"),
    ("side", side, "panel2"),
    ("rail", rail, "frame"),
    ("fender", fender, "fender"),
    ("flap", flap, "fender"),
    ("tail", tail, "panel"),
]
tr_cols = [col(bed), col(side), col(rail), col(fender), col(flap), col(tail)]
TRAILER = m.fixed("trailer", (TX, SY, TOP), tr_parts, tr_cols, LOOKS, friction=(0.8, 0.6))

# dock lip near the -x table edge, on the -y side of the work (not under the arm)
dock_wall = m.box((0.050, 0.22, 0.090), (0.0, 0.0, 0.045))
dock_top = m.box((0.090, 0.24, 0.018), (0.015, 0.0, 0.099))
bump = m.box((0.016, 0.20, 0.030), (0.028, 0.0, 0.070))
bump_stripe = m.box((0.017, 0.040, 0.031), (0.028, 0.0, 0.070))
dk_parts = [
    ("wall", dock_wall, "concrete"),
    ("top", dock_top, "concrete"),
    ("bump", bump, "bumper"),
    ("stripe", bump_stripe, "bumper2"),
]
dk_cols = [col(dock_wall), col(dock_top), col(bump)]
DOCK = m.fixed("dock", (0.055, 0.06, TOP), dk_parts, dk_cols, LOOKS, friction=(0.9, 0.7))

# ---- free wedge. Origin = mid-bottom, tip at local x = -CHK_L/2 ----
# prism in xy: tip (0,0), back-bottom (L,0), back-top (L,H), extrude ±W/2 in z.
# turned 90° about x: (x,y,z)->(x,-z,y) so x=length, y=width, z=height.
wedge = (
    m.prism([(0.0, 0.0), (CHK_L, 0.0), (CHK_L, CHK_H)], -CHK_W / 2, CHK_W / 2)
    .turned(90, "x")
    .moved((-CHK_L / 2, 0.0, 0.0))
)
# a slightly darker back face plate so the fat end reads at 320×176
back = m.box((0.010, CHK_W + 0.004, CHK_H), (CHK_L / 2 - 0.005, 0.0, CHK_H / 2))
# top grab bar, 30 mm in y so a yaw-0 pinch has a clear target (v2 close
# on the 70 mm body never touched the level chock). Many real chocks have
# a handle or rope hole; flagged in RISK.
handle = m.box((0.040, 0.030, 0.016), (CHK_L / 6, 0.0, CHK_H + 0.008))
chock_parts = [("wedge", wedge, "yel"), ("back", back, "yel2"), ("handle", handle, "yel2")]
chock_cols = [col(wedge), col(back), col(handle)]
CHOCK = m.free(
    "chock",
    (START_X, SY, TOP + START_Z),
    chock_parts,
    chock_cols,
    LOOKS,
    mass=0.35,
    friction=(0.85, 0.65),
    torsional_patch=0.010,
    solver_iters=48,
)

# low-friction pedestal under the START pose only
pedestal_mesh = m.box((0.050, 0.050, 0.008), (0.0, 0.0, 0.004))
PEDESTAL = m.fixed(
    "pedestal",
    (START_X + CHK_L / 6, SY, TOP),  # under the wedge CG, not the geometric centre
    [("block", pedestal_mesh, "wood")],
    [col(pedestal_mesh)],
    LOOKS,
    friction=(0.12, 0.08),
)
OBJECTS = [TIRE, TRAILER, DOCK, PEDESTAL, CHOCK]


def _clear(mem):
    mem.update(seated=False, near=False, rest=0, tilt=99, dx=9, dy=9, dz=9, clear=False, vx=9, gap=9)


TRIAL = hhkit.Trial(on_clear=_clear)


@m.on_reset
def _reset(env):
    print(
        f"[E] reset wheelchock v3d start x={START_X:.3f} seat x={SX:.3f} "
        f"y={SY} tire_x={TX} tr={TR} variant={V}",
        flush=True,
    )


@m.on_step
def _log(env):
    if not TRIAL.once():
        return
    st = m.state(env, "chock")
    p = np.asarray(st.pos, float)
    r = hhkit.rot(st.quat_xyzw)
    # local +z should stay world up; local -x is the tip, should point toward the tire (-x)
    up = r[:, 2]
    tip_dir = -r[:, 0]
    up_err = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    # tip world x: origin + R * (-CHK_L/2, 0, 0)
    tip = p + r @ np.array([-CHK_L / 2, 0.0, 0.0])
    tire_face_x = TX + TR
    gap = float(tip[0] - tire_face_x)  # >0 means the tip has not reached the tire
    dx, dy, dz = p[0] - SX, p[1] - SY, p[2] - (TOP + SEAT_Z)
    vel = float(np.linalg.norm(st.lin_vel))
    tip_points_at_tire = float(tip_dir[0]) < -0.7  # mostly −x
    seated = (
        abs(dy) < 0.020
        and abs(dz) < 0.010
        and -0.010 <= gap <= 0.018
        and up_err < 12.0
        and tip_points_at_tire
    )
    near = (
        abs(dy) < 0.030
        and abs(dz) < 0.015
        and 0.018 < gap <= 0.050
        and up_err < 18.0
        and tip_points_at_tire
        and not seated
    )
    grips = hhkit.grippers(env)
    # Only the left (acting) arm. v3b used max height over both grippers, so
    # the parked right arm at h=0.18 made "clear" true while the left still
    # held the handle (dmin to the origin is already ~12 cm at the top bar).
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
        gap=round(gap, 4),
        vx=round(vel, 4), clear=clear, dmin=round(dmin, 3),
        x=round(p[0], 4), y=round(p[1], 4), z=round(p[2] - TOP, 4),
        tipx=round(tip[0], 4),
    )
    n = TRIAL.step
    if n in (1, 2, 5) or n % 15 == 0 or seated or near:
        g0 = grips[0]["tcp"] if grips else (0, 0, 0)
        print(
            f"[E] n {n} ck ({p[0]:.3f},{p[1]:.3f},{(p[2]-TOP)*1000:.1f}mm) "
            f"dxyz ({dx*1000:.1f},{dy*1000:.1f},{dz*1000:.1f}) gap {gap*1000:.1f} "
            f"up {up_err:.1f} tip={int(tip_points_at_tire)} near={near} seat={seated} "
            f"rest={TRIAL.mem['rest']} clear={int(clear)} tcpL ({g0[0]:.3f},{g0[1]:.3f},{g0[2]-TOP:.3f})",
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
    return float(abs(mm.get("gap", 9.0)) + abs(mm.get("dy", 9.0)))


REST_R = (0.20, -0.30, TOP + 0.18)
REST_L = (0.20, 0.30, TOP + 0.18)
YAW = 0.0
# Pinch mid-to-upper body (h≈0.09): 70 mm width, yaw 0. Comfortable cell.
PX_START = START_X
PX_SEAT = SX + 0.008                  # 8 mm short; v1b overshot 28 mm after a tilted lower
PY = SY
HANDLE_Z = CHK_H + 0.008                    # handle centre above the wedge top
ZG_PINCH_START = TOP + START_Z + HANDLE_Z + PAD  # h ≈ 0.129
ZG_PINCH_SEAT = TOP + SEAT_Z + HANDLE_Z + PAD    # h ≈ 0.121 on the table
ZG_LIFT = TOP + 0.150
ZG_HIGH = TOP + 0.165


def plan_demo():
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the chock, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the chock", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the chock", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the pedestal", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (PX_SEAT, PY, ZG_LIFT), FINGER_CLOSED, "over the seat", yaw=YAW),
            Waypoint(1.4, (PX_SEAT, PY, ZG_PINCH_SEAT), FINGER_CLOSED, "lower the chock against the tire", yaw=YAW),
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
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the chock, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the chock", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the chock", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.6, (park[0], park[1], ZG_LIFT), FINGER_CLOSED, "carry it the wrong way, away from the tire", yaw=YAW),
            Waypoint(1.0, park, FINGER_CLOSED, "set it down short of the tire", yaw=YAW),
            Waypoint(0.6, park, FINGER_OPEN, "release", yaw=YAW),
            Waypoint(0.8, (park[0], park[1], ZG_HIGH), FINGER_OPEN, "up", yaw=YAW),
            Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
        ],
        "right": [Waypoint(12.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)],
    }


def plan_short():
    return {
        "left": [
            Waypoint(2.0, (PX_START, PY, ZG_HIGH), FINGER_OPEN, "high over the chock, fingers open", yaw=YAW),
            Waypoint(1.2, (PX_START, PY, ZG_PINCH_START + 0.010), FINGER_OPEN, "down, still open", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_PINCH_START), FINGER_OPEN, "around the chock", yaw=YAW),
            Waypoint(0.6, (PX_START, PY, ZG_PINCH_START), FINGER_CLOSED, "pinch the chock", yaw=YAW),
            Waypoint(0.8, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "lift off the pedestal", yaw=YAW),
            Waypoint(0.5, (PX_START, PY, ZG_LIFT), FINGER_CLOSED, "hold the lift", yaw=YAW),
            Waypoint(1.8, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_CLOSED, "over a point 50 mm short of the tire, do not lower", yaw=YAW),
            Waypoint(1.0, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_CLOSED, "hold over the short pose", yaw=YAW),
            Waypoint(0.6, (PX_SEAT + 0.050, PY, ZG_LIFT), FINGER_OPEN, "release above, short of the tire", yaw=YAW),
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
