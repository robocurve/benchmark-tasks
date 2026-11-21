# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Re-centre a hung cabinet flap door in its opening with the cam-adjust hinge's side screw, then close it.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; z = 0 below is the table top (0.7475).

- A run of three cabinet units stands on a 100 mm plinth, front plane at x = 0.20. The middle unit has a drop-down
  flap door (297 wide, 270 high, 18 mm thick, 1.05 kg) hinged along its bottom edge on a cup-hinge arm; open, it lies
  flat in front of the unit, flush with the bottom board. The flaps of the two neighbouring units are closed and
  fixed. The nominal reveal is 4.5 mm each side.
- The hinge arm carries three slotted heads on its mounting tab, in a row from the front: the clamp screw (loosening
  it lets the arm slip and the door sag), the side cam (an eccentric, 3.0 mm throw: door sideways) and the depth cam
  (2.0 mm throw: door in and out). Nothing marks which is which.
- The flap starts open, its side cam turned 39 deg off neutral, so it sits 1.9 mm toward its left neighbour:
  reveals 6.4 mm on the right and 2.6 mm on the left.
- A 75 mm stubby slotted screwdriver (140 g) stands in a holder rack on the table to the left.

Goal: the flap closed, the reveals even and the face flush, the grippers let go.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import linkkit as lk  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
VARIANT = os.environ.get("DEMO_VARIANT", "")
MM = 0.001

# -- cabinet geometry (m; the cabinet linkage frame is the world frame shifted down to the table top) ----------------
F = float(os.environ.get("CAB_F", 0.20))      # front plane (door inner face)
BOARD = 0.018
DW, DH = 0.297, 0.270        # door width (along y) and height
HH = 0.100                   # hinge axis / bottom board underside, above the table (a 100 mm plinth)
GAP0 = 0.0045                # nominal reveal each side
PIVOT = (F - BOARD, HH)      # hinge axis (along y): the door's outer bottom corner
DOOR_C = (F - BOARD / 2, 0.0, HH + DH / 2)   # door centre when closed
HEAD_Z = HH + BOARD + 0.006 + 0.0025         # head centre height (tab top 0.074, heads 5 mm)
HEAD_Y = 0.070
HEAD_X = {"hc": F + 0.020, "hs": F + 0.032, "hd": F + 0.044}   # clamp, side cam, depth cam
A_SIDE, A_DEPTH = 3.0 * MM, 2.0 * MM   # eccentric throws
SIDE0_DEG = 39.3             # starting side-cam angle (door 1.9 mm toward the left neighbour)
CLAMP_LATCH_DEG = 25.0       # clamp loosened this far: the arm slips 2.5 mm down and stays there
SAG_SLIP = 2.5 * MM
K_SLIDE = 2.0e4
DOOR_MASS = 1.05
SAG_COMP = (DOOR_MASS + 0.1) * 9.81 / K_SLIDE
STAY_K = 0.7                 # N m/rad: the lift stay's spring takes most of the flap's weight (gravity 1.4 N m at 90 deg)
CATCH_DEG = 50.0             # the self-closing (soft-close) hinge acts over the last 50 deg
GREY = (0.62, 0.64, 0.68)
OAK = (0.80, 0.74, 0.60)
ZINC = (0.78, 0.78, 0.80)
STEEL = (0.55, 0.57, 0.60)
RED = (0.75, 0.15, 0.12)

# -- thresholds ------------------------------------------------------------------------------------------------------
CLOSED_DEG = 3.0
EVEN_MM = 1.0                # |reveal difference|
BAND_MM = (3.0, 6.0)         # each reveal
FLUSH_MM = 0.6
SAG_GATE_MM = 1.2
IMPROVE_DEG = 10.0
STEADY_S = 1.0
RELEASE_S = 1.0
CLEAR_M = 0.030
TCP_STILL = 0.005
DOOR_STILL = 0.003
EPS = 1e-6


def _b(size, at):
    return lk.box(size, at)


_t = lk.Tree("cab", pos=(0.0, 0.0, TOP), fixed=True, color=OAK, self_collision=False)
_cx = F + 0.15
_zw = (HH + 0.018 + HH + DH + 0.018) / 2        # side wall centre height
_hw = DH + 0.018 + 0.018                        # side wall height
YEXT = 0.235                  # the run is modelled out to this half-width (the READY poses of the arms stay clear)
_t.link("cab", [
    _b((0.30, 2 * YEXT, BOARD), (_cx, 0.0, HH + BOARD / 2)),                 # bottom board
    _b((0.06, 2 * YEXT, BOARD), (F + 0.24, 0.0, HH + DH + BOARD / 2)),       # back stretcher (no top board: open-topped base unit)
    _b((BOARD, 2 * YEXT, DH + BOARD), (F + 0.291, 0.0, HH + (DH + BOARD) / 2)),   # back
    _b((0.30, BOARD, DH + BOARD), (_cx, 0.162, HH + (DH + BOARD) / 2)),   # dividers
    _b((0.30, BOARD, DH + BOARD), (_cx, -0.162, HH + (DH + BOARD) / 2)),
    _b((0.30, BOARD, DH + BOARD), (_cx, YEXT + BOARD / 2, HH + (DH + BOARD) / 2)),   # outer walls
    _b((0.30, BOARD, DH + BOARD), (_cx, -YEXT - BOARD / 2, HH + (DH + BOARD) / 2)),
    _b((0.30, 2 * YEXT, 0.050 - 0.004), (_cx, 0.0, (HH - 0.004) / 2)),       # plinth block
    lk.box((BOARD, YEXT - 0.153, DH), (F - BOARD / 2, (0.153 + YEXT) / 2, HH + DH / 2), color=GREY),    # left neighbour flap
    lk.box((BOARD, YEXT - 0.153, DH), (F - BOARD / 2, -(0.153 + YEXT) / 2, HH + DH / 2), color=GREY),   # right neighbour flap
    lk.box((0.040, 0.060, 0.006), (F + 0.032, HEAD_Y, HH + BOARD + 0.003), color=STEEL),         # hinge mounting tab
], mass=30.0, friction=(0.6, 0.5), contact_offset=0.001)


def _head(name, x):
    """A 7 x 7.8 x 5 mm slotted cam head: a 1.8 mm slot, 2.5 mm deep below a chamfered mouth (3.8 mm wide at the top)."""
    sh = []
    for sgn in (1, -1):
        sh.append(lk.box((0.007, 0.0028, 0.003), (0, sgn * 0.0023, -0.001)))                  # lower walls
        sh.append(lk.box((0.007, 0.0008, 0.00224), (0, sgn * 0.001758, 0.001321), rpy_deg=(-sgn * 26.57, 0, 0)))  # chamfer
    _t.link(name, sh, mass=0.005, at=(x, HEAD_Y, HEAD_Z), color=ZINC, friction=(0.5, 0.4), contact_offset=0.0003)


for _n, _x in HEAD_X.items():
    _head(_n, _x)
_AN = (F + 0.03, HEAD_Y, HH + 0.010)
_t.link("s1", [lk.box((0.004, 0.004, 0.004))], mass=0.05, at=_AN, color=STEEL)
_t.link("s2", [lk.box((0.004, 0.004, 0.004))], mass=0.05, at=_AN, color=STEEL)
_t.link("arm", [lk.box((0.064, 0.039, 0.010))], mass=0.05, at=(F + 0.020, HEAD_Y, HH + 0.010), color=STEEL,
        contact_offset=0.001)
_t.link("door", [
    lk.box((BOARD, DW, DH), (0, 0, 0)),
    lk.box((0.012, 0.035, 0.035), (0.015, HEAD_Y, -DH / 2 + 0.045), color=STEEL),   # hinge cup boss
], mass=DOOR_MASS, at=DOOR_C, color=GREY, friction=(0.5, 0.4), contact_offset=0.001)

for _n, _lim, _init in (("scr_c", (0, 360), 0.0), ("scr_s", (-90, 90), SIDE0_DEG), ("scr_d", (-90, 90), 0.0)):
    _hn = {"scr_c": "hc", "scr_s": "hs", "scr_d": "hd"}[_n]
    _t.revolute(_n, "cab", _hn, anchor=(HEAD_X[_hn], HEAD_Y, HEAD_Z), axis=(0, 0, 1), limits_deg=_lim, init_deg=_init,
                damping=0.002, friction=0.004)
D0 = A_SIDE * math.sin(math.radians(SIDE0_DEG))
_t.prismatic("sy", "cab", "s1", anchor=_AN, axis=(0, 1, 0), limits=(-0.01, 0.01), rest=D0, init=D0,
             stiffness=K_SLIDE, damping=200.0)
_t.prismatic("sx", "s1", "s2", anchor=_AN, axis=(-1, 0, 0), limits=(-0.01, 0.01), rest=0.0, init=0.0,
             stiffness=K_SLIDE, damping=200.0)
_t.prismatic("sz", "s2", "arm", anchor=_AN, axis=(0, 0, 1), limits=(-0.01, 0.01), rest=SAG_COMP,
             init=SAG_COMP, stiffness=K_SLIDE, damping=200.0)
_t.revolute("hinge", "arm", "door", anchor=(PIVOT[0], 0.0, PIVOT[1]), axis=(0, -1, 0), limits_deg=(0, 90),
            init_deg=90.0, damping=0.3, friction=0.1)
CAB = _t.build()

# -- the screwdriver: frame at the blade tip, +z up the axis. 45 mm handle, 22 mm shaft, 8 mm blade ---------------------
DRV_X, DRV_Y = float(os.environ.get("RK_X", 0.07)), float(os.environ.get("RK_Y", 0.21))
RACK_TOP = 0.0425
HANDLE_BOTTOM = 0.030
_t = lk.Tree("driver", pos=(DRV_X, DRV_Y, TOP + RACK_TOP - HANDLE_BOTTOM + 0.0005), color=RED)
_t.link("drv", [lk.box((0.005, 0.0009, 0.008), (0, 0, 0.004), color=STEEL),
                lk.cylinder(0.0025, 0.022, (0, 0, 0.019), color=STEEL),
                lk.cylinder(0.011, 0.045, (0, 0, HANDLE_BOTTOM + 0.0225))],
        mass=0.14, friction=(0.8, 0.7), contact_offset=0.0005)
DRIVER = _t.build()
TIP_HANDLE = 0.055   # grasp height above the tip: the handle's top 20 mm sits between the pads

# -- holder rack (fixed): a plate with a 16 mm square hole; the driver hangs by its handle shoulder -------------------
_t = lk.Tree("rack", pos=(DRV_X, DRV_Y, TOP), fixed=True, color=STEEL)
_rk = [lk.box((0.080, 0.032, 0.006), (0, s * 0.024, RACK_TOP - 0.003)) for s in (-1, 1)]
_rk += [lk.box((0.032, 0.016, 0.006), (s * 0.024, 0, RACK_TOP - 0.003)) for s in (-1, 1)]
_rk += [lk.box((0.008, 0.008, RACK_TOP - 0.006), (sx * 0.036, sy * 0.036, (RACK_TOP - 0.006) / 2))
        for sx in (-1, 1) for sy in (-1, 1)]
_t.link("rack", _rk, mass=1.0, friction=(0.6, 0.5), contact_offset=0.001)
RACK = _t.build()

OBJECTS = [CAB, DRIVER, RACK]


# -- measurement (pure functions on link states) ----------------------------------------------------------------------
def yaw_deg(M) -> float:
    return math.degrees(math.atan2(M[1, 0], M[0, 0]))


def measure(cab, driver, tools) -> dict:
    """One control-step sample. Poses are what a camera sees; angles of the screw heads come from their link poses."""
    D = cab.poses["door"]
    psi = math.degrees(math.atan2(-D[0, 2], D[2, 2]))      # opening angle from the door's own axes
    c = D[:3, 3] + D[:3, :3] @ np.array([-BOARD / 2, 0.0, -DH / 2])   # the hinge line's point, valid at any angle
    m = {
        "psi": psi,
        "gR": 1000 * (float(c[1]) - DW / 2 + 0.153),
        "gL": 1000 * (0.153 - float(c[1]) - DW / 2),
        "proud": 1000 * ((F - BOARD) - float(c[0])),
        "sag": 1000 * (TOP + HH - float(c[2])),
        "door_speed": float(np.linalg.norm(cab.lin_vels["door"])),
        "th": {n: yaw_deg(cab.poses[n]) for n in ("hs", "hd", "hc")},
    }
    tip = driver.poses["drv"][:3, 3]
    m["engaged"] = any(
        math.hypot(tip[0] - cab.poses[n][0, 3], tip[1] - cab.poses[n][1, 3]) < 0.006 and tip[2] < cab.poses[n][2, 3] + 0.002
        for n in ("hs", "hd", "hc"))
    m["clear"] = {s: min(lk.clearance(tl, cab, ["door"]), lk.clearance(tl, driver)) for s, tl in tools.items()}
    m["tcp_speed"] = {s: tl.speed for s, tl in tools.items()}
    return m


def closed(m) -> bool:
    return abs(m["psi"]) <= CLOSED_DEG


def even(m) -> bool:
    return (closed(m) and abs(m["gR"] - m["gL"]) <= EVEN_MM + EPS
            and BAND_MM[0] - EPS <= min(m["gR"], m["gL"]) and max(m["gR"], m["gL"]) <= BAND_MM[1] + EPS)


def flush(m) -> bool:
    return abs(m["proud"]) <= FLUSH_MM + EPS


class Record:
    """Whole-run record: a linkkit History fed every control step and cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
        self.n = 0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.ever("slipped", m["sag"] >= SAG_GATE_MM)
        H.peak("sag_max", m["sag"])
        H.ever("even_closed", even(m))
        H.held("steady", closed(m) and m["door_speed"] < DOOR_STILL)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL and not m["engaged"])


def ladder(rec: Record) -> tuple[float, str]:
    """Gate first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "slipped" in H.evers:
        return 0.0, f"gate: the door sagged {H.peaks['sag_max']:.1f} mm (clamp screw loosened) at {H.evers['slipped']:.2f} s"
    if even(m) and flush(m) and H.runs.get("steady", 0.0) >= STEADY_S - EPS:
        if H.runs.get("released", 0.0) >= RELEASE_S - EPS:
            return 1.0, "door closed, reveals even, face flush, both grippers let go"
        return 0.75, "door closed, even and flush; grippers not yet 30 mm clear and still for 1 s"
    if "even_closed" in H.evers:
        return 0.5, "the door was closed with even reveals at some time, but not at the end"
    if abs(m["th"]["hs"]) <= SIDE0_DEG - IMPROVE_DEG:
        return 0.25, "the side cam is turned toward neutral, but the door never closed with even reveals"
    return 0.0, "side cam not turned toward neutral"


REC = Record()
_LIVE: dict = {}


def _drive(obj, joint, value):
    if abs(_LIVE.get(joint, 1e9) - value) > 1e-7:
        _LIVE[joint] = value
        lk.set_rest(obj, joint, value)


@lk.on_reset
def _reset(tick):
    REC.reset()
    _LIVE.clear()
    _drive("cab", "sy", D0)
    _drive("cab", "sx", 0.0)
    _drive("cab", "sz", SAG_COMP)
    _LIVE["door_open"] = True
    lk.set_spring("cab", "hinge", STAY_K, 0.3)
    lk.set_friction("cab", "hinge", 0.1, 0.1)


@lk.on_step
def _track(tick):
    s = tick.states["cab"]
    th_s, th_d, th_c = s.q["scr_s"], s.q["scr_d"], s.q["scr_c"]
    if th_c > math.radians(CLAMP_LATCH_DEG):
        _LIVE["slip"] = True
    _drive("cab", "sy", A_SIDE * math.sin(th_s))
    _drive("cab", "sx", A_DEPTH * math.sin(th_d))
    _drive("cab", "sz", SAG_COMP - (SAG_SLIP if _LIVE.get("slip") else 0.0))
    is_open = s.q["hinge"] > math.radians(CATCH_DEG)
    if is_open != _LIVE.get("door_open"):   # the self-closing catch acts only over the last 15 deg
        _LIVE["door_open"] = is_open
        if is_open:
            lk.set_spring("cab", "hinge", STAY_K, 0.3)
            lk.set_friction("cab", "hinge", 0.1, 0.1)
        else:
            lk.set_spring("cab", "hinge", 3.0, 1.0)
            lk.set_friction("cab", "hinge", 0.01, 0.01)
    m = measure(s, tick.states["driver"], tick.tools)
    REC.update(m)
    if REC.n % 10 == 0:
        REC.trace.append((round(tick.t, 2), round(m["psi"], 1), round(m["gR"], 2), round(m["gL"], 2),
                          round(m["proud"], 2), round(m["sag"], 2), {k: round(v, 1) for k, v in m["th"].items()},
                          int(m["engaged"]), [round(1000 * m["clear"][k]) for k in ("left", "right")],
                          np.round(tick.tools["left"].tcp, 3).tolist(), round(1000 * tick.tools["left"].opening),
                          [round(1000 * m["tcp_speed"][k], 1) for k in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo (linkdemo.py runs demo_script; it reads only what a camera could measure) ----------------------------
VARIANTS = ("", "nothing", "depth_screw", "clamp_first", "wrong_way", "overturn")
LOG: dict = {}
CLEAR_Z = 0.128          # blade-tip height for carrying over the open flap (its top face is at 0.118)
GRIP_W = 0.0180          # pads on the 22 mm handle (4 mm squeeze, about 10 N a pad)
SLOT_Z = HEAD_Z          # blade tip height in the slot (slot floor 2.5 mm below, head top 2.5 mm above)
CLOSE_TO_DEG = 34.0      # the arc runs to here; the pinch lets go once the door is inside the self-closing range


def _rz(deg):
    from linkdemo import rot_about

    return rot_about((0, 0, 1), deg)


class Driver:
    """Hold the driver; command the TCP from the seen driver pose (grasp relation re-estimated after each settle)."""

    def __init__(self, ctx):
        self.ctx, self.arm = ctx, ctx.left
        self.G = None
        self.T = None
        self.off = np.zeros(3)

    def seen(self):
        return self.ctx.observe().pose("driver", "drv")

    def command(self, D, duration, width=None):
        T = D @ self.G
        self.T = T
        yield from self.arm.move(T[:3, 3] - self.off, T[:3, :3], width=width, duration=duration)

    def resettle(self, wait=0.5):
        """Wait until the gripper is still, then re-estimate the grasp from the measured gripper pose and the seen
        driver pose (the arm lags and sags a little; the driver may creep in the pads)."""
        yield from self.arm.wait(wait)
        for _ in range(20):
            tl = self.ctx.observe().tools["left"]
            if tl.speed < 0.004:
                break
            yield from self.arm.wait(0.1)
        LOG.setdefault("track_mm", []).append(round(1000 * float(np.linalg.norm(tl.tcp - self.arm.pos)), 1))
        D = self.seen()
        Tc = np.eye(4)
        Tc[:3, :3], Tc[:3, 3] = tl.rot, tl.tcp
        self.G = np.linalg.inv(D) @ Tc
        self.off = tl.tcp - self.arm.pos      # the arm's steady-state sag, fed back into the next command

    def pose(self, tip, yaw):
        D = np.eye(4)
        D[:3, :3] = _rz(yaw)
        D[:3, 3] = tip
        return D


def pick(ctx, dv):
    from linkdemo import OPEN_WIDTH, down

    D0 = dv.seen()
    P = D0[:3, :3] @ np.array([0.0, 0.0, TIP_HANDLE]) + D0[:3, 3]
    R = down(0.0)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, P
    dv.G = np.linalg.inv(D0) @ T
    yield from dv.arm.move(np.array([0.10, 0.32, TOP + 0.19]), R, width=OPEN_WIDTH, duration=1.5)  # round the cabinet's wall
    yield from dv.arm.move(P + [0, 0, 0.06], R, width=OPEN_WIDTH, duration=1.3)
    yield from dv.arm.move(P, R, duration=1.2)
    yield from dv.arm.wait(0.7)
    tl = ctx.observe().tools["left"]
    LOG["pre_grip_err_mm"] = np.round(1000 * (tl.tcp - P), 1).tolist()
    yield from dv.arm.grip(GRIP_W, 0.7)
    yield from dv.resettle()
    tl = ctx.observe().tools["left"]
    LOG["after_grip"] = {"opening_mm": round(1000 * tl.opening, 1), "tcp": np.round(tl.tcp, 4).tolist(),
                         "cmd": np.round(dv.arm.pos, 4).tolist(), "drv": np.round(dv.seen()[:3, 3], 4).tolist(),
                         "P": np.round(P, 4).tolist()}
    D = dv.seen()
    lift = D.copy()
    lift[2, 3] = TOP + CLEAR_Z   # straight up so the shaft clears the rack's hole and, later, the open flap
    yield from dv.command(lift, 1.0)
    yield from dv.resettle()
    D1 = dv.seen()
    LOG["lifted_mm"] = round(1000 * (D1[2, 3] - D0[2, 3]), 1)
    LOG["G_z_mm"] = round(1000 * dv.G[2, 3], 1)


def carry(ctx, dv, tip, yaw, duration):
    yield from dv.command(dv.pose(tip, yaw), duration)
    yield from dv.resettle()


def head_info(ctx, name):
    seen = ctx.observe()
    M = seen.pose("cab", name)
    return M[:3, 3].copy(), yaw_deg(M), seen.angle_deg("cab", {"hs": "scr_s", "hd": "scr_d", "hc": "scr_c"}[name])


def turn_screw(ctx, dv, name, goal_deg, st):
    """Carry the blade over the head, align it with the seen slot, drop it in, turn until the seen head angle is at
    ``goal_deg`` (closed loop), and lift out."""
    c, slot_yaw, th0 = head_info(ctx, name)
    yield from carry(ctx, dv, np.array([c[0], c[1], TOP + SLOT_Z + 0.020]), slot_yaw, 1.8)
    c, slot_yaw, th0 = head_info(ctx, name)
    yield from carry(ctx, dv, np.array([c[0], c[1], TOP + SLOT_Z + 0.004]), slot_yaw, 1.0)
    def over_slot(z_off):
        c, slot_yaw, _ = head_info(ctx, name)
        D = dv.seen()
        e = np.array([c[0] - D[0, 3], c[1] - D[1, 3]])
        ye = (slot_yaw - yaw_deg(D) + 90.0) % 180.0 - 90.0
        return D, e, ye

    for k in range(5):   # null the blade's lateral and yaw error over the slot (no contact yet)
        D, e, ye = over_slot(0.004)
        st.setdefault("servo", []).append([round(1000 * float(e[0]), 2), round(1000 * float(e[1]), 2), round(ye, 1)])
        if float(np.linalg.norm(e)) < 0.00025 and abs(ye) < 1.0:
            break
        yield from dv.command(dv.pose(np.array([D[0, 3] + e[0], D[1, 3] + e[1], TOP + SLOT_Z + 0.004]),
                                      yaw_deg(D) + ye), 0.5)
        yield from dv.resettle(0.45)
    for z_off in (0.002, 0.0):   # then straight down into the slot, holding the measured lateral error at zero
        D, e, ye = over_slot(z_off)
        yield from dv.command(dv.pose(np.array([D[0, 3] + e[0], D[1, 3] + e[1], TOP + SLOT_Z + z_off]),
                                      yaw_deg(D) + ye), 0.5)
        yield from dv.resettle(0.3)
    for it in range(14):
        c, slot_yaw, th = head_info(ctx, name)
        err = goal_deg - th
        Dd = dv.seen()
        st.setdefault("turn_dbg", []).append([round(1000 * (Dd[0, 3] - c[0]), 2), round(1000 * (Dd[1, 3] - c[1]), 2),
                                              round(1000 * (Dd[2, 3] - c[2]), 2), round(yaw_deg(Dd), 1),
                                              round(slot_yaw, 1), round(th, 2)])
        if abs(err) <= 1.0:
            break
        step = float(np.clip(err, -14.0, 14.0)) + math.copysign(1.5, err)
        D = dv.seen()
        yaw_now = yaw_deg(D)
        yield from dv.command(dv.pose(np.array([c[0], c[1], TOP + SLOT_Z]), yaw_now + step), max(0.4, abs(step) / 25.0))
        yield from dv.resettle(0.2)
    c, slot_yaw, th = head_info(ctx, name)
    st.setdefault("turned", {})[name] = round(th, 2)
    yield from dv.command(dv.pose(np.array([c[0], c[1], TOP + SLOT_Z + 0.012]), yaw_deg(dv.seen())), 0.6)
    yield from dv.resettle(0.2)


def return_driver(ctx, dv):
    D = dv.seen()
    yield from carry(ctx, dv, np.array([D[0, 3], D[1, 3], TOP + CLEAR_Z]), yaw_deg(D), 0.7)    # straight up, clear of the flap
    yield from carry(ctx, dv, np.array([DRV_X, DRV_Y, TOP + CLEAR_Z]), 0.0, 1.5)               # over the rack
    yield from carry(ctx, dv, np.array([DRV_X, DRV_Y, TOP + RACK_TOP - HANDLE_BOTTOM + 0.040]), 0.0, 0.9)
    D = dv.seen()
    yield from dv.command(dv.pose(np.array([DRV_X, DRV_Y, TOP + RACK_TOP - HANDLE_BOTTOM + 0.002]), yaw_deg(D)), 1.0)
    yield from dv.arm.grip(0.094, 0.4)
    yield from dv.arm.move(dv.arm.pos + np.array([0.0, 0.0, 0.06]), duration=0.8)


def door_estimate(ctx):
    """Sideways offset of the door's hinge line from nominal (m), from the seen door pose."""
    D = ctx.observe().pose("cab", "door")
    return float((D[:3, 3] + D[:3, :3] @ np.array([-BOARD / 2, 0.0, -DH / 2]))[1])


PINCH_R = 0.10           # pinch the door's left side edge this far along it from the hinge line


def close_door(ctx):
    from linkdemo import OPEN_WIDTH, down, grip_frame

    arm = ctx.left
    D = ctx.observe().pose("cab", "door")
    psi = math.degrees(math.atan2(-D[0, 2], D[2, 2]))
    edge = D[:3, 3] + D[:3, :3] @ np.array([0.0, DW / 2, -DH / 2 + PINCH_R])
    n = D[:3, :3] @ np.array([1.0, 0.0, 0.0])
    a = np.array([0.0, -1.0, 0.0])
    R = grip_frame(a, n)
    tcp = edge + a * 0.010
    yield from arm.move(tcp - a * 0.08 + [0, 0, 0.03], R, width=0.040, duration=1.8)   # pads 40 mm apart: 11 mm each side of the slab
    yield from arm.move(tcp - a * 0.04, R, duration=0.5)
    yield from arm.move(tcp, R, duration=0.7)
    yield from arm.wait(0.5)
    tl = ctx.observe().tools["left"]
    err = tl.tcp - tcp
    LOG["pinch_err_mm"] = np.round(1000 * err, 1).tolist()
    if float(np.linalg.norm(err)) > 0.002:
        yield from arm.move(arm.pos - err, R, duration=0.5)
        yield from arm.wait(0.4)
        LOG["pinch_err2_mm"] = np.round(1000 * (ctx.observe().tools["left"].tcp - tcp), 1).tolist()
    yield from arm.grip(BOARD - 0.008, 0.5)
    yield from arm.wait(0.2)
    LOG["pinch_opening_mm"] = round(1000 * ctx.observe().tools["left"].opening, 1)
    hinge = np.array([PIVOT[0], tcp[1], TOP + PIVOT[1]])
    yield from arm.arc(hinge, (0, 1, 0), psi - CLOSE_TO_DEG, point=(0.0, 0.0, 0.010), follow=1.0, duration=3.0)
    for k in range(3):   # closed loop on the seen door angle (the arm lags the commanded arc)
        yield from arm.wait(0.3)
        D = ctx.observe().pose("cab", "door")
        ps = math.degrees(math.atan2(-D[0, 2], D[2, 2]))
        LOG.setdefault("arc_psi", []).append(round(ps, 1))
        if ps <= CATCH_DEG - 3.0:
            break
        yield from arm.arc(hinge, (0, 1, 0), ps - CLOSE_TO_DEG, point=(0.0, 0.0, 0.010), follow=1.0, duration=1.0)
    D = ctx.observe().pose("cab", "door")
    LOG["psi_release"] = round(math.degrees(math.atan2(-D[0, 2], D[2, 2])), 2)
    yield from arm.grip(OPEN_WIDTH, 0.5)
    yield from arm.move(np.array([0.12, 0.25, TOP + 0.19]), down(0.0), duration=2.0)  # out along +y and clear of the door


def to_ready(ctx):
    """Back to the ready pose, clear of the cabinet and the rack."""
    from yam_arena import ik
    from yam_arena.constants import READY

    arm = ctx.left
    pos, rot = ik.tcp_pose(np.array(READY, float), arm.base)
    yield from arm.move(pos, rot, duration=2.2)


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    LOG["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    if variant == "probe":
        sn = ctx.observe()
        LOG["drv"] = np.round(sn.pose("driver", "drv"), 4).tolist()
        LOG["hs"] = np.round(sn.pose("cab", "hs"), 4).tolist()
        LOG["door"] = np.round(sn.pose("cab", "door"), 4).tolist()
        LOG["tools"] = {k: [np.round(v.tcp, 3).tolist(), round(v.opening, 4)] for k, v in sn.tools.items()}
        yield from ctx.wait(1.0)
        return
    dv = Driver(ctx)
    th_s = head_info(ctx, "hs")[2]
    d_est = door_estimate(ctx)
    LOG["d_est_mm"] = round(1000 * d_est, 2)
    goal = math.degrees(math.asin(max(-1.0, min(1.0, (A_SIDE * math.sin(math.radians(th_s)) - d_est) / A_SIDE))))
    ops = {"": [("hs", goal)], "depth_screw": [("hd", 50.0)], "clamp_first": [("hc", 60.0), ("hs", goal)],
           "wrong_way": [("hs", th_s + 30.0)], "overturn": [("hs", -85.0)]}[variant]
    LOG["goal"] = round(goal, 2)
    yield from pick(ctx, dv)
    for name, g in ops:
        yield from turn_screw(ctx, dv, name, g, LOG)
    yield from return_driver(ctx, dv)
    yield from close_door(ctx)
    yield from to_ready(ctx)
    yield from ctx.wait(1.6)


def demo_report() -> dict:
    score_, why = ladder(REC)
    H, m = REC.H, REC.last or {}
    last = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in m.items() if k not in ("clear", "tcp_speed", "th")}
    if m:
        last["th"] = {k: round(v, 2) for k, v in m["th"].items()}
        last["clear_mm"] = {s: round(1000 * v, 1) for s, v in m["clear"].items()}
    return {"score": score_, "reason": why, "variant": VARIANT,
            "evers": {k: round(v, 2) for k, v in H.evers.items()}, "peaks": {k: round(v, 2) for k, v in H.peaks.items()},
            "runs": {k: round(v, 2) for k, v in H.runs.items()}, "last": last, "log": dict(LOG), "trace": REC.trace}
