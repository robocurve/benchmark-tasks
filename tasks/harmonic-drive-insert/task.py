# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Insert a strain-wave (harmonic drive) flexspline unit into its circular spline, turning it so the teeth mesh.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A joint housing (anodised aluminium, black, 80 mm across, 30 mm tall, 0.35 kg) is bolted to the table at
  (0.30, -0.10) with its bore up. The bore is a circular spline: 52 internal teeth (tips at r 24 mm, roots at r 26 mm,
  12 mm long, 1.05 mm wide, 6.92 deg pitch) over the top 12 mm, then a plain cavity to 16 mm deep.
- A flexspline unit (steel, 0.18 kg, blue collar) stands on the table at (0.30, -0.25) on its lead end, hub up. The cup
  is 47 mm across and 14 mm long; the wave generator inside it makes the teeth engage in two patches of 5, on opposite
  sides, each tooth 1.0 mm wide and 12 mm long (tips at r 25.5 mm). A 63 mm collar sits above the cup and a hub with
  two wrench flats 14 mm apart rises 40 mm from it. The patches start in line with the circular spline's teeth.

Goal: the unit seated with its collar flat on the housing face, teeth meshed, both grippers let go.
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

HOUSING_XY = (0.30, -0.10)
FS_XY = (0.30, -0.25)
HOUSING_H = 30.0  # mm
N_CS = 52
PITCH_DEG = 360.0 / N_CS
TOOTH_LEN = 12.0  # mm, both toothed zones
FS_TEETH_PER_PATCH = 5

BLACK = (0.10, 0.10, 0.12)
BLUE = (0.15, 0.30, 0.75)
STEEL = (0.62, 0.64, 0.68)

# -- housing: root frame at the table, bore axis on +z through the origin ------------------------------------------------
_h = []
_h.append(lk.cylinder(0.040, 0.014, (0, 0, 0.007)))  # floor under the 16 mm cavity
for k in range(36):  # wall ring r 26..40 mm, 36 boxes
    a = 10.0 * k
    c = 33.0 * MM
    _h.append(lk.box((0.014, 0.0062, 0.016), (c * math.cos(math.radians(a)), c * math.sin(math.radians(a)), 0.022),
                     rpy_deg=(0, 0, a)))
for k in range(N_CS):  # circular spline teeth, r 24..26 mm
    a = PITCH_DEG * k
    c = 24.95 * MM
    _h.append(lk.box((0.0021, 0.00105, TOOTH_LEN * MM),
                     (c * math.cos(math.radians(a)), c * math.sin(math.radians(a)), (HOUSING_H - TOOTH_LEN / 2) * MM),
                     rpy_deg=(0, 0, a)))
_t = lk.Tree("housing", pos=(HOUSING_XY[0], HOUSING_XY[1], TOP), fixed=True, color=BLACK)
_t.link("housing", _h, mass=0.35, friction=(0.4, 0.3), contact_offset=0.0005, rest_offset=0.0)
HOUSING = _t.build()

# -- flexspline unit: root frame at the collar's bottom centre, axis +z (hub up) ------------------------------------------
_f = [
    lk.cylinder(0.0315, 0.003, (0, 0, 0.0015)),  # collar
    lk.cylinder(0.0235, 0.014, (0, 0, -0.007)),  # cup wall (solid body)
    lk.box((0.014, 0.022, 0.040), (0, 0, 0.023)),  # hub with two wrench flats 14 mm apart (closing axis x)
]
for base in (0.0, 180.0):
    for j in range(-(FS_TEETH_PER_PATCH // 2), FS_TEETH_PER_PATCH // 2 + 1):
        a = base + PITCH_DEG * j
        c = 24.45 * MM
        _f.append(lk.box((0.0021, 0.0010, TOOTH_LEN * MM),
                         (c * math.cos(math.radians(a)), c * math.sin(math.radians(a)), -TOOTH_LEN / 2 * MM),
                         rpy_deg=(0, 0, a)))
FS_START = (FS_XY[0], FS_XY[1], TOP + 0.014 + 0.0005)
_t = lk.Tree("fs", pos=FS_START, color=STEEL)
_t.link("fs", _f, mass=0.18, friction=(0.5, 0.4), contact_offset=0.0005, rest_offset=0.0)
FS = _t.build()

OBJECTS = [HOUSING, FS]

# -- scoring -------------------------------------------------------------------------------------------------------------
FACE_Z = HOUSING_H * MM  # housing frame z of the face the collar seats on
ALIGN_TOL_DEG = 1.0  # teeth in line with the gaps (clearance 0.5 mm of 25 mm radius = 1.15 deg)
LAT_TOL_MM = 1.0
TILT_TOL_DEG = 3.0
COCK_DEG = 8.0  # gate: cocked this far while its teeth are in the bore zone
STALL_G_MM = 13.0  # collar height at which the teeth tips reach the housing's tooth tips is 12.0
MESH_G_MM = 6.0
SEAT_G_MM = 0.5
STEADY_S, RELEASE_S = 1.0, 1.0
CLEAR_M, TCP_STILL = 0.030, 0.005
FS_STILL, FS_SPIN = 0.003, math.radians(3.0)
EPS = 1e-6


def phase_err_deg(theta_deg: float) -> float:
    """Signed turn (deg) that puts the unit's teeth in the housing's gaps, wrapped to +-pitch/2. Housing teeth sit at
    k x pitch; the unit's teeth sit at theta + k x pitch; they are in line with the gaps when theta = pitch/2."""
    d = (theta_deg - PITCH_DEG / 2.0) % PITCH_DEG
    return d - PITCH_DEG if d > PITCH_DEG / 2.0 else d


def measure(housing, fs, tools) -> dict:
    Hm = np.array(housing.poses["housing"], dtype=float)
    A = np.array(fs.poses["fs"], dtype=float)
    R = Hm[:3, :3].T @ A[:3, :3]
    p = Hm[:3, :3].T @ (A[:3, 3] - Hm[:3, 3])
    theta = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    near, clear = {}, {}
    for side, tool in tools.items():
        near[side] = lk.clearance(tool, fs)
        clear[side] = min(near[side], lk.clearance(tool, housing))
    return {
        "g": float((p[2] - FACE_Z) * 1000.0),
        "lat": float(math.hypot(p[0], p[1]) * 1000.0),
        "tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))),
        "phase": phase_err_deg(theta),
        "fs_speed": float(np.linalg.norm(fs.lin_vels["fs"])),
        "fs_spin": float(np.linalg.norm(fs.ang_vels["fs"])),
        "clear": clear,
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def pose_rates(A_prev, A, dt: float) -> tuple[float, float]:
    if dt <= 0.0:
        return 0.0, 0.0
    v = float(np.linalg.norm(A[:3, 3] - A_prev[:3, 3])) / dt
    R = A_prev[:3, :3].T @ A[:3, :3]
    sin = 0.5 * float(np.linalg.norm((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1])))
    return v, math.atan2(sin, (float(np.trace(R)) - 1.0) / 2.0) / dt


def lined_up(m) -> bool:
    return abs(m["phase"]) <= ALIGN_TOL_DEG and m["lat"] <= LAT_TOL_MM and m["tilt"] <= TILT_TOL_DEG


def seated(m) -> bool:
    return m["g"] <= SEAT_G_MM and m["lat"] <= LAT_TOL_MM and m["tilt"] <= TILT_TOL_DEG


class Record:
    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
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
        H.ever("cocked", m["tilt"] >= COCK_DEG and m["g"] <= STALL_G_MM and m["lat"] <= 12.0)
        H.low("g_min", m["g"])
        H.ever("lined", m["g"] <= STALL_G_MM and lined_up(m))
        H.ever("meshed", m["g"] <= MESH_G_MM and m["lat"] <= LAT_TOL_MM)
        H.held("steady", seated(m) and m["fs_speed"] < FS_STILL and m["fs_spin"] < FS_SPIN)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "cocked" in H.evers:
        return 0.0, f"gate: the unit was cocked {COCK_DEG} deg or more over the housing at {H.evers['cocked']:.2f} s"
    if seated(m):
        ends = {
            "steady 1 s": H.runs.get("steady", 0.0) >= STEADY_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, "unit seated, teeth meshed, both grippers let go"
        return 0.75, "seated at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "meshed" in H.evers:
        return 0.5, "teeth went 6 mm into the housing at some point but the unit is not seated at the end"
    if "lined" in H.evers:
        return 0.25, "teeth were lined up with the gaps over the housing but never went 6 mm in"
    return 0.0, "teeth never lined up with the gaps over the housing"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    m = measure(tick.states["housing"], tick.states["fs"], tick.tools)
    A = np.array(tick.states["fs"].poses["fs"], dtype=float)
    if _PREV:
        m["fs_speed"], m["fs_spin"] = pose_rates(_PREV["A"], A, tick.t - _PREV["t"])
    _PREV.update(A=A, t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(m["g"], 2), round(m["lat"], 2), round(m["tilt"], 2),
                          round(m["phase"], 2), [round(1000 * m["clear"][s], 0) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


# -- scripted demo (observable poses only) ---------------------------------------------------------------------------
VARIANTS = ("", "nothing", "straight_down", "half_turn", "drop_over_bore")
GRIP_OPEN, GRIP_CLOSED = 0.040, 0.0125  # hub flats are 14 mm apart: 1.5 mm squeeze a pad
HUB_Z = 0.028  # grasp height above the collar bottom along the hub
PAD_DEPTH = 0.009  # pad contact centre past the TCP along the approach
HOVER_G = 16.0  # mm collar height while turning (teeth 4 mm above the tips)
PRESS_G = -2.0  # command: 2 mm below the seat, so the arm presses the collar on
DESCENT_MM_S = 12.0
LOG: dict = {}


def _frames(ctx):
    seen = ctx.observe()
    return seen.pose("housing", "housing"), seen.pose("fs", "fs")


def demo_script(ctx):
    from linkdemo import grip_frame, rot_about

    v = VARIANT
    LOG.clear()
    LOG["variant"] = v
    yield from ctx.wait(1.0)
    if v == "nothing":
        yield from ctx.wait(8.0)
        return
    arm = ctx.right
    Hm, A = _frames(ctx)
    z_up = np.array([0.0, 0.0, 1.0])
    closing = A[:3, :3] @ np.array([1.0, 0.0, 0.0])
    rot = grip_frame(-z_up, closing)
    axis_pt = Hm[:3, :3] @ np.array([0.0, 0.0, FACE_Z]) + Hm[:3, 3]  # bore axis at the face

    def tcp_for(contact):
        return contact - rot @ np.array([0.0, 0.0, PAD_DEPTH])

    hub = A[:3, 3] + np.array([0.0, 0.0, HUB_Z])
    yield from arm.move(tcp_for(hub + np.array([0.0, 0.0, 0.08])), rot, width=GRIP_OPEN, duration=2.0)
    yield from arm.move(tcp_for(hub), rot, duration=1.5)
    yield from arm.grip(GRIP_CLOSED, 0.8)
    yield from arm.wait(0.4)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.07]), duration=1.2)

    def corr(target_g_mm):
        """TCP position that would put the seen unit's origin over the axis at collar height target_g."""
        _, A_ = _frames(ctx)
        want = axis_pt + np.array([0.0, 0.0, target_g_mm * MM])
        return arm.pos + (want - A_[:3, 3])

    # carry over the bore, null the seen offset in passes
    yield from arm.move(corr(40.0), duration=2.0)
    for _ in range(3):
        yield from arm.wait(0.3)
        yield from arm.move(corr(40.0), duration=0.6)
    if v == "drop_over_bore":
        yield from arm.grip(GRIP_OPEN, 0.5)
        yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), duration=1.2)
        yield from ctx.wait(3.0)
        return
    yield from arm.move(corr(HOVER_G), duration=(40 - HOVER_G) / DESCENT_MM_S)
    # turn the unit until the seen phase is in the gaps
    for k in range(4):
        yield from arm.wait(0.4)
        _, A_ = _frames(ctx)
        R = Hm[:3, :3].T @ A_[:3, :3]
        err = phase_err_deg(math.degrees(math.atan2(R[1, 0], R[0, 0])))
        LOG.setdefault("phase_trace", []).append(round(err, 2))
        if v == "straight_down":
            break
        if v == "half_turn":
            err = err / 2.0 if k == 0 else 0.0
        if abs(err) <= 0.3 and v != "half_turn":
            break
        yield from arm.move(corr(HOVER_G), rot_about(z_up, -err) @ arm.rot, duration=max(0.5, abs(err) / 4.0))
        yield from arm.move(corr(HOVER_G), duration=0.4)
    yield from arm.move(corr(STALL_G_MM + 1.0), duration=(HOVER_G - STALL_G_MM - 1.0) / DESCENT_MM_S)
    yield from arm.wait(0.3)
    yield from arm.move(corr(PRESS_G), duration=(STALL_G_MM + 1.0 - PRESS_G) / DESCENT_MM_S + 2.0)
    yield from arm.wait(1.5)
    yield from arm.grip(GRIP_OPEN, 0.6)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), duration=1.2)
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    s, why = ladder(REC)
    H = REC.H
    m = REC.last or {}
    return {
        "score": s, "reason": why, "variant": VARIANT,
        "evers": {k: round(x, 2) for k, x in H.evers.items()},
        "lows": {k: round(x, 2) for k, x in H.lows.items()},
        "runs": {k: round(x, 2) for k, x in H.runs.items()},
        "last": {k: (round(x, 2) if isinstance(x, float) else x) for k, x in m.items() if k not in ("clear", "tcp_speed")},
        "clear_mm": {k: round(1000 * x, 1) for k, x in (m.get("clear") or {}).items()},
        "log": dict(LOG), "trace": REC.trace,
    }
