# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Connect a garden hose to its tap with a ball-lock quick-connect coupler.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475. The tap-side coupler's mouth is at world (0.15, 0) on the table top; its bore runs
along world +x, away from the robot, with its axis 30 mm above the table.

- Tap-side female coupler (brass, fixed on a tap stand): a 30 x 30 x 50 mm body with a 20 x 20 mm bore 40 mm deep,
  mouth toward the robot. A steel locking pawl (3.5 x 10 x 10 mm) slides vertically in a slot 8 mm behind the mouth;
  at rest its tip stands 6.5 mm into the bore, 3.5 mm above the axis. The 15.5 mm shaft can sit at most 2.25 mm off
  the bore axis, so the 11 mm groove core is never more than 5.75 mm from the pawl tip's rest position: with the groove
  under it the pawl always drops onto the core (tip within 5.75 mm of the shaft axis), and on the 15.5 mm shaft its tip
  is at least 5.5 mm up from rest, 7.75 mm from the shaft axis.
- Sprung collar (black, 44 x 24 x 42 mm) over the front of the body, sliding 10 mm toward the tap against a spring
  (1.2 N at rest, 4.2 N at 10 mm), with two finger rails along its sides (52 mm over both) that end in 3 mm thumb
  ridges. Its inner roof is a land 1.6 mm over the pawl; the roof is cut away just in front of the land, so the pawl
  can rise only once the collar is pulled back 6 mm or more.
- Hose end (green hose, 25 mm outer diameter, 5 links over 0.20 m, anchored on the table by a clamp post at world
  (-0.10, 0) and lying straight toward the right): a brass male nipple, 15.5 mm shaft with a bullet nose, 28 mm long,
  a 9 mm groove 4 to 13 mm behind the flange (core 11 mm diameter), a 26 x 26 x 8 mm flange and a 24 x 24 x 62 mm
  crimp body with strain-relief boot (120 g).
Locking is physical, and the score checks it. Pushed in with the collar forward, the nipple's nose stops on the pawl,
which the collar's land holds down. With the collar pulled back the pawl can rise and the nipple slides in; when the
flange reaches the mouth the pawl drops into the groove, the collar springs forward over it and the nipple is held.
The scorer reads the pawl's pose against the nipple's: the lock counts only while the pawl tip is in the groove (centre
6 to 11 mm behind the flange face, within 7 mm of the shaft axis), the simulation's equivalent of the 10 N pull
the real trial applies.
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
OPTS = {"squeeze": 0.002, "pull": 0.0085, "pgrip": 0.010, "ks": 300.0, "lz": 0.0}
OPTS.update({k: float(v) for k, v in (kv.split("=") for kv in VARIANT.split(",") if "=" in kv)})

MOUTH = (0.15, 0.0)  # mouth of the coupler in the world; the bore runs along world +x
AZ = 0.030  # bore axis height above the table
COLLAR_BACK_M = 0.007  # collar pulled back at least this far counts as "held back"
TIP_IN_M = 0.010  # nipple tip this far inside the mouth
SEAT_FLANGE_M = 0.003  # flange within this of the mouth face (the pawl has 1.25 mm of slack each way in the groove)
SEAT_TILT_DEG = 12.0
SEAT_LAT_M = 0.004
GROOVE_X = (0.0060, 0.0110)  # pawl centre this far behind the flange face: the 3.5 mm pawl is inside the 4 to 13 mm groove with 0.25 mm margin
PAWL_R_MAX = 0.0070  # pawl tip within this of the shaft axis (core 5.5 mm, shoulder 7.75 mm): at least 0.75 mm below the shoulder
PAWL_TIP = 0.005  # pawl tip below its link centre
COLLAR_FWD_M = 0.001
CLEAR_M = 0.030
TCP_STILL = 0.005
PLUG_STILL = 0.003
RELEASE_S = 1.0
SEAT_HOLD_S = 0.5  # engaged and seated this long without a break at some time: a pass-through does not count
EPS = 1e-6
SHAFT = 0.028  # nipple length from flange face to tip
BRASS = (0.80, 0.62, 0.25)
BLACK = (0.12, 0.12, 0.14)
STEEL = (0.70, 0.72, 0.75)
GREEN = (0.10, 0.45, 0.20)

# -- tap-side coupler (fixed body, sprung collar, pawl) --------------------------------------------------------------
t = lk.Tree("tap", pos=(MOUTH[0], MOUTH[1], TOP), yaw_deg=0.0, fixed=True, color=BRASS)
body = [
    lk.box((0.050, 0.030, 0.005), (0.025, 0.0, 0.0175)),  # bottom wall
    lk.box((0.050, 0.005, 0.030), (0.025, 0.0125, AZ)),  # side walls
    lk.box((0.050, 0.005, 0.030), (0.025, -0.0125, AZ)),
    lk.box((0.00575, 0.030, 0.004), (0.002875, 0.0, 0.042)),  # top wall in front of the pawl slot, z 40 to 44 mm
    lk.box((0.03975, 0.030, 0.004), (0.030125, 0.0, 0.042)),  # top wall behind the pawl slot (slot 4.5 mm wide)
    lk.box((0.010, 0.030, 0.030), (0.045, 0.0, AZ)),  # end wall: bore 40 mm deep
    lk.cylinder(0.011, 0.040, (0.070, 0.0, AZ), axis="X"),  # tap spigot
    lk.box((0.054, 0.024, 0.015), (0.063, 0.0, 0.0075)),  # tap stand, behind the collar's travel
]
t.link("body", body, mass=0.5, friction=(0.3, 0.3), contact_offset=0.001)
t.link("collar", [
    lk.box((0.024, 0.044, 0.004), (0.012, 0.0, 0.010)),  # floor, z 8 to 12 mm (3 mm under the body)
    lk.box((0.024, 0.004, 0.038), (0.012, 0.020, AZ + 0.0025)),  # sides, inner face 3 mm off the body
    lk.box((0.024, 0.004, 0.038), (0.012, -0.020, AZ + 0.0025)),
    lk.box((0.020, 0.044, 0.0035), (0.014, 0.0, 0.04685)),  # roof from 4 mm back: the land, 1.6 mm over the pawl's top
    lk.box((0.020, 0.008, 0.028), (0.014, 0.022, AZ)),  # finger rails along both sides, x 4 to 24 mm, outer faces 52 mm apart
    lk.box((0.020, 0.008, 0.028), (0.014, -0.022, AZ)),
    lk.box((0.002, 0.003, 0.024), (0.023, 0.0275, AZ)),  # thumb ridges at the rear end of each rail
    lk.box((0.002, 0.003, 0.024), (-0.0 + 0.023, -0.0275, AZ)),
], mass=0.04, at=(0.0, 0.0, 0.0), color=BLACK, friction=(1.0, 0.9), contact_offset=0.001)
t.link("pawl", [lk.box((0.0035, 0.010, 0.010), (0.0, 0.0, 0.0))], mass=0.005, at=(0.008, 0.0, AZ + 0.0085), color=STEEL,
       friction=(0.2, 0.15), contact_offset=0.001)
t.prismatic("slide", "body", "collar", anchor=(0.0, 0.0, AZ), axis=(1, 0, 0), limits=(0.0, 0.010), rest=-0.004 * OPTS["ks"] / 800.0,
            stiffness=OPTS["ks"], damping=6.0)
t.prismatic("pawl_slide", "body", "pawl", anchor=(0.008, 0.0, AZ + 0.0085), axis=(0, 0, 1), limits=(0.0, 0.006),
            rest=-0.0015, stiffness=100.0, damping=1.0)
TAP = t.build()

# -- hose with its male nipple ---------------------------------------------------------------------------------------
HOSE_POS = (-0.10, 0.0)
t = lk.Tree("hose", pos=(HOSE_POS[0], HOSE_POS[1], TOP + 0.0125), yaw_deg=-18.0, fixed=True, color=GREEN)
t.link("clamp", [lk.box((0.03, 0.05, 0.025), (-0.015, 0.0, 0.0))], mass=0.5, color=(0.35, 0.35, 0.38))
N_LINK, L_LINK = 5, 0.040
for i in range(1, N_LINK + 1):
    t.link(f"h{i}", [lk.capsule(0.0125, 0.015, (0.02, 0.0, 0.0), axis="X")], mass=0.012,
           at=(L_LINK * (i - 1), 0.0, 0.0), friction=(0.5, 0.4))
t.link("plug", [
    lk.cylinder(0.00775, 0.0073, (0.01665, 0.0, 0.0), axis="X"),  # shaft, x 0.013 to 0.0203
    lk.capsule(0.00775, 0.0005, (0.02025, 0.0, 0.0), axis="X"),  # bullet nose, ends at x 0.028
    lk.cylinder(0.0055, 0.009, (0.0085, 0.0, 0.0), axis="X"),  # groove core, x 0.004 to 0.013
    lk.cylinder(0.00775, 0.004, (0.002, 0.0, 0.0), axis="X"),  # neck, x 0 to 0.004
    lk.box((0.008, 0.026, 0.026), (-0.004, 0.0, 0.0)),  # flange
    lk.box((0.062, 0.024, 0.024), (-0.039, 0.0, 0.0)),  # crimp body with its rubber strain-relief boot
], mass=0.12, at=(L_LINK * N_LINK + 0.070, 0.0, 0.0), color=BRASS, friction=(0.4, 0.3))
BENDS = {1: 0.0, 3: 0.0, 5: 0.0}  # yaw joints (the hose lies in an S-curve); the others bend up and down
prev = "clamp"
for i in range(1, N_LINK + 1):
    ax = (0, 0, 1) if i in BENDS else (0, -1, 0)
    deg = BENDS.get(i, 0.0)
    t.revolute(f"j{i}", prev, f"h{i}", anchor=(L_LINK * (i - 1), 0.0, 0.0), axis=ax, init_deg=deg, rest_deg=deg,
               limits_deg=(-120, 120), stiffness=0.003, damping=0.002)
    prev = f"h{i}"
t.revolute("jp", prev, "plug", anchor=(L_LINK * N_LINK, 0.0, 0.0), axis=(0, -1, 0), limits_deg=(-120, 120),
           stiffness=0.003, damping=0.002)
HOSE = t.build()
OBJECTS = [TAP, HOSE]


# -- measurement ------------------------------------------------------------------------------------------------------
def measure(tap, hose, tools=None, prev=None, dt=0.05):
    """The coupler and nipple state from link poses (m, deg). Returns (sample, plug pose)."""
    B = np.array(tap.poses["body"], dtype=float)
    P = np.array(hose.poses["plug"], dtype=float)
    rel = np.linalg.inv(B) @ P
    p, ax = rel[:3, 3], rel[:3, 0]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ax[0])))))
    m = {"collar": float(tap.q["slide"]), "pawl": float(tap.q["pawl_slide"]), "fx": float(p[0]),
         "tip": float(p[0] + SHAFT * ax[0]), "tilt": tilt, "lat": float(math.hypot(p[1], p[2] - AZ)),
         "engaged": False, "pawl_x": 0.0, "pawl_r": 1.0, "speed": 0.0, "clear": {}, "tcp_speed": {}}
    if "pawl" in tap.poses:  # engagement: the pawl tip is in the nipple's groove, below the shaft shoulder
        W = np.array(tap.poses["pawl"], dtype=float)
        tp = np.linalg.inv(P) @ (W @ np.array([0.0, 0.0, -PAWL_TIP, 1.0]))
        m["pawl_x"], m["pawl_r"] = float(tp[0]), float(math.hypot(tp[1], tp[2]))
        m["engaged"] = bool(GROOVE_X[0] <= tp[0] <= GROOVE_X[1] and m["pawl_r"] <= PAWL_R_MAX)
    if prev is not None and dt > 0:
        m["speed"] = float(np.linalg.norm(P[:3, 3] - prev[:3, 3]) / dt)
    if tools:
        for side, tool in tools.items():
            m["clear"][side] = min(lk.clearance(tool, tap), lk.clearance(tool, hose, ["plug", "h5"]))
            m["tcp_speed"][side] = float(tool.speed)
    return m, P


def seated(m) -> bool:
    """Flange at the mouth AND the pawl tip in the groove below the shoulder: the fitting cannot be pulled out."""
    return (-SEAT_FLANGE_M <= m["fx"] <= 0.0015 and m["tilt"] <= SEAT_TILT_DEG and m["lat"] <= SEAT_LAT_M
            and bool(m["engaged"]))


class Record:
    """Whole-run record (a linkkit History cleared only by the simulator reset) plus the latest sample."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
        self.n = 0

    def reset(self):
        self.H.reset()
        self.last, self.n = None, 0

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        back = m["collar"] >= COLLAR_BACK_M
        H.ever("tip_in", back and m["tip"] >= TIP_IN_M)
        H.held("seat_hold", seated(m))
        H.peak("collar_max", m["collar"])
        H.peak("pawl_max", m["pawl"])
        H.peak("tip_max", m["tip"])
        released = bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M and max(m["tcp_speed"].values()) < TCP_STILL
        H.held("released", released)
        H.held("done", seated(m) and m["collar"] <= COLLAR_FWD_M and m["speed"] < PLUG_STILL and released)


def ladder(rec: Record) -> tuple[float, str]:
    """Every rung needs the pawl engaged in the groove (the equivalent of the real 10 N pull test); positions alone earn 0."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if H.best_runs.get("seat_hold", 0.0) < SEAT_HOLD_S - EPS:
        return 0.0, "the pawl was never held in the groove with the flange at the mouth for 0.5 s: no lock"
    if seated(m) and m["collar"] <= COLLAR_FWD_M:
        if H.runs.get("done", 0.0) >= RELEASE_S - EPS:
            return 1.0, "pawl in the groove under the closed collar, fitting still, both grippers 30 mm clear for 1 s"
        return 0.75, "pawl in the groove under the closed collar at the end; not yet still with both grippers 30 mm clear for 1 s"
    return 0.5, "the pawl was held in the groove for 0.5 s at some time, but at the end the fitting is not locked with the collar forward"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    dt = tick.t - _PREV["t"] if _PREV else 0.05
    m, P = measure(tick.states["tap"], tick.states["hose"], tick.tools, _PREV.get("P"), dt)
    _PREV.update(P=P, t=tick.t)
    REC.update(m)
    REC.H.trace("diag", [round(1000 * m["collar"], 1), round(1000 * m["pawl"], 1), round(1000 * m["fx"], 0),
                         round(1000 * m["lat"], 0), round(m["tilt"], 0), round(1000 * m["pawl_x"], 1), round(1000 * m["pawl_r"], 1)]
                + [round(1000 * float(v), 0) for v in tick.tools["left"].tcp] + [round(1000 * float(v), 0) for v in tick.tools["right"].tcp],
                every=1.0)


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
LOG: dict = {}
VARIANTS = ("", "nothing", "no_collar", "collar_only", "drop_at_mouth", "left_only", "right_only", "short_stop")
ALIAS = {"left_only": "collar_only", "right_only": "no_collar"}  # one hand alone
STAGE_BACK = 0.040  # staging: flange this far in front of the mouth
GRASP_X = -0.052  # the right hand pinches the boot, in the plug frame


def _rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def pose(ctx, obj, link):
    return np.array(ctx.observe().pose(obj, link), dtype=float)


def hand_to(arm, hold, pos, yaw, duration):
    """Move the hand so the held plug's flange centre goes to pos with heading yaw (deg about the vertical).
    hold = (hand rotation at the grasp, offset plug - tcp in the hand frame, plug heading at the grasp)."""
    R0, off_h, h0 = hold
    Rh = _rz(yaw - h0) @ R0
    yield from arm.move(np.asarray(pos) - Rh @ off_h, Rh, duration=duration)


def right_arm(ctx, variant, B):
    from linkdemo import grip_frame

    arm = ctx.right
    st = LOG
    ax = B[:3, :3] @ np.array([1.0, 0.0, 0.0])  # the bore axis, world
    P = pose(ctx, "hose", "plug")
    c = P[:3, 3] + P[:3, :3] @ np.array([GRASP_X, 0.0, 0.0])
    R = grip_frame((0, 0, -1), P[:3, 1])
    tcp = c + np.array([0.0, 0.0, 0.006])
    yield from arm.move(tcp + [0, 0, 0.07], R, width=0.050, duration=1.8)
    yield from arm.move(tcp, R, duration=0.9)
    yield from arm.grip(0.024 - OPTS["pgrip"], 0.5)
    yield from arm.wait(0.3)
    P = pose(ctx, "hose", "plug")
    off_h = arm.rot.T @ (P[:3, 3] - arm.pos)
    heading = math.degrees(math.atan2(P[1, 0], P[0, 0]))
    hold = (arm.rot.copy(), off_h, heading)
    start = P[:3, 3] + np.array([0, 0, 0.045])
    yield from arm.move(arm.pos + [0, 0, 0.045], duration=0.7)
    gap = 0.020 if variant == "drop_at_mouth" else STAGE_BACK
    stage = B[:3, 3] - ax * gap + np.array([0.0, 0.0, AZ])
    target_heading = math.degrees(math.atan2(ax[1], ax[0]))
    lateral = (start - stage) - ((start - stage) @ ax) * ax
    behind = stage + lateral  # level with the staging pose along the bore, still beside the coupler
    path = [(behind, 1.4), (stage, 1.0)]
    for pos, dur in path:
        yield from hand_to(arm, hold, pos, target_heading, dur)
    st["ik_right_mm"] = round(arm.worst_mm, 1)
    for _ in range(3):  # close the loop on the seen plug pose
        yield from arm.wait(0.3)
        err = stage - pose(ctx, "hose", "plug")[:3, 3]
        st["stage_err_mm"] = [round(1000 * v, 2) for v in err]
        if float(np.linalg.norm(err)) < 0.02:  # a small slip only: a big one means the plug turned in the pads
            yield from arm.move(arm.pos + err, duration=0.7)
    st["staged"] = True
    if variant in ("collar_only", "drop_at_mouth"):
        yield from ctx.wait(4.0)
        yield from arm.open(0.6)
        yield from arm.retreat(0.08, 1.0)
        return
    if variant == "no_collar":
        yield from arm.move(arm.pos + ax * (gap + 0.002), duration=2.5)
        yield from arm.wait(1.0)
        yield from arm.open(0.6)
        yield from arm.retreat(0.08, 1.0)
        return
    while not st.get("collar_back"):
        yield from arm.wait(0.1)
    yield from arm.move(arm.pos + ax * (gap - 0.012), duration=1.2)
    stop_at = -0.0100 if variant == "short_stop" else -0.0004  # the cheat stops the flange 10 mm short of the mouth
    for _ in range(40):  # push in 3 mm at a time, taking out the sideways and height error seen at the flange
        Bn = pose(ctx, "tap", "body")
        rel = np.linalg.inv(Bn) @ pose(ctx, "hose", "plug")
        fx = float(rel[0, 3])
        if fx >= stop_at:
            break
        e = Bn[:3, :3] @ np.array([0.0, -rel[1, 3], AZ - rel[2, 3]])
        yield from arm.move(arm.pos + ax * min(0.003, -fx + 0.0015) + e, duration=0.35)
        yield from arm.wait(0.05)
    if variant != "short_stop":  # the cheat lets go of the collar at once, with the flange still 3 mm out
        yield from arm.move(arm.pos + ax * 0.002, duration=0.6)  # lean on the flange so the pawl can drop into the groove
        yield from arm.wait(1.5)
    Bn = pose(ctx, "tap", "body")
    st["flange_x_mm"] = round(1000 * float((np.linalg.inv(Bn) @ pose(ctx, "hose", "plug"))[0, 3]), 2)
    st["seated"] = True
    while not st.get("collar_fwd"):
        yield from arm.wait(0.1)
    yield from arm.wait(0.3)
    yield from arm.open(0.5)
    yield from arm.retreat(0.08, 0.8)
    yield from arm.move(arm.pos + [0.0, 0.0, 0.05], duration=0.7)


def left_arm(ctx, variant, B):
    """Pinch the collar's two finger tabs and pull it back along the bore; hold it until the nipple is seated."""
    from linkdemo import grip_frame

    arm = ctx.left
    st = LOG
    if variant in ("no_collar", "drop_at_mouth"):
        return
    ax = B[:3, :3] @ np.array([1.0, 0.0, 0.0])
    while not st.get("staged"):  # the right hand goes first: the two hands share the space in front of the mouth
        yield from arm.wait(0.1)
    C0 = pose(ctx, "tap", "collar")
    centre = C0[:3, 3] + C0[:3, :3] @ np.array([0.008, 0.0, AZ])
    R = grip_frame((0, 0, -1), C0[:3, 1])
    tcp = centre + np.array([0.0, 0.0, OPTS["lz"]])
    yield from arm.move(tcp + [0, 0, 0.08], R, width=0.066, duration=1.8)
    yield from arm.move(tcp, R, duration=0.9)
    yield from arm.grip(0.052 - OPTS["squeeze"], 0.5)
    yield from arm.wait(0.3)
    st["left_open_mm"] = round(1000 * ctx.observe().tools["left"].opening, 1)
    for _ in range(10):  # pull the collar back, closed loop on the seen collar position
        C = pose(ctx, "tap", "collar")
        back = float((C[:3, 3] - C0[:3, 3]) @ ax)
        st.setdefault("pull_mm", []).append(round(1000 * back, 2))
        err = OPTS["pull"] - back
        if abs(err) < 0.0007:
            break
        yield from arm.move(arm.pos + ax * min(err, 0.008), duration=0.5)
        yield from arm.wait(0.1)
    if variant == "collar_only":
        yield from arm.wait(1.0)
    else:
        st["collar_back"] = True
        while not st.get("seated"):
            yield from arm.wait(0.1)
    yield from arm.open(0.4)
    back = 1.0
    for _ in range(40):
        back = float((pose(ctx, "tap", "collar")[:3, 3] - C0[:3, 3]) @ ax)
        if back < 0.0007:
            break
        yield from arm.wait(0.1)
    st["collar_after_open_mm"] = round(1000 * back, 2)
    st["collar_fwd"] = True
    yield from arm.wait(0.3)
    yield from arm.retreat(0.08, 0.8)
    yield from arm.move(arm.pos + [0.0, 0.0, 0.05], duration=0.7)


def demo_script(ctx):
    LOG.clear()
    LOG["variant"] = VARIANT
    variant = ALIAS.get(VARIANT, VARIANT)
    yield from ctx.wait(0.5)
    if VARIANT == "nothing":
        yield from ctx.wait(8.0)
        return
    B = pose(ctx, "tap", "body")
    yield from ctx.both(left_arm(ctx, variant, B), right_arm(ctx, variant, B))
    yield from ctx.wait(3.6)


def demo_report() -> dict:
    score_, why = ladder(REC)
    m = REC.last or {}
    last = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items() if k not in ("clear", "tcp_speed")}
    last["pawl_x_mm"], last["pawl_r_mm"] = round(1000 * m.get("pawl_x", 0.0), 2), round(1000 * m.get("pawl_r", 0.0), 2)
    last["clear_mm"] = {s: round(1000 * v, 1) for s, v in (m.get("clear") or {}).items()}
    return {"score": score_, "why": why, "last": last, "log": dict(LOG), "opts": OPTS,
            "evers": REC.H.evers, "diag": REC.H.traces.get("diag"), "runs": {k: round(v, 2) for k, v in REC.H.runs.items()}}
