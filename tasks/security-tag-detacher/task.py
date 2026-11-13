# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Remove an anti-theft hard tag from a jacket sleeve cuff with a hand-held magnetic detacher.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A quilted jacket-sleeve cuff (navy, 90 mm wide, 23 mm thick, 0.18 kg) lies on the table. Its 3 mm hem tab lies over
  a notch that is open toward the robot's left; a retail hard tag (ivory, 56 x 46 x 22 mm, 30 g) sits in the notch
  under the tab, its round lock dome (14 mm, 3 mm proud) on the +y side. The tag's pin (3 mm shaft, grey 14 x 14 mm
  knob on top of the tab, 12 g) goes through a hole in the tab into the tag, held by a clutch.
- A palm-size magnetic detacher (orange, 55 x 85 x 20 mm, 0.12 kg) lies on the table to the left, nose toward the
  tag. Its nose has an 18 x 16 mm pocket 4 mm deep.

Goal: the pin out of the tag (at least 15 mm), which only happens with the detacher's pocket seated over the dome
(the clutch releases only then); both grippers let go and clear.
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

# -- layout (m) -------------------------------------------------------------------------------------------------------
TAG_X, TAG_Y = 0.19, -0.03  # tag body centre on the table
BODY = (0.056, 0.046, 0.022)  # tag housing x, y, z
DOME_R, DOME_PROUD = 0.007, 0.003
TAB_Z0, TAB_Z1 = 0.023, 0.026  # hem tab bottom and top above the table
KNOB_H, KNOB_R = 0.014, 0.007
PIN_Z = TAB_Z1 + 0.0005 + KNOB_H / 2  # knob centre above the table
PUCK_START = (0.19, 0.13)  # nose rim centre on the table
PUCK_LEN, PUCK_W, PUCK_H = 0.085, 0.055, 0.020
POCKET_W, POCKET_H, POCKET_D = 0.018, 0.016, 0.004
PIN_STROKE = 0.030
CLUTCH_K, CLUTCH_C = 20000.0, 40.0  # N/m, N s/m while locked
PIN_FRICTION = 0.4  # N: holds the freed pin where it is left (pin weight 0.12 N)

# scoring thresholds
SEAT_DX, SEAT_DZ, SEAT_TILT = 0.003, 0.002, 6.0
SEAT_GAP = (-0.0005, 0.0015)
SEAT_HOLD_S = 0.3
FORCED_M = 0.003
OUT_HALF, OUT_FULL = 0.005, 0.015
CLEAR_M, STILL_MS = 0.030, 0.005
RELEASE_HOLD_S = 1.0

# -- scene ------------------------------------------------------------------------------------------------------------
t = lk.Tree("tag", pos=(TAG_X, TAG_Y, TOP), color=(0.92, 0.90, 0.84))
t.link("body", [lk.box(BODY, at=(0, 0, BODY[2] / 2), contact_offset=0.001),
                lk.cylinder(DOME_R, DOME_PROUD, at=(0, BODY[1] / 2 + DOME_PROUD / 2, 0.010), axis="Y",
                            color=(0.75, 0.75, 0.78))],
       mass=0.030, friction=(0.5, 0.4), contact_offset=0.001)
t.link("pin", [lk.cylinder(KNOB_R, KNOB_H, at=(0, 0, PIN_Z), color=(0.50, 0.50, 0.55)),
               lk.cylinder(0.0015, 0.010, at=(0, 0, TAB_Z1 - 0.003), color=(0.50, 0.50, 0.55), contact_offset=0.0005)],
       mass=0.012, friction=(0.7, 0.6), contact_offset=0.001)
t.prismatic("pin_slide", "body", "pin", anchor=(0, 0, PIN_Z), axis=(0, 0, 1), limits=(0.0, PIN_STROKE), rest=0.0,
            stiffness=0.0, damping=2.0, friction=0.0, brake=lk.Brake(stiffness=CLUTCH_K, damping=CLUTCH_C, engaged=True))
TAG = t.build()

# cuff frame: origin at the tag centre on the table; lump toward -y, notch open toward +y
t = lk.Tree("cuff", pos=(TAG_X, TAG_Y, TOP), color=(0.12, 0.18, 0.42))
W = 0.090
HC = TAB_Z0
HOLE = 0.002  # half-width of the square hole in the tab for the pin shaft (shaft radius 1.5 mm)
TT = TAB_Z1 - TAB_Z0
TZ = (TAB_Z0 + TAB_Z1) / 2
shapes = [
    lk.box((W, 0.175, HC), at=(0, -0.025 - 0.0875, HC / 2), contact_offset=0.001),  # lump, y -0.200 to -0.025
    lk.box((0.015, 0.055, HC), at=(-0.0375, 0.0025, HC / 2), contact_offset=0.001),  # arms, x +-(0.030 to 0.045)
    lk.box((0.015, 0.055, HC), at=(0.0375, 0.0025, HC / 2), contact_offset=0.001),
    # hem tab (3 mm), y -0.085 to +0.030, with a 4 mm square hole at the pin: four boxes
    lk.box((W / 2 - HOLE, 0.115, TT), at=(-(W / 2 + HOLE) / 2, -0.0275, TZ), contact_offset=0.0005),
    lk.box((W / 2 - HOLE, 0.115, TT), at=((W / 2 + HOLE) / 2, -0.0275, TZ), contact_offset=0.0005),
    lk.box((2 * HOLE, 0.083, TT), at=(0, -0.0435, TZ), contact_offset=0.0005),
    lk.box((2 * HOLE, 0.028, TT), at=(0, 0.016, TZ), contact_offset=0.0005),
]
t.link("cuff", shapes, mass=0.18, friction=(0.6, 0.5), contact_offset=0.001)
CUFF = t.build()

# detacher: frame origin at the nose rim centre on the table, y toward the handle end
t = lk.Tree("puck", pos=(PUCK_START[0], PUCK_START[1], TOP), color=(0.95, 0.45, 0.10))
ph = PUCK_H
side = (PUCK_W - POCKET_W) / 2
zf = (PUCK_H - POCKET_H) / 2
t.link("puck", [
    lk.box((PUCK_W, PUCK_LEN - POCKET_D, ph), at=(0, POCKET_D + (PUCK_LEN - POCKET_D) / 2, ph / 2)),
    lk.box((PUCK_W, POCKET_D, zf), at=(0, POCKET_D / 2, zf / 2)),
    lk.box((PUCK_W, POCKET_D, zf), at=(0, POCKET_D / 2, ph - zf / 2)),
    lk.box((side, POCKET_D, POCKET_H), at=(-(POCKET_W / 2 + side / 2), POCKET_D / 2, ph / 2)),
    lk.box((side, POCKET_D, POCKET_H), at=((POCKET_W / 2 + side / 2), POCKET_D / 2, ph / 2)),
], mass=0.12, friction=(0.5, 0.4), contact_offset=0.001)
PUCK = t.build()

OBJECTS = [TAG, CUFF, PUCK]

# -- measurement ------------------------------------------------------------------------------------------------------
H_ = lk.History()  # whole run; cleared only by the simulator reset


def seat_error(body_pose, puck_pose) -> dict:
    """Detacher nose against the tag's dome, in the tag-body frame (m, deg). Pure numpy.

    dx, dz: pocket centre minus dome centre; gap: nose rim past the tag's +y face (0 = touching, the dome 3 mm into
    the 4 mm pocket); tilt: angle between the nose normal and -y of the tag."""
    B, P = np.asarray(body_pose, float), np.asarray(puck_pose, float)
    rim = P[:3, 3] + P[:3, :3] @ np.array([0.0, 0.0, PUCK_H / 2])  # pocket centre on the rim plane
    d = B[:3, :3].T @ (rim - B[:3, 3])  # tag body frame origin: body centre on the table (z 0)
    nose = B[:3, :3].T @ (P[:3, :3] @ np.array([0.0, -1.0, 0.0]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(nose @ np.array([0.0, -1.0, 0.0]))))))
    return {"dx": float(d[0]), "dz": float(d[2] - 0.010), "gap": float(d[1] - BODY[1] / 2), "tilt": tilt}


def is_seated(m) -> bool:
    return (abs(m["dx"]) <= SEAT_DX and abs(m["dz"]) <= SEAT_DZ and SEAT_GAP[0] <= m["gap"] <= SEAT_GAP[1]
            and m["tilt"] <= SEAT_TILT)


def measure(tag, puck, tools) -> dict:
    m = seat_error(tag.poses["body"], puck.poses["puck"])
    m["pin"] = float(tag.q["pin_slide"])
    for s in ("left", "right"):
        tl = tools[s]
        m[f"clear_{s}"] = min(lk.clearance(tl, tag), lk.clearance(tl, puck))
        m[f"spd_{s}"] = float(tl.speed)
    return m


def update(h: lk.History, m: dict) -> bool:
    """Feed one control step. Returns True on the step the clutch should release (seated for 0.3 s)."""
    locked = "released" not in h.evers
    h.held("seat", is_seated(m))
    h.ever("forced", locked and m["pin"] >= FORCED_M)  # pin forced out of a locked clutch
    h.peak("pin_peak", m["pin"])
    h.ever("released", h.runs.get("seat", 0.0) >= SEAT_HOLD_S)
    h.held("hands", m["clear_left"] >= CLEAR_M and m["clear_right"] >= CLEAR_M
           and m["spd_left"] <= STILL_MS and m["spd_right"] <= STILL_MS)
    h.set("pin", m["pin"])
    return locked and "released" in h.evers


def ladder(h: lk.History) -> tuple[float, str]:
    ev, v = h.evers, h.values
    if "forced" in ev:
        return 0.0, "gate: the pin was forced out of a locked clutch"
    if "released" not in ev:
        return 0.0, "clutch never released"
    rung, why = 0.25, "detacher seated, clutch released"
    if h.peaks.get("pin_peak", 0.0) >= OUT_HALF:
        rung, why = 0.5, "pin pulled 5 mm"
    if v.get("pin", 0.0) >= OUT_FULL:
        rung, why = 0.75, "pin out 15 mm at the end"
        if h.runs.get("hands", 0.0) >= RELEASE_HOLD_S:
            rung, why = 1.0, "pin out, both grippers clear and still for 1 s"
    return rung, why


@lk.on_step
def _track(tick):
    m = measure(tick.states["tag"], tick.states["puck"], tick.tools)
    if update(H_, m):
        lk.brake("tag", "pin_slide", False)
        lk.set_friction("tag", "pin_slide", PIN_FRICTION, PIN_FRICTION)
    H_.set("m", m)


def score(objects) -> float:
    return ladder(H_)[0]


def success(objects) -> bool:
    return ladder(H_)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(H_)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ----------------------------------------------------------------------------------------------------
# reads only what a camera sees: link poses (tag body, pin, puck) and the robot's grippers; never joints or the record

from linkdemo import down, grip_frame, rot_about  # noqa: E402

PAD_TIP = 0.013
VARIANTS = ("", "nothing", "no_detacher", "offset_detacher", "drag_cuff")
OFFSET = 0.008 if VARIANT == "offset_detacher" else 0.0
LOG: dict = {}


def left_seat(ctx, flag):
    arm = ctx.left
    R = down(-90.0)  # pads close along world x, across the detacher's 55 mm width
    seen = ctx.observe()
    P = seen.pose("puck", "puck")
    hold = P @ np.array([0.0, 0.072, 0.0, 1.0])  # grasp point on the handle half, on the table
    tcp_z = TOP + 0.004 + PAD_TIP
    yield from arm.grip(0.075, 0.3)
    yield from arm.move([hold[0], hold[1], TOP + 0.10], R, duration=2.5)
    yield from arm.move([hold[0], hold[1], tcp_z], R, duration=1.5)
    yield from arm.grip(PUCK_W - 0.0015, 0.8)
    def servo(gap_mm, steps, gain=0.35):
        """Visual servo on the seen poses: rim x to the dome's x, rim y to the tag's face plus gap, yaw to zero."""
        nonlocal psi
        for k in range(steps):
            seen = ctx.observe()
            B, P = seen.pose("tag", "body"), seen.pose("puck", "puck")
            g = gap_mm(k / max(1, steps - 1)) if callable(gap_mm) else gap_mm
            want = B[:3, 3] + np.array([OFFSET, BODY[1] / 2 + g * 0.001, 0.0])
            d = (want[:2] - P[:2, 3]) * gain
            psi = math.degrees(math.atan2(-P[0, 1], P[1, 1]))  # detacher's yaw seen from above (0: nose along -y)
            Rz = rot_about((0, 0, 1), -gain * psi)
            rim = P[:3, 3]
            cur = rim + Rz @ (arm.pos - rim)  # swing the hand about the nose to null the yaw
            yield from arm.move([cur[0] + d[0], cur[1] + d[1], tcp_z], Rz @ arm.rot, duration=0.05)

    psi = 0.0
    seen = ctx.observe()
    B, P = seen.pose("tag", "body"), seen.pose("puck", "puck")
    cur = arm.pos.copy()
    yield from arm.move([cur[0], B[1, 3] + BODY[1] / 2 + 0.030 + (hold[1] - P[1, 3]), tcp_z], R, duration=2.5)
    yield from servo(30.0, 40)  # settle x and yaw 30 mm short of the face
    yield from servo(lambda s: 30.0 - 29.7 * s, 100)  # then close in to 0.3 mm
    yield from servo(0.3, 20)
    seen = ctx.observe()
    e = seat_error(seen.pose("tag", "body"), seen.pose("puck", "puck"))
    LOG["psi_deg"] = round(psi, 2)
    LOG["seat_err"] = {k: round(1000 * v, 2) if k != "tilt" else round(v, 2) for k, v in e.items()}
    yield from arm.wait(1.0)
    flag["seated"] = True
    while "pulled" not in flag:  # hold the detacher on the dome while the other arm pulls
        yield
    yield from arm.open(0.8)
    yield from arm.move(arm.pos + [0, 0, 0.07], R, duration=1.0)
    yield from arm.move(arm.pos + [0, 0.07, 0], R, duration=1.0)
    yield from arm.wait(1.5)


def right_pull(ctx, flag, variant):
    arm = ctx.right
    TILT = math.radians(35.0)  # hand leans away from the detacher hand: approach down and toward +y, pads close along x
    R = grip_frame((0.0, math.sin(TILT), -math.cos(TILT)), (1.0, 0.0, 0.0))
    tip = -R[:, 2] * PAD_TIP  # TCP = pad-tip point minus 13 mm along the approach
    seen = ctx.observe()
    knob = (seen.pose("tag", "pin") @ np.array([0.0, 0.0, PIN_Z, 1.0]))[:3]
    body0 = seen.pose("tag", "body")[2, 3]
    yield from arm.grip(0.030, 0.3)
    yield from arm.move([knob[0], knob[1] - 0.10, TOP + 0.13], R, duration=3.0)  # wait clear of the other hand
    if variant != "no_detacher":
        while "seated" not in flag:
            yield
        yield from arm.wait(0.5)
    yield from arm.move([knob[0], knob[1], TOP + 0.13], R, duration=2.0)
    yield from arm.move(np.array([knob[0], knob[1], TOP + 0.0285]) + tip, R, duration=1.5)
    yield from arm.grip(KNOB_R * 2 - 0.0015, 0.8)
    yield from arm.wait(0.3)
    z0 = arm.pos[2]
    yield from arm.move([knob[0], knob[1], z0 + 0.024], R, duration=3.0)
    s2 = ctx.observe()
    LOG["pin_rise_mm"] = round(1000 * (s2.pose("tag", "pin")[2, 3] - seen.pose("tag", "pin")[2, 3]), 2)
    LOG["body_rise_mm"] = round(1000 * (s2.pose("tag", "body")[2, 3] - body0), 2)
    yield from arm.wait(0.5)
    flag["pulled"] = True
    yield from arm.open(0.8)
    yield from arm.move(arm.pos + [0, 0, 0.07], R, duration=1.0)
    yield from arm.move(arm.pos + [0, -0.07, 0], R, duration=1.0)
    yield from arm.wait(1.5)


def right_drag_cuff(ctx, flag):
    arm = ctx.right
    R = down(90.0)  # pads close along x across the 90 mm cuff
    seen = ctx.observe()
    C = seen.pose("cuff", "cuff")
    g = C @ np.array([0.0, -0.13, 0.0, 1.0])
    yield from arm.grip(0.0939, 0.3)
    yield from arm.move([g[0], g[1], TOP + 0.10], R, duration=2.5)
    yield from arm.move([g[0], g[1], TOP + 0.004 + PAD_TIP], R, duration=1.5)
    yield from arm.grip(W - 0.001, 0.8)
    yield from arm.move([g[0], g[1] - 0.05, TOP + 0.004 + PAD_TIP], R, duration=3.0)
    yield from arm.wait(1.0)
    yield from arm.open(0.8)
    yield from arm.move(arm.pos + [0, 0, 0.07], R, duration=1.0)
    yield from arm.wait(1.5)


def demo_script(ctx):
    yield from ctx.wait(1.0)
    flag: dict = {}
    if VARIANT == "nothing":
        yield from ctx.wait(10.0)
    elif VARIANT == "drag_cuff":
        yield from right_drag_cuff(ctx, flag)
    elif VARIANT == "no_detacher":
        yield from right_pull(ctx, flag, "no_detacher")
    else:
        yield from ctx.both(left_seat(ctx, flag), right_pull(ctx, flag, VARIANT))
    yield from ctx.wait(2.0)


def demo_report() -> dict:
    h = H_
    return {"variant": VARIANT, "rung": ladder(h), "evers": dict(h.evers),
            "pin_peak_mm": round(1000 * h.peaks.get("pin_peak", 0), 2),
            "pin_end_mm": round(1000 * h.values.get("pin", 0), 2), "hands_run_s": round(h.runs.get("hands", 0), 2),
            "best_seat_run_s": round(h.best_runs.get("seat", 0), 2), "seen": dict(LOG),
            "trace": h.traces.get("seat", [])[::4]}
