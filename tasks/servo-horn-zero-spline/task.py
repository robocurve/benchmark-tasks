# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Fit a servo horn onto its splined output shaft with the horn arm on the zero mark.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475 (TOP).

- A hobby servo (40.7 x 19.7 x 36 mm case, 55 g, black) is bolted to the bench, fixed, with its 25-tooth output
  shaft (5.9 mm major diameter, standing 5 mm proud of its 11.5 mm boss) at (0.250, -0.050), case long axis along x.
  A white engraved zero mark runs from the boss out along +x on the case top.
- A white nylon single-arm servo horn (4 g: a 14 mm x 7 mm hub with a 6.8 mm bore for the shaft, a 28 mm arm 2 mm
  thick) lies flat on the table at (0.220, -0.220), arm pointing 68 deg from +x (not on a spline position).

A 25-tooth spline lets the horn seat only at 25 angles, 14.4 deg apart. Crest on crest the horn stands 1.5 mm proud
of the boss; a hand press of about 3.6 N seats it only when a tooth of the shaft is in a groove of the bore (within
2.0 deg of an index). Pressed where it lands, it seats a whole tooth away from the zero mark. A horn on the wrong tooth can be pulled off
and refitted; the score is where the horn sits at the end.

Goal: the horn seated flush on the boss with its arm on the zero mark (index 0), both grippers let go.
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

SHAFT_XY = (0.250, -0.050)
HORN_XY = (0.220, -0.220)
HORN_YAW0 = 68.0  # deg; 4.0 deg from index 5 (72), 10.4 from index 4 (57.6): not on a spline position
TEETH = 25
PITCH = 360.0 / TEETH  # 14.4 deg
Z_BOSS = 0.039  # boss top above the table (seated hub bottom)
TIP = 0.0015  # crest-on-crest stand-off
SEAT_TRAVEL = 0.0015
WINDOW_DEG = 2.0  # engagement window around each index
OFF_MAX_MM = 0.8  # bore centred on the shaft
TILT_MAX = 4.0
HUB_H = 0.007
SEAT_SPRING = 2400.0  # N/m: 3.6 N at the full 1.5 mm
SEAT_Q_MM = 1.3  # ring pressed this far = seated (brake engages)

# ---- servo (fixed) with a spring-loaded seat ring standing for the tooth crests --------------------------------
t = lk.Tree("servo", pos=(SHAFT_XY[0], SHAFT_XY[1], TOP), fixed=True, color=(0.08, 0.08, 0.09))
t.link("case", [
    lk.box((0.0407, 0.0197, 0.036), (0.0105, 0.0, 0.018)),
    lk.box((0.0545, 0.0170, 0.0025), (0.0105, 0.0, 0.0285), color=(0.12, 0.12, 0.13)),
    lk.cylinder(0.00575, 0.003, (0.0, 0.0, 0.0375), color=(0.70, 0.70, 0.72)),
    lk.cylinder(0.00295, 0.005, (0.0, 0.0, 0.0415), color=(0.78, 0.78, 0.80)),
    lk.box((0.024, 0.0012, 0.0004), (0.018, 0.0, 0.0362), color=(0.97, 0.97, 0.97)),
], mass=0.055, at=(0.0, 0.0, 0.0))
t.link("ring", [lk.cylinder(0.0054, TIP, (0.0, 0.0, TIP / 2), color=(0.55, 0.55, 0.58))], mass=0.004,
       at=(0.0, 0.0, Z_BOSS))
t.prismatic("seat", "case", "ring", anchor=(0.0, 0.0, Z_BOSS), axis=(0, 0, -1), limits=(0.0, 0.0001), rest=0.0,
            stiffness=SEAT_SPRING, damping=5.0, brake=lk.Brake(stiffness=60000.0, damping=50.0))
SERVO = t.build()

# ---- horn (free) -----------------------------------------------------------------------------------------------
shapes = []
for k in range(8):  # octagonal bore, inner apothem 3.4 mm, outer apothem 7.0 mm, 5.5 mm tall
    a = math.radians(45 * k)
    shapes.append(lk.box((0.0036, 0.006, 0.0055), (0.0052 * math.cos(a), 0.0052 * math.sin(a), 0.00275),
                         rpy_deg=(0, 0, 45 * k), color=(0.95, 0.95, 0.93)))
shapes.append(lk.cylinder(0.007, 0.0015, (0.0, 0.0, 0.00625), color=(0.95, 0.95, 0.93)))  # hub cap
shapes.append(lk.box((0.022, 0.005, 0.002), (0.017, 0.0, 0.001), color=(0.95, 0.95, 0.93)))  # arm, 6 to 28 mm
t = lk.Tree("horn", pos=(HORN_XY[0], HORN_XY[1], TOP), yaw_deg=HORN_YAW0, color=(0.95, 0.95, 0.93))
t.link("h", shapes, mass=0.004, friction=(0.4, 0.3), contact_offset=0.001)
HORN = t.build()

OBJECTS = [SERVO, HORN]


# ---- measurement ----------------------------------------------------------------------------------------------


def wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def measure(hp: np.ndarray, sp: np.ndarray, q_seat: float) -> dict:
    """The horn in the servo frame: bore offset from the shaft (mm), hub-bottom height over the boss (mm), tilt (deg),
    arm azimuth from the zero mark (deg), signed distance to the nearest spline index d (deg), its index k, and the
    seat ring's travel (mm)."""
    S = sp[:3, 3]
    p = hp[:3, 3]
    rel = sp[:3, :3].T @ hp[:3, :3]
    yaw = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    return {
        "off": 1000 * math.hypot(p[0] - S[0], p[1] - S[1]),
        "hz": 1000 * (p[2] - (S[2] + Z_BOSS)),
        "tilt": math.degrees(math.acos(float(np.clip(rel[2, 2], -1, 1)))),
        "yaw": yaw,
        "d": (yaw + PITCH / 2) % PITCH - PITCH / 2,
        "k": int(round(yaw / PITCH)),
        "seat": 1000 * q_seat,
    }


def aligned(m: dict) -> bool:
    """The bore is on the shaft and a tooth is over a groove: the seat ring may travel."""
    return m["off"] <= OFF_MAX_MM and m["tilt"] <= TILT_MAX and abs(m["d"]) <= WINDOW_DEG


def record(H: lk.History, m: dict) -> None:
    """One control step of the whole-run record. m = measure() plus held, h_speed (m/s), h_spin (deg/s), clear and
    tcp_speed per side."""
    H.set("last", m)
    on_tips = m["off"] <= 1.0 and m["tilt"] <= TILT_MAX and m["hz"] <= 2.5
    H.ever("carried", bool(m["held"]) and on_tips and abs(m["yaw"]) <= 3.0)
    H.held("still", m["h_speed"] < 0.002 and m["h_spin"] < 2.0)
    H.held("release", all(m["clear"][s] >= 0.030 and m["tcp_speed"][s] < 0.005 for s in ("left", "right")))


def ladder(H: lk.History) -> tuple[float, dict]:
    """Final state only: where the horn is seated at the end. A horn that was seated on another tooth earlier and has
    been pulled off and refitted is scored by where it sits now; nothing is zeroed for the earlier try."""
    v, m = H.values, H.values.get("last")
    if m is None:
        return 0.0, {}
    seated_now = m["seat"] >= SEAT_Q_MM
    on_mark = abs(m["yaw"]) <= WINDOW_DEG
    wrong_now = seated_now and not on_mark  # seated on another tooth at the end: the joint is still off
    seated_ok = seated_now and on_mark and m["off"] <= OFF_MAX_MM and m["tilt"] <= TILT_MAX
    end = seated_ok and m["hz"] <= 0.5 and m["tilt"] <= 2.0
    info = {"wrong_index_now": wrong_now, "seated_ok": seated_ok, "end": end, "still_s": round(H.runs.get("still", 0), 2),
            "release_s": round(H.runs.get("release", 0), 2)}
    if wrong_now:
        return 0.0, info
    if end and H.runs.get("still", 0) >= 1.0 and H.runs.get("release", 0) >= 1.0:
        return 1.0, info
    if end:
        return 0.75, info
    if seated_ok:
        return 0.5, info
    if "carried" in H.evers:
        return 0.25, info
    return 0.0, info


# ---- on_step ----------------------------------------------------------------------------------------------------

REC = lk.History()
LOG: dict = {}
ST = {"aligned": False, "prev": None}


@lk.on_reset
def _reset(tick):
    ST["aligned"] = False
    ST["prev"] = None
    LOG.clear()


@lk.on_step
def track(tick):
    s = tick.states
    hs, ss = s["horn"], s["servo"]
    hp, sp = hs.poses["h"], ss.poses["case"]
    m = measure(hp, sp, ss.q["seat"])
    prev, dt = ST["prev"], max(REC.dt, 1e-3)
    if prev is None:
        m["h_speed"], m["h_spin"] = 0.0, 0.0
    else:
        m["h_speed"] = float(np.linalg.norm(hp[:3, 3] - prev[:3, 3])) / dt
        c = float(np.clip((np.trace(prev[:3, :3].T @ hp[:3, :3]) - 1) / 2, -1, 1))
        m["h_spin"] = math.degrees(math.acos(c)) / dt
    ST["prev"] = hp.copy()
    tools = tick.tools
    m["held"] = any(tools[k].opening <= 0.030 and lk.clearance(tools[k], hs) <= 0.004 for k in tools)
    m["clear"] = {k: min(lk.clearance(tools[k], hs), lk.clearance(tools[k], ss)) for k in tools}
    m["tcp_speed"] = {k: tools[k].speed for k in tools}
    record(REC, m)
    al = aligned(m)
    if al != ST["aligned"] and not lk.braked("servo", "seat"):
        lk.set_limits("servo", "seat", 0.0, SEAT_TRAVEL if al else 0.0001)
        ST["aligned"] = al
    if m["seat"] >= SEAT_Q_MM and not lk.braked("servo", "seat"):
        lk.brake("servo", "seat", True)  # the interference fit holds the ring down while the horn sits on it
    elif lk.braked("servo", "seat") and m["hz"] > 0.8:  # the horn was pulled off the shaft: the fit lets go
        lk.brake("servo", "seat", False)
        lk.set_limits("servo", "seat", 0.0, 0.0001)
        ST["aligned"] = False
    if 20.0 < tick.t < 40.0:  # diagnostics: right pad pose relative to the horn origin (mm) and the opening
        tr = tools["right"]
        REC.trace("tool", tuple(round(1000 * float(x), 1) for x in (tr.tcp - hp[:3, 3])) + (round(1000 * tr.opening, 1),),
                  every=0.25)
    REC.trace("m", (round(m["off"], 2), round(m["hz"], 2), round(m["yaw"], 2), round(m["seat"], 2), int(m["held"])),
              every=0.5)


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


# ---- demo -------------------------------------------------------------------------------------------------------


def demo():
    return {}


def demo_report() -> dict:
    sc, info = ladder(REC)
    return {"score": sc, "info": info, "values": {k: v for k, v in REC.values.items() if k != "last"},
            "last": {k: (round(v, 3) if isinstance(v, float) else v) for k, v in REC.values.get("last", {}).items()
                     if k in ("off", "hz", "tilt", "yaw", "d", "k", "seat")},
            "trace": REC.traces.get("m", [])[-40:], "tool": REC.traces.get("tool", []), "log": LOG}


Z_TIPS = Z_BOSS + TIP


def _seen_horn(ctx):
    return ctx.observe().pose("horn", "h")


def _horn_yaw(hp):
    return math.degrees(math.atan2(hp[1, 0], hp[0, 0]))


def _fix(ctx, arm, target_xy, target_z, target_yaw, n=3, dur=1.0):
    """Closed loop on the seen horn pose: move the hand by the remaining error (camera-observable only)."""
    from linkdemo import rot_about

    for _ in range(n):
        hp = _seen_horn(ctx)
        err = np.array([target_xy[0] - hp[0, 3], target_xy[1] - hp[1, 3], target_z - hp[2, 3]])
        dy = wrap(target_yaw - _horn_yaw(hp))
        LOG.setdefault("fix", []).append([round(1000 * float(e), 2) for e in err] + [round(dy, 2)])
        yield from arm.move(arm.pos + err, rot_about([0, 0, 1], dy) @ arm.rot, duration=dur)


def _pass(ctx, arm, v, target_yaw, last):
    from linkdemo import DOWN, grip_frame

    hp = _seen_horn(ctx)
    c = hp[:3, 3] + np.array([0.0, 0.0, 0.0025])
    r = grip_frame(DOWN, hp[:3, :3] @ np.array([0.0, 1.0, 0.0]))
    d = 0.0115
    tcp = c + np.array([0.0, 0.0, d])
    yield from arm.move(tcp + [0, 0, 0.07], r, width=0.094, duration=2.0)
    yield from arm.move(tcp, r, duration=1.5)
    yield from arm.grip(0.0136, 0.8)
    yield from arm.wait(0.4)
    hover = TOP + Z_TIPS + 0.035
    yield from arm.move(arm.pos + [0, 0, 0.08], duration=1.5)
    sx, sy = SHAFT_XY
    yield from _fix(ctx, arm, (sx, sy), hover, target_yaw, n=1, dur=3.0)
    yield from _fix(ctx, arm, (sx, sy), hover, target_yaw, n=2, dur=1.0)
    land = TOP + Z_TIPS + 0.0004
    if v == "drop":
        yield from _fix(ctx, arm, (sx, sy), TOP + Z_TIPS + 0.030, target_yaw, n=2, dur=1.0)
        yield from arm.open(0.6)
        yield from arm.retreat(0.08)
        yield from arm.wait(4.0)
        return
    yield from _fix(ctx, arm, (sx, sy), TOP + Z_TIPS + 0.004, target_yaw, n=2, dur=1.5)
    yield from _fix(ctx, arm, (sx, sy), land, target_yaw, n=2, dur=1.0)
    LOG["on_tips_z_mm"] = round(1000 * (_seen_horn(ctx)[2, 3] - TOP - Z_BOSS), 2)
    if v == "wiggle":  # hold it, push down and turn the wrist back and forth until it drops in
        from linkdemo import rot_about

        yield from arm.move(arm.pos + [0, 0, -0.0025], duration=0.8)
        for deg in (-12.0, 24.0, -24.0):
            yield from arm.move(arm.pos, rot_about([0, 0, 1], deg) @ arm.rot, duration=2.0)
        yield from arm.wait(1.0)
    yield from arm.open(0.6)
    yield from arm.wait(0.4)
    yield from arm.move(arm.pos + [0, 0, 0.06], duration=1.2)
    from linkdemo import down
    from yam_arena.constants import PAD_TIP_PAST_TCP

    yield from arm.grip(0.0, 0.5)
    for attempt in range(2):
        hp = _seen_horn(ctx)
        LOG.setdefault("before_press", []).append(round(1000 * (hp[2, 3] - TOP - Z_BOSS), 2))
        if hp[2, 3] - TOP - Z_BOSS < 0.0004:
            break
        ox, oy, top_z = hp[0, 3], hp[1, 3], hp[2, 3] + HUB_H
        yield from arm.move(np.array([ox, oy, top_z + PAD_TIP_PAST_TCP + 0.05]), down(0.0), duration=2.0)
        for h in (0.04, 0.012):  # centre the closed pads over the hub from the tool point the camera sees
            yield from arm.move(np.array([arm.pos[0], arm.pos[1], top_z + PAD_TIP_PAST_TCP + h]), duration=1.0)
            for _ in range(2):
                e = np.array([ox, oy]) - ctx.observe().tools["right"].tcp[:2]
                LOG.setdefault("centre_mm", []).append([round(1000 * float(x), 2) for x in e])
                yield from arm.move(arm.pos + [e[0], e[1], 0.0], duration=0.6)
        yield from arm.move(arm.pos + [0.0, 0.0, -(0.012 + 0.010 + 0.004 * attempt)], duration=2.0)
        yield from arm.wait(1.0)
        yield from arm.move(arm.pos + [0.0, 0.0, 0.08], duration=1.0)
    yield from arm.wait(0.3)
    LOG["after_press_hz_mm"] = round(1000 * (_seen_horn(ctx)[2, 3] - TOP - Z_BOSS), 2)
    yield from arm.wait(3.5 if last else 0.5)


def demo_script(ctx):
    v = VARIANT
    LOG["variant"] = v
    yield from ctx.wait(1.0)
    if v == "nothing":
        yield from ctx.wait(8.0)
        return
    arm = ctx.right
    target = {"": 0.0, "off_by_one": PITCH, "as_picked": HORN_YAW0, "wiggle": HORN_YAW0, "drop": 0.0}[v]
    yield from _pass(ctx, arm, v, target, True)
