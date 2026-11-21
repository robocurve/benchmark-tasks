"""Polish a glassy-carbon disk electrode in a figure-8 with its face flat, then hang it with only the tip in the rinse.

On the station table: a Pine E5-series glassy-carbon disk electrode tip standing face-down on a lab wipe; a microcloth
polishing pad on a glass plate taped to a load-cell platform, with a 3 mm spot of 0.05 um alumina at its centre; a
crystallizing dish of DI water; a ring stand whose spring clamp hangs over the dish.

- The maker's guide (Pine DRK10013): keep the electrode surface parallel to the cloth, polish in a figure-8, use about a
  3 mm spot of slurry, and suspend only the electrode surface in the rinse with a ring stand and clamp.
- Scored from the load cell (force), the tip's tilt, and the face centre's track: lobes out of and back into a 10 mm
  circle round the spot, each a loop 10 mm wide that turns the other way from the last and lies on the other side.

The tip, platform, dish and stand are free rigid bodies; the platform's top rides a stiff spring (the load cell), the
clamp's two jaws are sprung hinges. Water and alumina are not simulated: the rinse is the face's depth under a fixed
water line. Nothing is moved, latched or held by code.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1.0 state already includes 1 s released and still
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Polish the electrode's face on the alumina spot in figure-8 strokes, keeping the face flat on the pad "
               "with light pressure, then hang it in the stand's clamp with only its tip in the rinse water and let go.")
HZ, DT = 20.0, 0.05
_DEBUG = bool(os.environ.get("TIP_DEBUG"))
# One-factor robustness runs only (README): TASK_SWEEP='{"friction": 1.2}' scales a property; unset in every trial.
SWEEP = json.loads(os.environ.get("TASK_SWEEP", "") or "{}")
GAP = 0.0005  # spawn gap over the table

PTFE, GC, STEEL, BLACK = (0.95, 0.95, 0.93), (0.08, 0.08, 0.09), (0.70, 0.71, 0.73), (0.05, 0.05, 0.05)
CLOTH, GLASS, ALU, WIPE_C = (0.55, 0.20, 0.25), (0.80, 0.88, 0.90), (0.62, 0.64, 0.67), (0.98, 0.98, 0.98)
WATER, RUBBER, IRON, SLURRY = (0.70, 0.85, 0.98), (0.15, 0.15, 0.15), (0.25, 0.27, 0.30), (0.92, 0.92, 0.88)

# == the electrode tip (frame origin at the face centre, z up the tip's axis) ==========================================
TIP_D, SHROUD_L, TIP_L, STEM_D, DISK_D = 0.015, 0.0254, 0.049, 0.010, 0.005
TIP_KG = 0.025  # estimate: weigh the real tip
STEM_GRIP = 0.009  # the stem's collider across its flats (the pads bear on two lines of the 10 mm round)
RING_LO, RING_HI = 0.003, 0.010  # painted rings on the shroud, above the face
TIP_MU = 0.15  # PTFE on wet microcloth (estimate)
WIPE_XY = (0.13, -0.05)
WIPE = (0.060, 0.060, 0.001)


def _ring_z(r: float, z0: float, z1: float, color, n: int, t: float = 0.002, ghost: bool = False) -> list:
    """A round part about z as ``n`` flat plates (outer radius ``r``)."""
    w = 2 * r * math.tan(math.pi / n) + 0.0004
    return [k.box(((r - t / 2) * math.cos(a), (r - t / 2) * math.sin(a), (z0 + z1) / 2), (t, w, z1 - z0), color,
                  turn=(0.0, 0.0, a), ghost=ghost) for a in (2 * math.pi * i / n for i in range(n))]


def _tip_boxes() -> list:
    out = _ring_z(TIP_D / 2, 0.0, SHROUD_L, PTFE, 16, ghost=True)  # the round shroud as drawn
    out.append(k.box((0.0, 0.0, SHROUD_L / 2), (TIP_D, TIP_D, SHROUD_L), PTFE, hidden=True))  # its collider
    out.append(k.box((0.0, 0.0, 0.0002), (DISK_D * 0.89, DISK_D * 0.89, 0.0004), GC, ghost=True))  # the disk
    out += _ring_z(TIP_D / 2 + 0.0002, RING_LO - 0.0005, RING_LO + 0.0005, BLACK, 16, t=0.0004, ghost=True)
    out += _ring_z(TIP_D / 2 + 0.0002, RING_HI - 0.0005, RING_HI + 0.0005, BLACK, 16, t=0.0004, ghost=True)
    out += _ring_z(STEM_D / 2, SHROUD_L, TIP_L, STEEL, 12, ghost=True)  # the round stem as drawn
    out.append(k.box((0.0, 0.0, (SHROUD_L + TIP_L) / 2), (STEM_GRIP, STEM_GRIP, TIP_L - SHROUD_L), STEEL,
                     hidden=True))  # its collider: flats for the pads
    return out


TIP_BOXES = _tip_boxes()
TIP_START = (WIPE_XY[0], WIPE_XY[1], TOP + WIPE[2] + 2 * GAP)
TIP = k.mech("tip", TIP_START, TIP_BOXES, TIP_KG * SWEEP.get("mass", 1.0), color=PTFE,
              surface=TIP_MU * SWEEP.get("friction", 1.0))
WIPE_M = k.mech("wipe", (WIPE_XY[0], WIPE_XY[1], TOP + GAP), [k.box((0, 0, WIPE[2] / 2), WIPE, WIPE_C)], 0.005,
                color=WIPE_C, surface=0.5)

# == the load-cell platform with the plate and pad ======================================================================
SPOT = (0.25, 0.04)  # the slurry spot, the pad's centre
PLAT = (0.160, 0.160)
CELL_BASE_H = 0.032
PAD_Z = 0.047  # pad surface above the table
CELL_RATE = 10000.0  # N/m: the load cell as a stiff spring
PAD_MU = 0.15  # wet microcloth (estimate)
CELL_TOP = k.slider("top", (0.0, 0.0, CELL_BASE_H), "Z", -0.003, 0.001, [
    k.box((0.0, 0.0, 0.0045), (PLAT[0], PLAT[1], 0.007), ALU),
    k.box((0.0, 0.0, 0.011), (0.150, 0.150, 0.006), GLASS),
    k.box((0.0, 0.0, 0.0145), (0.150, 0.150, 0.001), CLOTH),
    k.box((0.0, 0.0, 0.01505), (0.003, 0.003, 0.0001), SLURRY, ghost=True)],
    rate=CELL_RATE, target=0.0, damping=60.0, mass=0.4, weightless=True)
CELL = k.mech("cell", (SPOT[0], SPOT[1], TOP + GAP), [k.box((0.0, 0.0, CELL_BASE_H / 2), (PLAT[0], PLAT[1], CELL_BASE_H),
                                                           IRON)], 1.5, joints=[CELL_TOP], color=IRON,
               surface=PAD_MU * SWEEP.get("friction", 1.0))

# == the rinse dish ======================================================================================================
DISH_XY = (0.22, -0.16)
DISH_R, DISH_H, DISH_FLOOR = 0.050, 0.050, 0.003
WATER_Z = 0.043  # the water line above the table (40 mm of water)


def _dish_boxes() -> list:
    out = _ring_z(DISH_R, 0.0, DISH_H, GLASS, 16, t=0.002)
    out.append(k.box((0.0, 0.0, DISH_FLOOR / 2), (0.070, 0.070, DISH_FLOOR), GLASS))
    out += [k.box((0.0, 0.0, (DISH_FLOOR + WATER_Z) / 2), (2 * (DISH_R - 0.002), 0.0186, WATER_Z - DISH_FLOOR), WATER,
                  turn=(0.0, 0.0, math.pi * i / 16), ghost=True) for i in range(16)]
    return out


DISH = k.mech("dish", (DISH_XY[0], DISH_XY[1], TOP + GAP), _dish_boxes(), 0.6, color=GLASS, surface=0.4)

# == the ring stand and its spring clamp ==================================================================================
# The clamp: two rubber-lined jaws on short slides, sprung shut to a 14 mm gap (the 15 mm shroud opens each 0.5 mm),
# flared at the top, over the dish centre: the tip is lowered into it from above and hangs where it is left, its face
# HANG_DEPTH under the water line.
HANG_DEPTH = 0.0065
HANG = (DISH_XY[0], DISH_XY[1])
FACE_HANG_Z = WATER_Z - HANG_DEPTH
JAW_Z = (FACE_HANG_Z + 0.0105, FACE_HANG_Z + 0.0165)  # jaw pads on the shroud, above its 10 mm ring
JAW_GAP = 0.014
JAW_T = 0.003
JAW_L = 0.020  # the pads' length along x
JAW_RATE, JAW_TRAVEL = 1500.0, 0.006  # N/m on each jaw (0.75 N at the 0.5 mm a 15 mm shroud opens it), m
FLARE_DEG = 20.0  # the mouth's lead-in, from vertical
CLAMP_MU = 0.6
BASE_XY = (0.375, -0.16)
BASE = (0.150, 0.180, 0.012)
ROD_X = 0.33
ARM_Z = (0.060, 0.066)
LINK_X = HANG[0] + 0.030  # the clamp frame post, clear of the fingers (their hull is about 40 mm across x)


def _local(x, y, z):
    return (x - BASE_XY[0], y - BASE_XY[1], z)


def _jaw(side: int) -> k.Joint:
    """The +y (side 1) or -y (side -1) jaw: a vertical pad with a flare above it; slides away from the tip."""
    s = float(side)
    zc, zh = (JAW_Z[0] + JAW_Z[1]) / 2, JAW_Z[1] - JAW_Z[0]
    yh = s * (JAW_GAP / 2 + JAW_T / 2)
    phi, ln, ov = math.radians(FLARE_DEG), 0.007, 0.0012
    # the flare's inner face meets the pad's top inner corner and runs 1.2 mm past it (0.4 mm inside the pad's face),
    # so the jaw is already open past that corner when the shroud's edge reaches it
    qy, qz = -s * JAW_T / 2, zh / 2
    cy = qy + s * (JAW_T / 2 * math.cos(phi) + (ln / 2 - ov) * math.sin(phi))
    cz = qz - JAW_T / 2 * math.sin(phi) + (ln / 2 - ov) * math.cos(phi)
    boxes = [k.box((0.0, 0.0, 0.0), (JAW_L, JAW_T, zh), RUBBER),
             k.box((0.0, cy, cz), (JAW_L, JAW_T, ln), RUBBER, turn=(-s * phi, 0.0, 0.0))]
    lo, hi = (0.0, JAW_TRAVEL) if side > 0 else (-JAW_TRAVEL, 0.0)
    return k.slider(f"jaw{'L' if side > 0 else 'R'}", _local(HANG[0], HANG[1] + yh, zc), "Y", lo, hi, boxes,
                    rate=JAW_RATE * SWEEP.get("spring", 1.0), target=0.0, damping=2.0, mass=0.02, weightless=True)


def _stand_boxes() -> list:
    return [
        k.box((0.0, 0.0, BASE[2] / 2), BASE, IRON),
        k.box(_local(ROD_X, BASE_XY[1], 0.130), (0.012, 0.012, 0.236), STEEL),
        k.box(_local((LINK_X + ROD_X) / 2, BASE_XY[1], sum(ARM_Z) / 2), (ROD_X - LINK_X, 0.016, ARM_Z[1] - ARM_Z[0]),
              STEEL),
        k.box(_local(LINK_X + 0.003, HANG[1], (JAW_Z[0] + ARM_Z[1]) / 2), (0.006, 0.034, ARM_Z[1] - JAW_Z[0]), STEEL),
        *[k.box(_local((HANG[0] + JAW_L / 2 + LINK_X) / 2, HANG[1] + s * 0.0135, (JAW_Z[0] + JAW_Z[1]) / 2),
                (LINK_X - HANG[0] - JAW_L / 2, 0.004, 0.004), STEEL) for s in (-1, 1)],
    ]


STAND = k.mech("stand", (BASE_XY[0], BASE_XY[1], TOP + GAP), _stand_boxes(), 2.0, joints=[_jaw(1), _jaw(-1)],
               color=IRON, surface=CLAMP_MU * SWEEP.get("friction", 1.0))
OBJECTS = [CELL, DISH, STAND, WIPE_M, TIP]

# == scoring ===============================================================================================================
F_LO, F_HI, F_MAX = 1.0, 5.0, 8.0  # N
TILT_FLAT, TILT_MAX = 3.0, 6.0  # deg
CROSS_R, LOBE_REACH, LOBE_WIDE = 0.010, 0.020, 0.010  # m
IN_BAND_FRAC = 0.90
FIG8_FULL = 5
DIP_LO, DIP_HI = 0.003, 0.010  # m under the water line
RINSE_S = 2.0
CLEAR = 0.020  # m: grippers this far from the tip
STILL_S, STILL_TOL = 1.0, 0.001
DROP = 0.020  # m fallen while held by nothing
PAD_HALF = 0.075
DROPPED, STRUCK, DUNKED = k.Sticky(), k.Sticky(), k.Sticky()
END = k.Settle(STILL_S, STILL_TOL)
RUN: dict = {}
REC = k.run_record.RunRecord(__file__, "cell", "stand")
TIP_SURF = [b for b in TIP_BOXES if not b[3].ghost]


def pad_z_world(states) -> float:
    return float(states["cell"].to_world((0.0, 0.0, 0.015), "top")[2])


def force(states) -> float:
    return CELL_RATE * max(0.0, -float(states["cell"].q["top"]))


def face(states) -> np.ndarray:
    return np.asarray(states["tip"].to_world((0.0, 0.0, 0.0)), float)


def depth(states) -> float:
    """How far the face centre is under the water line, over the dish (-1 m anywhere else)."""
    f = face(states)
    if math.hypot(f[0] - DISH_XY[0], f[1] - DISH_XY[1]) > DISH_R - 0.002:
        return -1.0
    return (TOP + GAP + WATER_Z) - float(f[2])


def _hull_points(grips) -> np.ndarray:
    pts = [np.asarray(h.vertices) for hulls in grips for _, h in hulls if getattr(h, "vertices", None) is not None]
    return np.concatenate(pts) if pts else np.zeros((0, 3))


def _contacts(states, grips) -> tuple[bool, bool]:
    """(another thing touches the pad, a finger is below the tip's 10 mm ring near the tip)."""
    v = _hull_points(grips)
    if not len(v):
        return False, False
    pz = pad_z_world(states)
    on = (np.abs(v[:, 0] - SPOT[0]) < PAD_HALF) & (np.abs(v[:, 1] - SPOT[1]) < PAD_HALF) & (v[:, 2] < pz + 0.003)
    f = face(states)
    near = np.hypot(v[:, 0] - f[0], v[:, 1] - f[1]) < 0.020
    low = near & (v[:, 2] < f[2] + RING_HI)
    return bool(on.any()), bool(low.any())


def _lobe_stats(pts: np.ndarray) -> dict:
    s = np.asarray(SPOT)
    d = pts - s
    r = np.hypot(d[:, 0], d[:, 1])
    far = d[int(np.argmax(r))]
    u = far / max(1e-9, np.linalg.norm(far))
    n = np.array([-u[1], u[0]])
    wide = float((d @ n).max() - (d @ n).min())
    poly = np.vstack([[0.0, 0.0], d, [0.0, 0.0]])
    area = 0.5 * float(np.sum(poly[:-1, 0] * poly[1:, 1] - poly[1:, 0] * poly[:-1, 1]))
    return {"reach": float(r.max()), "wide": wide, "turn": 1 if area > 0 else -1, "far": far}


def _close_lobe(rows) -> None:
    pts = np.array([r[0] for r in rows])
    st = _lobe_stats(pts)
    good = np.mean([r[1] and r[2] for r in rows])
    st["valid"] = (st["reach"] >= LOBE_REACH and st["wide"] >= LOBE_WIDE and good >= IN_BAND_FRAC
                   and max(r[3] for r in rows) <= TILT_MAX and max(r[4] for r in rows) <= F_MAX
                   and not any(r[5] for r in rows))
    st["good"] = float(good)
    RUN["lobes"].append(st)
    RUN["t_last_lobe"] = RUN["t"]


def figure8s() -> float:
    lobes = RUN.get("lobes", [])
    if not lobes:
        return 0.0
    g = max(lobes, key=lambda L: np.linalg.norm(L["far"]))["far"]
    best = run = 0
    prev = None
    for L in lobes:
        side = 1 if float(np.dot(L["far"], g)) > 0 else -1
        if not L["valid"]:
            run, prev = 0, None
            continue
        run = run + 1 if prev is not None and side != prev[0] and L["turn"] != prev[1] else 1
        prev = (side, L["turn"])
        best = max(best, run)
    return best / 2.0


def valid_lobes() -> int:
    return sum(1 for L in RUN.get("lobes", []) if L["valid"])


def hanging(states) -> bool:
    f = face(states)
    in_clamp = abs(f[1] - HANG[1]) < 0.004 and abs(f[0] - HANG[0]) < JAW_L / 2  # centred between the jaws
    return in_clamp and DIP_LO <= depth(states) <= DIP_HI and END.ok


def grade(states) -> float:
    if DROPPED or STRUCK:
        return 0.0
    full = figure8s() >= FIG8_FULL
    rinsed = RUN.get("rinsed", False) and not DUNKED
    return k.ladder((0.25, valid_lobes() >= 2), (0.5, full), (0.75, full and rinsed),
                    (1.0, full and rinsed and hanging(states)))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return float(max(0.0, FIG8_FULL - figure8s()))


@k.on_reset
def _reset(states) -> None:
    DROPPED.reset(), STRUCK.reset(), DUNKED.reset(), END.reset()
    RUN.clear()
    RUN.update(t=0.0, lobes=[], cur=None, inside=False, t_last_lobe=-1.0, last_pad=-1.0, dip_s=0.0, rinsed=False,
               z_ref=float(face(states)[2]), f=0.0)
    REC.reset()


@k.on_step
def _watch(states) -> None:
    RUN["t"] += DT
    t = RUN["t"]
    grips = k.score.grippers()
    st = states["tip"]
    f = face(states)
    tilt = st.tilt_deg()
    F = force(states)
    RUN["f"] = F
    other, low = _contacts(states, grips)
    pz = pad_z_world(states)
    on_pad = (abs(f[0] - SPOT[0]) < PAD_HALF and abs(f[1] - SPOT[1]) < PAD_HALF and f[2] - pz < 0.002)
    if on_pad:
        RUN["last_pad"] = t
    band = on_pad and F_LO <= F <= F_HI and not other
    flat = tilt <= TILT_FLAT
    # lobes: the face centre's track out of the crossing circle and back
    inside = math.hypot(f[0] - SPOT[0], f[1] - SPOT[1]) < CROSS_R
    if RUN["inside"] and not inside:
        RUN["cur"] = []
    if RUN["cur"] is not None and not inside:
        RUN["cur"].append(((f[0], f[1]), band, flat, tilt, F, low))
    if inside and RUN["cur"] is not None:
        if RUN["cur"]:
            _close_lobe(RUN["cur"])
        RUN["cur"] = None
    RUN["inside"] = inside
    # rinse: after the last lobe, no pad contact since, the face 3-10 mm under for 2 s in one dip
    dp = depth(states)
    if dp > DIP_HI:
        DUNKED.trip("the 10 mm ring went under water")
    after = RUN["t_last_lobe"] >= 0 and not on_pad and RUN["last_pad"] < t - DT / 2
    if after and DIP_LO <= dp <= DIP_HI:
        RUN["dip_s"] += DT
        if RUN["dip_s"] >= RINSE_S - 1e-9:
            RUN["rinsed"] = True
    else:
        RUN["dip_s"] = 0.0
    if on_pad and RUN["rinsed"]:
        RUN["rinsed"] = False  # polishing again after the rinse undoes it
    # drop: held by nothing and falling
    held = bool(grips) and k.score.clearance(k.score.surface_points(st, "", TIP_SURF, 0.006), grips, beyond=0.01) < 0.004
    if held or not grips:
        RUN["z_ref"] = float(f[2])
    elif float(f[2]) < RUN["z_ref"] - DROP:
        DROPPED.trip("the tip fell")
    # face strikes: the bench outside the wipe, the dish floor
    on_wipe = abs(f[0] - WIPE_XY[0]) < WIPE[0] / 2 + 0.004 and abs(f[1] - WIPE_XY[1]) < WIPE[1] / 2 + 0.004
    if f[2] < TOP + 0.0015 and not on_wipe:
        STRUCK.trip("face on the bench")
    if math.hypot(f[0] - DISH_XY[0], f[1] - DISH_XY[1]) < DISH_R and f[2] < TOP + GAP + DISH_FLOOR + 0.0015:
        STRUCK.trip("face on the dish floor")
    surf = k.score.surface_points(st, "", TIP_SURF, 0.008)
    END.push(np.array([st.to_world(c) for c in ((0, 0, 0), (0, 0, TIP_L))]).ravel(),
             k.hands_clear(surf, CLEAR * 1000) and not on_pad)
    if _DEBUG and abs(t / 0.5 - round(t / 0.5)) < 1e-6:
        R = st.rotation()
        print(f"DBG t {t:.1f} yaw {math.degrees(math.atan2(R[1, 0], R[0, 0])):.1f} F {F:.2f} tilt {tilt:.1f} face-pad {1000 * (f[2] - pz):.1f} mm face {1000 * (f[0] - SPOT[0]):+.0f},"
              f"{1000 * (f[1] - SPOT[1]):+.0f} lobes {len(RUN['lobes'])}/{valid_lobes()} depth {1000 * dp:.1f}", flush=True)
    REC.tick(states, lambda: f"F {F:.2f} N tilt {tilt:.1f} face {1000 * (f[0] - SPOT[0]):+.0f},{1000 * (f[1] - SPOT[1]):+.0f} "
                             f"mm lobes {len(RUN['lobes'])}/{valid_lobes()} fig8 {figure8s():.1f} depth "
                             f"{1000 * dp:.1f} rinsed {RUN['rinsed']} score {grade(states):g}")


# == the demo: one hand on the stem ========================================================================================
GRIP = D.Grip.deg(yaw=0)  # straight down, jaws closing along y
TCP_OVER_FACE = 0.030 + D.P.PAD_TIP  # pad tips 30 mm above the face: 19 mm of stem between the pads
SQUEEZE = 0.0025
F_TARGET = 2.5
K_V = 450.0  # N/m: the arm's vertical compliance
GERONO_A, PERIOD, PERIODS = 0.030, 5.0, 6
DIP_CMD = 0.0055  # face servoed to 5.5 mm under: it settles 1-2 mm deeper in the jaws after release


def tcp_for_face(fx, fy, fz) -> np.ndarray:
    return np.array([fx, fy, fz + TCP_OVER_FACE])


def hand(how: str = "demo") -> D.Plan:
    p = D.Plan("right")
    start = np.array([TIP_START[0], TIP_START[1], TIP_START[2]])
    pad0 = TOP + GAP + PAD_Z
    p.move(2.0, lambda st: tcp_for_face(*face(st)) + (0, 0, 0.060), finger=D.gap_finger(STEM_D + 0.020), grip=GRIP,
           label="over the stem", fallback=tcp_for_face(*start) + (0, 0, 0.060))
    p.move(1.5, lambda st: tcp_for_face(*face(st)), label="down round the stem", fallback=tcp_for_face(*start))
    p.fingers(STEM_GRIP - 2 * SQUEEZE, D.P.T_CLOSE, "grip the stem").hold(0.3, "settle")
    p.move(1.5, lambda st: tcp_for_face(*face(st)) + (0, 0, 0.060), label="lift", fallback=tcp_for_face(*start) + (0, 0, 0.06))
    over = tcp_for_face(SPOT[0], SPOT[1], pad0 + 0.010)
    p.move(2.0, tuple(over), label="over the spot")
    p.memo["d"] = 0.0  # m commanded past contact, raised by the force servo
    p.move(2.0, tuple(tcp_for_face(SPOT[0], SPOT[1], pad0 + 0.001)), label="face down to the pad")

    def path(i: int):
        tt = 2 * math.pi * i / (PERIOD * HZ)
        if how == "strokes":
            return GERONO_A * math.sin(tt), 0.0
        if how == "circles":
            return GERONO_A / 2 * (1 - math.cos(tt)), GERONO_A / 2 * math.sin(tt)
        return GERONO_A * math.sin(tt), GERONO_A * math.sin(tt) * math.cos(tt)

    def pressed(i: int):
        def fn(st):
            F = force(st)
            pz = pad_z_world(st)
            if face(st)[2] - pz < 0.003:  # in contact: servo the force
                p.memo["d"] = float(np.clip(p.memo["d"] + 0.3 * (F_TARGET - F) / K_V, 0.0, 0.020))
            dx, dy = path(i)
            return tcp_for_face(SPOT[0] + dx, SPOT[1] + dy, pz - p.memo["d"])
        return fn

    n_down = int(2.5 * HZ)
    for i in range(n_down):  # down onto the spot and settle the force
        p.move(DT, pressed(0), track=True, label="press onto the spot", fallback=tuple(over - (0, 0, 0.012)))
    n = int(PERIODS * PERIOD * HZ)
    for i in range(1, n + 1):
        p.move(DT, pressed(i), track=True, label="figure-8", fallback=tuple(over - (0, 0, 0.012)))
    lift = tuple(tcp_for_face(SPOT[0], SPOT[1], pad0 + 0.030))
    p.move(1.5, lift, label="lift off the pad")
    p.hold(0.5, "steady")

    def measure(st):  # the hand-to-face offset now (any creep of the tip in the grip is taken out below)
        p.memo["off"] = np.asarray(lift, float) - face(st)
        return lift

    p.move(0.2, measure, label="read where the face hangs", fallback=lift)

    def at_face(fx, fy, fz):
        dflt = np.array([0.0, 0.0, TCP_OVER_FACE])
        return lambda st: np.array([fx, fy, fz]) + p.memo.get("off", dflt)
    hi = TOP + GAP + JAW_Z[1] + 0.025  # the face over the clamp's mouth
    deep = DIP_CMD if how != "dunk" else 0.014
    face_dip = TOP + GAP + WATER_Z - deep
    p.move(2.5, at_face(HANG[0], HANG[1], hi), label="over the clamp", fallback=tuple(tcp_for_face(HANG[0], HANG[1], hi)))
    face_hi = hi

    def servo(i: int, n: int, z0: float, z1: float):
        """A face servo: the hand moves by 0.4 x (where the face should be - where it is) each step, so the dip depth
        is the face's own, whatever the grip has crept or the jaws hold back."""
        def fn(st):
            u = min(1.0, i / max(1, n))
            want = np.array([HANG[0], HANG[1], z0 + (z1 - z0) * (3 * u * u - 2 * u ** 3)])
            cmd = np.asarray(p.memo.get("cmd", tcp_for_face(*want)), float)
            cmd = cmd + np.clip(0.4 * (want - face(st)), -0.002, 0.002)
            p.memo["cmd"] = cmd
            return cmd
        return fn

    def start_cmd(st):
        p.memo["cmd"] = np.asarray(at_face(HANG[0], HANG[1], face_hi)(st), float)
        return p.memo["cmd"]

    p.move(0.1, start_cmd, label="servo start", fallback=tuple(tcp_for_face(HANG[0], HANG[1], hi)))
    n_down = int(3.5 * HZ)
    for i in range(1, n_down + 1):
        p.move(DT, servo(i, n_down, face_hi, face_dip), track=True, label="down through the jaws into the water",
               fallback=tuple(tcp_for_face(HANG[0], HANG[1], face_dip)))
    for i in range(int(2.5 * HZ)):
        p.move(DT, servo(1, 1, face_dip, face_dip), track=True, label="rinse",
               fallback=tuple(tcp_for_face(HANG[0], HANG[1], face_dip)))
    if how == "dunk":
        up = TOP + GAP + WATER_Z - DIP_CMD
        for i in range(1, 31):
            p.move(DT, servo(i, 30, face_dip, up), track=True, label="back up",
                   fallback=tuple(tcp_for_face(HANG[0], HANG[1], up)))
        face_dip = up
    p.hold(0.5, "settle in the jaws")
    if how == "held":
        return p.hold(6.0, "keep holding")
    p.release()
    p.move(1.5, lambda st: np.asarray(p.memo.get("cmd", tcp_for_face(HANG[0], HANG[1], face_dip))) + (0, 0, 0.060),
           label="up off the stem", fallback=tuple(tcp_for_face(HANG[0], HANG[1], face_dip) + (0, 0, 0.06)))
    return p.home(hold=3.0)


def demo():
    def two(**kw):
        return D.together(D.idle_plan("left", 0.1), hand(**kw))

    return D.choose(demo=lambda: two(), strokes=lambda: two(how="strokes"), circles=lambda: two(how="circles"),
                    held=lambda: two(how="held"), dunk=lambda: two(how="dunk"))


# == what the template tests read ===========================================================================================
TEXT_NUMBERS = [("15 mm", TIP_D), ("25.4 mm", SHROUD_L), ("49 mm", TIP_L), ("10 mm across", STEM_D),
                ("25 g", TIP_KG), ("4.7 cm", PAD_Z), ("10,000 N/m", CELL_RATE), ("5.0 N", F_HI),
                ("8 N", F_MAX), ("10 mm radius", CROSS_R), ("20 mm or more", LOBE_REACH), ("10 mm or more wide", LOBE_WIDE),
                ("3 mm and 10 mm", DIP_LO), ("4.3 cm", WATER_Z), ("2 cm or more", CLEAR), ("under 1 mm", STILL_TOL),
                ("20 mm or more held by nothing", DROP)]
CHEATS = {"none": 0.0, "strokes": 0.0, "circles": 0.25, "held": 0.75, "dunk": 0.5}  # mode: the most it may score
HOLD = {"cell.top": 0.5}
