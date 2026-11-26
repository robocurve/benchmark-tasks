"""Refit a sprung bed slat that has popped out of its holders.

A replacement sprung (bowed) beech bed slat, 53 x 8 mm, 755 mm long over its plastic end caps (the size sold for
small single and toddler beds, e.g. Green Design 53 mm x 755 mm sprung beech slats, Uncle Bob 755 x 53 x 8.5 mm),
lies crown up on the bench in front of a bed-frame section. The frame's two side rails carry plastic slat holders:
a pocket per slat end, open at the top and inboard, closed by an end wall whose top carries a 4 mm retaining lip.
The slat is a little longer than the holders' wall-to-wall span, so it sits sprung against both end walls, its caps
under the lips. Two neighbouring slats are already seated.

The job, as anyone who has had a slat pop out knows: push one end into its holder under the lip, then bow the slat
(lift its middle while holding the other end down) so its chord shortens enough for the second cap to pass inside
the lip, lower that cap onto the holder floor and let the slat spring straight so both caps seat under their lips.
Bend it too far and a beech slat cracks.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import _ENV, Slider, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Refit the loose bed slat in the empty pair of holders between the two fitted slats, crown up, both end "
               "caps seated under the holders' lips, then let go.")

BEECH, CAPC, PINE, HOLDC = (0.80, 0.60, 0.40), (0.45, 0.45, 0.47), (0.86, 0.71, 0.50), (0.06, 0.06, 0.07)

# ---- the slat (part frame: origin at the centre of the middle segment's wood, +y along the slat, z up) -------------
W, TH = 0.053, 0.008            # beech section 53 x 8 mm
CAP_L, CAP_W, CAP_T = 0.045, 0.058, 0.013   # plastic end cap sleeve: 45 mm long, 58 x 13 mm outside
S_ALL = 0.755                   # over both caps, along the slat
L_MID = 0.4 / 3                 # three middle segments of 133 mm
L_END = (S_ALL - 3 * L_MID) / 2  # two end segments of 177.5 mm (each carries its cap)
A = L_MID / 2                   # inner hinges at +-66.7 mm along the slat
RISE0 = 0.020                   # rest bow: mid-span 20 mm above the tips (centre line)
RATE = 25.0                     # N m/rad per hinge (see fidelity)
DAMP = 0.15
MASS = 0.25                     # 0.227 kg of beech (720 kg/m^3) plus two 10 g caps
SEG = ("s0", "s1", "body", "s3", "s4")   # along +y
LENS = (L_END, L_MID, L_MID, L_MID, L_END)


def shape(phi):
    """Hinge points and segment slopes of a chain bowed with angle ``phi`` at each of its four hinges: the middle
    segment level, the next ones sloping down by phi, the end ones by 2 phi. Returns (points along y from the -y tip to
    the +y tip as (y, z), slopes of the five segments)."""
    slopes = (2 * phi, phi, 0.0, -phi, -2 * phi)   # dz/dy angle of each segment, walking +y
    pts = [np.array([-A, 0.0])]
    for L, sl in zip((L_MID, L_END), (phi, 2 * phi)):        # walk toward -y: going -y the chain drops
        p = pts[0]
        pts.insert(0, p + np.array([-L * math.cos(sl), -L * math.sin(sl)]))
    pts.append(np.array([A, 0.0]))
    for L, sl in zip((L_MID, L_END), (phi, 2 * phi)):
        p = pts[-1]
        pts.append(p + np.array([L * math.cos(sl), -L * math.sin(sl)]))
    return pts, slopes


def _phi_for(fn, target, lo=0.0, hi=0.3):
    for _ in range(80):
        m = (lo + hi) / 2
        if fn(m) < target:
            lo = m
        else:
            hi = m
    return (lo + hi) / 2


PHI0 = _phi_for(lambda p: -shape(p)[0][0][1], RISE0)          # rest hinge angle (rise 20 mm)
_P0, _SL0 = shape(PHI0)
CHORD0 = _P0[-1][0] - _P0[0][0]                                 # rest tip-to-tip chord


def seg_boxes(pts, slopes, i, with_cap=True):
    """Wood (and cap) boxes of segment i in the part frame."""
    a, b = pts[i], pts[i + 1]
    c = (a + b) / 2
    L = float(np.linalg.norm(b - a))
    ang = -math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) * -1.0   # rotation about +x turning +y onto a->b
    out = [((0.0, float(c[0]), float(c[1])), (W, L + 0.002, TH), BEECH, ("X", ang), ("max", 1.0))]
    if with_cap and i in (0, 4):
        d = (b - a) / L
        tip = a if i == 0 else b
        cc = tip + (d if i == 0 else -d) * (CAP_L / 2)
        out.append(((0.0, float(cc[0]), float(cc[1])), (CAP_W, CAP_L, CAP_T), CAPC, ("X", ang), ("max", 0.7)))
    return out


def _segment_specs(pts, slopes):
    return [seg_boxes(pts, slopes, i) for i in range(5)]


_SPECS0 = _segment_specs(_P0, _SL0)
_mass = [MASS * 0.92 * L / S_ALL + (0.01 if i in (0, 4) else 0.0) for i, L in enumerate(LENS)]
# hinge points: s1 hangs from the body at the -y inner hinge (pts[2]), s0 from s1 at pts[1]; s3 at pts[3], s4 at pts[4]
_HINGE_PT = {"s1": _P0[2], "s0": _P0[1], "s3": _P0[3], "s4": _P0[4]}
_PARENT = {"s1": "", "s0": "s1", "s3": "", "s4": "s3"}
TRAVEL = 0.25                   # hinge stop (rad); the slat counts as cracked long before (CRACK_Q)
SLIDERS = tuple(
    Slider(n, rest=(0.0, float(_HINGE_PT[n][0]), float(_HINGE_PT[n][1])), boxes=tuple(_SPECS0[SEG.index(n)]),
           kind="revolute", axis="X", travel=TRAVEL, both_ways=True, rate=RATE, damping=DAMP,
           mass=_mass[SEG.index(n)], parent=_PARENT[n])
    for n in ("s1", "s0", "s3", "s4"))

# ---- bed frame: two fixed side rails with slat holders ------------------------------------------------------------
# holder end-wall inner faces at y +-YW. A cap's end face leans out with the slat's end slope, so its top edge reaches
# CAP_T/2 sin(2 phi0) past the face centre: the span is set 0.3 mm short of the slat's rest length over those edges.
YW = CHORD0 / 2 + (CAP_T / 2) * math.sin(2 * PHI0) - 0.00015
LIP = 0.004                    # retaining lip reaches 4 mm in from the end wall
POCKET_W = 0.064               # holder inside width (58 mm cap)
POCKET_L = 0.050               # holder floor length, end wall to its open inboard end
GAP_H = 0.016                  # floor to the lip's underside
HOLD_H = 0.018                 # holder walls 18 mm above the rail top
WALL = 0.004
RAIL_H, RAIL_W = 0.045, 0.064
ZF = TOP + RAIL_H              # holder floor (the rail top)
X_SLOTS = (0.30, 0.41, 0.52)   # slat pitch 110 mm; the middle pair of holders is empty
XS = X_SLOTS[1]
RAIL_X = (0.255, 0.625)
SIDE_LO = 0.012                # holder side walls: vertical to 12 mm, then a 45 deg lead-in to the 18 mm top
FRAME_MASS = 6.0               # two pine side rails and six holders (estimate); loose on the bench


def _frame_boxes():
    """The bed-frame section in its own frame: origin on the bench at x = XS, y = 0."""
    out = []
    for side in (1, -1):
        yc = side * (YW - POCKET_L + RAIL_W / 2)
        out.append((((RAIL_X[0] + RAIL_X[1]) / 2 - XS, yc, RAIL_H / 2), (RAIL_X[1] - RAIL_X[0], RAIL_W, RAIL_H), PINE))
        for xs in X_SLOTS:
            x0 = xs - XS
            out.append(((x0, side * (YW + WALL / 2), RAIL_H + HOLD_H / 2), (POCKET_W + 4 * WALL, WALL, HOLD_H), HOLDC))
            out.append(((x0, side * (YW - LIP / 2), RAIL_H + (GAP_H + HOLD_H) / 2),
                        (POCKET_W + 4 * WALL, LIP, HOLD_H - GAP_H), HOLDC))
            yw = side * (YW - POCKET_L / 2)
            for sx in (1, -1):
                out.append(((x0 + sx * (POCKET_W / 2 + WALL), yw, RAIL_H + SIDE_LO / 2), (2 * WALL, POCKET_L, SIDE_LO),
                            HOLDC))
                h = HOLD_H - SIDE_LO                       # the lead-in: a 45 deg face from the wall's inner top edge
                cxm, czm = POCKET_W / 2 + h / 2, SIDE_LO + h / 2
                t = 0.006
                cx, cz = cxm + t / 2 * 0.7071, czm - t / 2 * 0.7071
                out.append(((x0 + sx * cx, yw, RAIL_H + cz), (h * 1.4142 + 0.001, POCKET_L, t), HOLDC,
                            ("Y", -45.0 * sx)))
    return out


FRAME = part("frame", pos=(XS, 0.0, TOP + 0.0003), mass=FRAME_MASS, color=PINE, friction=0.8, boxes=_frame_boxes(),
             can_sleep=True)
OBJECTS: list = [FRAME]

# ---- the neighbouring slats: seated, loose, rigid (their bow fixed slightly deeper so they sit between the walls) ---
PHI_N = _phi_for(lambda p: -(shape(p)[0][-1][0] - shape(p)[0][0][0] + CAP_T * math.sin(2 * p)), -(2 * YW - 0.0006))
_PN, _SLN = shape(PHI_N)


def _tip_low(pts, slopes):
    """Height of the slat's lowest points (its caps' outer bottom corners) in the part frame."""
    s = slopes[4]
    return float(pts[-1][1] - (CAP_T / 2) * math.cos(s))


Z_SEAT_N = ZF - _tip_low(_PN, _SLN) + 0.0003
NEIGH = [part(f"slat{k}", pos=(X_SLOTS[k], 0.0, Z_SEAT_N), mass=MASS, color=BEECH, friction=0.7,
              boxes=[b for i in range(5) for b in seg_boxes(_PN, _SLN, i)]) for k in (0, 2)]

# ---- the loose slat: crown up on the bench in front of the frame ---------------------------------------------------
SX0, SY0 = 0.195, 0.0
Z_BENCH = TOP - _tip_low(_P0, _SL0) + 0.0005
SLAT = part("slat", pos=(SX0, SY0, Z_BENCH), mass=_mass[2], color=BEECH, friction=0.7, boxes=_SPECS0[2],
            sliders=SLIDERS, ang_damping=1.0)
OBJECTS += [*NEIGH, SLAT]

# ---- scoring -------------------------------------------------------------------------------------------------------
FLOOR_TOL = 0.003          # cap's outer bottom corner within 3 mm of the holder floor
UNDER_LIP = 0.002          # cap's end face at least 2 mm under the 4 mm lip (within 2 mm of the end wall)
RISE_OK = (0.010, 0.040)   # crown up: mid-span 10-40 mm above the caps
CRACK_RISE = 0.075         # over-bent: mid-span more than 75 mm above the line between the caps (55 mm past rest)
CRACK_Q = 0.20             # ... or any hinge bent more than 0.20 rad (11.5 deg) past its rest angle
NEIGH_MOVE = 0.010
FRAME_MOVE = 0.020
HANDS_CLEAR = 0.05
HOLD_STEPS, STILL_MOVE = 20, 0.002
_STATE: dict = {}


def _np(x):
    return np.asarray(getattr(x, "torch", x).detach().cpu(), dtype=np.float64)


def _quat_m(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _bodies(environment, name):
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    poses = _np(asset.data.body_link_pose_w)[0]
    out = {}
    for i, bn in enumerate(asset.body_names):
        out[bn] = (poses[i, :3] - origin, _quat_m(poses[i, 3:7]))
    return out


def slat_geometry(bodies):
    """World points of the loose slat: its cap ends (end-face centre, outer bottom and top corners) and mid-span."""
    out = {}
    for key, seg, i in (("a", "s0", 0), ("b", "s4", 5)):
        p, R = bodies[seg]
        o = np.array(_HINGE_PT[seg]) if seg in _HINGE_PT else np.zeros(2)
        tip = _P0[i]
        k = 0 if i == 0 else 4
        d = (_P0[k + 1] - _P0[k]) / np.linalg.norm(_P0[k + 1] - _P0[k])
        up = np.array([-d[1], d[0]])
        loc = lambda yz: R @ np.array([0.0, yz[0] - o[0], yz[1] - o[1]]) + p
        out[key] = loc(tip)
        out[key + "_lo"] = loc(tip - up * CAP_T / 2)
        out[key + "_hi"] = loc(tip + up * CAP_T / 2)
    out["mid"] = bodies["body"][0].copy()
    return out


def bend(environment):
    asset = environment.unwrapped.scene["slat"]
    names = list(asset.joint_names)
    q = _np(asset.data.joint_pos)[0]
    return {n: float(q[names.index(f"{n}_slide")]) for n in ("s0", "s1", "s3", "s4")}


def rise(g) -> float:
    """Mid-span centre line above the straight line between the two cap end faces' centres."""
    return float(g["mid"][2] - (g["a"][2] + g["b"][2]) / 2)


def cap_state(g, key):
    """For one cap: which holder side it is at, 'seated' (under the lip and on the floor), 'in' (inside the pocket,
    below the lip) or None."""
    tip, lo, hi = g[key], g[key + "_lo"], g[key + "_hi"]
    side = 1 if tip[1] > 0 else -1
    in_x = abs(tip[0] - XS) <= POCKET_W / 2
    ay = abs(tip[1])
    in_y = YW - POCKET_L + CAP_L / 2 <= ay <= YW + 0.001
    below = max(lo[2], hi[2]) < ZF + GAP_H and min(lo[2], hi[2]) > ZF - 0.002
    if not (in_x and in_y and below):
        return side, None
    if ay >= YW - UNDER_LIP and min(lo[2], hi[2]) <= ZF + FLOOR_TOL:
        return side, "seated"
    return side, "in"


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(gate=None, tcp={}, start=None, hist=[], ok=0, n0=None, f0=None, g=None, gw=None, q=None)
    if environment is None:
        return
    seed = _ENV.get("seed")
    rng = np.random.default_rng((int(seed) if seed is not None else 0) + 7307)
    dx, dy = rng.uniform(-0.010, 0.010), rng.uniform(-0.02, 0.02)
    yaw = rng.uniform(-math.radians(4), math.radians(4))
    _STATE["start"] = (round(dx, 4), round(dy, 4), round(math.degrees(yaw), 2))
    write_pose(environment, "slat", (SX0 + dx, SY0 + dy, Z_BENCH), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


_reset()


def _tcps(environment):
    from yam_arena import ik, station

    base = environment.unwrapped
    out = {}
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        qall = _np(rob.data.joint_pos)[0]
        out[side] = ik.tcp_pose(qall[:6], station.BASES[side])[0]
        _STATE.setdefault("fing", {})[side] = [round(float(v) * 1000, 1) for v in qall[6:8]]
    return out


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = max(0.0, min(1.0, float((p - a) @ ab / (ab @ ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


def hands_clear(g) -> list[bool]:
    pts = [g["a"], g["mid"], g["b"]]
    return [min(_seg_dist(np.asarray(t), pts[0], pts[1]), _seg_dist(np.asarray(t), pts[1], pts[2])) >= HANDS_CLEAR
            for t in (_STATE.get("tcp") or {}).values()]


def _level(g, q):
    """0, 0.25, 0.5 or 0.75 from the slat's geometry now (no gate, no hold)."""
    sa, ca = cap_state(g, "a")
    sb, cb = cap_state(g, "b")
    if sa == sb:
        return 0.0
    st = sorted([ca or "", cb or ""])
    if st == ["seated", "seated"] and RISE_OK[0] <= rise(g) <= RISE_OK[1]:
        return 0.75
    if "seated" in st and ("in" in st or st == ["seated", "seated"]):
        return 0.5
    if "seated" in st:
        return 0.25
    return 0.0


@on_step
def _watch(environment) -> None:
    b = _bodies(environment, "slat")
    gw = slat_geometry(b)
    fp, fR = _bodies(environment, "frame")["body"]
    if _STATE.get("f0") is None:
        _STATE["f0"] = fp.copy()
    nom = np.array(FRAME.pos)
    g = {k: fR.T @ (v - fp) + nom for k, v in gw.items()}    # in the frame's nominal pose (it is loose)
    _STATE["gw"] = gw
    q = bend(environment)
    _STATE["g"], _STATE["q"] = g, q
    _STATE["tcp"] = _tcps(environment)
    npos = [_bodies(environment, f"slat{k}")["body"][0] for k in (0, 2)]
    if _STATE.get("n0") is None:
        _STATE["n0"] = [p.copy() for p in npos]
    if not _STATE.get("gate"):
        if g["mid"][2] < TOP - 0.05:
            _STATE["gate"] = "slat fell off the bench"
        elif rise(g) > CRACK_RISE or max(abs(v) for v in q.values()) > CRACK_Q:
            _STATE["gate"] = f"slat over-bent (rise {rise(g) * 1000:.1f} mm, hinges {[round(v, 3) for v in q.values()]})"
        elif np.linalg.norm(fp - _STATE["f0"]) > FRAME_MOVE:
            _STATE["gate"] = f"bed frame moved {np.linalg.norm(fp - _STATE['f0']) * 1000:.1f} mm"
        else:
            for k, (p, p0) in enumerate(zip(npos, _STATE["n0"])):
                if np.linalg.norm(p - p0) > NEIGH_MOVE:
                    _STATE["gate"] = f"neighbouring slat {k} moved {np.linalg.norm(p - p0) * 1000:.1f} mm"
    h = _STATE.setdefault("hist", [])
    h.append(np.r_[g["a"], g["mid"], g["b"]])
    del h[:-HOLD_STEPS]
    lv = _level(g, q)
    hc = hands_clear(gw)
    if lv == 0.75 and hc and all(hc):
        _STATE["ok"] = _STATE.get("ok", 0) + 1
    else:
        _STATE["ok"] = 0


def _still() -> bool:
    h = _STATE.get("hist") or []
    if len(h) < HOLD_STEPS:
        return False
    a = np.array(h).reshape(len(h), 3, 3)
    return bool(np.linalg.norm(a - a[-1], axis=2).max() < STILL_MOVE)


def _update(objects=None):
    if _STATE.get("gate") or _STATE.get("g") is None:
        return 0.0, False
    lv = _level(_STATE["g"], _STATE["q"])
    if lv == 0.75 and _STATE.get("ok", 0) >= HOLD_STEPS and _still():
        return 1.0, True
    return lv, False


def success(objects) -> bool:
    return _update(objects)[1]


def score(objects) -> float:
    """0 after a gate, at any point: the slat fell off the bench (mid-span more than 5 cm below the bench top); the slat
    was over-bent (mid-span more than 75 mm
    above the straight line between its two end-face centres, or any of its hinges bent more than 0.20 rad past its
    rest angle); a neighbouring slat moved more than 10 mm; the bed frame moved more than 20 mm. Otherwise, at the
    end, judged in the frame's own coordinates and only with the two caps at opposite holders of the empty pair. A cap
    is 'in' its holder when its end-face centre lies within the 64 mm pocket width and within 27.5 mm of the end wall
    (at most 1 mm past it), with the whole end face below the lip's underside and no more than 2 mm below the floor; it is 'seated' when, in addition, its end-face centre is within 2 mm of the end
    wall (under the 4 mm lip) and its outer bottom edge is within 3 mm of the floor. 1.0 (success) when 0.75 has held
    for 1 s (20 steps), both grippers' centre points are at least 5 cm from the slat and the slat moved less than 2 mm
    over that second; 0.75 when both caps are seated and the slat is crown up, mid-span 10 to 40 mm above the line
    between its end-face centres; 0.5 when one cap is seated and the other is in its holder; 0.25 when one cap is
    seated (a seated cap also counts as in). Grippers' distance is to the two straight lines from the slat's end-face
    centres to mid-span. Otherwise 0."""
    return _update(objects)[0]


def report(objects=None) -> dict:
    g, q = _STATE.get("g"), _STATE.get("q")
    if g is None:
        return {"start": _STATE.get("start")}
    rel = lambda p: [round(float(p[0] - XS) * 1000, 1), round(float(p[1]) * 1000, 1), round(float(p[2] - ZF) * 1000, 1)]
    return {"start": _STATE.get("start"), "gate": _STATE.get("gate"), "score": _update(objects)[0],
            "caps": [cap_state(g, "a"), cap_state(g, "b")], "rise_mm": round(rise(g) * 1000, 1),
            "cap_a": rel(g["a"]), "cap_b": rel(g["b"]), "cap_a_lo": rel(g["a_lo"]), "cap_b_lo": rel(g["b_lo"]),
            "mid": rel(g["mid"]), "hinges": {k: round(v, 3) for k, v in q.items()},
            "tcp": {k: [round(float(x) * 1000) for x in v] for k, v in (_STATE.get("tcp") or {}).items()},
            "fing": _STATE.get("fing"), "hands_clear": hands_clear(_STATE["gw"]), "held": _STATE.get("ok", 0), "still": _still()}


def debug_line(objects) -> str:
    r = report(objects)
    return " ".join(f"{k}={v}" for k, v in r.items())
