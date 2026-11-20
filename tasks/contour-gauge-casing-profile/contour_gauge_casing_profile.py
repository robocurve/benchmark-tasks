"""Scribe a floor board's end to a door casing with a contour gauge: lay the gauge flat on the floor, push it against
the casing where it meets the wall until its frame seats on the casing, draw it straight back with the fingers holding
the copied outline, and lay it flat on the board with the copied wall line on the board's pencilled wall line and the
casing's inner edge at the board's side edge, ready to trace the cut.

Real product: a 10 in (254 mm) plastic-finger contour gauge of the General Tools 833 class (fingers 76 mm long, held
in the frame by friction, 32 mm usable depth), with a hardwood grip handle taped to each end of its frame.
Casing: colonial door casing 11/16 x 2-1/4 in (17.5 x 57 mm), pine, made to the profile below (README: fabrication),
nailed to a wall section beside a door opening. Board: 1x6 floor board (19 x 140 mm). Model grain and estimates
(README, sourced vs estimated): 31 fingers at 8 mm pitch (the real gauge has ~1.5 mm fingers); per-finger hold
FINGER_HOLD_N; finger mass; the casing's stepped profile.

Finger model (no weld, no teleport): each finger is a slide joint in the frame (0 = fully out, + = pushed back into
it). A drive holds it at the point where it was left (the latch) and slips past FINGER_HOLD_N (Coulomb-like friction in
the frame); the on_step hook moves the latch, the same in Isaac and MuJoCo. The gauge lies flat, so gravity acts across
the fingers, never along them. A press driven by one hand only swings the frame out of square with the wall and the
copy fails the square gate (README).

Bench = floor (station coordinates, TOP = station.TABLE_TOP, +X forward, +Y left): the wall section stands at the far
side, its room face at WALL_X, the door opening to its right (-y); the casing is nailed to the wall's face with its
inner (bead) edge toward the opening. The gauge lies flat on the floor in front of the casing, fingers toward the wall.
The board lies nearest the robot, long axis along x, its far end toward the wall, a pencilled wall line across it.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- contour gauge, lying flat (part frame: x = finger direction, tips at x = 0 when fully out; y = long axis;
#    z = up from the gauge's bottom face) --------------------------------------------------------------------------
N_FINGERS = 31
PITCH = 0.008  # finger pitch (real ~1.5 mm; grain disclosed)
F_THICK, F_WIDE, F_LEN = 0.007, 0.021, 0.076  # finger: along the gauge, through its thickness, length (maker: 76 mm)
DEPTH = 0.032  # usable depth (maker): finger tips sit 32 mm in front of the frame when fully out
FRAME_L, FRAME_W, FRAME_H = 0.254, 0.022, 0.010  # frame: length (maker: 254 mm wide gauge), thickness, depth
# a hardwood grip handle on each frame end: a bridge carrying the frame on past its last finger, then a block sticking
# back from it (the hands hold the blocks, behind the frame, clear of the fingers and of the wall)
BRIDGE_Y = (FRAME_L / 2, 0.165)  # bridge: from the frame's end out to the handle's outer face (y), frame depth and height
BLOCK = (0.058, 0.025, 0.054)  # grip block: length back from the frame (x), width (y), height (z)
BLOCK_X = -(DEPTH + FRAME_H + BLOCK[0] / 2)  # its centre behind the finger-tip line
BLOCK_Y = BRIDGE_Y[1] - BLOCK[1] / 2
MID_Z = FRAME_W / 2  # the fingers' mid-plane above the gauge's bottom face
M_FRAME, M_FINGER = 0.090, 0.0016  # 140 g in all with the grip handles (estimate)
FINGER_HOLD_N = 0.5  # friction hold of one model finger in the frame (engineering estimate, README)
K_FINGER = 200.0  # N/m: the latch drive (2.5 mm elastic at FINGER_HOLD_N; stiffer is unstable at the 240 Hz step)
FINGER_U = tuple(round((i - (N_FINGERS - 1) / 2) * PITCH, 6) for i in range(N_FINGERS))  # finger centres along y
FINGERS = tuple(f"f{i:02d}" for i in range(N_FINGERS))

GAUGE_GREY, FINGER_WHITE, BLOCK_WOOD = (0.20, 0.22, 0.24), (0.92, 0.92, 0.88), (0.55, 0.36, 0.20)
PINE, PINE_DARK, PLANK_WOOD, PENCIL = (0.86, 0.72, 0.50), (0.78, 0.62, 0.40), (0.80, 0.66, 0.46), (0.15, 0.15, 0.17)
WALL_WHITE = (0.90, 0.90, 0.87)

_fingers = tuple(
    Joint(n, kind="slide", axis_vec=(-1.0, 0.0, 0.0), anchor=(0.0, u, MID_Z), lower=0.0, upper=DEPTH,
          stiffness=K_FINGER, damping=0.5, target=0.0, initial=0.0, mass=M_FINGER,
          boxes=(((-(F_LEN / 2 + F_THICK / 4), u, MID_Z), (F_LEN - F_THICK / 2, F_THICK, F_WIDE), FINGER_WHITE),),
          cylinders=(((-F_THICK / 2, u, MID_Z), F_THICK / 2, F_WIDE, FINGER_WHITE, "Z"),))  # rounded tip
    for n, u in zip(FINGERS, FINGER_U))
_gauge_body = (
    ((-(DEPTH + FRAME_H / 2), 0.0, FRAME_W / 2), (FRAME_H, FRAME_L, FRAME_W), GAUGE_GREY),
    *(((-(DEPTH + FRAME_H / 2), sy * sum(BRIDGE_Y) / 2, FRAME_W / 2), (FRAME_H, BRIDGE_Y[1] - BRIDGE_Y[0], FRAME_W),
        BLOCK_WOOD) for sy in (1, -1)),
    *(((BLOCK_X, sy * BLOCK_Y, BLOCK[2] / 2), BLOCK, BLOCK_WOOD) for sy in (1, -1)),
)

# -- casing: colonial 11/16 x 2-1/4 in, nailed to the wall, standing on the floor; profile across its width (part y,
#    from the inner bead edge at -y to the outer band at +y), standing proud of the wall toward the room (-x) --------
CAS_W, CAS_H = 0.057, 0.0175
CAS_HT = 0.150  # the casing's leg, cropped 150 mm above the floor
# (x0, x1, h0, h1) from the inner bead edge (x = 0) to the thick outer band, metres; h = how far it stands off the wall
PROFILE = (
    (0.000, 0.010, 0.0120, 0.0120),  # raised bead at the inner edge, 12 mm high, 10 mm wide
    (0.010, 0.028, 0.0060, 0.0060),  # 6 mm drop to a flat field at 6 mm, 18 mm wide
    (0.028, 0.040, 0.0060, 0.0120),  # bevel rising 6 mm over 12 mm
    (0.040, 0.057, 0.0175, 0.0175),  # 5.5 mm step up to the 17.5 mm outer band, 17 mm wide
)
MIN_FEATURE_H, MIN_FEATURE_W = 0.0055, 0.010  # every step >= 5.5 mm high, every feature >= 10 mm wide


def profile_h(c: float) -> float:
    """How far the casing stands off the wall at c across its width (c = 0 at its centre line); 0 off the casing."""
    x = c + CAS_W / 2
    for x0, x1, h0, h1 in PROFILE:
        if x0 <= x <= x1:
            return h0 + (h1 - h0) * (x - x0) / (x1 - x0)
    return 0.0


def _casing_boxes():
    out = []
    for x0, x1, h0, h1 in PROFILE:
        lo, w, cy = min(h0, h1), x1 - x0, x0 + (x1 - x0) / 2 - CAS_W / 2
        out.append(((-lo / 2, cy, CAS_HT / 2), (lo, w, CAS_HT), PINE))
        if abs(h1 - h0) > 1e-9:  # sloped face: a 3 mm plate whose outer face is the slope line (turned about z)
            a, t = math.atan2(h1 - h0, w), 0.003
            length = math.hypot(w, h1 - h0)
            mid = np.array([-(h0 + h1) / 2, cy]) + t / 2 * np.array([w, h1 - h0]) / length
            out.append(((float(mid[0]), float(mid[1]), CAS_HT / 2), (t, length, CAS_HT), PINE_DARK, (a, 0.0)))
    return tuple(out)


# -- layout ----------------------------------------------------------------------------------------------------------
WALL_X = 0.400  # the wall's room face
CAS_Y = 0.000  # casing centre line; its inner (bead) edge at -28.5 mm, the opening beyond
REVEAL = 0.006  # the door jamb's face, flush with the wall, shows 6 mm beyond the casing's inner edge
JAMB_Y = CAS_Y - CAS_W / 2 - REVEAL  # the opening's side: no wall on the -y side of this
WALL_T, WALL_END_Y = 0.040, 0.170  # wall section (drywall on studs, cropped): depth, far end on the band side
G0_X, G0_Y = 0.300, 0.000  # gauge (its finger-tip line) lying flat on the floor at the start
PL_X, PL_LEN, PL_W, PL_H = 0.050, 0.300, 0.140, 0.019  # 1x6 floor board, long axis x, far end at x = 0.20
PL_Y = CAS_Y - CAS_W / 2 + PL_W / 2  # its right (-y) side edge in line with the casing's inner edge
CUT_X = PL_X + PL_LEN / 2 - 0.030  # pencilled wall line 30 mm from the board's far end
PL_TOP = TOP + PL_H
PL_EDGE_Y = PL_Y - PL_W / 2  # the board's side edge the casing's inner edge must meet
LINE_H = 0.0001

GAUGE = part("gauge", (G0_X, G0_Y, TOP), mass=M_FRAME, friction=0.6, boxes=_gauge_body, joints=_fingers)
CASING = part("casing", (WALL_X, CAS_Y, TOP), fixed=True, friction=0.5, boxes=_casing_boxes())
WALL = part("wall", (WALL_X, 0.0, TOP), fixed=True, friction=0.5,
            boxes=(((WALL_T / 2, (JAMB_Y + WALL_END_Y) / 2, CAS_HT / 2), (WALL_T, WALL_END_Y - JAMB_Y, CAS_HT),
                    WALL_WHITE),))
PLANK = part("plank", (PL_X, PL_Y, TOP), fixed=True, friction=0.6,
             boxes=(((0.0, 0.0, PL_H / 2), (PL_LEN, PL_W, PL_H), PLANK_WOOD),
                    ((CUT_X - PL_X, 0.0, PL_H + LINE_H / 2), (0.001, PL_W, LINE_H), PENCIL)))
OBJECTS = (GAUGE, CASING, WALL, PLANK)

# -- finger latch (single source for on_step and tests) ----------------------------------------------------------------


def latch(q: float, held: float) -> float:
    """New latch (drive target) of one finger at position q: it stays where it was left and slips past FINGER_HOLD_N
    (the drive follows to within FINGER_HOLD_N / K_FINGER of the finger)."""
    lim = FINGER_HOLD_N / K_FINGER
    return min(max(held, q - lim), q + lim)


_trial: dict = {}
_ids: dict = {}


def _fresh():
    return {"latch": [0.0] * N_FINGERS, "tcps": None, "step": -1}


_trial.update(_fresh())


@on_reset
def _reset(_env) -> None:
    _trial.clear()
    _trial.update(_fresh())
    _ids.clear()


def _arm_tcps(env):
    try:
        out = []
        for side in ("left", "right"):
            asset = env.unwrapped.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            out.append(ik.tcp_pose(q, station.BASES[side])[0])
        return tuple(out)
    except Exception:
        return None


@on_step
def _watch(env) -> None:
    import torch

    a = env.unwrapped.scene["gauge"]
    if "j" not in _ids:
        names = list(a.joint_names)
        _ids["j"] = [names.index(f"{n}_joint") for n in FINGERS]
    q = labkit._np(a.data.joint_pos)[0]
    _trial["latch"] = [latch(float(q[j]), h) for j, h in zip(_ids["j"], _trial["latch"])]
    a.set_joint_position_target(torch.tensor([_trial["latch"]], device=a.device), joint_ids=_ids["j"])
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)
    if _trial["step"] % 20 == 19:
        gs = labkit._part_state(env, labkit._SPECS["gauge"])
        print("STATE " + json.dumps({"step": _trial["step"] + 1, **_eval(gs)}), flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------
RMS_MM, MAX_MM = 1.5, 3.0  # copied outline vs the casing: RMS and worst finger over the fingers on the casing
REF_SPREAD_MM = 2.5  # wall fingers beside the casing's outer band: peak-to-peak spread about one straight line <= 2.5 mm
REF_BAND_M = 0.030  # ... those within 30 mm of its outer edge (the opening is beyond its inner edge)
EDGE_M = 0.001  # a finger reads the casing when its tip is at least 1 mm onto it
SQUARE_DEG = 3.0  # that line = how square to the wall the frame was pushed: within 3 deg
MIN_ON_CASING = 5  # at least 5 fingers read the casing (7 or 8 do wherever it lies fully in front of the gauge)
FLAT_DEG, ALONG_DEG = 3.0, 5.0  # gauge lying flat within 3 deg, its fingers pointing at the board's end within 5 deg
SEAT_TOL_M = 0.003  # its bottom face on the board's top (within 3 mm)
LINE_MEAN_M, LINE_MAX_M = 0.002, 0.003  # copied wall line on the pencilled wall line: mean within 2 mm, each within 3 mm
EDGE_TOL_M = 0.003  # copied inner edge of the casing at the board's side edge within 3 mm
HAND_CLEAR_M = 0.20  # both TCPs more than 20 cm from the gauge centre in plan (its grip blocks end 15 cm out), or 15 cm above the board
PARTIAL = 0.4  # outline copied and held, gauge not laid in the marking position
PASS_SCORE = 1.0 - 1e-9
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
_FOOT = np.linspace(-F_THICK / 2, F_THICK / 2, 15)
_ROUND = F_THICK / 2 - np.sqrt(np.maximum((F_THICK / 2) ** 2 - _FOOT ** 2, 0.0))  # rounded tip: rise off its centre
_S_GRID = np.arange(-(FRAME_L / 2 - CAS_W / 2 - 2 * PITCH), FRAME_L / 2 - CAS_W / 2 - 2 * PITCH + 1e-9, 0.0005)


def finger_reading(c: float) -> float:
    """How far back a finger whose centre is at c across the casing stops: its rounded tip (radius F_THICK / 2)
    resting on the most proud point of the casing in front of it."""
    return max(max(profile_h(c + d) - r, 0.0) for d, r in zip(_FOOT, _ROUND))


_U = np.array(FINGER_U)
# every (face, offset) laid against the casing: the casing height under each finger, one row per candidate
_CANDS = [(sign, float(s)) for sign in (1.0, -1.0) for s in _S_GRID]
_C_TAB = np.arange(-0.040, 0.040, 0.00005)
_R_TAB = np.array([finger_reading(float(c)) for c in _C_TAB])
_R_IN = np.array([max(profile_h(float(c) + d) for d in _FOOT * (1 - 2 * EDGE_M / F_THICK)) for c in _C_TAB])  # > 0: on
_CC = np.array([sign * _U - s for sign, s in _CANDS])  # finger centres across the casing
_H = np.interp(_CC, _C_TAB, _R_TAB, left=0.0, right=0.0)
_H_IN = np.interp(_CC, _C_TAB, _R_IN, left=0.0, right=0.0)
_ON = _H_IN > 0  # fingers at least EDGE_M onto the casing
_EDGE = (_H > 0) & ~_ON  # fingers that only graze its edge: left out, either way is a fair copy
_GAP = _CC - CAS_W / 2 - F_THICK / 2  # how far a finger's tip is past the casing's outer (band) edge
_NEAR = ~_ON & ~_EDGE & (_GAP > 0) & (_GAP <= REF_BAND_M)  # wall fingers beside the outer band (the copy's reference)
_W = (_ON | _NEAR).astype(float)
_N_ON = _ON.sum(axis=1)


def profile_fit(q: dict) -> dict:
    """Lay the gauge back against the casing (as one checks a copy): over every offset s along the gauge and
    both faces, the pushes q (m) minus the casing heights in front of the fingers on the casing and on the wall beside
    its outer band (within REF_BAND_M), less one straight line (how square the frame was pushed), must vanish.
    Candidates that meet every gate (count, RMS, worst, spread, slope) are screened first and the lowest RMS among
    them is returned; if none does, the lowest-RMS candidate is returned for diagnostics. Returns rms/max over the
    fingers on the casing, peak-to-peak spread of the wall fingers beside it, the line's slope (deg), how many fingers
    read the casing, the candidate (face, offset) and the indices of its wall fingers."""
    qv = np.array([float(q.get(n, 0.0)) for n in FINGERS])
    key = qv.tobytes()
    if _LAST.get("key") != key:
        _LAST.update(key=key, fit=_fit(qv))
    return dict(_LAST["fit"])


_LAST: dict = {}


def _fit(qv: np.ndarray) -> dict:
    r = qv[None, :] - _H
    s0, s1, s2 = _W.sum(1), (_W * _U).sum(1), (_W * _U ** 2).sum(1)
    t0, t1 = (_W * r).sum(1), (_W * _U * r).sum(1)
    det = s0 * s2 - s1 ** 2
    b = (s0 * t1 - s1 * t0) / det
    a = (t0 - s1 * b) / s0
    e = (r - a[:, None] - b[:, None] * _U[None, :]) * 1000.0
    rms_all = np.sqrt(np.where(_ON, e ** 2, 0.0).sum(axis=1) / np.maximum(_N_ON, 1))
    max_all = np.where(_ON, np.abs(e), 0.0).max(axis=1)
    spread_all = np.where(_NEAR.any(axis=1), np.where(_NEAR, e, -np.inf).max(axis=1) - np.where(_NEAR, e, np.inf).min(axis=1), 99.0)
    slope_all = np.degrees(np.arctan(np.abs(b)))
    valid = ((_N_ON >= MIN_ON_CASING) & (rms_all <= RMS_MM) & (max_all <= MAX_MM) & (spread_all <= REF_SPREAD_MM)
             & (slope_all <= SQUARE_DEG))
    k = int(np.argmin(np.where(valid if valid.any() else _N_ON > 0, rms_all, np.inf)))
    ek = e[k]
    ref = ek[_NEAR[k]]
    sign, s = _CANDS[k]
    return {"rms_mm": float(rms_all[k]), "max_mm": float(np.max(np.abs(ek[_ON[k]]))),
            "ref_spread_mm": float(ref.max() - ref.min()) if ref.size else 99.0,
            "slope_deg": math.degrees(math.atan(abs(float(b[k])))), "on_casing": int(_N_ON[k]),
            "s_mm": round(s * 1000, 2), "face": int(sign), "push_mm": round(float(a[k]) * 1000, 2),
            "wall_idx": [int(i) for i in np.flatnonzero(_NEAR[k])]}


def copied(ps: PartState) -> bool:
    f = profile_fit(ps.joints)
    return (f["on_casing"] >= MIN_ON_CASING and f["rms_mm"] <= RMS_MM and f["max_mm"] <= MAX_MM
            and f["ref_spread_mm"] <= REF_SPREAD_MM and f["slope_deg"] <= SQUARE_DEG)


def _tip(ps: PartState, i: int) -> np.ndarray:
    """World position of finger i's tip (on its mid-plane)."""
    return ps.to_world((-float(ps.joints.get(FINGERS[i], 0.0)), FINGER_U[i], MID_Z))


def _frame_centre(ps: PartState) -> np.ndarray:
    return ps.to_world((-(DEPTH + FRAME_H / 2), 0.0, MID_Z))


def _hands_clear(tcps, ps) -> bool:
    if tcps is None or len(tcps) < 2:
        return False
    c = _frame_centre(ps)
    return all(np.linalg.norm(np.asarray(t)[:2] - c[:2]) > HAND_CLEAR_M or np.asarray(t)[2] > PL_TOP + 0.15 for t in tcps)


def marking(ps: PartState) -> dict:
    """Where the copied outline lies on the board: the wall fingers' tips against the pencilled wall line (x, m) and
    the copied inner and outer edges of the casing across the board (y, m)."""
    f = profile_fit(ps.joints)
    idx = f["wall_idx"]
    tips_x = [float(_tip(ps, i)[0]) for i in idx]
    sign, s = float(f["face"]), f["s_mm"] / 1000.0
    x_line = -float(np.mean([ps.joints.get(FINGERS[i], 0.0) for i in idx])) if idx else 0.0
    inner = ps.to_world((x_line, sign * (s - CAS_W / 2), MID_Z))
    outer = ps.to_world((x_line, sign * (s + CAS_W / 2), MID_Z))
    return {"line_dx": [x - CUT_X for x in tips_x], "inner_y": float(inner[1]), "outer_y": float(outer[1])}


def on_mark(ps: PartState) -> bool:
    """Gauge lying flat on the board, fingers toward its far end, the copied wall line on the pencilled wall line and
    the copied inner edge of the casing at the board's side edge (the outer band over the board)."""
    r = ps.rotation()
    along = math.degrees(math.acos(max(-1.0, min(1.0, float(r[0, 0])))))
    if ps.tilt_deg() > FLAT_DEG or along > ALONG_DEG or abs(float(ps.pos[2]) - PL_TOP) > SEAT_TOL_M:
        return False
    m = marking(ps)
    if not m["line_dx"]:
        return False
    return (abs(float(np.mean(m["line_dx"]))) <= LINE_MEAN_M and max(abs(d) for d in m["line_dx"]) <= LINE_MAX_M
            and abs(m["inner_y"] - PL_EDGE_Y) <= EDGE_TOL_M and m["outer_y"] > m["inner_y"])


def _eval(gs: PartState) -> dict:
    f = profile_fit(gs.joints)
    c = _frame_centre(gs)
    m = marking(gs)
    return {"fit": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in f.items() if k != "wall_idx"},
            "gauge_c_mm": [round(float(v) * 1000, 1) for v in c], "tilt_deg": round(gs.tilt_deg(), 2),
            "bottom_mm": round((float(gs.pos[2]) - TOP) * 1000, 1),
            "line_dx_mm": [round(d * 1000, 1) for d in m["line_dx"]],
            "inner_dy_mm": round((m["inner_y"] - PL_EDGE_Y) * 1000, 1),
            "q_mm": [round(float(gs.joints.get(n, 0.0)) * 1000, 1) for n in FINGERS],
            "copied": bool(copied(gs)), "on_mark": bool(on_mark(gs)),
            "hands_clear": bool(_hands_clear(_trial["tcps"], gs))}


def score(objects: dict[str, PartState]) -> float:
    """Judged at the end of the trial. 0 unless the copied outline matches the casing laid back against it at some
    offset and face (candidates are screened against every gate, then the lowest RMS is taken): at least 5 fingers at
    least 1 mm onto the casing within RMS 1.5 mm and worst 3.0 mm of its heights, the wall fingers within 30 mm of its
    outer edge within a peak-to-peak spread of 2.5 mm about one straight line, and that line (how square to the wall
    the frame was pushed) within 3 deg. Every feature of the casing's profile is at least 5.5 mm high and 10 mm wide,
    so these gates tell a copy from an unpushed gauge, a push against the bare wall, a skewed push, a flat block and a
    plain bevel of the same rise. Then 0.4; 1.0 if the gauge also lies flat (within 3 deg) on the board, its bottom
    within 3 mm of the board's top, its fingers pointing at the board's far end (within 5 deg), the copied wall line on
    the pencilled wall line (wall fingers' tips within 2 mm on average and 3 mm each), the copied inner edge of the
    casing within 3 mm of the board's side edge with the outer band over the board, and each hand's tool point more
    than 20 cm from the gauge centre in plan or more than 15 cm above the board. Possible: 0, 0.4, 1.0."""
    gs = objects["gauge"]
    if not copied(gs):
        return 0.0
    return 1.0 if on_mark(gs) and _hands_clear(_trial["tcps"], gs) else PARTIAL


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def _set_test_trial(**kw) -> None:
    """Explicit test hook for offline scoring tests (never called by demo())."""
    for k, v in kw.items():
        if k not in _trial:
            raise KeyError(k)
        _trial[k] = v


# -- demo (open loop, constants only) ---------------------------------------------------------------------------------
GRIP_U = BLOCK_Y  # each hand takes one grip block
GRIP_DX = BLOCK_X  # TCP behind the finger-tip line: on the block's centre, 75 mm back (the palm stays clear of the wall)
GRIP_DZ = 0.026 + 0.013  # TCP above the gauge's bottom face: pad tips 26 mm up the 54 mm block, open jaws clear of the fingers (21.5 mm)
SEAT_X = WALL_X - CAS_H + DEPTH  # finger-tip line (fully out) when the frame seats on the casing's outer band
PRESS_OVER = 0.015  # commanded past the seat by each hand: the arms' compliance turns it into the push force
COPY_DZ = 0.003  # the gauge carried 3 mm off the floor while it is pushed home
PUSH_NOM = DEPTH - CAS_H - FINGER_HOLD_N / K_FINGER  # wall finger push after a press that seats the frame (relaxed)
# The demo's push stops about 11 mm short of seating (the hands' grip on the blocks yields as the fingers load up), so
# its wall fingers keep 0.2 to 0.8 mm of push after the elastic give (Isaac run); the copy is the same, only deeper.
PUSH_DEMO = 0.0005
PLACE_X = CUT_X + PUSH_DEMO  # finger-tip line (fully out) with the demo's copied wall line on the pencilled line
JAWS_Y = 0.0  # jaws closing along y, across the 25 mm block: the palm's narrow side faces the wall


def hand(side: float, park) -> list:
    """One hand's waypoints (side +1 = left hand on the +y block, -1 = right hand on the -y block); both hands run the
    same timing, so the gauge is carried square between them."""
    W = pitch_demo.Waypoint
    o, c = FINGER_OPEN, FINGER_CLOSED
    y, yaw = G0_Y + side * GRIP_U, JAWS_Y
    hi = TOP + 0.10
    floor = TOP + COPY_DZ + GRIP_DZ
    board = PL_TOP + 0.001 + GRIP_DZ
    return [
        W(1.5, (G0_X + GRIP_DX, y, TOP + GRIP_DZ + 0.06), o, "over the grip block", yaw=yaw),
        W(1.0, (G0_X + GRIP_DX, y, TOP + GRIP_DZ), o, "jaws round the block", yaw=yaw),
        W(0.6, (G0_X + GRIP_DX, y, TOP + GRIP_DZ), c, "grip the block", yaw=yaw),
        W(0.6, (G0_X + GRIP_DX, y, floor), c, "just off the floor", yaw=yaw),
        W(1.5, (WALL_X - CAS_H - 0.003 + GRIP_DX, y, floor), c, "slide it up to the casing", yaw=yaw),
        W(2.0, (SEAT_X + PRESS_OVER + GRIP_DX, y, floor), c, "push it square at the wall", yaw=yaw),
        W(0.8, (SEAT_X + PRESS_OVER + GRIP_DX, y, floor), c, "hold it seated", yaw=yaw),
        W(0.6, (SEAT_X - 0.005 + GRIP_DX, y, floor), c, "ease off", yaw=yaw),
        W(1.2, (SEAT_X - 0.060 + GRIP_DX, y, floor), c, "draw it straight back", yaw=yaw),
        W(1.0, (SEAT_X - 0.060 + GRIP_DX, y, hi), c, "lift it level", yaw=yaw),
        W(2.0, (PLACE_X + GRIP_DX, y, hi), c, "carry it over the board", yaw=yaw),
        W(1.2, (PLACE_X + GRIP_DX, y, board + 0.03), c, "down to the wall line", yaw=yaw),
        W(1.0, (PLACE_X + GRIP_DX, y, board), c, "lay it flat on the board", yaw=yaw),
        W(0.6, (PLACE_X + GRIP_DX, y, board), o, "let go", yaw=yaw),
        W(0.8, (PLACE_X + GRIP_DX, y, board + 0.08), o, "up clear of the block", yaw=yaw),
        W(1.5, park, o, "park clear"),
        W(8.0, park, o, "hold clear"),
    ]


def demo():
    """Open loop, fixed scene geometry, both hands in step:
    1. Each hand takes one of the gauge's grip blocks and they lift it just off the floor, level.
    2. They push it straight at the wall, commanded past where the frame would seat on the casing's outer band: the
       fingers in front of the casing are pushed back to its profile, the ones in front of the wall to the wall; those in
       the opening stay out.
    3. They draw it straight back and lift it; the fingers stay where the casing and wall left them.
    4. They lay it flat on the board with the copied wall line on the pencilled wall line and the casing's inner edge
       at the board's side edge, let go and park."""
    return {"left": hand(1.0, (0.20, 0.30, TOP + 0.20)), "right": hand(-1.0, (0.16, -0.32, TOP + 0.20))}
