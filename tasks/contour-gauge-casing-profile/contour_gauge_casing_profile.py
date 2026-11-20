"""Copy a door casing's profile with a contour gauge: press the gauge onto the casing offcut until its frame seats, lift it straight off with the fingers holding the copied profile, and stand it
on the plank across the pencilled cut line, the copied profile kept.

Real product: a 10 in (254 mm) plastic-finger contour gauge of the General Tools 833 class (fingers 76 mm long, held
in the frame by friction, 32 mm usable depth). Casing: colonial door casing 11/16 x 2-1/4 in (17.5 x 57 mm), pine.
Plank: 1x6 board (19 x 140 mm). Model grain and estimates (README, sourced vs estimated): 31 fingers at 8 mm pitch
(the real gauge has ~1.5 mm fingers); per-finger hold FINGER_HOLD_N; finger mass; the casing's stepped profile.

Finger model (no weld, no teleport): each finger is a slide joint in the frame (axis z, 0 = fully out below the
frame, + = pushed up into it). A drive holds it at the point where it was left (the latch) and slips past
FINGER_HOLD_N (Coulomb-like friction in the frame); the on_step hook moves the latch, the same in Isaac and MuJoCo.
The gauge can be steadied with one or both hands; the scorer does not ask which. A press driven down at one end only
lays the frame out of level and the copy fails the square gate (README).

Bench (station coordinates, TOP = station.TABLE_TOP, +X forward, +Y left): the casing offcut lies face up in the
middle, long axis along x; the gauge stands on its finger tips nearer the robot, long axis along y; the plank lies
nearest the robot, long axis along x, its pencilled cut line across it near its far end.
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

# -- contour gauge (part frame: z = 0 at the finger tips when every finger is fully out; long axis x) -----------------
N_FINGERS = 31
PITCH = 0.008  # finger pitch (real ~1.5 mm; grain disclosed)
F_THICK, F_WIDE, F_LEN = 0.007, 0.012, 0.076  # finger: along the gauge, across it, length (maker: 76 mm fingers)
DEPTH = 0.032  # usable depth (maker): finger tips sit 32 mm below the frame when fully out
FRAME_L, FRAME_W, FRAME_H = 0.254, 0.022, 0.010  # frame / spine (maker: 254 mm wide gauge)
FRAME_Z0 = DEPTH  # frame bottom above the finger-tip plane
M_FRAME, M_FINGER = 0.050, 0.0016  # 100 g in all (estimate)
FINGER_HOLD_N = 0.5  # friction hold of one model finger in the frame (engineering estimate, README)
K_FINGER = 200.0  # N/m: the latch drive (2.5 mm elastic at FINGER_HOLD_N; stiffer is unstable at the 240 Hz step)
FINGER_U = tuple(round((i - (N_FINGERS - 1) / 2) * PITCH, 6) for i in range(N_FINGERS))  # finger centres along x
FINGERS = tuple(f"f{i:02d}" for i in range(N_FINGERS))

GAUGE_GREY, FINGER_WHITE = (0.20, 0.22, 0.24), (0.92, 0.92, 0.88)
PINE, PINE_DARK, PLANK_WOOD, PENCIL = (0.86, 0.72, 0.50), (0.78, 0.62, 0.40), (0.80, 0.66, 0.46), (0.15, 0.15, 0.17)

_fingers = tuple(
    Joint(n, kind="slide", axis="Z", anchor=(u, 0.0, 0.0), lower=0.0, upper=DEPTH, stiffness=K_FINGER, damping=0.5,
          target=0.0, initial=0.0, mass=M_FINGER, boxes=(((u, 0.0, F_LEN / 2 + F_THICK / 4), (F_THICK, F_WIDE, F_LEN - F_THICK / 2), FINGER_WHITE),),
          cylinders=(((u, 0.0, F_THICK / 2), F_THICK / 2, F_WIDE, FINGER_WHITE, "Y"),))  # rounded tip
    for n, u in zip(FINGERS, FINGER_U))

# -- casing offcut: colonial 11/16 x 2-1/4 in, 0.2 m long; profile across its width (part x), extruded along part y --
CAS_W, CAS_LEN, CAS_H = 0.057, 0.200, 0.0175
M_CASING = 0.085  # pine 0.2 m offcut (estimate from section x length x 450 kg/m3)
MU_CASING = 0.4  # finished pine on the bench top (estimate)
# (x0, x1, h0, h1) from the inner bead edge (x = -CAS_W/2) to the thick outer band, metres; h linear between.
PROFILE = (
    (0.000, 0.010, 0.0120, 0.0120),  # raised bead at the inner edge, 12 mm high, 10 mm wide
    (0.010, 0.028, 0.0060, 0.0060),  # 6 mm drop to a flat field at 6 mm, 18 mm wide
    (0.028, 0.040, 0.0060, 0.0120),  # cove rising 6 mm over 12 mm
    (0.040, 0.057, 0.0175, 0.0175),  # 5.5 mm step up to the 17.5 mm outer band, 17 mm wide
)
MIN_FEATURE_H, MIN_FEATURE_W = 0.0055, 0.010  # every step >= 5.5 mm high, every feature >= 10 mm wide


def profile_h(c: float) -> float:
    """Casing top height at c across its width (c = 0 at its centre line); 0 off the casing."""
    x = c + CAS_W / 2
    for x0, x1, h0, h1 in PROFILE:
        if x0 <= x <= x1:
            return h0 + (h1 - h0) * (x - x0) / (x1 - x0)
    return 0.0


def _casing_boxes():
    out = []
    for x0, x1, h0, h1 in PROFILE:
        lo, w, cx = min(h0, h1), x1 - x0, x0 + (x1 - x0) / 2 - CAS_W / 2
        out.append(((cx, 0.0, lo / 2), (w, CAS_LEN, lo), PINE))
        if abs(h1 - h0) > 1e-9:  # sloped face: a 3 mm plate whose top face is the slope line
            a, t = math.atan2(h1 - h0, w), 0.003
            mid = np.array([cx, 0.0, (h0 + h1) / 2]) - t / 2 * np.array([-math.sin(a), 0.0, math.cos(a)])
            out.append((tuple(float(v) for v in mid), (w / math.cos(a), CAS_LEN, t), PINE_DARK, (0.0, -a)))
    return tuple(out)


# -- layout (gauge and casing parts turned 90 deg: their part x runs along world y) ------------------------------------
CAS_X, CAS_Y = 0.400, 0.000  # casing centre (x from 0.30 to 0.50)
G0_X, G0_Y = 0.250, 0.000  # gauge standing on the table at the start
PL_X, PL_Y, PL_LEN, PL_W, PL_H = 0.050, 0.000, 0.300, 0.140, 0.019  # 1x6 plank, long axis x, far end at x = 0.20
CUT_X = PL_X + PL_LEN / 2 - 0.030  # pencilled cut line 30 mm from the plank's end
PL_TOP = TOP + PL_H
LINE_H = 0.0001
TURN = math.pi / 2

GAUGE = part("gauge", (G0_X, G0_Y, TOP), mass=M_FRAME, friction=0.6, yaw=TURN,
             boxes=(((0.0, 0.0, FRAME_Z0 + FRAME_H / 2), (FRAME_L, FRAME_W, FRAME_H), GAUGE_GREY),), joints=_fingers)
CASING = part("casing", (CAS_X, CAS_Y, TOP), mass=M_CASING, friction=MU_CASING, yaw=TURN, boxes=_casing_boxes())
PLANK = part("plank", (PL_X, PL_Y, TOP), fixed=True, friction=0.6,
             boxes=(((0.0, 0.0, PL_H / 2), (PL_LEN, PL_W, PL_H), PLANK_WOOD),
                    ((CUT_X - PL_X, 0.0, PL_H + LINE_H / 2), (0.001, PL_W, LINE_H), PENCIL)))
OBJECTS = (GAUGE, CASING, PLANK)

# -- finger latch (single source for on_step and tests) ----------------------------------------------------------------


def latch(q: float, held: float) -> float:
    """New latch (drive target) of one finger at position q: it stays where it was left and slips past FINGER_HOLD_N
    (the drive follows to within FINGER_HOLD_N / K_FINGER of the finger)."""
    lim = FINGER_HOLD_N / K_FINGER
    return min(max(held, q - lim), q + lim)


_trial: dict = {}
_ids: dict = {}


def _fresh():
    return {"latch": [0.0] * N_FINGERS, "tcps": None, "step": -1, "casing0": None}


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
    cs = labkit._part_state(env, labkit._SPECS["casing"])
    if _trial["casing0"] is None:
        _trial["casing0"] = (tuple(float(v) for v in cs.pos), _yaw(cs))
    if _trial["step"] % 20 == 19:
        gs = labkit._part_state(env, labkit._SPECS["gauge"])
        print("STATE " + json.dumps({"step": _trial["step"] + 1, **_eval(gs, cs)}), flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------
RMS_MM, MAX_MM = 1.5, 3.0  # copied profile vs the casing: RMS and worst finger over the fingers on the casing
REF_SPREAD_MM = 2.5  # bench fingers beside the casing: peak-to-peak spread about one straight line at most 2.5 mm
REF_BAND_M = 0.030  # ... those within 30 mm of its edges
EDGE_M = 0.001  # a finger reads the casing when its tip is at least 1 mm onto it
SQUARE_DEG = 3.0  # that line = how the frame lay on the bench during the press: within 3 deg of level (square)
MIN_ON_CASING = 5  # at least 5 fingers read the casing (7 or 8 do wherever it lies fully under the gauge)
CASING_SHIFT_M, CASING_YAW_DEG = 0.008, 3.0  # casing kept in place (else the score is capped at CAP_SHOVED)
CAP_SHOVED = 0.5
CUT_TOL_M, CENTRE_TOL_M = 0.006, 0.010  # gauge centre within 6 mm of the cut line, 10 mm of the plank centre line
TILT_DEG, ALONG_DEG = 3.0, 5.0  # gauge upright within 3 deg, along the cut line within 5 deg
SEAT_TOL_M = 0.003  # its bearing finger tips on the plank top (within 3 mm)
HAND_CLEAR_M = 0.12  # both TCPs more than 12 cm from the gauge centre in plan (or more than 15 cm above the plank)
PARTIAL = 0.4  # profile copied and held, gauge not stood on the plank at the cut line
PASS_SCORE = 1.0 - 1e-9
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
_FOOT = np.linspace(-F_THICK / 2, F_THICK / 2, 15)
_ROUND = F_THICK / 2 - np.sqrt(np.maximum((F_THICK / 2) ** 2 - _FOOT ** 2, 0.0))  # rounded tip: rise off its centre
_S_GRID = np.arange(-(FRAME_L / 2 - CAS_W / 2 - 2 * PITCH), FRAME_L / 2 - CAS_W / 2 - 2 * PITCH + 1e-9, 0.0005)


def _yaw(ps: PartState) -> float:
    r = ps.rotation()
    return math.atan2(float(r[1, 0]), float(r[0, 0]))


def finger_reading(c: float) -> float:
    """Height at which a finger whose centre is at c across the casing stops: its rounded tip (radius F_THICK / 2)
    resting on the highest point of the casing under it."""
    return max(max(profile_h(c + d) - r, 0.0) for d, r in zip(_FOOT, _ROUND))


_U = np.array(FINGER_U)
_A = np.column_stack([np.ones_like(_U), _U])
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
_GAP = np.abs(_CC) - CAS_W / 2 - F_THICK / 2  # how far a finger's tip is off the casing's edge
_NEAR = ~_ON & ~_EDGE & (_GAP > 0) & (_GAP <= REF_BAND_M)  # bench fingers beside the casing (the copy's reference)
_W = (_ON | _NEAR).astype(float)
_N_ON = _ON.sum(axis=1)


def profile_fit(q: dict) -> dict:
    """Lay the gauge back against the casing (as one checks a copy): over every offset s along the gauge and
    both faces, the pushes q (m) minus the casing heights under the fingers on the casing and beside it (within
    REF_BAND_M), less one straight line (how square the frame was pressed), must vanish. Candidates that meet every
    gate (count, RMS, worst, spread, slope) are screened first and the lowest RMS among them is returned; if none
    does, the lowest-RMS candidate is returned for diagnostics. Returns rms/max over the fingers on the casing,
    peak-to-peak spread of the bench fingers beside it, the line's slope (deg) and how many fingers read the casing."""
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
            "s_mm": round(s * 1000, 2), "face": int(sign), "push_mm": round(float(a[k]) * 1000, 2)}


def copied(ps: PartState) -> bool:
    f = profile_fit(ps.joints)
    return (f["on_casing"] >= MIN_ON_CASING and f["rms_mm"] <= RMS_MM and f["max_mm"] <= MAX_MM
            and f["ref_spread_mm"] <= REF_SPREAD_MM and f["slope_deg"] <= SQUARE_DEG)


def _tip_z(ps: PartState) -> float:
    """Height of the lowest finger tip (gauge upright: tip = part origin + the finger's push)."""
    r, p = ps.rotation(), np.asarray(ps.pos, float)
    return min(float((r @ np.array([u, 0.0, float(ps.joints.get(n, 0.0))]) + p)[2]) for n, u in zip(FINGERS, FINGER_U))


def _frame_centre(ps: PartState) -> np.ndarray:
    return ps.rotation() @ np.array([0.0, 0.0, FRAME_Z0 + FRAME_H / 2]) + np.asarray(ps.pos, float)


def _hands_clear(tcps, ps) -> bool:
    if tcps is None or len(tcps) < 2:
        return False
    c = _frame_centre(ps)
    return all(np.linalg.norm(np.asarray(t)[:2] - c[:2]) > HAND_CLEAR_M or np.asarray(t)[2] > PL_TOP + 0.15 for t in tcps)


def on_cut_line(ps: PartState) -> bool:
    """Gauge standing upright on the plank top across the cut line: centre on the line and the plank's centre line,
    long axis along the line, bearing finger tips on the plank top."""
    c = _frame_centre(ps)
    along = abs(math.degrees(_yaw(ps))) % 180.0
    along = abs(along - 90.0)
    return (abs(c[0] - CUT_X) <= CUT_TOL_M and abs(c[1] - PL_Y) <= CENTRE_TOL_M and ps.tilt_deg() <= TILT_DEG
            and along <= ALONG_DEG and abs(_tip_z(ps) - (PL_TOP + LINE_H)) <= SEAT_TOL_M)


def casing_kept(cs: PartState) -> bool:
    p0, y0 = _trial["casing0"] or ((CAS_X, CAS_Y, TOP), TURN)
    d = math.hypot(float(cs.pos[0]) - p0[0], float(cs.pos[1]) - p0[1])
    dyaw = abs(math.degrees(math.atan2(math.sin(_yaw(cs) - y0), math.cos(_yaw(cs) - y0))))
    return d <= CASING_SHIFT_M and dyaw <= CASING_YAW_DEG


def _eval(gs: PartState, cs: PartState) -> dict:
    f = profile_fit(gs.joints)
    c = _frame_centre(gs)
    return {"fit": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in f.items()},
            "gauge_c_mm": [round(float(v) * 1000, 1) for v in c], "tilt_deg": round(gs.tilt_deg(), 2),
            "tip_mm": round((_tip_z(gs) - TOP) * 1000, 1), "casing_mm": [round(float(v) * 1000, 1) for v in cs.pos[:2]],
            "copied": bool(copied(gs)), "on_line": bool(on_cut_line(gs)), "casing_kept": bool(casing_kept(cs)),
            "hands_clear": bool(_hands_clear(_trial["tcps"], gs))}


def score(objects: dict[str, PartState]) -> float:
    """Judged at the end of the trial. 0 unless the copied profile matches the casing laid back against it at some
    offset and face (candidates are screened against every gate, then the lowest RMS is taken): at least 5 fingers
    at least 1 mm onto the casing within RMS 1.5 mm and worst 3.0 mm of its heights, the bench fingers within 30 mm
    of its edges within a peak-to-peak spread of 2.5 mm about one straight line, and that line (how level the frame
    was pressed) within 3 deg of level. Every feature of the casing's profile is at least 5.5 mm high and 10 mm
    wide, so these gates tell a copy from an unpressed gauge, a flat or tilted press, a flat block and a plain bevel
    of the same rise. Then 0.4; 1.0 if the gauge also stands upright (within 3 deg) on the plank across the cut
    line: centre within 6 mm of the line and 10 mm of the plank's centre line, along the line within 5 deg, lowest
    finger tip within 3 mm of the plank top, and each hand's tool point more than 12 cm from the gauge centre in
    plan or more than 15 cm above the plank top. Capped at 0.5 if the casing's final position is more than 8 mm or
    its final heading more than 3 deg from where it lay. Possible: 0, 0.4, 0.5, 1.0."""
    gs, cs = objects["gauge"], objects["casing"]
    if not copied(gs):
        return 0.0
    s = 1.0 if on_cut_line(gs) and _hands_clear(_trial["tcps"], gs) else PARTIAL
    return s if casing_kept(cs) else min(s, CAP_SHOVED)


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def _set_test_trial(**kw) -> None:
    """Explicit test hook for offline scoring tests (never called by demo())."""
    for k, v in kw.items():
        if k not in _trial:
            raise KeyError(k)
        _trial[k] = v


# -- demo (open loop, constants only) ---------------------------------------------------------------------------------
GRIP_U = 0.105  # each hand takes the spine 105 mm from the centre (22 mm in from its end)
GRIP_DZ = FRAME_Z0 + 0.013  # TCP above the finger-tip plane: pad tips level with the frame bottom
SEAT_Z = TOP + CAS_H + GRIP_DZ - DEPTH  # TCP when the frame seats on the casing's outer band
PRESS_OVER = 0.035  # commanded past the seat by each hand: the arms' compliance turns it into the press force
LIFT_BACK = 0.008  # lift-off aimed 8 mm back: cancels the arms' forward spring-back as the press load comes off
PRESS_X = 0.350  # gauge over the casing, 50 mm from its near end
PUSH_NOM = DEPTH - CAS_H - FINGER_HOLD_N / K_FINGER  # bench finger push after the press (seated, relaxed)
JAWS_X = math.pi / 2  # jaws closing along x, across the 22 mm spine (left hand turned the other way: wrist reach)


def hand(side: float, park) -> list:
    """One hand's waypoints (side +1 = left hand on the +y end, -1 = right hand on the -y end); both hands run the
    same timing, so the gauge is carried level between them."""
    W = pitch_demo.Waypoint
    o, c = FINGER_OPEN, FINGER_CLOSED
    y, yaw = side * GRIP_U, -side * JAWS_X
    hi = TOP + 0.13
    place_z = PL_TOP + LINE_H + GRIP_DZ - PUSH_NOM - 0.002
    return [
        W(1.5, (G0_X, y, TOP + GRIP_DZ + 0.06), o, "over the gauge's end", yaw=yaw),
        W(1.0, (G0_X, y, TOP + GRIP_DZ), o, "jaws round the spine", yaw=yaw),
        W(0.6, (G0_X, y, TOP + GRIP_DZ), c, "grip the spine", yaw=yaw),
        W(1.2, (G0_X, y, hi), c, "lift the gauge level", yaw=yaw),
        W(1.5, (PRESS_X, y, hi), c, "carry it over the casing", yaw=yaw),
        W(1.0, (PRESS_X, y, SEAT_Z + 0.006 + DEPTH - CAS_H), c, "finger tips just over the casing", yaw=yaw),
        W(2.0, (PRESS_X, y, SEAT_Z - PRESS_OVER), c, "press square until the frame seats", yaw=yaw),
        W(0.8, (PRESS_X, y, SEAT_Z - PRESS_OVER), c, "hold it seated", yaw=yaw),
        W(0.6, (PRESS_X - LIFT_BACK, y, SEAT_Z + 0.01), c, "ease off", yaw=yaw),
        W(1.2, (PRESS_X - LIFT_BACK, y, hi), c, "lift straight off", yaw=yaw),
        W(2.0, (CUT_X, y, hi), c, "carry it over the plank", yaw=yaw),
        W(1.2, (CUT_X, y, place_z + 0.03), c, "down to the cut line", yaw=yaw),
        W(1.0, (CUT_X, y, place_z), c, "stand it on the plank", yaw=yaw),
        W(0.6, (CUT_X, y, place_z), o, "let go", yaw=yaw),
        W(0.8, (CUT_X, y, place_z + 0.08), o, "up clear of the fingers", yaw=yaw),
        W(1.5, park, o, "park clear"),
        W(8.0, park, o, "hold clear"),
    ]


def demo():
    """Open loop, fixed scene geometry, both hands in step:
    1. Each hand takes the gauge's spine near one end (jaws across the 22 mm frame) and they lift it level.
    2. They carry it over the casing and press it straight down until the frame seats on the casing: the fingers over
       the casing are pushed back to its profile, the ones over the bench to the bench.
    3. They lift it straight off; the fingers stay where the casing left them.
    4. They stand the gauge on the plank across the cut line, the copied profile kept, let go and park."""
    return {"left": hand(1.0, (0.20, 0.26, TOP + 0.20)), "right": hand(-1.0, (0.16, -0.32, TOP + 0.20))}
