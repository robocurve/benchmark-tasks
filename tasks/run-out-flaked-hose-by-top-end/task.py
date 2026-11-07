"""Hose bed mock-up on the station table: run a layered hose out of its bed from the end on top, onto a hydrant pad.

An open-ended hose bed stands on the left of the table: three dark red walls 5 cm tall round an inner space
12 cm (x) by 22 cm (y), open at its right end (the end facing the middle of the table). An 8 mm black hose with a
brass coupling on each end lies in it in three stacked layers: the bottom layer runs from the open end to the
closed end, folds back in a round bight on top of itself (the middle layer), folds again near the open end and
runs back along the top (the top layer). The top layer's coupling lies on top of the load at the closed (far
left) end. The other coupling comes out from under the load at the open end, the one nearest the middle of the
table and the pad. A flat yellow hydrant pad lies far out on the right.

The robot must put the hose's end on the pad with the whole hose run out of the bed and lying clear: no part of
it across or against another part, no tight kink, and both grippers let go. Taking the top coupling pays the
layers off in order; pulling the bottom coupling drags the layers above it out with it (score 0, latched).

Two arms: the bed is reachable only by the left arm and the pad only by the right arm (offline IK, README), so the
coupling is handed over once near the middle of the table.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the cable is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Run the black hose out of its bed and put its end on the yellow pad, then let go."

# -- hose bed (three walls, open towards -y) ---------------------------------------------------------------------
BED_X0, BED_X1 = 0.240, 0.360  # inner faces
BED_Y0, BED_Y1 = 0.220, 0.440  # open end, inner face of the closed end
BED_H, BED_T = 0.050, 0.010
BED_X = (BED_X0 + BED_X1) / 2
BED_COL = (0.55, 0.10, 0.08)
BED = [
    Box("bed_far", pos=(BED_X, BED_Y1 + BED_T / 2, TOP + BED_H / 2), size=(BED_X1 - BED_X0 + 2 * BED_T, BED_T, BED_H),
        color=BED_COL),
    Box("bed_side_near", pos=(BED_X0 - BED_T / 2, (BED_Y0 + BED_Y1) / 2, TOP + BED_H / 2),
        size=(BED_T, BED_Y1 - BED_Y0, BED_H), color=BED_COL),
    Box("bed_side_far", pos=(BED_X1 + BED_T / 2, (BED_Y0 + BED_Y1) / 2, TOP + BED_H / 2),
        size=(BED_T, BED_Y1 - BED_Y0, BED_H), color=BED_COL),
]

# -- hydrant pad ----------------------------------------------------------------------------------------------------
PAD_X, PAD_Y, PAD_W, PAD_H = 0.250, -0.400, 0.120, 0.002
PAD = Box("hydrant_pad", pos=(PAD_X, PAD_Y, TOP + PAD_H / 2), size=(PAD_W, PAD_W, PAD_H), color=(0.95, 0.80, 0.10))

# -- hose and its layered load ---------------------------------------------------------------------------------------
R = 0.004  # 8 mm hose
ZA = TOP + R + 0.0005  # bottom layer centre
DZ = 2 * R + 0.0015
ZB, ZC = ZA + DZ, ZA + 2 * DZ
BR = 0.025  # bight radius (2 cm segments bend <= 0.8 rad)
KH = math.sqrt((2 * BR) ** 2 - BR ** 2)  # leg end to bight-head centre (4.33 cm)
Y_FAR = BED_Y1 - KH - BR - R - 0.004  # where the legs meet the far bight
Y_NEAR = BED_Y0 + KH + BR + R + 0.004  # ... and the near bight
Y_TAIL = 0.200  # bottom end of the hose (its coupling sticks out of the open end)
C_PAST = 0.030  # top layer runs this far past the far legs before its coupling


def _arc(c, r, a0, a1, n=40):
    a = np.linspace(a0, a1, n)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)]


def _bight(y_leg, sgn, side):
    """Keyhole fold from a leg on x = BED_X heading sgn*y to the leg above it heading back: a 60 deg turn,
    300 deg round a head of radius BR, a 60 deg turn. side +1: the first turn goes towards +x."""
    x = BED_X
    c1 = (x + side * BR, y_leg)
    hc = (x, y_leg + sgn * KH)
    c2 = (x - side * BR, y_leg)
    a_in = math.atan2(0 - 0, -side * BR)  # leg point seen from c1: angle pi (side +1) or 0 (side -1)
    a_in = math.pi if side > 0 else 0.0
    t1 = math.atan2(hc[1] - c1[1], hc[0] - c1[0])  # tangent point direction from c1
    rot = -sgn * side  # c1 turn sense (+1 ccw) so the path heads towards the head
    p1 = _arc(c1, BR, a_in, a_in + rot * abs(_wrap(t1 - a_in)))
    h0 = math.atan2(c1[1] - hc[1], c1[0] - hc[0])
    h1 = math.atan2(c2[1] - hc[1], c2[0] - hc[0])
    span = 2 * math.pi - abs(_wrap(h1 - h0))
    p2 = _arc(hc, BR, h0, h0 - rot * span, 120)
    t2 = math.atan2(hc[1] - c2[1], hc[0] - c2[0])
    a_out = 0.0 if side > 0 else math.pi
    p3 = _arc(c2, BR, t2, t2 + rot * abs(_wrap(a_out - t2)))
    return np.vstack([p1, p2[1:], p3[1:]])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _ramp(p2d, z0, z1):
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p2d, axis=0), axis=1))]
    return np.c_[p2d, z0 + (z1 - z0) * s / s[-1]]


def load_path() -> np.ndarray:
    """Hose centreline (N,3) from the bottom end: bottom layer A, far bight, middle layer B, near bight, top C."""
    yA = np.linspace(Y_TAIL, Y_FAR, 40)
    A = np.c_[np.full_like(yA, BED_X), yA, np.full_like(yA, ZA)]
    fb = _ramp(_bight(Y_FAR, +1, +1), ZA, ZB)
    yB = np.linspace(Y_FAR, Y_NEAR, 20)
    B = np.c_[np.full_like(yB, BED_X), yB, np.full_like(yB, ZB)]
    nb = _ramp(_bight(Y_NEAR, -1, +1), ZB, ZC)
    yC = np.linspace(Y_NEAR, Y_FAR + C_PAST, 30)
    C = np.c_[np.full_like(yC, BED_X), yC, np.full_like(yC, ZC)]
    return np.vstack([A, fb[1:], B[1:], nb[1:], C[1:]])


def _plen(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


HOSE_LEN = round(_plen(load_path()) / 0.02) * 0.02
COUPLING = (0.030, 0.016, 0.012)
BRASS = (0.78, 0.60, 0.22)
HOSE = cable("hose", start=(0.47, 0.20, TOP + R + 0.0005), yaw=-math.pi / 2, length=HOSE_LEN, radius=R,
             start_plug=Plug(size=COUPLING, mass=0.012, color=BRASS), end_plug=Plug(size=COUPLING, mass=0.012, color=BRASS),
             cable_mass=0.050, color=(0.08, 0.08, 0.08), bend_stiffness=0.003, bend_damping=0.0005)

OBJECTS = [*BED, PAD, HOSE]


def _shape(emb):
    cableshape.preshape(emb, HOSE, load_path(), hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)
    _reset_episode()


cableshape.on_reset(_shape)

# -- env handle (gripper release check) --------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_hosebed_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._hosebed_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ---------------------------------------------------------------------------------------------------------
def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * t for t in np.linspace(0, 1, k + 1)[1:]]
    return np.array(out)


def hose_points(objects) -> np.ndarray:
    """Dense centreline from the bottom coupling's outer face to the top coupling's outer face."""
    h = objects["hose"]
    pts = [h.start_plug - h.start_dir * COUPLING[0] / 2] if h.start_plug is not None else []
    pts += list(h.polyline())
    if h.end_plug is not None:
        pts.append(h.end_plug + h.end_dir * COUPLING[0] / 2)
    return _dense(np.asarray(pts, float))


def in_bed(p) -> np.ndarray:
    p = np.atleast_2d(p)
    return (p[:, 0] > BED_X0) & (p[:, 0] < BED_X1) & (p[:, 1] > BED_Y0) & (p[:, 1] < BED_Y1)


def top_on_pad(objects) -> bool:
    c = np.asarray(objects["hose"].end_plug, float)
    return bool(abs(c[0] - PAD_X) < PAD_W / 2 and abs(c[1] - PAD_Y) < PAD_W / 2 and c[2] < TOP + 0.03)


CROSS_GAP = 0.012  # two parts of the hose closer than this (top view) lie across or against each other
CROSS_ALONG = 0.08  # ... when they are at least this far apart along the hose
KINK_R = 0.030  # tightest allowed bend radius


def crossings(objects) -> int:
    """Pairs of points of the run-out hose (outside the bed) at least CROSS_ALONG apart along the hose and closer
    than CROSS_GAP in top view: the hose lies across or against itself."""
    p = hose_points(objects)
    out = ~in_bed(p)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    q, s = p[out][:, :2], s[out]
    d = np.linalg.norm(q[:, None, :] - q[None, :, :], axis=2)
    far = np.abs(s[:, None] - s[None, :]) >= CROSS_ALONG
    return int(((d < CROSS_GAP) & far).sum() // 2)


def min_bend_radius(objects) -> float:
    """Tightest bend of the run-out hose: segment length / turn angle between neighbouring segments, over the
    joints whose segments lie outside the bed."""
    pts = np.asarray(objects["hose"].points, float)
    d = np.diff(pts, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    ang = np.arccos(np.clip((d[:-1] * d[1:]).sum(axis=1), -1.0, 1.0))
    out = ~in_bed(pts)
    ok = out[:-2] & out[1:-1] & out[2:]
    if not ok.any():
        return 1.0
    return float(objects["hose"].seg_len / max(ang[ok].max(), 1e-9))


def length_in_bed(objects) -> float:
    p = hose_points(objects)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    return float(seg[in_bed((p[:-1] + p[1:]) / 2)].sum())


def clean(objects) -> bool:
    return crossings(objects) == 0 and min_bend_radius(objects) >= KINK_R


RELEASE_DIST = 0.03
LEAVE_MARGIN = 0.02  # a hose segment has left the bed once it is this far outside the bed's inner outline (top view)

_STATE = {"violated": False, "why": "", "inbed0": None}


def _reset_episode():
    _STATE["violated"] = False
    _STATE["why"] = ""
    _STATE["inbed0"] = None


def _beyond(p) -> np.ndarray:
    p = np.atleast_2d(p)
    m = LEAVE_MARGIN
    return ~((p[:, 0] > BED_X0 - m) & (p[:, 0] < BED_X1 + m) & (p[:, 1] > BED_Y0 - m) & (p[:, 1] < BED_Y1 + m))


def _track(objects) -> None:
    """Latch: a segment that started in the bed left it (2 cm beyond the inner outline) while the top coupling was
    still over the bed floor: the load was pulled from the wrong end or dragged out, not paid off from the top."""
    h = objects["hose"]
    if not np.isfinite(h.points).all() or h.end_plug is None or not np.isfinite(h.end_plug).all():
        _STATE["violated"], _STATE["why"] = True, "nan"
        return
    pts = np.asarray(h.points, float)
    if _STATE["inbed0"] is None:
        _STATE["inbed0"] = in_bed(pts)
    top_in = bool(in_bed(np.asarray(h.end_plug, float))[0])
    if top_in and (_STATE["inbed0"] & _beyond(pts)).any() and not _STATE["violated"]:
        _STATE["violated"], _STATE["why"] = True, "load left the bed before the top coupling"


def released(objects) -> bool:
    pts = hose_points(objects)
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


RIGHT_Y = -0.30  # 0.3: the top coupling lies at least this far right of the midline (only the right arm reaches)


def top_right_side(objects) -> bool:
    c = np.asarray(objects["hose"].end_plug, float)
    return bool(c[1] < RIGHT_Y and c[2] < TOP + 0.03)


def score(objects) -> float:
    """0 if: any part of the hose that started in the bed left it (2 cm beyond its inner outline) while the top
    coupling was still over the bed floor (latched);
    a gripper TCP is within 3 cm of the hose or a coupling at the end; the hose already out of the bed lies across
    or against itself (two parts 8 cm or more apart along it within 1.2 cm, top view) or bends tighter than 3 cm
    radius; or the top coupling is not down on the table at least 30 cm right of the midline. Otherwise 0.3; 0.6
    with the top coupling's centre on the pad; 1.0 with it on the pad and no part of the hose or couplings over the
    bed's inner floor."""
    _track(objects)
    if _STATE["violated"] or not released(objects) or not clean(objects) or not top_right_side(objects):
        return 0.0
    if not top_on_pad(objects):
        return 0.3
    if length_in_bed(objects) > 0.0:
        return 0.6
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["hose"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Top coupling's distance (m, top view) from the pad centre; 1.0 once the wrong-end latch fired."""
    _track(objects)
    if _STATE["violated"]:
        return 1.0
    c = np.asarray(objects["hose"].end_plug, float)
    return float(np.hypot(c[0] - PAD_X, c[1] - PAD_Y))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    _track(objects)
    return {
        "right_end": int(not _STATE["violated"]),
        "top_out_of_bed": int(not in_bed(np.asarray(objects["hose"].end_plug, float))[0]),
        "top_on_pad": int(top_on_pad(objects)),
        "fraction_out": float(1.0 - length_in_bed(objects) / HOSE_LEN),
        "clean_lay": int(clean(objects)),
        "released": int(released(objects)),
    }


VARIATION = {
    "group:bed": {"dx": 0.01, "dy": 0.01, "yaw": 0.0, "with": [b.name for b in BED] + ["hose"]},
    "fixed:pad": {"dx": 0.01, "dy": 0.01, "with": ["hydrant_pad"]},
}

TWIN_EXTRA = []

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------------
GRIP_OPEN = -0.016
HAND_X, HAND_Y = 0.280, 0.000  # handover: coupling set down on the midline
TAIL_TO = (0.150, 0.150)  # where the left arm lays the bottom coupling at the end
TAIL_IN_BED = (0.300, 0.300)  # tier tests: bottom coupling laid back in the bed


def demo_script(ctx):
    """Left arm pinches the top coupling (fingers across it), lifts it and draws it out over the open end to the
    middle of the table, sets it down and lets go. Right arm picks it up there, carries it far right onto the pad,
    sets it down and lets go. HOSE_CHEAT selects a test variant (see README)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("HOSE_CHEAT", "")
    yaw = -math.pi / 2  # fingers close along x; wrist camera looks towards -y (where the hose is going)

    def log(tag):
        st = ctx.states()
        h = st["hose"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: top={np.round(h.end_plug, 3)} bot={np.round(h.start_plug, 3)} "
              f"inbed={length_in_bed(st):.3f} cross={crossings(st)} rmin={min_bend_radius(st):.3f} "
              f"viol={_STATE['violated']}:{_STATE['why']} rel={released(st)} score={score(st):.2f}", flush=True)
        if os.environ.get("HOSE_DUMP"):
            with open(os.environ["HOSE_DUMP"], "a") as f:
                f.write(repr({"t": round(ctx.t, 2), "tag": tag, "pts": np.round(h.points, 4).tolist(),
                              "top": np.round(h.end_plug, 4).tolist(), "bot": np.round(h.start_plug, 4).tolist()}) + "\n")

    def plug_of(which):
        h = ctx.states()["hose"]
        return np.asarray(h.end_plug if which == "top" else h.start_plug, float)

    def take(arm, which):
        c = plug_of(which)
        yield from arm.move((c[0], c[1], TOP + 0.09), yaw=yaw, finger=GRIP_OPEN, duration=1.5)
        c = plug_of(which)
        yield from arm.move((c[0], c[1], c[2] + 0.006), yaw=yaw, finger=GRIP_OPEN, duration=1.0)
        yield from arm.grip(FINGER_CLOSED, 0.5)
        yield from arm.move((c[0], c[1], TOP + 0.08), yaw=yaw, duration=0.8)

    def put(arm, xy, which, let_go=True, slide=0.0, yw=None):
        yw = yaw if yw is None else yw
        for it in range(2):  # closed loop on the live coupling
            c = plug_of(which)
            t = arm.tcp()
            tgt = np.array([t[0], t[1]]) + (np.asarray(xy) - c[:2])
            yield from arm.move((tgt[0], tgt[1], TOP + 0.03), yaw=yw, duration=0.8 if it == 0 else 0.5)
        yield from arm.move((tgt[0], tgt[1], TOP + 0.014), yaw=yw, duration=0.6)
        if slide:  # draw the end on along the table so the hose behind it lies straight
            tgt = tgt + np.array([0.0, -slide])
            yield from arm.move((tgt[0], tgt[1], TOP + 0.014), yaw=yw, duration=1.0)
        if not let_go:
            return
        yield from arm.grip(GRIP_OPEN, 0.4)
        yield from arm.move((tgt[0], tgt[1], TOP + 0.12), yaw=yw, duration=0.7)

    def lay_tail(to=TAIL_TO):
        h = ctx.states()["hose"]
        c, d = np.asarray(h.start_plug, float), np.asarray(h.start_dir, float)
        gy = (math.atan2(d[1], d[0]) + math.pi / 2) % math.pi - math.pi / 2
        yield from L.move((c[0], c[1], TOP + 0.09), yaw=gy, finger=GRIP_OPEN, duration=1.5)
        yield from L.move((c[0], c[1], c[2] + 0.006), yaw=gy, finger=GRIP_OPEN, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.5)
        yield from L.move((c[0], c[1], TOP + 0.04), yaw=gy, duration=0.6)
        yield from L.move(to + (TOP + 0.04,), yaw=gy, duration=2.5)
        yield from L.move(to + (TOP + 0.014,), yaw=gy, duration=0.5)
        yield from L.grip(GRIP_OPEN, 0.4)
        yield from L.move(to + (TOP + 0.12,), yaw=gy, duration=0.7)
        log("tail laid")

    log("start")
    if cheat == "none":
        yield from L.wait(4.0)
        log("end")
        return
    which = "bottom" if cheat == "bottom" else "top"
    if cheat == "dump":  # grab the middle of the load and drag the whole pile out of the open end first
        p = np.asarray(ctx.states()["hose"].points, float)
        i = int(np.argmax(p[:, 2] - 0.2 * np.abs(p[:, 1] - 0.33)))
        g = p[i]
        yield from L.move((g[0], g[1], TOP + 0.09), yaw=yaw, finger=GRIP_OPEN, duration=1.5)
        yield from L.move((g[0], g[1], g[2] + 0.004), yaw=yaw, finger=GRIP_OPEN, duration=1.0)
        yield from L.grip(FINGER_CLOSED, 0.5)
        yield from L.move((g[0], g[1], TOP + 0.08), yaw=yaw, duration=0.8)
        yield from L.move((0.30, 0.08, TOP + 0.06), yaw=yaw, duration=2.5)
        yield from L.move((0.30, 0.08, TOP + 0.02), yaw=yaw, duration=0.5)
        yield from L.grip(GRIP_OPEN, 0.4)
        yield from L.move((0.30, 0.10, TOP + 0.12), yaw=0.0, duration=0.7)
        log("pile dumped")
    yield from take(L, which)
    log("left took")
    c = plug_of(which)
    ly = yaw
    if which == "top":  # the top coupling lies pointing into the bed: turn it to point the way the hose runs out
        ly = math.pi / 2
        yield from L.move((c[0] - 0.01, c[1] - 0.06, TOP + 0.10), yaw=0.0, duration=1.0)
        yield from L.move((c[0] - 0.01, c[1] - 0.10, TOP + 0.10), yaw=ly, duration=1.0)
    yield from L.move((c[0] - 0.01, 0.20, TOP + 0.08), yaw=ly, duration=2.0)
    yield from L.move((HAND_X, HAND_Y + 0.05, TOP + 0.06), yaw=ly, duration=2.5)
    yield from put(L, (HAND_X, HAND_Y), which, yw=ly)
    yield from L.move((0.25, 0.25, TOP + 0.15), yaw=0.0, duration=1.0)
    log("handed over")
    if cheat == "onearm":
        yield from L.wait(3.0)
        log("end")
        return
    yield from take(Rt, which)
    yield from Rt.move((0.30, -0.18, TOP + 0.06), yaw=yaw, duration=2.5)
    tail_to = TAIL_TO
    if cheat == "short":  # tier test: end set down right of the midline, short of the pad
        yield from Rt.move((0.40, -0.30, TOP + 0.05), yaw=yaw, duration=3.0)
        yield from put(Rt, (0.40, -0.32), which, let_go=False)
        tail_to = TAIL_IN_BED
    elif cheat == "near":  # tier test: end on the pad, the bottom coupling laid back in the bed
        yield from Rt.move((PAD_X, PAD_Y + 0.06, TOP + 0.05), yaw=yaw, duration=3.0)
        yield from put(Rt, (PAD_X, PAD_Y + 0.03), which, let_go=False, slide=0.04)
        tail_to = TAIL_IN_BED
    else:
        yield from Rt.move((PAD_X, PAD_Y + 0.06, TOP + 0.05), yaw=yaw, duration=3.0)
        log("at pad")
        yield from put(Rt, (PAD_X, PAD_Y + 0.03), which, let_go=False, slide=0.04)
    if which == "top":  # the last of the bottom layer trails out doubled back: left arm lays the tail clear
        yield from lay_tail(tail_to)
    if cheat != "hold":
        yield from Rt.grip(GRIP_OPEN, 0.4)
        t = Rt.tcp()
        yield from Rt.move((t[0], t[1], TOP + 0.12), yaw=yaw, duration=0.7)
    if cheat == "hold":
        yield from Rt.wait(3.0)
        log("end")
        return
    yield from Rt.move((0.25, -0.30, TOP + 0.15), yaw=0.0, duration=1.0)
    for _ in range(4):
        yield from L.wait(0.5)
    log("end")
