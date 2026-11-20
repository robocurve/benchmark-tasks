"""A printed US-letter sheet (21.6 x 27.9 cm, 80 g/m2) lies face down on the desk, its top edge away from the robot,
as it came out of the printer. Beyond it lies a #10 window envelope (24.1 x 10.5 cm), front up, its flap open toward
the robot. The robot must fold the letter in thirds and slide it into the envelope so the printed address shows
through the window, then let go.

The letter is a reduced position-based sheet (letter_sheet.py) stepped once per control step from the measured
gripper poses and jaw openings; score() reads its end state.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --scorer task_score --instruction "Fold the letter and put it in the envelope so the address shows in the window."
"""

from __future__ import annotations

import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0  # the full score must hold 2 s, released and clear, like the scoring rule
LETTER_TASK = True  # marks this module for the embodiment hooks below

# -- layout (m; x forward from the arm-base line, y left) --------------------------------------------------------
SX, SY = 0.279, 0.216  # US letter, 11 x 8.5 in; its 27.9 cm length runs along x
X0, Y0 = 0.14, -SY / 2  # near (bottom) edge and right edge of the letter
ENV_L, ENV_H = 0.241, 0.105  # #10 envelope, 9.5 x 4.125 in
ENV_T, ENV_GAP = 0.0003, float(__import__("os").environ.get("LGAP", "0.005"))  # paper panel thickness, room inside for the folded letter
ENV_TOT = 2 * ENV_T + ENV_GAP
MOUTH_X = 0.445  # the open (top) edge of the envelope faces the robot
ENV_CX = MOUTH_X + ENV_H / 2
ENV_POS = np.array([ENV_CX, 0.0, TOP])  # envelope frame origin: centre of its footprint on the desk
ENV_ROT = np.eye(3)
MOUTH_FLEX = 0.02
FLAP_D = 0.045  # the open flap lies flat on the desk in front of the mouth
# window: 4.5 x 1.125 in, 7/8 in from the envelope's left edge and 1/2 in from its bottom edge (front up, flap edge
# at the top). In the envelope frame the top (mouth) is -x and the left edge is -y.
WIN_X = (ENV_H / 2 - 0.0127 - 0.0286, ENV_H / 2 - 0.0127)
WIN_Y = (-ENV_L / 2 + 0.0222, -ENV_L / 2 + 0.0222 + 0.1143)
# printed address block on the letter (flat-sheet coordinates: u from the bottom edge, v from the -y edge, which is
# the letter's left edge since the print faces down). 59.5-72.5 mm below the top edge, 25-100 mm from the left edge.
ADDR_U = (SX - 0.0725, SX - 0.0595)
ADDR_V = (0.025, 0.100)

WHITE = (0.96, 0.96, 0.94)

OBJECTS = [
    # the rigid part stops 2 cm short of the mouth: there the paper front panel flexes over a fingertip pushing the
    # letter home (the paper model still has the whole envelope)
    Box("envelope", pos=(ENV_CX + MOUTH_FLEX / 2, 0.0, TOP + ENV_TOT / 2), size=(ENV_H - MOUTH_FLEX, ENV_L, ENV_TOT),
        color=WHITE),
]

VARIATION = {}
TWIN_EXTRA = [
    {"name": "letter", "kind": "body", "size_mm": [279, 216, 0.1], "colour": "white",
     "real": "US-letter 80 g/m2 copy paper, laser printed (letterhead, address block, body text), lying face down",
     "pose": [X0 + SX / 2, 0.0, 0.0001]},
    {"name": "envelope_window", "kind": "decal", "size_mm": [29, 114], "colour": "clear",
     "pose": [ENV_CX + (WIN_X[0] + WIN_X[1]) / 2, (WIN_Y[0] + WIN_Y[1]) / 2, 0.0036]},
]

# -- scoring constants -----------------------------------------------------------------------------------------------
CLEAR = 0.03  # grippers 3 cm clear of the letter
OPEN_GAP = 0.02  # a jaw counts as open from a 2 cm finger gap
HOLD_S = 2.0  # the ladder must hold for 2 s after release
MOUTH_TOL = 0.002  # no part of the letter more than 2 mm out of the mouth
LADDER_EVERY = 4


def _ray_hits(origins, dirs, tris, eps=1e-12):
    """For each ray, whether it hits any triangle (Moller-Trumbore, vectorised)."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    out = np.zeros(len(origins), bool)
    for s in range(0, len(origins), 64):
        o = origins[s:s + 64, None, :]
        d = dirs[s:s + 64, None, :]
        p = np.cross(d, e2[None])
        det = (e1[None] * p).sum(-1)
        ok = np.abs(det) > eps
        inv = 1.0 / np.where(ok, det, 1.0)
        tv = o - v0[None]
        u = (tv * p).sum(-1) * inv
        q = np.cross(tv, e1[None])
        vv = (d * q).sum(-1) * inv
        t = (e2[None] * q).sum(-1) * inv
        hit = ok & (u >= 0) & (vv >= 0) & (u + vv <= 1) & (t > 1e-5)
        out[s:s + 64] = hit.any(1)
    return out


def addr_samples(n_u=4, n_v=8):
    us = np.linspace(ADDR_U[0], ADDR_U[1], n_u)
    vs = np.linspace(ADDR_V[0], ADDR_V[1], n_v)
    return [(u, v) for u in us for v in vs]


TURN_H = 0.01  # a fold runs back at least 1 cm; smaller ripples are not counted as folds
END_CURL = 0.02  # a turn within 2 cm of either end is a curled edge strip, not a fold


def thirds(S):
    """Folded in thirds: along its length the letter runs one way, back, and forward again (three flat panels), with
    the two turns within 15 mm of the one-third lines, and it is not folded across its width (still 20 cm or more
    wide). Crumpling or extra folds give more turns."""
    X = S.x.reshape(S.nx, S.ny, 3)
    c = X.mean(1)[:, :2]  # centre line along the length (rows), in plan
    a = c - c.mean(0)
    ax = np.linalg.svd(a, full_matrices=False)[2][0]
    p = a @ ax
    h = TURN_H
    turns, dirn, hi, lo = [], 0, 0, 0
    for i in range(1, len(p)):  # direction reversals that run back by at least h (a squashed crease's ripple does not)
        if dirn == 0:
            hi = i if p[i] > p[hi] else hi
            lo = i if p[i] < p[lo] else lo
            if p[i] - p[lo] > h:
                dirn, hi = 1, i
            elif p[hi] - p[i] > h:
                dirn, lo = -1, i
        elif dirn == 1:
            if p[i] > p[hi]:
                hi = i
            elif p[hi] - p[i] > h:
                turns.append(hi); dirn, lo = -1, i
        else:
            if p[i] < p[lo]:
                lo = i
            elif p[i] - p[lo] > h:
                turns.append(lo); dirn, hi = 1, i
    out = []
    for e in turns:  # the crease's place: the middle of the rows near the turning point
        j0 = e
        while j0 > 0 and abs(p[j0 - 1] - p[e]) <= h:
            j0 -= 1
        j1 = e
        while j1 < len(p) - 1 and abs(p[j1 + 1] - p[e]) <= h:
            j1 += 1
        out.append((j0 + j1) / 2)
    turns = out
    turn_u = [t * S.dx for t in turns]
    turn_u = [u for u in turn_u if END_CURL < u < SX - END_CURL]  # a curled-up edge strip is not a fold
    width = float(np.median(np.linalg.norm(X[:, -1, :2] - X[:, 0, :2], axis=1)))
    ok = (len(turn_u) == 2 and abs(turn_u[0] - SX / 3) <= 0.015 and abs(turn_u[1] - 2 * SX / 3) <= 0.015
          and width >= 0.20)
    return bool(ok), [round(u * 1000) for u in turn_u]


def letter_state(S, env_pos=ENV_POS, env_rot=ENV_ROT):
    """End-state geometry of the letter relative to the envelope (all from the sheet's node positions)."""
    x = S.x
    q = (x - env_pos) @ env_rot
    # folded small enough: the packet's own extent (along its principal axes in plan, whichever way it lies)
    tri, turns = thirds(S)
    p2 = x[:, :2] - x[:, :2].mean(0)
    ax = np.linalg.svd(p2, full_matrices=False)[2]
    short, long_ = sorted(float(np.ptp(p2 @ a)) for a in ax)
    fits = bool(tri and short <= ENV_H and long_ <= ENV_L)
    # inside: nothing more than 2 mm out of the mouth, and no paper lying on top of the front panel (paper bunched
    # against the mouth's own edge, within 3 mm of it, is still in the mouth)
    on_front = (q[:, 2] > ENV_TOT + 0.0005) & (q[:, 0] > -ENV_H / 2 + 0.003)
    inside = bool(np.all(q[:, 0] >= -ENV_H / 2 - MOUTH_TOL) and np.all(q[:, 0] <= ENV_H / 2)
                  and np.all(np.abs(q[:, 1]) <= ENV_L / 2) and not on_front.any())
    out_mm = float(max(0.0, (-ENV_H / 2 - q[:, 0]).max()) * 1000)
    # address block: where its points are, which way its print faces, and whether any paper lies over it
    tris = S.triangles()
    pts, up, n_in, n_vis = [], [], 0, 0
    for u, v in addr_samples():
        p = S.at(u, v)
        du = S.at(min(u + 0.004, SX), v) - S.at(max(u - 0.004, 0.0), v)
        dv = S.at(u, min(v + 0.004, SY)) - S.at(u, max(v - 0.004, 0.0))
        nb = np.cross(du, dv)  # the blank side's normal (+z as the sheet lies at the start)
        nprint = -nb / (np.linalg.norm(nb) + 1e-12)
        pts.append(p)
        up.append(float(nprint @ env_rot[:, 2]))
    pts = np.array(pts)
    ql = (pts - env_pos) @ env_rot
    in_win = (ql[:, 0] >= WIN_X[0]) & (ql[:, 0] <= WIN_X[1]) & (ql[:, 1] >= WIN_Y[0]) & (ql[:, 1] <= WIN_Y[1])
    facing = np.array(up) > 0.5
    # paper over a sample: a sheet triangle hit straight up from it that belongs to a later fold (paper has no
    # self-collision in this model; which panel lies on top is set by fold order)
    lay = np.maximum(S.layer, 0)
    flay = lay[S.faces].max(1)
    covered = np.zeros(len(pts), bool)
    for k, (u, v) in enumerate(addr_samples()):
        i, j = int(round(u / S.dx)), int(round(v / S.dy))
        mine = lay[S.ids[min(i, S.nx - 1), min(j, S.ny - 1)]]
        over = flay > mine
        if over.any():
            covered[k] = _ray_hits(pts[k:k + 1] + env_rot[:, 2] * 0.0001, env_rot[:, 2][None], tris[over])[0]
    shown = in_win & facing & ~covered
    return dict(fits=fits, thirds=tri, turns_mm=turns, inside=inside, out_mm=out_mm, addr_shown=float(shown.mean()),
                addr_facing=float(facing.mean()), addr_covered=float(covered.mean()), addr_in_win=float(in_win.mean()),
                span=(short, long_))


def ladder(st) -> float:
    if not st["fits"]:
        return 0.0
    s = 0.25
    if st["inside"]:
        s = 0.5
        if st["addr_shown"] >= 0.5:
            s = 0.75
        if st["addr_shown"] >= 1.0:
            s = 1.0
    return s


def _box_dist(p, T, lo, hi):
    loc = (p - T[:3, 3]) @ T[:3, :3]
    d = np.maximum(np.maximum(lo - loc, loc - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def finger_boxes(tcp, rot, aperture):
    """Both pads of one gripper as boxes in its TCP frame (x: closing axis, z: approach): 12 mm pads, 22 mm wide,
    running from the pad tips (13 mm past the TCP) 6 cm back toward the wrist."""
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rot, tcp
    a = aperture / 2
    return [(T, np.array([a, -0.011, -0.06]), np.array([a + 0.012, 0.011, 0.013])),
            (T, np.array([-a - 0.012, -0.011, -0.06]), np.array([-a, 0.011, 0.013]))]


def _arr(t):
    t = t.torch if hasattr(t, "torch") else t
    return t.detach().cpu().numpy().astype(np.float64)


def make_sheet():
    from letter_sheet import Sheet

    return Sheet(X0, Y0, (SX, SY), TOP + 0.0006, table_z=TOP,
                 envelope=dict(L=ENV_L, H=ENV_H, gap=ENV_GAP, t=ENV_T, lip=MOUTH_FLEX + 0.01, lip_z=0.005),
                 bend_k=0.2, yield_strain=0.02, mu=0.15, muc=0.0, kcomp=1.0, cmin=0.97,
                 mut=float(__import__("os").environ.get("LMUT", "0.8")))


class Tracker:
    def __init__(self):
        self._axis = {}
        self.S = None
        self.mesh = None
        self.decorated = False

    def _origin(self, base):
        return _arr(base.scene.env_origins[0])

    def _hands(self, base):
        from yam_arena.runtime import link_poses, tcp_world

        o = self._origin(base)
        tcp, rot, ap = [], [], []
        for k in ("left_robot", "right_robot"):
            rob = base.scene[k]
            lp = link_poses(rob)
            g = lp["gripper"]
            z = g[:3, 2] / np.linalg.norm(g[:3, 2])
            # closing axis: one of the gripper link's own axes (found once, with the jaws open, as the one along
            # the line between the fingertips); the fingertip line itself is unreliable once the jaws are shut
            if k not in self._axis:
                tl = lp["tip_left"][:3, 3] - lp["tip_right"][:3, 3]
                if np.linalg.norm(tl) > 0.03:
                    dots = [float(g[:3, c] @ tl) / np.linalg.norm(tl) for c in range(3)]
                    c = int(np.argmax(np.abs(dots)))
                    self._axis[k] = (c, 1.0 if dots[c] > 0 else -1.0)
            c, sg = self._axis.get(k, (0, 1.0))
            xc = sg * g[:3, c]
            xc = xc - z * (xc @ z)
            xc = xc / max(np.linalg.norm(xc), 1e-9)
            rot.append(np.stack([xc, np.cross(z, xc), z], 1))
            tcp.append(np.asarray(tcp_world(rob), float) - o)
            ids = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
            q = _arr(rob.data.joint_pos)[0, ids]
            ap.append(float(abs(q[0]) + abs(q[1])))
        return np.array(tcp), np.array(rot), np.array(ap)

    def reset(self, env):
        base = env.unwrapped
        self.dt = float(base.step_dt)
        self.S = make_sheet()
        tcp, rot, ap = self._hands(base)
        for _ in range(10):
            self.S.step(tcp, rot, ap, ENV_POS, ENV_ROT,
                        [b for k in range(2) for b in finger_boxes(tcp[k], rot[k], ap[k])], dt=self.dt)
        self.t = 0.0
        self.n = 0
        self.clear_t = 0.0
        self.hist = deque()
        self.last = dict(held=False, clear=False)
        self.state = None
        self._render(base)
        self._decorate(base)

    def step(self, env):
        if self.S is None:
            return
        base = env.unwrapped
        tcp, rot, ap = self._hands(base)
        boxes = [b for k in range(2) for b in finger_boxes(tcp[k], rot[k], ap[k])]
        self.S.step(tcp, rot, ap, ENV_POS, ENV_ROT, boxes, dt=self.dt)
        self.t += self.dt
        self._trace(tcp, rot, ap)
        self.n += 1
        held = bool(self.S.pins)
        x = self.S.x
        dist = min(float(_box_dist(x, T, lo, hi).min()) for T, lo, hi in boxes)
        clear = (not held) and bool(np.all(ap >= OPEN_GAP)) and dist >= CLEAR
        self.clear_t = self.clear_t + self.dt if clear else 0.0
        if not clear:
            self.hist.clear()
        elif self.n % LADDER_EVERY == 0 or not self.hist:
            self.state = letter_state(self.S)
            self.hist.append((self.t, ladder(self.state)))
            while self.hist and self.hist[0][0] < self.t - HOLD_S:
                self.hist.popleft()
        self.last = dict(held=held, clear=clear, hand_dist=dist, aperture=ap.tolist())
        self._render(base)

    def _trace(self, tcp, rot, ap):
        """Debug only (LETTER_TRACE=path): record the sheet's inputs so a run can be replayed offline."""
        import os

        path = os.environ.get("LETTER_TRACE")
        if not path:
            return
        if self.n == 0:
            self._tr = {k: [] for k in ("tcp", "rot", "ap")}
        for k, v in zip(("tcp", "rot", "ap"), (tcp, rot, ap)):
            self._tr[k].append(np.array(v))
        if self.n % 50 == 0:
            np.savez_compressed(path, x=self.S.x, layer=self.S.layer, **{k: np.array(v) for k, v in self._tr.items()})

    def gate(self) -> str:
        if self.S is None:
            return "no sheet"
        if self.S.torn or self.S.bad:
            return "letter torn"
        if self.last.get("held"):
            return "gripper holds the letter"
        if not self.last.get("clear"):
            return "grippers not open and clear"
        if self.clear_t < HOLD_S - 1e-6:
            return "released < 2 s ago"
        return ""

    def score(self) -> float:
        if self.gate() or not self.hist:
            return 0.0
        return float(min(v for _, v in self.hist))

    def summary(self) -> str:
        st = self.state or {}
        return (f"gate={self.gate() or 'ok'} score={self.score()} pins={ {k: len(v) for k, v in self.S.pins.items()} } "
                f"fits={st.get('fits')} inside={st.get('inside')} out_mm={st.get('out_mm', 0):.1f} "
                f"addr_shown={st.get('addr_shown', 0):.2f} facing={st.get('addr_facing', 0):.2f} "
                f"covered={st.get('addr_covered', 0):.2f} in_win={st.get('addr_in_win', 0):.2f} span={st.get('span')} "
                f"strain={self.S.max_strain:.2f} clear_t={self.clear_t:.2f} hand_dist={self.last.get('hand_dist', 0):.3f}")

    # rendering -------------------------------------------------------------------------------------------------
    def _render(self, base):
        try:
            from letter_look import render_letter

            self.mesh = render_letter(self.mesh, self.S, self._origin(base))
        except Exception as e:  # noqa: BLE001
            if not getattr(self, "_render_warned", False):
                print(f"[letter] render failed: {e!r}", flush=True)
                self._render_warned = True

    def _decorate(self, base):
        if self.decorated:
            return
        self.decorated = True
        try:
            from letter_look import decorate_envelope

            decorate_envelope(base)
        except Exception as e:  # noqa: BLE001
            print(f"[letter] envelope decoration failed: {e!r}", flush=True)


# -- embodiment hooks (shared through the class: the task module can be loaded more than once) ------------------------
def tracker():
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return None
    return getattr(ie.YamArenaEmbodiment, "_letter_tracker", None)


def _ours(emb) -> bool:
    task = getattr(emb, "task", None)
    return bool(task is not None and getattr(getattr(task, "module", None), "LETTER_TASK", False))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_letter_hooked", False):
        return
    cls._letter_tracker = Tracker()
    base_reset, base_step = cls.reset, cls._task_step

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        if not _ours(self):
            return obs
        cls._letter_tracker.reset(self._env)
        for _ in range(2):
            self._env.step(self._ready)
            cls._letter_tracker.step(self._env)
        for _ in range(ie.RESET_RENDERS):
            self._base.sim.render()
        return self._observation()

    def _task_step(self, *a, **k):
        if _ours(self):
            cls._letter_tracker.step(self._env)
        return base_step(self, *a, **k)

    cls.reset, cls._task_step, cls._letter_hooked = reset, _task_step, True


_install_hooks()


# -- scoring ---------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    tr = tracker()
    return tr.score() if tr is not None else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    tr = tracker()
    if tr is None or tr.S is None:
        return {"letter": 0}
    st = tr.state or {}
    return {"folded_to_fit": float(bool(st.get("fits"))), "inside": float(bool(st.get("inside"))),
            "address_shown": float(st.get("addr_shown", 0.0)), "released_clear": float(tr.clear_t >= HOLD_S),
            "intact": float(not (tr.S.torn or tr.S.bad))}
