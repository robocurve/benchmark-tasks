"""Scripted full-score demo (and negative variants) for the carboy-cap task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the poses of the bottle, cap
and funnel and the cap's thread state through ``capkit.bench_state()`` (read-only, like an operator watching the
bench: where the parts are, how far the cap has come down); it never writes a pose, a joint state or a score. Its
joint commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF joint limits.

Plan:
1. Right hand: pinch the funnel's rim tab (fingers above and below it, approach along the tab toward the funnel's
   axis), lift the funnel straight up out of the neck and carry it over the tray, still holding it.
2. Left hand (meanwhile): take the cap from the bench from above (fingers across the skirt), lift it, and once the
   funnel is out set it straight down onto the neck (it rests on the thread start, centred by its skirt).
3. Left hand: screw the cap on in strokes: turn it clockwise (seen from above) about its axis while following its
   height down, let go, turn back open, grip again; until the liner meets the lip (one model turn) and the cap stops.
4. Left hand off and back; right hand lowers the funnel into the tray and lets go; both arms back to READY.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))

import fastik  # noqa: E402
import layout as L  # noqa: E402

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
FINGER_OPEN = 0.04695  # m per finger at grip 1.0
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
PAD_TIP = 0.013  # pad tips past the TCP
CAP_SQUEEZE = 0.0035  # fingers commanded this far inside the cap's skirt: about 17 N per finger
TAB_SQUEEZE = 0.0030  # fingers commanded closed past the 3 mm tab: about 7.5 N per finger (clipped at closed)
TIP_R = 0.041  # funnel tab pinch: pad tips this far from the funnel axis (tab spans 37.5..55.5 mm)
STROKE_DEG = 200.0  # largest wrist turn per screwing stroke (joint 6 has 240 deg)
BACK_DEG = 230.0  # largest turn back open between strokes
STROKE_S = 2.2
CAP_OPEN_HALF = L.CAP_R_OUT + 0.010  # finger half-opening between strokes
FUNNEL_LIFT = 0.055  # stem tip 41.7 mm below the lip at rest
SEAT_MARGIN_DEG = 1.0  # stop once the liner meets the lip (one-handed: more torque would turn the bottle)
def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


class Stop(Exception):
    pass


def _rotvec(r):
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(r).as_rotvec()


def _rotm(v):
    from scipy.spatial.transform import Rotation

    return Rotation.from_rotvec(v).as_matrix()


class Arm:
    """One arm: smooth Cartesian moves (URDF IK), each starting where the last command left it."""

    def __init__(self, name, base_pos):
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
        self.offset = np.zeros(6)
        self.measured = self.q.copy()
        self.measured_grip = 1.0
        self.comp = True
        self.comp_band = None
        self.label = ""
        self.worst_mm = 0.0
        _, _, lo, hi, _ = fastik._chain()
        self.lo, self.hi = lo, hi
        self.last_cmd = None

    @property
    def cmd(self):
        c = np.r_[np.clip(self.q + self.offset, self.lo, self.hi), self.grip]
        lim = np.r_[RATE * 0.05 * (self.hi - self.lo), RATE * 0.05]
        if self.last_cmd is not None:
            c = np.clip(c, self.last_cmd - lim, self.last_cmd + lim)
        return c

    def commit(self, c):
        self.last_cmd = np.asarray(c, float).copy()

    def set_comp(self, mode):
        """True: joint integral (free moves and carrying); 'hold': offsets frozen (in contact); False: offsets decay."""
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.comp == "hold":
            pass
        elif self.comp:
            self.offset = np.clip(
                self.offset + COMP_GAIN * (self.q - self.measured), -0.25, 0.25
            )
            if self.comp_band is not None:  # in contact: integral kept within a band (bounds the push force)
                self.offset = np.clip(self.offset, self.comp_band[0], self.comp_band[1])
        else:
            self.offset *= 0.85

    def relax(self):
        """Command the arm to where it is (keeping the sag offsets): releases the spring force it stores against the
        held part before the fingers open."""
        self.set_comp("hold")
        self.q = self.measured.copy()
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
        """IK from the current joint angles; other seeds only if that fails, and then the solution nearest the current
        angles (no jump to another arm configuration in mid-move)."""
        from yam_arena.constants import READY

        q, pe, re = fastik.solve(pos, rot, self.q, base=self.base, iters=80)
        if pe < 2e-4 and re < 3e-3:
            return q, pe, re
        q, pe, re = fastik.solve(pos, rot, q, base=self.base, iters=200)
        best = (q, pe, re)
        if pe < 5e-4 and re < 5e-3:
            return best
        for s in (np.array(READY, float), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])):
            q2, pe2, re2 = fastik.solve(pos, rot, s, base=self.base, iters=200)
            cost2 = pe2 + 0.1 * re2 + 0.002 * np.abs(q2 - self.q).sum()
            if cost2 < best[1] + 0.1 * best[2] + 0.002 * np.abs(best[0] - self.q).sum():
                best = (q2, pe2, re2)
        return best

    def set_pose(self, pos, rot, tol=0.004):
        q, pe, re = self.solve(pos, rot)
        if pe > tol or re > math.radians(4):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label="", tol=0.004):
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(
                p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s), tol=tol
            )
            yield

    def settle(self, tol=0.003, max_s=2.5, label=""):
        """Hold the command until the measured TCP is within ``tol`` of it (the joint rate limit and the PD lag can
        leave the arm behind after a long move)."""
        self.label = label or self.label
        for _ in range(int(max_s * CONTROL_HZ)):
            if np.linalg.norm(self.measured_tcp()[0] - self.pos) < tol:
                return
            yield

    def set_grip(self, grip, duration=0.8, label=""):
        """Open / close the fingers with the arm's joint command held (no new IK solve)."""
        from yam_arena import ik

        self.label = label or self.label
        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.grip = g0 + ik.smoothstep(k / n) * (grip - g0)
            yield

    def move_q(self, q_target, duration, grip=None, label=""):
        """Joint-space move (smoothstep) to ``q_target`` (used to change arm configuration before a Cartesian
        approach)."""
        from yam_arena import ik

        self.label = label or self.label
        q0, q1 = self.q.copy(), np.clip(np.asarray(q_target, float), self.lo, self.hi)
        g0 = self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield

    def home(self, duration=2.5, label=""):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or f"{self.name} home"
        q0, q1 = self.q.copy(), np.array(READY, float)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield


def wait(seconds):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(a, b):
    """Run two arm generators in parallel (one control step each per yield)."""
    gens = [a, b]
    while gens:
        alive = []
        for g in gens:
            try:
                next(g)
                alive.append(g)
            except StopIteration:
                pass
        gens = alive
        if gens:
            yield






class Ctx:
    def __init__(self, emb=None):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.arms = {"left": self.left, "right": self.right}
        self.log = []
        self.step = 0
        self.grip_peak = {"left": 0.0, "right": 0.0}

    def state(self):
        import capkit

        return capkit.bench_state()

    def detail(self):
        import capkit

        return capkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if getattr(self, "replay", None) is not None:  # open-loop replay of a recorded command trace
            a = np.asarray(self.replay[min(self.step, len(self.replay) - 1)], float)
            self.left.commit(a[:7])
            self.right.commit(a[7:])
            return Action(data=a.copy())
        cl, cr = self.left.cmd, self.right.cmd
        self.left.commit(cl)
        self.right.commit(cr)
        return Action(data=pack(cl, cr))

    def grip_force(self):
        """Estimated squeeze per finger (N): finger drive stiffness x (measured opening - commanded opening)."""
        from yam_arena.constants import GRIPPER

        out = {}
        for n, a in self.arms.items():
            if a.last_cmd is None:
                out[n] = 0.0
                continue
            f = GRIPPER["stiffness"] * max(0.0, (a.measured_grip - a.last_cmd[6]) * FINGER_OPEN)
            out[n] = min(f, GRIPPER["effort"])
            self.grip_peak[n] = max(self.grip_peak[n], out[n])
        return out

    def note(self, tag, **kw):
        d = self.detail()
        row = dict(at=tag, step=self.step, level=d.get("level"), thread=d.get("thread"), turns=d.get("turns"),
                   gap=d.get("gap_mm"), cap_r=d.get("cap_r_mm"), neck=d.get("funnel_in_neck"),
                   tray=d.get("funnel_in_tray"), veto=d.get("veto"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def frame(app, close):
    """Gripper rotation from its approach axis (z) and finger-closing axis (x), both in the station frame."""
    z = np.asarray(app, float) / np.linalg.norm(app)
    x = np.asarray(close, float)
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def down(psi_deg):
    """Straight-down TCP with the fingers closing along the horizontal direction at yaw ``psi_deg``."""
    a = math.radians(psi_deg)
    return frame([0.0, 0.0, -1.0], [math.cos(a), math.sin(a), 0.0])


def feasible(arm, pos, rot, q0=None, seeds=(), tol=1.5e-3, rtol_deg=1.5):
    q, pe, re = fastik.solve(pos, rot, arm.q if q0 is None else q0, base=arm.base, iters=150)
    if pe < tol and re < math.radians(rtol_deg):
        return True, q
    for s in seeds:
        q2, pe2, re2 = fastik.solve(pos, rot, np.asarray(s, float), base=arm.base, iters=200)
        if pe2 < tol and re2 < math.radians(rtol_deg):
            return True, q2
    return False, q


# --------------------------------------------------------------------------------------------------- funnel
def tab_pose(c, back=0.0, lift=0.0, pitch=0.0):
    """Right-hand TCP pinching the funnel's rim tab: approach along the tab toward the funnel's axis (pitched down
    ``pitch`` deg), fingers closing vertically; ``back`` m back along the approach, ``lift`` m up."""
    s = c.state()
    fp, fr = s["funnel"]
    u = fr[:, 0].copy()
    u[2] = 0.0
    u /= np.linalg.norm(u)
    a = math.radians(pitch)
    app = -math.cos(a) * u + np.array([0.0, 0.0, -math.sin(a)])
    tab_z = fp[2] + fr[2, 2] * (L.F_CONE_H - L.F_TAB[2] / 2)
    axis_pt = np.array([fp[0], fp[1], tab_z])
    tip = axis_pt + TIP_R * u
    p = tip - (PAD_TIP + back) * app + np.array([0.0, 0.0, lift])
    return p, frame(app, [0.0, 0.0, 1.0])


def plan_tab(c):
    arm = c.right
    for pitch in (0.0, 10.0, 20.0, 30.0):
        ok1, q1 = feasible(arm, *tab_pose(c, back=0.04, pitch=pitch))
        ok2, q2 = feasible(arm, *tab_pose(c, pitch=pitch), q0=q1)
        ok3, _ = feasible(arm, *tab_pose(c, lift=FUNNEL_LIFT, pitch=pitch), q0=q2)
        if ok1 and ok2 and ok3:
            return pitch
    raise Stop("funnel tab not reachable")


def funnel_out(c):
    """Right hand: pinch the tab, lift the funnel out of the neck, carry it over the tray (held)."""
    arm = c.right
    arm.set_comp(True)
    pitch = plan_tab(c)
    c.tab_pitch = pitch
    p, r = tab_pose(c, back=0.05, pitch=pitch)
    yield from arm.move(p, r, 2.5, grip_of(0.012), "right to funnel tab", tol=0.006)
    yield from arm.settle(0.004, 2.5, "right settle at tab")
    p, r = tab_pose(c, pitch=pitch)
    yield from arm.move(p, r, 1.2, None, "right onto tab")
    yield from arm.settle(0.002, 1.5, "right settle on tab")
    yield from arm.set_grip(0.0, 0.6, "right pinch tab")
    yield from wait(0.2)
    c.note("tab pinched")
    p0, r0 = arm.pos.copy(), arm.rot.copy()
    yield from arm.move(p0 + np.array([0.0, 0.0, FUNNEL_LIFT]), r0, 1.6, None, "lift funnel")
    yield from arm.settle(0.004, 1.5, "right settle lifted")
    c.note("funnel lifted")
    if c.state()["funnel_in_neck"]:
        raise Stop("funnel still in the neck after the lift")
    # carry over the tray: funnel axis above the tray's centre, stem tip 4 cm above the rim; the hand may turn
    # the held funnel about the vertical so that the hold, the lowering and the back-off are all reachable
    s = c.state()
    fp, fr = s["funnel"]
    tcp = arm.measured_tcp()[0]
    off = tcp - fp  # TCP relative to the stem top (funnel hangs from the tab)
    choice = None
    seeds = (arm.q, np.array([-0.6, 1.2, 0.8, 0.3, -0.4, -1.0]), np.array([-0.3, 1.0, 0.9, -0.2, 0.0, 0.0]))
    for shift in ((0.0, 0.0), (0.0, -0.02), (-0.02, 0.0), (0.02, 0.0), (-0.02, -0.02), (0.0, -0.035), (-0.03, -0.035)):
        hi, lo = tray_points(shift)
        for dl in (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0):
            rr = L.rz(dl)
            app = (rr @ r0)[:, 2]
            poses = [(hi + rr @ off, rr @ r0), (lo + rr @ off, rr @ r0),
                     (lo + rr @ off - 0.035 * app, rr @ r0)]
            good, q = feasible(arm, *poses[0], seeds=seeds, tol=3e-3, rtol_deg=3.0)
            for p, r in poses[1:]:
                if good:
                    good, q = feasible(arm, p, r, q0=q, tol=3e-3, rtol_deg=3.0)
            if good:
                choice = (dl, shift)
                break
        if choice is not None:
            break
    if choice is None:  # nothing passed the check: plain carry (the moves themselves accept 4 mm)
        choice = (0.0, (0.0, 0.0))
        c.note("carry search found nothing; plain carry")
    dl, shift = choice
    rr = L.rz(dl)
    hi, _ = tray_points(shift)
    c.funnel_off, c.funnel_rot, c.tray_shift = rr @ off, rr @ r0, shift
    mid = (arm.pos + hi + rr @ off) / 2 + np.array([0.0, 0.0, 0.03])
    yield from arm.move(mid, L.rz(dl / 2) @ r0, 1.5, None, "carry funnel")
    yield from arm.move(hi + rr @ off, rr @ r0, 1.5, None, "funnel over tray")
    c.note("funnel held over tray", turn_deg=dl, shift=list(shift))


def tray_points(shift=(0.0, 0.0)):
    """Funnel stem-top targets over the tray (+ ``shift``): held (stem tip 4 cm above the rim) and lowered (stem tip
    5 mm below the rim)."""
    x = (L.TRAY_X[0] + L.TRAY_X[1]) / 2 + 0.02 + shift[0]  # nominal: left of the tray's centre, clear of the
    y = (L.TRAY_Y[0] + L.TRAY_Y[1]) / 2 + 0.035 + shift[1]  # right arm at READY in the overhead view
    # lowered only until the stem tip is 5 mm inside the tray: lower, the gripper housing (13 cm behind the TCP)
    # would come down on the tray's front wall; the funnel then drops about 5 cm onto the tray bottom
    return (np.array([x, y, L.TOP + L.TRAY_Z[1] + 0.04 + L.F_STEM_L]),
            np.array([x, y, L.TOP + L.TRAY_Z[1] - 0.005 + L.F_STEM_L]))


def funnel_drop(c):
    """Right hand: lower the funnel into the tray, let go, back off."""
    arm = c.right
    _, lo = tray_points(c.tray_shift)
    yield from arm.move(lo + c.funnel_off, c.funnel_rot, 1.5, None, "lower funnel into tray", tol=0.006)
    yield from arm.set_grip(0.6, 0.5, "right let go of funnel")
    yield from wait(0.3)
    app = arm.rot[:, 2]
    # back off along the approach first (fingers slide off the tab), as far and as high as the arm reaches
    target = None
    for back, up in ((0.06, 0.03), (0.05, 0.02), (0.045, 0.0), (0.035, 0.0), (0.03, -0.005)):
        cand = arm.pos - back * app + np.array([0.0, 0.0, up])
        q, pe, re = arm.solve(cand, arm.rot)
        if pe < 0.003 and re < math.radians(3):
            target = cand
            break
    if target is None:
        raise Stop("no reachable back-off from the funnel tab")
    moved = round(float(np.linalg.norm(target - arm.pos)) * 1e3, 1)
    yield from arm.move(target, arm.rot, 0.9, None, "right back off", tol=0.01)
    # then straight up, so that the arm's way home does not sweep through the tray
    for up in (0.10, 0.08, 0.06, 0.04):
        cand = arm.pos + np.array([0.0, 0.0, up])
        q, pe, re = arm.solve(cand, arm.rot)
        if pe < 0.003 and re < math.radians(3):
            yield from arm.move(cand, arm.rot, 0.8, None, "right up from tray", tol=0.01)
            break
    c.note("funnel released", back_mm=moved)


# --------------------------------------------------------------------------------------------------- cap
def cap_center(c):
    """Measured cap liner-face centre (station) and its axis."""
    p, r = c.state()["cap"]
    return p, r[:, 2]


def pick_psi(c, arm, pos):
    """Closing-axis yaw for a straight-down grasp at ``pos``: the reachable one with the least wrist travel."""
    best = None
    for psi in range(-90, 90, 15):
        ok, q = feasible(arm, pos, down(psi))
        if ok:
            cost = abs(q[5] - arm.q[5])
            if best is None or cost < best[0]:
                best = (cost, psi)
    if best is None:
        raise Stop(f"{arm.name}: no straight-down grasp at {np.round(pos, 3).tolist()}")
    return best[1]


def cap_pick(c):
    arm = c.left
    arm.set_comp(True)
    cp, _ = cap_center(c)
    grasp = np.array([cp[0], cp[1], L.TOP + 0.004 + PAD_TIP])  # pad tips 4 mm above the bench
    psi = pick_psi(c, arm, grasp)
    yield from arm.move(grasp + np.array([0.0, 0.0, 0.06]), down(psi), 2.5, grip_of(CAP_OPEN_HALF + 0.01),
                        "left over cap", tol=0.006)
    cp, _ = cap_center(c)
    grasp = np.array([cp[0], cp[1], L.TOP + 0.004 + PAD_TIP])
    yield from arm.move(grasp, down(psi), 1.2, None, "left down to cap")
    yield from arm.settle(0.002, 1.5, "left settle at cap")
    yield from arm.set_grip(grip_of(L.CAP_R_OUT - CAP_SQUEEZE), 0.6, "left grip cap")
    yield from wait(0.2)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.07]), arm.rot, 1.2, None, "lift cap")
    c.note("cap lifted")


def lip_now(c):
    bp, br = c.state()["bottle"]
    return bp + br @ np.array([0.0, 0.0, L.H_LIP]), br


def grasp_offset(c, arm):
    """TCP relative to the cap's liner centre (station frame)."""
    cp, _ = cap_center(c)
    return arm.measured_tcp()[0] - cp


def cap_place(c):
    """Left hand: carry the cap over the neck and set it straight down onto the thread start."""
    arm = c.left
    off = grasp_offset(c, arm)
    lip, br = lip_now(c)
    above = lip + np.array([0.0, 0.0, L.CAP_DEPTH + 0.004]) + off
    # up first (the cap's skirt clear of the neck and shoulder), then across, so the cap never meets the bottle
    up = np.array([arm.pos[0], arm.pos[1], above[2] + 0.004])
    yield from arm.move(up, arm.rot, 1.0, None, "cap up", tol=0.006)
    psi = pick_psi(c, arm, above)
    yield from arm.move(above + np.array([0.0, 0.0, 0.004]), down(psi), 1.8, None, "cap over neck", tol=0.006)
    yield from arm.move(above, down(psi), 0.5, None, "cap over neck", tol=0.006)
    for _ in range(2):  # re-centre on the measured cap (the grasp can shift the cap in the fingers)
        cp, _ = cap_center(c)
        if np.linalg.norm(arm.measured_tcp()[0] - cp - off) > 0.012:
            raise Stop("cap slipped in or out of the left hand while carried")
        lip, br = lip_now(c)
        err = (lip - cp)[:2]
        yield from arm.move(arm.pos + np.r_[err, 0.0], arm.rot, 0.6, None, "centre cap")
        yield from wait(0.3)
    c.note("cap centred over neck", err_mm=np.round(err * 1e3, 2).tolist())
    # straight down until the cap rests on the thread start (captured) or 3 mm below it (the hand stops there)
    target = lip[2] + L.TRAVEL - 0.0005
    for k in range(60):
        cp, _ = cap_center(c)
        lip, br = lip_now(c)
        err = (lip - cp)[:2]
        pz = max(target + off[2], arm.pos[2] - 0.0008)
        arm.label = "lower cap onto neck"
        arm.set_pose(np.r_[arm.pos[:2] + 0.3 * err, pz], arm.rot, tol=0.006)
        yield
        if c.state()["thread"] != "free" and pz <= target + off[2] + 1e-4:
            break
    yield from wait(0.3)
    c.note("cap on neck")
    if c.state()["thread"] == "free":
        raise Stop("cap not on the thread start")


def max_turn(arm, cp, off, r0, sign, limit):
    """Largest turn (deg, <= limit, 15 deg steps) about the vertical through ``cp`` (sign -1 clockwise seen from above)
    whose end and middle poses are reachable from the current joints."""
    turn = limit
    while turn >= 30.0:
        okm, qm = feasible(arm, cp + L.rz(sign * turn / 2) @ off, L.rz(sign * turn / 2) @ r0)
        ok, _ = feasible(arm, cp + L.rz(sign * turn) @ off, L.rz(sign * turn) @ r0, q0=qm)
        if ok and okm:
            return turn
        turn -= 15.0
    return 0.0


def screw_cap(c, max_strokes=8, seat_margin=SEAT_MARGIN_DEG):
    """Left hand: clockwise strokes about the cap's axis until the cap is turned past the seat by ``seat_margin``."""
    arm = c.left
    for stroke in range(max_strokes):
        s = c.state()
        if s["thread"] != "free" and s["phi"] >= L.PHI_SEAT + math.radians(seat_margin):
            break
        cp, ax = cap_center(c)
        off = arm.measured_tcp()[0] - cp
        r0 = arm.rot.copy()
        turn = max_turn(arm, cp, off, r0, -1.0, STROKE_DEG)
        if turn < 40.0:
            raise Stop(f"wrist cannot turn the cap (stroke {stroke}: {turn:.0f} deg)")
        n = max(1, round(STROKE_S * turn / STROKE_DEG * CONTROL_HZ))
        phi0 = c.state()["phi"]
        from yam_arena import ik

        stalled = 0
        last_phi = phi0
        for k in range(1, n + 1):
            a = turn * ik.smoothstep(k / n)
            cpn, _ = cap_center(c)
            ctr = np.array([cp[0], cp[1], cpn[2]])  # follow the cap down
            arm.label = f"screw stroke {stroke}"
            arm.set_pose(ctr + L.rz(-a) @ off, L.rz(-a) @ r0, tol=0.008)
            yield
            st = c.state()
            if st["phi"] >= L.PHI_SEAT + math.radians(seat_margin):
                break
            if k > 6 and st["phi"] - last_phi < math.radians(0.3):
                stalled += 1
                if stalled > 8:
                    break
            else:
                stalled = 0
            last_phi = st["phi"]
        c.note("stroke done", stroke=stroke, turn=round(turn, 1),
               dphi=round(math.degrees(c.state()["phi"] - phi0), 1))
        # let go, turn back open, grip again
        arm.relax()
        yield from arm.set_grip(grip_of(CAP_OPEN_HALF), 0.4, "open on cap")
        s = c.state()
        if s["phi"] >= L.PHI_SEAT + math.radians(seat_margin) or stroke == max_strokes - 1:
            break
        cp, _ = cap_center(c)
        off2 = arm.pos - cp
        cur = arm.rot.copy()
        back = max_turn(arm, cp, off2, cur, 1.0, BACK_DEG)
        n = max(1, round(1.4 * back / 180.0 * CONTROL_HZ))
        for k in range(1, n + 1):
            a = back * ik.smoothstep(k / n)
            arm.label = "turn back open"
            arm.set_pose(cp + L.rz(a) @ off2, L.rz(a) @ cur, tol=0.008)
            yield
        yield from arm.set_grip(grip_of(L.CAP_R_OUT - CAP_SQUEEZE), 0.5, "regrip cap")
        yield from wait(0.15)
    c.note("cap screwed", phi_deg=round(math.degrees(c.state()["phi"]), 1))


def cap_let_go(c):
    arm = c.left
    arm.relax()
    if arm.grip < grip_of(CAP_OPEN_HALF) - 1e-3:
        yield from arm.set_grip(grip_of(CAP_OPEN_HALF), 0.4, "left open")
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.05]), arm.rot, 1.0, None, "left up off cap", tol=0.01)


# --------------------------------------------------------------------------------------------------- stages
def demo(c):
    yield from wait(0.3)
    yield from both(funnel_out(c), cap_pick(c))
    yield from cap_place(c)
    yield from screw_cap(c)
    yield from cap_let_go(c)
    yield from both(c.left.home(2.0), funnel_drop(c))
    yield from both(c.right.home(1.6), wait(0.1))
    yield from wait(4.0)


def cap_only(c):
    """Close and release the bottle but keep the funnel above the open tray."""
    yield from wait(0.3)
    yield from both(funnel_out(c), cap_pick(c))
    yield from cap_place(c)
    yield from screw_cap(c)
    yield from cap_let_go(c)
    yield from c.left.home(2.0)
    yield from wait(4.5)


def idle(c):
    yield from wait(8.0)


def neg_stow_first(c):
    """The obvious route: funnel out and straight into the tray, the bottle left open (cap untouched): 0."""
    yield from wait(0.3)
    yield from funnel_out(c)
    yield from funnel_drop(c)
    c.note("funnel stowed, bottle open")
    yield from both(c.right.home(2.0), wait(0.1))
    yield from wait(3.0)


def stow_then_cap(c):
    """Put the funnel into containment and immediately proceed to screw down the cap."""
    yield from wait(0.3)
    yield from both(funnel_out(c), cap_pick(c))
    yield from funnel_drop(c)
    yield from wait(0.8)
    c.note("funnel stowed, bottle open")
    yield from both(c.right.home(2.0), cap_place(c))
    yield from screw_cap(c)
    yield from cap_let_go(c)
    yield from both(c.left.home(2.0), wait(0.1))
    yield from wait(3.0)


def neg_cap_loose(c):
    """Cap set on and turned only one stroke (about half a turn, not down to its seat), then the funnel stowed: the
    bottle is still open: 0."""
    yield from wait(0.3)
    yield from both(funnel_out(c), cap_pick(c))
    yield from cap_place(c)
    yield from screw_cap(c, max_strokes=1)
    c.note("cap only started")
    yield from cap_let_go(c)
    yield from both(c.left.home(2.0), funnel_drop(c))
    yield from both(c.right.home(2.0), wait(0.1))
    yield from wait(3.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in CAP_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["CAP_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "cap-only": cap_only,
    "replay": replay,
    "neg-stow-first": neg_stow_first,
    "stow-then-cap": stow_then_cap,
    "neg-stow-then-cap": stow_then_cap,
    "neg-cap-loose": neg_cap_loose,
}

def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(
        str(path),
        fps=CONTROL_HZ,
        codec="libx264",
        quality=8,
        macro_block_size=8,
        ffmpeg_params=["-pix_fmt", "yuv420p"],
    )
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(
    task_path,
    out_dir,
    max_steps=2000,
    record=True,
    stage="demo",
    cam_width=320,
    cam_height=176,
    device="cpu",
    seed=None,
):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(
        task=str(Path(task_path).resolve()),
        object_jitter=0,
        control_hz=CONTROL_HZ,
        cam_width=cam_width,
        cam_height=cam_height,
        device=device,
    )
    stages = stage.split(",")
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st.replace('@', '_seed')}"
        st_seed = seed
        if "@" in st:  # "<stage>@<seed>": this trial's reset seed
            st, sd = st.split("@")
            st_seed = int(sd)
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=st_seed)
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report([x.split("@")[0] for x in stages], resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import capkit

    rt = capkit.runtime()
    poses = rt.read()
    return dict(
        score=capkit.STATE["score"],
        setup=rt.setup,
        joints={k: round(v, 4) for k, v in rt.dofs().items()},
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import layout as L

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        spec = L.setup_spec(setup)
        gp, gr = L.bottle_pose(spec)
        dev = float(np.linalg.norm(np.subtract(r["bodies"]["bottle"], gp * 1000)))
        fp, _ = L.funnel_start(spec)
        dev = max(dev, float(np.linalg.norm(np.subtract(r["bodies"]["funnel"], fp * 1000))))
        cpos = np.array([spec["cx"], spec["cy"], L.TOP + L.CAP_DEPTH])
        dev = max(dev, float(np.linalg.norm(np.subtract(r["bodies"]["cap"], cpos * 1000))))
        rdev = float(np.abs(np.subtract(r["rots"]["bottle"], gr)).max())
        jdev = max(abs(r["joints"][k]) for k in ("crx", "cry"))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         body_dev_from_authored_mm=round(dev, 4), max_rot_dev=round(rdev, 6),
                         max_joint_abs=round(jdev, 4), trial_score=results[i]["task_score"],
                         trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["body_dev_from_authored_mm"] < 2.0 and x["max_joint_abs"] < 0.5
             for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows), max_body_dev_mm=max(x["body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import capkit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    reset_state = reset_state_now()
    c = Ctx(emb)
    gen = STAGES[stage](c)
    frames = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, info, stop, success, res = 0, None, {}, None, False, None
    tracef = open(out / "trace.jsonl", "w")
    while steps < max_steps:
        if ended is None:
            try:
                next(gen)
            except StopIteration:
                ended = steps
            except Stop as e:
                stop, ended = str(e), steps
                print("STOP", stop, flush=True)
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = capkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=capkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=capkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            grip_n={k: round(v, 2) for k, v in c.grip_force().items()},
            tcp_mm={
                n: np.round(a.measured_tcp()[0] * 1000, 2).tolist()
                for n, a in c.arms.items()
            },
            tcp_cmd_mm={
                n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()
            },
            offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()},
        )
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} thread {d.get('thread')} "
                f"turns {d.get('turns')} gap {d.get('gap_mm')} capr {d.get('cap_r_mm')} funnel {d.get('funnel_com')} "
                f"neck {d.get('funnel_in_neck')} stowed {d.get('stowed')} contact {d.get('contact_n')} veto {d.get('veto')} "
                f"wall {time.time() - t1:.0f}s",
                flush=True,
            )
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(
        stage=stage,
        task_score=score,
        success=success,
        steps=steps,
        stop=stop,
        termination=getattr(res, "termination_reason", None),
        setup=reset_state["setup"],
        scorer=dict(score=capkit.STATE["score"], level=capkit.STATE["level"]),
        final=capkit.STATE["detail"],
        nonfinite=capkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        grip_peak_n={k: round(v, 2) for k, v in c.grip_peak.items()},
        physics_dt=capkit.PHYSICS_DT,
        control_hz=CONTROL_HZ,
    )
    tracef.close()
    (out / "result.json").write_text(json.dumps(result, indent=2, default=float))
    if record and frames:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((k for k in frames if "top" in k), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
            imageio.imwrite(out / "final.png", frames[top][-1])
        for cam, fs in frames.items():
            imageio.imwrite(out / f"final_{cam}.png", fs[-1])
    print(
        "RESULT",
        json.dumps(
            {
                k: result[k]
                for k in ("stage", "task_score", "success", "steps", "stop", "setup")
            }
        ),
        flush=True,
    )
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument(
        "--cam-size",
        default="320x176",
        help="camera WxH (evidence runs use the default)",
    )
    ap.add_argument(
        "--device",
        default="cpu",
        help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way",
    )
    ap.add_argument(
        "--seed",
        type=int,
        default=None,
        help="reset seed (seeded start state, as in inspect-robots runs)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
