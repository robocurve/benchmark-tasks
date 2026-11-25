"""Scripted full-score demo (and negative variants) for the t8-tube task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the poses of the fixture,
lamp and carton and the lampholder state through ``tubekit.bench_state()`` (read-only, like an operator watching the
lamp: where it is, how far it has turned (the printed lamp code turns with it), whether it is free); it never writes a
pose, a joint state or a score. Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to
the URDF joint limits.

Plan:
1. Both hands pinch the lamp across its diameter 15 cm either side of its centre, approaching from the front
   (10 deg up from horizontal), and turn it together about its own axis in strokes of up to 33 deg (the YAM wrist
   cannot go round the whole quarter turn in one grip, and past ~115 deg its palm meets the strip channel); let go, go
   round again, pinch, turn on until the pin pairs line up with the lampholder slots (90 deg). The lamp is then held
   only by the hands.
2. Both hands lower it straight down 5 cm (the pins slide down the slots and out under the lampholders).
3. Carry it over the carton, turning it about the vertical to lie along the carton and about its axis to a steeper
   grip, lower it in, let go about 8 mm above the bottom, back off; both arms back to READY.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings) during free
moves, because the joint PD sags under load (pattern reused from earlier demos); in contact the offsets are frozen.
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
SQUEEZE = 0.0042  # 21 N nominal per finger at the modelled drive stiffness
OPEN_HALF = 0.024  # finger half-opening around the 25.4 mm lamp
GRASP_G = 0.15  # pinch points from the lamp's centre along its axis
BACKOFF = 0.06
ALPHA0 = 80.0  # orbit angle of the pinch at the start of each stroke (90 = horizontal approach)
STROKE_DEG = 33.0  # past about 115 deg the gripper palm meets the strip channel
ALPHA_PLACE = 158.0  # steeper grip for lowering into the carton (clears its front wall)
LOWER = 0.05  # straight down out of the slots (slot 22 mm + end-cap radius 13 mm + margin)
CARRY_UP = 0.10  # lamp axis this far above its resting height in the carton while carried over it
PLACE_GAP = 0.013  # let go with the lamp this far above the carton bottom


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
        import tubekit

        return tubekit.bench_state()

    def detail(self):
        import tubekit

        return tubekit.STATE["detail"]

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
        row = dict(at=tag, step=self.step, level=d.get("level"), holder=d.get("holder"), spin=d.get("spin_deg"),
                   dz=d.get("lamp_dz_mm"), carton=d.get("in_carton"), veto=d.get("veto"), **kw)
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




# --------------------------------------------------------------------------------------------------- lamp
def lamp_now(c):
    """Measured lamp centre, unit axis (+y of the lamp, toward the robot's left), and u: the horizontal direction
    perpendicular to the axis pointing away from the robot."""
    s = c.state()
    lp, lr = s["lamp"]
    ax = lr[:, 1] / np.linalg.norm(lr[:, 1])
    if ax[1] < 0:
        ax = -ax
    u = np.cross(ax, [0.0, 0.0, 1.0])
    u /= np.linalg.norm(u)
    return np.asarray(lp, float), ax, u


def approach(alpha, ax, u):
    """Gripper approach direction at orbit angle ``alpha`` (deg) about the lamp axis: 90 = horizontal toward the wall,
    larger = pointing down."""
    a = math.radians(alpha)
    up = np.cross(u, ax)  # = +z for a level lamp
    return math.sin(a) * u + math.cos(a) * up


def grasp_pose(c, arm, alpha, back=0.0, centre=None, ax=None, u=None, g=None):
    """TCP pose pinching the lamp across its diameter at ``g`` m along the axis from its centre (left arm +, right
    arm -), fingers closing perpendicular to the axis, pad tips 6 mm past the axis; ``back`` m back along the
    approach."""
    if centre is None:
        centre, ax, u = lamp_now(c)
    g = (GRASP_G if arm.name == "left" else -GRASP_G) if g is None else g
    a = approach(alpha, ax, u)
    p = centre + g * ax - (PAD_TIP - 0.006 + back) * a
    close = np.cross(a, ax) * (-1.0 if arm.name == "left" else 1.0)  # wrist-roll sense that keeps the carry reachable
    return p, frame(a, close)


def rot_about(axis, deg):
    from scipy.spatial.transform import Rotation

    return Rotation.from_rotvec(np.asarray(axis, float) * math.radians(deg)).as_matrix()


def orbit(c, deg, duration, label, centre, ax):
    """Both hands turn about the lamp axis line (through ``centre``) by ``deg`` together (each from its current
    command), as one rigid motion."""
    starts = {n: (a.pos.copy(), a.rot.copy()) for n, a in c.arms.items()}
    n = max(1, round(duration * CONTROL_HZ))
    from yam_arena import ik

    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        r = rot_about(ax, deg * s)
        for nm, a in c.arms.items():
            p0, r0 = starts[nm]
            a.label = label
            a.set_pose(centre + r @ (p0 - centre), r @ r0, tol=0.006)
        yield
    c.alpha = getattr(c, "alpha", ALPHA0) + deg


def translate(c, delta, duration, label, rot=None):
    """Both hands move by ``delta`` (station frame) together, optionally turning about the vertical through the lamp
    centre (rot = (centre, R))."""
    starts = {n: (a.pos.copy(), a.rot.copy()) for n, a in c.arms.items()}
    n = max(1, round(duration * CONTROL_HZ))
    from yam_arena import ik

    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        for nm, a in c.arms.items():
            p0, r0 = starts[nm]
            a.label = label
            if rot is None:
                a.set_pose(p0 + s * np.asarray(delta), r0, tol=0.006)
            else:
                cen, rr = rot
                rs = rot_about([0, 0, 1], rr * s)
                a.set_pose(cen + s * np.asarray(delta) + rs @ (p0 - cen), rs @ r0, tol=0.006)
        yield


def take_hold(c, alpha, label):
    """Both hands: from open, back off, go round to ``alpha`` about the lamp axis, approach and pinch the lamp."""
    centre, ax, u = lamp_now(c)
    gens = []
    for arm in (c.left, c.right):
        arm.set_comp(True)
        p, r = grasp_pose(c, arm, alpha, back=BACKOFF, centre=centre, ax=ax, u=u)
        gens.append(arm.move(p, r, 2.0, grip_of(OPEN_HALF), f"{arm.name} to lamp ({label})", tol=0.006))
    yield from both(*gens)
    yield from both(c.left.settle(0.004, 2.0), c.right.settle(0.004, 2.0))
    centre, ax, u = lamp_now(c)
    gens = []
    for arm in (c.left, c.right):
        p, r = grasp_pose(c, arm, alpha, centre=centre, ax=ax, u=u)
        gens.append(arm.move(p, r, 1.2, None, f"{arm.name} onto lamp"))
    yield from both(*gens)
    yield from both(c.left.settle(0.003, 1.5), c.right.settle(0.003, 1.5))
    for arm in (c.left, c.right):
        arm.set_comp("hold")
    yield from both(c.left.set_grip(grip_of(L.TUBE_R - SQUEEZE), 0.7, "pinch"),
                    c.right.set_grip(grip_of(L.TUBE_R - SQUEEZE), 0.7, "pinch"))
    yield from wait(0.3)
    c.alpha = alpha
    c.note(f"pinched at {alpha}")


def let_go(c, back=BACKOFF, label="let go"):
    for arm in (c.left, c.right):
        arm.relax()
    yield from both(c.left.set_grip(grip_of(OPEN_HALF), 0.6, label), c.right.set_grip(grip_of(OPEN_HALF), 0.6, label))
    gens = []
    for arm in (c.left, c.right):
        arm.set_comp(True)
        a = arm.rot[:, 2]
        gens.append(arm.move(arm.pos - back * a, arm.rot, 1.0, None, f"{arm.name} back off"))
    yield from both(*gens)


def turn_open(c, a0=ALPHA0, stroke=STROKE_DEG, max_strokes=4):
    """Turn the lamp (both rotors) toward open in strokes: pinch, turn both hands together about the lamp axis, let
    go and go round again; the last stroke stops once the pins line up with the slots, still holding the lamp."""
    for k in range(max_strokes):
        yield from take_hold(c, a0, f"stroke {k + 1}")
        d = c.state()["dofs"]
        left = L.OPEN_DEG - math.degrees(d["spin"])
        last = left + 2.5 <= stroke
        deg = min(stroke, left + 2.5)
        centre, ax, _ = lamp_now(c)
        yield from orbit(c, deg, 0.06 * deg, f"turn stroke {k + 1}", centre, ax)
        yield from wait(0.4)
        c.note(f"stroke {k + 1} done", spin_now=round(math.degrees(c.state()["dofs"]["spin"]), 1))
        if c.state()["holder"] != "locked":
            return
        if last:  # rotor short of open (slip): a little more
            centre, ax, _ = lamp_now(c)
            yield from orbit(c, 4.0, 0.6, "turn a little more", centre, ax)
            yield from wait(0.3)
            if c.state()["holder"] != "locked":
                return
        yield from let_go(c, label=f"regrip {k + 1}")
    if c.state()["holder"] == "locked":
        raise Stop("lamp did not turn open")


def lower_out(c):
    """Both hands lower the freed lamp straight down out of the slots (pins slide down them), then forward clear of
    the fixture."""
    s = c.state()
    fz = s["fixture"][1][:, 2]
    yield from translate(c, -LOWER * fz, 2.0, "lower out of the slots")
    yield from wait(0.3)
    c.note("lowered", state_now=c.state()["holder"])
    if c.state()["holder"] != "out":
        raise Stop(f"lamp not out after lowering: {c.state()['holder']}")


def carton_target(c, bench=False):
    s = c.state()
    bp, br = s["carton"]
    cen = np.asarray(bp, float) + br @ np.array([0.0, 0.0, L.BOX_WALL + L.TUBE_R + PLACE_GAP])
    if bench:
        cen += br @ np.array([0.135, 0.0, -L.BOX_WALL])
    bax = br[:, 1] / np.linalg.norm(br[:, 1])
    return cen, bax


def carry(c, delta, yaw, dalpha, duration, label):
    """Both hands move the held lamp as one rigid body: turn it ``dalpha`` deg about its own axis and ``yaw`` deg about
    the vertical through its centre while its centre moves by ``delta``."""
    centre, ax, _ = lamp_now(c)
    starts = {n: (a.pos.copy(), a.rot.copy()) for n, a in c.arms.items()}
    n = max(1, round(duration * CONTROL_HZ))
    from yam_arena import ik

    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        r = rot_about([0.0, 0.0, 1.0], yaw * s) @ rot_about(ax, dalpha * s)
        for nm, a in c.arms.items():
            p0, r0 = starts[nm]
            a.label = label
            a.set_pose(centre + s * np.asarray(delta) + r @ (p0 - centre), r @ r0, tol=0.006)
        yield
    c.alpha = getattr(c, "alpha", ALPHA0) + dalpha


def place(c, gap=0.0, bench=False):
    """Carry the lamp over the carton (axis along the carton, turning it to the steeper placing grip on the way),
    lower it in, let go just above the bottom, back off."""
    centre, ax, u = lamp_now(c)
    cen, bax = carton_target(c, bench)
    yaw = math.degrees(math.atan2(bax[1], bax[0]) - math.atan2(ax[1], ax[0]))
    yaw = (yaw + 90.0) % 180.0 - 90.0
    above = cen + np.array([0.0, 0.0, CARRY_UP])
    yield from carry(c, above - centre, yaw, ALPHA_PLACE - c.alpha, 3.5, "carry over the carton")
    yield from both(c.left.settle(0.005, 2.0), c.right.settle(0.005, 2.0))
    centre, ax, u = lamp_now(c)
    cen, bax = carton_target(c, bench)
    yield from carry(c, cen - centre + np.array([0.0, 0.0, gap]), 0.0, 0.0, 2.5, "lower into the carton")
    yield from both(c.left.settle(0.004, 1.5), c.right.settle(0.004, 1.5))
    c.note("in the carton (held)", in_carton=c.state()["in_carton"])
    yield from let_go(c, back=0.06, label="release in the carton")
    yield from both(c.left.home(2.5), c.right.home(2.5))


# --------------------------------------------------------------------------------------------------- stages
def demo(c):
    yield from wait(0.3)
    yield from turn_open(c)
    yield from lower_out(c)
    yield from place(c)
    yield from wait(3.0)


def probe(c):
    """M1 probe: one stroke (45 deg) and let go: the lamp must stay turned (rotor drag) and seated."""
    yield from wait(0.3)
    yield from take_hold(c, ALPHA0, "probe")
    centre, ax, _ = lamp_now(c)
    yield from orbit(c, STROKE_DEG, 0.06 * STROKE_DEG, "probe stroke", centre, ax)
    yield from wait(0.5)
    c.note("probe stroke done", spin_now=round(math.degrees(c.state()["dofs"]["spin"]), 1))
    yield from let_go(c)
    yield from wait(1.5)
    c.note("probe released", spin_now=round(math.degrees(c.state()["dofs"]["spin"]), 1))
    yield from take_hold(c, ALPHA0, "probe 2")
    centre, ax, _ = lamp_now(c)
    yield from orbit(c, STROKE_DEG + 3.0, 0.06 * STROKE_DEG, "probe stroke 2", centre, ax)
    yield from wait(0.5)
    c.note("probe stroke 2 done", spin_now=round(math.degrees(c.state()["dofs"]["spin"]), 1))
    yield from let_go(c)
    yield from wait(2.0)
    c.note("probe let go when open")


def idle(c):
    yield from wait(8.0)


def look(c):
    """Setup view for the media (no contact): the left arm backs its wrist camera away in front of and below the lamp
    so that the camera shows the fixture, both lampholders and the lamp; the right arm stays at READY."""
    yield from wait(0.3)
    centre, ax, u = lamp_now(c)
    p, r = grasp_pose(c, c.left, 120.0, back=0.20, centre=centre, ax=ax, u=u, g=0.24)
    yield from c.left.move(p + np.array([0.0, 0.0, -0.05]), r, 3.0, grip_of(OPEN_HALF), "left look", tol=0.01)
    yield from wait(3.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in TUBE_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state. TUBE_MIRROR=1 plays it
    mirrored (arms swapped, joints 1, 5 and 6 negated)."""
    import os

    rows = [json.loads(line) for line in open(os.environ["TUBE_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    if os.environ.get("TUBE_MIRROR"):  # mirrored trace: arms swapped, joints 1, 5 and 6 negated
        out = []
        for a in c.replay:
            lft, rgt = list(a[:7]), list(a[7:])
            for arm in (lft, rgt):
                for j in (0, 4, 5):
                    arm[j] = -arm[j]
            out.append(rgt + lft)
        c.replay = out
    for _ in range(len(c.replay) + 40):
        yield


def neg_let_go_open(c):
    """Turns the lamp open, then lets go before lowering it (as if the quarter turn were the whole job): the lamp
    drops out of the slots: 0."""
    yield from wait(0.3)
    yield from turn_open(c)
    yield from let_go(c, label="let go at open")
    yield from both(c.left.home(2.0), c.right.home(2.0))
    yield from wait(3.0)


def place_on_bench(c):
    """Lower and release the tube on the bench beside the carton for partial credit."""
    yield from wait(0.3)
    yield from turn_open(c)
    yield from lower_out(c)
    yield from place(c, bench=True)
    yield from wait(5.0)


def neg_drop_high(c):
    """Takes the lamp down properly, then lets it go 10 cm above the carton bottom (inside the carton's outline): it
    lands in the carton, but after a free fall: 0."""
    yield from wait(0.3)
    yield from turn_open(c)
    yield from lower_out(c)
    yield from place(c, gap=0.10)
    yield from wait(3.0)


def neg_pull_down(c):
    """Pinches the locked lamp and pulls it straight down without the quarter turn: the lampholders hold it (the pull
    only rolls it in the fingers, short of open): 0."""
    yield from wait(0.3)
    yield from take_hold(c, ALPHA0, "pull")
    s = c.state()
    fz = s["fixture"][1][:, 2]
    for arm in (c.left, c.right):
        arm.set_comp(True)
    yield from translate(c, -0.10 * fz, 2.0, "pull straight down")
    for _ in range(int(4.0 * CONTROL_HZ)):  # keep pulling (the joint integral raises the arms' pull)
        if c.state()["holder"] == "broken":
            break
        yield
    c.note("pulled", state_now=c.state()["holder"])
    yield from wait(2.0)


def neg_one_hand(c):
    """Right hand alone turns the lamp open (left arm stays home) and carries on: whatever happens is measured."""
    yield from wait(0.3)
    yield from wait(5.0)


STAGES = {
    "demo": demo,
    "outside": place_on_bench,
    "probe": probe,
    "idle": idle,
    "look": look,
    "replay": replay,
    "neg-let-go-open": neg_let_go_open,
    "neg-pull-down": neg_pull_down,
    "neg-drop-high": neg_drop_high,
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
    import tubekit

    rt = tubekit.runtime()
    poses = rt.read()
    return dict(
        score=tubekit.STATE["score"],
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
        gp, gr = L.fixture_pose(spec)
        dev = float(np.linalg.norm(np.subtract(r["bodies"]["fixture"], gp * 1000)))
        lp_, _ = L.fixture_pose(spec)
        dev = max(dev, float(np.linalg.norm(np.subtract(r["bodies"]["lamp"], lp_ * 1000))))
        bp_, _ = L.carton_pose(spec)
        dev = max(dev, float(np.linalg.norm(np.subtract(r["bodies"]["carton"], bp_ * 1000))) - 0.6)
        rdev = float(np.abs(np.subtract(r["rots"]["fixture"], gr)).max())
        jdev = max(abs(v) for v in r["joints"].values())
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         body_dev_from_authored_mm=round(dev, 4), max_rot_dev=round(rdev, 6),
                         max_joint_abs=round(jdev, 4), trial_score=results[i]["task_score"],
                         trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["body_dev_from_authored_mm"] < 2.0 and x["max_joint_abs"] < 0.5
             for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows), max_body_dev_mm=max(x["body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import tubekit
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
        d = tubekit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=tubekit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=tubekit.STATE["nonfinite"],
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} holder {d.get('holder')} "
                f"spin {d.get('spin_deg')} dz {d.get('lamp_dz_mm')} z {d.get('lamp_z')} v {d.get('speed')} "
                f"carton {d.get('in_carton')} contact {d.get('contact_n')} peaks {d.get('peaks')} veto {d.get('veto')} "
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
        scorer=dict(score=tubekit.STATE["score"], level=tubekit.STATE["level"]),
        final=tubekit.STATE["detail"],
        nonfinite=tubekit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        grip_peak_n={k: round(v, 2) for k, v in c.grip_peak.items()},
        physics_dt=tubekit.PHYSICS_DT,
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
