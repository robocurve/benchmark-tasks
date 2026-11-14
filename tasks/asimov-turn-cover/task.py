"""Turn a humanoid's hung upper body round on its gantry hook so that its back faces you.

The open-source Asimov 1 humanoid's upper body (Menlo Research: the torso shell, both arms and the head, 18.3 kg) hangs by its two shoulder
shackles from a bench gantry's swivel hook, facing the arms. Its maker's build step 10.6 turns the robot round to work on its back. On the swivel
it turns freely: a push that starts it also loads it with momentum, which the swivel's drag alone takes well over a turn to remove. The score
asks only that it end at rest facing away, without a blow. Fitting the rear cover, the next build step, is left out.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode turn
Final:  python demokit.py task.py OUT --mode turn
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
SCENE = "station"
INSTRUCTION = "Turn the body hanging from the swivel hook half a turn, so its back faces you."
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20
TOP = d.TABLE_TOP
DT = d.DT
TRACE = os.environ.get("ASIMOV_TRACE", "1") != "0"

LOOKS = {
    "shell": m.Look(m.rgb(0.88, 0.88, 0.86), roughness=0.6),  # MJF PA12, dyed light
    "joint": m.Look(m.rgb(0.20, 0.20, 0.21), roughness=0.5),
    "gantry": m.Look(m.rgb(0.25, 0.27, 0.30), metallic=0.6, roughness=0.4),
    "chain": m.Look(m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.3),
}

UB = np.load(HERE / "upper_body.npz")
UB_COM = np.asarray(UB["com"], float)  # in the robot's pelvis frame
UB_KG = float(UB["mass"])
UB_I = np.asarray(UB["I"], float)
NAMES = [str(n) for n in UB["names"]]
HOOK = np.array([0.55, 0.0])  # the swivel's axis, over the body's centre of mass
COM_Z = TOP + 0.35  # the centre of mass's height: the hanging hands clear the table by about 3 cm
SWIVEL_FRICTION = 0.005  # PhysX's joint friction coefficient: a free spin slows from 45 to 27.5 deg/s in 10 s; a real hook's spin-down is to be measured
MARK = math.pi  # the swivel's angle at the mark: the body authored facing the arms (angle 0), the mark its back to them


def _to_hung(v) -> np.ndarray:
    """Points from the robot's pelvis frame to the hung frame: the centre of mass on the swivel's axis, at the origin, the body turned to face the
    arms (-x) at the swivel's angle 0."""
    q = np.atleast_2d(np.asarray(v, float)) - UB_COM
    q = q * np.array([-1.0, -1.0, 1.0])
    return q if np.ndim(v) > 1 else q[0]


def _body_link():
    parts, cols = [], []
    for n in NAMES:
        v, f = _to_hung(UB[f"v_{n}"]), np.asarray(UB[f"f_{n}"], int)
        parts.append((n, m.Mesh(v, f), "joint" if "link" in n and ("elbow" in n or "wrist" in n or "neck" in n) else "shell"))
        hv, hf = _to_hung(UB[f"hv_{n}"]), np.asarray(UB[f"hf_{n}"], int)
        cols.append(m.col_hull(f"c_{n}", m.Mesh(hv, hf)))
    ev = np.linalg.eigvalsh(UB_I)
    return m.hinge("body", "Z", (0.0, 0.0, 0.0), -math.inf, math.inf, parts, cols, mass=UB_KG, damping=0.0, friction=SWIVEL_FRICTION, gravity=True,
                   com=(0.0, 0.0, 0.0), inertia=(float(UB_I[0, 0]), float(UB_I[1, 1]), float(UB_I[2, 2])))


def _gantry_parts():
    """The bench gantry's frame: two posts at the sides and a beam over the body, the swivel hook and two shackle chains down to the shoulders."""
    top = 0.48  # the beam's height over the centre of mass
    parts = [("beam", m.box((0.04, 0.80, 0.04), (0.0, 0.0, top)), "gantry"),
             ("post_l", m.box((0.04, 0.04, top + 0.35 + 0.02), (0.0, 0.40, (top - 0.35) / 2)), "gantry"),
             ("post_r", m.box((0.04, 0.04, top + 0.35 + 0.02), (0.0, -0.40, (top - 0.35) / 2)), "gantry"),
             ("hook", m.cylinder(0.012, 0.05, (0.0, 0.0, top - 0.045)), "chain")]
    cols = [m.col_box("c_beam", (0.04, 0.80, 0.04), (0.0, 0.0, top)), m.col_box("c_post_l", (0.04, 0.04, top + 0.37), (0.0, 0.40, (top - 0.35) / 2)),
            m.col_box("c_post_r", (0.04, 0.04, top + 0.37), (0.0, -0.40, (top - 0.35) / 2))]
    return parts, cols


_gp, _gc = _gantry_parts()
OBJECTS = [m.mechanism("upper", (HOOK[0], HOOK[1], COM_Z), [_body_link()], LOOKS, frame_parts=_gp, frame_colliders=_gc)]


# -- the scorer -----------------------------------------------------------------------------------------------------------------------------------

MARK_TOL = math.radians(10.0)  # at rest on the mark: within 10 degrees of it
REST_RATE, STOP_RATE, SPIN_RATE = math.radians(1.0), math.radians(5.0), math.radians(20.0)  # still; stopped (the 0.25 rung); left spinning
DRAG_TAU = 20.0  # s: the swivel's spin-down time constant (a free spin from 45 deg/s: 27.5 after 10 s, 13 after 25 s): a body turning at w coasts on about w x 20 s
TURN_MIN = math.radians(90.0)
NEAR = 0.05  # a fingertip point within 5 cm of the body's colliders: a hand (fingers or palm) may be on it
FAST = math.radians(10.0)  # a collision counts while the body turns faster than this
WINDOW = 2  # steps (0.1 s): spin changes are read over this window, so a blow split between two samples still counts whole
BLOW = math.radians(25.0)  # the spin slowing or reversing by more than 25 deg/s within 0.1 s: a blow (about 11 N at 10 cm from the swivel, 0.257 kg m2), a zero
FREE = math.radians(5.0)  # with no fingertip within 5 cm, the spin changing by more than 5 deg/s within 0.1 s is a collision (the swivel's own drag: under 0.2)
HISTORY = 40  # steps of spin rates kept (2 s)


def _planes():
    """The body's collider hulls as plane sets in the hung frame (pelvis frame turned 180 degrees about z, the centre of mass at the origin): a
    point's largest plane value over a hull is a lower bound on its distance outside that hull."""
    out = []
    for n in NAMES:
        eq = np.asarray(UB[f"heq_{n}"], float)
        nrm, off = eq[:, :3], eq[:, 3]
        # pelvis p = flip(h) + com, flip(x, y, z) = (-x, -y, z): n.p + d = (n * flip) . h + (n . com + d)
        out.append((nrm * np.array([-1.0, -1.0, 1.0]), off + nrm @ UB_COM))
    return out


PLANES = _planes()


def body_gap(points, ang) -> np.ndarray:
    """Each point's distance outside the body's colliders (a lower bound; 0 or less inside), with the swivel at `ang`."""
    q = np.atleast_2d(np.asarray(points, float)) - np.array([HOOK[0], HOOK[1], COM_Z])
    c, sn = math.cos(ang), math.sin(ang)
    h = np.stack([c * q[:, 0] + sn * q[:, 1], -sn * q[:, 0] + c * q[:, 1], q[:, 2]], axis=1)  # into the hung frame
    return np.min(np.stack([np.max(h @ nrm.T + off, axis=1) for nrm, off in PLANES], axis=1), axis=1)


LADDER = gk.Ladder(("turned", 0.25), ("on_mark", 1.0))


class Trial:
    """The trial's record: the swivel's net angle and rate, the latches and the end hold."""

    def __init__(self):
        self.collision, self.struck = gk.Latch("stopped by a collision"), gk.Latch("a hand struck")
        self.hold_mark = gk.EndHold(HOLD_STEPS)
        self.reset()

    def reset(self, env=None):
        self.n, self.last, self.theta, self.rate, self.rates, self.nears = 0, None, 0.0, 0.0, [], []
        self.turned, self.peak, self.over, self.full, self.on_mark_now = False, 0.0, 0.0, False, False
        self.peak_change, self.peak_free, self.gap = 0.0, 0.0, math.inf
        for x in (self.collision, self.struck, self.hold_mark):
            x.reset()

    def step(self, env):
        self.n += 1
        ang = m.state(env, "upper").joint("body")
        if self.last is not None:
            d = wrap(ang - self.last)
            self.theta += d
            self.rate = d / DT
        self.last = ang
        gaps = body_gap(m.fingertips(env), ang)
        self.gap = float(gaps.min())
        if self.last is not None and self.n > 1:
            self.rates.append(self.rate)
            self.nears.append(self.gap <= NEAR)
            self.rates, self.nears = self.rates[-(HISTORY + 1):], self.nears[-(HISTORY + 1):]
            if len(self.rates) > WINDOW:
                recent = self.rates[-(WINDOW + 1):]
                change = max(abs(self.rate - r) for r in recent[:-1])
                slowing = max(math.copysign(1.0, r) * (r - self.rate) if r else 0.0 for r in recent[:-1])  # the drop in rate along the earlier motion: a reversal counts whole
                self.peak_change = max(self.peak_change, slowing)
                self.struck.trip(slowing > BLOW, at=self.n)
                if not any(self.nears[-(WINDOW + 1):]) and max(abs(r) for r in recent) > FAST:
                    self.peak_free = max(self.peak_free, change)
                    self.collision.trip(change > FREE, at=self.n)
        self.peak = max(self.peak, abs(self.theta))
        self.over = max(0.0, self.peak - math.pi)
        self.turned |= abs(self.theta) >= TURN_MIN and abs(self.rate) < STOP_RATE and not (self.collision or self.struck)
        coast_to = self.theta + DRAG_TAU * self.rate  # where the drag alone would leave it
        on_mark = (abs(abs(self.theta) - math.pi) < MARK_TOL and abs(abs(coast_to) - math.pi) < MARK_TOL
                   and abs(self.rate) < REST_RATE)  # turned 180 degrees net, either way round, still, and staying there: past it, or a turn more, is not 180
        self.hold_mark.step(on_mark)
        self.on_mark_now = self.hold_mark.count >= HOLD_STEPS
        self.full = self.score() >= 1.0
        if TRACE and self.n % 10 == 0:
            print(f"[asimov] step {self.n}: swivel {math.degrees(self.theta):.1f} deg (mark 180), rate {math.degrees(self.rate):.1f} deg/s, "
                  f"overshoot {math.degrees(self.over):.1f} deg, nearest fingertip {1000 * self.gap:.0f} mm, peak slowing in 0.1 s {math.degrees(self.peak_change):.1f} "
                  f"deg/s ({math.degrees(self.peak_free):.2f} with no hand near); turned {self.turned}, on the mark {self.hold_mark.count}; "
                  f"score {self.score()}", flush=True)

    def score(self) -> float:
        failed = bool(self.collision or self.struck) or abs(self.rate) > SPIN_RATE  # left spinning: read at whatever step the score is taken, the run's end
        return LADDER.score({"turned": self.turned, "on_mark": self.on_mark_now}, failed=failed)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score()


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def demo_script(ctx):
    """Modes: turn (the right pushes the torso's front near its left edge, away: the body starts turning; the left meets the back near its left
    edge as it comes round and rides it to rest on the mark, a moving stop), push (the push alone: it keeps turning), none."""
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        for _ in range(80):
            yield
        return

    def body_point(p_pelvis, ang):
        """A point on the body (pelvis frame) in the station frame with the swivel at `ang`."""
        q = _to_hung(p_pelvis)
        c, sn = math.cos(ang), math.sin(ang)
        return np.array([HOOK[0] + c * q[0] - sn * q[1], HOOK[1] + sn * q[0] + c * q[1], COM_Z + q[2]])

    front_l = np.array([0.034, 0.06, 0.25])  # the torso's front near its left edge (the shell curves in further out), 25 cm up the pelvis frame
    back_l = np.array([-0.143, 0.06, 0.25])  # its back near its left edge, on the collider's surface at this height (the back bulges to -0.151 at the middle): pressed from the arms' side, it is the point that slows the turn
    # the left waits outside the body's sweep (0.215 m round the swivel at this height), on its own side
    yield from l.move(np.array([HOOK[0] - 0.05, 0.26, COM_Z - 0.10]), grip=d.level(0.0), finger=d.FINGER_CLOSED, t=2.0, label="the left hand waits outside the body's sweep")
    ang0 = ctx.state("upper").joint("body")
    p = body_point(front_l, ang0)
    yield from r.move(p - [0.06, 0.0, 0.0], grip=d.level(0.0), finger=d.FINGER_CLOSED, t=2.0, label="the right fingertips before the torso's front")
    yield from r.move(p + [0.02, 0.0, 0.0], t=0.4, label="push it away: the body starts turning")
    yield from r.move(r.at() + [-0.15, -0.10, 0.04], t=0.8, label="the right hand back, well out of the sweep")
    if mode == "push":
        for _ in range(int(40.0 / DT)):  # to the run limit
            yield
        return
    # the catch, a moving stop on the rear face of the body's left upper arm. That face is flat (the arm's collider, hung frame: x 0.049 over
    # y -0.14 to -0.18 and z -0.06 to -0.14) and 16 cm out from the swivel, so near the mark it comes straight at a hand held in its path along
    # -x: the left braces it moving only along x, at a fixed height and y, and never slides across it (following the back instead dragged the
    # body on by friction). The hand meets the face 12 degrees before the mark and backs off with it, pressing in only as far as the body turns
    # faster than an even slowing to rest just short of the mark asks for; once the body is nearly stopped the hand stays where it is, unpressed.
    ARM = np.array([0.049, -0.16, -0.10])  # the contact point on the arm's rear face, hung frame
    TIP = 0.013  # the closed fingertips' ends ahead of the TCP, level grip
    meet = MARK - math.radians(12.0)  # a braced hand stops the arm within about 10 degrees
    REST = MARK - math.radians(2.0)

    def face_x(ang, y):
        """Where the arm's rear face crosses the line at height ARM[2] and world y `y`, with the swivel at `ang`."""
        c, sn = math.cos(ang), math.sin(ang)
        px, py = HOOK[0] + c * ARM[0] - sn * ARM[1], HOOK[1] + sn * ARM[0] + c * ARM[1]
        return px - (y - py) * sn / c  # the face's normal is (cos, sin): moving along it by dy shifts x by -dy tan

    Y = HOOK[1] - ARM[1]  # the face's y at the mark
    Z = COM_Z + ARM[2]

    def hand(x):
        return np.array([x, Y, Z])

    for _ in range(300):
        if wrap(MARK - ctx.state("upper").joint("body")) < math.radians(70.0):
            break
        yield
    yield from l.move(hand(face_x(meet, Y) - TIP - 0.01), grip=d.level(0.0), t=0.8, label="the left in the arm's path, 1 cm off its face")
    prev = ctx.state("upper").joint("body")
    rate = 0.0
    for _ in range(200):
        yield
        ang = ctx.state("upper").joint("body")
        rate, prev = (ang - prev) / DT, ang
        if wrap(meet - ang) < math.radians(1.0):
            break
    w0 = max(rate, math.radians(5.0))
    span = max(wrap(REST - ang), math.radians(5.0))
    K = 0.03  # m of press per rad/s of rate above the even slowing
    FF = 0.002  # m: a steady press while slowing
    print(f"[asimov] ride from {math.degrees(ang):.1f} deg at {math.degrees(w0):.1f} deg/s", flush=True)
    frozen, still = None, 0
    for k in range(int(6.0 / DT)):
        left = max(0.0, wrap(REST - ang))
        w_des = w0 * math.sqrt(min(1.0, left / span))
        press = min(0.012, max(0.0, (FF if w_des > 0 else 0.0) + K * (rate - w_des)))
        if frozen is None and k > 8 and abs(rate) < math.radians(2.0) and abs(wrap(MARK - ang)) < math.radians(8.0):
            frozen = face_x(ang, Y) - TIP
        gap = 0.01 * max(0.0, 1.0 - (k + 1) * DT / 0.4)  # the last 1 cm closed in the first 0.4 s
        x = face_x(ang, Y) - TIP - gap + press if frozen is None else frozen
        yield from l.move(hand(x), grip=d.level(0.0), t=DT, label="brace the arm, slowing the body to rest on the mark")
        ang = ctx.state("upper").joint("body")
        rate, prev = (ang - prev) / DT, ang
        still = still + 1 if abs(rate) < math.radians(0.5) else 0
        if TRACE and k % 4 == 0:
            print(f"[asimov] ride {k}: swivel {math.degrees(ang):.1f} deg, rate {math.degrees(rate):.1f} (wanted {math.degrees(w_des):.1f}) deg/s, press {1000 * press:.1f} mm{', held' if frozen is not None else ''}", flush=True)
        if still >= 10 and abs(wrap(MARK - ang)) < math.radians(8.0):
            break
    for _ in range(20):
        yield
    yield from l.move(l.at() + [-0.04, 0.0, 0.0], t=1.0, label="the fingertips off the arm")
    yield from l.move(l.at() + [-0.06, 0.05, 0.05], t=1.0, label="the left hand away")
    for _ in range(40):
        yield
