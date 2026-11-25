"""Bottle flip: throw a one-third-full 500 ml plastic water bottle so that it turns end over end once in the air and lands standing
on its base.

On the station table lies a 40 x 40 cm grey felt desk pad. A capped 500 ml (16.9 fl oz) PET water bottle, 6.2 cm across and 20.3 cm
tall, filled to 170 ml (one third of its height), stands upright on the pad. Beside the pad: a notebook, an apple and a mug (distractors).

Model (declared in code):
- Bottle ("bottle", a free articulation): the PET shell and cap are one 15 g body (body, shoulder and neck colliders, flat base). The 170 g of
  water is two 85 g slugs, each on its own slider along the bottle's axis, with gravity and viscous damping (the "tennis-ball bottle" of
  Dekker et al., Am. J. Phys. 86, 733 (2018), which reproduces the water bottle's spin-down). At rest both slugs sit at the base. Spinning in
  flight pushes the upper slug towards the cap, which raises the moment of inertia and slows the spin; on landing it slides back and its
  damping soaks up the impact, as the water does.
- Pad ("pad", fixed): a 3 mm felt desk pad, the landing zone.

Scoring (code, every control step, from the bottle's pose, its water slugs and the fingertips):
- A flight starts at the first step on which the bottle is airborne (no fingertip within 12 mm of it, lowest point over 12 mm above the
  table or pad) right after a step on which a fingertip was touching it (released from the hand, not bounced off the table). It ends at
  the first step on which a fingertip touches it, its lowest point is within 12 mm of the table or pad, or its centre of mass leaves a
  free-fall path (an impact). Its turn is the summed angle the bottle's axis sweeps, step by step, from the last step in the hand.
- A flip is a flight that turns 240 degrees or more and ends without any fingertip touching it.
- 0.25: a flip that comes down on the table or pad (lowest point within 3 cm of it when the flight ends). 0.5: such a flip whose first
  touchdown is base first on the pad (axis within 45 degrees of upright, base centre over the pad). 1.0: after a flip, with no
  fingertip touching the bottle since that flip, it stands on its base on the pad (axis within 10 degrees of vertical, base within 5 mm of
  the pad's top, base centre on the pad), still (under 2 cm/s and 0.3 rad/s), every fingertip 5 cm or more from it, for 2 s.
  The score is the best reached. Any number of attempts within the run limit.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Flip the water bottle so that it lands standing upright on the desk pad."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
DT = 0.05
G = 9.81
TOP = TABLE_TOP[SCENE]

# -- the desk pad ------------------------------------------------------------------------------------------------------------------------
PAD_C = (0.36, -0.16)
PAD_SIZE = 0.40
PAD_T = 0.003
PAD_TOP = TOP + PAD_T

# -- the bottle: 500 ml (16.9 fl oz) PET water bottle, 6.2 cm x 20.3 cm, 15 g with its cap ---------------------------------------------
H = 0.203
R_BODY = 0.031
Z_SHOULDER0, Z_SHOULDER1 = 0.125, 0.172  # the shoulder tapers from the body to the neck
R_NECK = 0.0135
Z_RING = 0.181  # the neck's support ring
R_CAP = 0.0155
Z_CAP0 = 0.184
SHELL_MASS = 0.015
WATER_MASS = 0.170  # 170 ml
SLUG = WATER_MASS / 2
BASE_T = 0.003
WATER_H = 0.059  # 170 ml over the body's 29 cm^2 inside: 5.9 cm, 0.29 of the bottle's height
Z_LO0 = BASE_T + 0.25 * WATER_H  # each slug's centre at rest
Z_HI0 = BASE_T + 0.75 * WATER_H
LO_TRAVEL = 0.150  # how far each slug can move up the bottle (to where the water sits with the bottle upside down)
HI_TRAVEL = 0.078  # the upper slug stops where the two slugs give the moment of inertia of water spread along the whole bottle (Dekker et al. eq. 14: I0/Imax = 0.30 for this bottle)
WATER_DAMP = 0.4  # N s/m on each slug: the water's sloshing loss (estimate)
BOTTLE_START = (0.30, -0.06)

PROFILE = [(0.0, 0.0), (0.0285, 0.0), (R_BODY, 0.004), (R_BODY, Z_SHOULDER0), (0.026, 0.150), (0.018, 0.165), (R_NECK, Z_SHOULDER1),
           (R_NECK, Z_RING - 0.002), (0.0165, Z_RING - 0.002), (0.0165, Z_RING), (R_NECK, Z_RING), (R_NECK, Z_CAP0), (R_CAP, Z_CAP0),
           (R_CAP, H - 0.001), (0.0145, H), (0.0, H)]


def radius_at(z: float) -> float:
    """The bottle's outside radius at height z (m) above its base."""
    zs = [p[1] for p in PROFILE[1:-1]]
    rs = [p[0] for p in PROFILE[1:-1]]
    return float(np.interp(z, zs, rs))


LOOKS = {
    "pet": m.Look(color=m.rgb(0.80, 0.88, 0.92), roughness=0.08, opacity=0.35),
    "label": m.Look(color=m.rgb(0.10, 0.45, 0.25), roughness=0.5),
    "label_white": m.Look(color=m.rgb(0.92, 0.94, 0.92), roughness=0.5),
    "cap": m.Look(color=m.rgb(0.08, 0.40, 0.20), roughness=0.45),
    "mark": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.6),
    "water": m.Look(color=m.rgb(0.45, 0.70, 0.90), roughness=0.05, opacity=0.6),
    "felt": m.Look(color=m.rgb(0.32, 0.33, 0.35), roughness=0.95),
}


def _shell_parts():
    body = m.lathe([(0.0, 0.0)] + PROFILE[1:8] + [(0.0, Z_RING - 0.002)], 48)
    label = m.lathe([(R_BODY + 0.0006, 0.070), (R_BODY + 0.0006, 0.118)], 48)
    stripe = m.lathe([(R_BODY + 0.0008, 0.088), (R_BODY + 0.0008, 0.096)], 48)
    fill_mark = m.lathe([(R_BODY + 0.0009, BASE_T + WATER_H - 0.001), (R_BODY + 0.0009, BASE_T + WATER_H + 0.001)], 48)
    ring = m.lathe([(R_NECK, Z_RING - 0.002), (0.0165, Z_RING - 0.002), (0.0165, Z_RING), (R_NECK, Z_RING)], 32)
    cap = m.lathe([(0.0, Z_CAP0), (R_CAP, Z_CAP0), (R_CAP, H - 0.001), (0.0145, H), (0.0, H)], 32)
    return [("shell", body, "pet"), ("label", label, "label"), ("stripe", stripe, "label_white"), ("fill_mark", fill_mark, "mark"),
            ("ring", ring, "pet"), ("cap", cap, "cap")]


def _shell_colliders():
    shoulder = m.lathe([(0.0, Z_SHOULDER0), (R_BODY, Z_SHOULDER0), (0.026, 0.150), (0.018, 0.165), (R_NECK, Z_SHOULDER1), (0.0, Z_SHOULDER1)], 24)
    return [m.col_cylinder("body_c", R_BODY, Z_SHOULDER0, (0.0, 0.0, Z_SHOULDER0 / 2)),
            m.col_hull("shoulder_c", shoulder),
            m.col_cylinder("neck_c", R_CAP, H - Z_SHOULDER1, (0.0, 0.0, (H + Z_SHOULDER1) / 2))]


def _shell_com() -> float:
    """The shell's centre of mass height, as PhysX takes it from the colliders' volumes."""
    v1, z1 = math.pi * R_BODY ** 2 * Z_SHOULDER0, Z_SHOULDER0 / 2
    zs = np.linspace(Z_SHOULDER0, Z_SHOULDER1, 200)
    rs = np.interp(zs, [Z_SHOULDER0, 0.150, 0.165, Z_SHOULDER1], [R_BODY, 0.026, 0.018, R_NECK])
    a = math.pi * rs ** 2
    v2 = float(np.trapezoid(a, zs))
    z2 = float(np.trapezoid(a * zs, zs)) / v2
    v3, z3 = math.pi * R_CAP ** 2 * (H - Z_SHOULDER1), (H + Z_SHOULDER1) / 2
    return (v1 * z1 + v2 * z2 + v3 * z3) / (v1 + v2 + v3)


SHELL_COM = _shell_com()


def _slug(name: str, z0: float, travel: float) -> m.Link:
    t = WATER_H / 2
    disc = m.cylinder(R_BODY - 0.0025, t, (0.0, 0.0, z0))
    r = R_BODY - 0.002
    inertia = (SLUG * (3 * r * r + t * t) / 12, SLUG * (3 * r * r + t * t) / 12, SLUG * r * r / 2)
    return m.slider(name, "Z", 0.0, travel, [(f"{name}_v", disc, "water")], [], mass=SLUG, damping=WATER_DAMP, friction=0.0, gravity=True,
                    com=(0.0, 0.0, z0), inertia=inertia, effort=50.0)


def _bottle():
    ix = SHELL_MASS * (R_BODY ** 2 / 2 + H ** 2 / 12)
    return m.mechanism("bottle", (BOTTLE_START[0], BOTTLE_START[1], PAD_TOP), [_slug("water_lo", Z_LO0, LO_TRAVEL), _slug("water_hi", Z_HI0, HI_TRAVEL)],
                       LOOKS, frame_parts=_shell_parts(), frame_colliders=_shell_colliders(), friction=(0.6, 0.5), free_mass=SHELL_MASS,
                       frame_inertia=(ix, ix, SHELL_MASS * R_BODY ** 2), solver_iters=32)


BOTTLE = _bottle()
PAD = m.fixed("pad", (PAD_C[0], PAD_C[1], TOP), [("felt", m.box((PAD_SIZE, PAD_SIZE, PAD_T), (0.0, 0.0, PAD_T / 2)), "felt")],
              [m.col_box("felt_c", (PAD_SIZE, PAD_SIZE, PAD_T), (0.0, 0.0, PAD_T / 2))], LOOKS, friction=(0.8, 0.7))

DISTRACTORS = [
    SimReady("notebook", pos=(0.20, 0.27, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd", yaw=0.15),
    SimReady("apple", pos=(0.56, 0.12, TOP), size=(0.070, 0.067, 0.068), mass=0.18, package="food_produce_apple_b01", usd="sm_food_produce_apple_b01_01.usd", convex=True),
    SimReady("mug", pos=(0.42, 0.22, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=0.6),
]

OBJECTS = [PAD, BOTTLE, *DISTRACTORS]


# -- reading the bottle ------------------------------------------------------------------------------------------------------------------


def _axis(s) -> np.ndarray:
    return s.rotation()[:, 2]


def lowest(s) -> float:
    """The bottle's lowest point above the table top (m): the base rim, the shoulder's start and the cap rim."""
    a = _axis(s)
    side = math.sqrt(max(0.0, 1.0 - a[2] ** 2))
    p = np.asarray(s.pos, float)
    zs = [p[2] - R_BODY * side, p[2] + Z_SHOULDER0 * a[2] - R_BODY * side, p[2] + H * a[2] - R_CAP * side]
    return float(min(zs) - TOP)


def on_pad(xy) -> bool:
    return abs(xy[0] - PAD_C[0]) <= PAD_SIZE / 2 and abs(xy[1] - PAD_C[1]) <= PAD_SIZE / 2


def com(s) -> np.ndarray:
    zl = Z_LO0 + s.joint("water_lo")
    zh = Z_HI0 + s.joint("water_hi")
    z = (SHELL_MASS * SHELL_COM + SLUG * (zl + zh)) / (SHELL_MASS + WATER_MASS)
    return np.asarray(s.to_world(np.array([[0.0, 0.0, z]]))[0], float)


def finger_gap(s, tips) -> float:
    """The smallest distance (m) from any fingertip point to the bottle's surface."""
    loc = s.to_local(tips)
    rr = np.hypot(loc[:, 0], loc[:, 1])
    z = loc[:, 2]
    best = 1.0
    for r_, z_ in zip(rr, z):
        if 0.0 <= z_ <= H:
            d = r_ - radius_at(z_)
        else:
            end_r = R_BODY if z_ < 0 else 0.0145
            dz = -z_ if z_ < 0 else z_ - H
            d = math.hypot(max(r_ - end_r, 0.0), dz)
        best = min(best, d)
    return float(best)


def now(env) -> dict:
    s = m.state(env, "bottle")
    a = _axis(s)
    low = lowest(s)
    gap = finger_gap(s, m.fingertips(env))
    support = PAD_T if on_pad(s.pos) else 0.0
    return dict(s=s, axis=a, tilt=float(math.degrees(math.acos(float(np.clip(a[2], -1.0, 1.0))))), low=low, gap=gap,
                touch=gap < 0.012, airborne=gap >= 0.012 and low > support + 0.012, com=com(s), support=support)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a trial's record is cleared
        self.best = 0.0
        self.full = False
        self.hold = 0
        self.armed = False  # a flip has landed and nothing has touched the bottle since
        self.in_flight = False
        self.prev_touch = False
        self.turn = 0.0
        self.coms: list = []
        self.last_axis = None
        self.touch_axis = np.array([0.0, 0.0, 1.0])
        self.flights: list = []  # (turn deg, ended by, tilt at the end)
        self.score = 0.0

    def _end_flight(self, c, how: str):
        self.in_flight = False
        self.flights.append((round(self.turn, 1), how, round(c["tilt"], 1), round(float(c["s"].pos[0]), 2), round(float(c["s"].pos[1]), 2)))
        if how == "touch":
            return
        flip = self.turn >= 240.0
        on_table = how == "land" and c["low"] <= c["support"] + 0.03  # came down on the table or pad, not on the robot or the floor
        if not flip or not on_table:
            return
        self.best = max(self.best, 0.25)
        a = c["axis"]
        if a[2] >= math.cos(math.radians(45.0)) and on_pad(c["s"].pos):
            self.best = max(self.best, 0.5)
        self.armed = True

    def step(self, env):
        c = now(env)
        if self.in_flight:
            a = c["axis"]
            self.turn += math.degrees(math.acos(float(np.clip(a @ self.last_axis, -1.0, 1.0))))
            self.last_axis = a
            self.coms.append(c["com"])
            ballistic = True
            if len(self.coms) >= 3:
                p2, p1, p0 = self.coms[-3:]
                dev = p0 - 2 * p1 + p2 - np.array([0.0, 0.0, -G * DT * DT])
                ballistic = float(np.linalg.norm(dev)) < 0.015
            if c["touch"]:
                self._end_flight(c, "touch")
            elif c["com"][2] < TOP - 0.05:
                self._end_flight(c, "lost")
            elif c["low"] <= c["support"] + 0.012 or not ballistic:
                self._end_flight(c, "land")
        elif c["airborne"] and self.prev_touch:
            self.in_flight = True
            self.turn = math.degrees(math.acos(float(np.clip(c["axis"] @ self.touch_axis, -1.0, 1.0))))  # from the last step in the hand
            self.last_axis = c["axis"]
            self.coms = [c["com"]]
        if c["touch"]:
            self.armed = False
            self.touch_axis = c["axis"]
        self.prev_touch = c["touch"]
        s = c["s"]
        still = float(np.linalg.norm(s.lin_vel)) < 0.02 and float(np.linalg.norm(s.ang_vel)) < 0.3
        standing = c["tilt"] <= 10.0 and on_pad(s.pos) and abs(s.pos[2] - PAD_TOP) <= 0.005
        ok = self.armed and standing and still and c["gap"] >= 0.05 and not self.in_flight
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else self.best


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------------------------

GRIP_Z = 0.192  # the TCP's height above the bottle's base when it pinches the cap
GRIP_N = 70.0  # newtons the fingers pinch the cap with
LIFT = np.array([0.28, -0.13, TOP + 0.42])  # where the throw starts, the bottle hanging cap up (level grip pointing +y)
# joint offsets from the arm's pose at LIFT (rad): [joint 3 (elbow), joint 4 (wrist pitch), joint 6 (wrist roll)]
WOUND = np.array([-0.4, -1.2, -0.1])  # the wind-up: elbow down, wrist cocked back, so the base swings back and up
THROWN = np.array([0.7, 1.0, -1.6])  # where the flick drives each joint (the wrist pitch to its upper limit)
RATE = np.array([5.5, 20.0, 3.25])  # rad/s: how fast each joint's target moves towards THROWN
RELEASE_STEP = 5  # control steps into the flick at which the gripper opens: the bottle is then about horizontal


def _report(ctx, env, label=""):
    c = now(env)
    s = c["s"]
    print(f"[bottle] t {ctx.t:5.2f} {label}pos {np.round(s.pos, 3)} tilt {c['tilt']:.1f} low {c['low'] * 1000:.0f} mm gap {c['gap'] * 1000:.0f} mm "
          f"water {s.joint('water_lo') * 1000:.0f}/{s.joint('water_hi') * 1000:.0f} mm flight {TRIAL.in_flight} turn {TRIAL.turn:.0f} "
          f"flights {TRIAL.flights} best {TRIAL.best} armed {TRIAL.armed} hold {TRIAL.hold} score {TRIAL.score}", flush=True)


def demo_script(ctx):
    """Closed loop on the bottle's position (read before the grasp); the throw itself is timed in joint space, as a flick has to be.
    The right gripper, level and pointing at the robot's left (+y), pinches the cap front to back (40 N), lifts the bottle and carries it
    to LIFT, hanging cap up. It squeezes to GRIP_N and winds up (elbow down, wrist cocked back: the base swings back and up), then flicks:
    each joint's target steps towards THROWN at RATE, and the gripper opens RELEASE_STEP steps in, with the bottle about horizontal and
    moving straight up. The arm then goes back to its ready pose in joint space, out of the bottle's way.
    Failure checks (--mode): none does nothing; place lifts the bottle and sets it down upright on the pad; drop lets go of it upright
    from 15 cm; half turns it cap down 25 cm up and lets go (a half flip); hard flicks with the wrist roll twice as fast; early lets go
    one step (50 ms) early."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN
    from yam_arena import ik

    env = ctx.emb._env
    r = ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "place", "drop", "half", "hard", "early"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(80):
            yield
        _report(ctx, env, "end: ")
        return

    grip = d.level(90.0)
    b = ctx.pos("bottle")
    neck = b + np.array([0.0, 0.0, GRIP_Z])
    yield from r.move(neck + np.array([0.0, -0.08, 0.03]), grip=grip, finger=FINGER_OPEN, t=2.0, label="beside the cap")
    yield from r.move(neck + np.array([0.0, -0.08, 0.0]), t=0.5, label="level with the cap")
    yield from r.move(neck, t=1.0, label="round the cap")
    yield from r.close(2 * R_CAP, 40.0, t=0.5, label="pinch the cap")
    yield from r.move(neck + np.array([0.0, 0.0, 0.06]), t=0.6, label="lift")
    _report(ctx, env, "lifted: ")

    if mode == "place":
        spot = np.array([0.36, -0.02, PAD_TOP + GRIP_Z + 0.003])
        yield from r.move(spot + np.array([0.0, 0.0, 0.04]), t=1.5, label="over the spot")
        yield from r.move(spot, t=0.8, label="set down")
        yield from r.open(t=0.4)
        yield from r.move(r.at() + np.array([0.0, -0.10, 0.0]), t=0.8, label="out")
        for _ in range(60):
            yield
        _report(ctx, env, "end: ")
        return
    if mode == "drop":
        spot = np.array([0.36, -0.02, PAD_TOP + GRIP_Z + 0.15])
        yield from r.move(spot, t=1.5, label="over the spot")
        yield from r.open(t=0.3)
        yield from r.move(r.at() + np.array([0.0, -0.10, 0.0]), t=0.8, label="out")
        for _ in range(60):
            yield
        _report(ctx, env, "end: ")
        return
    if mode == "half":
        yield from r.move(np.array([0.30, -0.06, TOP + 0.45]), t=1.5, label="up")
        yield from r.turn(180.0, axis=(0.0, 1.0, 0.0), t=1.5, label="cap down")
        _report(ctx, env, "inverted: ")
        yield from r.open(t=0.3)
        yield from r.move(r.at() + np.array([0.0, -0.10, 0.05]), t=0.6, label="out")
        for _ in range(60):
            yield
        _report(ctx, env, "end: ")
        return

    yield from r.move(LIFT, t=1.2, label="to the throw")
    q_l = np.array(r.q, dtype=float)
    idx = [2, 3, 5]
    wound, thrown = q_l.copy(), q_l.copy()
    wound[idx] += WOUND
    thrown[idx] += THROWN
    thrown = np.clip(thrown, ik.LOWER + 0.02, ik.UPPER - 0.02)
    rate = np.zeros(6)
    rate[idx] = RATE
    if mode == "hard":  # a careless flick: the wrist roll twice as fast
        rate[5] *= 2.0
    release = RELEASE_STEP - 1 if mode == "early" else RELEASE_STEP  # let go one step (50 ms) early
    pinch = float(np.clip((-(R_CAP - GRIP_N / 5000.0)) / FINGER_OPEN, 0.0, 1.0))  # demokit's close(): GRIPPER stiffness 5000 N/m
    n = 24
    for k in range(1, n + 1):  # wind up over 1.2 s
        q = q_l + (wound - q_l) * ik.smoothstep(k / n)
        r.q = q.copy()
        r.cmd = np.r_[q, pinch]
        yield
    yield from r.wait(0.4)
    _report(ctx, env, "wound: ")
    q = wound.copy()
    for k in range(1, 12):  # the flick: joint targets stepped at RATE, the gripper opened at RELEASE_STEP
        q = np.clip(q + np.clip(thrown - q, -rate * DT, rate * DT), ik.LOWER + 0.02, ik.UPPER - 0.02)
        r.q = q.copy()
        r.cmd = np.r_[q, 1.0 if k >= release else pinch]
        yield
    _report(ctx, env, "thrown: ")
    # out of the bottle's way: back to the ready pose in joint space
    from yam_arena.constants import READY
    q0 = np.array(ctx.observed("right")[:6], dtype=float)
    n = 30
    for k in range(1, n + 1):
        q = q0 + (np.array(READY) - q0) * ik.smoothstep(k / n)
        r.q = q.copy()
        r.cmd = np.r_[q, 1.0]
        yield
    r.finger = FINGER_OPEN
    for k in range(60):
        yield
        if k % 20 == 19:
            _report(ctx, env, "end: ")
