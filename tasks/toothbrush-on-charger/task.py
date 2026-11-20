"""Stand an electric toothbrush on its charger: pick up the handle, lower its hollow base straight down over the charger's short post, and let go so it stands by itself.

An Oral-B Pro-series handle with its brush head (one rigid body, 27 mm across, 232 mm tall, 0.13 kg) lies on its side on the counter, head to the robot's left, 20 cm right of
a Braun Oral-B Type 3757 charger (free, 0.1 kg, 65 x 55 x 33 mm, on rubber feet). A post 10 mm across and 13 mm tall, its top 3 mm tapered, stands on the charger's top; the handle's
base has a recess 11 mm across and 14 mm deep (a 16-sided bore, 0.5 mm clearance a side). Once the handle is near, its own base hides the post. Geometry is real: the post goes into
the recess or it does not, and a handle let go on the post's tip, or tilted, topples. Code: only the scorer.

    python demokit.py task.py OUT --no-record --mode full,none,hold,beside,counter,tilted,drop
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Stand the electric toothbrush on its charger: pick up the handle, lower its base straight down over the charger's post, and let go so it stands on the charger by itself."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

# -- sizes (metres) ------------------------------------------------------------------------------------------------------------------------
H_R = 13.5 * MM  # handle radius (27 mm across)
BORE_R = 5.6 * MM  # the recess's 16-gon corners; its flats are 5.49 mm from the axis (11 mm across)
BORE_D = 14.0 * MM  # recess depth
BODY_TOP = 0.170  # the handle body's top (it tapers from 0.155)
H_TOP = 0.232  # the brush head's top
H_MASS = 0.13
GRIP_Z = 0.095  # the hand closes on the handle this far up its body, near its centre of mass
CH_X, CH_Y, CH_H = 65 * MM, 55 * MM, 33 * MM  # charger body: 65 mm along x (away from the robot), 55 mm along y
CH_MASS = 0.10
POST_R, POST_H, POST_TIP_R = 5.0 * MM, 13.0 * MM, 3.0 * MM  # the post: 10 mm across, 13 mm tall, its top 3 mm tapered to 6 mm across

CHARGER_AT = (float(os.environ.get("TB_CX", "0.40")), float(os.environ.get("TB_CY", "-0.02")), TOP)
BRUSH_AT = (CHARGER_AT[0], CHARGER_AT[1] - 0.20 - 0.10, TOP)  # the base's centre; lying on its side, head toward +y (the robot's left), body centre about 20 cm right of the charger
N16 = 16

LOOKS = {
    "white": m.Look(m.rgb(0.93, 0.93, 0.92), roughness=0.35),
    "grey": m.Look(m.rgb(0.55, 0.57, 0.60), roughness=0.4),
    "dark": m.Look(m.rgb(0.10, 0.11, 0.13), roughness=0.3),
    "blue": m.Look(m.rgb(0.10, 0.35, 0.75), roughness=0.5),
    "bristle": m.Look(m.rgb(0.92, 0.95, 1.0), roughness=0.8),
    "green": m.Look(m.rgb(0.1, 0.8, 0.2), emissive=(0.05, 0.6, 0.1)),
    "chwhite": m.Look(m.rgb(0.95, 0.95, 0.95), roughness=0.5),
}


def ngon(r: float, n: int = N16, phase: float = math.pi / N16) -> np.ndarray:
    """A regular n-gon's corners, counter-clockwise; with the default phase a flat faces each of +-x and +-y."""
    a = phase + np.arange(n) * 2 * math.pi / n
    return np.c_[r * np.cos(a), r * np.sin(a)]


def rrect(w: float, dp: float, rad: float, n: int = 6) -> np.ndarray:
    out = []
    for cx, cy, a0 in ((w / 2 - rad, dp / 2 - rad, 0.0), (-w / 2 + rad, dp / 2 - rad, 90.0), (-w / 2 + rad, -dp / 2 + rad, 180.0), (w / 2 - rad, -dp / 2 + rad, 270.0)):
        for k in range(n + 1):
            t = math.radians(a0 + 90.0 * k / n)
            out.append((cx + rad * math.cos(t), cy + rad * math.sin(t)))
    return np.array(out)


def brush() -> m.Part:
    """The handle, origin at its base's centre, axis +z. Colliders: 16 convex segments make the bored base ring (0 to 14 mm), a 16-gon prism the body, and hulls the neck and head."""
    outer, inner = ngon(H_R), ngon(BORE_R)
    cols = []
    for i in range(N16):
        j = (i + 1) % N16
        seg = m.prism(np.array([inner[i], outer[i], outer[j], inner[j]]), 0.0, BORE_D)
        cols.append(m.col_hull(f"ring{i}_c", seg))
    body = m.join(m.prism(ngon(H_R), BORE_D, 0.155), m.prism(ngon(H_R * 0.93), 0.155, 0.163), m.prism(ngon(H_R * 0.62), 0.163, BODY_TOP))
    cols.append(m.col_hull("body_c", body))
    cols.append(m.col_hull("neck_c", m.lathe([(0.0, BODY_TOP - 0.005), (5.0 * MM, BODY_TOP - 0.005), (3.6 * MM, 0.212), (0.0, 0.212)], 12)))
    cols.append(m.col_hull("head_c", m.box((12 * MM, 9 * MM, 0.022), (2.0 * MM, 0.0, H_TOP - 0.011))))
    parts = [
        ("body", m.lathe([(0.0, BORE_D), (BORE_R, BORE_D), (BORE_R, 0.0), (H_R, 0.0), (H_R, 0.155), (H_R * 0.93, 0.163), (H_R * 0.62, BODY_TOP), (0.0, BODY_TOP)], 48), "white"),
        ("panel", m.box((3 * MM, 14 * MM, 0.085), (H_R - 1.0 * MM, 0.0, 0.085)), "dark"),  # the dark front strip, toward +x in the handle's frame
        ("button", m.cylinder(5.5 * MM, 3 * MM, (H_R + 0.6 * MM, 0.0, 0.105), axis="x"), "blue"),
        ("lights", m.box((2 * MM, 6 * MM, 6 * MM), (H_R, 0.0, 0.062)), "blue"),
        ("ring", m.cylinder(H_R + 0.2 * MM, 3 * MM, (0.0, 0.0, 0.020)), "grey"),
        ("neck", m.lathe([(0.0, BODY_TOP - 0.005), (5.0 * MM, BODY_TOP - 0.005), (3.6 * MM, 0.212), (0.0, 0.212)], 24), "white"),
        ("head", m.cylinder(7.0 * MM, 5.0 * MM, (0.0, 0.0, H_TOP - 0.011), axis="x"), "white"),
        ("bristles", m.cylinder(6.0 * MM, 8.0 * MM, (6.5 * MM, 0.0, H_TOP - 0.011), axis="x"), "bristle"),
        ("tuft", m.cylinder(2.5 * MM, 8.5 * MM, (6.7 * MM, 0.0, H_TOP - 0.011), axis="x"), "blue"),
    ]
    q = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # xyzw: -90 degrees about x, its axis along +y, lying on a flat of its body
    pos = (BRUSH_AT[0], BRUSH_AT[1], TOP + H_R * math.cos(math.pi / N16) + 0.0005)
    return m.free("brush", pos, parts, cols, {k: LOOKS[k] for k in ("white", "grey", "dark", "blue", "bristle")}, mass=H_MASS, friction=(0.6, 0.5), start_quat=q)


def charger() -> m.Part:
    """Braun Oral-B Type 3757: a rounded block with the post on its top, origin at the middle of its base."""
    out = rrect(CH_X, CH_Y, 20 * MM)
    shell = m.prism(out, 0.0, CH_H - 3 * MM)
    cap = m.prism(rrect(CH_X - 4 * MM, CH_Y - 4 * MM, 18 * MM), CH_H - 3 * MM, CH_H)
    post = [(0.0, CH_H), (POST_R, CH_H), (POST_R, CH_H + POST_H - 3 * MM), (POST_TIP_R, CH_H + POST_H), (0.0, CH_H + POST_H)]
    parts = [("shell", shell, "chwhite"), ("cap", cap, "grey"), ("post", m.lathe(post, 32), "chwhite"),
             ("light", m.box((1.0 * MM, 6 * MM, 3 * MM), (-CH_X / 2 - 0.2 * MM, 0.0, 0.012)), "green")]
    cols = [m.col_hull("body_c", m.prism(out, 0.0, CH_H)), m.col_hull("post_c", m.lathe(post, 32))]
    return m.free("charger", CHARGER_AT, parts, cols, {k: LOOKS[k] for k in ("chwhite", "grey", "green")}, mass=CH_MASS, friction=(0.6, 0.5))


OBJECTS = [charger(), brush()]

# -- the scorer -----------------------------------------------------------------------------------------------------------------------------
UPRIGHT_DEG = 15.0  # 0.33: lifted off the counter (base centre over 10 mm up) with its axis within 15 degrees of vertical, base down
ON_POST_MM, IN_MM = 3.0, 12.0  # 0.67: the base's centre within 3 mm of the post's axis and at most 12 mm above the charger's top (the post at least 1 mm into the recess)
SEAT_MM, SEAT_DEG = 2.0, 5.0  # 1.0: base on the charger's top (within 2 mm), tilt under 5 degrees, every part of both grippers 20 mm or more from the handle, still, for 2 s
CLEAR = 0.020  # over the 17 mm a point of the gripper can lie from its nearest lattice point (the lattice is 2 cm apart), so a touch anywhere on the gripper fails it
HOLD_STEPS = 40  # 2 s at 20 Hz
DROP_SPEED, CARRY = 0.30, 0.15  # void: the base falling faster than 0.3 m/s and more than 0.15 m/s faster than both hands move down: a release (free fall of about 5 mm); read after every physics step
CHARGER_MOVE, CHARGER_TILT = 0.020, 20.0  # void: the charger pushed more than 20 mm off its mark, or tipped


def hand_distance(b, pts) -> float:
    """The least distance from any of `pts` (world) to the handle's whole envelope: a capped cylinder 13.5 mm in radius from its base to the brush head's top (exact, not sampled)."""
    loc = b.to_local(pts)
    rho = np.hypot(loc[:, 0], loc[:, 1])
    dr = np.maximum(rho - H_R, 0.0)
    dz = np.maximum(np.maximum(-loc[:, 2], loc[:, 2] - H_TOP), 0.0)
    return float(np.min(np.hypot(dr, dz)))


class Trial:
    def reset(self, env=None) -> None:
        self.dropped = self.pushed = False  # only the failures latch
        self.lifted_once = False  # arms the drop check: the handle's base has been more than 10 mm off the counter
        self.held = 0
        self.tier_now = 0.0
        self.info = {}
        self.n = 0
        self.warned = False
        if env is not None:
            self.step(env)

    def _fall(self, env, b) -> None:
        """A drop: the base falls faster than 0.3 m/s and faster than every hand moves down (a release, not a controlled move that carries it)."""
        v = float(b.lin_vel[2])
        if not self.lifted_once or v >= -DROP_SPEED:
            return
        scene = env.unwrapped.scene
        hands = []
        for side in ("left", "right"):
            robot = scene[f"{side}_robot"]
            i = robot.find_bodies("gripper")[0][0]
            hands.append(float(robot.data.body_link_vel_w[0, i, 2]))
        if min(hands) > v + CARRY:
            self.dropped = True

    def step(self, env) -> None:
        b, c = m.state(env, "brush"), m.state(env, "charger")
        self._hook(env)
        base = np.asarray(b.pos, float)
        tilt = b.tilt_deg()
        if base[2] - TOP > 0.010:
            self.lifted_once = True
        self._fall(env, b)
        if float(np.linalg.norm(np.asarray(c.pos)[:2] - np.asarray(CHARGER_AT[:2]))) > CHARGER_MOVE or c.tilt_deg() > CHARGER_TILT:
            self.pushed = True
        loc = c.to_local(base[None, :])[0]  # the base's centre in the charger's frame
        off = float(np.hypot(loc[0], loc[1]))
        up = float(loc[2] - CH_H)  # above the charger's top
        hands = np.concatenate([m.fingertips(env), m.hand_body(env)])
        dist = hand_distance(b, hands)
        still = float(np.linalg.norm(b.lin_vel)) < 0.01 and float(np.linalg.norm(b.ang_vel)) < 0.1
        upright = tilt <= UPRIGHT_DEG
        lifted = upright and base[2] - TOP > 0.010
        on_post = upright and off <= ON_POST_MM * MM and -SEAT_MM * MM <= up <= IN_MM * MM
        seated = on_post and up <= SEAT_MM * MM and tilt < SEAT_DEG and dist >= CLEAR and still
        self.held = self.held + 1 if seated else 0
        if self.dropped or self.pushed:
            t = 0.0
        elif self.held >= HOLD_STEPS:
            t = 1.0
        elif on_post:
            t = 0.67
        elif lifted:
            t = 0.33
        else:
            t = 0.0
        self.tier_now = t
        self.info = dict(tilt=tilt, off_mm=off * 1000, up_mm=up * 1000, hand_mm=dist * 1000, still=still, held=self.held, dropped=self.dropped, pushed=self.pushed,
                         charger_mm=float(np.linalg.norm(np.asarray(c.pos)[:2] - np.asarray(CHARGER_AT[:2]))) * 1000)
        self.n += 1
        if self.n % 20 == 0 and os.environ.get("TB_TRACE"):
            report(f"step {self.n}")

    def _hook(self, env) -> None:
        """Read the fall after every physics step too: a short drop lasts a few tens of ms and slips between 20 Hz samples."""
        scene = env.unwrapped.scene
        if getattr(scene, "_tb_hooked", None) is self:
            return
        orig = scene.update
        trial = self

        def update(*a, **k):
            r = orig(*a, **k)
            try:
                trial._fall(env, m.state(env, "brush"))
            except Exception as e:  # the extra read must not break the run
                if not trial.warned:
                    trial.warned = True
                    print(f"[tb] substep read failed: {e!r}", flush=True)
            return r

        scene.update = update
        scene._tb_hooked = self


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.tier_now >= 1.0


def score(objects) -> float:
    return TRIAL.tier_now


def report(tag: str) -> None:
    i = TRIAL.info
    if i:
        print(f"[tb] {tag}: tier {TRIAL.tier_now}, tilt {i['tilt']:.1f}, off post {i['off_mm']:.1f} mm, base {i['up_mm']:.1f} mm over the charger top, hand {i['hand_mm']:.0f} mm, still {i['still']}, "
              f"held {i['held']}, dropped {i['dropped']}, charger moved {i['charger_mm']:.1f} mm", flush=True)


# -- the demo -------------------------------------------------------------------------------------------------------------------------------
MODES = ("full", "none", "hold", "beside", "counter", "tilted", "drop")
PICK_H = float(os.environ.get("TB_PICK_H", "0.009"))  # the TCP this far above the handle's axis as it lies: the pads span its equator
LIFT = 0.15


def demo_script(ctx):
    """Modes: full (pick the lying handle across its body, lift, turn it upright, centre its base over the post, lower it on, let go, back away); none; hold (as full, but keeps hold:
    0.67); beside (stood on the charger's top 20 mm beside the post and let go: 0.33 at most); counter (stood upright on the counter beside the charger and let go: 0); tilted (lowered
    onto the post's tip tilted 12 degrees and let go: it should topple, 0); drop (as full, but let go with the base 20 mm over the charger's top: a drop, 0)."""
    mode = ctx.mode
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    r = ctx.right
    if mode == "none":
        yield from r.wait(3.0)
        report("end")
        return
    B = lambda: ctx.state("brush")
    C = lambda: ctx.state("charger")
    base = lambda: np.asarray(B().pos, float)
    grip_pt = lambda: B().to_world(np.array([[0.0, 0.0, GRIP_Z]]))[0]
    post_top = lambda: C().to_world(np.array([[0.0, 0.0, CH_H]]))[0]  # the post's foot on the charger's top: where the base's centre goes

    yield from r.wait(1.0)
    yield from r.pick(grip_pt, PICK_H, width=2 * H_R, grip=d.down(90.0), above=0.08, lift=LIFT, force=15.0, t=3.0, label="handle")
    report("picked")
    yield from r.turn(90.0, axis=(1.0, 0.0, 0.0), t=2.5, label="turn the handle upright")
    yield from r.wait(0.5)
    report("upright")

    def level_up():
        """Read the handle's tilt and turn the hand to stand it straight (the grip may have let it lean)."""
        for _ in range(2):
            a = B().rotation()[:, 2]
            if math.degrees(math.acos(min(1.0, a[2]))) < 0.7:
                return
            ax = np.cross(a, [0.0, 0.0, 1.0])
            ang = math.degrees(math.asin(min(1.0, np.linalg.norm(ax))))
            yield from r.arc(r.tcp(), ax / np.linalg.norm(ax), ang, t=0.8, label="level the handle")
            yield from r.wait(0.4)

    target = {"full": (0.0, 0.0), "hold": (0.0, 0.0), "drop": (0.0, 0.0), "tilted": (0.0, 0.0), "beside": (0.020, 0.0)}.get(mode)
    if mode == "counter":
        spot = np.array([CHARGER_AT[0], CHARGER_AT[1] - 0.09, TOP])
    else:
        spot = None
    goal = lambda dz: (post_top() + np.array([target[0], target[1], dz])) if spot is None else spot + np.array([0.0, 0.0, dz])
    # over the post, the base 30 mm above the charger's top (the post's tip 17 mm below it)
    off = r.tcp() - base()
    yield from r.move(goal(0.030) + off, t=3.0, label="over the charger")
    yield from level_up()
    yield from r.servo(lambda s: goal(0.030), read=base, tol=0.0015, timeout=3.0, label="centre over the post")
    report("over the post")
    if mode == "tilted":
        yield from r.arc(r.tcp(), (1.0, 0.0, 0.0), 12.0, t=1.0, label="tilt it")
        yield from r.servo(lambda s: goal(0.016), read=base, tol=0.002, timeout=3.0, label="down onto the post's tip")
    else:
        stop = 0.020 if mode == "drop" else 0.0005
        yield from r.servo(lambda s: goal(max(stop, 0.030 - 0.015 * s)), read=base, tol=0.0015, timeout=4.0, gain=0.25, label="lower it straight down")
    yield from r.wait(0.5)
    report("lowered")
    if mode == "hold":
        yield from r.wait(3.0)
        report("end")
        return
    yield from r.open(t=0.6, label="let go")
    yield from r.wait(0.3)
    yield from r.move(r.at() - 0.09 * r.approach_axis(), t=1.5, label="back away")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="up")
    yield from r.wait(3.0)
    report("end")
