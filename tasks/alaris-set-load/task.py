"""Load a primed infusion set into a pump module, in the maker's order: the blue upper fitment lowered into its recess first, then the safety clamp pressed into
its recess below the mechanism, without the fitment rising out of its recess or the set being stretched, and with the set left untwisted.

A large-volume pump module (76 mm wide, 224 mm tall, 140 mm deep) is clamped to its pole stub with its door open, facing the arms. Down its face runs the
mechanism channel: the upper fitment recess at the top (a ledge with a slot the tubing passes through, open to the front: the fitment "fits loosely in the
recess"), and the safety clamp's recess below the mechanism. The primed set hangs from a set holder beside the pump: the blue upper fitment in the holder's fork,
the silicone pumping segment below it (a capsule chain), and the safety clamp at the segment's lower end.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --stills 20 --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import cordkit  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Load the primed set into the pump module: blue upper fitment into its recess first, then press the safety clamp in, without lifting the fitment or stretching the set; leave the set untwisted, let go and move clear."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 2 s hold: score() and success() agree
HOLD_STEPS = 40
TOP = TABLE_TOP[SCENE]

LOOKS = {
    "case": m.Look(color=m.rgb(0.86, 0.87, 0.85), roughness=0.5),
    "grey": m.Look(color=m.rgb(0.42, 0.44, 0.46), roughness=0.5),
    "dark": m.Look(color=m.rgb(0.10, 0.11, 0.12), roughness=0.6),
    "metal": m.Look(color=m.rgb(0.70, 0.71, 0.72), metallic=0.85, roughness=0.3),
    "blue": m.Look(color=m.rgb(0.10, 0.45, 0.85), roughness=0.4),
    "clamp": m.Look(color=m.rgb(0.93, 0.93, 0.90), roughness=0.5),
    "orange": m.Look(color=m.rgb(0.95, 0.50, 0.10), roughness=0.5),
    "stand": m.Look(color=m.rgb(0.16, 0.17, 0.18), metallic=0.5, roughness=0.5),
}

# -- the pump module (station metres; z over the table) --------------------------------------
STAND = 0.04  # the module's foot over the table, on its pole clamp
XF = 0.30  # its front face
W, H, D = 0.076, 0.224, 0.140  # 3 x 8.8 x 5.5 in (BD)
CH = 0.008  # the mechanism channel's half width, 6 mm deep
Z_UP = STAND + 0.226  # the upper fitment recess's ledge top, on the module's top at the front (BD's figure, p. 88)
X_SEAT = XF + 0.003  # the fitment's axis when seated
SLOT = 0.003  # half width of the ledge's tubing slot, open to the front
Z_LOW = Z_UP - 0.183  # the safety clamp recess's centre, below the mechanism
CLAMP = (0.020, 0.014, 0.020)  # the safety clamp: deep, wide, tall (to measure on a real set)
POCKET_D = 0.010  # the clamp recess's depth from the face: a seated clamp's body stands 10 mm proud (its thumb pad 14 mm), to be pressed and later pulled
SEATED_X = XF + POCKET_D - CLAMP[0] / 2  # the clamp's centre when pressed home: its back on the recess's back stop
CATCH_N = 2.0  # the recess's spring catch: once the clamp is pressed past it, it holds the clamp back against the recess with this force (approximated, to be measured)

# -- the set ----------------------------------------------------------------------------------
COLLAR_R, COLLAR_T = 0.006, 0.002
CONE = (0.0055, 0.0055, 0.022)  # the fitment's fluted body: radius at foot, at top, length
STUB = (0.0022, 0.010)  # the tubing stub under the collar: radius, length
SEG_L, SEG_R = 0.164, 0.0022  # the pumping segment: about 1 mm of slack between the seats
FIT_MASS, CLAMP_MASS, SEG_MASS = 0.008, 0.008, 0.006  # approximated up from a few grams each for the solver: stated
HOLDER = np.array([0.28, -0.14])
Z_HOLD = 0.21  # the fitment's collar foot in the holder's fork


def _pump() -> m.Part:
    parts, cols = [], []

    def add(name, size, center, look, collide=True):
        parts.append((name, m.box(size, center), look))
        if collide:
            cols.append(m.col_box(f"{name}_c", size, center))

    zc = STAND + H / 2
    back = XF + POCKET_D + 0.004  # the body behind the clamp recess's back
    add("body", (XF + D - back, W, H), ((back + XF + D) / 2, 0.0, zc), "case")
    zr0, zr1, yr = Z_LOW - CLAMP[2] / 2 - 0.0015, Z_LOW + CLAMP[2] / 2 + 0.0015, 0.019  # beside the clamp recess, a pocket the clip's pins slide in
    for s, n in ((1.0, "l"), (-1.0, "r")):  # the face either side of the channel, as deep as the clamp recess
        add(f"face_{n}lo", (back - XF, W / 2 - CH, zr0 - STAND), ((XF + back) / 2, s * (CH + (W / 2 - CH) / 2), (STAND + zr0) / 2), "grey")
        add(f"face_{n}hi", (back - XF, W / 2 - CH, STAND + H - zr1), ((XF + back) / 2, s * (CH + (W / 2 - CH) / 2), (zr1 + STAND + H) / 2), "grey")
        add(f"face_{n}mid", (back - XF, W / 2 - yr, zr1 - zr0), ((XF + back) / 2, s * (yr + (W / 2 - yr) / 2), (zr0 + zr1) / 2), "grey")
    hh0 = CLAMP[2] / 2 + 0.0035
    for n, z0, z1 in (("floor_hi", Z_LOW + hh0, Z_UP - 0.004), ("floor_lo", STAND, Z_LOW - hh0)):  # the channel's floor, 6 mm deep, open above the ledge
        add(n, (back - XF - 0.006, 2 * CH, z1 - z0), ((XF + 0.006 + back) / 2, 0.0, (z0 + z1) / 2), "grey")
    # the upper fitment recess: a ledge with a slot open to the front, and a low wall round the collar (at the front, a gap only the tubing passes)
    zl = Z_UP - 0.002
    add("ledge_l", (0.026, 0.012 - SLOT, 0.004), (X_SEAT, 0.006 + SLOT / 2 + 0.0, zl), "blue")
    add("ledge_r", (0.026, 0.012 - SLOT, 0.004), (X_SEAT, -(0.006 + SLOT / 2), zl), "blue")
    add("ledge_b", (0.006, 2 * SLOT, 0.004), (X_SEAT + SLOT + 0.003, 0.0, zl), "blue")
    for s, n in ((1.0, "l"), (-1.0, "r")):
        add(f"wall_{n}", (0.014, 0.003, 0.003), (X_SEAT + 0.002, s * (COLLAR_R + 0.0025), Z_UP + 0.0015), "grey")
    add("wall_b", (0.003, 0.016, 0.003), (X_SEAT + COLLAR_R + 0.0025, 0.0, Z_UP + 0.0015), "grey")
    for s, n in ((1.0, "fl"), (-1.0, "fr")):
        add(f"wall_{n}", (0.003, 0.0065, 0.003), (X_SEAT - COLLAR_R - 0.0025, s * 0.00675, Z_UP + 0.0015), "grey")
    # the safety clamp's recess below the mechanism
    zp, hw, hh = Z_LOW, CLAMP[1] / 2 + 0.0015, CLAMP[2] / 2 + 0.0015
    for sz, n in ((1.0, "top"), (-1.0, "bot")):  # the recess's top and bottom, each with a 5 mm lead-in at the mouth that squares a pitched clamp
        outline = [(XF, sz * (hh + 0.005)), (XF + 0.005, sz * hh), (XF + POCKET_D, sz * hh), (XF + POCKET_D, sz * (hh + 0.006)), (XF, sz * (hh + 0.006))]
        if sz < 0:
            outline = outline[::-1]
        spans = ((-(hw + 0.004), -0.003), (0.003, hw + 0.004)) if sz > 0 else ((-(hw + 0.004), hw + 0.004),)  # the top has a 6 mm slot the tubing comes down through
        for k, (y0, y1) in enumerate(spans):
            lip = m.prism(outline, y0, y1).turned(90.0, "x").moved((0.0, 0.0, zp))
            parts.append((f"pocket_{n}{k}", lip, "grey"))
            cols.append(m.col_hull(f"pocket_{n}{k}_c", lip))
    # its sides are the clip's two sprung pins (`_clip`), so that a clamp goes in only when pressed
    add("pocket_back", (0.004, 2 * hw, 2 * hh), (XF + POCKET_D + 0.002, 0.0, zp), "dark")
    # the mechanism's parts, seen in the channel: pumping fingers, occluders, pressure sensors, air-in-line detector
    for k, z in enumerate((0.17, 0.15, 0.13, 0.11, 0.09)):
        add(f"finger{k}", (0.002, 2 * CH - 0.002, 0.012), (XF + 0.0055, 0.0, STAND + z), "metal", collide=False)
    for k, z in enumerate((0.195, 0.065)):
        parts.append((f"sensor{k}", m.cylinder(0.0055, 0.002, (XF + 0.0055, 0.0, STAND + z), axis="x", segments=20), "dark"))
    add("ail", (0.004, 2 * CH, 0.012), (XF + 0.004, 0.0, STAND + 0.052), "dark", collide=False)
    add("label", (0.0005, 0.03, 0.006), (XF - 0.0003, -0.024, Z_UP - 0.008), "orange", collide=False)
    add("screen", (0.0005, 0.05, 0.03), (XF - 0.0003, 0.0, STAND + H - 0.002 + 0.0), "dark", collide=False)
    # the door, open: hinged on the left of the face, swung back toward the left side
    th = math.radians(150.0)
    door = m.box((0.008, W, H - 0.01), (0.0, -W / 2, 0.0)).turned(-math.degrees(th), "z").moved((XF - 0.004, W / 2, zc))
    parts.append(("door", door, "case"))
    cols.append(m.col_hull("door_c", door))
    # the pole stub and its clamp behind the module
    parts.append(("pole", m.cylinder(0.0125, 0.45, (XF + D + 0.03, 0.0, 0.225), segments=20), "metal"))
    add("pole_clamp", (0.05, 0.05, 0.06), (XF + D + 0.01, 0.0, STAND + 0.12), "stand")
    add("foot", (0.12, 0.12, 0.008), (XF + D + 0.03, 0.0, 0.004), "stand")
    return m.fixed("pump", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


def _holder() -> m.Part:
    """The set holder: a post with a fork at the top, the fitment's collar resting on its prongs and the tubing stub between them."""
    parts, cols = [], []
    x, y = HOLDER
    post = (0.012, 0.012, Z_HOLD - 0.004)
    parts.append(("post", m.box(post, (x + 0.03, y, post[2] / 2)), "stand"))
    cols.append(m.col_box("post_c", post, (x + 0.03, y, post[2] / 2)))
    for s in (1.0, -1.0):
        size, c = (0.03, 0.006, 0.004), (x + 0.012, y + s * (SLOT + 0.003), Z_HOLD - 0.002)
        parts.append((f"prong{int(s > 0)}", m.box(size, c), "stand"))
        cols.append(m.col_box(f"prong{int(s > 0)}_c", size, c))
    parts.append(("foot", m.box((0.06, 0.05, 0.006), (x + 0.03, y, 0.003)), "stand"))
    return m.fixed("holder", (0.0, 0.0, TOP), parts, cols, LOOKS)


def _fitment() -> m.Part:
    """The blue upper fitment; its origin is the collar's foot on the tubing's axis."""
    collar = m.box((2 * COLLAR_R, 2 * COLLAR_R, COLLAR_T), (0.0, 0.0, COLLAR_T / 2))  # flats each side: the recess's walls key it square
    cone = m.lathe([(0.0, COLLAR_T), (CONE[0], COLLAR_T), (CONE[1], COLLAR_T + CONE[2]), (0.0, COLLAR_T + CONE[2])], segments=24)
    ribs = m.join(*[m.cylinder(CONE[0] + 0.0005, 0.0012, (0.0, 0.0, COLLAR_T + 0.004 + 0.004 * k), segments=24) for k in range(3)])
    stub = m.cylinder(STUB[0], STUB[1], (0.0, 0.0, -STUB[1] / 2), segments=12)
    tube = m.cylinder(0.0021, 0.03, (0.0, 0.0, COLLAR_T + CONE[2] + 0.015), segments=12)
    parts = [("collar", collar, "blue"), ("cone", cone, "blue"), ("ribs", ribs, "blue"), ("stub", stub, "clamp"), ("tube", tube, "clamp")]
    cols = [m.col_box("collar_c", (2 * COLLAR_R, 2 * COLLAR_R, COLLAR_T), (0.0, 0.0, COLLAR_T / 2)), m.col_hull("cone_c", cone), m.col_hull("stub_c", stub)]
    return m.free("fitment", (HOLDER[0], HOLDER[1], TOP + Z_HOLD), parts, cols, LOOKS, mass=FIT_MASS, friction=(0.6, 0.5), torsional_patch=0.005, solver_iters=48)


CLAMP_Z0 = Z_HOLD - STUB[1] - SEG_L - CLAMP[2] / 2  # the clamp's centre, hanging under the fitment in the holder


def _clamp() -> m.Part:
    body = m.box(CLAMP, (0.0, 0.0, 0.0))
    grip = m.box((0.004, CLAMP[1] + 0.002, CLAMP[2] * 0.6), (-CLAMP[0] / 2 - 0.002, 0.0, 0.0))  # the thumb pad on its front
    tail = m.cylinder(0.0021, 0.02, (0.0, 0.0, -CLAMP[2] / 2 - 0.01), segments=12)
    band = [(f"band_{n}", m.box((0.008, 0.0035, 0.0006), (CLAMP[0] / 2 - 0.004, sy * 0.00525, CLAMP[2] / 2 + 0.0003)), "orange") for sy, n in ((1, "l"), (-1, "r"))]
    parts = [("body", body, "clamp"), ("pad", grip, "blue"), ("tail", tail, "clamp")] + band  # the band on its top's back 8 mm goes under the recess's lip when home
    cols = [m.col_box("body_c", CLAMP), m.col_hull("pad_c", grip)]
    return m.free("sclamp", (HOLDER[0], HOLDER[1], TOP + CLAMP_Z0), parts, cols, LOOKS, mass=CLAMP_MASS, friction=(0.4, 0.3), torsional_patch=0.008, solver_iters=48)


def _segment():
    top = np.array([HOLDER[0], HOLDER[1], TOP + Z_HOLD - STUB[1]])
    n = 6
    nodes = np.array([top - np.array([0.0, 0.0, SEG_L * k / n]) for k in range(n + 1)])
    seg = SEG_L / n
    ei = 2.0e-5  # silicone tubing, 4.4 mm outside: approximated, to be measured
    return cordkit.rope("segment", nodes=nodes, radius=SEG_R, mass=SEG_MASS, friction=0.5, bend_stiffness=ei / seg, bend_damping=ei / seg * 0.05, max_bend=1.2,
                        twist_stiffness=1.0e-4 / seg, twist_damping=2.0e-4, linear_damping=0.5, angular_damping=2.0, contact_offset=0.001,  # torsion raised to GJ 1e-4 N m^2 (silicone tubing about 1e-5), so that the hanging clamp does not spin freely: stated in fidelity
                        solver_position_iterations=32, solver_velocity_iterations=8, anchor_body="fitment", anchor_at=(HOLDER[0], HOLDER[1], TOP + Z_HOLD),
                        anchor_end_body="sclamp", anchor_end_at=(HOLDER[0], HOLDER[1], TOP + CLAMP_Z0), color=(0.92, 0.92, 0.88))


CLIP_K, CLIP_IN = 800.0, 0.0025  # the clip's pins: spring (N/m) and how far each stands into the recess at rest (to be measured)


def _clip() -> m.Part:
    """The clamp recess's sides: a sprung pin each side, ramped front and back, standing 2.5 mm into the 17 mm opening. A clamp pressed in cams them
    aside (each pushes back with up to 2 N, fully aside) and they then hold its sides; a clamp that only swings against the mouth stays out.
    Its own mechanism, so that the pins touch the clamp (kit README 20)."""
    zp, hw = Z_LOW, CLAMP[1] / 2 + 0.0015
    tip = hw - CLIP_IN  # the pin's face at rest, at joint zero, where its spring holds it
    links = []
    for s, n in ((1.0, "l"), (-1.0, "r")):
        outline = [(XF + 0.0005, s * (hw + 0.004)), (XF + 0.0005, s * hw), (XF + 0.0045, s * tip), (XF + 0.0060, s * tip), (XF + 0.0095, s * hw), (XF + 0.0095, s * (hw + 0.004))]
        if s < 0:
            outline = outline[::-1]
        pin = m.prism(outline, zp - 0.009, zp + 0.009)
        links.append(m.slider(f"pin_{n}", "Y", -0.006, 0.006, [(f"pin_{n}", pin, "grey")], [m.col_hull(f"pin_{n}_c", pin)], mass=0.02, spring=CLIP_K, damping=2.0, friction=0.0))
    return m.mechanism("clip", (0.0, 0.0, TOP), links, LOOKS, friction=(0.4, 0.3))


OBJECTS = [_pump(), _clip(), _holder(), _fitment(), _clamp(), _segment()]


# -- the scorer -------------------------------------------------------------------------------

def _yaw(s) -> float:
    R = s.rotation()
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def fit_seated(f) -> dict:
    off = f.pos - np.array([X_SEAT, 0.0, TOP + Z_UP])
    ok = float(np.hypot(off[0], off[1])) <= 0.002 and -0.0015 <= off[2] <= 0.002 and f.tilt_deg() <= 10.0
    return {"ok": bool(ok), "off_mm": [round(1000 * float(v), 1) for v in off], "tilt": round(f.tilt_deg(), 1)}


def clamp_seated(c) -> dict:
    """Home: its back within 1.5 mm of the recess's back stop, centred within 2 mm across and 3 mm up or down, square within 12 degrees. On the pins: its
    back pushed past where the hanging clamp stops, onto the pins' ramps (4 mm or less short of home, centred): a press, not a swing."""
    off = c.pos - np.array([SEATED_X, 0.0, TOP + Z_LOW])
    centred = abs(off[1]) <= 0.002 and abs(off[2]) <= 0.003
    ok = off[0] >= -0.0015 and centred and c.tilt_deg() <= 12.0
    return {"ok": bool(ok), "on_pins": bool(off[0] >= -0.004 and centred), "off_mm": [round(1000 * float(v), 1) for v in off], "tilt": round(c.tilt_deg(), 1)}


def stretch(rope) -> float:
    """The most the segment has opened over any 100 mm of it: the gaps at three joints in a row (four 27 mm links)."""
    g = np.asarray(rope.joint_gaps, float)
    return float(max((g[i:i + 3].sum() for i in range(max(1, len(g) - 2))), default=0.0))


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps, self.hold = 0, 0
        self.fit_first = self.elevated = self.clamp_early = self.dropped = self.stretched = self.full = self.caught = False
        self.net_twist, self.prev_rel = 0.0, None
        self.score = 0.0
        self.report: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        f, c = m.state(env, "fitment"), m.state(env, "sclamp")
        rope = cordkit.rope_state(env, "segment")
        for s in (f, c):
            if s.pos[2] < TOP - 0.05:
                self.dropped = True
        fs, cs = fit_seated(f), clamp_seated(c)
        if cs["ok"] != self.caught:  # the recess's catch holds a clamp that is home, and lets go of one pulled out
            self.caught = cs["ok"]
            m.push(env, "sclamp", (CATCH_N if self.caught else 0.0, 0.0, 0.0))
        if cs["on_pins"] and not self.fit_first:
            self.clamp_early = True  # BD: the upper fitment first (a clamp pressed onto the pins before it, even short of home)
        if fs["ok"]:
            self.fit_first = True
        if self.fit_first and f.pos[2] - (TOP + Z_UP) > 0.003:
            if not self.elevated:
                print(f"[alaris] fitment elevated at step {self.steps}: {fs}", flush=True)
            self.elevated = True  # BD: 'Ensure the upper fitment is not elevated above the fitment recess'
        rel = _yaw(c) - _yaw(f)  # the clamp's turn about the tubing, relative to the fitment: the segment's twist, end to end
        step = (rel - self.prev_rel + 180.0) % 360.0 - 180.0 if self.prev_rel is not None else (rel + 180.0) % 360.0 - 180.0
        self.net_twist += step  # counted through whole turns, so a clamp spun a full turn and pressed home square still reads as twisted
        self.prev_rel = rel
        st = stretch(rope)
        if st > 0.002:  # from the start: a set stretched before the fitment is seated is stretched too
            if not self.stretched:
                print(f"[alaris] segment stretched at step {self.steps}: {1000 * st:.1f} mm over 100 mm", flush=True)
            self.stretched = True
        pts = m.fingertips(env)  # both hands' fingertips
        fit_pts = f.to_world(np.array([[0.0, 0.0, z] for z in (-STUB[1], 0.0, COLLAR_T + CONE[2] / 2, COLLAR_T + CONE[2])]))
        clamp_pts = c.to_world(np.array([[sx * CLAMP[0] / 2, sy * CLAMP[1] / 2, sz * CLAMP[2] / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]))
        watch = np.vstack([fit_pts, clamp_pts, c.pos[None], rope.polyline()])  # points along the whole set: fitment, clamp corners, every joint of the segment
        hands = float(np.min(np.linalg.norm(pts[:, None, :] - watch[None, :, :], axis=2)))
        mo = 0.0
        if not self.dropped:
            if fs["ok"]:
                mo = 0.2
            if fs["ok"] and cs["ok"] and not (self.clamp_early or self.elevated or self.stretched) and abs(self.net_twist) <= 30.0:
                mo = 0.6
        clean = mo == 0.6 and hands >= 0.05
        self.hold = self.hold + 1 if clean else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else mo
        self.report = {"step": self.steps, "score": self.score, "fitment": fs, "clamp": cs, "fit_first": self.fit_first, "clamp_early": self.clamp_early,
                       "elevated": self.elevated, "net_twist": round(self.net_twist, 1), "stretch_mm": round(1000 * st, 2),
                       "stretched": self.stretched, "dropped": self.dropped, "hands_m": round(hands, 3)}
        if self.steps % 20 == 0:
            d = os.environ.get("ALARIS_SCORE_DIR", "alaris_scores")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{os.environ.get('TASK_DEMO', 'run')}.json"), "w") as fh:
                json.dump(self.report, fh, indent=0, default=str)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------

def demo_script(ctx):
    """One hand loads the set; the other arm is not needed and stays at rest. The right hand takes the blue fitment from the holder's fork, out through the
    fork's open end, brings it in front of the upper recess, slides its tubing into the slot and lowers it until its collar sits on the ledge, and lets go.
    It then pinches the hanging safety clamp across its sides and presses it square into its recess below the mechanism, lets go and backs off.

    Failure checks (--mode): none: the arms wait; fitonly: the fitment seated, the clamp left hanging; half: the clamp pushed only into the
    recess's mouth (2 mm in, 8 mm short of home) and let go."""
    import demokit as d

    r = ctx.right
    up = np.array([0.0, 0.0, 1.0])
    if ctx.mode not in ("full", "none", "fitonly", "half"):
        raise ValueError(f"unknown mode {ctx.mode!r}")
    for _ in range(5):
        yield
    if ctx.mode == "none":  # a failure check: the arms wait
        for _ in range(200):
            yield
        print(f"[alaris] final: {json.dumps(TRIAL.report)}", flush=True)
        return
    g = d.level(0.0)  # fingers level toward +x, their pads upright along the fitment's axis, so that it cannot pivot in the grip
    ax = r.approach_axis(g)
    grab = 0.022  # the TCP over the collar's foot: the fingers' lower edges 4 mm above it, clear of the fork and the recess's walls
    tcp = lambda: ctx.pos("fitment") + np.array([0.0, 0.0, grab]) - 0.004 * ax  # noqa: E731  (the real fingertips 4 mm past the axis, 8 mm past the TCP)
    yield from r.move(tcp() - 0.07 * ax, grip=g, t=2.0, label="fitment: behind")
    yield from r.servo(tcp() - 0.07 * ax, timeout=1.0, label="fitment: behind")
    yield from r.move(tcp(), t=1.0, label="fitment: in")
    yield from r.servo(tcp(), timeout=1.0, label="fitment: in")
    yield from r.close(None, label="fitment: grip")
    print(f"[alaris] fitment gripped: at {np.round(1000 * (ctx.pos('fitment') - np.array([0, 0, TOP])), 1)} mm, fingers {np.round(ctx.observed('right')[6], 3)}", flush=True)
    yield from r.move(r.at() + 0.02 * up, t=0.6, label="fitment: off the fork")
    yield from r.move(r.at() + np.array([-0.05, 0.0, 0.02]), t=1.0, label="fitment: out of the fork's open end")  # the segment hangs through the fork, which opens toward -x
    print(f"[alaris] fitment lifted: at {np.round(1000 * (ctx.pos('fitment') - np.array([0, 0, TOP])), 1)} mm, tilt {ctx.state('fitment').tilt_deg():.1f}", flush=True)
    held = lambda: ctx.pos("fitment")  # noqa: E731
    seat = np.array([X_SEAT, 0.0, TOP + Z_UP])
    front = seat + np.array([-0.03, 0.0, 0.008])
    yield from r.move(r.at() + (front - held()), t=2.0, label="fitment: to the recess")
    yield from r.servo(front, read=held, tol=0.001, timeout=2.0, label="fitment: in front")
    yield from r.servo(lambda t: front + (seat + np.array([0.0, 0.0, 0.006]) - front) * min(1.0, t / 1.0), read=held, tol=0.001, timeout=2.0, label="fitment: into the slot")
    yield from r.servo(seat + np.array([0.0, 0.0, 0.0005]), read=held, tol=0.0008, timeout=1.5, label="fitment: down")
    print(f"[alaris] fitment set: {json.dumps(TRIAL.report.get('fitment'))}", flush=True)
    yield from r.open(t=0.4, label="fitment: let go")
    yield from r.move(r.at() - 0.06 * ax, t=1.0, label="fitment: back")
    print(f"[alaris] fitment left at step {TRIAL.steps}: {json.dumps(TRIAL.report.get('fitment'))}, elevated {TRIAL.elevated}", flush=True)
    if ctx.mode == "fitonly":  # a failure check: the fitment seated, the clamp left hanging
        yield from r.move(np.array([0.20, -0.22, TOP + 0.20]), grip=d.down(0.0), t=2.0, label="park")
        yield from r.wait(3.0)
        print(f"[alaris] final: {json.dumps(TRIAL.report)}", flush=True)
        return

    # the clamp, from the front, pinched across its sides 8 mm deep (the grip sets its yaw; the recess's lead-in squares its pitch), pressed home
    gc = d.level(0.0)  # across its sides: the grip holds its yaw; its pitch is free to follow the recess's lead-in
    cl = lambda: ctx.pos("sclamp")  # noqa: E731
    yield from r.move(cl() + np.array([-0.08, 0.0, 0.0]), grip=gc, finger=-0.02, t=2.0, label="clamp: in front")
    yield from r.servo(cl() + np.array([-0.08, 0.0, 0.0]), timeout=1.0, label="clamp: in front")
    print(f"[alaris] clamp hanging: at {np.round(1000 * (cl() - np.array([0, 0, TOP])), 1)} mm, yaw {_yaw(ctx.state('sclamp')):.1f}, tilt {ctx.state('sclamp').tilt_deg():.1f}", flush=True)
    pinch = lambda: cl() + np.array([-CLAMP[0] / 2 + 0.008 - 0.013, 0.0, 0.0])  # noqa: E731  (the fingertips 8 mm onto its sides; 2 mm off the face when it is home, its body 10 mm proud)
    yield from r.move(pinch(), t=1.2, label="clamp: round it")
    yield from r.servo(pinch(), timeout=1.0, label="clamp: round it")
    yield from r.close(None, label="clamp: pinch")
    print(f"[alaris] clamp pinched at step {TRIAL.steps}: elevated {TRIAL.elevated}", flush=True)
    away = np.array([XF - 0.03, 0.0, TOP + Z_LOW])
    home = np.array([SEATED_X + 0.0005, 0.0, TOP + Z_LOW])  # aimed at the recess's back stop
    yield from r.servo(away, read=cl, tol=0.0015, timeout=2.0, label="clamp: square to the recess")
    if ctx.mode == "half":  # a lazy-route check: the clamp pushed only into the recess's mouth, not home
        home = np.array([XF - CLAMP[0] / 2 + 0.002, 0.0, TOP + Z_LOW])  # its back 2 mm into the recess, 8 mm short of home
    yield from r.servo(lambda t: away + (home - away) * min(1.0, t / 1.5), read=cl, tol=0.001, timeout=3.0, label="clamp: pressed in")
    print(f"[alaris] clamp pressed: {json.dumps(TRIAL.report)}, clip pins {ctx.state('clip').joints}", flush=True)
    yield from r.open(t=0.4, label="clamp: let go")
    yield from r.move(r.at() + np.array([-0.08, 0.0, 0.02]), t=1.0, label="clamp: back")
    yield from r.move(np.array([0.20, -0.22, TOP + 0.20]), grip=d.down(0.0), t=2.0, label="park")
    yield from r.wait(2.5)
    print(f"[alaris] final: {json.dumps(TRIAL.report)}", flush=True)
