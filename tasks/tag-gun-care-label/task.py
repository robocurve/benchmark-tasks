"""Ticket a garment through its care label only: lift the label clear of the body fabric, push a tagging gun's needle through the label alone, fire one
fastener and draw the needle back.

A folded cotton T-shirt lies flat on the table, its hem turned back at one side so the care label sewn into the side seam lies exposed on top (30 x 60 mm,
sewn along its short edge, its free edge curled up about 8 mm). A standard tagging gun (Avery Dennison Mark III class: 125 x 25 x 130 mm, 100 g by a
retailer's listing) lies on its side beside it with a card price ticket on its needle.

Sim (stated, all in code): the shirt's body is a stiff panel and the label a flap hinged along its sewn edge, its curl a light spring (0.01 N m/rad toward
8 degrees); cloth is not simulated. The needle has no collider: the scorer checks its tip against the body panel's volume (a pierce, recorded as a failure)
and against the label's (the passage: its tip must cross the label's face inside the outline, gated on the label standing at least 2 cm clear of the
body; a needle slid in sideways never counts, and one dragged sideways out through the label's edge tears it). The trigger is a sprung squeeze lever along the grip's
front (25 N preload); a fastener fires when it travels 2.5 mm with the needle through the label.

Full score: one fastener through the label alone, the body never pierced, the needle drawn back out, the label let go (down under 30 degrees, every
fingertip of both hands 5 cm from it), held 1 s. Partial: 0.5 the needle through the label alone; 0.25 the label lifted 2 cm at its free edge. A tip
inside the shirt's body, or within 4 mm above its top face, is a pierce.

    python demokit.py task.py OUT --mode scene --stills 1
    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Put the price ticket on this shirt through its care label only: lift the label clear of the shirt with one hand, push the tagging gun's "
               "needle through the label alone with the other, fire one fastener, draw the needle back, then let the label down and let go of it.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20

SHIRT = np.array([0.32, -0.04])  # the folded shirt's middle
SHIRT_SIZE = (0.24, 0.30, 0.016)  # folded: 24 x 30 cm, 16 mm thick
LABEL_W, LABEL_L, LABEL_T = 0.030, 0.060, 0.0006
SEAM = np.array([0.32, 0.005, SHIRT_SIZE[2]])  # the label's sewn edge, along x, on the turned-back hem
GUN0 = np.array([0.12, -0.16])  # where the gun lies, its grip's middle
FWD, UP = 0.115, 0.06
NEEDLE_L = 0.025  # the needle's length beyond the gun's nose
PIERCE_U = 0.037  # where the demo pierces the label: 37 mm out from its sewn edge, 2.4 cm clear of the body with the label held at about 40 degrees
HOLD_U = 0.048  # where the left pads pinch the label: 12 mm in from its free edge
CURL_DEG, CURL_K = 8.0, 0.01  # the free edge's curl: the label rests 8 degrees up (its free edge about 8 mm up) and springs back toward it at 0.01 N m/rad (unmeasured: an approximation of a satin label's set)

LOOKS = {"cotton": m.Look(color=m.rgb(0.10, 0.18, 0.36), roughness=0.95), "hem": m.Look(color=m.rgb(0.08, 0.15, 0.31), roughness=0.95),
         "label": m.Look(color=m.rgb(0.94, 0.94, 0.90), roughness=0.45), "print": m.Look(color=m.rgb(0.35, 0.35, 0.38), roughness=0.6),
         "gun": m.Look(color=m.rgb(0.82, 0.83, 0.85), roughness=0.5), "grip": m.Look(color=m.rgb(0.20, 0.21, 0.23), roughness=0.6),
         "trigger": m.Look(color=m.rgb(0.75, 0.12, 0.10), roughness=0.5), "steel": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=1.0, roughness=0.25),
         "ticket": m.Look(color=m.rgb(0.97, 0.96, 0.92), roughness=0.7), "price": m.Look(color=m.rgb(0.80, 0.10, 0.10), roughness=0.6),
         "fastener": m.Look(color=m.rgb(0.92, 0.92, 0.92), roughness=0.4)}


def _shirt() -> m.Part:
    """The folded shirt as a stiff panel, its hem turned back at the +y side seam, and the care label hinged along its sewn edge (x), lying flat toward +y."""
    sx, sy, sz = SHIRT_SIZE
    frame = [("body", m.box(SHIRT_SIZE, (0.0, 0.0, sz / 2)), "cotton"),
             ("hem", m.box((sx, 0.05, 0.004), (0.0, sy / 2 - 0.035, sz + 0.002)), "hem"),  # the turned-back hem, the label sewn under its fold
             ("collar", m.box((0.10, 0.012, 0.003), (0.0, -sy / 2 + 0.03, sz + 0.0015)), "hem"),
             ("fold", m.box((sx, 0.003, 0.0012), (0.0, -0.02, sz + 0.0006)), "hem")]
    cols = [m.col_box("body_c", SHIRT_SIZE, (0.0, 0.0, sz / 2))]
    at = (SEAM[0] - SHIRT[0], SEAM[1] - SHIRT[1], sz + 0.004)
    lab = m.box((LABEL_W, LABEL_L, LABEL_T), (at[0], at[1] + LABEL_L / 2, at[2] + LABEL_T / 2))
    curl = m.box((LABEL_W, 0.004, LABEL_T), (at[0], at[1] + LABEL_L - 0.0005, at[2] + 0.002))  # a lip along the free edge; the label itself rests 8 degrees up on its curl spring
    prints = [(f"print{k}", m.box((0.020, 0.0015, 0.0002), (at[0], at[1] + 0.012 + 0.008 * k, at[2] + LABEL_T + 0.0001)), "print") for k in range(5)]
    # once fired: the fastener's T-end through the label, its barb running out past the free edge to the card ticket, which lies on the shirt beyond the label
    tag = [("tag_fastener", m.box((0.0015, 0.0015, 0.006), (at[0], at[1] + PIERCE_U, at[2])), "fastener", True),
           ("tag_barb", m.box((0.0012, 0.040, 0.0012), (at[0], at[1] + PIERCE_U + 0.020, at[2] + LABEL_T + 0.0012)), "fastener", True),
           ("tag_ticket", m.box((0.030, 0.050, 0.0006), (at[0], at[1] + PIERCE_U + 0.065, at[2] + LABEL_T + 0.0015)), "ticket", True),
           ("tag_price", m.box((0.016, 0.010, 0.0003), (at[0], at[1] + PIERCE_U + 0.060, at[2] + LABEL_T + 0.002)), "price", True)]
    label = m.hinge("label", "X", at, 0.0, 120.0, [("label", lab, "label"), ("curl", curl, "label"), *prints, *tag],
                    [m.col_box("label_c", (LABEL_W, LABEL_L, 0.0015), (at[0], at[1] + LABEL_L / 2, at[2] + 0.00075))], mass=0.002, spring=CURL_K, damping=0.003, friction=0.0)
    return m.mechanism("shirt", (SHIRT[0], SHIRT[1], TOP), [label], LOOKS, frame_parts=frame, frame_colliders=cols, friction=(0.9, 0.8))


TRIG_TRAVEL, TRIG_FIRE = 0.004, 0.0025  # the trigger's travel back into the grip, and how far it must go to fire (unmeasured: approximations)
TRIG_K, TRIG_PRE = 2000.0, 25.0  # its spring: 25 N preload at rest, 33 N at full travel (unmeasured): a carrying pinch of 20 N does not move it, a 50 N squeeze fires it


def _gun() -> m.Part:
    """The tagging gun in its own frame: origin at the grip's middle, +y forward along the barrel and needle, +x up from the grip to the barrel, z its thickness.
    It lies on its side, so its frame is the table's. The trigger slides back into the grip against its spring; the needle has no collider."""
    t = 0.025
    frame = [("barrel", m.box((0.040, 0.120, t), (UP, 0.030, 0.0)), "gun"),
             ("grip", m.box((0.090, 0.030, t * 0.9), (0.0, -0.012, 0.0)), "grip"),
             ("nose", m.cylinder(0.006, 0.010, (UP, 0.095, 0.0), axis="y"), "steel"),
             ("needle", m.cylinder(0.001, 0.025, (UP, FWD - 0.0125, 0.0), axis="y"), "steel"),
             ("clip", m.box((0.010, 0.040, t * 0.7), (UP + 0.024, 0.0, 0.0)), "fastener"),
             ("ticket", m.box((0.050, 0.0006, 0.030), (UP - 0.022, 0.104, 0.0)), "ticket"),  # the card ticket on the needle
             ("price", m.box((0.016, 0.0003, 0.010), (UP - 0.030, 0.1035, 0.0)), "price")]
    cols = [m.col_box("barrel_c", (0.040, 0.120, t), (UP, 0.030, 0.0)), m.col_box("grip_c", (0.090, 0.030, t * 0.9), (0.0, -0.012, 0.0))]
    trig = m.box((0.080, 0.010, t * 0.9), (0.0, 0.012, 0.0))  # a squeeze lever along the grip's whole front: a finger on the front always bears on it
    trigger = m.slider("trigger", "Y", -TRIG_TRAVEL, 0.0, [("trigger", trig, "trigger")], [m.col_box("trigger_c", (0.080, 0.010, t * 0.9), (0.0, 0.012, 0.0))],
                       mass=0.004, spring=TRIG_K, damping=2.0, friction=0.0, effort=60.0)
    return m.mechanism("gun", (GUN0[0], GUN0[1], TOP + t / 2 + 0.0005), [trigger], LOOKS, frame_parts=frame, frame_colliders=cols, free_mass=0.10,
                       friction=(0.8, 0.7), frame_inertia=(1.6e-4, 0.6e-4, 2.0e-4))


OBJECTS = [_shirt(), _gun()]


def _rotx(a: float) -> np.ndarray:
    c, sn = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -sn], [0.0, sn, c]])


def label_frame(env):
    """The label's hinge point and rotation in the world (the shirt is fixed), and its angle."""
    sh = m.state(env, "shirt")
    a = float(sh.joint("label"))
    hinge = sh.to_world(np.array([[SEAM[0] - SHIRT[0], SEAM[1] - SHIRT[1], SHIRT_SIZE[2] + 0.004]]))[0]
    return hinge, sh.rotation() @ _rotx(a), a


def needle_tip(env):
    g = m.state(env, "gun")
    return g.to_world(np.array([[UP, FWD, 0.0]]))[0], g


def let_go(tips, mid, a) -> bool:
    """The label let down under 30 degrees and let go: every fingertip of both hands 5 cm or more from the label's middle."""
    return bool(np.min(np.linalg.norm(np.asarray(tips) - mid, axis=1)) > 0.05 and a < math.radians(30.0))


def needle_through(tip, base_pt, hinge, Rl, top):
    """Whether the needle is through the label now: its tip and its base on opposite sides of the label's face, crossing it inside its outline,
    where the label stands 2 cm or more over the shirt (`top`, the height of the label's seam). Returns that and the label's clearance there."""
    rt, rb = Rl.T @ (np.asarray(tip) - hinge), Rl.T @ (np.asarray(base_pt) - hinge)
    ht, hb = rt[2] - LABEL_T / 2, rb[2] - LABEL_T / 2
    if ht * hb >= 0.0:
        return False, 0.0
    c = rb + (rt - rb) * (hb / (hb - ht))  # where the needle crosses the label's plane
    clear = float((hinge + Rl @ np.array([0.0, c[1], 0.0]))[2] - top)
    return bool(0.0 < c[1] < LABEL_L and abs(c[0]) < LABEL_W / 2 and clear >= 0.02), clear


def track_entry(entered, prev_rt, tip, base_pt, hinge, Rl):
    """The needle's entry into the label, which needs memory: the needle has no collider, so it could straddle the label's plane beside the label and
    slide in sideways. Entered: the tip has crossed the label's face inside its outline, and the needle has straddled the face, crossing it inside the
    outline, at every step since. Returns (entered, the tip in label coordinates for the next step, torn): torn when an entered needle's crossing
    leaves the outline sideways while it still straddles the face (dragged out through the label's edge)."""
    rt, rb = Rl.T @ (np.asarray(tip) - hinge), Rl.T @ (np.asarray(base_pt) - hinge)
    ht, hb = rt[2] - LABEL_T / 2, rb[2] - LABEL_T / 2
    inside = lambda q: bool(0.0 < q[1] < LABEL_L and abs(q[0]) < LABEL_W / 2)  # noqa: E731
    straddle = ht * hb < 0.0
    crossing_in = straddle and inside(rb + (rt - rb) * (hb / (hb - ht)))
    torn = False
    if prev_rt is not None:
        hp = prev_rt[2] - LABEL_T / 2
        if hp * ht < 0.0 and inside(prev_rt + (rt - prev_rt) * (hp / (hp - ht))) and crossing_in:
            entered = True  # the tip went through the face, inside the outline, this step
    if entered and straddle and not crossing_in:
        entered, torn = False, True
    elif entered and not straddle:
        entered = False  # drawn back out of the label
    return entered, rt, torn


class Tag:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.pierced, self.torn, self.through, self.fired, self.fastened, self.lifted, self.gun_fell = 0, False, False, False, False, False, 0.0, False
        self.entered, self.prev_rt = False, None
        self.hold, self.full, self.last, self.shown = 0, False, {}, False
        if env is None:  # at import, before the simulator runs: the parts start hidden by themselves (touching the stage here breaks the app's start)
            return
        for path, vis in [(f"{m.ENV_PATH}/shirt/label/visual/{nm}", False) for nm in ("tag_fastener", "tag_barb", "tag_ticket", "tag_price")] + \
                         [(f"{m.ENV_PATH}/gun/frame/visual/{nm}", True) for nm in ("ticket", "price")]:
            try:  # the ticket on the gun's needle, nothing on the label
                m.set_visible(path, vis)
            except Exception:
                pass

    def step(self, env):
        self.steps += 1
        m.set_target(env, "gun", "trigger", TRIG_PRE / TRIG_K)  # the spring's rest beyond the stop: its preload
        m.set_target(env, "shirt", "label", math.radians(CURL_DEG))  # the label's curl
        hinge, Rl, a = label_frame(env)
        tip, g = needle_tip(env)
        top = TOP + SHIRT_SIZE[2] + 0.004
        # the body: the folded shirt's panel (and the hem on it): a tip inside it is a pierce, at any moment
        sx, sy, _ = SHIRT_SIZE
        if abs(tip[0] - SHIRT[0]) < sx / 2 and abs(tip[1] - SHIRT[1]) < sy / 2 and tip[2] < top:
            self.pierced = True
        if a > math.radians(110.0):
            self.torn = True  # pulled past what its sewn edge allows
        self.lifted = max(self.lifted, LABEL_L * math.sin(a))
        # through now: the needle's tip and its base on opposite sides of the label's face, the needle crossing it inside its outline, where the
        # label stands 2 cm or more clear of the shirt
        base_pt = tip - NEEDLE_L * (g.rotation() @ np.array([0.0, 1.0, 0.0]))
        in_label, clear_cm = needle_through(tip, base_pt, hinge, Rl, top)
        self.entered, self.prev_rt, ripped = track_entry(self.entered, self.prev_rt, tip, base_pt, hinge, Rl)
        if ripped:
            self.torn = True  # the needle dragged sideways out through the label's edge
        in_label = bool(in_label and self.entered)  # through only by way of the label's face
        rt = Rl.T @ (tip - hinge)
        u, w, h = rt[1], rt[0], rt[2] - LABEL_T / 2
        self.through = self.through or in_label
        travel = -float(g.joint("trigger"))
        if travel >= TRIG_FIRE and not self.fired:
            self.fired = True
            self.fastened = bool(in_label and not self.pierced)  # only the first squeeze counts: a fastener through the label alone if the needle is through it now, else wasted
        out = not in_label
        gun_down = bool(g.pos[2] - TOP < 0.03 and 0.0 < g.pos[0] < 0.6 and abs(g.pos[1]) < 0.6)
        if g.pos[2] < TOP - 0.05:
            self.gun_fell = True  # off the table
        mid = hinge + Rl @ np.array([0.0, LABEL_L / 2, 0.0])
        out = bool(out and np.linalg.norm(tip - mid) > 0.04)  # the needle drawn back out of the label
        clear = let_go(m.fingertips(env), mid, a)
        if self.shown != self.fastened:
            self.shown = self.fastened
            for nm in ("tag_fastener", "tag_barb", "tag_ticket", "tag_price"):
                try:
                    m.set_visible(f"{m.ENV_PATH}/shirt/label/visual/{nm}", self.fastened)
                except Exception:
                    pass
            for nm in ("ticket", "price"):
                try:
                    m.set_visible(f"{m.ENV_PATH}/gun/frame/visual/{nm}", not self.fastened)
                except Exception:
                    pass
        self.last = {"t": round(self.steps * float(env.unwrapped.step_dt), 2), "label_deg": round(math.degrees(a), 1), "lifted_mm": round(self.lifted * 1000, 1),
                     "tip_label_mm": [round(u * 1000, 1), round(w * 1000, 1), round(h * 1000, 1)], "clear_of_body_mm": round(clear_cm * 1000, 1),
                     "through": self.through, "in_label": bool(in_label), "trigger_mm": round(travel * 1000, 2), "fired": self.fired, "fastened": self.fastened,
                     "pierced": self.pierced, "torn": self.torn, "gun_fell": self.gun_fell, "out": bool(out), "gun_down": gun_down, "clear": clear,
                     "tip_above_body_mm": round((tip[2] - top) * 1000, 1)}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        if self.steps % 10 == 0:
            print("[tag] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    if not r or r["pierced"] or r["torn"] or r["gun_fell"]:
        return 0.0
    if r["fastened"] and r["out"] and r["clear"]:
        return 1.0
    if r["through"]:
        return 0.5
    if r["lifted_mm"] >= 20.0:
        return 0.25
    return 0.0


TAG = Tag()
m.on_step(TAG.step)
m.on_reset(TAG.reset)


def score(objects) -> float:
    return 1.0 if TAG.full else min(_tier(TAG.last), 0.5)


def success(objects) -> bool:
    return TAG.full


# -- the scripted demo ----------------------------------------------------------------------------

G_DOWN = 0.0  # the needle level as it goes in
LIFT_DEG = 60.0  # the label swung up to this, pinched: the barrel's lower front edge then stays about 12 mm off it
PUSH_PAST = 0.004  # how far the needle's tip goes beyond the label's face before the squeeze
CLEAR_MIN = 0.024  # pierce where the label stands at least this far above the body (the scorer gates the passage at 2 cm)


def demo_script(ctx):
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    if ctx.mode == "scene":
        yield from L.wait(1.5)
        print("[tag] END " + json.dumps({"mode": ctx.mode, **TAG.last}), flush=True)
        return
    nolift = ctx.mode == "nolift"  # failure check: the label left lying on the shirt, the needle pushed at it as in the demo
    hinge = label_frame(env)[0]

    def edge(deg, u=LABEL_L):
        return hinge + u * np.array([0.0, math.cos(math.radians(deg)), math.sin(math.radians(deg))])

    # the left hand: pads round the curled free edge, 12 mm in, the fingers pointing along the label toward its seam (tilted 60 degrees from straight down, rolled
    # to close across the label: the wrist leans well away from the right hand's); a light pinch, then the label swung up about its seam
    gl0 = d.tilted(60.0, -90.0)
    gl = d._as_grip(gl0.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.pi / 2))
    a_rest = label_frame(env)[2]
    back = -L.approach_axis(gl)
    P = edge(math.degrees(a_rest), HOLD_U) + np.array([0.0, 0.0, 0.001])
    if not nolift:
        yield from L.move(P + 0.06 * back + np.array([0.0, 0.0, 0.02]), grip=gl, finger=d.FINGER_OPEN, t=2.0, label="behind the label's curled free edge")
        yield from L.move(P + 0.03 * back, t=0.8, label="jaws either side of the free edge")
        yield from L.move(P, t=0.8, label="pads 12 mm in from the edge")
        yield from L.close(label="pinch the label")
        yield from L.arc(hinge, (1.0, 0.0, 0.0), LIFT_DEG - math.degrees(a_rest), t=2.0, label=f"swing the label up about its seam to {LIFT_DEG:.0f} degrees")
    print("[tag] lifted: " + json.dumps(TAG.last), flush=True)
    # the right hand: the gun from the table, the fingers across the grip's back and the trigger, a light pinch under the trigger's preload
    g0 = d.down(0.0)
    grip = m.state(env, "gun").pos
    yield from R.move(grip + np.array([0.0, 0.0, 0.07]), grip=g0, finger=d.FINGER_OPEN, t=2.0, label="over the gun's grip")
    yield from R.move(grip, t=1.0, label="fingers either side of the grip and trigger")
    yield from R.close(0.044, force=20.0, label="hold the gun at 20 N, under the trigger's 25 N preload")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lift the gun")
    tip, _ = needle_tip(env)
    in_hand = R.grip.rot.T @ (tip - R.at())  # the needle's tip in the hand's frame
    n = np.array([0.0, math.cos(math.radians(G_DOWN)), -math.sin(math.radians(G_DOWN))])
    aim = d._as_grip(_rotx(-math.radians(G_DOWN)) @ g0.rot)
    tip_off = aim.rot @ in_hand
    h0, R0, a0 = label_frame(env)  # the label as the left hand holds it now
    u = float(np.clip(CLEAR_MIN / max(math.sin(a0), 0.2), PIERCE_U, 0.040))  # far enough out to stand 2.4 cm clear, short of the pads
    pierce = h0 + R0 @ np.array([0.0, u, 0.0])
    print("[tag] aim: " + json.dumps({"label_deg": round(math.degrees(a0), 1), "pierce_u_mm": round(u * 1000, 1), "left_tcp_rel_hinge_mm": [round(float(v) * 1000, 1) for v in L.at() - h0]}), flush=True)
    yield from R.move(pierce - 0.05 * n - tip_off, grip=aim, t=2.0, label="aim the needle at the label")
    yield from R.servo(pierce - 0.05 * n, tol=0.0015, timeout=1.5, read=lambda: needle_tip(env)[0], label="the needle's tip 5 cm short of the label")
    yield from R.servo(lambda s: pierce + (-0.05 + min(s / 1.5, 1.0) * (0.05 + PUSH_PAST)) * n, tol=0.0015, timeout=2.5, read=lambda: needle_tip(env)[0],
                       label="push the needle through the label")
    print("[tag] through: " + json.dumps(TAG.last), flush=True)
    # squeeze until the fastener fires, then ease off at once: held on its stop, the trigger would let the gun slip in the hand, and the jolt would
    # shake the label out of the left hand's pinch
    for _ in R.close(0.030, force=50.0, t=1.0, label="squeeze at up to 50 N: the trigger fires a fastener"):
        yield
        if TAG.fired:
            break
    yield from R.close(0.044, force=20.0, t=0.3, label="ease off the trigger as it fires")
    print("[tag] fired: " + json.dumps(TAG.last), flush=True)
    yield from R.move(R.at() - 0.06 * n, t=1.2, label="draw the needle back")
    yield from R.move(R.at() + np.array([-0.06, -0.08, 0.04]), t=1.5, label="the gun away from the shirt, held")
    # the left lets the label down with the ticket on it
    # let it down to 25 degrees, open the jaws 12 mm (fully open, the lower jaw would go into the shirt), and draw them out level so the
    # label slips off the lower jaw's end onto the shirt
    if not nolift:
        yield from L.arc(hinge, (1.0, 0.0, 0.0), 25.0 - LIFT_DEG, t=1.5, label="let the label down to 25 degrees")
        yield from L.move(L.at(), finger=-0.006, t=0.5, label="let go of the label: jaws 12 mm apart")
        yield from L.move(L.at() + np.array([0.0, 0.05, 0.0]), t=1.2, label="draw the jaws out level, off the label's edge")
        yield from L.move(L.at() + np.array([0.0, 0.02, 0.06]), t=1.0, label="left clear")
    yield from L.wait(1.5)
    print("[tag] END " + json.dumps({"mode": ctx.mode, **TAG.last}), flush=True)
