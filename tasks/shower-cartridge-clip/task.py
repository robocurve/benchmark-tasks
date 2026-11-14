"""Put a shower valve's cartridge back and fit its retainer clip.

A Moen 1222 type cartridge (105 mm body, 25 mm across, 0.15 kg) goes into its valve body's bore against its O-rings' drag. The U-shaped retainer clip slides
down two slots in the valve's front lugs, its legs passing just in front of the cartridge's two ears. It passes only when the ears sit fully home in their
notches in the valve face, so the cartridge is pushed flush and held there while the clip goes down. Let go before the clip is in, and it creeps back 1 mm.

Model: the valve body, its bore (boxes, 1 mm clearance with a lead-in), the face notches and the slotted lugs are one fixed body; the cartridge slides
and turns in it on two joints, and the clip is a free body. Declared code: the O-rings' drag, a force against the cartridge going in over its last 15 mm
(only while it moves in; 15 N, approximated), and their spring-back, which pushes a cartridge left within 1 mm of home back out to 1 mm proud. The clip is
2 mm thick and 10 g (a real one is about 1.5 mm and 5 g), its slots 2.2 mm wide; the ears reach 17.5 mm off the axis. The scorer pulls the stem outward with
20 N through its 1 s hold.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --mode letgo           # cheat checks, each in its own session: let go before the clip;
    python demokit.py task.py OUT --no-record --mode proud           # the cartridge left proud; the arms do nothing
    python demokit.py task.py OUT --no-record --mode none
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Push the cartridge home, fit its retainer clip, then let go."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold: score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz
PULL_N = 20.0  # the scorer pulls the stem outward this hard through the hold: a clipped cartridge stays flush (its ears against the clip's legs)
TOP = TABLE_TOP[SCENE]

# -- the valve body (fixed): its face at x XV facing the arms, the bore along +x, its axis 25 cm above the table at y 0
XV, ZV, VY = 0.28, 0.07, 0.0  # the right hand's level push on the stem and the left hand's down grip over the slots both reach here (IK check)
BORE_R, BORE_D = 0.0135, 0.075  # 1 mm clearance round the 25 mm body: the real bore is about 0.3 mm, but has a lead-in chamfer the sketch leaves out
EAR = (0.003, 0.006, float(os.environ.get("EAR_R", "0.005")))  # x (thick), z (wide), radial: out to 17.5 mm, 3 mm under the clip's legs (14.5 to 18.5 mm); approximated; with only 2 mm of overlap a 20 N pull slides an ear past a leg, and with 6 mm the clip tips as it goes down
EAR_Y = 0.0125 + EAR[2] / 2
NOTCH = (0.0032, 0.0066, 0.0065)
LUG_X0, LUG_X1 = XV - 0.0034, XV  # the lugs stand proud of the face; the clip's slot runs between their front lip and the face itself
SLOT = (XV - float(os.environ.get("SLOT_W", "0.0022")), XV)  # 2.2 mm for the 2 mm legs: a clipped cartridge sits at most 0.2 mm out, plus the contact's give (with 2.4 mm a clipped cartridge rests 0.6 mm out, past the 0.5 mm flush limit)
LEG_Y = 0.0165  # the clip's legs (14.5 to 18.5 mm off the axis) pass in front of the ears' outer 2 mm, outside the bore
SLOT_WALLS, SLOT_PLAY = bool(int(os.environ.get("SLOT_WALLS", "0"))), 0.0004  # the slots' sides, and the legs' play between them, each way
FACE_T = 0.0035  # the face plate, as deep as the ears' notches
LEAD_R, LEAD_L = 0.0155, 0.012  # a lead-in: the opening and the bore's first 12 mm are wider, then the bore proper
NOTCH_Z = 0.005  # the notches' half height: an ear turned 8 degrees or more does not go in
LUG_TOP, LUG_BOT = ZV + 0.030, ZV - 0.024

# -- the cartridge (free): origin on its axis at the ears' front faces; it lies along x, the stem towards -x
C_R, C_L, C_MASS = 0.0125, 0.105, 0.15
STEM_R, STEM_L = 0.0055, 0.030
DRAG_N, DRAG_ZONE, CREEP = float(os.environ.get("DRAG_N", "15")), 0.015, 0.001  # assumed until measured on a real cartridge
# -- the clip (free): a U, crossbar on top, two legs 2 mm thick
CLIP_T, LEG_W, LEG_L = 0.002, 0.004, 0.050
CLIP_MASS = float(os.environ.get("CLIP_MASS", "0.010"))  # a real clip is about 5 g: at 5 g and 32 solver iterations the 20 N pull drove the ears through its legs

LOOKS = {
    "brass": m.Look(color=m.rgb(0.78, 0.62, 0.30), roughness=0.35, metallic=1.0),
    "wall": m.Look(color=m.rgb(0.88, 0.87, 0.84), roughness=0.9),
    "white": m.Look(color=m.rgb(0.93, 0.93, 0.92), roughness=0.4),
    "steel": m.Look(color=m.rgb(0.70, 0.71, 0.72), roughness=0.25, metallic=1.0),
    "black": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.6),
    "wood": m.Look(color=m.rgb(0.62, 0.45, 0.28), roughness=0.7),
    "copper": m.Look(color=m.rgb(0.72, 0.40, 0.22), roughness=0.3, metallic=1.0),
    "hot": m.Look(color=m.rgb(0.75, 0.08, 0.06), roughness=0.5),
    "cold": m.Look(color=m.rgb(0.08, 0.22, 0.70), roughness=0.5),
    "cavity": m.Look(color=m.rgb(0.16, 0.14, 0.12), roughness=0.9),
}


def _box(name, size, c, look):
    return (name, m.box(size, c), look), m.col_box(name + "_c", size, c)


def _valve():
    """The bore (boxes) behind a face plate with two notches for the ears, the front lugs with the clip's slots (cut away where the ears pass), the body, the wall."""
    items = []
    n = 16
    for ring, r_in, x0, x1 in (("lead", LEAD_R, XV + FACE_T, XV + FACE_T + LEAD_L), ("bore", BORE_R, XV + FACE_T + LEAD_L, XV + FACE_T + BORE_D)):
        for k in range(n):
            a = 2 * math.pi * k / n
            ch = 2 * (r_in + 0.003) * math.tan(math.pi / n) * 1.04
            seg = m.box((x1 - x0, 0.006, ch), ((x0 + x1) / 2, 0.0, 0.0)).moved((0.0, 0.0, r_in + 0.003)).turned(math.degrees(a), "x").moved((0.0, 0.0, ZV))
            items.append(((f"{ring}{k}", seg, "brass"), m.col_hull(f"{ring}{k}_c", seg)))
    xc, h = XV + FACE_T / 2, 0.035
    rr, ro = LEAD_R, EAR_Y + EAR[2] / 2 + 0.0007  # the opening's half width (the lead-in), and the notches' outer edge
    face = [("top", (FACE_T, 2 * h, h - rr), (xc, 0.0, ZV + rr + (h - rr) / 2)), ("bot", (FACE_T, 2 * h, h - rr), (xc, 0.0, ZV - rr - (h - rr) / 2))]
    for side, sg in (("l", 1), ("r", -1)):
        face.append((f"out_{side}", (FACE_T, h - ro, 2 * rr), (xc, sg * (ro + (h - ro) / 2), ZV)))
        for zs, zn in ((1, "u"), (-1, "d")):
            face.append((f"by_{side}{zn}", (FACE_T, ro - rr, rr - NOTCH_Z), (xc, sg * (rr + (ro - rr) / 2), ZV + zs * (NOTCH_Z + (rr - NOTCH_Z) / 2))))
    for nm, size, c in face:
        items.append(_box("face_" + nm, size, c, "brass"))
    for side, sg in (("l", 1), ("r", -1)):  # the lugs: a slot for each leg, open where the ear passes (|z - ZV| < NOTCH_Z)
        y = sg * 0.0175  # the lugs run from 14.5 mm off the axis (outside the bore) to 20.5 mm; the legs slide in them
        for nm, x0, x1 in (("f", LUG_X0, SLOT[0]),):
            for zn, za, zb in (("u", ZV + NOTCH_Z, LUG_TOP), ("d", LUG_BOT, ZV - NOTCH_Z)):
                items.append(_box(f"lug_{side}{nm}{zn}", (x1 - x0, 0.006, zb - za), ((x0 + x1) / 2, y, (za + zb) / 2), "brass"))
        items.append(_box(f"lug_{side}_stop", (SLOT[1] - SLOT[0], 0.006, 0.004), ((SLOT[0] + SLOT[1]) / 2, y, LUG_BOT - 0.002), "brass"))
        if SLOT_WALLS:  # the slot's sides: each leg held within SLOT_PLAY across, above and below the ears' window (open there, so the ears pass)
            for wn, wy in (("in", LEG_Y - LEG_W / 2 - SLOT_PLAY - 0.0005), ("out", LEG_Y + LEG_W / 2 + SLOT_PLAY + 0.0005)):
                for zn, za, zb in (("u", ZV + NOTCH_Z, LUG_TOP), ("d", LUG_BOT, ZV - NOTCH_Z)):
                    items.append(_box(f"lug_{side}_{wn}{zn}", (SLOT[1] - SLOT[0], 0.001, zb - za), ((SLOT[0] + SLOT[1]) / 2, sg * wy, (za + zb) / 2), "brass"))
    parts = [p for p, _ in items]
    cols = [c for _, c in items]
    body = m.box((BORE_D + 0.01, 0.07, 0.07), (XV + FACE_T + 0.005 + BORE_D / 2, 0.0, ZV))
    wall = [("stud", m.box((0.04, 0.09, 0.40), (XV + 0.13, 0.0, 0.20)), "wood"), ("board", m.box((0.012, 0.30, 0.36), (XV + FACE_T + BORE_D + 0.015, 0.0, 0.20)), "wall")]
    parts += [("valve", body, "brass")] + wall
    # what makes it a shower valve at a glance (visual only, out of the hands' way): the copper supplies, hot on the left and cold on the right, the outlet
    # up to the shower head, colour bands on the supplies, and the opening cut in the wall board behind the valve
    bx = XV + FACE_T + 0.005 + BORE_D / 2
    pr = 0.0079  # 15 mm copper
    for nm, sg, look in (("hot", 1, "hot"), ("cold", -1, "cold")):
        parts.append((f"supply_{nm}", m.cylinder(pr, 0.075, (bx + 0.01, sg * (0.035 + 0.0375), ZV - 0.012), axis="y"), "copper"))
        parts.append((f"band_{nm}", m.cylinder(pr + 0.0006, 0.012, (bx + 0.01, sg * 0.095, ZV - 0.012), axis="y"), look))
    parts.append(("outlet", m.cylinder(pr, 0.06, (bx + 0.025, 0.0, ZV + 0.035 + 0.03)), "copper"))
    board_x = XV + FACE_T + BORE_D + 0.015 - 0.006
    parts.append(("opening", m.box((0.0012, 0.20, 0.17), (board_x - 0.0006, 0.0, ZV + 0.02)), "cavity"))
    cols += [m.col_box("stud_c", (0.04, 0.09, 0.40), (XV + 0.13, 0.0, 0.20))]
    return parts, cols


def _cartridge(at=(0.0, 0.0, 0.0)):
    """Origin (at `at`) on the axis at the ears' front faces; body along +x into the bore, stem along -x."""
    a = np.asarray(at, float)
    parts, cols = [], []
    for nm, r, x0, x1, look in (("body", C_R, 0.0, C_L - STEM_L, "white"), ("stem", STEM_R, -STEM_L, 0.0, "brass")):
        c = a + np.array([(x0 + x1) / 2, 0.0, 0.0])
        mesh = m.cylinder(r, x1 - x0, tuple(c), axis="x") if nm == "body" else m.box((x1 - x0, 2 * r, 2 * r), tuple(c))
        parts.append((nm, mesh, look))
        if nm == "stem":
            cols.append(m.col_box(nm + "_c", (x1 - x0, 2 * r, 2 * r), tuple(c)))
        else:  # round: a square body's corners would stand 17 mm off the axis
            cols.append(m.col_hull(nm + "_c", m.cylinder(r * 0.99, x1 - x0, tuple(c), axis="x", segments=24)))
    for side, sg in (("l", 1), ("r", -1)):
        c = a + np.array([EAR[0] / 2, sg * EAR_Y, 0.0])
        parts.append((f"ear_{side}", m.box((EAR[0], EAR[2], EAR[1]), tuple(c)), "white"))
        cols.append(m.col_box(f"ear_{side}_c", (EAR[0], EAR[2], EAR[1]), tuple(c)))
    return parts, cols


def _leg(y):
    """A leg, its last 3 mm tapered to 1 mm thick: the lead-in that lets it find its slot."""
    t, L, w = CLIP_T, LEG_L, LEG_W
    loop = [(-t / 2, 0.0), (t / 2, 0.0), (t / 2, -L + 0.003), (0.0005, -L), (-0.0005, -L), (-t / 2, -L + 0.003)]
    n = len(loop)
    v = np.array([(x, y + yy, z) for yy in (-w / 2, w / 2) for x, z in loop])
    f = [(0, i + 1, i) for i in range(1, n - 1)] + [(n, n + i, n + i + 1) for i in range(1, n - 1)]
    f += [tt for i in range(n) for tt in ((i, n + (i + 1) % n, (i + 1) % n), (i, n + i, n + (i + 1) % n))]
    return m.Mesh(v, np.array(f))


def _clip():
    """Origin at the crossbar's middle; legs hang down at y +-LEG_Y."""
    span = 2 * LEG_Y + LEG_W
    items = [_box("bar", (CLIP_T, span, 0.004), (0.0, 0.0, 0.0), "steel")]
    for side, sg in (("l", 1), ("r", -1)):
        leg = _leg(sg * LEG_Y)
        items.append(((f"leg_{side}", leg, "steel"), m.col_hull(f"leg_{side}_c", leg)))
    return [p for p, _ in items], [c for _, c in items]


_vp, _vc = _valve()
PROUD = 0.015  # pre-seated: in the bore, resting on its O-rings 15 mm proud, ears level with their notches, as a plumber sets it before pushing
SLIDE_K, ROLL_K, ROLL_HOLD = 3000.0, 20.0, 0.10  # the O-rings' drives: stiffness before they slip (N/m, N m/rad), and the turning friction (N m)
_at = np.array([XV - PROUD, 0.0, ZV])  # the cartridge's origin at the start, in the valve's frame
_cp, _cc = _cartridge(_at)
_slide = m.slider("slide", "X", 0.0, PROUD + 0.0003, [], [], at=tuple(_at), mass=0.02, spring=SLIDE_K, damping=2.0, effort=DRAG_N, friction=0.0, inertia=(1e-6, 1e-6, 1e-6))
_roll = m.Link("roll", "hinge", "X", tuple(_at), -30.0, 30.0, tuple(_cp), tuple(_cc), mass=C_MASS, spring=ROLL_K, damping=0.02, effort=ROLL_HOLD, friction=0.0, parent="slide",
               com=tuple(_at + np.array([C_L / 2 - STEM_L, 0.0, 0.0])), gravity=True, inertia=(1.2e-5, 1.5e-4, 1.5e-4))
VALVE = m.mechanism("valve", (0.0, VY, TOP), [_slide, _roll], LOOKS, frame_parts=_vp, frame_colliders=_vc, friction=(0.3, 0.25), solver_iters=48, self_collide=True)
_kp, _kc = _clip()
CLIP_START = (0.20, 0.10, TOP + 0.010 + LEG_L + 0.001)  # in its holder, in front of the left arm
_hold = [("base", (0.03, 0.06, 0.01), (0.0, 0.0, 0.005))] + [(f"plate{i}", (0.004, 0.05, 0.03), (sg * (CLIP_T / 2 + 0.0006 + 0.002), 0.0, 0.025)) for i, sg in enumerate((-1, 1))]
HOLDER = m.fixed("holder", (CLIP_START[0], CLIP_START[1], TOP), [(n, m.box(sz, c), "black") for n, sz, c in _hold],
                 [m.col_box(n + "_c", sz, c) for n, sz, c in _hold], LOOKS)  # a slot that keeps the clip standing
CLIP = m.free("clip", CLIP_START, _kp, _kc, LOOKS, mass=CLIP_MASS, friction=(0.3, 0.25), torsional_patch=None, solver_iters=int(os.environ.get("CLIP_ITERS", "96")))
OBJECTS = [VALVE, HOLDER, CLIP]


# -- the O-rings: declared code ---------------------------------------------------------------------------------------------
def depth(st) -> float:
    """How far the ears' front faces are inside the valve face (0 at the face, positive in), from the valve's state."""
    return float(st.joint("slide") - PROUD)


def cart_pos(st) -> np.ndarray:
    """The cartridge's origin (the ears' front faces on its axis), in the station's frame."""
    return st.pos + np.array([XV - PROUD + st.joint("slide"), 0.0, ZV])


class Orings:
    """The O-rings as the slide's and the turn's drives. Dry friction: a drive's rest stays put while its spring holds (under DRAG_N, or ROLL_HOLD), and
    follows the joint at the spring's full stretch when it slips. Within CREEP of home, their spring-back: the slide's rest goes to CREEP proud, with 4 N."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.rest = 0.0
        self.roll_rest = 0.0
        self.mode = None

    def step(self, env):
        st = m.state(env, "valve")
        q, rq = st.joint("slide"), st.joint("roll")
        asset, j = m._joint(env, "valve", "slide")
        _, jr = m._joint(env, "valve", "roll")
        back = q - PROUD > -CREEP - 0.0002
        if back:
            if self.mode != "back":
                asset.write_joint_effort_limit_to_sim(m._tensor(asset, 4.0), joint_ids=[j])
                self.mode = "back"
            asset.set_joint_position_target(m._tensor(asset, PROUD - CREEP), joint_ids=[j])
            self.rest = q
        else:
            if self.mode != "drag":
                asset.write_joint_effort_limit_to_sim(m._tensor(asset, DRAG_N), joint_ids=[j])
                self.mode = "drag"
            slip = DRAG_N / SLIDE_K
            if abs(q - self.rest) > slip:
                self.rest = q - math.copysign(slip, q - self.rest)
            asset.set_joint_position_target(m._tensor(asset, self.rest), joint_ids=[j])
        slip_r = ROLL_HOLD / ROLL_K
        if abs(rq - self.roll_rest) > slip_r:
            self.roll_rest = rq - math.copysign(slip_r, rq - self.roll_rest)
        asset.set_joint_position_target(m._tensor(asset, self.roll_rest), joint_ids=[jr])


ORINGS = Orings()
m.on_step(ORINGS.step)
m.on_reset(ORINGS.reset)


def _trace(env) -> None:
    """Debug (VALVE_TRACE): the cartridge's depth, roll and pitch and the right fingertips, every 5 steps of the first 12 s."""
    if not os.environ.get("VALVE_TRACE"):
        return
    _trace.n = getattr(_trace, "n", 0) + 1
    if _trace.n % 5 or _trace.n > 240:
        return
    st = m.state(env, "valve")
    tips = m.fingertips(env)[12:]
    print(f"[valve] trace {_trace.n}: depth {1000 * depth(st):+6.1f} mm, roll {st.deg('roll'):+5.1f}; right tips x {tips[:, 0].max():.4f} "
          f"y {tips[:, 1].min():+.4f}..{tips[:, 1].max():+.4f} z {tips[:, 2].min() - TOP:.4f}..{tips[:, 2].max() - TOP:.4f}", flush=True)


m.on_step(_trace)


LEG_TIPS = np.array([[0.0, sg * LEG_Y, -LEG_L] for sg in (1.0, -1.0)])  # the legs' tips in the clip's frame: left, right


def legs(k) -> dict:
    """Where the clip's legs are against their slots: each tip inside its own slot's outline (across the slot within 0.8 mm, along the face within 2 mm
    of the slot's line), and how far it stands over its stop; and the clip's turn off square (its legs' line against the face's)."""
    tips = k.to_world(LEG_TIPS)
    inside, over = [], []
    for tip, sg in zip(tips, (1.0, -1.0)):
        inside.append(bool(SLOT[0] - 0.0008 <= tip[0] <= SLOT[1] + 0.0008 and abs(tip[1] - (VY + sg * LEG_Y)) < 0.002))
        over.append(float(tip[2] - (TOP + LUG_BOT)))
    across = k.rotation()[:, 1]  # the clip's crossbar runs along its y
    skew = float(np.degrees(np.arccos(np.clip(abs(across[1]), -1.0, 1.0))))
    return {"inside": inside, "over_stop": over, "skew": skew}


def state_now(env) -> dict:
    st = m.state(env, "valve")
    k = m.state(env, "clip")
    d = depth(st)
    roll = st.deg("roll")
    pitch = 0.0
    cp = cart_pos(st)
    lg = legs(k)
    entered = all(lg["inside"]) and all(-0.0005 < z < LUG_TOP - LUG_BOT for z in lg["over_stop"])  # both legs in their own slots: under the lugs' tops, over their stops
    clip_down = entered and all(z < 0.0015 for z in lg["over_stop"]) and lg["skew"] < 5.0 and k.tilt_deg() < 5.0  # both on their stops, square
    tips = m.fingertips(env)
    hands = min(float(np.min(np.linalg.norm(tips - pt, axis=1))) for pt in (cp, cp - np.array([STEM_L, 0.0, 0.0])))  # off the ears and the stem's end
    hands = min(hands, float(np.min(np.linalg.norm(tips - k.pos, axis=1))))
    return {"depth": d, "roll": roll, "pitch": pitch, "flush": d > -0.0005 and abs(roll) < 3.0, "home": d > -0.001 and abs(roll) < 3.0, "clip_down": bool(clip_down), "clip_z": float(k.pos[2] - TOP),
            "clip_started": bool(entered), "legs": lg, "hands": hands,
            "dropped": bool(k.pos[2] < TOP + 0.02),
            "speed": float(abs(st.joint_vel.get("slide_joint", 0.0))) if hasattr(st, "joint_vel") else 0.0}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.now = None
        self.hold, self.full = 0, False  # consecutive steps in the full state; any break starts it again
        self.ever_flush, self.ever_started = False, False  # real progress, for the 0.25 tier
        self.dropped = False  # the clip on the table at any moment: a failure for the whole trial

    def step(self, env):
        self.now = state_now(env)
        self.ever_flush = self.ever_flush or self.now["flush"]
        self.ever_started = self.ever_started or self.now["clip_started"]
        self.dropped = self.dropped or self.now["dropped"]
        ready = _full(self.now)
        stem = cart_pos(m.state(env, "valve")) - np.array([STEM_L, 0.0, 0.0])
        m.push(env, "valve", (-PULL_N if ready else 0.0, 0.0, 0.0), at=stem, body="roll")  # the plumber's tug: 20 N outward on the stem through the hold
        self.hold = self.hold + 1 if ready else 0
        self.full = self.hold >= HOLD_STEPS
        self.now["pulled"] = bool(ready)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def _full(s):
    """Pushed flush at some moment (within 0.5 mm), and now the clip down, the cartridge held home within 1 mm (under the scorer's pull a clip's play lets it
    come out a little), still, and the hands clear."""
    return TRIAL.ever_flush and not TRIAL.dropped and s["home"] and s["clip_down"] and s["hands"] > 0.03 and s["speed"] < 0.01


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    s = TRIAL.now
    if s is None or TRIAL.dropped:
        return 0.0
    if TRIAL.ever_flush and s["home"] and s["clip_down"]:
        return 1.0 if TRIAL.full else 0.5  # the full state not yet held 1 s scores 0.5
    if s["flush"] and s["clip_started"]:
        return 0.5
    return 0.25 if (TRIAL.ever_flush or TRIAL.ever_started) else 0.0  # progress: the cartridge pushed flush at some moment, or the clip started in its slots


def _report(ctx, env, label=""):
    s = state_now(env)
    k = m.state(env, "clip")
    print(f"[valve] t {ctx.t:4.1f} {label}cartridge {1000 * s['depth']:+6.1f} mm from home, roll {s['roll']:+5.1f} deg, flush {s['flush']}; clip at x {1000 * (k.pos[0] - (SLOT[0] + SLOT[1]) / 2):+.1f} "
          f"y {1000 * (k.pos[1] - VY):+.1f} mm from the slots, tilt {k.tilt_deg():.1f}, bar {1000 * s['clip_z']:.0f} mm up, "
          f"legs in their slots {s['legs']['inside']}, over their stops {np.round(1000 * np.array(s['legs']['over_stop']), 1)} mm, skew {s['legs']['skew']:.1f}; "
          f"down {s['clip_down']}; hands {1000 * s['hands']:.0f} mm off; speed {s['speed']:.4f}; pulled {TRIAL.now.get('pulled') if TRIAL.now else None}, held {TRIAL.hold}; score {score(None):.2f}", flush=True)


def demo_script(ctx):
    """The right hand, fingers closed and level, pushes the cartridge's stem by its end face along the bore to flush against the O-rings and keeps pressing;
    the left hand takes the clip from its holder by its crossbar, sets its legs in the slots, lets go, and drives the crossbar down with closed fingertips.
    Then both let go, the right hand last."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    pad = 0.013
    for _ in range(10):
        yield
    _report(ctx, env, "start: ")
    gp = d.level(0)  # fingers closed, pointing +x along the bore: the fingertips push the stem's end face

    def stem_end():
        return cart_pos(m.state(env, "valve")) + np.array([-STEM_L, 0.0, 0.0])

    e = stem_end()
    if ctx.mode == "none":  # cheat check: the arms do nothing
        for _ in range(200):
            yield
        _report(ctx, env, "idle: ")
        return
    if ctx.mode in ("proud", "aside"):  # cheat checks: the cartridge left on its O-rings, 15 mm proud; proud: the clip fitted, and it meets its ears;
        yield from r.wait(0.5)  # aside: the clip lowered into the lugs' gap 20 mm beside its slots and let go
    else:
        yield from r.move(e - np.array([pad + 0.03, 0.0, 0.0]), grip=gp, finger=0.0, t=2.5, label="behind the stem, fingers closed")
        yield from r.move(e - np.array([pad + 0.002, 0.0, 0.0]), t=1.0, label="fingertips at the stem's end face")
        home = XV - STEM_L  # the stem's end when the ears sit home
        yield from r.servo(lambda tt: np.array([min(e[0] + 0.006 * tt, home + 0.0003), VY, TOP + ZV]), tol=0.0003, timeout=5.0, gain=0.3,
                           read=stem_end, label="push the cartridge in against the O-rings to flush")  # the hand kept centred on the stem's end, not sliding across it
        if os.environ.get("VALVE_DEBUG"):
            _report(ctx, env, "servoed: ")
        yield from r.move(r.at() + np.array([0.0015, 0.0, 0.0]), t=0.4, label="keep it pressed flush")
        _report(ctx, env, "pushed flush: ")
    if ctx.mode == "letgo":  # one hand at a time: pushed flush, then let go before the clip comes; the O-rings push it back out
        yield from r.move(r.at() + np.array([-0.04, 0.0, 0.0]), t=1.0, label="let go of the cartridge")
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="up and away")
        _report(ctx, env, "let go before the clip: ")
    bar = m.state(env, "clip").pos
    gc = d.down(-90)  # fingers closing along x, across the clip's 2 mm
    yield from l.move(bar + np.array([0.0, 0.0, 0.06]), grip=gc, finger=FINGER_OPEN * 0.3, t=2.0, label="over the clip")
    yield from l.move(bar + np.array([0.0, 0.0, pad - 0.002]), t=1.0, label="round the crossbar")
    yield from l.close(CLIP_T, 10.0, t=0.6, label="pinch the crossbar")
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.07]), t=1.0, label="lift the clip")
    slot_x = (SLOT[0] + SLOT[1]) / 2
    over = np.array([slot_x, VY, TOP + LUG_TOP + LEG_L + 0.005 + pad - 0.002])
    yield from l.move(over, t=2.5, label="over the slots")
    for _ in range(2):  # square the clip in the pinch: turn the wrist by the clip's measured tilt, so its legs hang plumb over the slots
        Rk = m.state(env, "clip").rotation()
        yield from l.move(l.at(), grip=d._as_grip(Rk.T @ l.grip.rot), t=0.6, label="square the clip up")
    if os.environ.get("VALVE_DEBUG"):
        _report(ctx, env, "clip squared: ")
    clip_at = lambda: m.state(env, "clip").pos  # noqa: E731
    # the clip hangs about 0.7 degrees off upright in the pinch, which puts its legs' tips 0.6 mm towards the face: aim them at the slots' middle
    tip_at = lambda: m.state(env, "clip").to_world(np.array([[0.0, 0.0, -LEG_L]]))[0]  # noqa: E731
    if ctx.mode == "aside":
        yield from l.servo(np.array([slot_x, VY + 0.02, TOP + ZV - NOTCH_Z - 0.005]), tol=0.001, timeout=4.0, read=tip_at, label="the clip lowered beside its slots")
        yield from l.open(t=0.4)
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="up and away")
        for _ in range(40):
            yield
        _report(ctx, env, "aside end: ")
        return
    yield from l.servo(np.array([slot_x, VY, tip_at()[2]]), tol=0.0001, timeout=2.0, read=tip_at, label="the clip's leg tips over the slots' middle")
    for k in range(10):  # still pinched, so it cannot tip, until its leg tips are 5 mm into the lower slots, past the ears' window (pushed from inside the window, a leg catches the lower lip)
        if tip_at()[2] < TOP + ZV - NOTCH_Z - 0.005:
            break
        yield from l.move(l.at() - np.array([0.0, 0.0, 0.006]), t=0.5, label="its legs down the slots, past the ears")
    _report(ctx, env, "clip in the slots: ")
    yield from l.open(t=0.4)
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.015]), finger=0.0, t=0.6, label="up off the crossbar, fingers closed")
    down = TOP + LUG_BOT + LEG_L + 0.002 + pad  # the fingertips on the crossbar's top with the legs on their stops
    for z in np.linspace(l.at()[2] - 0.015, down - 0.001, 8):
        yield from l.move(np.array([slot_x, VY, z]), t=0.4, label="drive the clip down across the ears")
    _report(ctx, env, "clip down: ")
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="up off the clip")
    yield from r.move(r.at() + np.array([-0.04, 0.0, 0.0]), t=1.0, label="let the cartridge go")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="up and away")
    for k in range(80):  # the scorer pulls 20 N on the stem through its 1 s hold
        yield
        if k % 10 == 9:
            _report(ctx, env, "let go: ")
