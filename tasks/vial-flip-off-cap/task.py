"""Flip the plastic cap off a new 10 mL injection vial: one gripper holds the vial upright on the table, the other pushes the snap-on button up at
its edge until the seal breaks and the button flips off, clear of the bared rubber stopper.

Scene (station frame, metres; x forward from the midpoint between the arm bases, y to the robot's left, z up from the bench top).
A Pfizer (Hospira) 0.9% Sodium Chloride Injection, USP, 10 mL single-dose plastic Fliptop vial (NDC 0409-4888-10) stands upright on the bench.
The vial is one free rigid body (`SIZES`); its light green button is a second free rigid body, joined to the vial's aluminium crimp by a joint
locked in every direction but a tip about the button's far rim (the rim away from the pushing hand), a spring of `TIP_K` N m/rad, limited to
lifting the near edge (0 to 15 degrees; the button still collides with the crimp, so a push down on it meets the crimp). The seal breaks only when
the near edge lifts and the spring's moment reaches `BREAK_T` N m (a push of `BREAK_F` N at the near edge, `2 * BTN_R` from the pivot): the scorer then takes the
joint off the live stage and the button is a free body. The joint is made a few steps after each reset, once the parts have settled.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,onearm,pull,knock,pressdown,sidepush,hover
Final:  python demokit.py task.py OUT
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

SCENE = "station"
INSTRUCTION = ("Hold the vial upright in one gripper; push its flip-off cap up at the edge with the other until it snaps clear, without touching the stopper.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HOLD_STEPS = 60  # 3 s at 20 Hz

LOOKS = {
    "vial": m.Look(m.rgb(0.93, 0.94, 0.93), roughness=0.35),
    "label": m.Look(m.rgb(0.97, 0.97, 0.95), roughness=0.6),
    "alu": m.Look(m.rgb(0.78, 0.79, 0.81), metallic=0.85, roughness=0.3),
    "button": m.Look(m.rgb(0.80, 0.88, 0.55), roughness=0.45),
    "stopper": m.Look(m.rgb(0.42, 0.42, 0.44), roughness=0.8),
}

# -- the vial: Hospira NDC 0409-4888-10, 10 mL plastic Fliptop, 20 mm seal ------------------------------------------------------------------
BODY_R, BODY_H = 0.0120, 0.0400  # body 24 mm across, 40 mm tall (photo-scaled from the 21 mm button)
SHOULDER_H = 0.0060  # shoulder up to the neck
NECK_R, NECK_H = 0.0070, 0.0025  # neck 14 mm across
CRIMP_R, CRIMP_H = 0.0100, 0.0050  # aluminium crimp, 20 mm seal
BTN_R, BTN_H = 0.0105, 0.0045  # button 21 mm across, 4.5 mm tall: it stands 0.5 mm proud of the crimp all round (photo)
HOLE_R = 0.0040  # the crimp ring's opening over the stopper, 8 mm across (photo)
STOP_DEPTH = 0.0015  # the stopper face this far below the crimp's top
VIAL_M = float(os.environ.get("VF_VM", "0.015"))  # 10 mL saline plus the polyolefin vial (estimate)
BTN_M = float(os.environ.get("VF_BM", "0.002"))  # the polypropylene button (estimate)
CRIMP_Z0 = BODY_H + SHOULDER_H + NECK_H  # 48.5 mm
CRIMP_TOP = CRIMP_Z0 + CRIMP_H  # 53.5 mm: the button's underside
STOP_Z = CRIMP_TOP - STOP_DEPTH
CURL_H, CURL_IN = float(os.environ.get("VF_CH", "0.0015")), float(os.environ.get("VF_CI", "0.0012"))  # the crimp's rolled top edge: it curls in 1.2 mm over its top 1.5 mm, which leaves the pocket under the button's lip
CRIMP_PROFILE = [(0.0, CRIMP_Z0), (CRIMP_R, CRIMP_Z0), (CRIMP_R, CRIMP_TOP - CURL_H), (CRIMP_R - 0.35 * CURL_IN, CRIMP_TOP - 0.45 * CURL_H),
                 (CRIMP_R - CURL_IN, CRIMP_TOP), (0.0, CRIMP_TOP)]

# The break: West Pharmaceutical's flip-off removal force window for a 20 mm seal, 6 N (YBB 00372003-2015 minimum) to 35 N (ISO 8362-6 maximum).
BREAK_F = float(os.environ.get("VF_BF", "12.0"))  # N at the near edge (inside the window; no published single value)
BREAK_T = BREAK_F * 2 * BTN_R  # N m about the far rim
TIP_K = float(os.environ.get("VF_K", "4.0"))  # N m/rad: the snap's stiffness before it lets go (estimate); lets go at about 3.6 degrees
TIP_C = 0.0005
JOINT_AT_STEP = 4

X0, Y0 = float(os.environ.get("VF_X", "0.40")), float(os.environ.get("VF_Y", "0.0"))

SIZES = [  # (what, scene mm, real mm, source)
    ("seal / crimp diameter", 2 * CRIMP_R * 1000, 20.0, "20 mm seal (West page)"),
    ("button diameter", 2 * BTN_R * 1000, 21.0, "photo, scaled to the 20 mm crimp"),
    ("button height", BTN_H * 1000, 4.5, "photo"),
    ("body diameter", 2 * BODY_R * 1000, 24.0, "photo"),
    ("height to button top", (CRIMP_TOP + BTN_H) * 1000, 58.0, "photo"),
    ("stopper opening", 2 * HOLE_R * 1000, 8.0, "photo"),
]


def size_check() -> list[str]:
    return [f"{w:24s} scene {v:6.2f}  real {r:6.2f}  ({s})" for w, v, r, s in SIZES]


def _vial() -> m.Part:
    prof = [(0.0, 0.0), (BODY_R - 0.001, 0.0), (BODY_R, 0.001), (BODY_R, BODY_H), (NECK_R + 0.002, BODY_H + SHOULDER_H * 0.8),
            (NECK_R, BODY_H + SHOULDER_H), (NECK_R, CRIMP_Z0), (0.0, CRIMP_Z0)]
    shoulder = m.lathe([(0.0, BODY_H), (BODY_R, BODY_H), (NECK_R + 0.002, BODY_H + SHOULDER_H * 0.8), (NECK_R, BODY_H + SHOULDER_H), (NECK_R, CRIMP_Z0), (0.0, CRIMP_Z0)])
    parts = [
        ("body", m.lathe(prof), "vial"),
        ("label", m.cylinder(BODY_R + 0.0003, 0.022, (0.0, 0.0, 0.019)), "label"),
        ("crimp", m.lathe(CRIMP_PROFILE), "alu"),
        ("stopper", m.cylinder(HOLE_R, 0.0003, (0.0, 0.0, STOP_Z + 0.00015)), "stopper"),
    ]
    cols = [m.col_cylinder("c_body", BODY_R, BODY_H, (0.0, 0.0, BODY_H / 2)), m.col_hull("c_shoulder", shoulder),
            m.col_hull("c_crimp", m.lathe(CRIMP_PROFILE))]
    return m.free("vial", (X0, Y0, TOP), parts, cols, LOOKS, mass=VIAL_M, friction=(0.9, 0.8))


def _button() -> m.Part:
    """Origin at the underside's centre. A shallow cup: the top face, a skirt, and the 0.5 mm lip that stands proud of the crimp."""
    parts = [("cap", m.lathe([(0.0, 0.0), (BTN_R, 0.0), (BTN_R, BTN_H - 0.0008), (BTN_R - 0.0008, BTN_H), (0.0, BTN_H)]), "button")]
    cols = [m.col_cylinder("c_cap", BTN_R, BTN_H, (0.0, 0.0, BTN_H / 2))]
    return m.free("button", (X0, Y0, TOP + CRIMP_TOP), parts, cols, LOOKS, mass=BTN_M, friction=(0.9, 0.8))


OBJECTS = [_vial(), _button()]
PIVOT_X = BTN_R  # the far rim, in the vial's frame: the pushing hand comes from -x (the robot's side)


# -- the seal: a sprung joint made on the live stage, taken off when its moment reaches the break ---------------------------------------------


def _joint_path() -> str:
    return f"{m.ENV_PATH}/button/seal"


def drop_joint() -> None:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    p = stage.GetPrimAtPath(_joint_path())
    if p.IsValid():
        stage.RemovePrim(p.GetPath())


def seat_button(env) -> None:
    """Put the button back square on the crimp where a seeded reset's jitter left the vial (one part, as moulded and crimped)."""
    import torch

    base = env.unwrapped
    v, b = base.scene["vial"], base.scene["button"]
    pv = v.data.root_link_pose_w[0].clone()
    R = m.quat_matrix(m._np(pv[3:7]))
    want = pv.clone()
    want[:3] = pv[:3] + torch.tensor(R @ np.array([0.0, 0.0, CRIMP_TOP]), dtype=pv.dtype, device=pv.device)
    b.write_root_pose_to_sim(want.unsqueeze(0))
    b.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=pv.dtype, device=pv.device))


def make_joint() -> None:
    import omni.usd
    from pxr import Gf, Sdf, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    j = UsdPhysics.Joint.Define(stage, Sdf.Path(_joint_path()))
    j.CreateBody0Rel().SetTargets([Sdf.Path(f"{m.ENV_PATH}/vial")])
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"{m.ENV_PATH}/button")])
    j.CreateLocalPos0Attr(Gf.Vec3f(PIVOT_X, 0.0, CRIMP_TOP))
    j.CreateLocalPos1Attr(Gf.Vec3f(PIVOT_X, 0.0, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateCollisionEnabledAttr().Set(True)  # the button still rests on the crimp: a push down meets it
    prim = j.GetPrim()
    lim = UsdPhysics.LimitAPI.Apply(prim, "rotY")  # degrees: the near edge only lifts
    lim.CreateLowAttr(-0.5)
    lim.CreateHighAttr(15.0)
    for ax in ("transX", "transY", "transZ", "rotX", "rotZ"):
        lim = UsdPhysics.LimitAPI.Apply(prim, ax)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)
    drv = UsdPhysics.DriveAPI.Apply(prim, "rotY")  # per degree in USD
    drv.CreateTypeAttr("force")
    drv.CreateTargetPositionAttr(0.0)
    drv.CreateStiffnessAttr(float(TIP_K * math.pi / 180.0))
    drv.CreateDampingAttr(float(TIP_C * math.pi / 180.0))


NEAR_EDGE = np.array([[-BTN_R, 0.0, 0.0]])  # the button's near underside edge, in the button's frame


def tip_deg(vial, btn) -> float:
    """The near edge's lift about the far rim, degrees: positive when the near edge rises off the crimp, negative when pressed down."""
    w = btn.to_world(NEAR_EDGE) if hasattr(btn, "to_world") else btn.pos + NEAR_EDGE @ btn.rotation().T
    z = float(vial.to_local(w)[0, 2]) - CRIMP_TOP
    return math.degrees(math.asin(float(np.clip(z / (2 * BTN_R), -1.0, 1.0))))


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------

HOLD = gk.Grip(half=BODY_R, axis="z", lo=(-1, -1, 0.003), hi=(1, 1, BODY_H))  # jaws closed on the body, below the shoulder
_th = np.linspace(0, 2 * np.pi, 16, endpoint=False)
BTN_PTS = np.concatenate([np.c_[r * np.cos(_th), r * np.sin(_th), np.full(16, z)] for r in (0.0035, 0.007, BTN_R) for z in (0.0, BTN_H)])
OFF_MM, CLEAR_MM = 15.0, 30.0
STILL_MM = 1.0  # fingertip movement a control step that counts as still (20 mm/s)


FINGER_MARGIN = 0.004  # fingertip points stand a few millimetres off the fingers' real surface (they are sparse): a wider zone for them


def _in_hole(local, margin: float = 0.0005) -> np.ndarray:
    """Points (vial frame) down on the stopper: within the 8 mm opening (plus `margin` round it) and below the crimp's top + `margin`."""
    p = np.atleast_2d(local)
    return (np.hypot(p[:, 0], p[:, 1]) < HOLE_R + max(0.0, margin - 0.0005)) & (p[:, 2] < CRIMP_TOP + margin)


class Trial:
    def __init__(self):
        self.env = None
        self.reset()

    def reset(self, env=None) -> None:
        self.n, self.ok_n, self.score, self.full = 0, 0, 0.0, False
        self.jointed = False
        self.broken = gk.Latch("seal broken")
        self.lifted = gk.Latch("button lifted off the opening")
        self.touch = gk.Latch("stopper touched")
        self.tipped = gk.Latch("vial tipped over or off the table")
        self.unheld = gk.Latch("seal broken with the vial unheld")
        self.peak = 0.0
        self.prev_tips = None
        self.rep = {}
        if env is not None:
            drop_joint()
            self.env = env

    def step(self, env) -> None:
        self.n += 1
        if self.n == 1:
            seat_button(env)
        if not self.jointed and self.n >= JOINT_AT_STEP:
            make_joint()
            self.jointed = True
        vial, btn = m.state(env, "vial"), m.state(env, "button")
        tips = m.fingertips(env)
        ang = tip_deg(vial, btn)
        held = gk.held(tips, vial, HOLD, resting=False)  # the vial stands held in place on the table: it never rests on a stand
        if self.jointed and not self.broken:
            mo = TIP_K * math.radians(max(ang, 0.0))  # only a lifting near edge loads the snap
            self.peak = max(self.peak, mo)
            if self.broken.trip(mo >= BREAK_T):
                drop_joint()
                self.unheld.trip(not held)
                print(f"[vial] seal broke at t {self.n * d.DT:.2f} s: tip {ang:.2f} deg, {mo:.3f} N m ({mo / (2 * BTN_R):.1f} N at the edge)", flush=True)
        tilt = math.degrees(math.acos(float(np.clip(vial.rotation()[2, 2], -1, 1))))
        self.tipped.trip(tilt > 30.0 or vial.pos[2] < TOP - 0.01)
        bpts = vial.to_local(btn.to_world(BTN_PTS)) if hasattr(btn, "to_world") else vial.to_local(btn.pos + BTN_PTS @ btn.rotation().T)
        tl = vial.to_local(tips)
        if self.broken:
            over = np.hypot(bpts[:, 0], bpts[:, 1]) < HOLE_R
            self.lifted.trip(not over.any() or bpts[over, 2].min() > CRIMP_TOP + 0.002)
            if self.lifted and not self.touch:
                if self.touch.trip(bool(_in_hole(bpts).any() or _in_hole(tl, FINGER_MARGIN).any())):
                    print(f"[vial] stopper touched at t {self.n * d.DT:.2f} s (button {bool(_in_hole(bpts).any())}, fingers {bool(_in_hole(tl, FINGER_MARGIN).any())})", flush=True)
        top = vial.to_world(np.array([[0.0, 0.0, CRIMP_TOP]]))[0] if hasattr(vial, "to_world") else vial.pos + vial.rotation() @ np.array([0.0, 0.0, CRIMP_TOP])
        away = float(np.linalg.norm(btn.pos - top)) * 1000
        moved = 0.0 if self.prev_tips is None else float(np.max(np.linalg.norm(tips - self.prev_tips, axis=1))) * 1000
        self.prev_tips = tips.copy()
        upright = tilt < 10.0
        end_ok = self.broken and held and away > CLEAR_MM and upright and moved < STILL_MM
        self.ok_n = self.ok_n + 1 if end_ok else 0
        if self.tipped or self.touch or self.unheld:
            s = 0.0
        elif self.broken and away > OFF_MM:
            s = 1.0 if self.ok_n >= HOLD_STEPS else 0.75
        elif self.broken:
            s = 0.5
        elif held:
            s = 0.25
        else:
            s = 0.0
        self.score = s  # from the current state each step; only the failures and the seal's break (which cannot be undone) latch
        self.full = self.score >= 1.0
        self.rep = {"t": round(self.n * d.DT, 2), "score": self.score, "held": held, "tip_deg": round(ang, 2), "peak_Nm": round(self.peak, 3),
                    "broken": bool(self.broken), "away_mm": round(away, 1), "tilt": round(tilt, 1), "touch": bool(self.touch), "unheld": bool(self.unheld), "ok_n": self.ok_n}
        if self.n % 20 == 0:
            print(f"[vial] {self.rep}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Seal broken by a lift at the edge with the vial held, button over 30 mm from the crimp's top, vial still held and upright, the stopper
    not touched by the button or the fingers once the button lifted, both hands still for 3 s."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the scripted demo -------------------------------------------------------------------------------------------------------------------

HOLD_Z = float(os.environ.get("VF_HZ", "0.014"))  # the holding jaws' height on the body
HOLD_TILT = float(os.environ.get("VF_HT", "45"))
PUSH_TILT = float(os.environ.get("VF_PT", "88"))  # the pushing fingers point +x, this far from straight down (over 90: tipped up)
TIP_IN = float(os.environ.get("VF_TIN", "0.013"))  # the TCP this far back (-x) from the fingertips' reported centre
PUSH_DZ = float(os.environ.get("VF_PDZ", "-0.003"))  # the fingertips' reported centre this far below the button's underside at the start
PUSH_MM = float(os.environ.get("VF_PMM", "14"))  # millimetres of rise
PUSH_IN = float(os.environ.get("VF_PIN", "0.0008"))  # m in (+x) per millimetre of rise
FLICK = tuple(float(x) for x in os.environ.get("VF_FL", "0.03,0.05,0.15").split(","))  # m forward, m up, s
ROLL_DEG = float(os.environ.get("VF_RD", "0"))  # degrees the near edge is carried round the far rim
ROLL_STEP = 8.0


def _vial_top(ctx) -> np.ndarray:
    return ctx.pos("vial") + np.array([0.0, 0.0, CRIMP_TOP])


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        for _ in range(100):
            yield
        return
    hg = d.tilted(HOLD_TILT, 90)
    if mode not in ("onearm", "hover"):
        # the right hand comes from -y and closes across x on the body
        yield from R.pick("vial", HOLD_Z, width=2 * BODY_R, grip=hg, above=0.04, back=0.06, lift=0.0, force=25.0, t=2.0, label="hold")
    if mode == "knock":  # push the vial over with the left hand's fingertips at the button
        g = d.tilted(PUSH_TILT, 0)
        p = _vial_top(ctx) + np.array([-0.05, 0.0, 0.0])
        yield from L.move(p, grip=g, finger=0.0, t=2.0)
        yield from L.move(p + np.array([0.09, 0.0, 0.0]), t=1.0)
        for _ in range(60):
            yield
        return
    if mode == "pressdown":  # press straight down on the button, 8 mm in from its near edge, with 20 N
        g = d.down(0.0)
        p = _vial_top(ctx) + np.array([-BTN_R + 0.008, 0.0, BTN_H + 0.03])
        yield from L.move(p, grip=g, finger=0.0, t=2.5, label="pressdown: above")
        yield from L.servo(p, timeout=1.0)
        yield from L.press((0.0, 0.0, -1.0), force=20.0, max_travel=0.04, settle=1.0, timeout=4.0, label="pressdown")
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.05]), t=1.0)
        for _ in range(60):
            yield
        return
    if mode == "pull":  # straddle the button with open jaws and lift straight up
        g = d.down(0.0)
        yield from L.pick(lambda: _vial_top(ctx), 0.004, width=2 * BTN_R, grip=g, above=0.06, lift=0.03, force=25.0, t=2.0, label="pull")
        for _ in range(80):
            yield
        return
    g = d.tilted(PUSH_TILT, 0)
    top = _vial_top(ctx)
    side = 0.0095 if mode == "sidepush" else 0.0  # sidepush: the same push at the button's side edge (on the hinge axis side), not its near edge
    # fingers shut, pointing +x, their reported centre just outside the crimp wall and just under the button's lip
    start = top + np.array([-(CRIMP_R + TIP_IN + 0.004), side, PUSH_DZ])
    if mode == "hover":  # no hold: the pushing hand goes in under the lip and touches nothing
        yield from L.move(start - np.array([0.03, 0.0, 0.0]), grip=g, finger=0.0, t=2.5, label="hover: approach")
        yield from L.move(start - np.array([0.01, 0.0, 0.0]), t=1.0, label="hover: in")
        for _ in range(60):
            yield
        return
    yield from L.move(start - np.array([0.03, 0.0, 0.0]), grip=g, finger=0.0, t=2.5, label="push: approach")
    yield from L.servo(start - np.array([0.03, 0.0, 0.0]), timeout=1.0)
    yield from L.move(start, t=1.0, label="push: in")
    yield from L.servo(start, timeout=1.0)
    yield from L.press((1.0, 0.0, 0.0), force=3.0, max_travel=0.006, settle=0.3, timeout=2.0, label="push: to the crimp")
    # push up and in under the lip in small position steps: the arm's position hold, not a force cap, carries the push
    for k in range(int(PUSH_MM)):
        yield from L.move(L.at() + np.array([PUSH_IN, 0.0, 0.001]), t=0.15, label="push: up at the edge")
    print(f"[vial] edge push done; broken {bool(TRIAL.broken)}; {TRIAL.rep}", flush=True)
    # a quick flick up and toward the far rim throws the freed button off, away from the bared stopper
    yield from L.move(L.at() + np.array([FLICK[0], 0.0, FLICK[1]]), t=FLICK[2], label="flick")
    # carry the near edge on round the far rim, the grip kept as it is, so the freed button turns over and drops off on the far side
    piv = ctx.pos("vial") + np.array([PIVOT_X, 0.0, CRIMP_TOP])
    v0 = L.at() - piv
    for th in np.radians(np.arange(ROLL_STEP, 0.0, 1.0) if ROLL_DEG <= 0 else np.arange(ROLL_STEP, ROLL_DEG + 1e-6, ROLL_STEP)):
        c, s_ = math.cos(th), math.sin(th)
        v = np.array([v0[0] * c + v0[2] * s_, v0[1], -v0[0] * s_ + v0[2] * c])
        yield from L.move(piv + v, t=0.15, label="roll over")
    yield from L.move(L.at() + np.array([-0.05, 0.0, 0.04]), t=1.2, label="away")
    for _ in range(90):
        yield
