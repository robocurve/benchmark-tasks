"""A hand-crank hose reel stands on the right half of the station table: a grey drum (60 mm across, 140 mm long)
between two red flanges (120 mm across) turns on a horizontal axle running left-right (along y) at x 34 cm, 7.5 cm
above the table, carried by two dark uprights. A dark crank arm on the drum's right end carries a black ball handle
(24 mm) on a pin, 5 cm from the axle and 5 cm right of the robot's midline, so only the right arm can turn it.
A green 10 mm garden hose, 60 cm long, is fixed to the drum core at its right end and leaves the bottom of the drum
toward the robot; it lies on the table in a U (straight 16 cm toward the robot, a 10 cm radius bend to the left, then
back away from the robot) and ends in a black nozzle. Turning the crank winds the hose in under the drum; where each
turn lands across the drum depends on where the hose is held as it goes on. Left alone the hose winds on in a loose open spiral (4 cm between turns); held in one
place it piles turn on turn. The robot must crank the hose on while guiding it so that each turn lies beside the last
in one layer, and let go. A pawl on the axle lets the drum turn freely to wind but holds it against unwinding.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: REEL_CHEAT=nothing | crankonly | fixedguide | hold | handturn | slowrim | tier03 | tier06 | stop39.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the hose below is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Wind the green hose onto the reel neatly, then let go."

GREY, DARK, GREEN, BLACK, RED = ((0.55, 0.57, 0.60), (0.25, 0.26, 0.28), (0.10, 0.55, 0.15),
                                 (0.08, 0.08, 0.08), (0.80, 0.10, 0.08))

# -- reel: drum on an axle along y (m; x forward from the robot, y to its left) -----------------------------------------
AX_X = 0.34
AX_Z = TOP + 0.075
CORE_R = 0.030
DRUM_Y0, DRUM_Y1 = 0.070, 0.210  # drum ends; right flange outside y0, left flange outside y1
DRUM_YC = (DRUM_Y0 + DRUM_Y1) / 2
FL_R, FL_T = 0.060, 0.006
DRUM_DAMP = 1e-3  # axle drag (N m s per degree; about 0.06 N m at one turn per second)
# one-way pawl: free to wind; against unwinding a stiff catch (spring + damper) that slips above PAWL_MAX
PAWL_K, PAWL_C, PAWL_MAX = 0.5, 0.02, 0.15  # N m/rad, N m s/rad, N m (3 N at the handle radius)
DRUM = Cylinder("reel_drum", pos=(AX_X, DRUM_YC, AX_Z), radius=CORE_R, height=DRUM_Y1 - DRUM_Y0, color=GREY,
                mass=0.15)
FLANGE_R = Cylinder("reel_flange_r", pos=(AX_X, DRUM_Y0 - 0.0005 - FL_T / 2, AX_Z), radius=FL_R, height=FL_T,
                    color=RED, mass=0.04)
FLANGE_L = Cylinder("reel_flange_l", pos=(AX_X, DRUM_Y1 + 0.0005 + FL_T / 2, AX_Z), radius=FL_R, height=FL_T,
                    color=RED, mass=0.04)
CRANK_R = 0.050  # handle radius about the axle
CRANK_Y = DRUM_Y0 - 0.001 - FL_T - 0.016  # crank arm plane
ARM_T = 0.006
CRANK_ARM = Box("crank_arm", pos=(AX_X, CRANK_Y, AX_Z), size=(0.014, ARM_T, 2 * CRANK_R + 0.018), color=DARK,
                mass=0.02)  # runs through the axle: handle on one end, counterweight on the other
CRANK_CW = Box("crank_counterweight", pos=(AX_X, CRANK_Y, AX_Z - CRANK_R), size=(0.022, ARM_T, 0.022), color=DARK,
               mass=0.02)  # balances the handle and its stem, so the crank does not swing round when let go
KNOB_Y = -0.050
KNOB_R = 0.012
STEM_Y0, STEM_Y1 = KNOB_Y + KNOB_R - 0.002, CRANK_Y - ARM_T / 2 - 0.0005
CRANK_STEM = Cylinder("crank_stem", pos=(AX_X, (STEM_Y0 + STEM_Y1) / 2, AX_Z + CRANK_R), radius=0.005,
                      height=STEM_Y1 - STEM_Y0, color=DARK, mass=0.01)
KNOB = Sphere("crank_knob", pos=(AX_X, KNOB_Y, AX_Z + CRANK_R), radius=KNOB_R, color=BLACK, mass=0.01)
AXIS_Y = {DRUM.name, FLANGE_R.name, FLANGE_L.name, CRANK_STEM.name}  # cylinders lying along y
UP_W, UP_T = 0.016, 0.006
UP_H = AX_Z - TOP - 0.004
UPRIGHT_L = Box("reel_upright_l", pos=(AX_X, DRUM_Y1 + 0.001 + FL_T + 0.002 + UP_T / 2, TOP + UP_H / 2),
                size=(UP_W, UP_T, UP_H), color=DARK)
UPRIGHT_R = Box("reel_upright_r", pos=(AX_X, DRUM_Y0 - 0.001 - FL_T - 0.002 - UP_T / 2, TOP + UP_H / 2),
                size=(UP_W, UP_T, UP_H), color=DARK)

# -- hose ---------------------------------------------------------------------------------------------------------------
R = 0.005  # 10 mm hose
HOSE_LEN = 0.60
NOZZLE = Plug(size=(0.040, 0.016, 0.016), mass=0.02, color=BLACK)
RC = CORE_R + R / math.cos(math.pi / 8) + 0.0015  # centreline radius of the first wound layer
WELD_Y = DRUM_Y0 + R + 0.003  # where the hose is fixed to the core (near the right flange)
Z_BOT = AX_Z - RC
Z_TABLE = TOP + R + 0.0005


def _layout() -> np.ndarray:
    """Start centreline: off the drum bottom toward the robot (dropping to the table), a U to the left, back out."""
    pts = []
    for x in np.linspace(AX_X, AX_X - 0.16, 40):
        t = np.clip((AX_X - 0.02 - x) / 0.07, 0, 1)
        pts.append((x, WELD_Y, Z_BOT + (Z_TABLE - Z_BOT) * (3 * t * t - 2 * t ** 3)))
    cx, cy, rr = AX_X - 0.16, WELD_Y + 0.10, 0.10
    for a in np.linspace(-math.pi / 2, -3 * math.pi / 2, 60)[1:]:
        pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a), Z_TABLE))
    for x in np.linspace(cx, cx + 0.30, 40)[1:]:
        pts.append((x, cy + rr, Z_TABLE))
    return np.array(pts)


LAYOUT = _layout()
_proto = cable("hose", start=(0.2, 0.2, Z_TABLE), yaw=0.0, length=HOSE_LEN, radius=R)
SP = cableshape.spawn_for(_proto, LAYOUT, lift=0.0)
HOSE = cable("hose", start=SP["start"], yaw=SP["yaw"], length=HOSE_LEN, radius=R, end_plug=NOZZLE, color=GREEN,
             bend_stiffness=0.004, bend_damping=0.0006, cable_mass=0.06)

OBJECTS = [DRUM, FLANGE_R, FLANGE_L, CRANK_ARM, CRANK_CW, CRANK_STEM, KNOB, UPRIGHT_L, UPRIGHT_R, HOSE]
REEL_PARTS = (FLANGE_R, FLANGE_L, CRANK_ARM, CRANK_CW, CRANK_STEM)

TWIN_EXTRA = [
    {"name": "reel_axle", "kind": "joint", "real": "M8 axle in two 608ZZ bearings at the upright tops",
     "note": "drum, flanges and crank turn together about y with light drag and a one-way pawl (free to wind, stiff catch against unwinding, slips above 0.15 N m)"},
    {"name": "knob_pin", "kind": "joint", "real": "ball handle free to turn on an M5 shoulder bolt",
     "note": "handle turns freely on the crank"},
    {"name": "hose_clamp", "kind": "joint", "real": "hose end clamped in a slot in the drum core",
     "note": "inner hose end fixed to the core at the bottom, 13 mm from the right flange"},
]


# -- reel joints: axle world->drum, flanges/crank welded to the drum, knob on a pin, hose end clamped to the core -------
def _install_reel() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_reel_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != HOSE.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        for n in AXIS_Y:  # lie these cylinders along y (geometry axis; the body frames stay world-aligned)
            for p in Usd.PrimRange(stage.GetPrimAtPath(f"{root}/{n}")):
                if p.IsA(UsdGeom.Cylinder):
                    UsdGeom.Cylinder(p).GetAxisAttr().Set("Y")
        hub = f"{root}/{DRUM.name}"
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/reel_axle")
        h.CreateBody1Rel().SetTargets([Sdf.Path(hub)])
        h.CreateAxisAttr("Y")
        h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + DRUM.pos[0]), float(w[1] + DRUM.pos[1]), float(w[2] + DRUM.pos[2])))
        h.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        h.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        h.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        h.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(h.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(DRUM_DAMP)

        def joint(name, b0, b1, off, revolute=False, rot0=(1.0, 0.0, 0.0, 0.0)):
            j = (UsdPhysics.RevoluteJoint if revolute else UsdPhysics.FixedJoint).Define(stage, f"{prim_path}/{name}")
            if revolute:
                j.CreateAxisAttr("Y")
            j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, off)))
            j.CreateLocalRot0Attr(Gf.Quatf(*map(float, rot0)))
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateExcludeFromArticulationAttr(True)

        for part in REEL_PARTS:
            joint(f"{part.name}_weld", hub, f"{root}/{part.name}", np.asarray(part.pos) - np.asarray(DRUM.pos))
        joint("knob_pin", f"{root}/{CRANK_STEM.name}", f"{root}/{KNOB.name}",
              np.asarray(KNOB.pos) - np.asarray(CRANK_STEM.pos), revolute=True)
        x, y, z, qw = (float(v) for v in SP["seg0_quat_xyzw"])
        joint("hose_clamp", hub, f"{prim_path}/seg_0", np.asarray(SP["seg0_pos"]) - np.asarray(DRUM.pos),
              rot0=(qw, x, y, z))
        return prim

    spawn._reel_hook = True
    cablekit._spawn_cable = spawn


_install_reel()


def _shape(emb):
    cableshape.preshape(emb, HOSE, LAYOUT, hold="ramp", settle_s=0, lift=0.0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

SET_R = 0.006  # a segment centre within 6 mm of the first-layer radius, between the flanges, lies on the drum


def _coil_set(env) -> None:
    """Hose lying on the drum takes the drum's curve (coil set): every bend joint whose two segments both lie on the
    drum has its rest angle set to its present angle, so the wound hose does not spring the drum backwards when the
    crank is let go. The set stays (as in a real hose) while both segments remain inside the flange rim; hose that
    leaves the reel goes back to a straight rest shape."""
    import torch
    from cablekit import _cable_state

    pts = _cable_state(env, HOSE).points
    if not np.isfinite(pts).all():
        return
    d = np.hypot(pts[:, 0] - AX_X, pts[:, 2] - AX_Z)
    between = (pts[:, 1] > DRUM_Y0 - 0.003) & (pts[:, 1] < DRUM_Y1 + 0.003)
    on = (np.abs(d - RC) < SET_R) & between
    reel = (d < FL_R) & between
    both = on[:-1] & on[1:]  # joint i+1 joins segments i and i+1
    held = _ENV.get("coil_mask")
    keep = (held & reel[:-1] & reel[1:] & ~both) if held is not None else np.zeros_like(both)
    if not both.any() and not keep.any() and not _ENV.get("set_any"):
        return
    _ENV["set_any"] = bool(both.any() or keep.any())
    base = env.unwrapped
    art = base.scene[HOSE.name]
    i1, i2 = cableshape._dof_index(art)
    jp = cableshape._t(art.data.joint_pos)
    q = torch.zeros_like(jp)
    prev = _ENV.get("coil_q")
    m = torch.tensor(both, dtype=torch.bool, device=base.device)
    k = torch.tensor(keep, dtype=torch.bool, device=base.device)
    for idx in (i1, i2):
        cur = jp[0, idx]
        old = prev[0, idx] if prev is not None else torch.zeros_like(cur)
        q[0, idx] = torch.where(m, cur, torch.where(k, old, torch.zeros_like(cur)))
    _ENV["coil_q"] = q.clone()
    _ENV["coil_mask"] = both | keep
    art.set_joint_position_target_index(target=q)


def _pawl_torque(drum, dt) -> None:
    """Called before every physics step. Winding turns the drum about -y. Any turn the other way since the pawl last
    caught is resisted by a stiff spring and damper (a pawl tooth against its ratchet), up to PAWL_MAX, beyond which
    the pawl slips a tooth; winding re-seats it at once."""
    import torch

    w = float(cableshape._t(drum.data.root_ang_vel_w)[0, 1])
    if not math.isfinite(w):
        return
    if not _ENV["knob_held"]:  # drum turn (either way) made while the handle is not held, summed every physics step
        _ENV["free_turn"] += abs(w) * dt
    back = max(0.0, _ENV["pawl_back"] + w * dt)
    tq = PAWL_K * back + PAWL_C * max(w, 0.0) if back > 0 else 0.0
    if tq > PAWL_MAX:
        tq = PAWL_MAX
        back = min(back, PAWL_MAX / PAWL_K)
    _ENV["pawl_back"] = back
    if tq > 0:
        t = torch.zeros((1, 1, 3), dtype=torch.float32, device=drum.device)
        t[0, 0, 1] = -tq
        drum.instantaneous_wrench_composer.set_forces_and_torques_index(torques=t, is_global=True)


def _install_pawl(env) -> None:
    base = env.unwrapped
    drum = base.scene.rigid_objects[DRUM.name]
    if getattr(drum, "_pawl", False):
        return
    orig = drum.write_data_to_sim
    dt = float(base.physics_dt)

    def write_data_to_sim():
        _pawl_torque(drum, dt)
        return orig()

    drum.write_data_to_sim = write_data_to_sim
    drum._pawl = True


# -- env handle, per-episode latch -------------------------------------------------------------------------------------
_ENV = {"env": None, "step": 0, "pawl_back": 0.0, "free_turn": 0.0, "knob_held": False}
_MON = {"violated": "", "last_step": -1, "held_wound": 0.0}


def _reset_episode() -> None:
    _MON.update(violated="", last_step=-1, held_wound=0.0)
    _ENV["step"] = 0
    _ENV["set_any"] = False
    _ENV["coil_q"] = None
    _ENV["coil_mask"] = None
    _ENV["pawl_back"] = 0.0
    _ENV["free_turn"] = 0.0
    _ENV["knob_held"] = False


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_reel_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            _install_pawl(self._env)
            _ENV["step"] += 1
            _coil_set(self._env)
            return base(self, *a, **k)

        _task_step._reel_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()

# -- scoring --------------------------------------------------------------------------------------------------------------
CLOSED_MAX = 0.030  # |finger joint| below this = fingers closed (open is 0.047)
ON_KNOB = 0.025  # a closed right gripper with its TCP within 2.5 cm of the handle centre is turning the crank
FREE_TURN = 0.3  # rad: total drum turn allowed while the handle is not held (settling, release, hose pull)
RELEASE_DIST = 0.03  # at the end every TCP at least 3 cm from the hose, nozzle and handle
ON_DRUM = 0.015  # a hose segment centre within this of the first-layer radius, between the flanges, is wound on
LAYER_TOL = 0.004  # one layer: wound segments within 4 mm of the first-layer radius
SAG = 0.06  # once let go, up to 6 cm of the wound end may sag off the underside of the drum (no hose tension)
PITCH_MIN, PITCH_MAX = 0.008, 0.017  # beside the previous turn: 8-17 mm across the drum from it (hose 10 mm)
TIER_WOUND = (0.27, 0.34, 0.37)  # m of hose on the drum for 0.3 / 0.6 / 1.0


def grippers() -> list[tuple[np.ndarray, float]]:
    """(TCP position in env frame, |finger joint|) for (left, right); [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    out = []
    for k in ("left_robot", "right_robot"):
        art = base.scene[k]
        jp = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0].detach().cpu().numpy()
        out.append((np.asarray(tcp_world(art), dtype=np.float64) - origin,
                    float(abs(jp[list(art.joint_names).index("joint7")]))))
    return out


def _finite(objects) -> bool:
    c = objects[HOSE.name]
    return bool(np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all()
                and np.isfinite(objects[KNOB.name].pos).all())


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _hose_dist(objects, g) -> float:
    c = objects[HOSE.name]
    poly = c.polyline()
    d = min(_seg_dist(g, u, v) for u, v in zip(poly[:-1], poly[1:]))
    p, a = np.asarray(c.end_plug), np.asarray(c.end_dir)
    return min(d, _seg_dist(g, p - a * NOZZLE.size[0] / 2, p + a * NOZZLE.size[0] / 2))


def released(objects) -> bool:
    k = np.asarray(objects[KNOB.name].pos)
    return all(_hose_dist(objects, g) >= RELEASE_DIST and np.linalg.norm(g - k) >= RELEASE_DIST
               for g, _ in grippers())


def on_knob(objects) -> bool:
    g = grippers()
    if len(g) < 2:
        return False
    tcp, f = g[1]
    return f < CLOSED_MAX and float(np.linalg.norm(tcp - np.asarray(objects[KNOB.name].pos))) <= ON_KNOB


def lay(objects) -> dict:
    """The hose on the drum: wound length (contiguous from the clamped end), lift of the wound turns above the first
    layer, and the turn-to-turn spacing across the drum (each wound segment vs the hose one turn earlier)."""
    pts = objects[HOSE.name].points
    d = np.hypot(pts[:, 0] - AX_X, pts[:, 2] - AX_Z)
    on = (d < RC + ON_DRUM) & (pts[:, 1] > DRUM_Y0 - 0.005) & (pts[:, 1] < DRUM_Y1 + 0.005)
    n = int(np.argmin(on)) if not on.all() else len(on)
    out = {"wound": n * HOSE.seg_len, "lift": 0.0, "pitch_min": None, "pitch_max": None, "turns": 0.0}
    beyond = np.arange(len(pts)) >= n + 3  # 6 cm or more past the wound part (past the lead-off from the drum)
    inside = (d < FL_R - 0.005) & (pts[:, 1] > DRUM_Y0) & (pts[:, 1] < DRUM_Y1)
    out["loose"] = int((beyond & inside).sum())  # hose bunched on the drum but not lying on the wound layer
    if n < 2:
        return out
    phi = np.unwrap(np.arctan2(pts[:n, 2] - AX_Z, pts[:n, 0] - AX_X))
    psi = np.abs(phi - phi[0])
    body = psi <= psi.max() - 0.6  # the last ~2 cm is still coming onto the drum
    out["lift"] = float((d[:n][body] - RC).max()) if body.any() else 0.0
    out["turns"] = float(psi.max() / (2 * math.pi))
    y = pts[:n, 1]
    pitches = [y[i] - float(np.interp(psi[i] - 2 * math.pi, psi[:i], y[:i]))
               for i in range(n) if psi[i] >= 2 * math.pi]
    if pitches:
        out["pitch_min"], out["pitch_max"] = float(min(pitches)), float(max(pitches))
    return out


def neat(L: dict) -> bool:
    """One layer, every turn beside the previous one (8-17 mm over, always the same way), and no hose bunched loose
    between the flanges beyond the wound part."""
    if L["lift"] > LAYER_TOL or L["pitch_min"] is None or L.get("loose", 0):
        return False
    lo, hi = L["pitch_min"], L["pitch_max"]
    return (PITCH_MIN <= lo and hi <= PITCH_MAX) or (-PITCH_MAX <= lo and hi <= -PITCH_MIN)


def _track(objects) -> None:
    """Once per control step: latch a NaN state and a drum turned other than by the crank handle."""
    if not _finite(objects):
        _MON["violated"] = _MON["violated"] or "nan"
        return
    step = _ENV["step"]
    if step == _MON["last_step"] or _ENV["env"] is None:
        return
    _MON["last_step"] = step
    _ENV["knob_held"] = on_knob(objects)
    if not released(objects):  # most hose on the drum while a hand was on or near the hose or handle
        _MON["held_wound"] = max(_MON["held_wound"], lay(objects)["wound"])
    if _ENV["free_turn"] > FREE_TURN and not _MON["violated"]:
        _MON["violated"] = f"drum turned {_ENV['free_turn']:.2f} rad without the crank handle by step {step}"


def score(objects) -> float:
    """0 after a drum turn not made with the crank handle, if a gripper is within 3 cm of the hose, nozzle or handle at
    the end, or if the wound hose is not neat (more than 4 mm above the first layer anywhere, or any turn not 8-17 mm
    beside the previous one). Wound = the most hose on the drum while a hand worked it, as long as no more than 6 cm
    of it has since sagged off. Neat: 0.3 for >= 27 cm wound, 0.6 for >= 34 cm, 1.0 for >= 37 cm."""
    _track(objects)
    if _MON["violated"] or not _finite(objects) or not released(objects):
        return 0.0
    L = lay(objects)
    if not neat(L):
        return 0.0
    w = max(L["wound"], min(_MON["held_wound"], L["wound"] + SAG)) + 1e-6
    if w >= TIER_WOUND[2]:
        return 1.0
    if w >= TIER_WOUND[1]:
        return 0.6
    if w >= TIER_WOUND[0]:
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[HOSE.name].resting(max_speed=0.02)


def distance(objects) -> float:
    _track(objects)
    if _MON["violated"] or not _finite(objects):
        return 3.0
    return max(0.0, TIER_WOUND[2] - lay(objects)["wound"])


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    _track(objects)
    ok = _finite(objects)
    L = lay(objects) if ok else {"wound": 0.0, "lift": 0.0, "pitch_min": None}
    return {"wound": round(min(L["wound"] / TIER_WOUND[2], 1.0), 2), "one_layer": int(ok and L["lift"] <= LAYER_TOL),
            "neat": int(ok and neat(L)),
            "crank_only": int(not _MON["violated"]), "released": int(ok and released(objects))}


VARIATION = {"cable:hose": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}

# -- closed-loop scripted demo (cabledemo.py) ---------------------------------------------------------------------------
# Right arm: pinch the ball handle (fingers along x, wrist camera looking left at the drum) and walk it round the axle
# in 10-degree steps from the handle's live position. Left arm, at the same time: straddle the hose on the table 10 cm
# in front of the drum (fingers across the hose, partly closed so it slides through) and move along y in step with the
# drum's turns (10.5 mm per turn), so each turn goes on beside the last. Then both arms let go and lift clear; the
# pawl holds the drum.
RYAW = math.pi / 2
GX = AX_X - 0.10  # guide station: 10 cm in front of the axle
GUIDE_Z = TOP + 0.0145  # fingertips just above the table: the hose lies between the fingers
GUIDE_F = -0.009  # partly closed: an 18 mm slot round the 10 mm hose
PITCH = 0.0105
LEAD = 0.6  # turns the guide runs ahead of the drum
STOP_WOUND = 0.41


def _log(ctx, tag):
    st = ctx.states()
    if not _finite(st):
        print(f"[demo t={ctx.t:.2f}] {tag}: NaN", flush=True)
        return
    L = lay(st)
    print(f"[demo t={ctx.t:.2f}] {tag}: lay={ {k: (round(v, 4) if isinstance(v, float) else v) for k, v in L.items()} } "
          f"neat={neat(L)} nozzle={np.round(st[HOSE.name].end_plug, 3).tolist()} "
          f"free={_ENV['free_turn']:.2f} held={_MON['held_wound']:.2f} viol='{_MON['violated']}' rel={released(st)} score={score(st)}", flush=True)


def _crank(ctx, stop_wound, info, step_deg=10.0, dur=0.12):
    Rt = ctx.right
    last = None
    total = 0.0
    k = 0
    while True:
        st = ctx.states()
        if not _finite(st):
            return
        kp = st[KNOB.name].pos
        a = math.atan2(kp[2] - AX_Z, kp[0] - AX_X)
        if last is not None:
            total += (a - last + math.pi) % (2 * math.pi) - math.pi
        last = a
        info["turns"] = total / (2 * math.pi)
        if lay(st)["wound"] >= stop_wound - 1e-6 or total > 4 * 2 * math.pi:
            return
        t = a + math.radians(step_deg)
        yield from Rt.move((AX_X + CRANK_R * math.cos(t), KNOB_Y, AX_Z + CRANK_R * math.sin(t) + 0.006), yaw=RYAW,
                           duration=dur)
        k += 1
        if k % 18 == 0:
            _log(ctx, f"crank {math.degrees(total):.0f} deg")


def _guide(ctx, info, mode):
    L = ctx.left
    while not info.get("done"):
        y = WELD_Y if mode == "fixed" else WELD_Y + PITCH * (info["turns"] + LEAD)
        yield from L.move((GX, min(y, DRUM_Y1 - 0.01), GUIDE_Z), yaw=0.0, duration=0.15)


def _take_hose(ctx, finger):
    L = ctx.left
    pts = ctx.states()[HOSE.name].points
    q = pts[5]  # 11 cm from the clamped end, on the table in front of the drum
    yield from L.move((q[0], q[1], TOP + 0.07), yaw=0.0, finger=-0.020, duration=1.5)
    yield from L.move((q[0], q[1], GUIDE_Z), yaw=0.0, duration=0.8)
    yield from L.grip(finger, 0.5)
    yield from L.move((GX, WELD_Y, GUIDE_Z), yaw=0.0, duration=0.5)


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("REEL_CHEAT", "")
    _log(ctx, "start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return
    if cheat in ("handturn", "slowrim"):  # left arm turns the drum by its left flange rim (no crank), guiding nothing
        slow = cheat == "slowrim"  # slow swings: about 0.15 rad/s
        for _ in range(4 if slow else 6):
            top = (AX_X + 0.012, DRUM_Y1 + 0.0005 + FL_T / 2, AX_Z + FL_R + 0.006)
            yield from L.move((top[0], top[1], top[2] + 0.04), yaw=math.pi / 2, finger=-0.020, duration=0.8)
            yield from L.move(top, yaw=math.pi / 2, duration=0.5)
            yield from L.grip(FINGER_CLOSED, 0.4)
            a1 = math.radians(150)
            yield from L.move((AX_X + (FL_R - 0.004) * math.cos(a1), top[1], AX_Z + (FL_R - 0.004) * math.sin(a1)),
                              yaw=math.pi / 2, duration=7.0 if slow else 0.5)
            yield from L.grip(-0.020, 0.3)
        _log(ctx, "after rim turn")
        yield from L.move((0.25, 0.25, TOP + 0.10), yaw=0.0, duration=1.0)
        yield from L.wait(1.0)
        _log(ctx, "end")
        return

    # 1) right arm takes the crank handle
    k = ctx.states()[KNOB.name].pos
    yield from Rt.move((k[0], k[1], k[2] + 0.06), yaw=RYAW, finger=-0.025, duration=1.5)
    yield from Rt.move((k[0], k[1], k[2] + 0.006), yaw=RYAW, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    # 2) left arm puts its fingers round the hose in front of the drum
    info = {"turns": 0.0}
    if cheat != "crankonly":
        yield from _take_hose(ctx, GUIDE_F)
    _log(ctx, "ready")
    stop = {"tier03": 0.31, "tier06": 0.35, "stop39": 0.39}.get(cheat, STOP_WOUND)

    # 3) crank and guide together
    def crank():
        yield from _crank(ctx, stop, info)
        info["done"] = True

    if cheat == "crankonly":
        yield from crank()
    else:
        yield from ctx.both(crank(), _guide(ctx, info, "fixed" if cheat == "fixedguide" else "walk"))
    _log(ctx, "wound")
    if cheat == "hold":
        yield from Rt.wait(2.0)
        _log(ctx, "end")
        return
    # 4) let go of the handle and the hose
    yield from Rt.grip(FINGER_OPEN, 0.4)
    g = Rt.tcp()
    yield from Rt.move((g[0], g[1] - 0.02, g[2] + 0.06), yaw=RYAW, duration=0.6)
    yield from Rt.move((0.25, -0.25, TOP + 0.10), yaw=0.0, duration=1.0)
    if cheat != "crankonly":
        yield from L.grip(-0.020, 0.3)
        g = L.tcp()
        yield from L.move((g[0], g[1], TOP + 0.07), yaw=0.0, duration=0.6)
    yield from L.move((0.25, 0.25, TOP + 0.10), yaw=0.0, duration=1.0)
    yield from L.wait(1.5)
    _log(ctx, "end")
