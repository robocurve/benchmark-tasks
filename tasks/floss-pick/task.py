"""On the station table: a dental training head clamped upright on a plywood board on a shipping scale, mouth open,
facing the robot; its typodont's two lower central incisors (31, 41) are coated green on their facing sides. A
disposable F-shaped floss pick lies flat on the table to the robot's right. Floss between the two lower front teeth
with the pick: ease the floss through the contact with a back-and-forth, wipe each tooth's side, bring it back out.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score --max-steps 3600 ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import Shape, UsdBody, after_spawn, before_reset, on_reset, on_step, physics  # noqa: E402

import geometry as g  # noqa: E402
from floss import Floss, Rules, in_region  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Floss between the two lower front teeth of the dental training head with the floss pick: ease the "
               "floss through the contact with a gentle back-and-forth without snapping it into the gum, wipe each "
               "tooth's side up and down, then bring it back out.")

PHYSICS_HZ = 240
physics(PHYSICS_HZ)

TOP = g.TOP
GREY = (0.55, 0.56, 0.58)
DARK = (0.08, 0.08, 0.09)
WOOD = (0.76, 0.62, 0.43)
LCD = (0.62, 0.70, 0.60)

# -- objects -----------------------------------------------------------------------------------------------------------
HEAD = UsdBody("head", usd="assets/head.usdc", pos=tuple(g.CONTACT), size=(0.17, 0.15, 0.27), friction=(0.5, 0.5))
LIP = UsdBody("lip", usd="assets/lip.usda", pos=tuple(g.CONTACT), size=(0.01, 0.04, 0.015), articulation=True,
              joint_pos=(("slide", 0.0),), drives=(("slide", g.LIP_K, g.LIP_DAMP),),
              friction=(g.LIP_MU, g.LIP_MU), friction_min=True, solver_iters=32)
_PICK_Y = g.PICK_START[1] - (g.Z_HANDLE[0] + g.Z_HANDLE[1]) / 2 / 1000
_PICK_X = g.PICK_START[0] - (g.X_REAR + g.X_TIP) / 2 / 1000
PICK_POS0 = (_PICK_X, _PICK_Y, TOP + g.BOARD_TOP + g.HANDLE_T / 2 / 1000 + 0.0002)
PICK_ROT0 = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))     # lying flat on the board: arms toward -y, handle along x
PICK = UsdBody("pick", usd="assets/pick.usda", pos=PICK_POS0, rot_xyzw=PICK_ROT0, size=(0.075, 0.020, 0.006),
               mass=g.PICK_MASS, friction=(2.0, 1.8), solver_iters=32, torsional_patch=0.0065)


@after_spawn("lip")
def _lip_filtered(prim) -> None:
    """The lip touches only the pick and the robot: its contact with the head (it rests on the teeth) is the joint's
    stop and spring, not a collision."""
    from pxr import UsdPhysics

    body = prim.GetChild("body")
    fp = UsdPhysics.FilteredPairsAPI.Apply(body)
    fp.CreateFilteredPairsRel().SetTargets([prim.GetPath().GetParentPath().AppendChild("head")])
    _hide(prim, "visual")


def _hide(prim, name) -> None:
    """Display only: the named descendant is not drawn (its collider, if any, still acts)."""
    from pxr import Usd, UsdGeom

    for p in Usd.PrimRange(prim):
        if p.GetName() == name:
            UsdGeom.Imageable(p).MakeInvisible()


@after_spawn("head")
def _head_cutaway(prim) -> None:
    """Display only: the mouth's dark interior is not drawn, so the cameras see into the mouth (the lower lip is drawn
    cut away too, above), and a small lamp inside the mouth lights it. Physics and scoring are untouched."""
    from pxr import Gf, UsdGeom, UsdLux

    _hide(prim, "cavity")
    lamp = UsdLux.SphereLight.Define(prim.GetStage(), prim.GetPath().AppendChild("mouth_lamp"))
    lamp.CreateRadiusAttr(0.005)
    lamp.CreateIntensityAttr(MOUTH_LAMP)
    UsdGeom.Xformable(lamp.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.012, 0.0, 0.012))


MOUTH_LAMP = 1.0e5                  # display only: the lamp inside the mouth (12 mm behind and above the contact)


def _box(name, x, y, z, color, collide=True, kinematic=False):
    return Shape(name, pos=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, TOP + (z[0] + z[1]) / 2),
                 size=(x[1] - x[0], y[1] - y[0], z[1] - z[0]), color=color, collide=collide, kinematic=kinematic)


_px, _py, _pz = g.PHONE_C
SHAPES = [
    _box("scale", g.SCALE_X, g.SCALE_Y, (0.0, g.SCALE_H - 0.004), GREY),
    _box("scale_platform", (g.SCALE_X[0] + 0.004, g.SCALE_X[1] - 0.004), (g.SCALE_Y[0] + 0.004, g.SCALE_Y[1] - 0.004),
         (g.SCALE_H - 0.004, g.SCALE_H), DARK),
    _box("board", g.SCALE_X, g.SCALE_Y, (g.SCALE_H, g.BOARD_TOP), WOOD, kinematic=True),
    _box("display", *g.DISPLAY, DARK, kinematic=True),
    _box("display_screen", (g.DISPLAY[0][0] + 0.006, g.DISPLAY[0][1] - 0.006), (g.DISPLAY[1][1], g.DISPLAY[1][1] + 0.001),
         (g.DISPLAY[2][0] + 0.010, g.DISPLAY[2][1] - 0.004), LCD, collide=False),
    _box("phone", (_px - 0.0375, _px + 0.0375), (_py - 0.004, _py + 0.004), (_pz - 0.075, _pz + 0.075), DARK),
    _box("phone_clamp", (_px - 0.012, _px + 0.012), (_py - 0.010, _py + 0.010), (_pz - 0.090, _pz - 0.075), GREY),
]
for _k, _a in enumerate((0.0, 2.094, 4.189)):
    _dx, _dy = 0.07 * math.cos(_a), 0.07 * math.sin(_a)
    _len = math.sqrt(0.07 ** 2 + (_pz - 0.09) ** 2)
    _tilt = math.atan2(0.07, _pz - 0.09)
    _ax = np.array([-math.sin(_a), math.cos(_a), 0.0])          # tilt about this horizontal axis
    _q = (*(_ax * math.sin(-_tilt / 2)), math.cos(-_tilt / 2))
    SHAPES.append(Shape(f"tripod_leg{_k}", kind="capsule", radius=0.004, height=_len - 0.008, axis="Z",
                        pos=(_px + _dx / 2, _py + _dy / 2, TOP + (_pz - 0.09) / 2), rot_xyzw=tuple(map(float, _q)),
                        color=DARK))
OBJECTS = [HEAD, LIP, PICK] + SHAPES

HAND_LINKS = ("gripper", "tip_left", "tip_right")
ROBOT_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
_STATE: dict = {}
_VIEWS: dict = {}
_HOOK: dict = {}
_VIS: dict = {}
_DEBUG = bool(os.environ.get("FP_DEBUG"))
_TRACE = bool(os.environ.get("FP_TRACE"))
SHOW = os.environ.get("FP_SHOW", "1") == "1"        # draw the floss and the band (display only)
FL = Floss()
RU = Rules(substeps_snap=round(g.SNAP_S * PHYSICS_HZ))


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), np.float64)
    return np.asarray(x, np.float64)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@before_reset
def _restart(environment) -> None:
    """The lip at rest on the teeth (its spring preloaded against the stop)."""
    from scenekit import set_joint_targets, write_joint_positions

    write_joint_positions(environment, "lip", {"slide": 0.0})
    set_joint_targets(environment, "lip", {"slide": g.LIP_PRELOAD / g.LIP_K})


@on_reset
def _reset_state(environment=None) -> None:
    FL.reset()
    RU.reset()
    _HOOK.update(sub=0, fdown=[], vmax=0.0, pick=None, mid_hist=[])
    _STATE.update(step=0, why=[], zero=False, press_steps=0, press_peak=0.0, press_win=[], press_hold_peak=0.0, press_now=0.0, contact_down=0.0,
                  rung=0.0, rungs=[], out=False, cov={"t41": 0.0, "t31": 0.0}, lip=0.0, mode="above", zs=g.ZONE[1],
                  F_plug=0.0, T=g.T0, load=0.0, speed_land=[], press_hist=[], f_plug_max=0.0)
    _VIS.pop("n_wiped", None)
    if _VIS.pop("strip_marked", None) and "strip" in _VIS:
        from pxr import Gf, Vt
        _VIS["strip"].GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.93, 0.93, 0.90)]))
    if environment is not None:
        _install(environment)


_reset_state()


def _fail(why: str) -> None:
    if why not in _STATE["why"]:
        _STATE["why"].append(why)
    _STATE["zero"] = True


# -- the floss, every physics step ---------------------------------------------------------------------------------------
def _floss_substep(base) -> None:
    """Solve the floss for the pick's current pose and apply its pull to the pick (force and torque), before the
    physics step. Advances the floss-side rules (the film strip at 240 Hz, wipe, the floss's load)."""
    import torch

    pick = base.scene["pick"]
    origin = _HOOK["origin"]
    pose = _np(pick.data.root_link_pose_w)[0]
    vel = _np(pick.data.root_link_vel_w)[0]
    p0 = pose[:3] - origin
    R = _rot(pose[3:7])
    A = g.to_head(p0 + R @ g.ANCHOR_A / 1000)
    B = g.to_head(p0 + R @ g.ANCHOR_B / 1000)
    v_lin, w = vel[:3], vel[3:6]
    p0h = g.to_head(p0)

    def vel_at(pts_h):
        r = (np.asarray(pts_h) - p0h) / 1000.0
        return (v_lin[None] + np.cross(w[None], r)) * 1000.0      # mm/s

    t_hat = (B - A) / max(np.linalg.norm(B - A), 1e-9)
    mid = (A + B) / 2
    hist = _HOOK.setdefault("mid_hist", [])
    hist.append(mid)
    n_win = max(2, round(g.VS_WINDOW / _HOOK["dt"]) + 1)
    del hist[:-n_win]
    # the floss's sliding along itself: its travel along its own line over the last VS_WINDOW (sawing), not the
    # contact jitter of a single physics step
    v_slide = float((hist[-1] - hist[0]) @ t_hat / ((len(hist) - 1) * _HOOK["dt"])) if len(hist) > 1 else 0.0
    mode0 = FL.mode
    P = FL.step(A, B, v_slide, _HOOK["dt"])
    RU.step(FL, P, vel_at)
    if _TRACE:
        fn = float(np.linalg.norm(FL.F_A + FL.F_B))
        if fn > 1.0 or FL.mode != mode0:
            print(f"[sub {_HOOK['sub']}] mode {mode0}->{FL.mode} zs {FL.zs:.3f} F {np.round(FL.F_A + FL.F_B, 2).tolist()} "
                  f"Fplug {FL.F_plug:.2f} T {FL.T:.2f} A {np.round(A, 2).tolist()} B {np.round(B, 2).tolist()} "
                  f"on {dict((k, round(v, 2)) for k, v in FL.on.items())} vs {v_slide:.1f} zmin {P[:, 2].min():.2f}",
                  flush=True)
    _HOOK["sub"] += 1
    _HOOK["fdown"].append(RU.f_down)
    _HOOK["on41"] = max(_HOOK.get("on41", 0.0), FL.on.get("t41", 0.0))
    _HOOK["on31"] = max(_HOOK.get("on31", 0.0), FL.on.get("t31", 0.0))
    _HOOK["P"] = P
    if RU.failed:
        return
    fa = torch.tensor(FL.F_A, dtype=torch.float32, device=pick.device).reshape(1, 1, 3)
    fb = torch.tensor(FL.F_B, dtype=torch.float32, device=pick.device).reshape(1, 1, 3)
    pa = torch.tensor(g.to_world(A) + origin, dtype=torch.float32, device=pick.device).reshape(1, 1, 3)
    pb = torch.tensor(g.to_world(B) + origin, dtype=torch.float32, device=pick.device).reshape(1, 1, 3)
    comp = pick.instantaneous_wrench_composer
    comp.set_forces_and_torques_index(forces=fa, positions=pa, body_ids=[0], is_global=True)
    comp.add_forces_and_torques_index(forces=fb, positions=pb, body_ids=[0], is_global=True)


def _install(environment) -> None:
    """Hook the scene's per-substep write so the floss is solved and applied before every physics step."""
    import yam_arena.constants as C

    base = environment.unwrapped
    scene = base.scene
    _HOOK["origin"] = _np(scene.env_origins[0])
    _HOOK["dt"] = float(C.PHYSICS_DT)
    if getattr(scene, "_floss_hook", False):
        return
    write = scene.write_data_to_sim

    def write_with_floss(*a, **k):
        try:
            _floss_substep(base)
        except Exception as e:                               # never take the simulation down
            _HOOK["err"] = repr(e)
            if _DEBUG:
                raise
        return write(*a, **k)

    scene.write_data_to_sim = write_with_floss
    scene._floss_hook = True


# -- contact views -------------------------------------------------------------------------------------------------------
def _views(environment) -> dict:
    """The head and the lip against the pick and each arm's hand and wrist links (net force on the head / lip)."""
    if not _VIEWS:
        from yam_arena.robot import asset_link_paths

        import yam_arena.constants as C

        psv = environment.unwrapped.sim.physics_sim_view
        ns = "/World/envs/env_*"
        lp = asset_link_paths()
        robot = [f"{ns}/{arm}{lp[link]}" for arm in ("LeftArm", "RightArm") for link in ROBOT_LINKS]
        filters = [f"{ns}/pick"] + robot
        _VIEWS["head"] = psv.create_rigid_contact_view(f"{ns}/head", filter_patterns=filters)
        _VIEWS["lip"] = psv.create_rigid_contact_view(f"{ns}/lip/body", filter_patterns=filters)
        # the board and the scale's display sit on the scale too: the pick's and the hands' contacts on them
        for name in ("board", "display"):
            _VIEWS[name] = None
            for pat in (f"{ns}/{name}",):
                try:
                    v = psv.create_rigid_contact_view(pat, filter_patterns=filters)
                    if v._backend is not None and v.sensor_count > 0:
                        _VIEWS[name] = v
                        print("[contact view]", name, pat, v.sensor_count, flush=True)
                        break
                except Exception as e:                                    # noqa: BLE001
                    print("[contact view]", name, pat, "failed:", e, flush=True)
        if _DEBUG:
            links = [f"{ns}/RightArm{lp[link]}" for link in C.LINKS]
            _VIEWS["pick_dbg"] = psv.create_rigid_contact_view(f"{ns}/pick", filter_patterns=links)
            _VIEWS["dbg_names"] = list(C.LINKS)
        _VIEWS["dt"] = float(C.PHYSICS_DT)
    return _VIEWS


def floss_out(P) -> bool:
    """The floss back above the incisal edges: no part of it over the two crowns (the lower front teeth) is below
    the edges."""
    over = (np.abs(P[:, 1]) <= 2 * g.B0 + g.GAP_HALF + 0.5) & (P[:, 0] >= -3.5) & (P[:, 0] <= 3.5)
    return bool(not over.any() or P[over, 2].min() >= g.Z_EDGE + g.FLOSS_R)


# -- every control step ---------------------------------------------------------------------------------------------------
@on_step
def _monitor(environment) -> None:
    S = _STATE
    S["step"] += 1
    if "err" in _HOOK:
        print("[floss hook error]", _HOOK.pop("err"), flush=True)
    V = _views(environment)
    dt = V["dt"]
    f_head = _np(V["head"].get_contact_force_matrix(dt))[0]          # (filters, 3): on the head
    f_lip = _np(V["lip"].get_contact_force_matrix(dt))[0]
    contact_down = max(0.0, -float(f_head[:, 2].sum() + f_lip[:, 2].sum()))
    board_down = 0.0                                                    # on the board and the scale's display
    for name in ("board", "display"):
        if V[name] is not None:
            board_down -= float(_np(V[name].get_contact_force_matrix(dt))[..., 2].sum())
    board_down = max(0.0, board_down)
    S["board_down"] = board_down
    contact_down += board_down
    if _DEBUG:
        names = ["pick"] + [f"{a[0]}{l}" for a in ("L", "R") for l in ROBOT_LINKS]
        S["contacts"] = {f"head:{n}": np.round(f, 2).tolist() for n, f in zip(names, f_head) if np.linalg.norm(f) > 0.02}
        S["contacts"].update({f"lip:{n}": np.round(f, 2).tolist() for n, f in zip(names, f_lip) if np.linalg.norm(f) > 0.02})
    floss_down = float(np.mean(_HOOK["fdown"])) if _HOOK["fdown"] else 0.0
    _HOOK["fdown"] = []
    S["on41_max"], S["on31_max"] = _HOOK.pop("on41", 0.0), _HOOK.pop("on31", 0.0)
    press = floss_down + contact_down
    S.update(contact_down=contact_down, press_now=press, mode=FL.mode, zs=FL.zs, F_plug=FL.F_plug, T=FL.T)
    S["f_plug_max"] = max(S["f_plug_max"], abs(FL.F_plug))
    if _DEBUG and "pick_dbg" in V:
        fp = _np(V["pick_dbg"].get_contact_force_matrix(dt))[0]
        nz = {n: round(float(np.linalg.norm(f)), 2) for n, f in zip(V["dbg_names"], fp) if np.linalg.norm(f) > 0.01}
        S["pick_contacts"] = nz
        S["floss_net"] = round(float(np.linalg.norm(FL.F_A + FL.F_B)), 3)
    lip = environment.unwrapped.scene["lip"]
    S["lip"] = float(_np(lip.data.joint_pos)[0][0])
    S["press_peak"] = max(S["press_peak"], press)                       # the scale, the whole trial
    S["press_win"] = (S["press_win"] + [press])[-round(g.PRESS_S * 20):]
    if len(S["press_win"]) == round(g.PRESS_S * 20):                    # the largest press held for 0.5 s
        S["press_hold_peak"] = max(S["press_hold_peak"], min(S["press_win"]))
    S["press_steps"] = S["press_steps"] + 1 if press >= g.PRESS_N else 0
    if S["press_steps"] >= round(g.PRESS_S * 20):
        _fail("pressed on the head or board: 3 N or more for 0.5 s")
    if RU.snap is not None:
        _fail(f"snapped onto the gum: the pressure film marked ({RU.snap:.2f} N on the strip)")
    if RU.failed:
        _fail("the floss's load passed 22 N (pulled out or broke)")
    S["load"] = RU.load_max
    cov = RU.coverage()
    S["cov"] = cov
    P = _HOOK.get("P")
    S["out"] = bool(P is not None and floss_out(P))
    wiped = sum(cov[k] >= g.WIPED for k in ("t41", "t31"))
    rung = 0.0
    passed = any(RU.cells(w).reshape(g.BAND_COLS, g.BAND_ROWS)[:, 1:].any()   # a bare cell in rows 2-4 of either band
                 for w in RU.wiped.values())
    S["passed"] = passed
    if passed and not RU.failed:
        rung = {0: 0.25, 1: 0.5, 2: 0.75}[wiped]
        if wiped == 2 and S["out"]:
            rung = 1.0
    S["rung"] = rung
    S["rungs"] = (S["rungs"] + [rung])[-g.HOLD_STEPS:]
    S["summary"] = {"passed": passed, "coverage": {k: round(v, 3) for k, v in cov.items()},
                    "landings_mm_per_25ms": [round(x, 3) for x in RU.landings],
                    "peak_drop_mm_per_25ms": round(RU.peak_drop, 3), "strip_peak_N": round(RU.pap_peak, 3),
                    "peak_press_N": round(S["press_peak"], 3), "peak_press_held_05s_N": round(S["press_hold_peak"], 3), "floss_load_max_N": round(RU.load_max, 2),
                    "plug_force_max_N": round(S["f_plug_max"], 2), "out": S["out"], "mode": FL.mode,
                    "coverage_at_f_wipe": {str(thr): {k: round(float(RU.cells(v).mean()), 3) for k, v in w.items()}
                                           for thr, w in RU.wiped_alt.items()}}
    if SHOW:
        _draw(environment)


FLOSS_DRAW_W = 0.001                # m: the floss drawn 1 mm thick (display only; the string is 0.5 mm)


def _draw(environment) -> None:
    """Display only: the floss (a row of 1 mm bright blue dots between the arm tips), the green coating on each
    incisor's band (dots removed where wiped) and its four scribed marks, and the pressure film strip on the papilla
    (red once marked), USD Points with no collider, mass or contact; physics and score are untouched."""
    import omni.usd
    from pxr import Gf, UsdGeom, Vt

    stage = omni.usd.get_context().get_stage()
    origin = _HOOK["origin"]
    if "floss" not in _VIS or not _VIS["floss"].GetPrim().IsValid():
        f = UsdGeom.Points.Define(stage, "/World/FlossLine")
        f.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.45, 1.0)]))     # bright blue, drawn 1 mm thick
        _VIS["floss"] = f
        b = UsdGeom.Points.Define(stage, "/World/BandCoat")
        b.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.10, 0.62, 0.22)]))
        _VIS["band"] = b
        m = UsdGeom.Points.Define(stage, "/World/BandMarks")
        m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.15, 0.12, 0.10)]))
        _VIS["marks"] = m
        st = UsdGeom.Points.Define(stage, "/World/FilmStrip")              # the pressure film strip on the papilla
        st.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.93, 0.93, 0.90)]))
        sp = [(x, y, float(g.pap_z(x)) + 0.06) for x in np.arange(-g.STRIP_L / 2, g.STRIP_L / 2 + 1e-9, 0.12)
              for y in np.linspace(-g.STRIP_W / 2 + 0.1, g.STRIP_W / 2 - 0.1, 5)]
        SP = np.array([g.to_world(np.array(p)) + origin for p in sp])
        st.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in SP]))
        st.GetWidthsAttr().Set(Vt.FloatArray([0.00022] * len(SP)))
        _VIS["strip"] = st
        pts = []
        for k, s in (("t41", 1.0), ("t31", -1.0)):       # the four scribed marks framing each band
            n_ = RU.band_n[k].mean(0)
            n_ /= np.linalg.norm(n_)

            def at(x, z):
                return np.array([x, s * g.mesial_y41(x, z), z]) + n_ * 0.06

            for x in np.linspace(-g.BAND_W / 2, g.BAND_W / 2, 21):
                pts.append(at(x, g.ZONE[0] + 0.1))                          # the groove at the contact's lower edge
                pts.append(at(x, float(g.pap_z(x)) + g.BAND_GUM_CLEAR - 0.1))  # the groove 1 mm above the gum line
            for x in (-g.BAND_W / 2 - 0.1, g.BAND_W / 2 + 0.1):              # the two vertical ticks, 2 mm apart
                for z in np.linspace(g.ZONE[0], float(g.pap_z(x)) + g.BAND_GUM_CLEAR, 8):
                    pts.append(at(x, z))
        P = np.array([g.to_world(p) + origin for p in pts])
        m.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in P]))
        m.GetWidthsAttr().Set(Vt.FloatArray([0.00012] * len(P)))
    P = _HOOK.get("P")
    if P is not None:
        seg = np.linspace(0, 1, 4)[:-1]
        dense = np.vstack([P[i] + seg[:, None] * (P[i + 1] - P[i]) for i in range(len(P) - 1)] + [P[-1:]])
        W = np.array([g.to_world(p) + origin for p in dense])
        _VIS["floss"].GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in W]))
        _VIS["floss"].GetWidthsAttr().Set(Vt.FloatArray([FLOSS_DRAW_W] * len(W)))
    if RU.snap is not None and not _VIS.get("strip_marked"):             # the film turns red where it was struck
        _VIS["strip"].GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.85, 0.10, 0.12)]))
        _VIS["strip_marked"] = True
    n = int(sum(v.sum() for v in RU.wiped.values()))
    if _VIS.get("n_wiped") != n:
        pts = []
        for k in ("t41", "t31"):
            keep = ~RU.wiped[k]
            pts.append(RU.band[k][keep] + RU.band_n[k][keep] * 0.05)
        Q = np.vstack(pts)
        W = np.array([g.to_world(p) + origin for p in Q]) if len(Q) else np.zeros((0, 3))
        _VIS["band"].GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in W]))
        _VIS["band"].GetWidthsAttr().Set(Vt.FloatArray([0.00022] * len(W)))
        _VIS["n_wiped"] = n


def success(objects) -> bool:
    """The full-score state with no zero condition (yam_arena ends the trial once it has held for 1 s)."""
    S = _STATE
    return bool(not S["zero"] and S["rung"] == 1.0)


def score(objects) -> float:
    """Read every control step (20 Hz); the floss and its rules every physics step (240 Hz).
    0 for the run (latched) if:
    - snap: the floss marks the pressure film strip on the papilla under the contact (2 N on the strip);
    - press: at any time, the downward force on the head and board (the floss's push on the teeth, gum and contact,
      plus the pick's and the grippers' contact forces on the head, lip, board and display) is 3 N or more for 0.5 s;
    - the floss's load passes 22 N (it pulls out of the pick or breaks).
    Passed: any cell in rows 2-4 of either band is wiped (the bands lie under the contact). A side is wiped when 32 of
    its band's 40 cells are (the band: 2 mm wide, centred under the contact, from the contact's lower edge to 1 mm
    above the gum, as
    10 columns by 4 rows); a cell when the floss has slid over 3 of its 4 points (within 0.1 mm, at 1 mm/s or more)
    while pressing that tooth with 0.5 N or more.
    Otherwise, from the final 1 s (the lowest rung any of its steps reached):
    1.0 passed, both sides wiped, the floss back above the incisal edges; 0.75 passed, both wiped, the floss still
    down between the teeth; 0.5 passed, one side wiped; 0.25 passed, no side wiped; 0 never passed."""
    S = _STATE
    if S["zero"] or not S["rungs"]:
        return 0.0
    return float(min(S["rungs"]))
