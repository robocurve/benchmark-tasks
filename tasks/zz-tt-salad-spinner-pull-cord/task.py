"""Spin washed lettuce dry in a pull-cord salad spinner (Zyliss Easy Spin 2, large: 26 cm across, 14.5 cm tall).

Scene (metres from the arm-base midpoint, x forward, y left; heights above the counter). The spinner stands on the
counter at (0.30, -0.08): a clear bowl, a green slotted basket on the bowl's centre pivot holding six pieces of a
romaine lettuce replica, and a white lid that sits on the bowl rim. The pull cord leaves a raised hub at the lid's
centre; its green T-handle stands docked in the hub socket (the bar runs front to back, resting on the 1.2 cm hub,
bar centre 16.5 cm above the counter).

Mechanism (as in any pull-cord / recoil-starter spinner): pulling the cord out turns a spool in the lid that drives the
basket through a one-way clutch; a coil spring rewinds the cord, and while it rewinds the clutch free-wheels, so the
basket keeps its speed and the next pull adds to it. The cord pulls the lid up and the basket's start-up torque twists
it, so one hand must press the lid (and with it the bowl) down on the counter while the other pulls.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "Spin the lettuce dry in the salad spinner."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Spin the lettuce dry in the salad spinner."

CLEAR_C = (0.86, 0.92, 0.95)
WHITE = (0.94, 0.94, 0.92)
GREEN = (0.36, 0.70, 0.18)
BASK_G = (0.16, 0.48, 0.14)
DGREEN = (0.20, 0.45, 0.12)
LEAF = (0.64, 0.88, 0.32)
GREY = (0.35, 0.36, 0.38)
RED = (0.90, 0.10, 0.08)
WALL_OPACITY = 0.28  # clear SAN bowl

# -- geometry (m) -----------------------------------------------------------------------------------------------------
C = np.array([0.30, -0.08])  # spinner centre on the counter
R_OUT = 0.130  # 26 cm across
BOWL_H = 0.125  # bowl rim height; the lid on it brings the top to 14.5 cm
LID_T = 0.020
LID_TOP = BOWL_H + LID_T  # 0.145 above the counter
HUB_R0, HUB_R1, HUB_H = 0.0078, 0.024, 0.012  # raised cord hub: socket radius, outer radius, height
STEM_R, STEM_L = 0.006, 0.012  # T-handle stem: as tall as the hub, so the docked bar lies on the hub top
BAR_L, BAR_W = 0.080, 0.016  # T-handle bar (along x)
BASK_R, BASK_H, BASK_Z0 = 0.112, 0.100, 0.012  # basket radius, height, floor height above the counter

BOWL_M, LID_M, BASK_M, HANDLE_M = 0.42, 0.30, 0.19, 0.03  # bowl + lid + basket + handle = 0.94 kg (896-915 g listed)
LEAVES_M = 0.054  # six replica leaf pieces, 9 g each
B_C = (float(C[0]), float(C[1]), TOP + BOWL_H / 2)
L_C = (float(C[0]), float(C[1]), TOP + BOWL_H + LID_T / 2)
K_C = (float(C[0]), float(C[1]), TOP + BASK_Z0 + BASK_H / 2)
EYE_L = np.array([0.0, 0.0, LID_T / 2])  # cord exit (socket floor) in the lid frame
BAR_Z = TOP + LID_TOP + STEM_L + BAR_W / 2  # docked bar centre (0.165 above the counter)
H_C = (float(C[0]), float(C[1]), BAR_Z + 0.0015)  # spawned just above its seat
ANCH_L = np.array([0.0, 0.0, -(STEM_L + BAR_W / 2)])  # cord end at the stem's foot, handle frame
H_COM = np.array([0.0, 0.0, -0.006])

BOWL = Cylinder("spinner_bowl", pos=B_C, radius=R_OUT, height=BOWL_H, mass=BOWL_M, color=CLEAR_C)
BASKET = Cylinder("spinner_basket", pos=K_C, radius=BASK_R, height=BASK_H, mass=BASK_M + LEAVES_M, color=GREEN)
LID = Cylinder("spinner_lid", pos=L_C, radius=R_OUT, height=LID_T, mass=LID_M, color=WHITE)
HANDLE = Box("cord_handle", pos=H_C, size=(BAR_L, BAR_W, BAR_W), mass=HANDLE_M, color=GREEN)

OBJECTS = [BOWL, BASKET, LID, HANDLE]

# -- drive model --------------------------------------------------------------------------------------------------------
I_B = 0.0016  # basket + leaves about the spin axis (kg m^2)
GEAR = 66.0  # basket rad per metre of cord paid out (about 4.7 turns per full 45 cm pull)
T_REWIND0, K_REWIND = 1.5, 2.0  # rewind spring: 1.5 N docked, +2 N/m drawn out
S_MAX = 0.45  # cord length
S_ENGAGE = 0.004  # the spool only turns once the cord is drawn out of the hub
T_CATCH = 0.04  # clutch/cord compliance: the basket catches up with the spool within ~0.04 s
TAU_MAX = 0.33  # most torque the cord/clutch passes (N m; ~20 N on the cord)
DRAG_C0, DRAG_C1 = 0.0010, 1.0e-4  # pivot friction (N m) and air/viscous drag (N m s): ~20 s coast from 300 rpm
K_STOP, C_STOP = 4000.0, 20.0  # cord fully drawn: stop
DT = 1.0 / 240.0
RPM = 60.0 / (2 * math.pi)
RPM_SPIN, SPIN_S = 200.0, 5.0  # drying: at least 5 s in total at 200 rpm or more

TWIN_EXTRA = [
    {"name": "lid_on_rim", "kind": "joint", "real": "lid resting in the bowl rim", "note": "sim: slides only "
     "straight up off the rim (0-4 cm), seated by gravity and any press"},
    {"name": "basket_pivot", "kind": "joint", "real": "basket on the bowl's centre pivot", "note": "revolute"},
    {"name": "cord_spool_clutch", "kind": "model", "real": "cord spool + one-way clutch + rewind spring in the lid",
     "note": "cord force applied between hub and handle each physics step; drives the basket only while paying out"},
]

_P: dict = {}


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# -- authoring --------------------------------------------------------------------------------------------------------
def _author(root: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    bowl, lid, bask, hnd = (f"{root}/{o.name}" for o in (BOWL, LID, BASKET, HANDLE))
    _P.update(root=root, bowl=bowl, lid=lid, basket=bask, handle=hnd)

    def hide_own(path, collide_off=True):
        for p in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if collide_off and p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()

    def mass(path, m, diag, com=(0.0, 0.0, 0.0)):
        a = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(path))
        a.CreateMassAttr(float(m))
        a.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        a.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, diag)))

    looks = {}

    def look(rgb, op=1.0, rough=0.4):
        key = (tuple(rgb), op)
        if key not in looks:
            path = f"{root}/Looks/spin_{len(looks)}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough, opacity=op)
            cfg.func(path, cfg)
            looks[key] = path
        return looks[key]

    mats = {}

    def pmat(mu):
        if mu not in mats:
            path = f"{root}/spin_pmat_{len(mats)}"
            mc = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu * 0.9)
            mc.func(path, mc)
            mats[mu] = path
        return mats[mu]

    def part(parent, name, kind, pos, size=None, radius=None, height=None, yaw=0.0, rgb=WHITE, op=1.0,
             collide=True, mu=0.7, visible=True):
        path = f"{parent}/{name}"
        if kind == "box":
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            x = UsdGeom.Xformable(g)
            x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
            if yaw:
                x.AddRotateZOp().Set(float(math.degrees(yaw)))
            x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        else:
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(float(radius))
            g.CreateHeightAttr(float(height))
            g.CreateAxisAttr("Z")
            UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        if visible:
            bind_visual_material(path, look(rgb, op), stage=stage)
        else:
            UsdGeom.Imageable(g).MakeInvisible()
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            bind_physics_material(path, pmat(mu), stage=stage)
        return path

    def ring(parent, name, r_mid, thick, h, zc, n, rgb, op=1.0, collide=True, gap=1.02, mu=0.7, phase=0.0,
             visible=True):
        chord = 2 * r_mid * math.tan(math.pi / n) * gap
        for i in range(n):
            a = 2 * math.pi * (i + phase) / n
            part(parent, f"{name}_{i}", "box", (r_mid * math.cos(a), r_mid * math.sin(a), zc), size=(thick, chord, h),
                 yaw=a, rgb=rgb, op=op, collide=collide, mu=mu, visible=visible)

    # bowl: clear wall ring + floor, grey non-slip foot ring (TPE)
    hide_own(bowl)
    mass(bowl, BOWL_M, (0.5 * BOWL_M * R_OUT ** 2 * 0.8, 0.5 * BOWL_M * R_OUT ** 2 * 0.8, BOWL_M * R_OUT ** 2 * 0.8))
    zb = -BOWL_H / 2
    part(bowl, "floor", "cyl", (0, 0, zb + 0.004), radius=R_OUT - 0.012, height=0.006, rgb=CLEAR_C, op=WALL_OPACITY)
    part(bowl, "foot", "cyl", (0, 0, zb + 0.0015), radius=R_OUT - 0.010, height=0.003, rgb=GREY, mu=0.9)
    # the clear wall: colliding panels drawn see-through (only the rolled rim, the base band and faint seams show)
    ring(bowl, "wall", R_OUT - 0.002, 0.004, BOWL_H - 0.006, zb + 0.006 + (BOWL_H - 0.006) / 2, 32, CLEAR_C,
         visible=False)
    ring(bowl, "lip", R_OUT - 0.001, 0.005, 0.008, BOWL_H / 2 - 0.004, 32, CLEAR_C, collide=False)
    ring(bowl, "baseband", R_OUT - 0.002, 0.004, 0.012, zb + 0.006, 32, CLEAR_C, collide=False)
    ring(bowl, "seam", R_OUT - 0.001, 0.002, BOWL_H - 0.02, 0.0, 8, CLEAR_C, collide=False, gap=0.04)
    part(bowl, "pivot", "cyl", (0, 0, zb + 0.010), radius=0.010, height=0.008, rgb=WHITE, collide=False)

    # basket: no contacts (it turns inside the bowl on its pivot); slats, floor, rim, tape mark and the leaves
    hide_own(bask)
    mass(bask, BASK_M + LEAVES_M, (0.012, 0.012, I_B))
    part(bask, "floor", "cyl", (0, 0, -BASK_H / 2 + 0.002), radius=BASK_R - 0.004, height=0.004, rgb=BASK_G,
         collide=False)
    ring(bask, "slat", BASK_R - 0.002, 0.004, BASK_H - 0.012, 0.0, 20, BASK_G, collide=False, gap=0.42)
    ring(bask, "rim", BASK_R - 0.002, 0.006, 0.010, BASK_H / 2 - 0.005, 20, BASK_G, collide=False, gap=1.03)
    ring(bask, "band", BASK_R - 0.002, 0.005, 0.008, -BASK_H / 2 + 0.006, 20, BASK_G, collide=False, gap=1.03)
    part(bask, "tape_mark", "box", (-BASK_R - 0.002, 0, BASK_H / 2 - 0.005), size=(0.002, 0.030, 0.011), rgb=RED,
         collide=False)
    part(bask, "tape_mark2", "box", (-BASK_R - 0.0015, 0, 0.0), size=(0.002, 0.012, BASK_H - 0.02), rgb=RED,
         collide=False)
    for i in range(6):  # torn leaf pieces lying against the basket wall and on its floor
        a = 2 * math.pi * i / 6 + 0.3
        rr = BASK_R - 0.014 if i % 2 == 0 else BASK_R - 0.026
        z = -BASK_H / 2 + 0.036 + 0.010 * (i % 3)
        part(bask, f"leaf_{i}", "box", (rr * math.cos(a), rr * math.sin(a), z), size=(0.005, 0.078, 0.060),
             yaw=a, rgb=LEAF if i % 2 == 0 else (0.52, 0.80, 0.25), collide=False)
        part(bask, f"rib_{i}", "box", ((rr + 0.0025) * math.cos(a), (rr + 0.0025) * math.sin(a),
                                        z), size=(0.003, 0.014, 0.056), yaw=a, rgb=(0.90, 0.96, 0.74), collide=False)

    # lid: its own white disc collides; raised cord hub with the socket the handle's stem docks in; brake button
    mass(lid, LID_M, (0.25 * LID_M * R_OUT ** 2, 0.25 * LID_M * R_OUT ** 2, 0.5 * LID_M * R_OUT ** 2))
    for p in Usd.PrimRange(stage.GetPrimAtPath(lid)):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics_material(str(p.GetPath()), pmat(0.8), stage=stage)
    ring(lid, "hub", (HUB_R0 + HUB_R1) / 2, HUB_R1 - HUB_R0, HUB_H, LID_T / 2 + HUB_H / 2, 12, GREEN, gap=1.10)
    part(lid, "rimband", "cyl", (0, 0, -LID_T / 2 + 0.002), radius=R_OUT + 0.001, height=0.004, rgb=GREEN,
         collide=False)
    part(lid, "brake", "cyl", (0.0, 0.065, LID_T / 2 + 0.003), radius=0.012, height=0.006, rgb=GREY, collide=False)

    # handle: the bar is the body's own box; stem under its middle
    mass(hnd, HANDLE_M, (2.0e-5, 2.0e-5, 2.0e-5), com=H_COM)
    for p in Usd.PrimRange(stage.GetPrimAtPath(hnd)):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics_material(str(p.GetPath()), pmat(1.0), stage=stage)
    part(hnd, "stem", "cyl", (0, 0, -(BAR_W / 2 + STEM_L / 2)), radius=STEM_R, height=STEM_L, rgb=DGREEN, mu=0.6)

    # joints: lid on the rim (straight up only, 0-4 cm), basket on the pivot
    jr = f"{root}/spinner_joints"
    UsdGeom.Xform.Define(stage, jr)
    d = UsdPhysics.Joint.Define(stage, f"{jr}/lid_on_rim")
    d.CreateBody0Rel().SetTargets([Sdf.Path(bowl)])
    d.CreateBody1Rel().SetTargets([Sdf.Path(lid)])
    d.CreateLocalPos0Attr(Gf.Vec3f(0, 0, BOWL_H / 2 + LID_T / 2))
    d.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    d.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    d.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    for dof in ("transX", "transY", "rotX", "rotY", "rotZ"):
        lim = UsdPhysics.LimitAPI.Apply(d.GetPrim(), dof)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)
    lim = UsdPhysics.LimitAPI.Apply(d.GetPrim(), "transZ")
    lim.CreateLowAttr(0.0)
    lim.CreateHighAttr(0.04)
    d.CreateExcludeFromArticulationAttr(True)
    rj = UsdPhysics.RevoluteJoint.Define(stage, f"{jr}/basket_pivot")
    rj.CreateBody0Rel().SetTargets([Sdf.Path(bowl)])
    rj.CreateBody1Rel().SetTargets([Sdf.Path(bask)])
    rj.CreateAxisAttr("Z")
    rj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, float(K_C[2] - B_C[2])))
    rj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    rj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    rj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    rj.CreateExcludeFromArticulationAttr(True)

    # the cord: a thin white tube mesh from the hub to the stem's foot, rewritten every control step
    mesh = UsdGeom.Mesh.Define(stage, f"{root}/pull_cord")
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 4))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(_cord_points(np.array([*C, TOP + LID_TOP]),
                                                               np.array([*C, TOP + LID_TOP + 0.001]))))
    bind_visual_material(str(mesh.GetPath()), look((0.22, 0.22, 0.24)), stage=stage)  # grey cord
    _P["cord_mesh"] = mesh
    print(f"[spinner] authored under {root}", flush=True)


def _cord_points(a, b, w=0.0022) -> np.ndarray:
    d = np.asarray(b, float) - np.asarray(a, float)
    n = np.linalg.norm(d)
    u = d / n if n > 1e-6 else np.array([0.0, 0.0, 1.0])
    e1 = np.cross(u, [1.0, 0.0, 0.0])
    if np.linalg.norm(e1) < 0.1:
        e1 = np.cross(u, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    off = [w * e1, w * e2, -w * e1, -w * e2]
    return np.array([np.asarray(a) + o for o in off] + [np.asarray(b) + o for o in off], dtype=np.float32)


def _wrap_cfg(spec, obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    orig = cfg.func
    base = getattr(orig, "__wrapped__", orig)

    def spawn(prim_path, c, *a, **k):
        prim = base(prim_path, c, *a, **k)
        _author(str(prim_path).rsplit("/", 1)[0])
        return prim

    cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_spinner_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) == OBJECTS[-1].name:
            _wrap_cfg(spec, obj)
        return obj

    task_object._spinner_patched = True
    scene.task_object = task_object


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _SceneFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneFinder())


# -- quaternions (xyzw) -------------------------------------------------------------------------------------------------
def quat_to_mat(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# -- the cord / spool / clutch model (every physics step) ---------------------------------------------------------------
_FRESH = dict(views=None, err=None, subs=0, step=0, nan=False, w=0.0, peak=0.0, s=0.0, s_max=0.0, tau=0.0,
              force=0.0, bowl0=None, slide=0.0, lift=0.0, tilt=0.0, lid_up=0.0, nohold=False, letgo=False,
              still=0, fmax=0.0, spin_s=0.0, held=None, on_lid=None, pull_steps=0, hold_steps=0, last_tcp=None, why="")
_M: dict = {}


def _reset_model() -> None:
    env = _M.get("env")
    _M.clear()
    _M.update(_FRESH)
    _M["env"] = env


_reset_model()


def _views(mgr):
    import warp as wp

    if _M["views"] is None:
        sv = mgr.get_physics_sim_view()
        if sv is None or "lid" not in _P:
            return None
        v = {k: sv.create_rigid_body_view(_P[k]) for k in ("bowl", "lid", "basket", "handle")}
        raw = v["lid"].get_transforms()
        _M["dev"] = str(getattr(raw, "device", "cuda:0"))
        _M["idx"] = wp.array(np.array([0], dtype=np.int32), dtype=wp.int32, device=_M["dev"])
        _M["views"] = v
    return _M["views"]


def _coulomb(t_max: float, w: float, inertia: float) -> float:
    return math.copysign(min(t_max, 0.5 * inertia * abs(w) / DT), w)


def cord_geometry(tl, th):
    """Cord exit (hub socket floor) and cord end (stem foot), world frame, from lid and handle poses."""
    eye = tl[:3] + quat_to_mat(tl[3:7]) @ EYE_L
    anc = th[:3] + quat_to_mat(th[3:7]) @ ANCH_L
    return eye, anc


def _drive(mgr) -> None:
    try:
        import warp as wp

        v = _views(mgr)
        if v is None:
            return
        _M["subs"] += 1
        T = {k: np.asarray(v[k].get_transforms().numpy(), dtype=np.float64).reshape(-1) for k in v}
        V = {k: np.asarray(v[k].get_velocities().numpy(), dtype=np.float64).reshape(-1) for k in v}
        if not all(np.isfinite(T[k]).all() and np.isfinite(V[k]).all() for k in T):
            _M["nan"] = True
            return
        tl, th, tb, tw = T["lid"], T["handle"], T["basket"], T["bowl"]
        Rl, Rh, Rw = quat_to_mat(tl[3:7]), quat_to_mat(th[3:7]), quat_to_mat(tw[3:7])
        eye, anc = cord_geometry(tl, th)
        hcom = th[:3] + Rh @ H_COM
        ve = V["lid"][:3] + np.cross(V["lid"][3:6], eye - tl[:3])
        va = V["handle"][:3] + np.cross(V["handle"][3:6], anc - hcom)
        d = anc - eye
        s = float(np.linalg.norm(d))
        u = d / s if s > 1e-5 else Rl[:, 2]
        vr = float(np.dot(va - ve, u))
        ax = Rw[:, 2]
        w = float(np.dot(V["basket"][3:6] - V["bowl"][3:6], ax))
        _M["w"], _M["s"] = w, s
        _M["s_max"] = max(_M["s_max"], s)
        _M["peak"] = max(_M["peak"], w)
        if w * RPM >= RPM_SPIN:
            _M["spin_s"] += DT
        # bowl / lid bookkeeping (latched)
        if _M["bowl0"] is None:
            _M["bowl0"] = tw[:3].copy()
        b0 = _M["bowl0"]
        _M["slide"] = max(_M["slide"], float(np.linalg.norm(tw[:2] - b0[:2])))
        _M["lift"] = max(_M["lift"], float(tw[2] - b0[2]))
        _M["tilt"] = max(_M["tilt"], math.degrees(math.acos(min(1.0, max(-1.0, ax[2])))))
        _M["lid_up"] = max(_M["lid_up"], float(np.dot(tl[:3] - tw[:3], ax) - (BOWL_H / 2 + LID_T / 2)))
        _M["bowl"], _M["lidT"], _M["handleT"] = tw.copy(), tl.copy(), th.copy()
        # cord tension: rewind spring + clutch (only while paying out faster than the basket turns) + end stop
        f = T_REWIND0 + K_REWIND * s
        drive = GEAR * vr
        tau = 0.0
        if s > S_ENGAGE and vr > 0.0 and drive > w:
            tau = min(TAU_MAX, I_B * (drive - w) / T_CATCH)
        f += tau * GEAR
        if s > S_MAX:
            f += K_STOP * (s - S_MAX) + C_STOP * max(vr, 0.0)
        _M["tau"], _M["force"] = tau, f
        _M["fmax"] = max(_M["fmax"], f)
        drag = _coulomb(DRAG_C0, w, I_B) + DRAG_C1 * w

        def push(view, force, torque):
            v[view].apply_forces_and_torques_at_position(
                wp.array(np.asarray([force], dtype=np.float32), dtype=wp.float32, device=_M["dev"]),
                wp.array(np.asarray([torque], dtype=np.float32), dtype=wp.float32, device=_M["dev"]), None,
                _M["idx"], True)

        F = f * u
        lcom = tl[:3]
        push("handle", -F, np.cross(anc - hcom, -F))
        push("lid", F, np.cross(eye - lcom, F) - tau * ax)
        push("basket", np.zeros(3), (tau - drag) * ax)
        push("bowl", np.zeros(3), drag * ax)
    except Exception as e:  # pragma: no cover
        if _M["err"] is None:
            _M["err"] = repr(e)
            print(f"[spinner] model error: {e!r}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_spinner_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        base_step(cls)
        _drive(cls)

    step._spinner_hooked = True
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def _origin():
    env = _M.get("env")
    if env is None:
        return np.zeros(3)
    try:
        return env.unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    except Exception:  # pragma: no cover
        return np.zeros(3)


def gripper_points() -> dict:
    """Per gripper: TCP, pad-tip midpoint and both fingertip links (env frame)."""
    env = _M.get("env")
    if env is None:
        return {}
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    base = env.unwrapped
    o = _origin()
    out = {}
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        pts = [np.asarray(tcp_world(rob), dtype=np.float64) - o, np.asarray(pad_tips_world(rob), dtype=np.float64) - o]
        lp = link_poses(rob)
        for t in ("tip_left", "tip_right"):
            pts.append(np.asarray(lp[t][:3, 3], dtype=np.float64) - o)
        out[k] = pts
    return out


def finger_gaps() -> dict:
    """Gap between each gripper's finger pads (m): 0 when they touch, 0.094 fully open."""
    env = _M.get("env")
    if env is None:
        return {}
    from yam_arena.constants import FINGER_JOINTS

    base = env.unwrapped
    out = {}
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        ids = rob.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = rob.data.joint_pos.torch[0, ids].cpu().numpy()
        out[k] = float(np.abs(q).sum())
    return out


# -- per control step checks --------------------------------------------------------------------------------------------
HOLD_R = 0.045  # a gripper's TCP this close to the bar centre with pads closed on the bar is holding the handle
GRIP_GAP = (0.006, 0.035)  # pad gap (m) when squeezing the 1.6 cm bar (not shut on air and not open)
PULLED = 0.05  # cord counts as pulled once 5 cm out
LET_GO = 0.03  # handle may only be let go with the cord under 3 cm out
LID_BAND = 0.010  # "on the lid": a fingertip touching the lid top (within 1 cm), inside its rim
STILL_V = 0.02  # m/s


def on_lid(pts, tl) -> bool:
    Rl = quat_to_mat(tl[3:7])
    for p in pts[1:]:
        q = Rl.T @ (np.asarray(p) - tl[:3])
        if math.hypot(q[0], q[1]) < R_OUT - 0.005 and -0.005 < q[2] - LID_T / 2 < LID_BAND:
            return True
    return False


def _check() -> None:
    tl, th = _M.get("lidT"), _M.get("handleT")
    if tl is None or th is None:
        return
    o = _origin()
    tl = np.r_[tl[:3] - o, tl[3:7]]
    th = np.r_[th[:3] - o, th[3:7]]
    g = gripper_points()
    gaps = finger_gaps()
    if not g or not gaps:
        return
    held = {k: float(np.linalg.norm(p[0] - th[:3])) < HOLD_R and GRIP_GAP[0] <= gaps[k] <= GRIP_GAP[1]
            for k, p in g.items()}
    lid = {k: (not held[k]) and on_lid(p, tl) for k, p in g.items()}
    _M["held"], _M["on_lid"] = held, lid
    s = _M["s"]
    if s > PULLED:
        _M["pull_steps"] += 1
        if any(lid.values()):
            _M["hold_steps"] += 1
        elif not _M["nohold"]:
            _M["nohold"] = True
            print(f"[spinner] cord pulled {s * 100:.1f} cm with no hand on the lid at step {_M['step']}", flush=True)
    if s > LET_GO and not any(held.values()) and not _M["letgo"]:
        _M["letgo"] = True
        print(f"[spinner] handle let go with the cord {s * 100:.1f} cm out at step {_M['step']}", flush=True)
    tcp = np.array([g["left_robot"][0], g["right_robot"][0]])
    if _M["last_tcp"] is not None:
        sp = np.linalg.norm(tcp - _M["last_tcp"], axis=1) * 20.0
        _M["still"] = _M["still"] + 1 if float(sp.max()) < STILL_V else 0
    _M["last_tcp"] = tcp
    # draw the cord
    mesh = _P.get("cord_mesh")
    if mesh is not None:
        from pxr import Vt

        eye, anc = cord_geometry(tl, th)
        mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(_cord_points(eye, anc)))


def _install_hooks() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_spinner_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _M["env"] = self._env
            res = base(self, *a, **k)
            _M["step"] += 1
            try:
                _check()
            except Exception as e:  # pragma: no cover
                if _M["err"] is None:
                    _M["err"] = repr(e)
                    print(f"[spinner] check error: {e!r}", flush=True)
            return res

        _task_step._spinner_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_model not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_model)


_install_hooks()


# -- scoring --------------------------------------------------------------------------------------------------------------
RPM_HALF, RPM_FULL = 150.0, 300.0
SLIDE_MAX, LIFT_MAX, TILT_MAX, LID_MAX = 0.03, 0.01, 5.0, 0.005
DOCK_S, DOCK_TILT = 0.008, 12.0
CLEAR = 0.10
STILL_STEPS = 20  # 1 s at 20 Hz


def spinner_clearance(pts, tw) -> float:
    """Smallest distance from any gripper point to the spinner: a 13 cm-radius cylinder from the counter up to 20 cm
    (above the docked handle), about the bowl's axis."""
    best = np.inf
    for p in pts:
        dr = max(0.0, math.hypot(p[0] - tw[0], p[1] - tw[1]) - R_OUT)
        dz = max(0.0, p[2] - (TOP + 0.20), TOP - p[2])
        best = min(best, math.hypot(dr, dz))
    return float(best)


def handle_docked(tl, th) -> bool:
    eye, anc = cord_geometry(tl, th)
    up = quat_to_mat(th[3:7])[:, 2]
    tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(np.dot(up, quat_to_mat(tl[3:7])[:, 2]))))))
    return bool(np.linalg.norm(anc - eye) < DOCK_S and tilt < DOCK_TILT)


def evaluate(st: dict) -> dict:
    """Pure scoring on the logged state (unit-testable)."""
    peak = st["peak"] * RPM
    bad = []
    if st["nan"]:
        bad.append("nan")
    if st["slide"] > SLIDE_MAX:
        bad.append("slid")
    if st["lift"] > LIFT_MAX or st["tilt"] > TILT_MAX:
        bad.append("lifted/tipped")
    if st["lid_up"] > LID_MAX:
        bad.append("lid lifted")
    if st["nohold"]:
        bad.append("pulled without a hand on the lid")
    if st["letgo"]:
        bad.append("handle let go with the cord out")
    full_end = st["docked"] and st["clear"] and st["still"] >= STILL_STEPS
    if bad:
        s = 0.0
    elif peak >= RPM_FULL and st["spin_s"] >= SPIN_S:
        s = 1.0 if full_end else 0.7
    elif peak >= RPM_HALF:
        s = 0.3
    else:
        s = 0.0
    return dict(score=s, peak_rpm=peak, spin_s=st["spin_s"], bad=bad, docked=st["docked"], clear=st["clear"], still=st["still"])


def _state(objects) -> dict:
    o = _origin()
    tw, tl, th = _M.get("bowl"), _M.get("lidT"), _M.get("handleT")
    if tw is None:
        b, l, h = objects[BOWL.name], objects[LID.name], objects[HANDLE.name]
        tw = np.r_[np.asarray(b.pos, float), np.asarray(b.quat_xyzw, float)]
        tl = np.r_[np.asarray(l.pos, float), np.asarray(l.quat_xyzw, float)]
        th = np.r_[np.asarray(h.pos, float), np.asarray(h.quat_xyzw, float)]
    else:
        tw, tl, th = (np.r_[t[:3] - o, t[3:7]] for t in (tw, tl, th))
    g = gripper_points()
    pts = [p for v in g.values() for p in v]
    st = {k: _M[k] for k in ("peak", "spin_s", "nan", "slide", "lift", "tilt", "lid_up", "nohold", "letgo",
                                "still")}
    st["docked"] = handle_docked(tl, th)
    st["clear"] = bool(pts) and spinner_clearance(pts, tw) >= CLEAR
    st["clearance"] = spinner_clearance(pts, tw) if pts else float("nan")
    return st


def evaluate_live(objects) -> dict:
    st = _state(objects)
    e = evaluate(st)
    e["clearance"] = st["clearance"]
    return e


def score(objects) -> float:
    try:
        return float(evaluate_live(objects)["score"])
    except Exception as e:  # pragma: no cover
        if _M["err"] is None:
            _M["err"] = repr(e)
            print(f"[spinner] score error: {e!r}", flush=True)
        return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def progress(objects) -> dict:
    try:
        e = evaluate_live(objects)
    except Exception:  # pragma: no cover
        return {}
    return {"peak_rpm": round(e["peak_rpm"], 1), "spin_s": round(e["spin_s"], 1), "rpm_now": round(_M["w"] * RPM, 1), "cord_out_cm": round(_M["s"] * 100, 1),
            "docked": float(e["docked"]), "clear": float(e["clear"]), "no_fault": float(not e["bad"])}


