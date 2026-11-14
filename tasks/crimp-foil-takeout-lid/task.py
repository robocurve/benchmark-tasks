"""On the station table, in front of the arms, a blue polyethylene cutting board
(305 x 457 x 12 mm, long side along y, centred at x 31.5 cm, y 0) lies fixed. On it stands a filled 2.25 lb oblong aluminium foil take-out pan
(about 222 x 156 mm over the rim, 43 mm deep, sloped walls; 400 g of rice inside, 425 g in all), its long side
along y, centred at x 28.5 cm, y 0. Its white foil-laminated board lid (213 x 152 mm) lies seated on the pan's flat top
flange. Round the lid, the pan's 8 mm rim lip stands turned straight up, 1.5 mm clear of the lid edge, all the way
round (24 sections: 6 per long side, 4 per short side, one across each cut corner). The lip bends where it is pushed
and stays where it is left. The robot must fold the lip down flat onto the lid all the way round so the lid is
crimped on, without lifting the lid off its seat, and then let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""


import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, ObjectState

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475
SCORE_OVER = "end"

INSTRUCTION = (
    "Crimp the board lid onto the filled foil take-out pan: fold the pan's rim down flat onto the lid all the way "
    "round, so the lid stays on when the pan is carried or turned on its side. Then let go."
)

# -- geometry (m). Pan frame: origin at the centre of the pan bottom, x across the pan, y along it, z up. ----------
BOARD_T = 0.012
BOARD_SIZE = (0.305, 0.457, BOARD_T)
PAN_X, PAN_Y = 0.285, 0.0
PAN_Z = TOP + BOARD_T  # pan bottom
PAN_DEPTH = 0.043  # 1 11/16 in
BOT_HX, BOT_HY = 0.060, 0.090  # bottom half sizes
FL_HX, FL_HY = 0.069, 0.098  # inner edge of the top flange (top of the sloped walls)
LID_HX, LID_HY = 0.076, 0.1065  # 152 x 213 mm board lid
LID_T = 0.0015
LID_MASS = 0.012
CLEAR = 0.0015  # lid edge to the lip's inner face
BEAD_W = 0.0015  # lid stop under each hinge (the lip's root), outward from the lip's inner face
HX, HY = LID_HX + CLEAR, LID_HY + CLEAR  # hinge line (lip inner face), half sizes: 0.0775, 0.108
CHAMFER = 0.012  # cut corners of the hinge line
HINGE_Z = PAN_DEPTH + LID_T + 0.0005  # lip hinge height in the pan frame (just above the lid top)
LIP_H = 0.008  # lip height (turned up)
LIP_T = 0.0015
LIP_MASS = 0.002
PAN_MASS = 0.425  # 25 g pan + 400 g rice
WALL_T = 0.001
FOIL = (0.80, 0.81, 0.83)
LID_COLOR = (0.96, 0.96, 0.94)
BOARD_COLOR = (0.30, 0.52, 0.80)

# Bending resistance of the rim lip: a yield moment per metre of rim, held as static joint friction (no springback).
YIELD_PER_M = 0.25  # N*m per m of rim: about 1 N pushed at the 8 mm lip's top edge for a 32 mm section
CORNER_FACTOR = 2.0  # cut corners gather the foil: twice as stiff per metre
LIP_DAMPING = 2e-4  # N*m*s/rad
LIP_ARMATURE = 2e-6
LIP_LOWER, LIP_UPPER = -60.0, 90.0  # deg; + folds inward over the lid, 90 = flat on the lid


def _edges():
    """(start, end, n_sections, kind) of the hinge-line polygon, counter-clockwise from the far long side."""
    B, A, c = HX, HY, CHAMFER
    pts = [((B, -A + c), (B, A - c), 6, "s"), ((B, A - c), (B - c, A), 1, "c"),
           ((B - c, A), (-B + c, A), 4, "s"), ((-B + c, A), (-B, A - c), 1, "c"),
           ((-B, A - c), (-B, -A + c), 6, "s"), ((-B, -A + c), (-B + c, -A), 1, "c"),
           ((-B + c, -A), (B - c, -A), 4, "s"), ((B - c, -A), (B, -A + c), 1, "c")]
    return pts


@dataclass(frozen=True)
class Lip:
    name: str  # joint name: lip_sNN (side) or lip_cNN (corner)
    p: tuple[float, float, float]  # hinge midpoint, pan frame
    n: tuple[float, float]  # outward normal (xy)
    length: float
    side: int  # 0 far (+x), 1 left (+y), 2 near (-x), 3 right (-y); corners: -1


def _lips() -> list[Lip]:
    out, k, side = [], 0, 0
    for (p0, p1, m, kind) in _edges():
        p0, p1 = np.array(p0), np.array(p1)
        t = (p1 - p0) / np.linalg.norm(p1 - p0)
        n = np.array([t[1], -t[0]])  # CCW polygon: outward normal
        L = float(np.linalg.norm(p1 - p0)) / m
        for i in range(m):
            mid = p0 + t * L * (i + 0.5)
            out.append(Lip(f"lip_{kind}{k:02d}", (float(mid[0]), float(mid[1]), HINGE_Z), (float(n[0]), float(n[1])),
                           L, side if kind == "s" else -1))
            k += 1
        if kind == "s":
            side += 1
    return out


LIPS = _lips()
RIM_LEN = sum(l.length for l in LIPS)
SIDE_LEN = [sum(l.length for l in LIPS if l.side == s) for s in range(4)]


@dataclass(frozen=True)
class FoilPan(Box):
    """The pan with its rim lip, simulated as one articulation (pan body + 24 hinged lip sections)."""

    @property
    def free(self) -> bool:  # not jittered by yam_arena's reset (it writes rigid poses only)
        return False


@dataclass(frozen=True)
class PanState(ObjectState):
    lip_angle: np.ndarray = None  # rad, per LIPS order; 0 upright, +pi/2 flat on the lid
    lip_pos: np.ndarray = None  # (N,3) hinge midpoints, world
    pan_pos: np.ndarray = None  # pan-frame origin (bottom centre), world
    pan_quat: np.ndarray = None  # xyzw


PAN = FoilPan("pan", pos=(PAN_X, PAN_Y, PAN_Z + 0.025), size=(2 * HX + 0.003, 2 * HY + 0.003, 0.05), color=FOIL)
@dataclass(frozen=True)
class CutLid(Box):
    """The board lid: a flat board with its four corners cut back (like the real lid's rounded corners)."""


LID_NOTCH = 0.012  # corner cut (m): the lid's rounded corner stays 2 mm inside the rim's cut corner, as on the sides

LID = CutLid("lid", pos=(PAN_X, PAN_Y, PAN_Z + PAN_DEPTH + LID_T / 2 + 0.0002), size=(2 * LID_HX, 2 * LID_HY, LID_T),
          color=LID_COLOR, mass=LID_MASS)
BOARD = Box("board", pos=(PAN_X + 0.03, 0.0, TOP + BOARD_T / 2), size=BOARD_SIZE, color=BOARD_COLOR)

OBJECTS = [BOARD, PAN, LID]


# -- Isaac Sim side: author the pan articulation -------------------------------------------------------------------
def _quat_wxyz_yaw(yaw: float):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def _spawn_pan(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    vis = f"{prim_path}/Looks/foil"
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(FOIL), roughness=0.25, metallic=0.6)
    look.func(vis, look)
    phys = f"{prim_path}/Looks/foil_physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.4, dynamic_friction=0.35, friction_combine_mode="min")
    pm.func(phys, pm)

    def body(path, pos, yaw, mass):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat_wxyz_yaw(yaw)
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*q))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateLinearDampingAttr(0.1)
        rb.CreateAngularDampingAttr(0.5)
        return prim

    def box(parent, name, center, size, rot_wxyz=(1.0, 0.0, 0.0, 0.0)):
        g = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3f(*map(float, center)))
        g.AddOrientOp().Set(Gf.Quatf(*map(float, rot_wxyz)))
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        prim = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0015)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), vis, stage=stage)
        return prim

    def qaxis(axis, ang):
        s = math.sin(ang / 2)
        return (math.cos(ang / 2), axis[0] * s, axis[1] * s, axis[2] * s)

    pan = f"{prim_path}/pan"
    body(pan, (0.0, 0.0, 0.0), 0.0, PAN_MASS)
    mp = UsdPhysics.MassAPI(stage.GetPrimAtPath(pan))
    mp.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.015))
    m = PAN_MASS
    mp.CreateDiagonalInertiaAttr(Gf.Vec3f(m / 12 * (0.19 ** 2 + 0.03 ** 2), m / 12 * (0.13 ** 2 + 0.03 ** 2),
                                          m / 12 * (0.19 ** 2 + 0.13 ** 2)))
    box(pan, "bottom", (0, 0, WALL_T / 2), (2 * BOT_HX, 2 * BOT_HY, WALL_T))
    # sloped walls
    dz = PAN_DEPTH
    for sgn in (1, -1):
        dx = FL_HX - BOT_HX
        ang = math.atan2(dx, dz)
        ln = math.hypot(dx, dz)
        box(pan, f"wall_x{'p' if sgn > 0 else 'm'}", (sgn * (BOT_HX + FL_HX) / 2, 0, dz / 2),
            (WALL_T, 2 * FL_HY, ln), qaxis((0, 1, 0), sgn * ang))
        dy = FL_HY - BOT_HY
        ang = math.atan2(dy, dz)
        ln = math.hypot(dy, dz)
        box(pan, f"wall_y{'p' if sgn > 0 else 'm'}", (0, sgn * (BOT_HY + FL_HY) / 2, dz / 2),
            (2 * FL_HX, WALL_T, ln), qaxis((1, 0, 0), -sgn * ang))
        # flat top flange (the lid seat) out to the hinge line, and the bead under the hinge
        box(pan, f"flange_x{'p' if sgn > 0 else 'm'}", (sgn * (FL_HX + HX + LIP_T) / 2, 0, dz - WALL_T / 2),
            (HX + LIP_T - FL_HX, 2 * (HY - CHAMFER), WALL_T))
        box(pan, f"flange_y{'p' if sgn > 0 else 'm'}", (0, sgn * (FL_HY + HY + LIP_T) / 2, dz - WALL_T / 2),
            (2 * (HX - CHAMFER), HY + LIP_T - FL_HY, WALL_T))
    for i, lip in enumerate(LIPS):
        nx, ny = lip.n
        yaw = math.atan2(-nx, ny)  # lip frame: x along the rim, y outward, z up
        cx, cy = lip.p[0] + nx * BEAD_W / 2, lip.p[1] + ny * BEAD_W / 2
        bh = HINGE_Z - dz + WALL_T  # bead (the rim's root fold) from flange underside up to the hinge: the lid stop
        box(pan, f"bead_{i:02d}", (cx, cy, dz - WALL_T + bh / 2), (lip.length + 0.0005, BEAD_W, bh),
            _quat_wxyz_yaw(yaw))
        if lip.side == -1:  # flange patch under the cut corner
            box(pan, f"cflange_{i:02d}", (lip.p[0] - nx * 0.006, lip.p[1] - ny * 0.006, dz - WALL_T / 2),
                (lip.length, 0.012, WALL_T), _quat_wxyz_yaw(yaw))

    for lip in LIPS:
        nx, ny = lip.n
        yaw = math.atan2(-nx, ny)
        lp = f"{prim_path}/{lip.name}_b"
        body(lp, lip.p, yaw, LIP_MASS)
        box(lp, "geom", (0.0, LIP_T / 2, LIP_H / 2), (lip.length - 0.0004, LIP_T, LIP_H))
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{lp}/{lip.name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(pan)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(lp)])
        j.CreateAxisAttr("X")
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, lip.p)))
        j.CreateLocalRot0Attr(Gf.Quatf(*map(float, _quat_wxyz_yaw(yaw))))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLowerLimitAttr(LIP_LOWER)
        j.CreateUpperLimitAttr(LIP_UPPER)

    root_link = stage.GetPrimAtPath(pan)
    UsdPhysics.ArticulationRootAPI.Apply(root_link)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root_link)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: FoilPan):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _register_module()
    side_len = next(l.length for l in LIPS if l.side >= 0 and l.side in (0, 2))
    short_len = next(l.length for l in LIPS if l.side in (1, 3))
    corner_len = next(l.length for l in LIPS if l.side == -1)

    @configclass
    class PanSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_pan)

    def act(expr, length, factor=1.0):
        tau = YIELD_PER_M * length * factor
        return ImplicitActuatorCfg(joint_names_expr=expr, stiffness=0.0, damping=LIP_DAMPING, armature=LIP_ARMATURE,
                                   friction=tau, dynamic_friction=0.8 * tau)

    long_names = [l.name for l in LIPS if l.side in (0, 2)]
    short_names = [l.name for l in LIPS if l.side in (1, 3)]
    corner_names = [l.name for l in LIPS if l.side == -1]

    class PanObject(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(
                prim_path=self.prim_path,
                spawn=self.spawner_cfg,
                actuators={"long": act(long_names, side_len), "short": act(short_names, short_len),
                           "corner": act(corner_names, corner_len, CORNER_FACTOR)},
            )
            return self._add_initial_pose_to_cfg(cfg)

    return PanObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=PanSpawnerCfg(),
                     initial_pose=Pose(position_xyz=(PAN_X, PAN_Y, PAN_Z + 0.0003), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _spawn_lid(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(LID_MASS)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(0.5)
    vis = f"{prim_path}/Looks/board"
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(LID_COLOR), roughness=0.6)
    look.func(vis, look)
    phys = f"{prim_path}/Looks/board_physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0], dynamic_friction=GRIPPY_FRICTION[1])
    pm.func(phys, pm)
    for name, size in (("long", (2 * LID_HX - 2 * LID_NOTCH, 2 * LID_HY, LID_T)),
                       ("wide", (2 * LID_HX, 2 * LID_HY - 2 * LID_NOTCH, LID_T))):
        g = UsdGeom.Cube.Define(stage, f"{prim_path}/{name}")
        g.CreateSizeAttr(1.0)
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        gp = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(gp)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(gp)
        cp.CreateContactOffsetAttr(0.003)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(gp.GetPath()), phys, stage=stage)
        bind_visual_material(str(gp.GetPath()), vis, stage=stage)
    return prim


def _lid_object(spec: CutLid):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _register_module()

    @configclass
    class LidSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(_spawn_lid)

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=LidSpawnerCfg(),
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:  # configclass needs this module registered (task files load unregistered)
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _quat_rot(q_xyzw, v):
    x, y, z, w = q_xyzw
    u = np.array([x, y, z])
    v = np.asarray(v, dtype=float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _pan_state(env) -> PanState:
    base = env.unwrapped
    art = base.scene[PAN.name]
    origin = base.scene.env_origins[0]

    def t(x):
        return getattr(x, "torch", x)

    to_np = lambda a: a.detach().cpu().numpy().astype(np.float64)  # noqa: E731
    bnames = list(art.body_names)
    jnames = list(art.joint_names)
    bp = to_np(t(art.data.body_pos_w)[0] - origin)
    bq = to_np(t(art.data.body_quat_w)[0])
    bv = to_np(t(art.data.body_lin_vel_w)[0])
    bw = to_np(t(art.data.body_ang_vel_w)[0])
    jp = to_np(t(art.data.joint_pos)[0])
    ip = bnames.index("pan")
    q = bq[ip]
    # Isaac Lab 3 quaternions are xyzw
    ang = np.array([jp[jnames.index(l.name)] for l in LIPS])
    lp = np.array([bp[bnames.index(f"{l.name}_b")] for l in LIPS])
    pts = np.vstack([lp, bp[ip][None]])
    return PanState(pos=bp[ip] + _quat_rot(q, [0, 0, PAN_DEPTH / 2]), quat_xyzw=q, lin_vel=bv[ip], ang_vel=bw[ip],
                    low=pts.min(0) - [0, 0, 0], high=pts.max(0) + [0, 0, LIP_H], lip_angle=ang, lip_pos=lp,
                    pan_pos=bp[ip], pan_quat=q)


def _patch_scene(scene) -> None:
    if getattr(scene, "_foilpan_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, FoilPan):
            return _arena_object(spec)
        if isinstance(spec, CutLid):
            return _lid_object(spec)
        return orig_task_object(spec)

    def object_states(env, names):
        plain = [n for n in names if n != PAN.name]
        out = orig_states(env, plain) if plain else {}
        if PAN.name in names:
            out[PAN.name] = _pan_state(env)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._foilpan_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _foilpan = True

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


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_foilpan", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- grippers (score() needs them for the release check) ----------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_foilpan_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._foilpan_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- scoring ---------------------------------------------------------------------------------------------------------
FOLD_DEG = 80.0  # a lip section is folded when it is within 10 deg of flat: its edge within 1.4 mm of the lid
LID_SHIFT_MAX = 0.004  # lid centre within 4 mm of the pan centre ...
LID_LIFT_MAX = 0.002  # ... its centre within 2 mm of the seated height ...
LID_TILT_MAX = 3.0  # ... and tilted less than 3 deg relative to the pan
PAN_TILT_MAX = 10.0
RELEASE_DIST = 0.10  # each gripper TCP at least 10 cm from the lid's top surface (hands let go and moved clear)


def _inv_rot(q, v):
    x, y, z, w = q
    return _quat_rot((-x, -y, -z, w), v)


def _lid_rel(o) -> tuple[np.ndarray, float]:
    pan, lid = o[PAN.name], o[LID.name]
    rel = _inv_rot(pan.pan_quat, lid.pos - pan.pan_pos)
    zl = _inv_rot(pan.pan_quat, _quat_rot(lid.quat_xyzw, [0, 0, 1]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(zl[2])))))
    return rel, tilt


def _gate(o) -> dict:
    pan = o[PAN.name]
    finite = all(np.all(np.isfinite(a)) for a in (pan.lip_angle, pan.pan_pos, o[LID.name].pos))
    if not finite:
        return {"finite": False}
    rel, tilt = _lid_rel(o)
    seated = (math.hypot(rel[0], rel[1]) <= LID_SHIFT_MAX and abs(rel[2] - (PAN_DEPTH + LID_T / 2)) <= LID_LIFT_MAX
              and tilt <= LID_TILT_MAX)
    zp = _quat_rot(pan.pan_quat, [0, 0, 1])
    upright = math.degrees(math.acos(max(-1.0, min(1.0, float(zp[2]))))) <= PAN_TILT_MAX
    bx, by = BOARD.pos[0], BOARD.pos[1]
    on_board = abs(pan.pan_pos[0] - bx) <= BOARD_SIZE[0] / 2 and abs(pan.pan_pos[1] - by) <= BOARD_SIZE[1] / 2
    released = True
    for g in grippers():
        r = _inv_rot(pan.pan_quat, g - pan.pan_pos)
        c = np.array([np.clip(r[0], -HX, HX), np.clip(r[1], -HY, HY), PAN_DEPTH + LID_T])
        if np.linalg.norm(r - c) < RELEASE_DIST:
            released = False
    return {"finite": True, "seated": seated, "upright": upright, "on_board": on_board, "released": released}


def _folded(o) -> np.ndarray:
    return o[PAN.name].lip_angle >= math.radians(FOLD_DEG)


def fold_fractions(o) -> tuple[float, list[float]]:
    f = _folded(o)
    lens = np.array([l.length for l in LIPS])
    total = float((f * lens).sum() / RIM_LEN)
    sides = [float(sum(l.length for l, k in zip(LIPS, f) if k and l.side == s) / SIDE_LEN[s]) for s in range(4)]
    return total, sides


def score(objects) -> float:
    g = _gate(objects)
    if not g.get("finite") or not all(g.values()):
        return 0.0
    total, sides = fold_fractions(objects)
    if total >= 1.0 - 1e-9:
        return 1.0
    if total < 0.25:
        return 0.0
    if min(sides) < 0.5:
        return 0.25
    return 0.75 if total >= 0.9 else 0.5


def success(objects) -> bool:
    if os.environ.get("FOIL_LIFT"):  # lift-by-lid evidence runs only: keep the episode going after the crimp
        return False
    return score(objects) >= 1.0 and objects[PAN.name].resting(0.01, 0.2) and objects[LID.name].resting(0.01, 0.2)


def distance(objects) -> float:
    g = _gate(objects)
    if not g.get("finite"):
        return 1.0
    total, _ = fold_fractions(objects)
    return float(1.0 - total) + (0.0 if all(g.values()) else 1.0)


def progress(objects) -> dict:
    g = _gate(objects)
    if not g.get("finite"):
        return {"finite": 0}
    total, sides = fold_fractions(objects)
    return {"lid_seated": int(g["seated"]), "pan_upright_on_board": int(g["upright"] and g["on_board"]),
            "folded_fraction": round(total, 3), "every_side_half_folded": int(min(sides) >= 0.5),
            "all_folded": int(total >= 1.0 - 1e-9), "released": int(g["released"])}


VARIATION = {"obj:pan": {"dx": 0.01, "dy": 0.01, "yaw": 0.05}, "obj:lid": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}

TWIN_EXTRA = [
    {"name": "rim_lip", "kind": "joint", "real": "the pan's own foil rim, turned up 8 mm",
     "note": "24 sections hinged on the hinge line, held by friction (no springback)"},
]


# -- scripted demo (reads the simulated pan pose and lip angles to plan) ---------------------------------------------
# grip yaw per lip section (offline IK on the URDF, straight-down grip): finger plates edge-on along the push
PUSH_OUT, PUSH_IN, PUSH_Z = 0.022, 0.020, 0.0165  # TCP offsets from the lip hinge (fingertips 13 mm below the TCP)
PRESS_IN, PRESS_Z = 0.0075, 0.004
LIP_YAW = [(math.pi / 4 if l.n[0] * l.n[1] > 0 else -math.pi / 4) if l.side == -1 else
           (-math.pi / 2 if l.side == 1 else math.pi / 2 if l.side == 3 else 0.0) for l in LIPS]
for _i, _l in enumerate(LIPS):  # sections next to the midline: wrist camera turned away from the holding hand
    if _l.side in (0, 2) and abs(_l.p[1]) < 0.03:
        LIP_YAW[_i] = math.copysign(math.pi / 3, _l.p[1])

def _wrap_yaw(a: float) -> float:
    while a > math.pi / 2:
        a -= math.pi
    while a < -math.pi / 2:
        a += math.pi
    return a


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    cheat = os.environ.get("FOIL_CHEAT", "")
    L, R = ctx.left, ctx.right
    F = FINGER_CLOSED

    def log(tag):
        o = ctx.states()
        total, sides = fold_fractions(o)
        rel, tilt = _lid_rel(o)
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(o):.2f} folded={total:.2f} sides="
              f"{[round(s, 2) for s in sides]} gate={_gate(o)} lid_rel=({rel[0]*1000:.1f},{rel[1]*1000:.1f},"
              f"{(rel[2]-PAN_DEPTH-LID_T/2)*1000:.1f})mm tilt={tilt:.1f} pan=({o['pan'].pan_pos[0]:.3f},"
              f"{o['pan'].pan_pos[1]:.3f}) angles={np.round(np.degrees(o['pan'].lip_angle)).astype(int).tolist()}",
              flush=True)

    def geo(i):
        o = ctx.states()
        pan = o[PAN.name]
        p = pan.lip_pos[i]
        n = _quat_rot(pan.pan_quat, [LIPS[i].n[0], LIPS[i].n[1], 0.0])
        n = n / (np.linalg.norm(n[:2]) + 1e-12)
        n[2] = 0.0
        lid_top = pan.pan_pos[2] + PAN_DEPTH + LID_T
        return p, n, lid_top, pan

    dbg = bool(os.environ.get("FOIL_DEBUG"))

    def trace(arm, i, tag, p):
        if not dbg:
            return
        from yam_arena.runtime import pad_tips_world

        env = ctx.emb._env.unwrapped
        org = env.scene.env_origins[0].detach().cpu().numpy()
        tip = np.asarray(pad_tips_world(env.scene[f"{arm.name}_robot"])) - org
        o = ctx.states()
        rel = tip - p
        print(f"  [{arm.name} lip {i} {tag}] cmd={np.round(arm.tcp(), 4).tolist()} tip-hinge(mm)=("
              f"{1000 * float(rel @ n_of(i)):.1f} out,{1000 * rel[2]:.1f} up) angle="
              f"{math.degrees(o[PAN.name].lip_angle[i]):.0f} pan=({1000 * o[PAN.name].pan_pos[0]:.1f},"
              f"{1000 * o[PAN.name].pan_pos[1]:.1f})mm", flush=True)

    def n_of(i):
        return geo(i)[1]

    def crimp(arm, i, first=False, retry=0):
        """Push section i over from outside, then press it flat. On a retry of a section that is already well over,
        skip the push and only press, a little lower each time (read back from the section's hinge angle)."""
        p, n, lid_top, pan = geo(i)
        zh = p[2]
        yaw = LIP_YAW[i]
        turned = abs(abs(yaw) - math.pi / 3) < 1e-6  # wrist turned 60 deg next to the midline: it sits higher
        near_mid = turned and LIPS[i].side == 2  # near side, wrist turned: the press lands ~4 mm high there
        # push height: the turned wrist sags ~4 mm low, except the right arm on the near side, which rides ~3 mm high;
        # a retry that has to push again (section still up or bent outward) comes in lower each time
        pz = PUSH_Z + (0.004 if turned else 0.0) - (0.007 if near_mid and arm.name == "right" else 0.0) - 0.003 * retry
        zp = PRESS_Z - (0.004 if near_mid else 0.0) - 0.003 * retry
        out = p + n * PUSH_OUT
        b = p - n * PRESS_IN
        press_only = retry > 0 and math.degrees(pan.lip_angle[i]) >= 45
        if press_only:
            yield from arm.move((b[0], b[1], zh + 0.07), yaw=yaw, finger=F, duration=1.2)
            yield from arm.move((b[0], b[1], zh + 0.03), yaw=yaw, duration=0.4)
        else:
            if first:  # long move from elsewhere: travel high so the lagging fingertips can't sweep through a lip
                yield from arm.move((out[0], out[1], zh + 0.07), yaw=yaw, finger=F, duration=1.2)
            yield from arm.move((out[0], out[1], zh + 0.035), yaw=yaw, finger=F, duration=0.5)
            yield from arm.move((out[0], out[1], zh + pz), yaw=yaw, duration=0.35)
            trace(arm, i, "down", p)
            a = p - n * PUSH_IN
            yield from arm.move((a[0], a[1], zh + pz), yaw=yaw, duration=0.5)
            trace(arm, i, "pushed", p)
            p, n, _, _ = geo(i)  # the pan may have shifted under the push
            b = p - n * PRESS_IN
        for _ in range(3):
            yield from arm.move((b[0], b[1], zh + zp), yaw=yaw, duration=0.3)
            trace(arm, i, "pressed", p)
            if math.degrees(ctx.states()[PAN.name].lip_angle[i]) >= FOLD_DEG + 4:
                break
            yield from arm.move((b[0], b[1], zh + 0.02), yaw=yaw, duration=0.2)
        yield from arm.move((b[0], b[1], zh + 0.035), yaw=yaw, duration=0.25)

    def hold(arm, sgn, press=0.015):
        o = ctx.states()
        pan = o[PAN.name]
        c = pan.pan_pos + _quat_rot(pan.pan_quat, [0.0, sgn * 0.093, 0.0])
        lid_top = pan.pan_pos[2] + PAN_DEPTH + LID_T
        yield from arm.move((c[0], c[1], lid_top + 0.05), yaw=0.0, finger=F, duration=1.0)
        yield from arm.move((c[0], c[1], lid_top + 0.016), yaw=0.0, duration=0.5)
        yield from arm.move((c[0], c[1], lid_top + 0.013 - press), yaw=0.0, duration=1.0)

    def unhold(arm):
        """Leave the lid by sliding up and in toward the pan centre, so the fingertips come off the lid sideways."""
        o = ctx.states()
        pan = o[PAN.name]
        p = arm.tcp()
        lid_top = pan.pan_pos[2] + PAN_DEPTH + LID_T
        d = pan.pan_pos[:2] - p[:2]
        d = d / (np.linalg.norm(d) + 1e-9)
        yield from arm.move((p[0] + 0.04 * d[0], p[1] + 0.04 * d[1], lid_top + 0.05), yaw=arm._yaw, duration=1.2)

    def lift(arm, dz=0.06, duration=0.5):
        p = arm.tcp()
        yield from arm.move((p[0], p[1], p[2] + dz), yaw=arm._yaw, duration=duration)

    def park(arm, side):
        yield from arm.move((0.18, side * 0.30, TOP + 0.20), yaw=0.0, duration=1.2)

    def order(sgn):  # from the midline of the far side round the arm's own end to the midline of the near side
        idx = [i for i, l in enumerate(LIPS) if sgn * l.p[1] > 0]
        idx = sorted(idx, key=lambda i: sgn * math.atan2(LIPS[i].p[1], LIPS[i].p[0]))
        return idx

    tries = {}

    def crimp_all(arm, idxs, redo=False):
        for k, i in enumerate(idxs):
            near = k > 0 and abs(idxs[k - 1] - i) in (1, len(LIPS) - 1)  # next section along the rim
            r = 0
            if redo:
                tries[i] = r = tries.get(i, 0) + 1
            yield from crimp(arm, i, first=(k == 0) or (redo and not near), retry=r)

    def lift_test():
        yield from _lift_test_gen(ctx, log)

    left_set, right_set = order(+1), order(-1)
    log("start")
    if cheat == "probe":  # debug: crimp three lips with the right hand holding
        yield from ctx.both(hold(R, -1), L.wait(0.1))
        yield from crimp_all(L, left_set[:3] + [i for i in left_set if LIPS[i].side == 1][:1])
        log("probe end")
        return
    if cheat == "probe15":  # debug: the right hand crimps the near-side midline section with the left holding
        yield from ctx.both(hold(L, +1), R.wait(0.1))
        yield from crimp_all(R, [15])
        for _ in range(2):
            if math.degrees(ctx.states()[PAN.name].lip_angle[15]) >= FOLD_DEG + 2:
                break
            yield from crimp_all(R, [15], redo=True)
        log("probe end")
        return
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return
    if cheat == "press":  # press straight down round the rim, never pushing the lip inward
        def press_ring(arm, idxs):
            for k, i in enumerate(idxs):
                p, n, _, _ = geo(i)
                yaw = LIP_YAW[i]
                q = p + n * 0.002
                yield from arm.move((q[0], q[1], p[2] + 0.035), yaw=yaw, finger=F, duration=1.2 if k == 0 else 0.5)
                yield from arm.move((q[0], q[1], p[2] + 0.004), yaw=yaw, duration=0.4)
                yield from arm.move((q[0], q[1], p[2] + 0.035), yaw=yaw, duration=0.3)
        yield from ctx.both(press_ring(L, left_set), press_ring(R, right_set))
        yield from ctx.both(park(L, 1), park(R, -1))
        yield from L.wait(2.0)
        log("end")
        return
    if cheat == "corners":  # fold only the four cut corners
        lc = [i for i in left_set if LIPS[i].side == -1]
        rc = [i for i in right_set if LIPS[i].side == -1]
        yield from hold(R, -1)
        yield from crimp_all(L, lc)
        yield from lift(L, 0.04, 0.3)
        yield from hold(L, +1)
        yield from unhold(R)
        yield from crimp_all(R, rc)
        yield from ctx.both(unhold(L), lift(R))
        yield from ctx.both(park(L, 1), park(R, -1))
        yield from L.wait(2.0)
        log("end")
        if os.environ.get("FOIL_LIFT"):
            yield from lift_test()
        return
    if cheat == "onearm":  # left arm only, nobody holds the pan: crimp every lip the left arm can reach
        yield from crimp_all(L, left_set)
        log("left half done")
        yield from lift(L)
        yield from park(L, 1)
        yield from L.wait(2.0)
        log("end")
        return
    skip = set()
    if cheat == "tier50":  # leave two far-side sections and one right-end section open (every side >= 50%)
        skip = {0, 1, 21}
    if cheat == "tier75":  # leave one long-side section open
        skip = {1}
    lset = [i for i in left_set if i not in skip]
    rset = [i for i in right_set if i not in skip]

    # right hand keeps the lid seated (and the pan from sliding) while the left crimps its half, then swap
    yield from ctx.both(hold(R, -1), L.wait(0.1))
    log("right holding")
    yield from crimp_all(L, lset)
    log("left half crimped")
    yield from lift(L, 0.04, 0.3)
    yield from hold(L, +1)
    yield from unhold(R)
    log("left holding")
    yield from crimp_all(R, rset)
    log("right half crimped")
    # second pass on anything that sprang short of the mark
    for _ in range(2):
        o = ctx.states()
        bad = [i for i in rset if math.degrees(o[PAN.name].lip_angle[i]) < FOLD_DEG + 2]
        if not bad:
            break
        log(f"redo right {bad}")
        yield from crimp_all(R, bad, redo=True)
    yield from lift(R)
    yield from hold(R, -1)
    yield from unhold(L)
    for _ in range(2):
        o = ctx.states()
        bad = [i for i in lset if math.degrees(o[PAN.name].lip_angle[i]) < FOLD_DEG + 2]
        if not bad:
            break
        log(f"redo left {bad}")
        yield from crimp_all(L, bad, redo=True)
    if cheat == "hold":  # finish, then keep both grippers pressing on the lid
        yield from hold(L, +1)
        yield from L.wait(3.0)
        log("end (holding)")
        return
    yield from ctx.both(lift(L), unhold(R))
    yield from ctx.both(park(L, 1), park(R, -1))
    yield from L.wait(1.5)
    log("end")
    if os.environ.get("FOIL_LIFT"):
        yield from lift_test()


def _lift_test_gen(ctx, log, factor: float = 1.5):
    """Evidence only (never part of a scored run): pull the lid straight up with factor x the weight of pan + lid
    and see whether the crimped rim carries the pan up with it (the real-world 'lifted by the lid' check)."""
    import torch

    env = ctx.emb._env.unwrapped
    lid = env.scene[LID.name]
    f = factor * (PAN_MASS + LID_MASS) * 9.81
    forces = torch.zeros((env.num_envs, 1, 3), device=env.device)
    forces[..., 2] = f
    z0 = ctx.states()[PAN.name].pan_pos[2]
    lid.permanent_wrench_composer.set_forces_and_torques(forces=forces, torques=torch.zeros_like(forces),
                                                         is_global=True)
    for k in range(40):
        yield
    o = ctx.states()
    rel, tilt = _lid_rel(o)
    print(f"[lift test] force={f:.2f} N pan rose {1000 * (o[PAN.name].pan_pos[2] - z0):.1f} mm, lid rel z "
          f"{1000 * (rel[2] - PAN_DEPTH - LID_T / 2):.1f} mm, lid tilt {tilt:.1f} deg", flush=True)
    log("after lift test")
