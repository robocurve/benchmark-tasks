"""hhpatch: a used transdermal patch for a yam_arena task file, as a thin, floppy film.

A real patch is a thin flexible film. Here it is a strip of ``n`` thin rigid slats (0.8 mm thick) side by side across
its width, each joined to the next along its long edge by a hinge in the adhesive face, with a weak spring (the film's
own bending stiffness) and a little damping: it bends, sags and flops anywhere across its width, at 8 mm spacing, and
must be folded by moving one half over the other, as a real film is. The slats collide with each other (self-collision),
so the folded half lies on the other half rather than passing through it. The film counts as folded shut once every
slat of the folded half lies face down over its mirror slat of the other half (within ``align_m`` across the film,
within ``flip_deg`` of face to face, touching): then the adhesive faces stick, and from then on stiff drives hold every
hinge where it is (a stuck patch stays folded). The strip is one PhysX articulation (root link ``s0``), spawned the way
the accepted cable task's cablekit spawns its cable.

Importing this module hooks ``yam_arena.scene`` so a :class:`Patch` in ``OBJECTS`` is spawned as that articulation and
``success()/score()`` receive a :class:`PatchState` (an ObjectState for the whole patch plus each slat's pose). yam_arena
is not modified. Pure Python at import time.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState


@dataclass(frozen=True)
class Patch(Box):
    """A patch lying flat, adhesive face up, centred on ``pos``; its slats run along its own y and follow one another
    along its own x (own axes = world axes turned ``yaw`` about the vertical). The fold line is its own y axis through
    ``pos``: the slats on its own +x side are the half that folds over. ``size`` = (width along its own x, length along
    its own y, thickness)."""

    n: int = 6  # slats (even: the fold line falls between two)
    mass_total: float = 0.004
    friction: float = 0.9
    k_bend: float = 2e-5  # N m/rad per hinge: the film's own springiness (floppy: a slat sags under its own weight)
    damping: float = 2e-5  # N m s/rad per hinge
    k_stick: float = 0.05  # N m/rad: the adhesive holding a stuck patch's shape
    align_m: float = 0.009  # a folded slat this close (across the film and along it) to its mirror slat ...
    flip_deg: float = 25.0  # ... and within this of face to face, touching: the faces meet
    gap: float = 0.0003  # between neighbouring slats
    face_color: tuple = (0.92, 0.88, 0.80)  # the adhesive face (translucent beige)
    back_color: tuple = (0.80, 0.55, 0.30)  # the printed backing
    print_color: tuple = None  # visual only: a printed border, diagonal stripes and a name block (as on a Duragesic)
    border_m: float = 0.003  # width of the printed border

    @property
    def free(self) -> bool:  # not jittered by yam_arena's reset (it writes rigid poses only)
        return False

    @property
    def slat(self) -> tuple:
        """(width across, length along, thickness) of one slat."""
        return ((self.size[0] - (self.n - 1) * self.gap) / self.n, self.size[1], self.size[2])

    def slat_x(self, k: int) -> float:
        """Slat k's centre along the patch's own x, from its centre (flat)."""
        w = self.slat[0]
        return -self.size[0] / 2 + w / 2 + k * (w + self.gap)


@dataclass(frozen=True)
class PatchState(ObjectState):
    hinge_deg: float = 0.0  # angle between the first and the last slat's faces: 0 flat, 180 folded over
    stuck: bool = False
    slat_pos: np.ndarray = None  # (n, 3)
    slat_quat: np.ndarray = None  # (n, 4) xyzw
    a_pos: np.ndarray = None  # the middles of the unfolded half (own -x) and the folding half (own +x)
    b_pos: np.ndarray = None


_SPECS: dict = {}
STUCK: dict = {}  # name -> True once the faces have met
MODE: dict = {}  # name -> 0 free, 1 stuck (-1: rewrite the drives)


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat(m) -> np.ndarray:
    """xyzw quaternion of a rotation matrix."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / S, (m[0, 2] - m[2, 0]) / S, (m[1, 0] - m[0, 1]) / S, 0.25 * S]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * S, (m[0, 1] + m[1, 0]) / S, (m[0, 2] + m[2, 0]) / S, (m[2, 1] - m[1, 2]) / S]
    elif m[1, 1] > m[2, 2]:
        S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / S, 0.25 * S, (m[1, 2] + m[2, 1]) / S, (m[0, 2] - m[2, 0]) / S]
    else:
        S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / S, (m[1, 2] + m[2, 1]) / S, 0.25 * S, (m[1, 0] - m[0, 1]) / S]
    return np.array(q)


def folded_shut(spec: Patch, pos, quat) -> bool:
    """Every slat of the folding half lies face down on its mirror slat of the other half."""
    n, t = spec.n, spec.size[2]
    for i in range(n // 2, n):
        j = n - 1 - i
        Rj, Ri = _rot(quat[j]), _rot(quat[i])
        d = Rj.T @ (np.asarray(pos[i], float) - np.asarray(pos[j], float))
        if (Rj.T @ Ri)[2, 2] > -math.cos(math.radians(spec.flip_deg)):
            return False
        if abs(d[0]) > spec.align_m or abs(d[1]) > spec.align_m or not (0.0 < d[2] < 3 * t + 0.002):
            return False
    return True


def fold_angle(quat) -> float:
    za, zb = _rot(quat[0])[:, 2], _rot(quat[-1])[:, 2]
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(za @ zb))))))


def _print_marks(spec: Patch, k: int) -> list:
    """The printed marks on slat k, as (centre along the slat's x, centre along its y, size x, size y) in its frame: a
    border round the whole patch, diagonal stripes, and a name and strength block in the middle of each half. Visual
    only (no collision, no mass); drawn on both faces, as the print shows through the translucent film."""
    sw, L, _ = spec.slat
    b, n = spec.border_m, spec.n
    out = [(0.0, -L / 2 + b / 2, sw, b), (0.0, L / 2 - b / 2, sw, b)]  # the two ends of the border
    if k in (0, n - 1):  # the two long sides of the border
        out.append(((-1 if k == 0 else 1) * (sw - b) / 2, 0.0, b, L - 2 * b))
    else:  # a diagonal stripe: one square per slat, stepping along the patch
        for y in (-L / 2 + b + (k * 0.006) % (L - 2 * b - 0.004) + 0.002,):
            out.append((0.0, y, 0.6 * sw, 0.004))
    if k in (n // 4, n - 1 - n // 4):  # the name and strength block, one in the middle of each half
        out.append((0.0, 0.0, 0.8 * sw, 0.35 * L))
    return out


def _print_slat(stage, path, spec: Patch, k: int, look: str) -> None:
    from isaaclab.sim.utils import bind_visual_material
    from pxr import Gf, UsdGeom

    t, ink = spec.size[2], 4e-5
    for i, (x, y, sx, sy) in enumerate(_print_marks(spec, k)):
        for side, z in (("up", t / 2 + ink), ("dn", -t / 2 - ink)):
            g = UsdGeom.Cube.Define(stage, f"{path}/print_{side}{i}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            g.AddScaleOp().Set(Gf.Vec3f(sx, sy, ink))
            bind_visual_material(str(g.GetPrim().GetPath()), look, stage=stage)


def _spawn_patch(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Patch = _SPECS[cfg.patch_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    phys = f"{prim_path}/Looks/patch_physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction * 0.9)
    pm.func(phys, pm)
    looks = {}
    for k, c in (("face", spec.face_color), ("back", spec.back_color), ("print", spec.print_color or spec.back_color)):
        looks[k] = f"{prim_path}/Looks/{k}"
        lk = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(c), roughness=0.6)
        lk.func(looks[k], lk)
    sw, L, t = spec.slat
    paths = []
    for k in range(spec.n):  # s0 centred on the root frame (the root link)
        path = f"{prim_path}/s{k}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(spec.slat_x(k) - spec.slat_x(0), 0.0, 0.0))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass_total / spec.n)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateLinearDampingAttr(0.3)
        rb.CreateAngularDampingAttr(0.8)
        for part, zc, frac, look in (("geom", -0.2, 0.6, "back"), ("face", 0.3, 0.4, "face")):
            g = UsdGeom.Cube.Define(stage, f"{path}/{part}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, t * zc))
            g.AddScaleOp().Set(Gf.Vec3f(sw, L, t * frac))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.0006)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPrim().GetPath()), phys, stage=stage)
            bind_visual_material(str(g.GetPrim().GetPath()), looks[look], stage=stage)
        if spec.print_color is not None:
            _print_slat(stage, path, spec, k, looks["print"])
        paths.append(path)
    for k in range(spec.n - 1):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{paths[k + 1]}/bend{k}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[k])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[k + 1])])
        j.CreateAxisAttr("Y")
        # the hinge: the adhesive face's shared edge, in each slat's frame
        j.CreateLocalPos0Attr(Gf.Vec3f(sw / 2 + spec.gap / 2, 0, t / 2))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-sw / 2 - spec.gap / 2, 0, t / 2))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(-180.0)  # folding up and over towards s0 turns the next slat about -y
        j.CreateUpperLimitAttr(90.0)  # and the film can sag the other way
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(spec.k_bend * math.pi / 180)
        drv.CreateDampingAttr(spec.damping * math.pi / 180)
        drv.CreateTargetPositionAttr(0.0)
    root_link = stage.GetPrimAtPath(paths[0])
    UsdPhysics.ArticulationRootAPI.Apply(root_link)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root_link)
    art.CreateEnabledSelfCollisionsAttr(True)  # the folded half lies on the other half
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Patch):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PatchSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_patch)
        patch_name: str = ""

    class PatchObject(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(
                prim_path=self.prim_path, spawn=self.spawner_cfg,
                actuators={"bend": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=spec.k_bend,
                                                       damping=spec.damping, armature=2e-6)})
            return self._add_initial_pose_to_cfg(cfg)

    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    d = spec.slat_x(0)  # s0's centre from the patch's centre along its own x
    s0 = (spec.pos[0] + c * d, spec.pos[1] + s * d, spec.pos[2])
    return PatchObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                       spawner_cfg=PatchSpawnerCfg(patch_name=spec.name),
                       initial_pose=Pose(position_xyz=s0,
                                         rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _corners(p, q, size) -> np.ndarray:
    from hhkit import rot

    h = np.asarray(size) / 2
    c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    return p + c @ rot(q).T


def _state(spec: Patch, pos, quat, vel, angv, stuck) -> PatchState:
    n = spec.n
    pts = np.vstack([_corners(pos[k], quat[k], spec.slat) for k in range(n)])
    return PatchState(pos=pos.mean(0), quat_xyzw=quat[0], lin_vel=vel.mean(0), ang_vel=angv.mean(0), low=pts.min(0),
                      high=pts.max(0), hinge_deg=fold_angle(quat), stuck=stuck, slat_pos=pos, slat_quat=quat,
                      a_pos=pos[: n // 2].mean(0), b_pos=pos[n // 2:].mean(0))


def _patch_state(env, spec: Patch) -> PatchState:
    import torch

    base = env.unwrapped
    art = base.scene[spec.name]
    origin = _np(base.scene.env_origins)[0]
    names = list(art.body_names)
    idx = [names.index(f"s{k}") for k in range(spec.n)]
    pos = (_np(art.data.body_pos_w)[0] - origin)[idx]
    quat = _np(art.data.body_quat_w)[0][idx]  # Isaac Lab here gives (x, y, z, w)
    vel = _np(art.data.body_lin_vel_w)[0][idx]
    angv = _np(art.data.body_ang_vel_w)[0][idx]
    if not STUCK.get(spec.name) and folded_shut(spec, pos, quat):
        STUCK[spec.name] = True
    stuck = STUCK.get(spec.name, False)
    mode = 1 if stuck else 0  # 0 the film's own springiness, 1 stuck (the adhesive: every hinge held where it is)
    if MODE.get(spec.name, 0) != mode:
        MODE[spec.name] = mode
        nj = art.num_joints
        art.write_joint_stiffness_to_sim(torch.full((1, nj), spec.k_stick if mode else spec.k_bend, device=art.device))
        q = art.data.joint_pos.clone() if mode else torch.zeros((1, nj), device=art.device)
        art.set_joint_position_target(q)
    return _state(spec, pos, quat, vel, angv, stuck)


def offline_state(spec: Patch, hinge_deg: float = 0.0, pos=None, vel=(0.0, 0.0, 0.0)) -> PatchState:
    """A PatchState without the simulator (offline tests): the patch flat, or folded ``hinge_deg`` at its middle hinge
    (the folding half kept flat), its centre (flat: its middle) at ``pos`` (default: where it starts), turned
    ``spec.yaw``."""
    p0 = np.asarray(pos if pos is not None else spec.pos, float)
    n, t = spec.n, spec.size[2]
    th = math.radians(hinge_deg)
    c, s_ = math.cos(spec.yaw), math.sin(spec.yaw)
    rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    hinge = np.array([0.0, 0.0, t / 2])  # the middle hinge, in the patch's own frame (from its centre)
    ry = np.array([[math.cos(-th), 0.0, math.sin(-th)], [0.0, 1.0, 0.0], [-math.sin(-th), 0.0, math.cos(-th)]])
    q0 = np.array([0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)])
    qf = _quat(rz @ ry)
    P, Q = [], []
    for k in range(n):
        local = np.array([spec.slat_x(k), 0.0, 0.0])
        if k >= n // 2:
            local = hinge + ry @ (local - hinge)
        P.append(p0 + rz @ local)
        Q.append(qf if k >= n // 2 else q0)
    P, Q = np.array(P), np.array(Q)
    v = np.tile(np.asarray(vel, float), (n, 1))
    return _state(spec, P, Q, v, np.zeros((n, 3)), folded_shut(spec, P, Q))


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhpatch", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Patch):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(env, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(env, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _patch_state(env, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._hhpatch = task_object, object_states, True


class _Finder(importlib.abc.MetaPathFinder):
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


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
