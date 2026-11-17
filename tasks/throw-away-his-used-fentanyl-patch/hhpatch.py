"""hhpatch: a used transdermal patch for a yam_arena task file, as two thin rigid halves joined by a hinge.

A real patch is a thin flexible film. Here it is two rigid halves (``half_a`` and ``half_b``, each half the patch)
joined along their shared edge by a free hinge that lies in the adhesive face, so ``half_b`` can be folded over onto
``half_a`` until the two adhesive faces meet (the hinge's 180 degree limit; the halves do not collide with each
other). Once folded within ``180 - stick_deg`` of shut the faces stick: from then on a joint drive holds the hinge
fully shut (the adhesive), as a folded patch stays shut. The pair is one PhysX
articulation (root link ``half_a``), spawned the way the accepted cable task's cablekit spawns its cable.

Importing this module hooks ``yam_arena.scene`` so a :class:`Patch` in ``OBJECTS`` is spawned as that articulation and
``success()/score()`` receive a :class:`PatchState` (an ObjectState for the whole patch plus the hinge angle and each
half's pose). yam_arena is not modified. Pure Python at import time.
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
    """A patch lying flat, adhesive face up, its hinge (the fold line) through ``pos`` along its own y; ``half_a`` on
    its own -x side, ``half_b`` on its own +x side (own axes = world axes turned ``yaw`` about the vertical).
    ``size`` = (whole width along its own x, length along its own y, thickness)."""

    mass_half: float = 0.002
    friction: float = 0.9
    stick_deg: float = 172.0  # folded at least this far: the adhesive faces meet and stick
    k_stick: float = 0.05  # N m/rad: the adhesive holding a stuck patch shut (a drive to fully folded)
    damping: float = 2e-5  # N m s/rad, the film's own resistance to folding (tiny; otherwise the hinge is free)
    face_color: tuple = (0.92, 0.88, 0.80)  # the adhesive face (translucent beige)
    back_color: tuple = (0.80, 0.55, 0.30)  # the printed backing

    @property
    def free(self) -> bool:  # not jittered by yam_arena's reset (it writes rigid poses only)
        return False

    @property
    def half(self) -> tuple:
        return (self.size[0] / 2, self.size[1], self.size[2])


@dataclass(frozen=True)
class PatchState(ObjectState):
    hinge_deg: float = 0.0  # 0 flat, 180 folded shut (adhesive faces together)
    stuck: bool = False
    a_pos: np.ndarray = None
    b_pos: np.ndarray = None
    a_quat: np.ndarray = None
    b_quat: np.ndarray = None
    a_vel: np.ndarray = None
    b_vel: np.ndarray = None


_SPECS: dict = {}
STUCK: dict = {}  # name -> True once the faces have met
MODE: dict = {}  # name -> 0 free, 1 stuck (-1: rewrite the drive)


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
    for k, c in (("face", spec.face_color), ("back", spec.back_color)):
        looks[k] = f"{prim_path}/Looks/{k}"
        lk = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(c), roughness=0.6)
        lk.func(looks[k], lk)
    hw, L, t = spec.half
    gap = 0.0004  # 0.4 mm between the halves at the fold line
    paths = {}
    for name, lx in (("half_a", 0.0), ("half_b", hw + gap)):  # half_a centred on the root frame (root link)
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(lx, 0.0, 0.0))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass_half)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateLinearDampingAttr(0.2)
        rb.CreateAngularDampingAttr(0.5)
        geom = UsdGeom.Cube.Define(stage, f"{path}/geom")
        geom.CreateSizeAttr(1.0)
        geom.AddScaleOp().Set(Gf.Vec3f(hw, L, t * 0.6))
        geom.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -t * 0.2))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
        cp.CreateContactOffsetAttr(0.0015)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(geom.GetPrim().GetPath()), phys, stage=stage)
        bind_visual_material(str(geom.GetPrim().GetPath()), looks["back"], stage=stage)
        face = UsdGeom.Cube.Define(stage, f"{path}/face")  # the adhesive layer (visual; part of the collider above)
        face.CreateSizeAttr(1.0)
        face.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, t * 0.3))
        face.AddScaleOp().Set(Gf.Vec3f(hw, L, t * 0.4))
        UsdPhysics.CollisionAPI.Apply(face.GetPrim())
        cp2 = PhysxSchema.PhysxCollisionAPI.Apply(face.GetPrim())
        cp2.CreateContactOffsetAttr(0.0015)
        cp2.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(face.GetPrim().GetPath()), phys, stage=stage)
        bind_visual_material(str(face.GetPrim().GetPath()), looks["face"], stage=stage)
        paths[name] = path
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{paths['half_b']}/fold")
    j.CreateBody0Rel().SetTargets([Sdf.Path(paths["half_a"])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(paths["half_b"])])
    j.CreateAxisAttr("Y")
    # the fold line: the adhesive face's edge, in each half's frame
    j.CreateLocalPos0Attr(Gf.Vec3f(hw / 2 + gap / 2, 0, t / 2))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(-hw / 2 - gap / 2, 0, t / 2))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(-180.0)  # half_b folded up and over onto half_a turns it -180 deg about +y
    j.CreateUpperLimitAttr(3.0)
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    drv.CreateStiffnessAttr(0.0)
    drv.CreateDampingAttr(spec.damping * math.pi / 180)
    root_link = stage.GetPrimAtPath(paths["half_a"])
    UsdPhysics.ArticulationRootAPI.Apply(root_link)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root_link)
    art.CreateEnabledSelfCollisionsAttr(False)
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
                actuators={"fold": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=0.0, damping=spec.damping,
                                                       armature=2e-6)})
            return self._add_initial_pose_to_cfg(cfg)

    hw = spec.half[0]
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    d = -(hw / 2 + 0.0002)  # half_a's centre from the fold line along the patch's own x (gap 0.4 mm)
    a_xyz = (spec.pos[0] + c * d, spec.pos[1] + s * d, spec.pos[2])
    return PatchObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                       spawner_cfg=PatchSpawnerCfg(patch_name=spec.name),
                       initial_pose=Pose(position_xyz=a_xyz,
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


def _patch_state(env, spec: Patch) -> PatchState:
    import torch

    base = env.unwrapped
    art = base.scene[spec.name]
    origin = _np(base.scene.env_origins)[0]
    names = list(art.body_names)
    ia, ib = names.index("half_a"), names.index("half_b")
    pos = _np(art.data.body_pos_w)[0] - origin
    quat = _np(art.data.body_quat_w)[0]  # Isaac Lab here gives (x, y, z, w)
    vel = _np(art.data.body_lin_vel_w)[0]
    angv = _np(art.data.body_ang_vel_w)[0]
    qa, qb = quat[ia], quat[ib]
    ang = float(-math.degrees(_np(art.data.joint_pos)[0][0]))  # 0 flat .. 180 folded
    if ang >= spec.stick_deg:
        STUCK[spec.name] = True
    stuck = STUCK.get(spec.name, False)
    mode = 1 if stuck else 0  # 0 free hinge, 1 stuck shut (the adhesive: a drive holding it fully folded)
    if MODE.get(spec.name, 0) != mode:
        MODE[spec.name] = mode
        art.write_joint_stiffness_to_sim(torch.tensor([[spec.k_stick if mode else 0.0]], device=art.device))
        art.set_joint_position_target(torch.tensor([[-math.pi if mode else 0.0]], device=art.device))
    pts = np.vstack([_corners(pos[ia], qa, spec.half), _corners(pos[ib], qb, spec.half)])
    return PatchState(pos=(pos[ia] + pos[ib]) / 2, quat_xyzw=qa, lin_vel=(vel[ia] + vel[ib]) / 2,
                      ang_vel=(angv[ia] + angv[ib]) / 2, low=pts.min(0), high=pts.max(0), hinge_deg=ang, stuck=stuck,
                      a_pos=pos[ia], b_pos=pos[ib], a_quat=qa, b_quat=qb, a_vel=vel[ia], b_vel=vel[ib])


def offline_state(spec: Patch, hinge_deg: float = 0.0, pos=None, vel=(0.0, 0.0, 0.0)) -> PatchState:
    """A PatchState without the simulator (offline tests): the patch flat, or folded ``hinge_deg``, with its fold line's
    middle at ``pos`` (default: where it starts), turned ``spec.yaw``."""
    p = np.asarray(pos if pos is not None else spec.pos, float)
    hw, t = spec.half[0], spec.size[2]
    th = math.radians(hinge_deg)
    a_l = np.array([-hw / 2, 0.0, 0.0])  # in the patch's own frame, from the fold line's middle at mid-thickness
    b_l = np.array([hw / 2 * math.cos(th) + t / 2 * math.sin(th), 0.0,
                    t / 2 + hw / 2 * math.sin(th) - t / 2 * math.cos(th)])
    c, s_ = math.cos(spec.yaw), math.sin(spec.yaw)
    rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    a, b = p + rz @ a_l, p + rz @ b_l
    qa = np.array([0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)])
    qb = qa  # (orientation of half_b is not used by the scoring)
    pts = np.vstack([_corners(a, qa, spec.half), _corners(b, qa, spec.half)])
    v = np.asarray(vel, float)
    return PatchState(pos=(a + b) / 2, quat_xyzw=qa, lin_vel=v, ang_vel=np.zeros(3), low=pts.min(0), high=pts.max(0),
                      hinge_deg=hinge_deg, stuck=hinge_deg >= spec.stick_deg, a_pos=a, b_pos=b, a_quat=qa, b_quat=qb,
                      a_vel=v, b_vel=v)


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
