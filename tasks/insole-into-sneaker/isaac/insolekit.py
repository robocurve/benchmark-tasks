"""Soft foam insole and the shoe's soft tongue for a yam_arena task file.

A chain of N rigid foam slices (boxes, the insole's outline width, 7 mm thick) joined by D6 joints. Translations are
locked; bending through the thickness, in-plane bending and twist are free, each with a linear spring
k = E w t^3 / 12 / s, G w t^3 / 3 / s and E t w^3 / 12 / s from one foam modulus E (geom.E). The springs are the
same for the whole trial: nothing stiffens or relaxes. Only a reset event writes insole state (its start pose).
A ``Flap`` (the tongue) is the same kind of chain, padded and short, its first slice jointed to the shoe's body.
"""
import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

import geom


@dataclass(frozen=True)
class Insole(Box):
    model: str = "chain"
    heel: tuple = (0.0, 0.0, 0.0)
    heading: float = 0.0
    n_seg: int = 18
    mass_kg: float = geom.MASS
    youngs: float = geom.E
    friction: float = 0.8
    bend_damping: float = 2e-4

    @property
    def free(self):
        return False


@dataclass(frozen=True)
class Flap(Box):
    """A soft flap (the shoe's tongue): a chain of padded slices; the first is jointed to a part's body (the shoe)
    at ``root`` (part frame) with orientation ``rot`` (3x3 row-major, part frame) at rest. Bending, twist and in-plane
    springs per joint; nothing else holds it."""
    anchor: str = "shoe"
    root: tuple = (0.0, 0.0, 0.0)
    rot: tuple = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    root_rot: tuple = ()                       # the root joint's frame (part frame); its spring holds ``rot`` at rest
    part_pos: tuple = (0.0, 0.0, 0.0)          # the anchor part's start position (world); its start rotation is identity
    n_seg: int = 4
    seg_len: float = 0.0125
    width: float = 0.05
    thick: float = 0.009
    mass_kg: float = 0.012
    k_bend: float = 0.015                      # N m/rad per joint
    k_twist: float = 0.03
    k_plane: float = 0.05
    friction: float = 0.8
    tip_color: tuple = (0.6, 0.6, 0.6)

    @property
    def free(self):
        return False


@dataclass(frozen=True)
class InsoleState(ObjectState):
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))      # centre line, heel -> toe
    cloud: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))       # all bottom-surface points
    under: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))       # centre line on the underside


def insole(name, model, heel, heading=0.0, **kw):
    c = np.asarray(heel, float) + 0.5 * geom.L * np.array([math.cos(heading), math.sin(heading), 0.0])
    return Insole(name=name, pos=tuple(c), size=(geom.L, 0.1, geom.T), color=(0.30, 0.55, 0.80), model=model,
                  heel=tuple(heel), heading=heading, **kw)


SPECS = {}
FLAPS = {}


def flap(name, anchor, root, rot, part_pos, root_rot=None, **kw):
    R = np.asarray(rot, float).reshape(3, 3)
    F = R if root_rot is None else np.asarray(root_rot, float).reshape(3, 3)
    c = np.asarray(part_pos, float) + np.asarray(root, float)
    return Flap(name=name, pos=tuple(map(float, c)), size=(0.05, 0.05, 0.05), anchor=anchor,
                root=tuple(map(float, root)), rot=tuple(map(float, R.ravel())), root_rot=tuple(map(float, F.ravel())),
                part_pos=tuple(map(float, part_pos)), **kw)


def _root_target_deg(spec):
    """Bend (about the joint's y axis) that turns the root joint's frame into the rest orientation."""
    F = np.asarray(spec.root_rot, float).reshape(3, 3)
    R = np.asarray(spec.rot, float).reshape(3, 3)
    rel = F.T @ R                                   # = Ry(theta): x -> (cos, 0, -sin)
    return math.degrees(math.atan2(-rel[2, 0], rel[0, 0]))


def _mat_to_quat_xyzw(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


def flap_initial(spec):
    """Segment centres (world, env frame) and orientations xyzw at rest."""
    R = np.asarray(spec.rot, float).reshape(3, 3)
    c = (np.asarray(spec.part_pos, float) + np.asarray(spec.root, float)
         + np.outer((np.arange(spec.n_seg) + 0.5) * spec.seg_len, R[:, 0]))
    return c, np.tile(_mat_to_quat_xyzw(R), (spec.n_seg, 1))


def _spawn_flap_seg(prim_path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = FLAPS[cfg.flap_name]
    i, n, s = cfg.seg_index, spec.n_seg, spec.seg_len
    stage = get_current_stage()
    path = str(prim_path)
    root = path.rsplit("/", 1)[0]
    UsdGeom.Xform.Define(stage, root)
    body = UsdGeom.Xform.Define(stage, path)
    body.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0., 0., 0.))))
    x, y, z, qw = orientation or (0., 0., 0., 1.)
    body.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, x, y, z))
    prim = body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(spec.mass_kg / n))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(8)
    rb.CreateLinearDampingAttr(0.1)
    rb.CreateAngularDampingAttr(0.5)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    rb.CreateSleepThresholdAttr(0.0)
    cube = UsdGeom.Cube.Define(stage, path + "/collision")
    cube.CreateSizeAttr(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(float(s), float(spec.width), float(spec.thick)))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    cp = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    cp.CreateContactOffsetAttr(0.001)
    cp.CreateRestOffsetAttr(0.0)
    mat = root + "/Looks/physics"
    if not stage.GetPrimAtPath(mat).IsValid():
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction * 0.9,
                                            restitution=0.0)
        pm.func(mat, pm)
    bind_physics_material(str(cube.GetPath()), mat, stage=stage)
    vis = root + ("/Looks/tip" if i == n - 1 else "/Looks/pad")
    if not stage.GetPrimAtPath(vis).IsValid():
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.tip_color if i == n - 1 else spec.color),
                                           roughness=0.9)
        look.func(vis, look)
    bind_visual_material(str(cube.GetPath()), vis, stage=stage)
    anchor_body = root.rsplit("/", 1)[0] + f"/{spec.anchor}/body"
    others = [Sdf.Path(root + f"/seg_{j:03d}") for j in range(n) if abs(j - i) == 1]
    if i <= 1 or __import__("os").environ.get("INSOLE_TONGUE_NOSHOE") == "1":   # (diagnostic: every slice)
        others.append(Sdf.Path(anchor_body))     # the sewn-in root and its neighbour do not collide with the shoe
    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(others)
    j = UsdPhysics.Joint.Define(stage, root + f"/joint_{i:03d}")
    if i == 0:
        qx, qy, qz, qw = _mat_to_quat_xyzw(np.asarray(spec.root_rot, float).reshape(3, 3))
        j.CreateBody0Rel().SetTargets([Sdf.Path(anchor_body)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, spec.root)))
        j.CreateLocalRot0Attr(Gf.Quatf(float(qw), float(qx), float(qy), float(qz)))
    else:
        j.CreateBody0Rel().SetTargets([Sdf.Path(root + f"/seg_{i - 1:03d}")])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(s) / 2, 0., 0.))
        j.CreateLocalRot0Attr(Gf.Quatf(1., 0., 0., 0.))
    j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
    j.CreateLocalPos1Attr(Gf.Vec3f(-float(s) / 2, 0., 0.))
    j.CreateLocalRot1Attr(Gf.Quatf(1., 0., 0., 0.))
    j.CreateCollisionEnabledAttr(False)
    jp = j.GetPrim()
    for ax in ("transX", "transY", "transZ"):
        lim = UsdPhysics.LimitAPI.Apply(jp, ax)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)
    target = _root_target_deg(spec) if i == 0 else 0.0
    for ax, k, lim_deg in (("rotX", spec.k_twist, 45.0), ("rotY", spec.k_bend, 85.0), ("rotZ", spec.k_plane, 20.0)):
        lim = UsdPhysics.LimitAPI.Apply(jp, ax)
        lim.CreateLowAttr(-lim_deg)
        lim.CreateHighAttr(lim_deg)
        d = UsdPhysics.DriveAPI.Apply(jp, ax)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(k) * math.pi / 180)
        d.CreateDampingAttr(float(k) * 0.02 * math.pi / 180)
        d.CreateTargetPositionAttr(float(target) if ax == "rotY" else 0.)
        d.CreateTargetVelocityAttr(0.)
    return prim


def _chain_params(spec):
    s = geom.L / spec.n_seg
    u = (np.arange(spec.n_seg) + 0.5) / spec.n_seg
    w = np.array([geom.width(np.linspace(a - 0.5 / spec.n_seg, a + 0.5 / spec.n_seg, 9)).min() for a in u]) - 0.002
    E, t = spec.youngs, geom.T
    G = E / (2 * (1 + geom.NU))
    k_bend = E * w * t ** 3 / 12 / s
    k_twist = G * w * t ** 3 / 3 / s
    k_plane = E * t * w ** 3 / 12 / s
    m = spec.mass_kg * w / w.sum()
    return s, u, w, m, k_bend, k_twist, k_plane


def _quat_yaw(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def chain_initial(spec):
    centres, _, _ = geom.chain_layout(spec.n_seg, spec.heel, spec.heading)
    return centres, np.tile(_quat_yaw(spec.heading), (spec.n_seg, 1))


def _spawn_seg(prim_path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = SPECS[cfg.insole_name]
    i = cfg.seg_index
    s, u, w, m, kb, kt, kp = _chain_params(spec)
    stage = get_current_stage()
    path = str(prim_path)
    root = path.rsplit("/", 1)[0]
    UsdGeom.Xform.Define(stage, root)
    body = UsdGeom.Xform.Define(stage, path)
    body.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0., 0., 0.))))
    x, y, z, qw = orientation or (0., 0., 0., 1.)
    body.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, x, y, z))
    prim = body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(m[i]))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(int(__import__("os").environ.get("INSOLE_POS_ITERS", "32")))
    rb.CreateSolverVelocityIterationCountAttr(8)
    rb.CreateLinearDampingAttr(0.05)
    rb.CreateAngularDampingAttr(0.05)
    rb.CreateMaxDepenetrationVelocityAttr(float(__import__("os").environ.get("INSOLE_DEPEN", "0.5")))
    rb.CreateSleepThresholdAttr(0.0)
    # continuous collision detection on the slices (diagnostic knob, off: it made no difference)
    rb.CreateEnableCCDAttr(__import__("os").environ.get("INSOLE_CCD", "0") == "1")   # tried: no effect
    cube = UsdGeom.Cube.Define(stage, path + "/collision")
    cube.CreateSizeAttr(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(float(s), float(w[i]), geom.T))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    cp = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    cp.CreateContactOffsetAttr(0.001)
    cp.CreateRestOffsetAttr(0.0)
    mat = root + "/Looks/physics"
    if not stage.GetPrimAtPath(mat).IsValid():
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction * 0.9,
                                            restitution=0.0)
        pm.func(mat, pm)
    bind_physics_material(str(cube.GetPath()), mat, stage=stage)
    vis = root + "/Looks/foam"
    if not stage.GetPrimAtPath(vis).IsValid():
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.8)
        look.func(vis, look)
    bind_visual_material(str(cube.GetPath()), vis, stage=stage)
    others = [j for j in range(spec.n_seg) if abs(j - i) == 1]
    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
        [Sdf.Path(root + f"/seg_{j:03d}") for j in others])
    if i:
        j = UsdPhysics.Joint.Define(stage, root + f"/joint_{i:03d}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(root + f"/seg_{i - 1:03d}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(s) / 2, 0., 0.))
        j.CreateLocalPos1Attr(Gf.Vec3f(-float(s) / 2, 0., 0.))
        j.CreateLocalRot0Attr(Gf.Quatf(1., 0., 0., 0.))
        j.CreateLocalRot1Attr(Gf.Quatf(1., 0., 0., 0.))
        j.CreateCollisionEnabledAttr(False)
        jp = j.GetPrim()
        for ax in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(jp, ax)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        # joint i sits between slices i-1 and i: average their section stiffness
        for ax, k, lim_deg in (("rotX", (kt[i - 1] + kt[i]) / 2, 60.0), ("rotY", (kb[i - 1] + kb[i]) / 2, 85.0),
                               ("rotZ", (kp[i - 1] + kp[i]) / 2, 30.0)):
            lim = UsdPhysics.LimitAPI.Apply(jp, ax)
            lim.CreateLowAttr(-lim_deg)
            lim.CreateHighAttr(lim_deg)
            d = UsdPhysics.DriveAPI.Apply(jp, ax)
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(float(k) * math.pi / 180)
            d.CreateDampingAttr(spec.bend_damping * math.pi / 180 * (1 if ax != "rotZ" else 10))
            d.CreateTargetPositionAttr(0.)
            d.CreateTargetVelocityAttr(0.)
    return prim


def reset_chain(env, env_ids, name):
    import torch
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    coll = base.scene[name]
    spec = SPECS[name] if name in SPECS else FLAPS[name]
    origin = base.scene.env_origins
    origin = getattr(origin, "torch", origin)
    ids = torch.arange(len(origin), device=origin.device, dtype=torch.long) if env_ids is None else env_ids
    p, q = chain_initial(spec) if name in SPECS else flap_initial(spec)
    pose = torch.as_tensor(np.concatenate((p, q), 1), device=origin.device, dtype=origin.dtype)
    pose = pose.unsqueeze(0).repeat(len(ids), 1, 1)
    pose[:, :, :3] += origin[ids, None, :]
    coll.write_body_link_pose_to_sim_index(body_poses=pose, env_ids=ids)
    coll.write_body_link_velocity_to_sim_index(
        body_velocities=torch.zeros((len(ids), spec.n_seg, 6), device=origin.device, dtype=origin.dtype), env_ids=ids)


def _chain_asset(spec):
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset

    @configclass
    class SegCfg(SpawnerCfg):
        func: object = clone(_spawn_seg)
        insole_name: str = ""
        seg_index: int = 0

    class ChainAsset(Asset):
        def __init__(self):
            super().__init__(spec.name, tags=["insole", "rigid-chain"])
            p, q = chain_initial(spec)
            objs = {f"seg_{i:03d}": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + spec.name + f"/seg_{i:03d}",
                spawn=SegCfg(insole_name=spec.name, seg_index=i),
                init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(p[i]), rot=tuple(q[i])))
                for i in range(spec.n_seg)}
            self.object_cfg = RigidObjectCollectionCfg(rigid_objects=objs)

        def get_object_cfg(self):
            return self.name, self.object_cfg

        def get_event_cfg(self):
            return self.name, EventTermCfg(func=reset_chain, mode="reset", params={"name": self.name})

    return ChainAsset()


def _flap_asset(spec):
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset

    @configclass
    class FlapSegCfg(SpawnerCfg):
        func: object = clone(_spawn_flap_seg)
        flap_name: str = ""
        seg_index: int = 0

    class FlapAsset(Asset):
        def __init__(self):
            super().__init__(spec.name, tags=["flap", "rigid-chain"])
            p, q = flap_initial(spec)
            objs = {f"seg_{i:03d}": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + spec.name + f"/seg_{i:03d}",
                spawn=FlapSegCfg(flap_name=spec.name, seg_index=i),
                init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(p[i]), rot=tuple(q[i])))
                for i in range(spec.n_seg)}
            self.object_cfg = RigidObjectCollectionCfg(rigid_objects=objs)

        def get_object_cfg(self):
            return self.name, self.object_cfg

        def get_event_cfg(self):
            return self.name, EventTermCfg(func=reset_chain, mode="reset", params={"name": self.name})

    return FlapAsset()


# ---- state -----------------------------------------------------------------------------------------------

def _np(t):
    return getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def insole_state(env, name):
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    spec, obj = SPECS[name], base.scene[name]
    o = _np(base.scene.env_origins)[0]
    if True:
        names = list(getattr(obj, "body_names", None) or getattr(obj, "object_names"))
        ids = [names.index(f"seg_{i:03d}") for i in range(spec.n_seg)]
        d = obj.data
        pos = _np(getattr(d, "body_link_pos_w", None) if hasattr(d, "body_link_pos_w") else d.object_link_pos_w)[0, ids] - o
        quat = _np(getattr(d, "body_link_quat_w", None) if hasattr(d, "body_link_quat_w") else d.object_link_quat_w)[0, ids]
        vel = _np(getattr(d, "body_link_lin_vel_w", None) if hasattr(d, "body_link_lin_vel_w") else d.object_link_lin_vel_w)[0, ids]
        s, u, w, *_ = _chain_params(spec)
        R = [_rot(q) for q in quat]
        line = np.vstack([pos[0] - R[0][:, 0] * s / 2] + [p + r[:, 0] * s / 2 for p, r in zip(pos, R)])
        under = line - geom.T / 2 * np.vstack([R[0][:, 2]] + [r[:, 2] for r in R])
        cloud = np.vstack([p + r @ np.array([sx * s / 2, sy * wi / 2, -geom.T / 2])
                           for p, r, wi in zip(pos, R, w) for sx in (-1, 1) for sy in (-1, 0, 1)])
    allp = np.vstack([line, cloud])
    if not np.isfinite(allp).all():
        raise FloatingPointError(f"{name}: non-finite insole state")
    lo, hi = allp.min(0), allp.max(0)
    return InsoleState(pos=(lo + hi) / 2, quat_xyzw=np.array([0., 0., 0., 1.]), lin_vel=vel.mean(0),
                       ang_vel=np.zeros(3), low=lo, high=hi, points=line, cloud=cloud, under=under)


def flap_state(env, name):
    """Segment centres (env frame, root -> tip), their x axes, and the tip point."""
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    spec, obj = FLAPS[name], base.scene[name]
    o = _np(base.scene.env_origins)[0]
    names = list(getattr(obj, "body_names", None) or getattr(obj, "object_names"))
    ids = [names.index(f"seg_{i:03d}") for i in range(spec.n_seg)]
    d = obj.data
    pos = _np(d.body_link_pos_w if hasattr(d, "body_link_pos_w") else d.object_link_pos_w)[0, ids] - o
    quat = _np(d.body_link_quat_w if hasattr(d, "body_link_quat_w") else d.object_link_quat_w)[0, ids]
    ax = np.array([_rot(q)[:, 0] for q in quat])
    return pos, ax, pos[-1] + ax[-1] * spec.seg_len / 2


def _flap_obj_state(env, name):
    pos, ax, tip = flap_state(env, name)
    lo, hi = np.minimum(pos.min(0), tip), np.maximum(pos.max(0), tip)
    return ObjectState(pos=(lo + hi) / 2, quat_xyzw=np.array([0., 0., 0., 1.]), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=lo, high=hi)


def _patch(scene):
    if getattr(scene, "_insole_patched", False):
        return
    orig_obj, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Insole):
            SPECS[spec.name] = spec
            return _chain_asset(spec)
        if isinstance(spec, Flap):
            FLAPS[spec.name] = spec
            return _flap_asset(spec)
        return orig_obj(spec)

    def object_states(env, names):
        plain = [n for n in names if n not in SPECS and n not in FLAPS]
        out = orig_states(env, plain) if plain else {}
        out.update({n: insole_state(env, n) for n in names if n in SPECS})
        out.update({n: _flap_obj_state(env, n) for n in names if n in FLAPS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._insole_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        orig = spec.loader.exec_module

        def execute(module):
            orig(module)
            _patch(module)

        spec.loader.exec_module = execute
        return spec


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _Finder) for f in sys.meta_path):
    sys.meta_path.insert(0, _Finder())
