"""Exposed drawstring loops as flat film-tape straps (chains of thin rigid links).

Each exposed loop is a doubled flat strap: a chain of thin box links (strap
width x numerical thickness x link length) joined by native D6 joints. Joints
lock translation; the strap bends easily about its width axis (the flat tape's
soft bending direction), twists with a moderate spring, and bends only a little
in its own plane, like a flat tape. Only adjacent links skip collision. There
are no grasp attachments, kinematic links or per-step pose writes; a reset
event is the only operation that writes link poses. Link mass, stiffness and
friction are engineering values, not measurements of a particular drawstring.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def resample(points, segment):
    """Equal-chord resampling of a polyline (construction-time only)."""
    p = np.asarray(points, float)
    out = [p[0]]
    edge, cursor = 0, p[0].copy()
    while edge < len(p) - 1:
        d = p[edge + 1] - cursor
        dist = np.linalg.norm(d)
        if dist < 1e-12:
            edge += 1
            if edge < len(p) - 1:
                cursor = p[edge].copy()
            continue
        u = d / dist
        rel = cursor - out[-1]
        b = float(rel @ u)
        disc = b * b + segment * segment - float(rel @ rel)
        travel = -b + math.sqrt(max(0., disc)) if disc >= -1e-12 else math.inf
        if 1e-10 < travel <= dist + 1e-10:
            cursor = cursor + min(travel, dist) * u
            out.append(cursor.copy())
        else:
            edge += 1
            if edge < len(p) - 1:
                cursor = p[edge].copy()
    return np.asarray(out)


def _x_to(direction):
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    if d[0] < -1 + 1e-10:
        return np.array((0., 0., 1., 0.))
    q = np.array((0., -d[2], d[1], 1 + d[0]))
    return q / np.linalg.norm(q)


@dataclass(frozen=True)
class Chain:
    name: str
    nodes: tuple
    width: float = .018              # m, strap width (vertical when the strap stands on edge)
    thickness: float = .002          # m, numerical collision thickness of the doubled tape
    mass: float = .004               # kg, whole loop (solver value)
    soft_bend: tuple = (3e-4, 3e-5, 150.)    # about the width axis: N m/rad, N m s/rad, limit deg
    twist: tuple = (2e-3, 1e-4, 60.)         # about the strap axis
    stiff_bend: tuple = (5e-3, 2e-4, 25.)    # in the strap's own plane
    friction: float = 1.
    color: tuple = (.8, .8, .78)

    @property
    def segments(self):
        return len(self.nodes) - 1

    @property
    def seg_len(self):
        return float(np.linalg.norm(np.subtract(self.nodes[1], self.nodes[0])))

    def initial_poses(self):
        p = np.asarray(self.nodes, float)
        return (p[:-1] + p[1:]) / 2, np.stack([_x_to(d) for d in np.diff(p, axis=0)])


def chain(name, polyline, segment=.010, **kw):
    return Chain(name=name, nodes=tuple(map(tuple, resample(polyline, segment))), **kw)


_SPECS: dict = {}


def _spawn_link(prim_path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    spec = _SPECS[cfg.chain_name]
    index = cfg.link_index
    stage = get_current_stage()
    path = str(prim_path)
    root = path.rsplit("/", 1)[0]
    UsdGeom.Xform.Define(stage, root)
    body = UsdGeom.Xform.Define(stage, path)
    body.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0., 0., 0.))))
    x, y, z, w = orientation or (0., 0., 0., 1.)
    body.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    prim = body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass / spec.segments)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(64)
    rb.CreateSolverVelocityIterationCountAttr(16)
    rb.CreateLinearDampingAttr(.1)
    rb.CreateAngularDampingAttr(.5)
    rb.CreateMaxDepenetrationVelocityAttr(.5)
    rb.CreateSleepThresholdAttr(0.0)
    # Link frame: x along the strap, y across its thickness, z across its width.
    box = UsdGeom.Cube.Define(stage, path + "/collision")
    box.CreateSizeAttr(1.0)
    box.AddScaleOp().Set(Gf.Vec3f(spec.seg_len * .98, spec.thickness, spec.width))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    cp = PhysxSchema.PhysxCollisionAPI.Apply(box.GetPrim())
    cp.CreateContactOffsetAttr(.0008)
    cp.CreateRestOffsetAttr(0.0)
    material_path = root + "/Looks/physics"
    if not stage.GetPrimAtPath(material_path).IsValid():
        material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction * .9, restitution=0.)
        material.func(material_path, material)
    bind_physics_material(str(box.GetPath()), material_path, stage=stage)
    visual_path = root + "/Looks/visual"
    if not stage.GetPrimAtPath(visual_path).IsValid():
        sim_utils.PreviewSurfaceCfg(diffuse_color=spec.color, roughness=.6).func(visual_path, sim_utils.PreviewSurfaceCfg(diffuse_color=spec.color, roughness=.6))
    bind_visual_material(str(box.GetPath()), visual_path, stage=stage)
    others = [i for i in (index - 1, index + 1) if 0 <= i < spec.segments]
    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
        [Sdf.Path(root + f"/seg_{i:03d}") for i in others])
    if index:
        # Joint frames follow each link's own axes so that the drives act
        # about the strap's width (soft), axis (twist) and thickness (stiff).
        p0, q0 = spec.initial_poses()
        from scipy.spatial.transform import Rotation
        r0 = Rotation.from_quat(q0[index - 1]); r1 = Rotation.from_quat(q0[index])
        rel = (r0.inv() * r1).as_quat()  # parent-frame rotation of the child at rest
        joint = UsdPhysics.Joint.Define(stage, root + f"/joint_{index:03d}")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(root + f"/seg_{index - 1:03d}")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(spec.seg_len / 2, 0., 0.))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-spec.seg_len / 2, 0., 0.))
        joint.CreateLocalRot0Attr(Gf.Quatf(float(rel[3]), float(rel[0]), float(rel[1]), float(rel[2])))
        joint.CreateLocalRot1Attr(Gf.Quatf(1., 0., 0., 0.))
        joint.CreateCollisionEnabledAttr(False)
        jp = joint.GetPrim()
        for axis in ("transX", "transY", "transZ"):
            limit = UsdPhysics.LimitAPI.Apply(jp, axis)
            limit.CreateLowAttr(1.0)
            limit.CreateHighAttr(-1.0)
        for axis, (k, c, lim) in (("rotX", spec.twist), ("rotY", spec.stiff_bend), ("rotZ", spec.soft_bend)):
            limit = UsdPhysics.LimitAPI.Apply(jp, axis)
            limit.CreateLowAttr(-lim)
            limit.CreateHighAttr(lim)
            drive = UsdPhysics.DriveAPI.Apply(jp, axis)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k * math.pi / 180.)
            drive.CreateDampingAttr(c * math.pi / 180.)
            drive.CreateTargetPositionAttr(0.)
            drive.CreateTargetVelocityAttr(0.)
    return prim


def reset_chain(env, env_ids, chain_name: str):
    """Reset event only: restore each link's authored pose and zero its velocity."""
    import torch
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    collection = base.scene[chain_name]
    spec = _SPECS[chain_name]
    origin = base.scene.env_origins
    ids = torch.arange(len(origin), device=origin.device, dtype=torch.long) if env_ids is None else env_ids
    positions, quats = spec.initial_poses()
    pose = torch.as_tensor(np.concatenate((positions, quats), axis=1), device=origin.device, dtype=origin.dtype)
    pose = pose.unsqueeze(0).repeat(len(ids), 1, 1)
    pose[:, :, :3] += origin[ids, None, :]
    collection.write_body_link_pose_to_sim_index(body_poses=pose, env_ids=ids)
    collection.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids), spec.segments, 6), device=origin.device, dtype=origin.dtype), env_ids=ids)


def chain_asset(spec: Chain):
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset

    _SPECS[spec.name] = spec

    @configclass
    class LinkSpawnCfg(SpawnerCfg):
        func: object = clone(_spawn_link)
        chain_name: str = ""
        link_index: int = 0

    class ChainAsset(Asset):
        def __init__(self):
            super().__init__(spec.name, tags=["drawstring", "rigid-chain", "physx"])
            positions, quats = spec.initial_poses()
            self.object_cfg = RigidObjectCollectionCfg(rigid_objects={
                f"seg_{i:03d}": RigidObjectCfg(prim_path="{ENV_REGEX_NS}/" + spec.name + f"/seg_{i:03d}",
                                               spawn=LinkSpawnCfg(chain_name=spec.name, link_index=i),
                                               init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(p), rot=tuple(q)))
                for i, (p, q) in enumerate(zip(positions, quats))})

        def get_object_cfg(self):
            return self.name, self.object_cfg

        def get_event_cfg(self):
            return self.name, EventTermCfg(func=reset_chain, mode="reset", params={"chain_name": self.name})

        def validate_simulation_cfg(self, sim_cfg):
            return None

    return ChainAsset()


def chain_points(env, name):
    """Measured link centres (N, 3) in env coordinates."""
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    asset = base.scene[name]
    spec = _SPECS[name]
    names = list(getattr(asset, "body_names", None) or getattr(asset, "object_names"))
    order = [names.index(f"seg_{i:03d}") for i in range(spec.segments)]
    data = asset.data
    value = getattr(data, "body_link_pos_w", None)
    if value is None:
        value = data.object_link_pos_w
    value = getattr(value, "torch", value)[0, order].detach().cpu().numpy().astype(float)
    return value - getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy()
