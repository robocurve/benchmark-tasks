"""hoistkit: reduced-coordinate PhysX articulations for a friction-bearing pulley and a rope+load hoist assembly.

Plugs into yam_arena.scene cleanly alongside kit.py (pure Python at import time; Isaac Sim imported lazily).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
import os

# TGS velocity iterations for the rope, load and block (1 was tried: grips slipped and the rope blew up)
VEL_ITERS = int(os.environ.get("HOIST_VEL_ITERS", "8"))

import kit
from yam_arena.task_file import Box, ObjectState

__all__ = ["PulleyBlock", "PulleyState", "HoistAssembly", "HoistState", "install"]


def _quat_wxyz_from_mat(r: np.ndarray) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = kit.quat_from_matrix(r)
    return (float(qw), float(qx), float(qy), float(qz))


def _mat_from_quat_wxyz(q) -> np.ndarray:
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _add_shapes(stage, link_path: str, shapes, phys_path: str, mats: dict, root_path: str, is_dynamic: bool):
    """Author kit BoxShape/CylShape/MeshShape/SphereShape under ``link_path``."""
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, Vt
    from yam_arena.station import linear_rgb

    def look_for(color):
        if color not in mats:
            mp = f"{root_path}/Looks/m_{len(mats)}"
            cfg = kit.material_cfg(color, linear_rgb)
            cfg.func(mp, cfg)
            mats[color] = mp
        return mats[color]

    for idx, sh in enumerate(shapes):
        sp = f"{link_path}/s_{idx}"
        if isinstance(sh, kit.BoxShape):
            g = UsdGeom.Cube.Define(stage, sp)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3f(*sh.centre))
            if sh.quat_xyzw is not None:
                x, y, z, w = sh.quat_xyzw
                g.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
            elif sh.yaw:
                g.AddOrientOp().Set(Gf.Quatf(math.cos(sh.yaw / 2), 0.0, 0.0, math.sin(sh.yaw / 2)))
            g.AddScaleOp().Set(Gf.Vec3f(*sh.size))
        elif isinstance(sh, kit.CylShape):
            g = UsdGeom.Cylinder.Define(stage, sp)
            g.CreateRadiusAttr(sh.radius)
            g.CreateHeightAttr(sh.height)
            g.CreateAxisAttr(sh.axis)
            g.AddTranslateOp().Set(Gf.Vec3f(*sh.centre))
            if sh.quat_xyzw is not None:
                x, y, z, w = sh.quat_xyzw
                g.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        elif isinstance(sh, kit.SphereShape):
            g = UsdGeom.Sphere.Define(stage, sp)
            g.CreateRadiusAttr(sh.radius)
            g.AddTranslateOp().Set(Gf.Vec3f(*sh.centre))
        elif isinstance(sh, kit.MeshShape):
            g = UsdGeom.Mesh.Define(stage, sp)
            g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(sh.points, np.float32)))
            faces = np.asarray(sh.faces, np.int32)
            g.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
            g.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
            g.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            if sh.smooth:
                pts_a = np.asarray(sh.points, np.float64)
                g.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(kit._vertex_normals(pts_a, faces)))
                g.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        else:
            raise TypeError(f"unsupported shape {type(sh)}")

        prim = g.GetPrim()
        if sh.color is None:
            g.CreatePurposeAttr("guide")
        else:
            bind_visual_material(str(prim.GetPath()), look_for(sh.color), stage=stage)
        if sh.collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            if isinstance(sh, kit.MeshShape):
                mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mc.CreateApproximationAttr("convexHull" if is_dynamic else "none")
            bind_physics_material(str(prim.GetPath()), phys_path, stage=stage)


# -- PulleyBlock (1-DOF fixed-base articulation with real Coulomb bearing friction) ----------------------------------


@dataclass(frozen=True)
class PulleyBlock(Box):
    axle_pos: tuple[float, float, float] = (0.50, 0.0, 1.10)
    frame_shapes: tuple = ()
    sheave_shapes: tuple = ()
    sheave_mass: float = 0.35
    sheave_friction: tuple[float, float] = (1.2, 1.0)
    frame_friction: tuple[float, float] = (0.3, 0.25)
    bearing_friction: float = 0.45  # N·m Coulomb dry friction torque on revolute axle
    bearing_damping: float = 0.03  # N·m·s/rad viscous damping
    bearing_armature: float = 0.002  # kg·m²

    @property
    def free(self) -> bool:
        return False


@dataclass(frozen=True)
class PulleyState(ObjectState):
    angle_rad: float = 0.0
    wheel_vel: float = 0.0


def _spawn_pulley(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    spec: PulleyBlock = _PULLEYS[cfg.spec_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    tx, ty, tz = translation if translation is not None else spec.axle_pos
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(tx, ty, tz))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    f_phys = f"{prim_path}/Looks/frame_phys"
    sim_utils.RigidBodyMaterialCfg(
        static_friction=spec.frame_friction[0], dynamic_friction=spec.frame_friction[1]
    ).func(f_phys, sim_utils.RigidBodyMaterialCfg(
        static_friction=spec.frame_friction[0], dynamic_friction=spec.frame_friction[1]
    ))
    s_phys = f"{prim_path}/Looks/sheave_phys"
    pm_s = sim_utils.RigidBodyMaterialCfg(
        static_friction=spec.sheave_friction[0], dynamic_friction=spec.sheave_friction[1],
        friction_combine_mode="min",  # the sheave's own friction decides rope-on-sheave grip
    )
    pm_s.func(s_phys, pm_s)
    mats: dict = {}

    # Fixed root link: frame (welded to world)
    frame_path = f"{prim_path}/frame"
    fx = UsdGeom.Xform.Define(stage, frame_path)
    fx.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    fx.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    fprim = fx.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(fprim)
    UsdPhysics.MassAPI.Apply(fprim).CreateMassAttr(50.0)
    UsdPhysics.ArticulationRootAPI.Apply(fprim)
    art = PhysxSchema.PhysxArticulationAPI.Apply(fprim)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(VEL_ITERS)
    _add_shapes(stage, frame_path, spec.frame_shapes, f_phys, mats, prim_path, is_dynamic=True)

    # Weld frame to world
    fj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/world_weld")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(frame_path)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(tx, ty, tz))
    fj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # Child link: spinning sheave
    sheave_path = f"{prim_path}/sheave"
    sx = UsdGeom.Xform.Define(stage, sheave_path)
    sx.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    sx.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    sprim = sx.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(sprim)
    UsdPhysics.MassAPI.Apply(sprim).CreateMassAttr(spec.sheave_mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(sprim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(VEL_ITERS)
    _add_shapes(stage, sheave_path, spec.sheave_shapes, s_phys, mats, prim_path, is_dynamic=True)

    # Revolute joint about Y
    rj = UsdPhysics.RevoluteJoint.Define(stage, f"{sheave_path}/wheel_joint")
    rj.CreateBody0Rel().SetTargets([Sdf.Path(frame_path)])
    rj.CreateBody1Rel().SetTargets([Sdf.Path(sheave_path)])
    rj.CreateAxisAttr("Y")
    rj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    rj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    rj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    rj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    # an angular drive on the axle, so its spring (the ratchet's pawl) can be switched on and off while running
    drv = UsdPhysics.DriveAPI.Apply(rj.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    drv.CreateStiffnessAttr(0.0)
    drv.CreateDampingAttr(float(spec.bearing_damping) * math.pi / 180.0)
    drv.CreateTargetPositionAttr(0.0)
    drv.CreateMaxForceAttr(float(spec.bearing_friction))
    return root.GetPrim()


def _arena_pulley(spec: PulleyBlock):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PulleySpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_pulley)
        spec_name: str = ""

    class PulleyObject(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(
                prim_path=self.prim_path,
                spawn=self.spawner_cfg,
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=spec.axle_pos,
                    rot=(1.0, 0.0, 0.0, 0.0),
                    joint_pos={"wheel_joint": 0.0},
                    joint_vel={"wheel_joint": 0.0},
                ),
                actuators={
                    "bearing": ImplicitActuatorCfg(
                        joint_names_expr=["wheel_joint"],
                        stiffness=0.0,
                        damping=spec.bearing_damping,
                        joint_effort_limit=spec.bearing_friction,
                        friction=0.0,
                        armature=spec.bearing_armature,
                    )
                },
            )
            return self._add_initial_pose_to_cfg(cfg)

    return PulleyObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=PulleySpawnerCfg(spec_name=spec.name),
        initial_pose=Pose(position_xyz=spec.axle_pos, rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


def _pulley_state(env, spec: PulleyBlock) -> PulleyState:
    base = env.unwrapped
    art = base.scene[spec.name]
    jp = kit.as_numpy(art.data.joint_pos)[0]
    jv = kit.as_numpy(art.data.joint_vel)[0]
    p = np.asarray(spec.axle_pos, float)
    return PulleyState(
        pos=p,
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        lin_vel=np.zeros(3),
        ang_vel=np.array([0.0, float(jv[0]), 0.0]),
        low=p - 0.04,
        high=p + 0.04,
        angle_rad=float(jp[0]),
        wheel_vel=float(jv[0]),
    )


# -- HoistAssembly (floating-root reduced-coordinate heavy load + pre-routed articulated rope) -----------------------


@dataclass(frozen=True)
class HoistAssembly(Box):
    load_pos: tuple[float, float, float] = (0.54, 0.0, 0.80)
    load_half_size: tuple[float, float, float] = (0.036, 0.044, 0.0325)
    load_mass: float = 3.6
    load_shapes: tuple = ()
    load_friction: tuple[float, float] = (0.7, 0.6)
    load_bottom_pts: tuple = ()  # points on the load's underside (load frame); its lowest is the load's bottom
    rope_nodes: tuple = ()  # ((x, y, z), ...) world polyline starting at load's lifting eye
    rope_radius: float = 0.007
    rope_seg_mass: float = 0.006
    rope_friction: tuple[float, float] = (0.45, 0.40)
    rope_damping: float = 0.003
    rope_armature: float = 1e-4
    rope_max_bend_deg: float = 75.0
    collision_sides: int = 8
    capsule_links: bool = True

    @property
    def free(self) -> bool:
        return False

    @property
    def segments(self) -> int:
        return len(self.rope_nodes) - 1


@dataclass(frozen=True)
class HoistState(ObjectState):
    load_pos: np.ndarray = None
    load_rot: np.ndarray = None  # 3x3
    load_bottom_z: float = 0.0
    load_lin_vel: np.ndarray = None
    rope_points: np.ndarray = None  # (N, 3) link centres
    rope_rots: np.ndarray = None  # (N, 3, 3)
    rope_lengths: np.ndarray = None  # (N,)


def _seg_frame(d: np.ndarray) -> np.ndarray:
    """Right-handed frame with +X along segment direction ``d`` and +Y along world +Y (for x-z plane curves)."""
    ex = d / max(float(np.linalg.norm(d)), 1e-12)
    ey = np.array([0.0, 1.0, 0.0])
    if abs(float(np.dot(ex, ey))) > 0.95:
        ey = np.array([0.0, 0.0, 1.0])
    ez = np.cross(ex, ey)
    ez /= max(float(np.linalg.norm(ez)), 1e-12)
    ey = np.cross(ez, ex)
    return np.column_stack([ex, ey, ez])


def _spawn_hoist(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    spec: HoistAssembly = _HOISTS[cfg.spec_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    lp = np.asarray(translation if translation is not None else spec.load_pos, float)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*lp.tolist()))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    load_phys = f"{prim_path}/Looks/load_phys"
    pm_l = sim_utils.RigidBodyMaterialCfg(
        static_friction=spec.load_friction[0], dynamic_friction=spec.load_friction[1]
    )
    pm_l.func(load_phys, pm_l)

    rope_phys = f"{prim_path}/Looks/rope_phys"
    pm_r = sim_utils.RigidBodyMaterialCfg(
        static_friction=spec.rope_friction[0], dynamic_friction=spec.rope_friction[1]
    )
    pm_r.func(rope_phys, pm_r)
    mats: dict = {}

    # Floating root link: heavy load
    load_path = f"{prim_path}/load"
    lx = UsdGeom.Xform.Define(stage, load_path)
    lx.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    lx.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    lprim = lx.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(lprim)
    UsdPhysics.MassAPI.Apply(lprim).CreateMassAttr(spec.load_mass)
    lrb = PhysxSchema.PhysxRigidBodyAPI.Apply(lprim)
    lrb.CreateMaxDepenetrationVelocityAttr(1.0)
    lrb.CreateSolverPositionIterationCountAttr(64)
    lrb.CreateSolverVelocityIterationCountAttr(VEL_ITERS)
    lrb.CreateLinearDampingAttr(0.15)
    lrb.CreateAngularDampingAttr(0.8)
    _add_shapes(stage, load_path, spec.load_shapes, load_phys, mats, prim_path, is_dynamic=True)

    # ArticulationRoot on load (floating base)
    UsdPhysics.ArticulationRootAPI.Apply(lprim)
    art = PhysxSchema.PhysxArticulationAPI.Apply(lprim)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(64)
    art.CreateSolverVelocityIterationCountAttr(VEL_ITERS)

    # Rope links routed along spec.rope_nodes
    nodes = np.asarray(spec.rope_nodes, float)
    n = len(nodes) - 1
    r = spec.rope_radius
    sides = int(spec.collision_sides)

    def prism_points(L: float) -> list:
        rv = r / math.cos(math.pi / sides)
        pts = []
        # full-width ends that overlap the next link: no waist at the joints for the fingers to wedge into
        for xx, rr in ((L / 2, rv), (L / 2 + 0.6 * r, 0.85 * rv)):
            for sx in (-xx, xx):
                for k in range(sides):
                    a = 2 * math.pi * (k + 0.5) / sides - math.pi / 2
                    pts.append(Gf.Vec3f(sx, rr * math.cos(a), rr * math.sin(a)))
        return pts

    seg_paths, seg_rots, seg_lens = [], [], []
    for i in range(n):
        a, b = nodes[i], nodes[i + 1]
        L = float(np.linalg.norm(b - a))
        mid_rel = (a + b) / 2.0 - lp
        R = _seg_frame(b - a)
        qw, qx, qy, qz = _quat_wxyz_from_mat(R)

        sp = f"{prim_path}/seg_{i:02d}"
        sx = UsdGeom.Xform.Define(stage, sp)
        sx.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*mid_rel.tolist()))
        sx.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))
        sprim = sx.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(sprim)
        UsdPhysics.MassAPI.Apply(sprim).CreateMassAttr(spec.rope_seg_mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(sprim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(64)
        rb.CreateSolverVelocityIterationCountAttr(VEL_ITERS)
        rb.CreateLinearDampingAttr(0.35)
        rb.CreateAngularDampingAttr(1.2)

        if spec.capsule_links:
            # a capsule along the link, its round ends overlapping the neighbours': smooth everywhere, so a finger
            # squeezing it pushes straight across the rope and can never drive it along its length
            mesh = UsdGeom.Capsule.Define(stage, f"{sp}/collider")
            mesh.CreateRadiusAttr(r)
            mesh.CreateHeightAttr(L)
            mesh.CreateAxisAttr("X")
            mesh.CreatePurposeAttr("guide")  # rendered via ropeskin DynamicMesh
        else:
            mesh = UsdGeom.Mesh.Define(stage, f"{sp}/collider")
            mesh.CreatePointsAttr(prism_points(L))
            counts, idxs = [], []
            for ring in range(2):
                base_i = ring * 2 * sides
                counts += [sides, sides]
                idxs += [base_i + k for k in range(sides)] + [base_i + sides + k for k in range(sides)]
            mesh.CreateFaceVertexCountsAttr(counts)
            mesh.CreateFaceVertexIndicesAttr(idxs)
            mesh.CreatePurposeAttr("guide")  # rendered via ropeskin DynamicMesh
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(mesh.GetPrim().GetPath()), rope_phys, stage=stage)

        seg_paths.append(sp)
        seg_rots.append(R)
        seg_lens.append(L)

    def add_d6(parent: str, child: str, jname: str, pos0, rot0_wxyz, pos1, lim_y, lim_z, lock_twist=True):
        j = UsdPhysics.Joint.Define(stage, f"{child}/{jname}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in pos0]))
        j.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in rot0_wxyz]))
        j.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in pos1]))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        jp = j.GetPrim()
        locked = ("transX", "transY", "transZ", "rotX") if lock_twist else ("transX", "transY", "transZ")
        for dof in locked:
            lim = UsdPhysics.LimitAPI.Apply(jp, dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        for dof, (lo, hi) in (("rotY", lim_y), ("rotZ", lim_z)):
            lim = UsdPhysics.LimitAPI.Apply(jp, dof)
            lim.CreateLowAttr(float(lo))
            lim.CreateHighAttr(float(hi))
            drv = UsdPhysics.DriveAPI.Apply(jp, dof)
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(0.0)
            drv.CreateDampingAttr(spec.rope_damping * math.pi / 180.0)
            drv.CreateTargetPositionAttr(0.0)
        if not lock_twist:
            drv = UsdPhysics.DriveAPI.Apply(jp, "rotX")
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(0.0)
            drv.CreateDampingAttr(spec.rope_damping * math.pi / 180.0)
            drv.CreateTargetPositionAttr(0.0)

    # Shackle joint from load to seg_0 (free swivel twist + generous swing)
    eye_rel = nodes[0] - lp
    add_d6(
        load_path,
        seg_paths[0],
        "bend_00",
        eye_rel,
        _quat_wxyz_from_mat(seg_rots[0]),
        (-seg_lens[0] / 2.0, 0.0, 0.0),
        (-85.0, 85.0),
        (-85.0, 85.0),
        lock_twist=False,
    )

    mb = float(spec.rope_max_bend_deg)
    for i in range(1, n):
        r_rel = seg_rots[i - 1].T @ seg_rots[i]
        # Initial pitch angle about Y (so limits remain symmetric around straight)
        delta_deg = math.degrees(math.atan2(r_rel[0, 2], r_rel[0, 0]))
        add_d6(
            seg_paths[i - 1],
            seg_paths[i],
            f"bend_{i:02d}",
            (seg_lens[i - 1] / 2.0, 0.0, 0.0),
            _quat_wxyz_from_mat(r_rel),
            (-seg_lens[i] / 2.0, 0.0, 0.0),
            (-mb - delta_deg, mb - delta_deg),
            (-mb, mb),
            lock_twist=True,
        )

    return root.GetPrim()


def _arena_hoist(spec: HoistAssembly):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class HoistSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_hoist)
        spec_name: str = ""

    class HoistObject(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(
                prim_path=self.prim_path,
                spawn=self.spawner_cfg,
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=spec.load_pos,
                    rot=(1.0, 0.0, 0.0, 0.0),
                    joint_pos={".*": 0.0},
                    joint_vel={".*": 0.0},
                ),
                actuators={
                    "rope_bend": ImplicitActuatorCfg(
                        joint_names_expr=[".*"],
                        stiffness=0.0,
                        damping=spec.rope_damping,
                        armature=spec.rope_armature,
                    )
                },
            )
            return self._add_initial_pose_to_cfg(cfg)

    return HoistObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=HoistSpawnerCfg(spec_name=spec.name),
        initial_pose=Pose(position_xyz=spec.load_pos, rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


def _hoist_state(env, spec: HoistAssembly) -> HoistState:
    base = env.unwrapped
    art = base.scene[spec.name]
    origin = kit.as_numpy(base.scene.env_origins[0])
    names = list(art.body_names)
    pos_w = kit.as_numpy(art.data.body_pos_w)[0] - origin
    quat_w = kit.as_numpy(art.data.body_quat_w)[0]  # this Isaac Lab's body_quat_w is (x, y, z, w)
    vel_w = kit.as_numpy(art.data.body_lin_vel_w)[0]
    ang_w = kit.as_numpy(art.data.body_ang_vel_w)[0]

    li = names.index("load")
    lpos = pos_w[li]
    lrot = kit.rotation(quat_w[li])
    lvel = vel_w[li]
    lang = ang_w[li]

    # Exact lowest corner of the heavy load block
    hx, hy, hz = spec.load_half_size
    corners = np.array(
        [[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
        dtype=float,
    )
    world_corners = lpos[None, :] + corners @ lrot.T
    if spec.load_bottom_pts:
        bp = np.asarray(spec.load_bottom_pts, float)
        load_bot_z = float((lpos[None, :] + bp @ lrot.T)[:, 2].min())
    else:
        load_bot_z = float(world_corners[:, 2].min())

    seg_idx = [names.index(f"seg_{i:02d}") for i in range(spec.segments)]
    rpts = pos_w[seg_idx]
    rrots = np.stack([kit.rotation(quat_w[k]) for k in seg_idx])
    nodes = np.asarray(spec.rope_nodes, float)
    rlens = np.linalg.norm(np.diff(nodes, axis=0), axis=1)

    return HoistState(
        pos=lpos,
        quat_xyzw=np.array(quat_w[li], float),
        lin_vel=lvel,
        ang_vel=lang,
        low=world_corners.min(0),
        high=world_corners.max(0),
        load_pos=lpos,
        load_rot=lrot,
        load_bottom_z=load_bot_z,
        load_lin_vel=lvel,
        rope_points=rpts,
        rope_rots=rrots,
        rope_lengths=rlens,
    )


# -- hooking yam_arena.scene -----------------------------------------------------------------------------------------

_PULLEYS: dict[str, PulleyBlock] = {}
_HOISTS: dict[str, HoistAssembly] = {}


def _patch_scene(scene) -> None:
    if getattr(scene, "_hoistkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, PulleyBlock):
            _PULLEYS[spec.name] = spec
            return _arena_pulley(spec)
        if isinstance(spec, HoistAssembly):
            _HOISTS[spec.name] = spec
            return _arena_hoist(spec)
        return orig_task_object(spec)

    def object_states(env, names):
        plain = [n for n in names if n not in _PULLEYS and n not in _HOISTS]
        out = orig_states(env, plain) if plain else {}
        for n in names:
            if n in _PULLEYS:
                out[n] = _pulley_state(env, _PULLEYS[n])
            elif n in _HOISTS:
                out[n] = _hoist_state(env, _HOISTS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._hoistkit_patched = True


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
        loader = spec.loader
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
