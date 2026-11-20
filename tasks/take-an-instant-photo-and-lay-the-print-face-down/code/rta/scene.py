"""Memory task objects as Arena assets, and the Arena environment around them. Import after app start."""

from __future__ import annotations

import os

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.sim import schemas
from isaaclab.sim.spawners.shapes import spawn_cuboid
from isaaclab.sim.utils import clone
from isaaclab_arena.assets.object import Object
from isaaclab_arena.assets.object_type import ObjectType
from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
from isaaclab_arena.scene.scene import Scene
from isaaclab_arena.utils.pose import Pose
from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg
from yam_arena.scene import _contact_props, _grippy, _yaw_xyzw, env_cfg_callback, task_object, task_room
from yam_arena.station import linear_rgb

from rta import local_assets
from rta.spec import Compound, MemoryTask, Sheet, SheetState, quat_matrix

local_assets.register()  # models kept in the repo (rta/local_assets.py), usable as Asset(asset=<name>)


def compound_spawner(parts):
    """A cuboid spawner that adds ``parts`` as more cuboids of the same body, with the root's collision and
    materials. Children of ``{prim_path}/geometry`` share the root's rigid body, so PhysX collides them as one
    compound shape (a free compound) or as one static collider (a fixed one)."""

    @clone
    def spawn(prim_path: str, cfg: sim_utils.CuboidCfg, *args, **kwargs):
        from pxr import Gf, UsdGeom, UsdShade

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        geom = f"{prim_path}/geometry"
        mesh = stage.GetPrimAtPath(f"{geom}/mesh")
        bindings = []
        for purpose in (UsdShade.Tokens.allPurpose, "physics"):
            material = UsdShade.MaterialBindingAPI(mesh).GetDirectBinding(purpose).GetMaterial()
            if material:
                bindings.append((purpose, material))
        for i, part in enumerate(parts):
            path = f"{geom}/part{i}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(*part.offset))
            xf.AddScaleOp().Set(Gf.Vec3f(*part.size))
            if cfg.collision_props is not None:
                schemas.define_collision_properties(path, cfg.collision_props, stage=stage)
            api = UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
            for purpose, material in bindings:
                api.Bind(material, materialPurpose=purpose)
        return prim

    return spawn


def _material(spec: Compound) -> sim_utils.RigidBodyMaterialCfg:
    """The compound's own friction (e.g. a plastic slide in its rail) or the rubbery default."""
    if spec.friction is None:
        return _grippy()
    return sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])


def compound_object(spec: Compound) -> Object:
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4)
    if spec.free:
        body = dict(
            rigid_props=PhysxRigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
            collision_props=_contact_props(),
        )
    else:
        body = dict(collision_props=PhysxCollisionPropertiesCfg())
    cfg = sim_utils.CuboidCfg(
        func=compound_spawner(spec.parts), size=tuple(spec.size), physics_material=_material(spec), visual_material=look, **body
    )
    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
        spawner_cfg=cfg,
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)),
    )


# Optional PhysX settings for free bodies, applied by ``apply_solver_iters`` only when RTA_SOLVER_ITERS=1.
# Every task verification so far (2026-11-05, 16 bundles) ran at the PhysX defaults (4/1 iterations, no damping):
# the values set on the spawner never reached the bodies (the Arena's scene composition drops them), so turning
# them on is a physics change that needs re-verification. Motivation: thin objects in snug pockets and bodies
# pinned by the fingers rattled at 4-7 cm/s and never counted as resting; a tube in a square pocket precessed
# indefinitely (J4 j04d, since fixed by geometry).
SOLVER_ITERS = dict(
    solver_position_iteration_count=16, solver_velocity_iteration_count=4,
    # mild air/contact dissipation: with zero damping a tube standing in a pocket precessed around the wall
    # indefinitely (1.2 cm orbit at 1.4 Hz, J4 j04d) and never counted as resting
    linear_damping=0.5, angular_damping=1.0,
)


# -- sheets (paper, card): an articulation of thin strips joined by spring hinges -------------------------------

_SHEETS: dict[str, Sheet] = {}


def _spawn_sheet(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the sheet under ``prim_path``: strip_0 (the top strip, the articulation's root link) on the root frame,
    strip_i hanging ``i`` strip heights below it, each joined to the one above by a revolute hinge along local y."""
    import math

    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    spec = _SHEETS[cfg.sheet_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab 3: (x, y, z, w)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    vis = f"{prim_path}/Looks/sheet"
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.8)
    look.func(vis, look)
    phys = f"{prim_path}/Looks/sheet_physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
    pm.func(phys, pm)

    h, n = spec.strip_h, spec.strips
    paths = []
    for i in range(n):
        path = f"{prim_path}/strip_{i}"
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, -i * h))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.sheet_mass / n)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateLinearDampingAttr(0.5)  # air drag on a light, broad sheet
        rb.CreateAngularDampingAttr(2.0)
        geom = UsdGeom.Cube.Define(stage, f"{path}/geom")
        geom.CreateSizeAttr(1.0)
        geom.AddScaleOp().Set(Gf.Vec3f(spec.thickness, spec.width, h))
        gp = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(gp)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(gp)
        cp.CreateContactOffsetAttr(0.001)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(gp.GetPath()), phys, stage=stage)
        bind_visual_material(str(gp.GetPath()), vis, stage=stage)
        paths.append(path)
    for i in range(1, n):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{paths[i]}/hinge_{i}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[i - 1])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[i])])
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, -h / 2))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, h / 2))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(-math.degrees(spec.max_bend))
        j.CreateUpperLimitAttr(math.degrees(spec.max_bend))
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(spec.hinge_stiffness * math.pi / 180)  # USD angular drive gains are per degree
        drv.CreateDampingAttr(spec.hinge_damping * math.pi / 180)
        drv.CreateTargetPositionAttr(0.0)
    root_link = stage.GetPrimAtPath(paths[0])
    UsdPhysics.ArticulationRootAPI.Apply(root_link)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root_link)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def sheet_object(spec: Sheet) -> Object:
    import math

    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.utils import configclass

    _SHEETS[spec.name] = spec

    @configclass
    class SheetSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_sheet)
        sheet_name: str = ""

    class SheetObject(Object):
        def _generate_articulation_cfg(self):
            # the implicit actuators carry the hinge springs (Isaac Lab writes these gains over the USD drive's)
            actuators = {"hinge": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=spec.hinge_stiffness,
                                                      damping=spec.hinge_damping, armature=spec.armature)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return SheetObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=SheetSpawnerCfg(sheet_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.top_centre), rotation_xyzw=spec.quat_xyzw()),
    )


def sheet_state(env, spec: Sheet) -> SheetState:
    """Every strip's pose (env 0, world frame relative to the env origin)."""
    import numpy as np

    base = env.unwrapped
    art = base.scene[spec.name]

    def arr(x):
        return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)

    origin = arr(base.scene.env_origins[0])
    names = list(art.body_names)
    idx = [names.index(f"strip_{i}") for i in range(spec.strips)]
    pose = arr(art.data.body_link_pose_w)[0][idx]  # (strips, 7): position, quaternion (x, y, z, w)
    pos = pose[:, :3] - origin
    rots = np.stack([quat_matrix(q) for q in pose[:, 3:7]])
    ang = getattr(art.data, "body_com_ang_vel_w", None)
    ang_vel = arr(ang)[0][idx].mean(0) if ang is not None else np.zeros(3)
    half = np.array([spec.thickness / 2, spec.width / 2, spec.strip_h / 2])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    pts = np.vstack([c + corners @ r.T for c, r in zip(pos, rots, strict=True)])
    return SheetState(pos=pos.mean(0), quat_xyzw=pose[0, 3:7], lin_vel=np.zeros(3), ang_vel=ang_vel, low=pts.min(0),
                      high=pts.max(0), centres=pos, rotations=rots, strip_h=spec.strip_h, width=spec.width,
                      thickness=spec.thickness)


def memory_states(env, task: MemoryTask) -> dict:
    """Every task object's state: yam_arena's for primitives, compounds and assets, :func:`sheet_state` for sheets.
    (The embodiment replaces ``lin_vel`` with the bounding-box centre's displacement over the step.)"""
    from yam_arena.scene import object_states

    sheets = {o.name: o for o in task.objects if isinstance(o, Sheet)}
    articulated = set(getattr(task.module, "JOINTS", {}))
    out = object_states(env, [n for n in task.names if n not in sheets and n not in articulated])
    out.update({n: sheet_state(env, s) for n, s in sheets.items()})
    out.update({n: articulated_state(env, n) for n in articulated})
    return {n: out[n] for n in task.names}


_ART_BOXES: dict = {}  # name -> (low, high): an articulated model's local bounds (its file's, at its scale)


def articulated_state(env, name: str):
    """An articulated model's state as ``yam_arena.scene.object_states`` gives a rigid one (Arena's geometry queries
    take no articulations): its root link's pose and velocities, its local bounds' centre carried by that pose (the
    links' small motions, a button's 2 mm, are not followed) and the world-aligned box of those bounds."""
    from yam_arena.task_file import ObjectState

    base = env.unwrapped
    art = base.scene[name]

    def arr(x):
        return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)

    origin = arr(base.scene.env_origins[0])
    pose = arr(art.data.root_pose_w)[0]
    pos, q = pose[:3] - origin, pose[3:7]
    rot = quat_matrix(q)
    lo, hi = _ART_BOXES[name]
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    world = pos + corners @ rot.T
    lin = arr(art.data.root_lin_vel_w)[0] if hasattr(art.data, "root_lin_vel_w") else np.zeros(3)
    ang = arr(art.data.root_ang_vel_w)[0] if hasattr(art.data, "root_ang_vel_w") else np.zeros(3)
    return ObjectState(pos=pos + rot @ ((lo + hi) / 2), quat_xyzw=q, lin_vel=lin, ang_vel=ang, low=world.min(0), high=world.max(0))


def articulated_asset_object(spec, joints: dict) -> Object:
    """A free model whose USD file is an articulation (links and joints authored in the file, e.g. a camera body with
    a sprung shutter button on a prismatic joint), placed as a library asset is. ``joints``: {joint name: (stiffness,
    damping)}: an implicit actuator holds each joint at 0 with that spring (N/m or N m/rad) and damping; the links'
    masses come from the file. The joints' positions reach the task as ``Snapshot.joints[name]``."""
    from isaaclab.actuators import ImplicitActuatorCfg

    rigid = task_object(spec)  # the same model as a rigid library item: its file, scale and initial pose
    from isaaclab_arena.utils.usd.helpers import compute_local_bounding_box_from_usd

    b = compute_local_bounding_box_from_usd(rigid.usd_path)
    k = float(rigid.scale[0]) if rigid.scale is not None else 1.0
    _ART_BOXES[spec.name] = (np.asarray(b.min_point[0].tolist(), float) * k, np.asarray(b.max_point[0].tolist(), float) * k)

    class ArticulatedAsset(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {j: ImplicitActuatorCfg(joint_names_expr=[j], stiffness=k, damping=c) for j, (k, c) in joints.items()}
            return cfg

    return ArticulatedAsset(name=spec.name, usd_path=rigid.usd_path, object_type=ObjectType.ARTICULATION, scale=rigid.scale,
                            initial_pose=rigid.initial_pose, spawn_cfg_addon={"physics_material": _grippy(), "collision_props": _contact_props()})


def memory_object(spec, joints: dict | None = None) -> Object:
    if isinstance(spec, Sheet):
        return sheet_object(spec)
    if joints:
        return articulated_asset_object(spec, joints)
    return compound_object(spec) if isinstance(spec, Compound) else task_object(spec)


# PhysX GPU pair buffers are allocated in pinned host memory at the Isaac Lab defaults (2**25 aggregate pairs =
# 268 MB), which fails on a 24 GB consumer GPU ("PxgPinnedMemoryAllocate failed ... lost found pairs") as soon as a scene has
# aggregates (our compound bodies). One env with a dozen bodies needs a tiny fraction of that.
GPU_PAIR_CAPACITY = 2**18


def apply_solver_iters(env_cfg, free_names) -> list[str]:
    """Write ``SOLVER_ITERS`` onto the composed scene cfg of every free object and return the names written.
    Values set on a task object's spawner do not survive the Arena's scene composition (``Scene.get_scene_cfg()``
    hands Isaac Lab a rigid_props with every field None — probed 2026-11-05), so this runs in the env-cfg
    callback, on the cfg Isaac Lab actually spawns from. The trace header's ``reset.rigid_props`` shows the result."""
    done = []
    for name in free_names:
        spawn = getattr(getattr(env_cfg.scene, name, None), "spawn", None)
        if spawn is None:
            raise RuntimeError(f"{name}: no spawn cfg on the composed scene cfg")
        if getattr(spawn, "rigid_props", None) is None:  # library asset (UsdFileCfg): no rigid_props yet
            spawn.rigid_props = PhysxRigidBodyPropertiesCfg()
        for k, v in SOLVER_ITERS.items():
            setattr(spawn.rigid_props, k, v)
        done.append(name)
    return done


def memory_env_cfg_callback(control_hz: float | None = None, free_names: tuple[str, ...] = ()):
    configure = env_cfg_callback(control_hz)

    def configure_memory(env_cfg):
        env_cfg = configure(env_cfg)
        if os.environ.get("RTA_SOLVER_ITERS") == "1":
            apply_solver_iters(env_cfg, free_names)
        from isaaclab_physx.physics import PhysxCfg

        physics = env_cfg.sim.physics
        if physics is None:
            physics = env_cfg.sim.physics = PhysxCfg()
        if isinstance(physics, PhysxCfg):
            for attr in ("gpu_found_lost_pairs_capacity", "gpu_found_lost_aggregate_pairs_capacity", "gpu_total_aggregate_pairs_capacity"):
                if hasattr(physics, attr):
                    setattr(physics, attr, GPU_PAIR_CAPACITY)
        return env_cfg

    return configure_memory


def build_memory_environment(embodiment, task: MemoryTask, control_hz: float | None = None):
    return IsaacLabArenaEnvironment(
        name=f"yam_memory_{task.name}",
        scene=Scene([*task_room(task.scene, embodiment.rig.top_cam),
                     *(memory_object(o, getattr(task.module, "JOINTS", {}).get(o.name)) for o in task.objects)]),
        embodiment=embodiment,
        env_cfg_callback=memory_env_cfg_callback(control_hz, tuple(o.name for o in task.objects if o.free)),
    )


def make_memory_env(embodiment, task: MemoryTask, *, device: str = "cuda:0", control_hz: float | None = None):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg

    cfg = ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)
    return ArenaEnvBuilder(build_memory_environment(embodiment, task, control_hz), cfg).make_registered()
