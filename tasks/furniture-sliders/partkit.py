"""Task-local compound rigid/articulated asset adapter.

Derived from hand-can-opener/openerkit.py, copyright 2026 The task authors (MIT;
see LICENSE). No robot attachment or weld facilities are included. Simulator imports
are deferred until AppLauncher starts. The installed packages are not modified.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
# box: (centre, size, colour); cylinder: (centre, radius, height, colour[, axis "X" | "Y" | "Z"]); part frame, m


@dataclass(frozen=True)
class Hinge:
    """A link on a revolute joint. Its shapes are given in the part frame with the joint at 0; ``anchor`` is a point on
    the joint axis (part frame) and ``axis`` one of "X", "Y", "Z". The link starts at ``start`` (rad). A drive of
    ``rate`` N m/rad pulls it toward ``start`` and ``damping`` N m s/rad resists its speed."""

    name: str
    anchor: tuple[float, float, float]
    axis: str = "Z"
    boxes: tuple = ()
    cylinders: tuple = ()
    meshes: tuple = ()
    parent: str = ""  # "" hangs it from the part's body, otherwise from the hinge of that name
    lower: float | None = None  # rad
    upper: float | None = None
    start: float = 0.0
    rate: float = 0.0
    damping: float = 0.0
    mass: float = 0.05
    armature: float = 0.0   # kg m^2 added about the joint (e.g. a gear train's inertia)
    friction: float = 0.0   # joint friction coefficient (PhysX)
    target: float | None = None  # spring's rest angle (rad), if not ``start`` (e.g. past a stop, so it presses on it)

    @property
    def rest(self) -> float:
        return self.start if self.target is None else self.target


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is its bounding box
    and ``pos`` is the body origin. ``mass=None`` fixes its body to the world."""

    boxes: tuple = ()
    cylinders: tuple = ()
    meshes: tuple = ()
    hinges: tuple[Hinge, ...] = ()
    friction: float = 0.8
    body_mass: float = 1.0
    quat: tuple | None = None  # start orientation (XYZW); overrides yaw
    materials: dict = field(default_factory=dict, compare=False, hash=False)
    center_of_mass: tuple | None = None
    diagonal_inertia: tuple | None = None
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not (self.boxes or self.cylinders or self.meshes):
            raise ValueError(f"{self.name}: a Part needs at least one shape")


def _cyl(spec):
    centre, radius, height, color = spec[:4]
    return centre, radius, height, color, (spec[4] if len(spec) > 4 else "Z")


def _cyl_half(spec):
    _, r, h, _, axis = _cyl(spec)
    return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])


def part(name, pos, boxes=(), mass=None, yaw=0.0, hinges=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         body_mass=1.0, quat=None, meshes=(), materials=None, center_of_mass=None, diagonal_inertia=None) -> Part:
    """Build a Part; ``size`` is filled in from the body's shapes."""
    half = [np.array(b[1]) / 2 for b in boxes] + [_cyl_half(c) for c in cylinders]
    centres = [np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cylinders]
    for mesh in meshes:
        vertices=np.asarray(mesh[0])
        half.append((vertices.max(0)-vertices.min(0))/2)
        centres.append((vertices.max(0)+vertices.min(0))/2)
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), cylinders=tuple(cylinders), meshes=tuple(meshes), hinges=tuple(hinges), friction=friction,
                body_mass=body_mass if mass is None else mass, quat=None if quat is None else tuple(float(v) for v in quat),
                materials=materials or {}, center_of_mass=center_of_mass, diagonal_inertia=diagonal_inertia,
                meta={"lo": tuple(lo.tolist()), "hi": tuple(hi.tolist())})


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each hinge's angle (rad)."""

    joints: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)


# -- spawning -----------------------------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}


def _link_anchor(spec: Part, link: str) -> np.ndarray:
    if not link:
        return np.zeros(3)
    return np.array(next(h.anchor for h in spec.hinges if h.name == link), float)


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the body and links as siblings."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb
    from config import NUMERICS

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    material_paths = {}
    for key, value in spec.materials.items():
        path = f"{prim_path}/Looks/physics_{key}"
        cfg_mat = sim_utils.RigidBodyMaterialCfg(
            static_friction=value["static"], dynamic_friction=value["dynamic"],
            friction_combine_mode="min", restitution=0., restitution_combine_mode="min")
        cfg_mat.func(path, cfg_mat)
        material_paths[key] = path
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            metal = color[3] if len(color) > 3 else 0.0
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color[:3]), roughness=0.45 - 0.2 * metal,
                                              metallic=metal)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(path: str, pos, mass: float, wxyz=(1.0, 0.0, 0.0, 0.0)):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*wxyz))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(NUMERICS['position_iterations'])
        rb.CreateSolverVelocityIterationCountAttr(NUMERICS['velocity_iterations'])
        rb.CreateAngularDampingAttr(0.0)
        rb.CreateLinearDampingAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    def collide(prim, color, visual=False, material=None):
        if visual:
            bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
            return
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(NUMERICS['contact_offset_m'])
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material_paths.get(material, phys), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def shapes(path: str, box_specs, cyl_specs, offset=(0.0, 0.0, 0.0), mesh_specs=()):
        off = np.array(offset, float)
        for i, b in enumerate(box_specs):
            centre, size, color = b[:3]
            opt = b[3] if len(b) > 3 else {}
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            if opt.get("yaw"):
                cube.AddRotateZOp().Set(math.degrees(opt["yaw"]))
            if opt.get("pitch"):
                cube.AddRotateYOp().Set(math.degrees(opt["pitch"]))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color, opt.get("visual", False), opt.get("material"))
            if opt.get("hidden"):
                cube.MakeInvisible()
        for i, c in enumerate(cyl_specs):
            centre, radius, height, color, axis = _cyl(c)
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(axis)
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            opt = c[5] if len(c) > 5 else {}
            collide(cyl.GetPrim(), color, opt.get("visual", False), opt.get("material"))
            if opt.get("hidden"):
                cyl.MakeInvisible()
        for i, mesh_spec in enumerate(mesh_specs):
            pts, faces, color = mesh_spec[:3]
            opt = mesh_spec[3] if len(mesh_spec)>3 else {'visual':True}
            mesh = UsdGeom.Mesh.Define(stage, f"{path}/mesh_{i}")
            mesh.CreatePointsAttr([Gf.Vec3f(*(np.array(q) - off)) for q in pts])
            mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
            mesh.CreateFaceVertexIndicesAttr([k for f in faces for k in f])
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            collide(mesh.GetPrim(), color, opt.get('visual',False), opt.get('material'))
            if not opt.get('visual',False):
                approximation=opt.get('approximation','convexHull')
                if approximation=='none' and (spec.mass is not None or spec.hinges):
                    raise ValueError('Triangle mesh without approximation must be static')
                UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(approximation)

    main = f"{prim_path}/body"
    if spec.mass is None and not spec.hinges:
        main_prim=UsdGeom.Xform.Define(stage,main).GetPrim()
    else:
        main_prim = body(main, (0.0, 0.0, 0.0), spec.body_mass)
    mass_api = UsdPhysics.MassAPI(main_prim)
    if spec.center_of_mass is not None:
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*spec.center_of_mass))
    if spec.diagonal_inertia is not None:
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*spec.diagonal_inertia))
        mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1.0))
    shapes(main, spec.boxes, spec.cylinders, mesh_specs=spec.meshes)
    paths = {"": main}
    for h in spec.hinges:
        path = f"{prim_path}/{h.name}"
        paths[h.name] = path
        # author the link already turned to its start angle (relative to its parent's start pose), so the
        # simulation starts without snapping it there
        ax = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[h.axis]
        half = h.start / 2
        body(path, h.anchor, h.mass, (math.cos(half), *(math.sin(half) * a for a in ax)))
        shapes(path, h.boxes, h.cylinders, offset=h.anchor, mesh_specs=h.meshes)
        parent_anchor = _link_anchor(spec, h.parent)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{h.name}_joint")
        j.CreateAxisAttr(h.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[h.parent])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(h.anchor) - parent_anchor)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if h.lower is not None:
            j.CreateLowerLimitAttr(math.degrees(h.lower))
            j.CreateUpperLimitAttr(math.degrees(h.upper))
        # USD drives are per degree; the spring pulls toward the start angle
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate * math.pi / 180)
        drive.CreateDampingAttr(h.damping * math.pi / 180)
        drive.CreateTargetPositionAttr(math.degrees(h.rest))
        state = PhysxSchema.JointStateAPI.Apply(j.GetPrim(), "angular")
        state.CreatePositionAttr(math.degrees(h.start))
        state.CreateVelocityAttr(0.0)
    if spec.mass is None and spec.hinges:
        fix = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fix_to_world")
        fix.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        world = UsdGeom.Xformable(stage.GetPrimAtPath(main)).ComputeLocalToWorldTransform(0)
        fix.CreateLocalPos0Attr(Gf.Vec3f(*world.ExtractTranslation()))
        fix.CreateLocalRot0Attr(Gf.Quatf(world.ExtractRotationQuat()))
        fix.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        fix.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if spec.hinges:
        art_prim = stage.GetPrimAtPath(prim_path if spec.mass is None else main)
        UsdPhysics.ArticulationRootAPI.Apply(art_prim)
        art = PhysxSchema.PhysxArticulationAPI.Apply(art_prim)
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            # Implicit actuators carry the springs and dampers; their target is each hinge's start angle.
            actuators = {h.name: ImplicitActuatorCfg(joint_names_expr=[f"{h.name}_joint"], stiffness=h.rate,
                                                     damping=h.damping, friction=h.friction,
                                                     armature=h.armature) for h in spec.hinges}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {f"{h.name}_joint": h.start for h in spec.hinges}
            cfg.init_state.joint_vel = {".*": 0.0}
            return cfg

    kind = ObjectType.ARTICULATION if spec.hinges else (ObjectType.BASE if spec.mass is None else ObjectType.RIGID)
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=spec.quat if spec.quat is not None else (
            0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    if spec.mass is None and not spec.hinges:
        origin=np.zeros(3)
        quat=spec.quat if spec.quat is not None else (0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))
        pose=np.r_[spec.pos,quat]
        vel=np.zeros(6)
    else:
        asset = base.scene[spec.name]
        origin = _np(base.scene.env_origins[0])
        pose = _np(asset.data.root_link_pose_w)[0]
        vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    probe = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                      low=np.zeros(3), high=np.zeros(3))
    lo, hi = np.array(spec.meta["lo"]), np.array(spec.meta["hi"])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = np.array([probe.to_world(c) for c in corners])
    joints = {}
    if spec.hinges:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        joints = {h.name: float(q[names.index(f"{h.name}_joint")]) for h in spec.hinges}
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), joints=joints)


def set_targets(environment, name: str, targets: dict[str, float]) -> None:
    """Set drive targets (rad) of some hinges of an articulated part (env 0)."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    ids = [names.index(f"{k}_joint") for k in targets]
    t = torch.tensor([list(targets.values())], dtype=torch.float32, device=asset.device)
    asset.set_joint_position_target(t, joint_ids=ids)


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def gripper_points(environment) -> list[np.ndarray]:
    """World points (env frame) on both grippers: the palm, the centre between the finger pads and each pad tip."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = []
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        names = list(robot.body_names)
        g = _np(robot.data.body_link_pose_w)[0][names.index("gripper")]
        R, p = _rot(g[3:7]), g[:3] - origin
        jn = list(robot.joint_names)
        half = abs(float(_np(robot.data.joint_pos)[0][jn.index("joint7")])) + 0.008
        for local in ((0, 0, 0.06), (0, 0, 0.134), (half, 0, 0.147), (-half, 0, 0.147), (half, 0, 0.12),
                      (-half, 0, 0.12)):
            out.append(p + R @ np.array(local, float))
    return out


# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset, once objects have settled."""
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_furniture_sliders_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _part_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    original_build = scene.build_task_environment

    def build_task_environment(embodiment, task, control_hz=None):
        if "furniture" in task.names:
            from adapter import add_sensors
            add_sensors(embodiment)
        environment=original_build(embodiment, task, control_hz)
        if 'furniture' in task.names:
            from config import NUMERICS,HZ
            native=environment.env_cfg_callback
            def configure(cfg):
                cfg=native(cfg)
                cfg.sim.dt=1/NUMERICS['physics_hz']
                cfg.decimation=NUMERICS['physics_hz']//HZ
                cfg.sim.render_interval=cfg.decimation
                return cfg
            environment.env_cfg_callback=configure
        return environment

    scene.build_task_environment = build_task_environment
    scene.task_object = task_object
    scene.object_states = object_states
    scene._furniture_sliders_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_furniture_sliders_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        if "furniture" not in self.task.names:
            return base_task_step(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        if self.task is None or "furniture" not in self.task.names:
            return obs
        _ENV["env"] = self._env
        for spec in _SPECS.values():  # drive targets back to each hinge's start angle (springs, dampers)
            if spec.hinges:
                set_targets(self._env, spec.name, {h.name: h.rest for h in spec.hinges})
        for fn in _RESET:
            fn(self._env)
        return self._observation()

    cls._task_step, cls.reset = _task_step, reset
    cls._furniture_sliders_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
