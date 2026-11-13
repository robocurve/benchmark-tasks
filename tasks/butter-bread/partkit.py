"""partkit: rigid parts built from several boxes, optionally with jointed sliders and turning parts, for yam_arena task files.

Extended from the copy in tasks/dual-latch-lid: a ``Slider`` can also be a revolute joint (``joint="revolute"``), can
hang from another slider (``parent``), and can take explicit limits (or none). Parts and sliders can also carry
``decals``: named boxes that are drawn but do not collide, and that ``set_visible`` shows or hides during a run. A part
made with ``contact_report=True`` reports its contacts: ``Contacts`` reads the forces (and points) that given other
bodies, such as the robot's links, put on its links. ``push`` applies a force to one of a part's links.

A task file uses it like this (keep this file next to the task file):

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from partkit import Part, Slider, on_step, on_reset

    LID = Part("lid", pos=(0.2, 0.0, 0.76), yaw=0.0, mass=0.04,
               boxes=[((0, 0, 0), (0.114, 0.064, 0.0075), (0.1, 0.3, 0.9))])
    OBJECTS = [LID, ...]

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body with several box and cylinder colliders
(a box with a pocket, a lid with tabs, a cap with lugs). Give it ``sliders`` and it becomes an articulation whose root is the part and whose
sliders are shapes on prismatic joints along one of the part's axes, each pushed to its rest stop by a spring drive.

Importing this module hooks ``yam_arena.scene`` (and the embodiment's step and reset) without modifying yam_arena: ``success()/score()`` receive a :class:`PartState` (the body origin's pose, not the bounding-box centre that
yam_arena reports for primitives, plus slider positions). ``on_step`` and ``on_reset`` register callbacks that run
after every control step and after every reset, with the Isaac Lab environment.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
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

__all__ = ["Part", "PartState", "Slider", "Contacts", "on_reset", "on_step", "env", "write_pose", "set_visible", "push",
           "install"]

Color = tuple[float, float, float]
BoxSpec = tuple[tuple[float, float, float], tuple[float, float, float], Color]  # centre, size, colour (part frame, m)
CylSpec = tuple  # (centre, radius, height, colour[, axis "X" | "Y" | "Z" = "Z"]) in the part frame
DecalSpec = tuple  # (name, centre, size, colour, visible at start) in the part (or slider) frame: drawn, never collides


@dataclass(frozen=True)
class Slider:
    """Shapes on a prismatic joint along the part's ``axis`` ("X", "Y" or "Z"). ``rest`` is where its frame sits in
    the part frame; it can move by ``travel`` in the axis's negative direction (``sign=1``: positive), and a spring of
    ``rate`` N/m pushes it back to ``rest``."""

    name: str
    rest: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple = ()
    travel: float = 0.004
    rate: float = 2500.0
    damping: float = 20.0
    mass: float = 0.002
    axis: str = "Z"
    sign: int = -1
    joint: str = "prismatic"  # or "revolute": then travel, limits, rate and damping are in rad and N m / rad
    parent: str | None = None  # another slider's name to hang from; None hangs from the part's body
    limits: tuple | None = ()  # () derives (lower, upper) from travel and sign; None leaves the joint unlimited
    friction: float | None = None          # static joint friction (PhysX: a coefficient on the joint's transmitted force)
    dynamic_friction: float | None = None  # dynamic joint friction once moving
    decals: tuple = ()                     # DecalSpecs riding on this slider, centres in the part frame
    surface_friction: float | None = None  # this link's own contact friction (None: the part's ``friction``)
    surface_combine: str = "average"       # PhysX friction combine mode for that material ("min" makes it win)

    @property
    def joint_name(self) -> str:
        return f"{self.name}_{'turn' if self.joint == 'revolute' else 'slide'}"

    def joint_limits(self):
        if self.limits is None:
            return None
        if self.limits:
            return tuple(self.limits)
        return (-self.travel, 0.0) if self.sign < 0 else (0.0, self.travel)


@dataclass(frozen=True)
class Part(Box):
    """A rigid part made of ``boxes``. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is
    its bounding box and ``pos`` is the body origin."""

    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple[CylSpec, ...] = ()
    sliders: tuple[Slider, ...] = ()
    friction: float = 0.8
    decals: tuple = ()
    contact_report: bool = False
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.boxes and not self.cylinders:
            raise ValueError(f"{self.name}: a Part needs at least one box or cylinder")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Part must be loose (give it a mass)")


def part(name, pos, boxes, mass, yaw=0.0, sliders=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         decals=(), contact_report=False) -> Part:
    """Build a Part; ``size`` is filled in from the shapes' extent."""
    def cyl_half(spec):
        r, h, axis = spec[1], spec[2], spec[4] if len(spec) > 4 else "Z"
        return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])

    half = [np.array(s) / 2 for _, s, _ in boxes] + [cyl_half(c) for c in cylinders]
    centres = [np.array(c) for c, _, _ in boxes] + [np.array(c[0]) for c in cylinders]
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), cylinders=tuple(cylinders), sliders=tuple(sliders), friction=friction,
                decals=tuple(decals), contact_report=contact_report)


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each slider's position (m, 0 at its
    rest stop, negative when pressed)."""

    sliders: dict = field(default_factory=dict)

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
_DECALS: dict[str, dict[str, str]] = {}  # part name -> decal name -> prim path below the part's root prim


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the body (and sliders) as siblings."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(path: str, pos, mass: float):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateAngularDampingAttr(1.0)  # light damping: small stiff parts otherwise chatter in tight fits
        if spec.contact_report:
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    def collide(prim, color, material=None):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0005)  # below the 1 mm fits, so tight parts do not touch on every face at once
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material or phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def boxes(path: str, specs, offset=(0.0, 0.0, 0.0), material=None):
        for i, (centre, size, color) in enumerate(specs):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color, material)

    def cylinders(path: str, specs, offset=(0.0, 0.0, 0.0), material=None):
        for i, spec in enumerate(specs):
            centre, radius, height, color = spec[:4]
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(spec[4] if len(spec) > 4 else "Z")
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            collide(cyl.GetPrim(), color, material)

    def decals(path: str, specs, offset=(0.0, 0.0, 0.0)):
        for name, centre, size, color, shown in specs:
            cube = UsdGeom.Cube.Define(stage, f"{path}/decal_{name}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            bind_visual_material(str(cube.GetPath()), look(color), stage=stage)
            if not shown:
                UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
            _DECALS.setdefault(spec.name, {})[name] = str(cube.GetPath())[len(prim_path):]

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.mass)
    boxes(main, spec.boxes)
    cylinders(main, spec.cylinders)
    decals(main, spec.decals)
    rests = {s.name: s.rest for s in spec.sliders}
    for s in spec.sliders:
        path = f"{prim_path}/{s.name}"
        body(path, s.rest, s.mass)
        material = None
        if s.surface_friction is not None:  # the link's own contact material
            material = f"{prim_path}/Looks/physics_{s.name}"
            lm = sim_utils.RigidBodyMaterialCfg(static_friction=s.surface_friction,
                                                dynamic_friction=s.surface_friction,
                                                friction_combine_mode=s.surface_combine)
            lm.func(material, lm)
        boxes(path, s.boxes, offset=s.rest, material=material)
        cylinders(path, s.cylinders, offset=s.rest, material=material)
        decals(path, s.decals, offset=s.rest)
        revolute = s.joint == "revolute"
        Joint = UsdPhysics.RevoluteJoint if revolute else UsdPhysics.PrismaticJoint
        j = Joint.Define(stage, f"{path}/{s.joint_name}")
        j.CreateAxisAttr(s.axis)
        parent_path = main if s.parent is None else f"{prim_path}/{s.parent}"
        parent_rest = np.zeros(3) if s.parent is None else np.asarray(rests[s.parent], float)
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.asarray(s.rest, float) - parent_rest).tolist()))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        lim = s.joint_limits()
        if lim is not None:  # USD revolute limits are in degrees
            k = math.degrees(1.0) if revolute else 1.0
            j.CreateLowerLimitAttr(lim[0] * k)
            j.CreateUpperLimitAttr(lim[1] * k)
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if revolute else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(s.rate * (math.pi / 180 if revolute else 1.0))  # USD angular gains are per degree
        drive.CreateDampingAttr(s.damping * (math.pi / 180 if revolute else 1.0))
        drive.CreateTargetPositionAttr(0.0)
    if spec.sliders:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
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
            # Implicit actuators carry the springs; their default target is each slider's rest stop.
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[s.joint_name], stiffness=s.rate,
                                                     damping=s.damping, friction=s.friction,
                                                     dynamic_friction=s.dynamic_friction) for s in spec.sliders}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    kind = ObjectType.ARTICULATION if spec.sliders else ObjectType.RIGID
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2),
                                                                       math.cos(spec.yaw / 2))),
    )


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _anp(x) -> np.ndarray:
    """A torch tensor or a warp array (as the PhysX tensor views return) as a float64 numpy array."""
    if hasattr(x, "detach"):
        return _np(x)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    probe = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                      low=np.zeros(3), high=np.zeros(3))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = np.array([probe.to_world(c) for c in corners])
    sliders = {}
    if spec.sliders:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        sliders = {s.name: float(q[names.index(s.joint_name)]) for s in spec.sliders}
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), sliders=sliders)


def write_pose(environment, name: str, pos, quat_xyzw, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)) -> None:
    """Teleport a part's body (env 0) to a world pose (relative to the env origin) with the given velocity."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins[0])
    pose = torch.tensor([*pos, *quat_xyzw], dtype=origin.dtype, device=origin.device).unsqueeze(0)
    pose[:, :3] += origin
    asset.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([*lin_vel, *ang_vel], dtype=origin.dtype, device=origin.device).unsqueeze(0)
    asset.write_root_velocity_to_sim_index(root_velocity=v)


def set_visible(part_name: str, decal_names, visible: bool) -> None:
    """Show or hide some of a part's decals (env 0) during a run. The part is spawned once and cloned into each
    environment, so the decal is looked up under env 0's copy of the part."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    root = _env0(_ENV["env"].unwrapped.scene[part_name].cfg.prim_path)
    paths = _DECALS.get(part_name, {})
    for n in ([decal_names] if isinstance(decal_names, str) else decal_names):
        prim = stage.GetPrimAtPath(root + paths[n])
        if not prim.IsValid():
            raise KeyError(f"no decal prim at {root + paths[n]}")
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if visible else img.MakeInvisible()


def _env0(expr: str) -> str:
    import re

    return re.sub(r"env_(\.\*|\[\^/\]\+)", "env_0", expr)


def push(environment, part_name: str, link: str, force_world) -> None:
    """Apply a constant force (N, world frame, at the link's centre of mass) to one link of a part (env 0) until the
    next call; ``link`` is "body" for the part's own body, or a slider's name."""
    import torch

    asset = environment.unwrapped.scene[part_name]
    names = list(asset.body_names)
    ref = _t(asset.data.root_link_pose_w)
    f = torch.tensor([[list(force_world)]], dtype=ref.dtype, device=ref.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=f, torques=torch.zeros_like(f), body_ids=[names.index(link)], is_global=True)


class Contacts:
    """Contact forces that the rigid bodies under ``filter_roots`` (prim paths below env 0, e.g. "left_robot") put on
    some links of a part made with ``contact_report=True``. Built lazily on first use, after the simulation starts."""

    def __init__(self, part_name: str, links, filter_roots, max_points: int = 256):
        self.part_name, self.links, self.filter_roots, self.max_points = part_name, list(links), list(filter_roots), max_points
        self.view = None
        self.filters: list[str] = []

    def _build(self, environment):
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics

        stage = get_current_stage()
        root = _env0(environment.unwrapped.scene[self.part_name].cfg.prim_path)
        bodies = [f"{root}/{link}" for link in self.links]
        filters = []
        for r in self.filter_roots:  # a scene asset's name (its prim path from the scene), or a path below env 0
            try:
                path = _env0(environment.unwrapped.scene[r].cfg.prim_path)
            except (KeyError, AttributeError):
                path = f"/World/envs/env_0/{r}"
            base = stage.GetPrimAtPath(path)
            for prim in ([base] if base.IsValid() else []) + (list(base.GetAllChildren()) if base.IsValid() else []):
                stack = [prim]
                while stack:
                    p = stack.pop()
                    if p.HasAPI(UsdPhysics.RigidBodyAPI):
                        filters.append(str(p.GetPath()))
                    stack.extend(p.GetChildren())
        self.filters = sorted(set(filters))
        if not self.filters:
            raise RuntimeError(f"Contacts({self.part_name}): no rigid bodies under {self.filter_roots}")
        print(f"partkit: contacts on {self.part_name} {self.links} from {len(self.filters)} bodies", flush=True)
        sim_view = PhysxManager.get_physics_sim_view()
        self.view = sim_view.create_rigid_contact_view(bodies, filter_patterns=[self.filters] * len(bodies),
                                                       max_contact_data_count=self.max_points * len(bodies))

    def read(self, environment, dt: float):
        """(force matrix (links, filters, 3) in N, contact points per link as a list of (point_world, normal force N,
        filter index)). Points are relative to the env origin."""
        if self.view is None:
            self._build(environment)
        fm = _anp(self.view.get_contact_force_matrix(dt=dt)).reshape(len(self.links), len(self.filters), 3)
        forces, points, _normals, _sep, counts, starts = self.view.get_contact_data(dt=dt)
        forces, points = _anp(forces).reshape(-1), _anp(points).reshape(-1, 3)
        counts, starts = _anp(counts).astype(int), _anp(starts).astype(int)
        counts = counts.reshape(len(self.links), len(self.filters))
        starts = starts.reshape(len(self.links), len(self.filters))
        origin = _np(environment.unwrapped.scene.env_origins[0])
        per_link = []
        for i in range(len(self.links)):
            pts = []
            for j in range(len(self.filters)):
                for k in range(starts[i, j], starts[i, j] + counts[i, j]):
                    pts.append((points[k] - origin, float(forces[k]), j))
            per_link.append(pts)
        return fm, per_link


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
    if getattr(scene, "_partkit_patched", False):
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

    scene.task_object = task_object
    scene.object_states = object_states
    scene._partkit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_partkit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._partkit_patched = True


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
