"""gatekit: a mechanism built from boxes and cylinders on a chain of hinges and slides, fixed to the world, for
yam_arena task files. Adapted from the part kit used by other tasks in this repository: the root ``Part`` is fixed in
place (a static post), and ``Link`` bodies hang from it, or from each other, on revolute ("hinge") or prismatic
("slide") joints with end stops and damping.

    from gatekit import Part, Link, part, on_reset
    POST = part("gate", pos=(0.35, -0.08, TOP), boxes=[...], links=(Link("leaf", "hinge", None, ...), ...))

Joint zero is the pose in which a link is authored. Links that are not joined directly collide with each other and
with the root (articulation self-collisions are on; PhysX skips pairs joined by a joint).

Importing this module hooks ``yam_arena.scene`` (and the embodiment's step and reset) without modifying yam_arena:
``success()/score()`` receive a :class:`PartState` (the root's pose, plus every joint's position: metres for a slide,
radians for a hinge). ``on_step`` and ``on_reset`` register callbacks that run after every control step and after
every reset, with the Isaac Lab environment.

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

__all__ = ["Part", "PartState", "Link", "part", "on_reset", "on_step", "env", "install"]

Color = tuple[float, float, float]
BoxSpec = tuple  # (centre, size, colour[, yaw]) in the part frame (m; yaw in radians about the part's z, default 0)
CylSpec = tuple  # (centre, radius, height, colour[, axis "X" | "Y" | "Z" = "Z"]) in the part frame


@dataclass(frozen=True)
class Link:
    """A rigid body on a joint. ``parent`` is another link's name (None: the root part). ``at`` and ``rot`` (w, x, y, z)
    place the link's frame in its parent's frame, as authored; the joint sits at the link's origin, along the link's
    ``axis`` ("X", "Y" or "Z"), and reads 0 in the authored pose. ``lower``/``upper`` are its end stops (m for a slide,
    rad for a hinge); ``damping`` is N s/m or N m s/rad. Shapes are in the link's frame."""

    name: str
    kind: str                      # "hinge" or "slide"
    parent: str | None
    at: tuple[float, float, float]
    axis: str = "Y"
    lower: float = 0.0
    upper: float = 0.0
    boxes: tuple = ()
    cylinders: tuple = ()
    mass: float = 0.05
    damping: float = 1.0
    rot: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    com: tuple | None = None
    friction: float | None = None


@dataclass(frozen=True)
class Part(Box):
    """A rigid part made of ``boxes``. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is
    its bounding box and ``pos`` is the body origin."""

    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple[CylSpec, ...] = ()
    links: tuple[Link, ...] = ()
    friction: float = 0.8
    fixed: bool = True
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.boxes:
            raise ValueError(f"{self.name}: a Part needs at least one box")
        if not self.fixed:
            raise ValueError(f"{self.name}: gatekit parts are fixed to the world")
        names = [ln.name for ln in self.links]
        for ln in self.links:
            if ln.parent is not None and ln.parent not in names[:names.index(ln.name)]:
                raise ValueError(f"{self.name}: link {ln.name!r} needs its parent declared before it")


def part(name, pos, boxes, links=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(), yaw=0.0) -> Part:
    """Build a Part; ``size`` is filled in from the shapes' extent."""
    def cyl_half(spec):
        r, h, axis = spec[1], spec[2], spec[4] if len(spec) > 4 else "Z"
        return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])

    half = [np.array(b[1]) / 2 for b in boxes] + [cyl_half(c) for c in cylinders]   # yawed boxes: approximate
    centres = [np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cylinders]
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=None, color=color,
                yaw=yaw,
                boxes=tuple(boxes), cylinders=tuple(cylinders), links=tuple(links), friction=friction, fixed=True)


@dataclass(frozen=True)
class PartState(ObjectState):
    """The root's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each link's joint position (``joints``: m for
    a slide, rad for a hinge, 0 as authored)."""

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


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the root body and its links as siblings."""
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

    def body(path: str, pos, mass: float | None, com=None, rot=(1.0, 0.0, 0.0, 0.0)):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*rot))
        prim = xform.GetPrim()
        if mass is None:
            return prim
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(mass)
        if com is not None:
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateAngularDampingAttr(1.0)
        return prim

    def collide(prim, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0005)  # below the 1 mm fits, so tight parts do not touch on every face at once
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def boxes(path: str, specs, offset=(0.0, 0.0, 0.0)):
        for i, spec in enumerate(specs):
            centre, size, color = spec[:3]
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            if len(spec) > 3 and spec[3]:
                cube.AddRotateZOp().Set(math.degrees(spec[3]))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color)

    def cylinders(path: str, specs, offset=(0.0, 0.0, 0.0)):
        for i, spec in enumerate(specs):
            centre, radius, height, color = spec[:4]
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(spec[4] if len(spec) > 4 else "Z")
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            collide(cyl.GetPrim(), color)

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), 1.0 if spec.links else None)
    boxes(main, spec.boxes)
    cylinders(main, spec.cylinders)
    poses = link_poses(spec)
    for ln in spec.links:
        path = f"{prim_path}/{ln.name}"
        pos, rot = poses[ln.name]
        body(path, tuple(pos), ln.mass, ln.com, tuple(_mat_to_wxyz(rot)))
        if ln.friction is not None:
            own = f"{prim_path}/Looks/physics_{ln.name}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=ln.friction, dynamic_friction=ln.friction)
            pm.func(own, pm)
        boxes(path, ln.boxes)
        cylinders(path, ln.cylinders)
        if ln.friction is not None:
            for child in stage.GetPrimAtPath(path).GetChildren():
                bind_physics_material(str(child.GetPath()), own, stage=stage)
        parent = main if ln.parent is None else f"{prim_path}/{ln.parent}"
        cls = UsdPhysics.RevoluteJoint if ln.kind == "hinge" else UsdPhysics.PrismaticJoint
        j = cls.Define(stage, f"{path}/{ln.name}_joint")
        j.CreateAxisAttr(ln.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*ln.at))
        j.CreateLocalRot0Attr(Gf.Quatf(*ln.rot))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        hinge = ln.kind == "hinge"
        j.CreateLowerLimitAttr(math.degrees(ln.lower) if hinge else ln.lower)
        j.CreateUpperLimitAttr(math.degrees(ln.upper) if hinge else ln.upper)
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if hinge else "linear")
    if spec.links:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
        art.CreateEnabledSelfCollisionsAttr(True)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
        anchor = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/anchor")
        anchor.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        ax, ay, az = translation or (0.0, 0.0, 0.0)
        anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(ax, ay, az))
        anchor.CreateLocalRot0Attr().Set(Gf.Quatf(w, x, y, z))
        anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return root.GetPrim()


def _wxyz_to_mat(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _mat_to_wxyz(r) -> np.ndarray:
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return np.array([w, (r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w),
                         (r[1, 0] - r[0, 1]) / (4 * w)])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1 + r[i, i] - r[j, j] - r[k, k])) * 2
    q = np.zeros(4)
    q[0] = (r[k, j] - r[j, k]) / s
    q[1 + i] = s / 4
    q[1 + j] = (r[j, i] + r[i, j]) / s
    q[1 + k] = (r[k, i] + r[i, k]) / s
    return q


def _axis_rot(axis: str, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return {"X": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "Y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "Z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def link_poses(spec: Part, joints: dict | None = None) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Each link's (position, rotation matrix) in the root part's frame, with joint values ``joints`` (default: as
    authored, all 0). Pure NumPy: the task's scoring uses it to turn joint readings into positions."""
    joints = joints or {}
    out: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for ln in spec.links:
        pp, pr = (np.zeros(3), np.eye(3)) if ln.parent is None else out[ln.parent]
        frame_r = pr @ _wxyz_to_mat(ln.rot)
        frame_p = pp + pr @ np.asarray(ln.at, float)
        v = float(joints.get(ln.name, 0.0))
        if ln.kind == "hinge":
            out[ln.name] = (frame_p, frame_r @ _axis_rot(ln.axis, v))
        else:
            e = {"X": 0, "Y": 1, "Z": 2}[ln.axis]
            out[ln.name] = (frame_p + frame_r[:, e] * v, frame_r)
    return out


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
            # Implicit actuators with no stiffness: each joint is free between its stops, with damping only.
            actuators = {ln.name: ImplicitActuatorCfg(joint_names_expr=[f"{ln.name}_joint"], stiffness=0.0,
                                                      damping=ln.damping) for ln in spec.links}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    kind = ObjectType.ARTICULATION if spec.links else ObjectType.BASE
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_quat(spec)),
    )


def _quat(spec: Part) -> tuple[float, float, float, float]:
    """The part's orientation (x, y, z, w): ``yaw`` about the vertical."""
    return (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _fixed_state(spec: Part) -> PartState:
    half = np.array(spec.size) / 2
    quat = _quat(spec)
    pos = np.array(spec.pos, float)
    return PartState(pos=pos, quat_xyzw=np.array(quat), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos - half,
                     high=pos + half)


def _part_state(environment, spec: Part) -> PartState:
    st = _fixed_state(spec)
    if not spec.links:
        return st
    asset = environment.unwrapped.scene[spec.name]
    names = list(asset.joint_names)
    q = _np(asset.data.joint_pos)[0]
    joints = {ln.name: float(q[names.index(f"{ln.name}_joint")]) for ln in spec.links}
    return PartState(pos=st.pos, quat_xyzw=st.quat_xyzw, lin_vel=st.lin_vel, ang_vel=st.ang_vel, low=st.low,
                     high=st.high, joints=joints)


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
    if getattr(scene, "_gatekit_patched", False):
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
    scene._gatekit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_gatekit_patched", False):
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
    cls._gatekit_patched = True


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
