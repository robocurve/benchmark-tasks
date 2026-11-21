"""Native rigid parts for the egg task, adapted from toilet-roll-spindle's rollkit.

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body made of several box, cylinder and
sphere colliders (a shape spec may end with the string "visual" to make it visual only). Give it ``hinges`` and it
becomes an articulation: each hinge is a link on a revolute joint about an axis of the part frame, or on a prismatic
joint along it (``kind="prismatic"``), hung from the part's body or from another hinge, with optional limits and a
spring and damper drive (a spring toward its rest position, or a pure damper). A part without a mass is a fixed base:
its body is fixed to the world; a part with a mass and hinges is a free articulation that can be picked up.

Importing this module hooks ``yam_arena.scene`` and the embodiment without
modifying the installed adapter. The task module gets its post-step and
post-reset callbacks only when its parts are present.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
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
    spheres: tuple = ()
    kind: str = "revolute"  # or "prismatic": then lower/upper/start are metres, rate N/m, damping N s/m
    rest: float | None = None  # where the spring pulls it (defaults to start)
    parent: str = ""  # "" hangs it from the part's body, otherwise from the hinge of that name
    lower: float | None = None  # rad
    upper: float | None = None
    start: float = 0.0
    rate: float = 0.0
    damping: float = 0.0
    mass: float = 0.05
    friction: float = 0.0


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is its bounding box
    and ``pos`` is the body origin. ``mass=None`` fixes its body to the world."""

    boxes: tuple = ()
    cylinders: tuple = ()
    spheres: tuple = ()
    hinges: tuple[Hinge, ...] = ()
    friction: float = 0.8
    body_mass: float = 1.0
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not (self.boxes or self.cylinders or self.spheres):
            raise ValueError(f"{self.name}: a Part needs at least one shape")


def _visual(spec) -> bool:
    return spec[-1] == "visual"


def _cyl(spec):
    centre, radius, height, color = spec[:4]
    axis = spec[4] if len(spec) > 4 and spec[4] in ("X", "Y", "Z") else "Z"
    return centre, radius, height, color, axis


def _rest(h) -> float:
    return h.start if h.rest is None else h.rest


def _cyl_half(spec):
    _, r, h, _, axis = _cyl(spec)
    return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])


def part(name, pos, boxes=(), mass=None, yaw=0.0, hinges=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         body_mass=1.0, spheres=()) -> Part:
    """Build a Part; ``size`` is filled in from the body's shapes."""
    half = ([np.array(b[1]) / 2 for b in boxes] + [_cyl_half(c) for c in cylinders]
            + [np.full(3, sp[1]) for sp in spheres])
    centres = [np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cylinders] + [np.array(sp[0]) for sp in spheres]
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), cylinders=tuple(cylinders), spheres=tuple(spheres), hinges=tuple(hinges), friction=friction,
                body_mass=body_mass if mass is None else mass)


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
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
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
        rb.CreateAngularDampingAttr(0.5)
        return prim

    def collide(prim, color, visual=False):
        if visual:
            bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
            return
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.003)
        cp.CreateRestOffsetAttr(0.0)
        # contacts with finger pads act over a patch, so they resist twisting about the contact normal
        cp.CreateTorsionalPatchRadiusAttr(0.010)
        cp.CreateMinTorsionalPatchRadiusAttr(0.005)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def shapes(path: str, box_specs, cyl_specs, offset=(0.0, 0.0, 0.0), sph_specs=()):
        off = np.array(offset, float)
        for i, b in enumerate(box_specs):
            centre, size, color = b[:3]
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            if len(b) > 3 and b[3] != "visual":   # optional rotation (rad) about the part's y axis, or ("Z", rad)
                axis, ang = b[3] if isinstance(b[3], tuple) else ("Y", b[3])
                (cube.AddRotateZOp() if axis == "Z" else cube.AddRotateYOp()).Set(math.degrees(ang))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color, _visual(b))
        for i, sp in enumerate(sph_specs):
            centre, radius, color = sp[:3]
            ball = UsdGeom.Sphere.Define(stage, f"{path}/sph_{i}")
            ball.CreateRadiusAttr(radius)
            ball.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            collide(ball.GetPrim(), color, _visual(sp))
        for i, c in enumerate(cyl_specs):
            centre, radius, height, color, axis = _cyl(c)
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(axis)
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            collide(cyl.GetPrim(), color, _visual(c))

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.body_mass)
    shapes(main, spec.boxes, spec.cylinders, sph_specs=spec.spheres)
    paths = {"": main}
    for h in spec.hinges:
        path = f"{prim_path}/{h.name}"
        paths[h.name] = path
        body(path, h.anchor, h.mass)
        shapes(path, h.boxes, h.cylinders, offset=h.anchor, sph_specs=h.spheres)
        parent_anchor = _link_anchor(spec, h.parent)
        prismatic = h.kind == "prismatic"
        jcls = UsdPhysics.PrismaticJoint if prismatic else UsdPhysics.RevoluteJoint
        j = jcls.Define(stage, f"{path}/{h.name}_joint")
        j.CreateAxisAttr(h.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[h.parent])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(h.anchor) - parent_anchor)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        conv = (lambda v: v) if prismatic else math.degrees
        per = 1.0 if prismatic else math.pi / 180
        kind = "linear" if prismatic else "angular"
        if h.lower is not None:
            j.CreateLowerLimitAttr(conv(h.lower))
            j.CreateUpperLimitAttr(conv(h.upper))
        # USD angular drives are per degree; the spring pulls toward the rest position
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate * per)
        drive.CreateDampingAttr(h.damping * per)
        drive.CreateTargetPositionAttr(conv(_rest(h)))
        state = PhysxSchema.JointStateAPI.Apply(j.GetPrim(), kind)
        state.CreatePositionAttr(conv(h.start))
        state.CreateVelocityAttr(0.0)
    if spec.mass is None and not spec.hinges:   # a fixed part without links: a kinematic body that stays put
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(main)).CreateKinematicEnabledAttr(True)
    elif spec.mass is None:
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
    if decorate := spec.meta.get("decorate"):
        decorate(stage, main)
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
                                                     damping=h.damping, friction=h.friction) for h in spec.hinges}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {f"{h.name}_joint": h.start for h in spec.hinges}
            cfg.init_state.pos = tuple(spec.pos)
            cfg.init_state.joint_vel = {".*": 0.0}
            return cfg

    kind = ObjectType.ARTICULATION if spec.hinges else ObjectType.RIGID
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


def finger_points(environment) -> list:
    """Points on both arms' finger pads (world, relative to the env origin): at the TCP and at the pad tips, 6 mm
    outside each pad face. For "released" checks in an ``on_step`` hook; score() does not see the robot."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        jp, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        half_gap = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8") if j in jn)
        r = PartState(pos=pose[:3], quat_xyzw=pose[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3),
                      high=np.zeros(3)).rotation()
        for along in (0.0, PAD_TIP_PAST_TCP):
            centre = pose[:3] + r @ (np.array(TCP_OFFSET) + [0.0, 0.0, along]) - origin
            pts += [centre + r[:, 0] * side * (half_gap + 0.006) for side in (1.0, -1.0)]
    return pts


# -- hooking yam_arena -------------------------------------------------------------------------------------------

def _patch_scene(scene) -> None:
    if getattr(scene, "_egg_parts_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        _SPECS.pop(spec.name, None)
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
    scene._egg_parts_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_egg_parts_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        if any(isinstance(spec, Part) for spec in self.task.objects):
            self.task.module._step(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        if self.task is None or not any(isinstance(spec, Part) for spec in self.task.objects):
            return obs
        for spec in self.task.objects:
            if isinstance(spec, Part) and spec.hinges:
                set_targets(self._env, spec.name, {h.name: _rest(h) for h in spec.hinges})
        self.task.module._reset(self._env)
        return self._observation()

    cls._task_step, cls.reset = _task_step, reset
    cls._egg_parts_patched = True


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
