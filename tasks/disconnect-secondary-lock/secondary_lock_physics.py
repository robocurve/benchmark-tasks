"""Compound connector housings, physical locks and captive four-wire pigtails.

Adapted from din-rail-mount/partkit.py.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.station import aim
from yam_arena.task_file import Box, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix

__all__ = ["Lead", "Part", "PartState", "Slider", "on_reset", "on_step", "env", "part_state"]

Color = tuple[float, float, float]
# centre, size, colour (part frame, m), and optionally the box's rotation in the part frame as a (w, x, y, z) quaternion
BoxSpec = tuple


@dataclass(frozen=True)
class Slider:
    """Geometry on a bounded joint. Rotary travel and drive gains use radians."""

    name: str
    rest: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...]
    travel: float = 0.004
    rate: float = 2500.0
    damping: float = 20.0
    mass: float = 0.002
    wedges: tuple = ()
    axis: str = "Z"
    negative: bool = True
    friction: float | None = None  # its own material, which wins its contacts; None uses the part's
    angular: bool = False


@dataclass(frozen=True)
class Lead:
    points: tuple[tuple[float, float, float], ...]
    colors: tuple[Color, ...]
    wire_radius: float = .0014
    mass: float = .011
    anchored: bool = True

    def root_offset(self):
        return np.mean(self.points[-2:], axis=0) if self.anchored else np.zeros(3)


@dataclass(frozen=True)
class Part(Box):
    """A rigid part made of ``boxes``. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is
    its bounding box and ``pos`` is the body origin."""

    boxes: tuple[BoxSpec, ...] = ()
    sliders: tuple[Slider, ...] = ()
    wedges: tuple = field(default=(), compare=False, hash=False)
    friction: float = 0.8
    lead: Lead | None = None
    mounted: bool = False

    def _check(self) -> None:
        if not self.boxes:
            raise ValueError(f"{self.name}: a Part needs at least one box")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Part needs a positive mass")
        if self.mounted and self.sliders:
            raise ValueError(f"{self.name}: mounted cable model expects a rigid housing")


def _box(spec):
    """(centre, size, colour, rotation wxyz) of a box spec, with no rotation when it has none."""
    centre, size, color = spec[:3]
    return centre, size, color, (tuple(spec[3]) if len(spec) > 3 else (1.0, 0.0, 0.0, 0.0))


def _lead_root_offset(spec):
    if spec.lead is None or spec.mounted:
        return np.zeros(3)
    x, y, z = spec.lead.root_offset()
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    return np.array([c * x - s * y, s * x + c * y, z])


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each slider's position (m, 0 at its
    rest stop, negative when pressed)."""

    sliders: dict = field(default_factory=dict)
    clip_error: float | None = None

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
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the body (and sliders) as siblings."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root_position = np.array(translation if translation is not None else (0., 0., 0.)) - _lead_root_offset(spec)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*root_position))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    materials = {}
    for s in spec.sliders:
        if s.friction is not None:
            materials[s.name] = f"{prim_path}/Looks/physics_{s.name}"
            sm = sim_utils.RigidBodyMaterialCfg(static_friction=s.friction, dynamic_friction=s.friction,
                                                friction_combine_mode="min")
            sm.func(materials[s.name], sm)
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
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateAngularDampingAttr(1.0)  # light damping: small stiff parts otherwise chatter in tight fits
        return prim

    def collide(prim, color, material=phys):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0005)  # below the 1 mm fits, so tight parts do not touch on every face at once
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def boxes(path: str, specs, offset=(0.0, 0.0, 0.0), material=phys):
        for i, (centre, size, color, rot) in enumerate(map(_box, specs)):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            if rot != (1.0, 0.0, 0.0, 0.0):
                cube.AddOrientOp().Set(Gf.Quatf(*rot))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color, material)

    main = f"{prim_path}/body"
    housing = body(main, (0.0, 0.0, 0.0), spec.mass)
    if spec.mounted:
        UsdPhysics.RigidBodyAPI(housing).CreateKinematicEnabledAttr(True)
    boxes(main, spec.boxes)

    def wedges(path, pieces, offset=(0., 0., 0.), material=phys):
        for i, (vertices, color) in enumerate(pieces):
            n = len(vertices) // 2
            faces = [tuple(range(n)), tuple(range(2 * n - 1, n - 1, -1))]
            for a in range(n):
                b = (a + 1) % n
                faces.append((a, a + n, b + n, b))
            mesh = UsdGeom.Mesh.Define(stage, f"{path}/hull_{i}")
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in np.array(vertices) - offset])
            mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
            mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
            collide(mesh.GetPrim(), color, material)
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

    wedges(main, spec.wedges)
    for s in spec.sliders:
        path = f"{prim_path}/{s.name}"
        body(path, s.rest, s.mass)
        boxes(path, s.boxes, offset=s.rest, material=materials.get(s.name, phys))
        wedges(path, s.wedges, offset=s.rest, material=materials.get(s.name, phys))
        joint_type = UsdPhysics.RevoluteJoint if s.angular else UsdPhysics.PrismaticJoint
        j = joint_type.Define(stage, f"{path}/{s.name}_joint")
        j.CreateAxisAttr(s.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(main)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*s.rest))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        limit = math.degrees(s.travel) if s.angular else s.travel
        j.CreateLowerLimitAttr(-limit if s.negative else 0.0)
        j.CreateUpperLimitAttr(0.0 if s.negative else limit)
        j.CreateCollisionEnabledAttr(False)
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if s.angular else "linear")
        drive.CreateTypeAttr("force")
        drive_scale = math.pi / 180 if s.angular else 1.
        drive.CreateStiffnessAttr(s.rate * drive_scale)
        drive.CreateDampingAttr(s.damping * drive_scale)
        drive.CreateTargetPositionAttr(0.0)

    art_root = stage.GetPrimAtPath(main)
    if spec.lead:
        lead = spec.lead
        material = f"{prim_path}/Looks/wire_physics"
        wire_material = sim_utils.RigidBodyMaterialCfg(static_friction=.40, dynamic_friction=.35)
        wire_material.func(material, wire_material)
        segments = [(main, np.zeros(3))]
        for i, (p0, p1) in enumerate(zip(lead.points, lead.points[1:])):
            start, end = np.array(p0), np.array(p1)
            center = (start + end) / 2
            length = float(np.linalg.norm(end - start))
            direction = (end - start) / length
            x_axis = np.cross(direction, [0., 0., 1.])
            x_axis /= np.linalg.norm(x_axis)
            if x_axis[0] < 0.:
                x_axis = -x_axis
            z_axis = np.array([0., 0., 1.]) - direction[2] * direction
            z_axis /= np.linalg.norm(z_axis)
            x, y, z, w = aim(direction)
            if spec.mounted:
                path = f"{main}/lead_{i}"
                section = UsdGeom.Xform.Define(stage, path)
                section.AddTranslateOp().Set(Gf.Vec3d(*center))
            else:
                path = f"{prim_path}/lead_{i}"
                wire_body = body(path, center, lead.mass / (len(lead.points) - 1))
                PhysxSchema.PhysxRigidBodyAPI(wire_body).CreateAngularDampingAttr(2.0)

            collider = UsdGeom.Capsule.Define(stage, f"{path}/bundle_collision")
            collider.CreateRadiusAttr(.00425)
            collider.CreateHeightAttr(length)
            collider.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
            collide(collider.GetPrim(), lead.colors[0], material)
            collider.MakeInvisible()
            for j, (sign_x, sign_z) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
                wire = UsdGeom.Capsule.Define(stage, f"{path}/wire_{j}")
                wire.CreateRadiusAttr(lead.wire_radius)
                wire.CreateHeightAttr(length)
                offset = sign_x * .00265 * x_axis + sign_z * .00225 * z_axis
                wire.AddTranslateOp().Set(Gf.Vec3d(*offset))
                wire.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
                bind_visual_material(str(wire.GetPath()), look(lead.colors[j]), stage=stage)

            segments.append((path, center))

        if spec.mounted:
            # The chassis mount and loom clip keep this half and its unmanipulated pigtail stationary.
            return root.GetPrim()

        for i, (a, b) in enumerate(zip(segments, segments[1:])):
            (parent, parent_center), (child, child_center) = (b, a) if lead.anchored else (a, b)
            datum = np.array(lead.points[i])
            j = UsdPhysics.Joint.Define(stage, f"{child}/bend_{i}")
            j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
            j.CreateLocalPos0Attr(Gf.Vec3f(*(datum - parent_center)))
            j.CreateLocalPos1Attr(Gf.Vec3f(*(datum - child_center)))
            j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateCollisionEnabledAttr(False)
            for dof in ("transX", "transY", "transZ"):
                limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                limit.CreateLowAttr(1.)
                limit.CreateHighAttr(-1.)
            for dof, span in (("rotX", 60.), ("rotY", 40.), ("rotZ", 60.)):
                limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                limit.CreateLowAttr(-span)
                limit.CreateHighAttr(span)
                drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), dof)
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(.006 * math.pi / 180)
                drive.CreateDampingAttr(.0008 * math.pi / 180)
                drive.CreateTargetPositionAttr(0.)

        internal = [Sdf.Path(part) for part, _ in segments]
        internal.extend(Sdf.Path(f"{prim_path}/{s.name}") for s in spec.sliders)
        for path, _ in segments[1:]:
            prim = stage.GetPrimAtPath(path)
            UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
                [target for target in internal if str(target) != path])

        if lead.anchored:
            terminal, terminal_center = segments[-1]
            anchor = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/lead_routing_clip")
            anchor.CreateCollisionEnabledAttr(False)
            anchor.CreateBody1Rel().SetTargets([Sdf.Path(terminal)])
            world = root.ComputeLocalToWorldTransform(0)
            point = world.Transform(Gf.Vec3d(*lead.points[-1]))
            rotation = world.ExtractRotationQuat()
            anchor.CreateLocalPos0Attr(Gf.Vec3f(*point))
            anchor.CreateLocalRot0Attr(Gf.Quatf(rotation.GetReal(), *rotation.GetImaginary()))
            anchor.CreateLocalPos1Attr(Gf.Vec3f(*(np.array(lead.points[-1]) - terminal_center)))
            anchor.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            art_root = anchor.GetPrim()

    if spec.sliders or spec.lead:
        UsdPhysics.ArticulationRootAPI.Apply(art_root)
        art = PhysxSchema.PhysxArticulationAPI.Apply(art_root)
        art.CreateEnabledSelfCollisionsAttr(True)
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
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[f"{s.name}_joint"], stiffness=s.rate,
                                                     damping=s.damping) for s in spec.sliders}
            if spec.lead:
                actuators["lead"] = ImplicitActuatorCfg(joint_names_expr=["bend_.*"], stiffness=.006,
                                                        damping=.0008, armature=5e-6)
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    kind = ObjectType.ARTICULATION if spec.sliders or (spec.lead and not spec.mounted) else ObjectType.RIGID
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(float(value) for value in np.array(spec.pos) + _lead_root_offset(spec)),
                          rotation_xyzw=(0., 0., math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    clip_error = None
    if spec.lead and spec.lead.anchored and not spec.mounted:
        housing = asset.body_names.index("body")
        pose = _np(asset.data.body_link_pose_w)[0, housing]
        vel = _np(asset.data.body_link_vel_w)[0, housing]
        clip = _np(asset.data.root_link_pose_w)[0]
        local = spec.lead.points[-1] - spec.lead.root_offset()
        tip = clip[:3] - origin + quat_xyzw_to_matrix(clip[3:7]) @ local
        initial_rot = np.array([0., 0., math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)])
        expected = spec.pos + quat_xyzw_to_matrix(initial_rot) @ spec.lead.points[-1]
        clip_error = float(np.linalg.norm(tip - expected))
    else:
        pose = _np(asset.data.root_link_pose_w)[0]
        vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
        if spec.lead and spec.mounted:
            terminal = np.asarray(spec.lead.points[-1])
            initial_rot = np.array([0., 0., math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)])
            expected = spec.pos + quat_xyzw_to_matrix(initial_rot) @ terminal
            actual = pose[:3] - origin + quat_xyzw_to_matrix(pose[3:7]) @ terminal
            clip_error = float(np.linalg.norm(actual - expected))
    probe = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                      low=np.zeros(3), high=np.zeros(3))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = np.array([probe.to_world(c) for c in corners])
    sliders = {}
    if spec.sliders:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        sliders = {s.name: float(q[names.index(f"{s.name}_joint")]) for s in spec.sliders}
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), sliders=sliders, clip_error=clip_error)


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
    if getattr(scene, "_secondary_patched", False):
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
                out[n] = part_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._secondary_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_secondary_patched", False):
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
    cls._secondary_patched = True


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
