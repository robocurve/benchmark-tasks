"""A physical capsule-chain rope for the current PhysX YAM task adapter.

Integration reference (independently implemented): Robocurve's accepted cablekit,
https://github.com/robocurve/benchmark-tasks/blob/7df8bd8ad8cb19df51718e5ff46d4d610a9da6f1/tasks/cable-through-panel-grommet/cablekit.py
It establishes the Box subclass and scene hook convention. This implementation
uses maximal-coordinate rigid bodies instead of an articulation, so each link's
authored pose can reset a curved rope without inventing a curved spring rest pose.

Every pair of consecutive nodes defines one capsule. Native D6 joints lock all
translation, leave axial twist free, and apply two bending spring/damper drives
toward straight alignment. Only adjacent links have collision filtering. No
policy-triggered attachments, kinematic grasps or per-step pose writes are
used. An optional standing-end clamp is fixed at scene construction. Separate
assessor load probes are explicitly marked diagnostic. A reset event is the only operation that writes rope body poses.

These are rigid segments, not a model of braided fibres, stretch, plasticity or
torsional stiffness. Parameters are engineering starting values, not calibrated
rope measurements. Knot validity and retention require physical validation.

Usage: ``ROPE = rope('cord', nodes=nodes); OBJECTS = [ROPE, ...]; install()``.
Nodes must have equal consecutive spacing. ``polyline=`` resamples a polyline by
fixed chord length and trims its final incomplete segment. All coordinates and
quaternions are metres and xyzw, matching the retained Isaac Lab 3 API.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState


def _points(value) -> np.ndarray:
    a = np.asarray(value, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != 3 or len(a) < 3 or not np.isfinite(a).all():
        raise ValueError("Rope needs at least three finite xyz nodes")
    return a


def resample_polyline(points, segment: float = 0.01) -> np.ndarray:
    """Sample forward along a polygonal path with equal Euclidean chord lengths.

    The final incomplete segment is discarded; corners may be cut by the chords.
    This is initial-layout construction, never a simulation-time operation.
    """
    p = _points(points)
    if not np.isfinite(segment) or segment <= 0:
        raise ValueError("segment must be finite and positive")
    if np.any(np.linalg.norm(np.diff(p, axis=0), axis=1) < 1e-10):
        raise ValueError("Polyline has repeated consecutive nodes")
    result = [p[0].copy()]
    edge, cursor = 0, p[0].copy()
    while edge < len(p) - 1:
        delta = p[edge + 1] - cursor
        distance = np.linalg.norm(delta)
        if distance < 1e-12:
            edge += 1
            if edge < len(p) - 1:
                cursor = p[edge].copy()
            continue
        direction = delta / distance
        rel = cursor - result[-1]
        b = float(np.dot(rel, direction))
        disc = b * b + segment * segment - float(np.dot(rel, rel))
        travel = -b + math.sqrt(max(0.0, disc)) if disc >= -1e-12 else math.inf
        if 1e-10 < travel <= distance + 1e-10:
            cursor = cursor + min(travel, distance) * direction
            result.append(cursor.copy())
        else:
            edge += 1
            if edge < len(p) - 1:
                cursor = p[edge].copy()
    if len(result) < 3:
        raise ValueError("Polyline is too short for two complete rope links")
    return np.asarray(result)


def axes_from_quats(quats_xyzw) -> np.ndarray:
    """World directions of each body's local +X axis; normalize measured quats."""
    q = np.asarray(quats_xyzw, dtype=np.float64)
    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    if q.shape[-1] != 4 or not np.isfinite(q).all() or np.any(norms < 1e-10):
        raise ValueError("Invalid measured rope quaternion")
    x, y, z, w = np.moveaxis(q / norms, -1, 0)
    return np.stack((1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)), axis=-1)


def _x_to_direction(direction: np.ndarray) -> np.ndarray:
    d = np.asarray(direction, dtype=np.float64)
    d = d / np.linalg.norm(d)
    if d[0] < -1 + 1e-10:
        return np.array((0.0, 0.0, 1.0, 0.0))
    q = np.array((0.0, -d[2], d[1], 1 + d[0]))
    return q / np.linalg.norm(q)


@dataclass(frozen=True)
class Rope(Box):
    """Fixed starting node layout; inherited Box bounds are loader metadata only."""

    nodes: tuple[tuple[float, float, float], ...] = ()
    radius: float = 0.004
    rope_mass: float = 0.025
    bend_stiffness: float = 0.0002  # N m/rad, per joint
    bend_damping: float = 0.000001  # N m s/rad, per joint
    max_bend: float = 1.50  # radians per swing axis
    friction: float = 0.8
    contact_offset: float = 0.0005
    linear_damping: float = 0.05
    angular_damping: float = 0.05
    solver_position_iterations: int = 64
    solver_velocity_iterations: int = 16
    self_collision: bool = True
    enable_ccd: bool = True
    anchor_start: bool = False
    start_color: tuple[float, float, float] | None = None
    end_color: tuple[float, float, float] | None = None

    @property
    def free(self) -> bool:
        # Prevent the adapter's rigid-root jitter; these are still dynamic bodies.
        return False

    @property
    def segments(self) -> int:
        return len(self.nodes) - 1

    @property
    def seg_len(self) -> float:
        return float(np.linalg.norm(np.asarray(self.nodes[1]) - self.nodes[0]))

    @property
    def length(self) -> float:
        """Joint-to-joint centreline length; rounded end caps extend by radius."""
        return self.segments * self.seg_len

    def initial_poses(self) -> tuple[np.ndarray, np.ndarray]:
        p = np.asarray(self.nodes, dtype=np.float64)
        return (p[:-1] + p[1:]) / 2, np.stack([_x_to_direction(d) for d in np.diff(p, axis=0)])

    def _check(self) -> None:
        super()._check()
        p = _points(self.nodes)
        lengths = np.linalg.norm(np.diff(p, axis=0), axis=1)
        if not np.allclose(lengths, lengths[0], rtol=1e-5, atol=1e-8):
            raise ValueError("Rope nodes must have equal chord lengths; use polyline= to resample")
        positive = (self.radius, self.rope_mass, self.contact_offset)
        nonnegative = (self.bend_stiffness, self.bend_damping, self.friction, self.linear_damping, self.angular_damping)
        if not all(np.isfinite(v) and v > 0 for v in positive):
            raise ValueError("Rope radius, mass and contact offset must be positive")
        if not all(np.isfinite(v) and v >= 0 for v in nonnegative):
            raise ValueError("Rope stiffness, damping and friction must be nonnegative")
        if lengths[0] <= 2 * self.radius:
            raise ValueError("Link spacing must exceed rope diameter to avoid straight nonadjacent overlap")
        if not 0 < self.max_bend < math.pi / 2:
            raise ValueError("max_bend must be between 0 and pi/2")
        if not (1 <= self.solver_position_iterations <= 255 and 0 <= self.solver_velocity_iterations <= 255):
            raise ValueError("Invalid solver iteration count")
        for color in (self.start_color, self.end_color):
            if color is not None and (len(color) != 3 or not all(np.isfinite(v) and 0 <= v <= 1 for v in color)):
                raise ValueError("Endpoint colors must be three values in [0,1]")


def rope(name: str, *, nodes=None, polyline=None, start=(0.15, -0.20, 0.752),
         length: float = 0.70, yaw: float = math.pi / 2, segment: float = 0.01,
         radius: float = 0.004, color=(0.12, 0.35, 0.85), **kwargs) -> Rope:
    """Construct a rope from equal-length nodes, a polyline, or a straight layout."""
    if nodes is not None and polyline is not None:
        raise ValueError("Specify nodes or polyline, not both")
    if "mass" in kwargs:
        if "rope_mass" in kwargs:
            raise ValueError("Specify mass or rope_mass, not both")
        kwargs["rope_mass"] = kwargs.pop("mass")
    if polyline is not None:
        p = resample_polyline(polyline, segment)
    elif nodes is not None:
        p = _points(nodes)
    else:
        if not np.isfinite(length) or length <= 0 or not np.isfinite(segment) or segment <= 0:
            raise ValueError("length and segment must be positive")
        n = max(2, round(length / segment))
        p = np.asarray(start, dtype=float) + np.linspace(0, length, n + 1)[:, None] * np.array((math.cos(yaw), math.sin(yaw), 0.0))
    lo, hi = p.min(axis=0) - radius, p.max(axis=0) + radius
    spec = Rope(name=name, pos=tuple((lo + hi) / 2), size=tuple(hi - lo),
                color=tuple(color), nodes=tuple(map(tuple, p)), radius=radius, **kwargs)
    spec._check()
    return spec


@dataclass(frozen=True)
class RopeState(ObjectState):
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    quats_xyzw: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))
    linear_velocities: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    angular_velocities: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    endpoints: np.ndarray = field(default_factory=lambda: np.empty((0, 2, 3)))
    joint_gaps: np.ndarray = field(default_factory=lambda: np.empty(0))
    seg_len: float = 0.0
    radius: float = 0.0

    @property
    def start_tip(self) -> np.ndarray:
        return self.endpoints[0, 0]

    @property
    def end_tip(self) -> np.ndarray:
        return self.endpoints[-1, 1]

    @property
    def max_joint_gap(self) -> float:
        return float(np.max(self.joint_gaps, initial=0.0))

    def polyline(self) -> np.ndarray:
        """Joint midpoints plus end nodes; inspect joint_gaps before using topology."""
        joints = (self.endpoints[:-1, 1] + self.endpoints[1:, 0]) / 2
        return np.vstack((self.start_tip, joints, self.end_tip))

    def resting(self, max_speed: float = 0.01, max_spin: float = 0.5) -> bool:
        return bool(np.all(np.linalg.norm(self.linear_velocities, axis=1) < max_speed)
                    and np.all(np.linalg.norm(self.angular_velocities, axis=1) < max_spin))

    def to_record(self) -> dict:
        """JSON-compatible measured state; no averaged data replaces individual links."""
        return {
            "points": self.points.tolist(), "quats_xyzw": self.quats_xyzw.tolist(),
            "linear_velocities": self.linear_velocities.tolist(),
            "angular_velocities": self.angular_velocities.tolist(),
            "endpoints": self.endpoints.tolist(), "joint_gaps": self.joint_gaps.tolist(),
            "max_joint_gap": self.max_joint_gap, "seg_len": self.seg_len, "radius": self.radius,
            "polyline": self.polyline().tolist(), "start_tip": self.start_tip.tolist(),
            "end_tip": self.end_tip.tolist(), "low": self.low.tolist(), "high": self.high.tolist(),
        }


def state_from_arrays(spec: Rope, positions, quats_xyzw, linear_velocities, angular_velocities) -> RopeState:
    """Pure NumPy telemetry conversion, shared by live capture and offline checks."""
    pos, q, vel, ang = (np.asarray(v, dtype=np.float64) for v in
                        (positions, quats_xyzw, linear_velocities, angular_velocities))
    n = spec.segments
    if pos.shape != (n, 3) or q.shape != (n, 4) or vel.shape != (n, 3) or ang.shape != (n, 3):
        raise ValueError("Incomplete rope link telemetry")
    if not all(np.isfinite(v).all() for v in (pos, q, vel, ang)):
        raise ValueError("Nonfinite rope link telemetry")
    axes = axes_from_quats(q)
    ends = np.stack((pos - axes * spec.seg_len / 2, pos + axes * spec.seg_len / 2), axis=1)
    gaps = np.linalg.norm(ends[:-1, 1] - ends[1:, 0], axis=1)
    lo, hi = ends.min(axis=(0, 1)) - spec.radius, ends.max(axis=(0, 1)) + spec.radius
    return RopeState(pos=(lo + hi) / 2, quat_xyzw=np.array((0., 0., 0., 1.)),
                     lin_vel=vel.mean(axis=0), ang_vel=ang.mean(axis=0), low=lo, high=hi,
                     points=pos, quats_xyzw=q, linear_velocities=vel, angular_velocities=ang,
                     endpoints=ends, joint_gaps=gaps, seg_len=spec.seg_len, radius=spec.radius)


_SPECS: dict[str, Rope] = {}


def _spawn_link(prim_path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.rope_name]
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
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.rope_mass / spec.segments)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateEnableCCDAttr(spec.enable_ccd)
    rb.CreateSolverPositionIterationCountAttr(spec.solver_position_iterations)
    rb.CreateSolverVelocityIterationCountAttr(spec.solver_velocity_iterations)
    rb.CreateLinearDampingAttr(spec.linear_damping)
    rb.CreateAngularDampingAttr(spec.angular_damping)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSleepThresholdAttr(0.0)
    if index == 0 and spec.anchor_start:
        # A permanent, declared standing-end clamp. This is authored once at
        # construction, never enabled in response to a policy or task state.
        fixed = UsdPhysics.FixedJoint.Define(stage, root + '/standing_end_clamp')
        fixed.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        fixed.CreateLocalPos0Attr(Gf.Vec3f(*(translation or (0., 0., 0.))))
        fixed.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        fixed.CreateLocalPos1Attr(Gf.Vec3f(0., 0., 0.))
        fixed.CreateLocalRot1Attr(Gf.Quatf(1., 0., 0., 0.))
    cap = UsdGeom.Capsule.Define(stage, path + "/collision")
    cap.CreateAxisAttr("X")
    cap.CreateRadiusAttr(spec.radius)
    cap.CreateHeightAttr(spec.seg_len)
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    cp = PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim())
    cp.CreateContactOffsetAttr(spec.contact_offset)
    cp.CreateRestOffsetAttr(0.0)
    material_path = root + "/Looks/physics"
    if not stage.GetPrimAtPath(material_path).IsValid():
        material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction,
                                                 restitution=0.0)
        material.func(material_path, material)
    bind_physics_material(str(cap.GetPath()), material_path, stage=stage)
    color = spec.start_color if index == 0 and spec.start_color is not None else spec.color
    if index == spec.segments - 1 and spec.end_color is not None:
        color = spec.end_color
    visual_path = root + f"/Looks/link_{index:03d}"
    visual = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.65)
    visual.func(visual_path, visual)
    bind_visual_material(str(cap.GetPath()), visual_path, stage=stage)

    # Filtering is explicit and restricted to adjacent overlapping capsule ends.
    other_ids = [i for i in range(spec.segments) if i != index and
                 (abs(i - index) == 1 or not spec.self_collision)]
    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
        [Sdf.Path(root + f"/seg_{i:03d}") for i in other_ids])
    if index:
        joint = UsdPhysics.Joint.Define(stage, root + f"/joint_{index:03d}")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(root + f"/seg_{index - 1:03d}")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(spec.seg_len / 2, 0., 0.))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-spec.seg_len / 2, 0., 0.))
        joint.CreateLocalRot0Attr(Gf.Quatf(1., 0., 0., 0.))
        joint.CreateLocalRot1Attr(Gf.Quatf(1., 0., 0., 0.))
        joint.CreateCollisionEnabledAttr(False)
        jp = joint.GetPrim()
        for axis in ("transX", "transY", "transZ"):
            limit = UsdPhysics.LimitAPI.Apply(jp, axis)
            limit.CreateLowAttr(1.0)
            limit.CreateHighAttr(-1.0)
        # No rotX limit/drive: axial twist is physically free.
        for axis in ("rotY", "rotZ"):
            limit = UsdPhysics.LimitAPI.Apply(jp, axis)
            limit.CreateLowAttr(-math.degrees(spec.max_bend))
            limit.CreateHighAttr(math.degrees(spec.max_bend))
            drive = UsdPhysics.DriveAPI.Apply(jp, axis)
            drive.CreateTypeAttr("force")
            # USD angular drive gains are per degree, public Rope parameters per radian.
            drive.CreateStiffnessAttr(spec.bend_stiffness * math.pi / 180.)
            drive.CreateDampingAttr(spec.bend_damping * math.pi / 180.)
            drive.CreateTargetPositionAttr(0.)
            drive.CreateTargetVelocityAttr(0.)

    if index == 0 and spec.enable_ccd:
        scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if not scenes:
            raise RuntimeError("Rope CCD requested, but no physics scene exists at spawn")
        for scene in scenes:
            PhysxSchema.PhysxSceneAPI.Apply(scene).CreateEnableCCDAttr(True)
    return prim


def _tensor(value):
    return getattr(value, "torch", value)


def _first_attr(obj, *names):
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    raise AttributeError(f"{type(obj).__name__} lacks any of {names}")


def reset_rope(env, env_ids, rope_name: str):
    """Reset event only: restore every initial link pose and zero its velocity."""
    import torch

    base = env.unwrapped if hasattr(env, "unwrapped") else env
    collection = base.scene[rope_name]
    spec = _SPECS[rope_name]
    origin = base.scene.env_origins
    ids = torch.arange(len(origin), device=origin.device, dtype=torch.long) if env_ids is None else env_ids
    positions, quats = spec.initial_poses()
    pose = torch.as_tensor(np.concatenate((positions, quats), axis=1), device=origin.device, dtype=origin.dtype)
    pose = pose.unsqueeze(0).repeat(len(ids), 1, 1)
    pose[:, :, :3] += origin[ids, None, :]
    # Names are stable and ordered by the explicitly constructed cfg dictionary.
    collection.write_body_link_pose_to_sim_index(body_poses=pose, env_ids=ids)
    velocity = torch.zeros((len(ids), spec.segments, 6), device=origin.device, dtype=origin.dtype)
    collection.write_body_link_velocity_to_sim_index(body_velocities=velocity, env_ids=ids)


def _arena_asset(spec: Rope):
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset

    @configclass
    class LinkSpawnCfg(SpawnerCfg):
        func: object = clone(_spawn_link)
        rope_name: str = ""
        link_index: int = 0

    class RopeAsset(Asset):
        def __init__(self):
            super().__init__(spec.name, tags=["rope", "rigid-chain", "physx"])
            positions, quats = spec.initial_poses()
            objects = {}
            for i, (pos, quat) in enumerate(zip(positions, quats)):
                objects[f"seg_{i:03d}"] = RigidObjectCfg(
                    prim_path="{ENV_REGEX_NS}/" + spec.name + f"/seg_{i:03d}",
                    spawn=LinkSpawnCfg(rope_name=spec.name, link_index=i),
                    init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(pos), rot=tuple(quat)),
                )
            self.object_cfg = RigidObjectCollectionCfg(rigid_objects=objects)

        def get_object_cfg(self):
            return self.name, self.object_cfg

        def get_event_cfg(self):
            return self.name, EventTermCfg(func=reset_rope, mode="reset", params={"rope_name": self.name})

        def validate_simulation_cfg(self, sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics, PhysxCfg):
                raise ValueError("This maximal-coordinate rope requires PhysX")

    return RopeAsset()


def rope_state(env, name: str) -> RopeState:
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    spec, asset = _SPECS[name], base.scene[name]
    names = list(_first_attr(asset, "body_names", "object_names"))
    indices = [names.index(f"seg_{i:03d}") for i in range(spec.segments)]
    data = asset.data

    def read(*attrs):
        value = _tensor(_first_attr(data, *attrs))[0, indices]
        return value.detach().cpu().numpy().astype(np.float64)

    pos = read("body_link_pos_w", "object_link_pos_w")
    pos -= _tensor(base.scene.env_origins)[0].detach().cpu().numpy()
    return state_from_arrays(spec, pos, read("body_link_quat_w", "object_link_quat_w"),
                             read("body_link_lin_vel_w", "object_link_lin_vel_w"),
                             read("body_link_ang_vel_w", "object_link_ang_vel_w"))


def _patch_scene(scene):
    if getattr(scene, "_overhand_rope_installed", False):
        return
    original_object, original_states = scene.task_object, scene.object_states

    def task_object(spec):
        from fixtures import Peg, arena_peg
        if isinstance(spec, Peg):
            return arena_peg(spec)
        if not isinstance(spec, Rope):
            return original_object(spec)
        spec._check()
        _SPECS[spec.name] = spec
        return _arena_asset(spec)

    def object_states(env, names):
        ordinary = [n for n in names if n not in _SPECS]
        states = original_states(env, ordinary) if ordinary else {}
        states.update({n: rope_state(env, n) for n in names if n in _SPECS})
        return {n: states[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._overhand_rope_installed = True


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        original_exec = spec.loader.exec_module

        def execute(module):
            original_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = execute
        return spec


def install():
    """Install only this task's lazy scene hooks; no simulator starts here."""
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(finder, _SceneFinder) for finder in sys.meta_path):
        sys.meta_path.insert(0, _SceneFinder())


install()
