"""hingekit: multi-shape parts and spring-return hinges for yam_arena task files.

Put this file next to a task file and import it there. At import it registers in-memory patches on yam_arena (nothing
on disk changes), so the standard ``inspect-robots run --embodiment yam_arena -E task=...`` command runs the task:

- yam_arena.scene.task_object spawns a ``KitPart`` (several boxes and Z-axis cylinders in one rigid body, or one
  static collider when it has no mass);
- after Isaac Lab builds the scene, each declared hinge is authored as a USD Physics RevoluteJoint between the world
  and a free part, with angle limits and a spring/damper drive (DriveAPI) toward a rest angle;
- the embodiment calls the task's ``on_step`` functions every control step, before success()/score(), and the
  ``on_reset`` functions at every episode reset.

Everything the scorer needs is computed from object poses (the ``objects`` dict a task's score() receives), so it is
geometry a reviewer can measure; the gripper positions (``tcps()``) are read from the robots in sim only.

Without Isaac (offline checks) the module only needs numpy and yam_arena.task_file.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass, field

import numpy as np

from yam_arena.task_file import Box, ObjectState


# -- small rigid-transform helpers --------------------------------------------------------------------------------

def quat_to_mat(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(m) -> np.ndarray:
    """Rotation matrix -> quaternion (x, y, z, w)."""
    m = np.asarray(m, float)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def axis_angle(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def T(rot=None, pos=(0.0, 0.0, 0.0)) -> np.ndarray:
    out = np.eye(4)
    if rot is not None:
        out[:3, :3] = rot
    out[:3, 3] = pos
    return out


def inv(t) -> np.ndarray:
    r = t[:3, :3]
    return T(r.T, -r.T @ t[:3, 3])


def _frame_with_x(axis) -> np.ndarray:
    """A rotation whose x axis is ``axis`` (USD joints turn about their local X)."""
    x = np.asarray(axis, float) / np.linalg.norm(axis)
    helper = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(helper, x)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


# -- declarations -------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Shape:
    kind: str  # "box" | "cyl"
    size: tuple  # box: (x, y, z); cylinder: (radius, height)
    at: tuple  # centre in the part frame
    color: tuple | None = None
    axis: str = "Z"  # a cylinder's axis in the part frame: "X", "Y" or "Z"
    friction: float | None = None  # None: the arena's grippy material; else this friction, combined by the minimum


def box(size, at=(0.0, 0.0, 0.0), color=None, friction=None) -> Shape:
    return Shape("box", tuple(float(s) for s in size), tuple(float(a) for a in at), color, "Z", friction)


def cyl(radius, height, at=(0.0, 0.0, 0.0), color=None, axis="Z", friction=None) -> Shape:
    return Shape("cyl", (float(radius), float(height)), tuple(float(a) for a in at), color, axis.upper(), friction)


@dataclass(frozen=True)
class KitPart(Box):
    """A kit part as a task-file object: ``pos``/``size`` are its world bounding box at spawn (so tools that treat it
    as a Box see the right footprint); the spawner uses ``shapes`` and ``origin``."""

    shapes: tuple = ()
    origin: tuple = (0.0, 0.0, 0.0)
    origin_quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass
class _Part:
    name: str
    ref: np.ndarray  # world pose of the part frame with every hinge at 0
    shapes: tuple
    mass: float | None
    color: tuple
    lo: np.ndarray = field(default=None)
    hi: np.ndarray = field(default=None)

    @property
    def centre(self) -> np.ndarray:  # the local bounding-box centre: what object_states reports as pos
        return (self.lo + self.hi) / 2


@dataclass
class _Hinge:
    name: str
    child: str
    anchor: np.ndarray  # world, reference pose
    axis: np.ndarray  # world unit vector
    lower: float  # rad
    upper: float
    start: float
    stiffness: float  # N*m/rad
    rest: float  # rad
    damping: float  # N*m*s/rad


class Kit:
    """The parts and hinges of one task file."""

    def __init__(self) -> None:
        self.parts: dict[str, _Part] = {}
        self.hinges: dict[str, _Hinge] = {}
        self.steps: list = []
        self.resets: list = []
        _KITS.append(self)

    def part(self, name: str, origin, shapes, mass: float | None = None, color=(0.6, 0.6, 0.6)) -> str:
        """A rigid body made of ``shapes``; its frame at world ``origin`` (axes aligned with the world) with every
        hinge at 0. ``mass`` None: a static collider."""
        shapes = tuple(shapes)
        corners = []
        for s in shapes:
            c = np.asarray(s.at)
            if s.kind == "box":
                h = np.asarray(s.size) / 2
            else:
                h = np.full(3, s.size[0])
                h["XYZ".index(s.axis)] = s.size[1] / 2
            corners += [c - h, c + h]
        corners = np.array(corners)
        p = _Part(name, T(None, np.asarray(origin, float)), shapes, mass, tuple(color))
        p.lo, p.hi = corners.min(axis=0), corners.max(axis=0)
        self.parts[name] = p
        return name

    def hinge(self, name, child, anchor, axis, lower_deg, upper_deg, start_deg=0.0, stiffness=0.0, rest_deg=0.0,
              damping=0.0) -> str:
        """A revolute joint holding free part ``child`` to the world, about world ``axis`` through ``anchor``. The
        angle is the child's right-hand turn about +axis from its declared pose; a spring of ``stiffness`` N*m/rad
        pulls it toward ``rest_deg``."""
        if self.parts[child].mass is None:
            raise ValueError(f"hinge {name!r}: child {child!r} must be free (give it a mass)")
        a = np.asarray(axis, float)
        self.hinges[name] = _Hinge(name, child, np.asarray(anchor, float), a / np.linalg.norm(a),
                                   math.radians(lower_deg), math.radians(upper_deg), math.radians(start_deg),
                                   float(stiffness), math.radians(rest_deg), float(damping))
        return name

    def on_step(self, fn) -> None:
        """``fn(objects)`` after every control step in sim, before success() and score()."""
        self.steps.append(fn)

    def on_reset(self, fn) -> None:
        """``fn()`` at every episode reset in sim."""
        self.resets.append(fn)

    # -- poses ------------------------------------------------------------------------------------------------

    def _hinge_of(self, part: str):
        return next((h for h in self.hinges.values() if h.child == part), None)

    def pose_at(self, part: str, angles: dict | None = None) -> np.ndarray:
        """The part frame's world pose with the given hinge angles (rad; unlisted hinges at start)."""
        p = self.parts[part]
        h = self._hinge_of(part)
        if h is None:
            return p.ref.copy()
        a = (angles or {}).get(h.name, h.start)
        r = axis_angle(h.axis, a)
        return T(r, h.anchor - r @ h.anchor) @ p.ref

    def objects(self) -> list[KitPart]:
        out = []
        for name, p in self.parts.items():
            t = self.pose_at(name)
            lo, hi = _world_aabb(t, p.lo, p.hi)
            fl = lambda v: tuple(float(c) for c in v)  # noqa: E731  plain floats for Isaac
            out.append(KitPart(name=name, pos=fl((lo + hi) / 2), size=fl(np.maximum(hi - lo, 1e-4)), mass=p.mass,
                               color=fl(p.color), shapes=p.shapes, origin=fl(t[:3, 3]),
                               origin_quat=fl(mat_to_quat(t[:3, :3]))))
        return out

    # -- readouts from object states ---------------------------------------------------------------------------

    def frame(self, objects, part: str) -> np.ndarray:
        """The part frame's world pose, from its ObjectState (pos is the bounding-box centre)."""
        p = self.parts[part]
        st = objects[part]
        r = quat_to_mat(st.quat_xyzw)
        return T(r, np.asarray(st.pos, float) - r @ p.centre)

    def point(self, objects, part: str, local) -> np.ndarray:
        t = self.frame(objects, part)
        return t[:3, :3] @ np.asarray(local, float) + t[:3, 3]

    def angle(self, objects, hinge: str) -> float:
        """The hinge angle (rad) from the child's pose."""
        h = self.hinges[hinge]
        m = self.frame(objects, h.child) @ inv(self.parts[h.child].ref)
        q = mat_to_quat(m[:3, :3])
        return float(2 * math.atan2(float(np.dot(q[:3], h.axis)), float(q[3])))

    def state(self, part: str, angles: dict | None = None, pose=None) -> ObjectState:
        """An ObjectState for offline tests: the part at hinge ``angles`` (rad), or at ``pose`` (4x4 world)."""
        p = self.parts[part]
        t = self.pose_at(part, angles) if pose is None else pose
        lo, hi = _world_aabb(t, p.lo, p.hi)
        z = np.zeros(3)
        return ObjectState(pos=t[:3, :3] @ p.centre + t[:3, 3], quat_xyzw=mat_to_quat(t[:3, :3]), lin_vel=z,
                           ang_vel=z.copy(), low=lo, high=hi)


def _world_aabb(t, lo, hi):
    pts = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = pts @ t[:3, :3].T + t[:3, 3]
    return w.min(axis=0), w.max(axis=0)


_KITS: list = []
_SIM: dict = {"env": None, "dt": 0.05, "step": 0}
TCP_PAST_GRIPPER = 0.134  # the TCP is this far from the gripper link toward the fingertips' midpoint


def sim_time() -> float:
    """Seconds since the episode's reset (control steps x step time); 0 offline."""
    return _SIM["step"] * _SIM["dt"]


def tcps() -> list:
    """World positions of both grippers' TCPs [left, right] in sim; [] offline."""
    env = _SIM["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        g = pos[names.index("gripper")]
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]) / 2
        out.append(g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * TCP_PAST_GRIPPER)
    return out


# -- yam_arena patches (in memory, applied when those modules are imported) ----------------------------------------

def _all_parts() -> dict:
    return {n: (k, p) for k in _KITS for n, p in k.parts.items()}


def _patch_scene(mod) -> None:
    if getattr(mod, "_hingekit", False):
        return
    mod._hingekit = True
    original = mod.task_object

    import isaaclab.sim as sim_utils
    from isaaclab.sim import schemas
    from isaaclab.sim.spawners.shapes import spawn_cuboid, spawn_cylinder
    from isaaclab.sim.utils import clone, create_prim, get_current_stage

    pending: dict = {}

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        spec = pending[prim_path.rsplit("/", 1)[-1]]
        stage = get_current_stage()
        create_prim(prim_path, prim_type="Xform", translation=translation, orientation=orientation, stage=stage)
        free = spec.mass is not None
        coll = mod._contact_props() if free else mod.PhysxCollisionPropertiesCfg()
        def material(s):
            if s.friction is None:
                return mod._grippy()
            return sim_utils.RigidBodyMaterialCfg(static_friction=s.friction, dynamic_friction=s.friction,
                                                  friction_combine_mode="min")

        for i, s in enumerate(spec.shapes):
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=mod.linear_rgb(s.color or spec.color), roughness=0.4)
            if s.kind == "box":
                c = sim_utils.CuboidCfg(size=tuple(s.size), collision_props=coll, physics_material=material(s),
                                        visual_material=look)
                spawn_cuboid.__wrapped__(f"{prim_path}/s{i}", c, translation=tuple(s.at), orientation=(1, 0, 0, 0))
            else:
                c = sim_utils.CylinderCfg(radius=s.size[0], height=s.size[1], axis=s.axis, collision_props=coll,
                                          physics_material=material(s), visual_material=look)
                spawn_cylinder.__wrapped__(f"{prim_path}/s{i}", c, translation=tuple(s.at), orientation=(1, 0, 0, 0))
        if free:
            schemas.define_rigid_body_properties(
                prim_path, mod.PhysxRigidBodyPropertiesCfg(solver_position_iteration_count=16,
                                                           solver_velocity_iteration_count=2), stage=stage)
            schemas.define_mass_properties(prim_path, sim_utils.MassPropertiesCfg(mass=spec.mass), stage=stage)
        return stage.GetPrimAtPath(prim_path)

    def task_object(spec):
        if not isinstance(spec, KitPart):
            return original(spec)
        pending[spec.name] = spec
        cfg = sim_utils.CuboidCfg(func=spawn, size=tuple(spec.size))
        return mod.Object(
            name=spec.name,
            object_type=mod.ObjectType.RIGID if spec.mass is not None else mod.ObjectType.BASE,
            spawner_cfg=cfg,
            initial_pose=mod.Pose(position_xyz=tuple(spec.origin), rotation_xyzw=tuple(spec.origin_quat)),
        )

    mod.task_object = task_object

    from isaaclab.scene import interactive_scene as isc

    cls = isc.InteractiveScene
    if not getattr(cls, "_hingekit", False):
        init0 = cls.__init__

        def __init__(self, *a, **k):
            init0(self, *a, **k)
            _author_hinges(self.stage, self.env_prim_paths[0])

        cls.__init__ = __init__
        cls._hingekit = True


def _author_hinges(stage, env_path: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    m = UsdGeom.Xformable(stage.GetPrimAtPath(env_path)).ComputeLocalToWorldTransform(0)
    env_t = T(None, (float(m[3][0]), float(m[3][1]), float(m[3][2])))
    made = []
    for kit in _KITS:
        for h in kit.hinges.values():
            child_path = f"{env_path}/{h.child}"
            if not stage.GetPrimAtPath(child_path).IsValid():
                continue
            frame = env_t @ T(_frame_with_x(h.axis), h.anchor)  # the joint frame in the sim world
            f1 = inv(env_t @ kit.parts[h.child].ref) @ frame  # ... in the child body at angle 0
            joint = UsdPhysics.RevoluteJoint.Define(stage, f"{env_path}/hingekit_joints/{h.name}")
            joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])  # body0 empty: the world

            def gq(r):
                x, y, z, w = mat_to_quat(r)
                return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in frame[:3, 3]]))
            joint.CreateLocalRot0Attr().Set(gq(frame[:3, :3]))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in f1[:3, 3]]))
            joint.CreateLocalRot1Attr().Set(gq(f1[:3, :3]))
            joint.CreateAxisAttr().Set("X")
            joint.CreateLowerLimitAttr().Set(math.degrees(h.lower))  # USD angles are degrees
            joint.CreateUpperLimitAttr().Set(math.degrees(h.upper))
            joint.CreateCollisionEnabledAttr().Set(False)
            joint.CreateExcludeFromArticulationAttr().Set(True)
            if h.stiffness > 0 or h.damping > 0:
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
                drive.CreateTypeAttr().Set("force")
                drive.CreateStiffnessAttr().Set(float(h.stiffness * math.pi / 180.0))  # per degree
                drive.CreateDampingAttr().Set(float(h.damping * math.pi / 180.0))
                drive.CreateTargetPositionAttr().Set(float(math.degrees(h.rest)))
                drive.CreateTargetVelocityAttr().Set(0.0)
            # The child never collides with the static parts it is mounted on.
            fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(child_path))
            for name, p in kit.parts.items():
                if p.mass is None and stage.GetPrimAtPath(f"{env_path}/{name}").IsValid():
                    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{env_path}/{name}"))
            made.append(h.name)
    print(f"[hingekit] hinges {made}", flush=True)


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_hingekit", False):
        return
    cls._hingekit = True
    ensure0, reset0, step0 = cls._ensure_sim, cls.reset, cls._task_step

    def _ensure_sim(self, *a, **k):
        out = ensure0(self, *a, **k)
        hinged = {h.child for kit in _KITS for h in kit.hinges.values()}
        self._free_objects = tuple(n for n in self._free_objects if n not in hinged)  # no jitter on mechanisms
        return out

    def reset(self, *a, **k):
        obs = reset0(self, *a, **k)
        _SIM.update(env=self._env, dt=float(self._base.step_dt), step=0)
        for kit in _KITS:
            for fn in kit.resets:
                fn()
        return obs

    def _task_step(self, *a, **k):
        _SIM.update(env=self._env)
        _SIM["step"] += 1
        if any(kit.steps for kit in _KITS):
            from yam_arena.scene import object_states

            states = object_states(self._env, list(self.task.names))
            for kit in _KITS:
                for fn in kit.steps:
                    fn(states)
        return step0(self, *a, **k)

    cls._ensure_sim, cls.reset, cls._task_step = _ensure_sim, reset, _task_step


class _AfterImport(importlib.abc.MetaPathFinder):
    """Runs ``patch(module)`` right after a module is first imported."""

    def __init__(self, name: str, patch) -> None:
        self.name, self.patch, self._busy = name, patch, False

    def find_spec(self, fullname, path=None, target=None):
        if fullname != self.name or self._busy:
            return None
        self._busy = True
        try:
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        finally:
            self._busy = False
        if spec is None or spec.loader is None:
            return None
        inner, patch = spec.loader, self.patch

        class _Loader(importlib.abc.Loader):
            def create_module(self, s):
                return inner.create_module(s)

            def exec_module(self, module):
                inner.exec_module(module)
                patch(module)

        spec.loader = _Loader()
        return spec


def _when_imported(name: str, patch) -> None:
    if name in sys.modules:
        patch(sys.modules[name])
    elif not any(isinstance(f, _AfterImport) and f.name == name for f in sys.meta_path):
        sys.meta_path.insert(0, _AfterImport(name, patch))


_when_imported("yam_arena.scene", _patch_scene)
_when_imported("yam_arena.inspect_embodiment", _patch_embodiment)
