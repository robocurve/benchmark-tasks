"""Articulated, multi-colour parts for yam_arena task files: rigid parts built from boxes and cylinders, with hinges
(revolute joints) and spring sliders (prismatic joints). Copy this file next to your task file and import it there:

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import parts  # noqa: E402

A :class:`Part` goes in ``OBJECTS`` like a Box. It is a rigid body made of :func:`box` and :func:`cylinder` shapes,
each with its own colour, given in the part's frame (origin at ``pos``, turned by ``yaw``). ``mass=None`` fixes the
part in place; a mass makes it loose. Each :class:`Hinge` or :class:`Slider` in ``joints`` adds one moving link,
with its own shapes and mass, jointed to the part's body. Shapes are given where they are at joint position 0, so
the pose you write down is the starting pose. A joint with ``spring > 0`` is pulled back to 0.

success()/score() receive a :class:`PartState` for each part: ``pos``/``quat_xyzw`` are the part's body origin and
orientation (not a bounding-box centre), ``low``/``high`` the world box around all its shapes, ``joints`` each
joint's position (rad or m) and ``links`` each moving link's pose. ``on_step``/``on_reset`` register callbacks that
run in the simulator after every control step / reset with a :class:`Sim` handle (states, joint targets).

This copy adds, for a jointed hand tool and a scored food surface: a joint may hang from another joint's link
(``parent=``); a :class:`Fixed` link welded to its parent (a separate rigid body, so the simulator can report its
own contacts); shapes without a collider (``collide=False``); visual-only meshes (:class:`Visual`) that a task can hide
at run time; and on a Part its own contact offset and the links that report contacts (``report=``).

It also has :func:`sphere` shapes, turned boxes (``box(..., quat=...)``), a part's full start orientation
(``Part(..., quat=...)``), and on a :class:`Slider` an extension below 0, a spring rest point (``target``) and static and
dynamic joint friction (``friction``, ``dynamic_friction``, in N, passed to Isaac Lab's implicit actuator).

Importing this file is pure Python (NumPy and ``yam_arena.task_file``), so the task still loads offline. Isaac is
imported only once the embodiment builds its scene: an import hook then patches ``yam_arena.scene`` (spawning and
state reading) and the ``yam_arena`` embodiment's step and reset. Nothing in yam_arena is modified on disk.
"""

from __future__ import annotations

import importlib.abc
import math
import sys
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import GREY, ObjectState

FRICTION = (1.2, 1.0)  # static, dynamic: yam_arena's grippy material
CONTACT_OFFSET, REST_OFFSET = 0.005, 0.0  # as yam_arena's free objects
SOLVER_ITERATIONS = (32, 4)  # articulation position, velocity iterations
MAX_DEPENETRATION = 1.0  # m/s: caps how hard PhysX pushes overlapping bodies apart


# -- geometry (NumPy) -------------------------------------------------------------------------------------------


def _rot(axis, angle: float) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(m) -> np.ndarray:
    """xyzw, w >= 0."""
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def _x_to(axis) -> np.ndarray:
    """The rotation that turns +x onto ``axis`` (a USD joint's axis is its frame's x)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    c = float(np.clip(a[0], -1.0, 1.0))
    v = np.cross([1.0, 0.0, 0.0], a)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else _rot((0, 0, 1), math.pi)
    return _rot(v, math.acos(c))


# -- shapes, joints, parts (pure Python: what a task file declares) ----------------------------------------------


@dataclass(frozen=True)
class Shape:
    kind: str  # "box", "cylinder" or "sphere"
    size: tuple[float, float, float]  # box: edge lengths; cylinder: (radius, height, 0); sphere: (radius, 0, 0)
    at: tuple[float, float, float]  # centre in the part frame, at joint position 0 (m)
    color: tuple[float, float, float] = GREY  # sRGB 0..1
    axis: str = "z"  # cylinder axis in the part frame: "x", "y" or "z"
    quat: tuple[float, float, float, float] | None = None  # the shape turned in the part frame (xyzw), if given
    collide: bool = True

    @property
    def R(self) -> np.ndarray:
        return np.eye(3) if self.quat is None else _quat_to_matrix(self.quat)

    @property
    def half(self) -> np.ndarray:
        """Half edge lengths of the shape's own box (a cylinder's bounding box) in the part frame."""
        if self.kind == "box":
            return np.asarray(self.size, float) / 2
        if self.kind == "sphere":
            return np.full(3, self.size[0])
        r, h = self.size[0], self.size[1]
        return np.array([h / 2 if self.axis == a else r for a in "xyz"])


def box(size, at=(0.0, 0.0, 0.0), color=GREY, quat=None, collide=True) -> Shape:
    q = None if quat is None else tuple(float(v) for v in quat)
    return Shape("box", tuple(float(s) for s in size), tuple(float(a) for a in at), tuple(color), "z", q, bool(collide))


def cylinder(radius, height, at=(0.0, 0.0, 0.0), color=GREY, axis="z", collide=True) -> Shape:
    if axis not in ("x", "y", "z"):
        raise ValueError(f"cylinder axis must be 'x', 'y' or 'z', got {axis!r}")
    return Shape("cylinder", (float(radius), float(height), 0.0), tuple(float(a) for a in at), tuple(color), axis,
                 None, bool(collide))


def prism(radius, length, sides, at=(0.0, 0.0, 0.0), color=GREY, collide=True) -> Shape:
    """A regular prism along the part's x axis (``radius`` to its corners), collided as its own convex hull. A
    many-sided prism stands in for a round body that should lie still where it is put (PhysX cylinders creep)."""
    return Shape("prism", (float(radius), float(length), float(sides)), tuple(float(a) for a in at), tuple(color),
                 "x", None, bool(collide))


def sphere(radius, at=(0.0, 0.0, 0.0), color=GREY, collide=True) -> Shape:
    return Shape("sphere", (float(radius), 0.0, 0.0), tuple(float(a) for a in at), tuple(color), "z", None,
                 bool(collide))


@dataclass(frozen=True)
class Visual:
    """A visual-only triangle mesh on one link (no collider), in the part frame at joint position 0. A task can
    hide it at run time with :meth:`Sim.set_visible`."""

    name: str
    points: tuple[tuple[float, float, float], ...]
    triangles: tuple[tuple[int, int, int], ...]
    color: tuple[float, float, float] = GREY
    link: str = "base"


@dataclass(frozen=True)
class Hinge:
    """A revolute joint: the link (``shapes``, ``mass``) turns about ``axis`` (part frame) through ``pivot``.
    Position 0 is the pose the shapes are given in; ``lower``/``upper`` (rad) bound it. ``spring`` (N m/rad) pulls it
    back to 0 (or to a target set with :meth:`Sim.set_target`); ``damping`` (N m s/rad) resists turning."""

    name: str
    shapes: tuple[Shape, ...]
    pivot: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float
    upper: float
    mass: float = 0.1
    spring: float = 0.0
    damping: float = 0.01
    max_effort: float = 50.0  # N m the spring/drive may exert
    target: float = 0.0
    friction: float = 0.0
    dynamic_friction: float = 0.0
    parent: str = "base"  # the link this joint hangs from: "base" or an earlier joint's name

    @property
    def origin(self) -> np.ndarray:
        return np.asarray(self.pivot, float)

    @property
    def lower_upper(self):
        return self.lower, self.upper

    def local(self, q: float) -> tuple[np.ndarray, np.ndarray]:
        """(rotation, translation) the joint adds at position q (part-frame axes)."""
        return _rot(self.axis, q), np.zeros(3)


@dataclass(frozen=True)
class Slider:
    """A prismatic joint: the link slides along ``axis`` (part frame) from ``-extend`` to ``travel`` (m). ``spring``
    (N/m) pulls it toward ``target`` (default 0, or a target set later); ``damping`` (N s/m) resists motion.
    ``friction`` is the joint's static (Coulomb) friction force in N: the joint does not move while the force along it
    stays below this; ``dynamic_friction`` (N) resists it while it slides."""

    name: str
    shapes: tuple[Shape, ...]
    axis: tuple[float, float, float]
    travel: float
    mass: float = 0.05
    spring: float = 0.0
    damping: float = 1.0
    max_effort: float = 200.0  # N
    extend: float = 0.0  # m the link may move the other way, past 0
    target: float = 0.0  # m: the spring's rest point
    friction: float = 0.0  # N, static
    dynamic_friction: float = 0.0  # N
    parent: str = "base"

    @property
    def lower(self) -> float:
        return -float(self.extend)

    @property
    def upper(self) -> float:
        return float(self.travel)

    @property
    def origin(self) -> np.ndarray:
        return np.mean([s.at for s in self.shapes], axis=0)

    def local(self, q: float) -> tuple[np.ndarray, np.ndarray]:
        a = np.asarray(self.axis, float) / np.linalg.norm(self.axis)
        return np.eye(3), q * a


@dataclass(frozen=True)
class Fixed:
    """A link welded to its ``parent`` link: one rigid piece with it, but its own rigid body, so the simulator
    reports its contacts separately. Its frame origin is ``at`` (part frame)."""

    name: str
    shapes: tuple[Shape, ...]
    at: tuple[float, float, float]
    mass: float = 0.01
    parent: str = "base"

    @property
    def origin(self) -> np.ndarray:
        return np.asarray(self.at, float)

    def local(self, q: float) -> tuple[np.ndarray, np.ndarray]:
        return np.eye(3), np.zeros(3)


Joint = Hinge | Slider | Fixed


def actuated(part) -> tuple:
    return tuple(j for j in part.joints if not isinstance(j, Fixed))


@dataclass(frozen=True)
class PartState(ObjectState):
    """A part after a control step (world frame). ``pos`` is the body origin, not the bounding-box centre."""

    joints: dict[str, float] = field(default_factory=dict)  # rad (hinge) or m (slider)
    links: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)  # joint name -> (pos, quat_xyzw)


@dataclass(frozen=True)
class Part(task_file.Box):
    """A rigid part made of ``shapes`` (part frame; origin at ``pos``, turned by ``yaw``), with optional ``joints``.
    ``mass=None``: fixed in place. ``size`` and ``color`` are derived (the starting box around every shape, the first
    shape's colour) so offline tools that read a Box still see something sensible."""

    shapes: tuple[Shape, ...] = ()
    joints: tuple[Joint, ...] = ()
    friction: tuple[float, float] = FRICTION
    quat: tuple[float, float, float, float] | None = None  # start orientation (xyzw); overrides yaw when given
    contact_offset: float = CONTACT_OFFSET
    report: tuple[str, ...] = ()  # links whose contacts the simulator reports (PhysxContactReportAPI)
    visuals: tuple[Visual, ...] = ()
    link_friction: tuple[tuple[str, tuple[float, float]], ...] = ()  # (link, (static, dynamic)) overrides
    angular_damping: float | None = None  # 1/s, PhysX rigid-body angular damping (None: PhysX default)

    def __post_init__(self):
        object.__setattr__(self, "shapes", tuple(self.shapes))
        object.__setattr__(self, "joints", tuple(self.joints))
        object.__setattr__(self, "report", tuple(self.report))
        object.__setattr__(self, "visuals", tuple(self.visuals))
        if self.shapes:
            object.__setattr__(self, "color", self.shapes[0].color)
        lo, hi = self._local_box()
        object.__setattr__(self, "size", tuple(float(v) for v in hi - lo))

    def _check(self) -> None:
        if not self.shapes:
            raise ValueError(f"{self.name}: a Part needs at least one shape")
        super()._check()
        names = [j.name for j in self.joints]
        if len(set(names)) != len(names) or "base" in names or not all(n.isidentifier() for n in names):
            raise ValueError(f"{self.name}: joint names must be unique identifiers other than 'base', got {names}")
        for k, j in enumerate(self.joints):
            if j.parent != "base" and j.parent not in names[:k]:
                raise ValueError(f"{self.name}.{j.name}: parent {j.parent!r} must be 'base' or an earlier joint")
            if isinstance(j, Fixed):
                if not j.shapes or not j.mass > 0:
                    raise ValueError(f"{self.name}.{j.name}: a fixed link needs shapes and a positive mass")
                continue
            if not j.shapes or not j.mass > 0 or not j.lower <= 0.0 <= j.upper or j.lower == j.upper:
                raise ValueError(f"{self.name}.{j.name}: needs shapes, a positive mass and limits around 0")
            if j.spring < 0 or j.damping < 0 or not np.linalg.norm(j.axis) > 0:
                raise ValueError(f"{self.name}.{j.name}: spring and damping must be >= 0 and the axis non-zero")
        for s in self.shapes + tuple(s for j in self.joints for s in j.shapes):
            if any(not 0.0 <= c <= 1.0 for c in s.color) or any(v <= 0 for v in s.half):
                raise ValueError(f"{self.name}: shape {s} needs positive sizes and an sRGB colour in 0..1")

    @property
    def rotation(self) -> np.ndarray:
        return _quat_to_matrix(self.quat) if self.quat is not None else _rot((0, 0, 1), self.yaw)

    @property
    def links(self) -> tuple[tuple[str, np.ndarray, tuple[Shape, ...]], ...]:
        """(name, origin in the part frame, shapes) for the body ("base") and each joint's link."""
        return (("base", np.zeros(3), self.shapes), *((j.name, j.origin, j.shapes) for j in self.joints))

    def _local_box(self):
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        pts = [np.asarray(s.at) + (corners * s.half) @ s.R.T for _, _, shapes in self.links for s in shapes]
        pts = np.concatenate(pts) if pts else np.zeros((1, 3))
        return pts.min(axis=0), pts.max(axis=0)

    def world_boxes(self, link_poses: dict[str, tuple[np.ndarray, np.ndarray]]):
        """(link, centre, rotation, size) of every shape of the links in ``link_poses`` (world pos, rotation)."""
        out = []
        for name, origin, shapes in self.links:
            if name not in link_poses:
                continue
            p, r = link_poses[name]
            for s in shapes:
                out.append((name, p + r @ (np.asarray(s.at) - origin), r @ s.R, 2 * s.half))
        return out

    def make_state(self, link_poses, joints, lin_vel=None, ang_vel=None) -> PartState:
        """A PartState from each link's world (pos, rotation matrix) and the joint positions."""
        boxes = self.world_boxes(link_poses)
        corners = np.concatenate([c + (np.abs(r) @ (s / 2)) * [[-1], [1]] for _, c, r, s in boxes])
        p, r = link_poses["base"]
        return PartState(
            pos=np.asarray(p, float),
            quat_xyzw=_matrix_to_quat(r),
            lin_vel=np.zeros(3) if lin_vel is None else np.asarray(lin_vel, float),
            ang_vel=np.zeros(3) if ang_vel is None else np.asarray(ang_vel, float),
            low=corners.min(axis=0),
            high=corners.max(axis=0),
            joints={k: float(v) for k, v in joints.items()},
            links={k: (np.asarray(v[0], float), _matrix_to_quat(v[1])) for k, v in link_poses.items() if k != "base"},
        )

    def state(self, joints=None, pos=None, quat_xyzw=None, lin_vel=None) -> PartState:
        """The state the simulator would report with the body at ``pos``/``quat_xyzw`` (default: where it starts) and
        the given joint positions (default 0), from the part's own kinematics. For offline checks and near misses."""
        q = {j.name: 0.0 for j in actuated(self)} | dict(joints or {})
        unknown = sorted(set(q) - {j.name for j in actuated(self)})
        if unknown:
            raise KeyError(f"{self.name} has no joints {unknown}")
        p = np.asarray(self.pos if pos is None else pos, float)
        r = self.rotation if quat_xyzw is None else _quat_to_matrix(quat_xyzw)
        return self.make_state(self.link_poses(p, r, q), q, lin_vel)

    def joint_origin(self, name: str) -> np.ndarray:
        return np.zeros(3) if name == "base" else next(j.origin for j in self.joints if j.name == name)

    def link_poses(self, p, r, q) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """World (pos, rotation) of every link with the body at p, r and joint positions q (chains followed).
        Joint axes are given in the part frame at position 0, so a chained joint's axis turns with its parent."""
        poses = {"base": (np.asarray(p, float), np.asarray(r, float))}
        for j in self.joints:  # parents come first (checked)
            pp, pr = poses[j.parent]
            jr, jt = j.local(q.get(j.name, 0.0))
            o = pp + pr @ (j.origin - self.joint_origin(j.parent))
            poses[j.name] = (o + pr @ jt, pr @ jr)
        return poses


# -- callbacks ---------------------------------------------------------------------------------------------------

_ON_STEP: list[Callable] = []
_ON_RESET: list[Callable] = []


def on_step(fn: Callable) -> Callable:
    """Decorator: ``fn(sim)`` runs after every control step, before success()/score() read the states."""
    _ON_STEP.append(fn)
    return fn


def on_reset(fn: Callable) -> Callable:
    """Decorator: ``fn(sim)`` runs after every reset (after the embodiment's settle steps)."""
    _ON_RESET.append(fn)
    return fn


class Sim:
    """What a callback gets: the running embodiment, its states and the joints' drive targets."""

    def __init__(self, embodiment):
        self.embodiment = embodiment
        self.env = embodiment._env
        self.t = embodiment._t  # seconds since the episode began

    def states(self) -> dict[str, ObjectState]:
        from yam_arena.scene import object_states

        return object_states(self.env, self.embodiment.task.names)

    def set_visible(self, part: str, visual: str, visible: bool) -> None:
        """Show or hide one of ``part``'s :class:`Visual` meshes."""
        from isaaclab.sim.utils import get_current_stage
        from pxr import UsdGeom

        spec = _SPECS[part]
        v = next(v for v in spec.visuals if v.name == visual)
        root = _PATHS[part]
        path = f"{root}/vis_{v.name}" if not spec.joints else f"{root}/{v.link}/vis_{v.name}"
        img = UsdGeom.Imageable(get_current_stage().GetPrimAtPath(path))
        img.MakeVisible() if visible else img.MakeInvisible()

    def set_target(self, part: str, joint: str, value: float) -> None:
        """Move the spring's rest point of ``part``'s ``joint`` (rad or m); it needs ``spring > 0`` to act."""
        _set_target(self.env.unwrapped, part, joint, value)


# -- runtime: Isaac Lab / Arena (imported only inside the simulator) ---------------------------------------------

_SPECS: dict[str, Part] = {}  # parts in the env being built or run, by name
_PATHS: dict[str, str] = {}  # part name -> its root prim path in env 0 (set while authoring)


def _usd_quat(m):
    from pxr import Gf

    x, y, z, w = _matrix_to_quat(m)
    return Gf.Quatf(float(w), float(x), float(y), float(z))


def _author(part: Part, root_path: str, translation, orientation):
    """Write the part to the stage: rigid bodies with their colliders and looks, and its joints."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.materials.physics_materials import spawn_physics_material
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material, create_prim, get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = create_prim(root_path, "Xform", translation=translation, orientation=orientation, stage=stage)
    friction = f"{root_path}/friction"
    spawn_physics_material(
        friction, sim_utils.RigidBodyMaterialCfg(static_friction=part.friction[0], dynamic_friction=part.friction[1])
    )
    mats = {}
    for i, (lk, fr) in enumerate(part.link_friction):
        mats[lk] = f"{root_path}/friction_{i}"
        spawn_physics_material(mats[lk], sim_utils.RigidBodyMaterialCfg(static_friction=fr[0], dynamic_friction=fr[1]))
    looks: dict[tuple, str] = {}

    def look(color) -> str:
        if color not in looks:
            looks[color] = f"{root_path}/looks/c{len(looks)}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4)
            cfg.func(looks[color], cfg)
        return looks[color]

    def body(path: str, mass: float | None):
        prim = stage.GetPrimAtPath(path)
        if mass is None:
            return
        UsdPhysics.RigidBodyAPI.Apply(prim)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(MAX_DEPENETRATION)
        if part.angular_damping is not None:
            rb.CreateAngularDampingAttr(float(part.angular_damping))
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))

    def colliders(path: str, origin, shapes, link="base"):
        for v in part.visuals:
            if v.link != link:
                continue
            mesh = UsdGeom.Mesh.Define(stage, f"{path}/vis_{v.name}")
            pts = np.asarray(v.points, float) - origin
            mesh.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in pts])
            mesh.CreateFaceVertexCountsAttr([3] * len(v.triangles))
            mesh.CreateFaceVertexIndicesAttr([int(i) for t in v.triangles for i in t])
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            bind_visual_material(f"{path}/vis_{v.name}", look(v.color), stage=stage)
        for i, s in enumerate(shapes):
            sp = f"{path}/s{i}"
            if s.kind == "box":
                geom = UsdGeom.Cube.Define(stage, sp)
                geom.CreateSizeAttr(1.0)
                geom.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(s.at) - origin)))
                if s.quat is not None:
                    x, y, z, w = (float(v) for v in s.quat)
                    geom.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
                geom.AddScaleOp().Set(Gf.Vec3f(*s.size))
            elif s.kind == "prism":
                geom = UsdGeom.Mesh.Define(stage, sp)
                r, length, n = s.size[0], s.size[1], int(s.size[2])
                c = np.asarray(s.at) - origin
                pts = [Gf.Vec3f(float(c[0] + e * length / 2), float(c[1] + r * math.cos(2 * math.pi * k / n)),
                                float(c[2] + r * math.sin(2 * math.pi * k / n))) for e in (-1, 1) for k in range(n)]
                faces, counts = [], []
                for k in range(n):
                    k2 = (k + 1) % n
                    faces += [k, k2, n + k2, n + k]
                    counts.append(4)
                faces += list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
                counts += [n, n]
                geom.CreatePointsAttr(pts)
                geom.CreateFaceVertexCountsAttr(counts)
                geom.CreateFaceVertexIndicesAttr(faces)
                geom.CreateSubdivisionSchemeAttr("none")
                UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
                UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")
            elif s.kind == "sphere":
                geom = UsdGeom.Sphere.Define(stage, sp)
                geom.CreateRadiusAttr(s.size[0])
                geom.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(s.at) - origin)))
            else:
                geom = UsdGeom.Cylinder.Define(stage, sp)
                geom.CreateRadiusAttr(s.size[0])
                geom.CreateHeightAttr(s.size[1])
                geom.CreateAxisAttr(s.axis.upper())
                geom.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(s.at) - origin)))
            bind_visual_material(sp, look(s.color), stage=stage)
            if not s.collide:
                continue
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            col = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
            col.CreateContactOffsetAttr(float(part.contact_offset))
            col.CreateRestOffsetAttr(REST_OFFSET)
            bind_physics_material(sp, mats.get(link, friction), stage=stage)

    def report(path: str, link: str):
        if link in part.report:
            api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
            api.CreateThresholdAttr(0.0)

    _PATHS[part.name] = root_path
    if not part.joints:  # one rigid body (loose) or a static collider (fixed) at the root
        body(root_path, part.mass)
        colliders(root_path, np.zeros(3), part.shapes)
        report(root_path, "base")
        return root

    UsdPhysics.ArticulationRootAPI.Apply(root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateEnabledSelfCollisionsAttr(True)  # links collide with each other, except jointed pairs
    art.CreateSolverPositionIterationCountAttr(SOLVER_ITERATIONS[0])
    art.CreateSolverVelocityIterationCountAttr(SOLVER_ITERATIONS[1])
    for name, origin, shapes in part.links:
        create_prim(f"{root_path}/{name}", "Xform", translation=tuple(float(v) for v in origin), stage=stage)
        j = next((j for j in part.joints if j.name == name), None)
        body(f"{root_path}/{name}", j.mass if j else (part.mass or 1.0))
        colliders(f"{root_path}/{name}", origin, shapes, name)
        report(f"{root_path}/{name}", name)
    UsdGeom.Scope.Define(stage, f"{root_path}/joints")
    if part.mass is None:  # fixed: the body is welded to the world where it was spawned
        weld = UsdPhysics.FixedJoint.Define(stage, f"{root_path}/joints/world")
        pose = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(f"{root_path}/base")).RemoveScaleShear()
        weld.CreateBody1Rel().SetTargets([Sdf.Path(f"{root_path}/base")])
        weld.CreateLocalPos0Attr().Set(Gf.Vec3f(pose.ExtractTranslation()))
        weld.CreateLocalRot0Attr().Set(Gf.Quatf(pose.ExtractRotationQuat()))
        weld.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        weld.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    for j in part.joints:
        if isinstance(j, Fixed):
            weld = UsdPhysics.FixedJoint.Define(stage, f"{root_path}/joints/{j.name}")
            weld.CreateBody0Rel().SetTargets([Sdf.Path(f"{root_path}/{j.parent}")])
            weld.CreateBody1Rel().SetTargets([Sdf.Path(f"{root_path}/{j.name}")])
            weld.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, j.origin - part.joint_origin(j.parent))))
            weld.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            weld.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
            weld.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            continue
        hinge = isinstance(j, Hinge)
        kind = UsdPhysics.RevoluteJoint if hinge else UsdPhysics.PrismaticJoint
        joint = kind.Define(stage, f"{root_path}/joints/{j.name}")
        frame = _usd_quat(_x_to(j.axis))
        joint.CreateAxisAttr("X")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(f"{root_path}/{j.parent}")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(f"{root_path}/{j.name}")])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, j.origin - part.joint_origin(j.parent))))
        joint.CreateLocalRot0Attr().Set(frame)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        joint.CreateLocalRot1Attr().Set(frame)
        unit = math.degrees(1.0) if hinge else 1.0  # USD revolute limits and drive gains are per degree
        joint.CreateLowerLimitAttr(j.lower * unit)
        joint.CreateUpperLimitAttr(j.upper * unit)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if hinge else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(j.spring / unit)
        drive.CreateDampingAttr(j.damping / unit)
        drive.CreateTargetPositionAttr(float(j.target) * unit)
        drive.CreateMaxForceAttr(j.max_effort)
    return root


def _arena_object(part: Part):
    """The part as an Arena object: a static collider, a rigid object, or an articulation."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
    from isaaclab_arena.utils.pose import Pose

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        return _author(part, prim_path, translation, orientation)

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {
                j.name: ImplicitActuatorCfg(
                    joint_names_expr=[j.name], stiffness=j.spring, damping=j.damping, effort_limit_sim=j.max_effort,
                    **({"friction": float(j.friction), "dynamic_friction": float(j.dynamic_friction)}
                       if j.friction or j.dynamic_friction else {}),
                )
                for j in actuated(part)
            }
            return self._add_initial_pose_to_cfg(
                ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            )

        def _build_reset_event(self):
            event = super()._build_reset_event()
            # Arena puts an articulation's root back on reset; the joints go back to 0 as well.
            if event is not None and self.object_type == ObjectType.ARTICULATION:
                event.func = reset_articulation_pose_and_joints
            return event

    if part.joints:
        kind = ObjectType.ARTICULATION
    else:
        kind = ObjectType.BASE if part.mass is None else ObjectType.RIGID
    yaw = part.yaw
    rot = tuple(float(v) for v in part.quat) if part.quat is not None else (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return PartObject(
        name=part.name,
        object_type=kind,
        spawner_cfg=SpawnerCfg(func=spawn),
        initial_pose=Pose(position_xyz=tuple(part.pos), rotation_xyzw=rot),
    )


def _np(x) -> np.ndarray:
    x = x.torch if hasattr(x, "torch") else x
    return x.detach().cpu().numpy().astype(np.float64)


def _read(base, part: Part) -> PartState:
    """A part's state in env 0 (world frame relative to the env origin)."""
    if part.joints == () and part.mass is None:
        return part.state()
    asset = base.scene[part.name]
    origin = _np(base.scene.env_origins[0])
    lin, ang = _np(asset.data.root_link_lin_vel_w)[0], _np(asset.data.root_link_ang_vel_w)[0]
    if not part.joints:
        pose = _np(asset.data.root_link_pose_w)[0]
        return part.make_state({"base": (pose[:3] - origin, _quat_to_matrix(pose[3:]))}, {}, lin, ang)
    bodies = _np(asset.data.body_link_pose_w)[0]
    poses = {n: (bodies[i, :3] - origin, _quat_to_matrix(bodies[i, 3:])) for i, n in enumerate(asset.body_names)}
    q = _np(asset.data.joint_pos)[0]
    joints = {n: q[i] for i, n in enumerate(asset.joint_names)}
    return part.make_state({name: poses[name] for name, _, _ in part.links}, joints, lin, ang)


def _set_target(base, part: str, joint: str, value: float) -> None:
    import torch

    asset = base.scene[part]
    ids = asset.find_joints([joint], preserve_order=True)[0]
    target = torch.tensor([[float(value)]], dtype=torch.float32, device=base.device)
    asset.set_joint_position_target_index(target=target, joint_ids=ids)


def _reset_targets(base) -> None:
    for part in _SPECS.values():
        for j in actuated(part):
            _set_target(base, part.name, j.name, float(j.target))


# -- hooks: patch yam_arena when its modules load ---------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_parts_patched", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def patched_task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def patched_object_states(env, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(env, plain) if plain else {}
        out.update({n: _read(env.unwrapped, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = patched_task_object, patched_object_states
    scene._parts_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_parts_patched", False):
        return
    task_step, reset = cls._task_step, cls.reset

    def patched_task_step(self):
        for fn in _ON_STEP:
            fn(Sim(self))
        return task_step(self)

    def patched_reset(self, scene, *, seed=None):
        if self._env is not None:
            _reset_targets(self._base)
        obs = reset(self, scene, seed=seed)
        for fn in _ON_RESET:
            fn(Sim(self))
        return obs

    cls._task_step, cls.reset = patched_task_step, patched_reset
    cls._parts_patched = True


_PATCHES = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _Loader(importlib.abc.Loader):
    def __init__(self, loader, patch):
        self.loader, self.patch = loader, patch

    def create_module(self, spec):
        return self.loader.create_module(spec)

    def exec_module(self, module):
        self.loader.exec_module(module)
        self.patch(module)


class _Finder(importlib.abc.MetaPathFinder):
    """Finds the yam_arena modules we patch with the other finders, then patches them once they have run."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _PATCHES:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None and spec.loader is not None:
                spec.loader = _Loader(spec.loader, _PATCHES[fullname])
                return spec
        return None


def _install() -> None:
    for name, patch in _PATCHES.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(type(f).__name__ == "_Finder" and getattr(f, "parts_kit", False) for f in sys.meta_path):
        finder = _Finder()
        finder.parts_kit = True
        sys.meta_path.insert(0, finder)


_install()
