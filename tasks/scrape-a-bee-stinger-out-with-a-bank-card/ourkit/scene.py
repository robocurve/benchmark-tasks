"""ourkit.scene: mechanisms for yam_arena task files, authored the same way for Isaac Lab and for physcheck.

A :class:`Mech` is one rigid body made of boxes (a wall section, a housing, a tool) carrying :class:`Joint` parts:
sliders and hinges with a spring (``rate`` towards ``target``), viscous ``damping``, limits, and dry ``friction``
whose static and sliding values are equal (Isaac Lab 3.0's ``dynamic_friction`` is set to it, as physcheck's
frictionloss has it). A joint may hang from another joint (``parent``). Everything is given in the mech's own frame
with every joint at 0; a hinge turns about its ``axis`` through ``at``, a slider slides along it.

Importing the kit hooks ``yam_arena.scene`` and the embodiment (real or physcheck's fake) without changing yam_arena:

- ``success()`` / ``score()`` receive a :class:`MechState` for each Mech: body pose, bounds, joint positions ``q``
  and speeds ``qd`` (m or rad) and named points in the world (points on a moving part follow it).
- :func:`on_step`, :func:`on_reset` and :func:`pre_reset` register callbacks. :data:`LATEST` always holds the last
  states and the number of control steps since the reset (the demo helpers read it).
- Runtime writes, the same calls in Isaac and in physcheck: :func:`set_targets` (spring targets: GPU-proven in the
  shade and pail tasks), :func:`set_gains` (stiffness, damping: Isaac Lab ``write_joint_stiffness_to_sim``, as
  accepted K-XZY tasks use; physcheck edits its MuJoCo model), :func:`set_limits` (joint limits: Isaac Lab
  ``write_joint_position_limit_to_sim``; physcheck edits ``jnt_range``). Anything a trial changed is put back before
  the next reset's settling steps.

Nothing that needs Isaac Sim is imported until the simulator builds the scene, so task files and tests load anywhere.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import inspect
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
AXES = {"X": np.array([1.0, 0.0, 0.0]), "Y": np.array([0.0, 1.0, 0.0]), "Z": np.array([0.0, 0.0, 1.0])}


# == geometry ==========================================================================================================


@dataclass(frozen=True)
class Look:
    """Extras for one box (its fourth element). ``turn``: (roll, pitch, yaw) rad about the frame's x, y, z in that
    order. ``ghost``: drawn, no collider. ``hidden``: collides, not drawn (a stop inside a housing). ``label``: its own
    prim name and material, so :func:`recolor` can change it alone."""

    turn: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ghost: bool = False
    hidden: bool = False
    label: str = ""


def box(centre, size, color: Color, **look) -> tuple:
    """A box spec: (centre, size, colour, Look) in its part's frame, m."""
    return (tuple(float(v) for v in centre), tuple(float(v) for v in size), tuple(color), Look(**look))


def span(lo, hi, color: Color, **look) -> tuple:
    """A box spec from two opposite corners."""
    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    return box((lo + hi) / 2, hi - lo, color, **look)


def hook(seat, gap: float, *, wall: float = 0.005, width: float = 0.020, stem: float = 0.060, lip: float = 0.015,
         color: Color = (0.60, 0.61, 0.63), along: str = "Y") -> list:
    """An inverted-U hook (boxes in its part's frame) that hangs over a pin running ``along`` "Y" or "X". ``seat`` is
    the middle of its inside top face, where it rests on the pin; ``gap`` the opening across the pin. The stem runs
    ``stem`` down on the - side, the lip ``lip`` down on the + side; each wall ``wall`` thick, ``width`` long."""
    sx, sy, sz = (float(v) for v in seat)
    ax = 0 if along == "Y" else 1  # the hook's across direction: x for a pin along y, y for a pin along x

    def part(u0, u1, z0, z1):
        lo, hi = [sx, sy - width / 2, z0], [sx, sy + width / 2, z1]
        if ax == 1:
            lo, hi = [sx - width / 2, sy, z0], [sx + width / 2, sy, z1]
        c = sx if ax == 0 else sy
        lo[ax], hi[ax] = c + u0, c + u1
        return span(lo, hi, color)

    h = gap / 2
    return [part(-h - wall, h + wall, sz, sz + wall), part(-h - wall, -h, sz - stem, sz),
            part(h, h + wall, sz - lip, sz)]


def pin(centre, side: float, length: float, color: Color = (0.62, 0.64, 0.67), *, along: str = "Y",
        round_: bool = False) -> tuple:
    """A pin or rod: a square bar ``side`` across, ``length`` long, running ``along`` "X", "Y" or "Z". ``round_``
    turns it 45 degrees about its length (a diamond: two line contacts on a gripper's pads, as a round bar gives)."""
    size = {"X": (length, side, side), "Y": (side, length, side), "Z": (side, side, length)}[along]
    turn = {"X": (math.pi / 4, 0, 0), "Y": (0, math.pi / 4, 0), "Z": (0, 0, math.pi / 4)}[along] if round_ else (0, 0, 0)
    return box(centre, size, color, turn=turn)


def look_of(b) -> Look:
    return b[3] if len(b) > 3 and isinstance(b[3], Look) else Look()


def rpy(turn) -> np.ndarray:
    """Rotation for (roll, pitch, yaw): about x, then y, then z."""
    r, p, y = (float(v) for v in turn)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_rotation(axis, angle: float) -> np.ndarray:
    k = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def quat_xyzw(rot) -> tuple[float, float, float, float]:
    m = np.asarray(rot, dtype=float)
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return (float(x), float(y), float(z), float(w))


def quat_matrix(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def corners(b) -> np.ndarray:
    """The eight corners of a box spec in its part's frame."""
    c, s = np.asarray(b[0], dtype=float), np.asarray(b[1], dtype=float)
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], dtype=float)
    return c + (rpy(look_of(b).turn) @ (signs * s / 2).T).T


# == mechanisms ========================================================================================================


@dataclass(frozen=True)
class Joint:
    """A moving part on a slider (``kind`` "slide", m) or a hinge ("hinge", rad). ``at`` is the joint's anchor and the
    part's frame origin in the mech frame with every joint at 0; the part's ``boxes`` are in that frame. ``axis`` is
    "X", "Y" or "Z" of the mech frame. It moves between ``lower`` and ``upper``; a spring of ``rate`` (N/m or N m/rad)
    pulls it to ``target`` (a target beyond a limit is a preload); ``damping`` is viscous; ``friction`` (N or N m) is
    its dry friction, static = sliding. ``start`` is where it is at reset (keep it 0 unless needed). ``weightless``:
    gravity off for the part (a counterbalanced or spring-carried part, so its weight never loads the joint)."""

    name: str
    kind: str
    at: tuple[float, float, float]
    axis: str = "Z"
    lower: float = -0.01
    upper: float = 0.01
    boxes: tuple = ()
    rate: float = 0.0
    target: float = 0.0
    damping: float = 0.0
    friction: float = 0.0
    mass: float = 0.05
    parent: str = ""
    weightless: bool = False
    armature: float = 0.0
    start: float = 0.0

    @property
    def joint_name(self) -> str:
        return f"{self.name}_{'slide' if self.kind == 'slide' else 'turn'}"

    def motion(self, q: float) -> tuple[np.ndarray, np.ndarray]:
        """(rotation, translation) of the part's frame at position ``q`` relative to its rest frame."""
        a = AXES[self.axis]
        if self.kind == "slide":
            return np.eye(3), a * float(q)
        return axis_rotation(a, float(q)), np.zeros(3)


def slider(name, at, axis, lower, upper, boxes, **kw) -> Joint:
    return Joint(name=name, kind="slide", at=tuple(float(v) for v in at), axis=axis, lower=float(lower),
                 upper=float(upper), boxes=tuple(boxes), **kw)


def hinge(name, at, axis, lower, upper, boxes, **kw) -> Joint:
    """A hinge; ``lower``/``upper`` in rad (``math.radians``). Keep the range within a full turn either way."""
    return Joint(name=name, kind="hinge", at=tuple(float(v) for v in at), axis=axis, lower=float(lower),
                 upper=float(upper), boxes=tuple(boxes), **kw)


@dataclass(frozen=True)
class Mech(Box):
    """A mechanism: a rigid body of ``boxes`` (mech frame) with ``joints``. It subclasses Box so yam_arena's checks
    accept it: ``pos`` is the body origin and ``size`` the bounding box at rest. ``points`` name points: (name, xyz)
    on the body or (name, xyz, joint) on a moving part, in that part's frame. ``surface`` is the colliders' friction
    coefficient (static = dynamic). ``anchored`` joins the body to the world (never moves; GPU-unproven in our tasks:
    a heavy free body standing on the table is the proven way). ``self_collide`` lets parts of the mech touch each
    other (PhysX never lets two bodies joined by a joint touch; MuJoCo skips parent-child pairs too)."""

    boxes: tuple = ()
    joints: tuple = ()
    surface: float = 1.0
    points: tuple = ()
    weightless: bool = False
    roll: float = 0.0
    pitch: float = 0.0
    self_collide: bool = False
    anchored: bool = False
    contact_offset: float = 0.002

    def joint(self, name: str) -> Joint:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"{self.name} has no joint {name!r}; it has {[j.name for j in self.joints]}")

    def start_rotation(self) -> np.ndarray:
        return rpy((self.roll, self.pitch, self.yaw))

    def frames(self, q: dict | None = None) -> dict:
        """Each part's frame in the mech frame: {"" (the body) or joint name: (rotation, translation)}."""
        q = q or {}
        out = {"": (np.eye(3), np.zeros(3))}
        pending = list(self.joints)
        while pending:
            progressed = False
            for j in list(pending):
                if j.parent in out:
                    pr, pt = out[j.parent]
                    rest = np.asarray(j.at) - (np.asarray(self.joint(j.parent).at) if j.parent else 0.0)
                    mr, mt = j.motion(q.get(j.name, j.start))
                    out[j.name] = (pr @ mr, pt + pr @ (rest + mt))
                    pending.remove(j)
                    progressed = True
            if not progressed:
                raise ValueError(f"{self.name}: joints {[j.name for j in pending]} hang from a missing parent")
        return out

    def _check(self) -> None:
        super()._check()
        if not self.boxes:
            raise ValueError(f"{self.name}: a Mech needs at least one box")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Mech needs a mass (anchored or not)")
        names = [j.name for j in self.joints]
        if len(set(names)) != len(names):
            raise ValueError(f"{self.name}: joint names repeat: {names}")
        for j in self.joints:
            if j.kind not in ("slide", "hinge") or j.axis not in AXES:
                raise ValueError(f"{self.name}.{j.name}: kind slide/hinge and axis X/Y/Z")
            if not (j.lower <= j.start <= j.upper) or j.lower >= j.upper:
                raise ValueError(f"{self.name}.{j.name}: needs lower < upper with start between them")
            if j.kind == "hinge" and (j.upper - j.lower) > 2 * math.pi:
                raise ValueError(f"{self.name}.{j.name}: a limited hinge turns at most a full turn")
            if j.mass <= 0 or min(j.rate, j.damping, j.friction, j.armature) < 0:
                raise ValueError(f"{self.name}.{j.name}: positive mass, no negative gains")
            if not j.boxes:
                raise ValueError(f"{self.name}.{j.name}: a moving part needs boxes")
        self.frames()


def mech(name, pos, boxes, mass, *, joints=(), yaw=0.0, roll=0.0, pitch=0.0, color=(0.5, 0.5, 0.5), surface=1.0,
         points=(), weightless=False, self_collide=False, anchored=False, contact_offset=0.002) -> Mech:
    """Build a Mech; its bounding box is taken from every box at rest."""
    tmp = Mech(name=name, pos=tuple(float(v) for v in pos), size=(1, 1, 1), mass=mass, boxes=tuple(boxes),
               joints=tuple(joints))
    fr = tmp.frames()
    pts = [corners(b) for b in boxes]
    for j in joints:
        r, t = fr[j.name]
        pts += [t + (r @ corners(b).T).T for b in j.boxes]
    pts = np.concatenate(pts)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return Mech(name=name, pos=tuple(float(v) for v in pos), size=tuple(float(v) for v in hi - lo), mass=float(mass),
                color=tuple(color), yaw=float(yaw), boxes=tuple(boxes), joints=tuple(joints), surface=float(surface),
                points=tuple(points), weightless=weightless, roll=float(roll), pitch=float(pitch),
                self_collide=self_collide, anchored=anchored, contact_offset=float(contact_offset))


@dataclass(frozen=True)
class MechState(ObjectState):
    """A mech as the simulator has it: body pose (``pos`` is the body origin), velocities, world bounds, joint
    positions ``q`` and speeds ``qd`` by joint name, named points in the world."""

    q: dict = field(default_factory=dict)
    qd: dict = field(default_factory=dict)
    named: dict = field(default_factory=dict)
    spec: object = None

    def rotation(self) -> np.ndarray:
        return quat_matrix(self.quat_xyzw)

    def to_world(self, local, part: str = "") -> np.ndarray:
        """A point given in a part's frame ("" the body, or a joint name) in the world, at the current q."""
        r, t = self.spec.frames(self.q)[part]
        return self.pos + self.rotation() @ (t + r @ np.asarray(local, dtype=float))

    def point(self, name: str) -> np.ndarray:
        return self.named[name]

    def tilt_deg(self) -> float:
        return float(np.degrees(np.arccos(np.clip(self.rotation()[2, 2], -1.0, 1.0))))


def mech_state(spec: Mech, pos, quat_xyzw=(0, 0, 0, 1), lin_vel=(0, 0, 0), ang_vel=(0, 0, 0), q=None,
               qd=None) -> MechState:
    """A MechState from a body pose and joint positions: what the simulator reports, and what tests build by hand."""
    pos, quat = np.asarray(pos, dtype=float), np.asarray(quat_xyzw, dtype=float)
    q = {j.name: float(j.start) for j in spec.joints} | dict(q or {})
    rot = quat_matrix(quat)
    fr = spec.frames(q)
    pts = [corners(b) for b in spec.boxes if not look_of(b).ghost]
    for j in spec.joints:
        r, t = fr[j.name]
        pts += [t + (r @ corners(b).T).T for b in j.boxes if not look_of(b).ghost]
    pts = pos + (rot @ np.concatenate(pts).T).T
    named = {}
    for p in spec.points:
        part = p[2] if len(p) > 2 else ""
        r, t = fr[part]
        named[p[0]] = pos + rot @ (t + r @ np.asarray(p[1], dtype=float))
    return MechState(pos=pos, quat_xyzw=quat, lin_vel=np.asarray(lin_vel, dtype=float),
                     ang_vel=np.asarray(ang_vel, dtype=float), low=pts.min(0), high=pts.max(0),
                     q={k: float(v) for k, v in q.items()},
                     qd={j.name: 0.0 for j in spec.joints} | {k: float(v) for k, v in (qd or {}).items()},
                     named=named, spec=spec)


# == authoring (Isaac Lab, or physcheck's recording stage) =============================================================

MECHS: dict[str, Mech] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec = MECHS[cfg.mech_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
    material = f"{prim_path}/Looks/surface"
    surf = sim_utils.RigidBodyMaterialCfg(static_friction=spec.surface, dynamic_friction=spec.surface)
    surf.func(material, surf)
    looks: dict = {}

    def look(color, label="") -> str:
        key = label or color
        if key not in looks:
            path = f"{prim_path}/Looks/{label or f'c{len(looks)}'}"
            shader = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            shader.func(path, shader)
            looks[key] = path
        return looks[key]

    def body(path, pos, rot, mass, weightless):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(float(v) for v in pos)))
        x, y, z, w = quat_xyzw(rot)
        xf.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        if spec.weightless or weightless:
            rb.CreateDisableGravityAttr(True)
        return prim

    def add_box(parent, i, b):
        lk = look_of(b)
        cube = UsdGeom.Cube.Define(stage, f"{parent}/{lk.label or f'box_{i}'}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*b[0]))
        if any(lk.turn):
            x, y, z, w = quat_xyzw(rpy(lk.turn))
            cube.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        cube.AddScaleOp().Set(Gf.Vec3f(*b[1]))
        prim = cube.GetPrim()
        if not lk.ghost:
            UsdPhysics.CollisionAPI.Apply(prim)
            contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            contact.CreateContactOffsetAttr(spec.contact_offset)
            contact.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(prim.GetPath()), material, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(b[2], lk.label), stage=stage)
        if lk.hidden:
            UsdGeom.Imageable(prim).MakeInvisible()

    main = f"{prim_path}/body"
    body(main, (0, 0, 0), np.eye(3), spec.mass, False)
    for i, b in enumerate(spec.boxes):
        add_box(main, i, b)
    frames = spec.frames({j.name: 0.0 for j in spec.joints})  # bodies authored at q = 0; ``start`` is the reset state
    for j in spec.joints:
        path = f"{prim_path}/{j.name}"
        r, t = frames[j.name]
        body(path, t, r, j.mass, j.weightless)
        for i, b in enumerate(j.boxes):
            add_box(path, i, b)
        cls = UsdPhysics.PrismaticJoint if j.kind == "slide" else UsdPhysics.RevoluteJoint
        joint = cls.Define(stage, f"{path}/{j.joint_name}")
        joint.CreateAxisAttr(j.axis)
        parent = f"{prim_path}/{j.parent}" if j.parent else main
        joint.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        anchor = np.asarray(j.at) - (np.asarray(spec.joint(j.parent).at) if j.parent else 0.0)
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in anchor)))
        joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        per = 1.0 if j.kind == "slide" else math.pi / 180  # USD counts revolute limits and gains in degrees
        conv = 1.0 if j.kind == "slide" else 180 / math.pi
        joint.CreateLowerLimitAttr(j.lower * conv)
        joint.CreateUpperLimitAttr(j.upper * conv)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear" if j.kind == "slide" else "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(j.rate * per)
        drive.CreateDampingAttr(j.damping * per)
        drive.CreateTargetPositionAttr(j.start * conv)
    if spec.anchored:
        fixed = UsdPhysics.FixedJoint.Define(stage, f"{main}/world_joint")
        fixed.CreateBody1Rel().SetTargets([Sdf.Path(main)])
    if spec.joints or spec.anchored:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
        art.CreateEnabledSelfCollisionsAttr(bool(spec.self_collide))
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _has_field(cls, name: str) -> bool:
    return name in getattr(cls, "__dataclass_fields__", {}) or hasattr(cls, name) or name in getattr(cls, "_defaults", {})


def _arena_object(spec: Mech):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class MechSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        mech_name: str = ""

    class MechObject(Object):
        def _generate_articulation_cfg(self):
            # SI here (m, rad, N/m, N m/rad). Isaac Lab 3.0 takes ``friction`` as the static effort only; the sliding
            # effort is set equal to it, so a joint never runs free once it breaks loose (physcheck: frictionloss).
            sliding = _has_field(ImplicitActuatorCfg, "dynamic_friction")
            acts = {}
            for j in spec.joints:
                kw = dict(joint_names_expr=[j.joint_name], stiffness=j.rate, damping=j.damping, friction=j.friction)
                if sliding:
                    kw["dynamic_friction"] = j.friction
                if j.armature:
                    kw["armature"] = j.armature
                acts[j.name] = ImplicitActuatorCfg(**kw)
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=acts)
            if any(j.start for j in spec.joints):  # one exact name per joint: overlapping patterns are an error
                cfg.init_state.joint_pos = {j.joint_name: float(j.start) for j in spec.joints}
            return self._add_initial_pose_to_cfg(cfg)

    articulated = bool(spec.joints) or spec.anchored
    return MechObject(name=spec.name, object_type=ObjectType.ARTICULATION if articulated else ObjectType.RIGID,
                      spawner_cfg=MechSpawnerCfg(mech_name=spec.name),
                      initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                        rotation_xyzw=quat_xyzw(spec.start_rotation())))


# == reading ===========================================================================================================


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64) if hasattr(x, "detach") else np.asarray(x, dtype=np.float64)


def _read(environment, spec: Mech) -> MechState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    data = asset.data
    pose = _np(data.root_link_pose_w)[0]
    if hasattr(data, "root_lin_vel_w") and hasattr(data, "root_ang_vel_w"):
        lin, ang = _np(data.root_lin_vel_w)[0], _np(data.root_ang_vel_w)[0]
    elif hasattr(data, "root_vel_w"):
        v = _np(data.root_vel_w)[0]
        lin, ang = v[:3], v[3:6]
    else:  # a silent zero would make every mech look still
        raise RuntimeError(f"ourkit: can't read {spec.name}'s velocity from this Isaac Lab version")
    q, qd = {}, {}
    if spec.joints:
        names = list(asset.joint_names)
        missing = [j.joint_name for j in spec.joints if j.joint_name not in names]
        if missing:
            raise RuntimeError(f"ourkit: {spec.name} has joints {names}, missing {missing}")
        pos, vel = _np(data.joint_pos)[0], _np(data.joint_vel)[0]
        q = {j.name: float(pos[names.index(j.joint_name)]) for j in spec.joints}
        qd = {j.name: float(vel[names.index(j.joint_name)]) for j in spec.joints}
    return mech_state(spec, pose[:3] - origin, pose[3:7], lin, ang, q, qd)


# == hooks =============================================================================================================

_STEP: dict = {}
_RESET: dict = {}
_PRE: dict = {}
_ENV: dict = {"env": None}
LATEST: dict = {"states": None, "steps": 0, "resets": 0}  # last object states, control steps since the reset, resets
_RULES: dict = {}  # rule sets by name: run after every step, before the task's own hooks (rules.py)


def _key(fn):
    return (getattr(fn, "__module__", ""), getattr(fn, "__qualname__", repr(fn)))


def on_step(fn):
    """Register ``fn(states)``: after each control step, after the mechanism rules, before scoring. A task file
    loaded again replaces its earlier hooks."""
    _STEP[_key(fn)] = fn
    return fn


def on_reset(fn):
    """Register ``fn(states)``: after every reset, once objects have settled (rules reset first)."""
    _RESET[_key(fn)] = fn
    return fn


def pre_reset(fn):
    """Register ``fn()``: before a reset's settling steps (the kit already restores everything it changed)."""
    _PRE[_key(fn)] = fn
    return fn


def env():
    return _ENV["env"]


# == runtime writes ====================================================================================================

_AUTHORED: dict = {}  # (mech, joint) -> what was changed during the trial, to put back
_TARGET: dict = {}  # (mech, joint) -> the spring target now


def _joint_ids(asset, spec: Mech, joints) -> list[int]:
    names = list(asset.joint_names)
    return [names.index(spec.joint(j).joint_name) for j in joints]


def _sim(e):
    """physcheck's MuJoCo Sim behind its fake env, or None in Isaac."""
    return getattr(e.unwrapped, "sim_ref", None) if e is not None else None


def _tensor(asset, rows):
    import torch

    device = getattr(asset.data.joint_pos, "torch", asset.data.joint_pos).device
    return torch.tensor(rows, dtype=torch.float32, device=device)


def _isaac_write(asset, names, kws, value, ids, quiet: bool = False) -> None:
    """Call the first of ``names`` the asset has, passing ``value`` by whichever of ``kws`` it takes (``quiet``: no
    limit-violation warning, where the call takes that switch)."""
    for n in names:
        fn = getattr(asset, n, None)
        if fn is None:
            continue
        params = inspect.signature(fn).parameters
        kw = next((k for k in kws if k in params), None)
        extra = {"warn_limit_violation": False} if quiet and "warn_limit_violation" in params else {}
        if kw is None:
            fn(value, joint_ids=ids, **extra)
        else:
            fn(**{kw: value, "joint_ids": ids}, **extra)
        return
    raise RuntimeError(f"ourkit: this Isaac Lab has none of {names}")


def _remember(name, joint, what, value):
    _AUTHORED.setdefault((name, joint), {}).setdefault(what, value)


def target_of(name: str, joint: str) -> float:
    """The joint's spring target now (as last set, else as authored)."""
    return _TARGET.get((name, joint), MECHS[name].joint(joint).target if name in MECHS else 0.0)


def set_targets(name: str, targets: dict) -> None:
    """Move joints' spring targets (m or rad) for the rest of the trial or until set again: a preload, a stop moved
    in, a driven joint. Does nothing outside a simulator. Errors go through: a part that can't be driven must stop the
    first short run, not leave the trial quietly wrong."""
    spec = MECHS.get(name)
    for j, v in targets.items():
        if spec is not None:
            _remember(name, j, "target", spec.joint(j).start)
        _TARGET[(name, j)] = float(v)
    e = env()
    if e is None or not targets:
        return
    asset = e.unwrapped.scene[name]
    ids = _joint_ids(asset, spec, targets)
    asset.set_joint_position_target_index(target=_tensor(asset, [[float(v) for v in targets.values()]]), joint_ids=ids)
    asset.write_data_to_sim()


def mj_stable_stiffness(sim, jid: int, k: float, ratio: float = 1.0) -> float:
    """MuJoCo integrates joint springs explicitly (its implicit step covers damping only): keep omega * dt under
    ``ratio`` (stable to 2). A drive lock on a light hinge is softer in physcheck than on the GPU for this."""
    m = sim.m
    dof = int(m.jnt_dofadr[jid])
    inv = max(float(m.dof_invweight0[dof]), 1e-9)
    return min(float(k), (ratio / m.opt.timestep) ** 2 / inv)


def set_gains(name: str, joint: str, stiffness: float | None = None, damping: float | None = None) -> None:
    """Change a joint's spring rate and damping for the rest of the trial (SI). Isaac: ``write_joint_stiffness_to_sim``
    and ``..._damping_...``; physcheck: the MuJoCo joint's stiffness (capped for its explicit integration) and
    damping. Put back by :func:`restore` before the next reset."""
    j = MECHS[name].joint(joint)
    _remember(name, joint, "stiffness", j.rate)
    _remember(name, joint, "damping", j.damping)
    e = env()
    if e is None:
        return
    asset = e.unwrapped.scene[name]
    i = _joint_ids(asset, MECHS[name], [joint])
    sim = _sim(e)
    if sim is not None:
        jid = asset.jids[i[0]]
        if stiffness is not None:
            sim.m.jnt_stiffness[jid] = mj_stable_stiffness(sim, jid, stiffness)
        if damping is not None:
            sim.m.dof_damping[sim.m.jnt_dofadr[jid]] = float(damping)
        return
    if stiffness is not None:
        _isaac_write(asset, ("write_joint_stiffness_to_sim_index", "write_joint_stiffness_to_sim"), ("stiffness",),
                     _tensor(asset, [[float(stiffness)]]), i)
    if damping is not None:
        _isaac_write(asset, ("write_joint_damping_to_sim_index", "write_joint_damping_to_sim"), ("damping",),
                     _tensor(asset, [[float(damping)]]), i)
    asset.write_data_to_sim()


LIMIT_SOLREF = 0.002  # s: physcheck's limit time constant while a rule holds it (MuJoCo's default 0.02 lets a preload
LIMIT_SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)  # push through by mm); floored at 2 physics steps


def set_limits(name: str, joint: str, lower: float, upper: float) -> None:
    """Change a joint's limits (m or rad) for the rest of the trial: a catch, a pawl, a lock point. Isaac:
    ``write_joint_position_limit_to_sim``; physcheck: ``jnt_range``. Put back by :func:`restore` before the next reset."""
    j = MECHS[name].joint(joint)
    _remember(name, joint, "limits", (j.lower, j.upper))
    e = env()
    if e is None:
        return
    asset = e.unwrapped.scene[name]
    i = _joint_ids(asset, MECHS[name], [joint])
    sim = _sim(e)
    if sim is not None:
        jid = asset.jids[i[0]]
        sim.m.jnt_range[jid] = (float(lower), float(upper))
        sim.m.jnt_limited[jid] = 1
        authored = (j.lower, j.upper) == (lower, upper)
        if authored:  # MuJoCo's defaults, as physcheck compiled them
            sim.m.jnt_solref[jid], sim.m.jnt_solimp[jid] = (0.02, 1.0), (0.9, 0.95, 0.001, 0.5, 2.0)
        else:  # a rule's catch: as stiff as MuJoCo allows, so a preload doesn't push through it
            sim.m.jnt_solref[jid] = (max(LIMIT_SOLREF, 2 * sim.m.opt.timestep), 1.0)
            sim.m.jnt_solimp[jid] = LIMIT_SOLIMP
        return
    # Isaac Lab clamps the default joint positions into new limits (and so the next reset's start): keep them
    default = getattr(asset.data, "default_joint_pos", None)
    default = getattr(default, "torch", default)
    saved = default.clone() if hasattr(default, "clone") else None
    _isaac_write(asset, ("write_joint_position_limit_to_sim_index", "write_joint_position_limit_to_sim",
                         "write_joint_limits_to_sim"), ("limits", "joint_limits"),
                 _tensor(asset, [[[float(lower), float(upper)]]]), i, quiet=True)
    if saved is not None:
        now = getattr(asset.data.default_joint_pos, "torch", asset.data.default_joint_pos)
        now.copy_(saved)
    asset.write_data_to_sim()


def restore() -> None:
    """Put back every gain, limit and target a trial changed (called before each reset's settling steps)."""
    changed = list(_AUTHORED.items())
    _AUTHORED.clear()
    _TARGET.clear()
    if env() is None:
        return
    for (name, joint), what in changed:
        if name not in MECHS:
            continue
        j = MECHS[name].joint(joint)
        if "limits" in what:
            set_limits(name, joint, j.lower, j.upper)
        if "stiffness" in what or "damping" in what:
            set_gains(name, joint, j.rate, j.damping)
        if "target" in what:
            set_targets(name, {joint: j.start})
    _AUTHORED.clear()
    _TARGET.clear()


def apply_authored_starts() -> None:
    """After a reset, in Isaac: put every joint with ``start`` != 0 at its start, at rest.

    Nov 20 (lb7, height gauge GPU run 1): an undriven friction slider authored with start 0.060 read 0.0009 m on every
    Isaac reset (the articulation's init_state.joint_pos did not reach the reset state; the shower hinge in lessons.md
    showed the same for a driven joint). MuJoCo (physcheck) applies ``start`` itself, so this writes in Isaac only."""
    e = env()
    if e is None or _sim(e) is not None:
        return
    for name, spec in MECHS.items():
        moved = [j for j in spec.joints if j.start != 0]
        if not moved:
            continue
        asset = e.unwrapped.scene[name]
        ids = _joint_ids(asset, spec, [j.name for j in moved])
        _isaac_write(asset, ("write_joint_position_to_sim_index", "write_joint_position_to_sim"), ("position", "joint_pos"),
                     _tensor(asset, [[float(j.start) for j in moved]]), ids)
        _isaac_write(asset, ("write_joint_velocity_to_sim_index", "write_joint_velocity_to_sim"), ("velocity", "joint_vel"),
                     _tensor(asset, [[0.0 for _ in moved]]), ids)
        asset.write_data_to_sim()


def apply_authored_targets() -> None:
    """After a reset: each joint's spring target to its authored ``target`` (the actuator starts at ``start``)."""
    for name, spec in MECHS.items():
        # Nov 20 (ls8, vise GPU run 1): Isaac's default drive target is 0, so a joint with start != 0 and target == start
        # was pulled to 0 during the reset settle (MuJoCo reads the USD target and never showed it). Write the target
        # whenever it differs from Isaac's default too.
        moved = {j.name: j.target for j in spec.joints if j.target != j.start or j.start != 0}
        if moved:
            set_targets(name, moved)


def recolor(name: str, srgb: Color) -> None:
    """Change a prim's colour mid-trial (a labelled box's ``label``). Cosmetic: never stops a trial."""
    e = env()
    if e is None:
        return
    try:
        from pxr import Gf, Usd, UsdShade

        from yam_arena.station import linear_rgb

        stage = e.unwrapped.sim.stage
        prims = [p for p in stage.Traverse() if p.GetName() == name and "env_0" in str(p.GetPath())]
        colour = Gf.Vec3f(*linear_rgb(srgb))
        for root in prims[:1]:
            for p in Usd.PrimRange(root):
                shaders = [UsdShade.Shader(p)] if p.IsA(UsdShade.Shader) else []
                mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
                if mat:
                    shaders += [UsdShade.Shader(s) for s in Usd.PrimRange(mat.GetPrim()) if s.IsA(UsdShade.Shader)]
                for sh in shaders:
                    for k in ("diffuseColor", "diffuse_color_constant"):
                        port = sh.GetInput(k)
                        if port:
                            port.Set(colour)
    except Exception as err:  # noqa: BLE001  (colour is cosmetic)
        print(f"[ourkit] could not recolour {name!r}: {err}")


# == patches ===========================================================================================================


def _patch_scene(scene) -> None:
    if getattr(scene, "_ourkit", False):
        return
    plain_object, plain_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Mech):
            MECHS[spec.name] = spec
            return _arena_object(spec)
        return plain_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in MECHS]
        out = plain_states(environment, plain) if plain else {}
        for n in names:
            if n in MECHS:
                out[n] = _read(environment, MECHS[n])
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._ourkit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_ourkit", False):
        return
    plain_step, plain_reset = cls._task_step, cls.reset

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self._env, self.task.names)

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        now = states(self)
        LATEST["states"], LATEST["steps"] = now, LATEST["steps"] + 1
        for rs in list(_RULES.values()):
            rs.step(now)
        for fn in list(_STEP.values()):
            fn(now)
        return plain_step(self, *a, **k)

    def reset(self, *a, **k):
        if getattr(self, "_env", None) is not None:
            _ENV["env"] = self._env
            restore()
            for fn in list(_PRE.values()):
                fn()
        obs = plain_reset(self, *a, **k)
        _ENV["env"] = self._env
        _AUTHORED.clear()
        _TARGET.clear()
        apply_authored_starts()  # before the first reading: Isaac leaves every joint at 0 otherwise
        now = states(self)
        LATEST["states"], LATEST["steps"] = now, 0
        LATEST["resets"] = LATEST.get("resets", 0) + 1
        apply_authored_targets()
        for rs in list(_RULES.values()):
            rs.reset(now)
        for fn in list(_RESET.values()):
            fn(now)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._ourkit = True


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
        plain_exec = loader.exec_module

        def exec_module(module):
            plain_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena now for modules already imported, otherwise when they are. Safe to call twice."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
