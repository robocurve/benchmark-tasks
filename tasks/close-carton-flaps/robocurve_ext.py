"""Spring hinges, slides and compound (holed) parts for yam_arena task files, shipped inside the task folder (no
edits to yam_arena).

A compound part is a loose or fixed ``Box`` in ``OBJECTS`` whose size is the part's bounding box;
``install({"ring": Compound(pieces=...)})`` replaces its single box with several box pieces in one rigid body, so a
part can have a hole, a slot or a step.

A jointed part is an ordinary loose ``Box`` in ``OBJECTS`` (so the task's hooks read its pose like any other
object). ``install({"lid": Revolute(...)})`` asks yam_arena, when it builds the scene, to tie that box to the world
with a PhysX joint: a revolute hinge or a prismatic slide, with limits and an optional spring drive.

How it hooks in: ``task.py`` is imported before Isaac starts, and ``yam_arena.scene`` (which needs Isaac) is imported
only once the simulator is up. ``install()`` therefore registers an import hook that wraps
``yam_arena.scene.task_object`` as soon as that module loads; the wrapper swaps the jointed box's spawn function for
one that spawns the box and then adds the joint prim next to it. Nothing here imports Isaac until then, so offline
tests can import the task file.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

_SCENE = "yam_arena.scene"
JOINTS: dict[str, "Revolute | Prismatic | Spherical | Compound | Mesh"] = {}


@dataclass(frozen=True)
class Revolute:
    """A hinge through ``anchor`` (m, in the box's own frame at spawn) about the box's local ``axis`` ("X"/"Y"/"Z").
    Angles in degrees from the spawn pose (positive = right-handed about the axis). ``stiffness`` in N*m/deg pulls
    toward ``target``; ``damping`` in N*m*s/deg. stiffness 0 = a free hinge (with damping as viscous friction)."""

    anchor: tuple[float, float, float]
    axis: str
    lower: float
    upper: float
    target: float = 0.0
    stiffness: float = 0.0
    damping: float = 0.0

    parent: str | None = None  # another loose box listed earlier in OBJECTS; None = tied to the world
    initial: float = 0.0  # spawn this far from the closed/zero pose (deg or m); the spec's pose is zero

    kind = "angular"


@dataclass(frozen=True)
class Spherical:
    """A ball joint at ``anchor`` (m, box frame): the box may tilt up to ``cone_deg`` away from its spawn pose about
    the two axes normal to ``axis``, and turn freely about ``axis`` (a trailer tongue on its axle and wheels)."""

    anchor: tuple[float, float, float]
    axis: str = "Z"
    cone_deg: float = 30.0
    initial: float = 0.0

    kind = "angular"


@dataclass(frozen=True)
class Piece:
    """One box of a compound part: ``size`` (m) and ``offset`` of its centre (m) in the part's frame;
    ``color`` None = the part's colour."""

    size: tuple[float, float, float]
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: tuple[float, float, float] | None = None
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # tilt a piece (chamfers, funnels)


@dataclass(frozen=True)
class Compound:
    pieces: tuple[Piece, ...]


@dataclass(frozen=True)
class Mesh:
    """A part whose collision and look come from a triangle OBJ file shipped in the task folder (metres, origin at
    the ``Box``'s centre, so the ``Box`` size is the mesh's bounding box). ``approximation``: "convexDecomposition"
    or "convexHull" for loose parts, "none" (the exact triangle mesh) for fixed parts only (PhysX rule). An OBJ with
    several ``o`` objects gives one collider per object (a non-convex loose part as convex pieces, each "convexHull")."""

    path: str
    approximation: str = "convexDecomposition"


def read_obj(path: str):
    """Vertices [(x, y, z)] and faces [(i, j, k, ...)] (0-based) of an OBJ file; pure Python for offline tests."""
    pts, faces = [], []
    for _, p, f in read_obj_parts(path):
        faces += [tuple(i + len(pts) for i in t) for t in f]
        pts += p
    return pts, faces


def read_obj_parts(path: str):
    """[(name, vertices, faces)] per ``o`` object of an OBJ file (one part if it has none), faces re-indexed per part."""
    parts, pts, faces, name = [], [], [], "shape"
    with open(path) as f:
        for line in f:
            w = line.split()
            if not w:
                continue
            if w[0] == "o" and (pts or faces):
                parts.append((name, pts, faces))
                pts, faces = [], []
            if w[0] == "o":
                name = w[1] if len(w) > 1 else f"part{len(parts)}"
            elif w[0] == "v":
                pts.append(tuple(float(v) for v in w[1:4]))
            elif w[0] == "f":
                faces.append(tuple(int(v.split("/")[0]) - 1 for v in w[1:]))
    parts.append((name, pts, faces))
    base = 0
    out = []
    for n, p, fs in parts:  # OBJ indices are global; make them per part
        out.append((n, p, [tuple(i - base for i in t) for t in fs]))
        base += len(p)
    return out


@dataclass(frozen=True)
class Prismatic:
    """A slide along the box's local ``axis`` through ``anchor``; travel in metres from the spawn pose.
    ``stiffness`` in N/m pulls toward ``target``; ``damping`` in N*s/m."""

    anchor: tuple[float, float, float]
    axis: str
    lower: float
    upper: float
    target: float = 0.0
    stiffness: float = 0.0
    damping: float = 0.0

    parent: str | None = None  # another loose box listed earlier in OBJECTS; None = tied to the world
    initial: float = 0.0  # spawn this far from the closed/zero pose (deg or m); the spec's pose is zero

    kind = "linear"


def install(joints: dict) -> None:
    """Register boxes by object name: a joint, a Compound, or a (Compound, joint) pair for a compound part on a joint.
    Safe to call more than once (the task file may be loaded twice)."""
    JOINTS.update(joints)
    if _SCENE in sys.modules:
        _patch(sys.modules[_SCENE])
    elif not any(isinstance(f, _PatchOnImport) for f in sys.meta_path):
        sys.meta_path.insert(0, _PatchOnImport())


class _PatchOnImport(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name != _SCENE:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(name, path, target)
            if spec is not None:
                break
        else:
            return None
        run = spec.loader.exec_module

        def exec_module(module):
            run(module)
            _patch(module)

        spec.loader.exec_module = exec_module
        return spec


def _patch(scene) -> None:
    if getattr(scene, "_robocurve_joints", False):
        return
    original = scene.task_object

    def task_object(spec):
        obj = original(spec)
        entry = JOINTS.get(spec.name)
        items = entry if isinstance(entry, tuple) else (() if entry is None else (entry,))
        compound = next((i for i in items if isinstance(i, (Compound, Mesh))), None)
        joint = next((i for i in items if not isinstance(i, (Compound, Mesh))), None)
        zero = (tuple(float(v) for v in spec.pos), _yaw_quat_xyzw(spec.yaw))
        ZERO_POSE[spec.name] = zero
        spawn = zero
        if joint is not None:
            spawn = _moved_pose(zero, joint) if joint.initial else zero
            parent = getattr(joint, "parent", None)
            if parent is not None:
                if parent not in SPAWN_POSE:
                    raise ValueError(f"{spec.name}: joint parent {parent!r} must be listed before it in OBJECTS")
                # carry the child along with its parent's own initial move
                spawn = _compose(_compose(SPAWN_POSE[parent], _inverse(ZERO_POSE[parent])), spawn)
            if spawn != zero:
                from isaaclab_arena.utils.pose import Pose

                obj.initial_pose = Pose(position_xyz=tuple(spawn[0]), rotation_xyzw=tuple(spawn[1]))
                obj.object_cfg = obj._init_object_cfg()
                obj._pose_event_cfg = obj._build_reset_event()
        SPAWN_POSE[spec.name] = spawn
        if compound is not None or joint is not None:
            # Object() builds object_cfg (a copy of the spawn cfg) at construction, so swap the spawn function there
            spawn_cfg = obj.object_cfg.spawn
            spawn_cfg.func = _spawner(spawn_cfg.func, spec.name, compound, joint)
            obj.spawner_cfg = spawn_cfg
        return obj

    scene.task_object = task_object
    scene._robocurve_joints = True
    print(f"robocurve ext: yam_arena.scene.task_object wrapped for {sorted(JOINTS)}", flush=True)


ZERO_POSE: dict[str, tuple] = {}  # every object's pose as written in OBJECTS (its joint's zero)
SPAWN_POSE: dict[str, tuple] = {}  # where it actually spawns (after its own and its parent's initial moves)


def _inverse(pose):
    (px, py, pz), (x, y, z, w) = pose
    qi = (-x, -y, -z, w)
    ip = _qrot(qi, (px, py, pz))
    return (-ip[0], -ip[1], -ip[2]), qi


def _compose(a, b):
    """a then b, both (pos, quat_xyzw): the pose b expressed in a's frame, mapped to the world."""
    (ap, aq), (bp, bq) = a, b
    r = _qrot(aq, bp)
    return (ap[0] + r[0], ap[1] + r[1], ap[2] + r[2]), _qmul(aq, bq)


def _yaw_quat_xyzw(yaw):
    import math

    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    p = _qmul(_qmul(q, (vx, vy, vz, 0.0)), (-x, -y, -z, w))
    return p[:3]


def _moved_pose(zero, joint):
    """The box's pose with the joint at ``joint.initial`` (deg about, or m along, the axis) from its zero pose."""
    import math

    pos, rot = zero
    axis = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[joint.axis]
    u = _qrot(rot, axis)
    if isinstance(joint, Prismatic):
        return tuple(p + joint.initial * c for p, c in zip(pos, u)), rot
    a = [p + c for p, c in zip(pos, _qrot(rot, joint.anchor))]
    h = math.radians(joint.initial) / 2
    turn = (u[0] * math.sin(h), u[1] * math.sin(h), u[2] * math.sin(h), math.cos(h))
    rel = _qrot(turn, [p - c for p, c in zip(pos, a)])
    return tuple(c + r for c, r in zip(a, rel)), _qmul(turn, rot)


def _callable(func):
    """configclass copies may hold a spawn function as its "module:function" string; return the function."""
    if isinstance(func, str):
        from isaaclab.utils.string import string_to_callable

        func = string_to_callable(func)
    return getattr(func, "__wrapped__", func)


def _spawner(base, name, compound, joint):
    from isaaclab.sim.utils import clone

    @clone
    def spawn(prim_path, cfg, *args, **kwargs):
        if isinstance(compound, Mesh):
            prim = _spawn_mesh(base, compound, prim_path, cfg, *args, **kwargs)
        elif compound is not None:
            prim = _spawn_compound(base, compound, prim_path, cfg, *args, **kwargs)
        else:
            prim = _callable(base)(prim_path, cfg, *args, **kwargs)
        if joint is not None:
            _add_joint(prim, name, joint)
        return prim

    return spawn


def _spawn_compound(base, compound, prim_path, cfg, *args, **kwargs):
    """The root keeps the rigid body and mass; its own box is removed and every piece is a child collider."""
    from yam_arena.station import linear_rgb

    prim = _callable(base)(prim_path, cfg.replace(size=tuple(compound.pieces[0].size)), *args, **kwargs)
    prim.GetStage().RemovePrim(prim_path + "/geometry")
    for i, piece in enumerate(compound.pieces):
        piece_cfg = cfg.replace(size=tuple(piece.size), rigid_props=None, mass_props=None)
        if piece.color is not None:
            piece_cfg = piece_cfg.replace(visual_material=cfg.visual_material.replace(diffuse_color=linear_rgb(piece.color)))
        _callable(base)(f"{prim_path}/piece_{i}", piece_cfg, translation=tuple(piece.offset),
                        orientation=tuple(piece.rot_xyzw))
    print(f"robocurve_ext: compound {prim_path} with {len(compound.pieces)} pieces", flush=True)
    return prim


def _spawn_mesh(base, mesh, prim_path, cfg, *args, **kwargs):
    """Spawn the box as usual (rigid body, mass, materials, contact settings), then swap its cube for the OBJ mesh,
    carrying over the cube's collision schemas, attributes and material bindings."""
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    prim = _callable(base)(prim_path, cfg, *args, **kwargs)
    stage = prim.GetStage()
    cube = stage.GetPrimAtPath(prim_path + "/geometry/mesh")
    parts = read_obj_parts(mesh.path)
    for i, (_, pts, faces) in enumerate(parts):
        m = UsdGeom.Mesh.Define(stage, prim_path + ("/geometry/shape" if len(parts) == 1 else f"/geometry/shape_{i}"))
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
        m.CreateSubdivisionSchemeAttr("none")
        shape = m.GetPrim()
        for schema in cube.GetAppliedSchemas():
            shape.AddAppliedSchema(schema)
        for attr in cube.GetAuthoredAttributes():
            name = attr.GetName()
            if ":" in name and not name.startswith(("xformOp", "primvars")):
                shape.CreateAttribute(name, attr.GetTypeName()).Set(attr.Get())
        for rel in cube.GetRelationships():
            shape.CreateRelationship(rel.GetName()).SetTargets(rel.GetTargets())
        UsdPhysics.MeshCollisionAPI.Apply(shape).CreateApproximationAttr(mesh.approximation)
    stage.RemovePrim(cube.GetPath())
    print(f"robocurve_ext: mesh {prim_path} from {mesh.path.rsplit('/', 1)[-1]}: {len(parts)} part(s), "
          f"{sum(len(p) for _, p, _ in parts)} vertices, collision {mesh.approximation}", flush=True)
    return prim


def _add_joint(prim, name, joint) -> None:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = prim.GetStage()
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    parent = getattr(joint, "parent", None)
    if parent is not None:
        # frame0 in the parent's body frame, both at their zero poses
        rel = _compose(_inverse(ZERO_POSE[parent]), ZERO_POSE[name])
        (px, py, pz), (qx, qy, qz, qw) = rel
        m = Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Quatd(qw, qx, qy, qz)), Gf.Vec3d(px, py, pz))
    elif name in ZERO_POSE and SPAWN_POSE.get(name, ZERO_POSE[name]) != ZERO_POSE[name]:
        # the box spawns at its initial (moved) pose; the joint's frames sit at the zero pose
        (px, py, pz), (qx, qy, qz, qw) = ZERO_POSE[name]
        local = Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Quatd(qw, qx, qy, qz)), Gf.Vec3d(px, py, pz))
        m = local * UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    anchor_local = Gf.Vec3d(*joint.anchor)
    anchor_world = m.Transform(anchor_local)
    rot_world = m.ExtractRotationQuat()
    path = prim.GetPath().GetParentPath().AppendChild(f"{name}_joint")
    if isinstance(joint, Spherical):
        j = UsdPhysics.SphericalJoint.Define(stage, path)
        j.CreateAxisAttr(joint.axis)
        j.CreateConeAngle0LimitAttr(float(joint.cone_deg))
        j.CreateConeAngle1LimitAttr(float(joint.cone_deg))
    else:
        j = (UsdPhysics.RevoluteJoint if isinstance(joint, Revolute) else UsdPhysics.PrismaticJoint).Define(stage, path)
        j.CreateAxisAttr(joint.axis)
        j.CreateLowerLimitAttr(float(joint.lower))
        j.CreateUpperLimitAttr(float(joint.upper))
    if parent is not None:
        j.CreateBody0Rel().SetTargets([prim.GetParent().GetPath().AppendChild(parent)])
    # with no body0 the joint ties the box to the world frame
    j.CreateBody1Rel().SetTargets([prim.GetPath()])
    j.CreateLocalPos0Attr(Gf.Vec3f(anchor_world))
    j.CreateLocalRot0Attr(Gf.Quatf(rot_world))
    j.CreateLocalPos1Attr(Gf.Vec3f(anchor_local))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    if getattr(joint, "stiffness", 0.0) or getattr(joint, "damping", 0.0):
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), joint.kind)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(joint.stiffness))
        drive.CreateDampingAttr(float(joint.damping))
        drive.CreateTargetPositionAttr(float(joint.target))
    print(f"robocurve_ext: {name} {type(joint).__name__} at {tuple(round(v, 4) for v in anchor_world)} axis {joint.axis}",
          flush=True)
