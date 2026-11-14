"""simready: NVIDIA SimReady props (OpenUSD) in a yam_arena task file.

yam_arena's task files spawn primitives and models from Arena's object library by name. This module adds
:class:`SimReady`, a prop from NVIDIA's SimReady library on Hugging Face (``nvidia/simready-assets``), pinned to one
dataset revision. Its files are downloaded once into ``~/.cache/simready`` (``SIMREADY_CACHE`` overrides) the first
time the scene is built, so nothing large is committed here. The props are CC-BY 4.0 (see README.md).

``SimReady`` subclasses ``yam_arena.task_file.Box`` only so the task-file checks accept it. Importing this module
hooks ``yam_arena.scene.task_object`` when that module is imported (after Isaac Sim starts), without changing
yam_arena. ``success()`` and ``score()`` get the usual ``ObjectState`` for these props.

At spawn, each prop's single rigid body is set up for the task:

- ``mass=None``: the body is kinematic (fixed where it is placed), like a fixed ``Asset``;
- a ``mass``: the body gets that mass, and its authored inertia is scaled to match;
- its ``Grasp`` guide curves (invisible annotations) are deactivated, so they don't count in its bounds;
- a prop with several bodies on joints (pliers, an adjustable wrench) is made one rigid body: its joints are
  deactivated and its bodies merged into their common parent, so it moves as one piece;
- ``strip``: parts of a mesh are removed before the simulation starts (a drill's moulded-on battery pack, which the
  task replaces with a pack of its own), so they are neither drawn nor collide;
- ``convex=True``: its meshes collide as convex hulls instead of signed distance fields (SDF), with contacts starting
  5 mm out. Use it for a convex prop that is gripped: an SDF body pinched by the finger pads can stick to them.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import hashlib
import importlib.abc
import importlib.util
import json
import math
import os
import sys
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from yam_arena.task_file import Box

__all__ = ["REPO", "REVISION", "SimReady", "fetch"]

REPO = "nvidia/simready-assets"  # Hugging Face dataset
REVISION = "9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd"  # pinned commit (2026-11-02)
HF = "https://huggingface.co"
CACHE = Path(os.environ.get("SIMREADY_CACHE", "~/.cache/simready")).expanduser()


@dataclass(frozen=True)
class SimReady(Box):
    """A SimReady prop: ``package`` is its folder in the dataset (e.g. ``"power_battery_9v_a01"``) and ``usd`` its
    root file in ``simready_usd/``. ``pos`` places the file's origin (for these props, the centre of the bottom),
    ``yaw`` turns it about the vertical, or ``quat_xyzw`` sets its full orientation (for a prop that doesn't stand
    the way its file lies, like a hanging tool). ``scale`` scales it uniformly. ``size`` is only nominal.
    ``convex`` collides its meshes as convex hulls. ``angular_damping`` damps the body's spin."""

    package: str = ""
    usd: str = ""
    scale: float = 1.0
    convex: bool = False
    quat_xyzw: tuple[float, float, float, float] | None = None
    angular_damping: float | None = None  # 1/s: slows spin a rough contact would stop (PhysX has no rolling friction)
    # Parts of meshes to remove: (mesh path under the prop's root, its point count, ((first, end), ...) point index
    # ranges). Every face that uses one of those points is deleted.
    strip: tuple = ()

    def _check(self) -> None:
        super()._check()
        if not self.package or "/" in self.package or not self.usd.endswith((".usd", ".usda", ".usdc")):
            raise ValueError(f"{self.name}: give the SimReady package folder and its root .usd file")
        if not self.scale > 0:
            raise ValueError(f"{self.name}.scale must be positive")
        if self.quat_xyzw is not None and abs(math.fsum(v * v for v in self.quat_xyzw) - 1.0) > 1e-6:
            raise ValueError(f"{self.name}.quat_xyzw must be a unit quaternion")


# -- download --------------------------------------------------------------------------------------------------------


def _get(url: str) -> bytes:
    headers = {"User-Agent": "robocurve-benchmark-task"}
    if os.environ.get("HF_TOKEN"):
        headers["Authorization"] = f"Bearer {os.environ['HF_TOKEN']}"
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=120) as response:
        return response.read()


def _wanted(path: str, package: str) -> bool:
    """The files a prop needs to load and render: its USD layers, textures and MDL, and its licence. Thumbnails and
    the copies of the textures in ``source_assets`` are left out."""
    if path == f"{package}/.metadata/LICENSE.txt":
        return True
    if not path.startswith(f"{package}/simready_usd/") or "/.thumbs/" in path:
        return False
    return not (path.startswith(f"{package}/simready_usd/source_assets/") and path.endswith(".png"))


def fetch(package: str, usd: str) -> Path:
    """The local path of ``package``'s root USD, downloading the package at :data:`REVISION` if needed."""
    root = CACHE / REPO / REVISION
    target = root / package / "simready_usd" / usd
    done = root / package / ".complete"
    if done.is_file() and target.is_file():
        return target
    listing = json.loads(_get(f"{HF}/api/datasets/{REPO}/tree/{REVISION}/{package}?recursive=true"))
    files = [e for e in listing if e.get("type") == "file" and _wanted(e["path"], package)]
    if not files:
        raise FileNotFoundError(f"{REPO}@{REVISION[:8]} has no package {package!r}")
    for entry in files:
        dest = root / entry["path"]
        if dest.is_file() and dest.stat().st_size == entry.get("size"):
            continue
        data = _get(f"{HF}/datasets/{REPO}/resolve/{REVISION}/{entry['path']}")
        sha = (entry.get("lfs") or {}).get("oid")
        if sha and hashlib.sha256(data).hexdigest() != sha:
            raise OSError(f"checksum mismatch for {entry['path']}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = dest.with_suffix(dest.suffix + ".part")
        tmp.write_bytes(data)
        tmp.replace(dest)
    if not target.is_file():
        raise FileNotFoundError(f"{package}: no {usd} in simready_usd/")
    done.write_text(REVISION)
    return target


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, SimReady] = {}
_SPAWN: dict = {}


def _prepare(prim, spec: SimReady) -> None:
    """Deactivate the guide curves and set up the prop's one rigid body (kinematic, or with the spec's mass)."""
    from pxr import Usd, UsdPhysics

    # Instance proxies (the instanced meshes) are not traversed. Collect first: deactivating expires descendants.
    for guide in [p for p in Usd.PrimRange(prim) if p.GetName() == "Grasp"]:
        guide.SetActive(False)
    for part in spec.strip:
        _strip(prim, *part)
    if spec.convex:
        _convex_colliders(prim)
    bodies = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies) > 1:
        bodies = [_merge_bodies(prim, bodies)]
    if len(bodies) != 1:
        raise ValueError(f"{spec.name}: expected one rigid body in {spec.usd}, found {len(bodies)}")
    body = bodies[0]
    if spec.mass is None:
        UsdPhysics.RigidBodyAPI(body).CreateKinematicEnabledAttr(True)
        ccd = body.GetAttribute("physxRigidBody:enableCCD")
        if ccd:
            ccd.Set(False)  # PhysX ignores CCD on kinematic bodies, with an error
        return
    if spec.angular_damping is not None:
        from pxr import Sdf

        if "PhysxRigidBodyAPI" not in body.GetAppliedSchemas():
            body.AddAppliedSchema("PhysxRigidBodyAPI")
        body.CreateAttribute("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float).Set(float(spec.angular_damping))
    mass = UsdPhysics.MassAPI.Apply(body)
    authored = mass.GetMassAttr().Get()
    mass.CreateMassAttr(float(spec.mass))
    inertia = mass.GetDiagonalInertiaAttr().Get()
    if authored and inertia is not None:
        ratio = float(spec.mass) / float(authored) * spec.scale**2
        mass.GetDiagonalInertiaAttr().Set(type(inertia)(*(float(v) * ratio for v in inertia)))


def _strip(prim, rel: str, n_points: int, ranges) -> None:
    """Delete the faces of the mesh ``rel`` (a path under the prop's root) that use a point in ``ranges``. The
    mesh's face-varying and uniform data and its face subsets are cut to match, the unused points are moved onto a
    kept one, and its extent is recomputed, so the removed part is neither drawn, nor collides, nor counts in the
    prop's bounds. The mesh is made local first (the props' meshes are instanced, which is read-only)."""
    import numpy as np
    from pxr import Gf, UsdGeom, Vt

    path = prim.GetPath()
    for name in rel.split("/"):
        path = path.AppendChild(name)
        p = prim.GetStage().GetPrimAtPath(path)
        if p.IsInstance():
            p.SetInstanceable(False)
    mesh = UsdGeom.Mesh(prim.GetStage().GetPrimAtPath(path))
    if not mesh:
        raise ValueError(f"{prim.GetPath()}: no mesh {rel} to strip")
    points = np.array(mesh.GetPointsAttr().Get(), dtype=np.float32)
    if len(points) != n_points:
        raise ValueError(f"{rel} has {len(points)} points, not {n_points}: another revision of the prop?")
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    gone = np.zeros(len(points), bool)
    for a, b in ranges:
        gone[a:b] = True
    face_of = np.repeat(np.arange(len(counts)), counts)  # each face-vertex's face
    drop = np.zeros(len(counts), bool)
    np.logical_or.at(drop, face_of, gone[indices])
    keep_fv = ~drop[face_of]
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(counts[~drop].astype(np.int32)))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(indices[keep_fv].astype(np.int32)))
    kept_point = points[indices[keep_fv][0]]
    points[gone] = kept_point
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points))
    used = points[np.unique(indices[keep_fv])]
    mesh.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, used.min(0))), Gf.Vec3f(*map(float, used.max(0)))]))

    normals = mesh.GetNormalsAttr()
    if normals.HasValue() and mesh.GetNormalsInterpolation() == UsdGeom.Tokens.faceVarying:
        normals.Set(Vt.Vec3fArray.FromNumpy(np.asarray(normals.Get(), dtype=np.float32)[keep_fv]))
    for pv in UsdGeom.PrimvarsAPI(mesh).GetPrimvars():
        mask = {UsdGeom.Tokens.faceVarying: keep_fv, UsdGeom.Tokens.uniform: ~drop}.get(pv.GetInterpolation())
        if mask is None or not pv.HasValue():
            continue
        if pv.IsIndexed():
            pv.SetIndices(Vt.IntArray.FromNumpy(np.asarray(pv.GetIndices())[mask].astype(np.int32)))
        else:
            values = pv.Get()
            pv.Set(type(values).FromNumpy(np.ascontiguousarray(np.asarray(values)[mask])))
    new_index = np.cumsum(~drop) - 1
    for subset in UsdGeom.Subset.GetAllGeomSubsets(mesh):
        faces = np.asarray(subset.GetIndicesAttr().Get(), dtype=np.int64)
        faces = faces[~drop[faces]]
        subset.GetIndicesAttr().Set(Vt.IntArray.FromNumpy(new_index[faces].astype(np.int32)))


def _merge_bodies(prim, bodies):
    """Make a prop with several jointed bodies one rigid body: deactivate its joints, remove the bodies' rigid-body
    and mass APIs, and make their nearest common transformable parent the body. Returns the new body."""
    from pxr import Usd, UsdGeom, UsdPhysics

    for joint in [p for p in Usd.PrimRange(prim) if p.IsA(UsdPhysics.Joint)]:
        joint.SetActive(False)
    # Removed by name: PhysxSchema's Python module only loads inside Isaac Sim.
    for p in [prim, *Usd.PrimRange(prim)]:
        for api in ("PhysicsArticulationRootAPI", "PhysxArticulationAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    for body in bodies:
        for api in ("PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"):
            if api in body.GetAppliedSchemas():
                body.RemoveAppliedSchema(api)
    common = bodies[0].GetPath()
    for body in bodies[1:]:
        common = common.GetCommonPrefix(body.GetPath())
    if common in [b.GetPath() for b in bodies]:
        common = common.GetParentPath()
    merged = prim.GetStage().GetPrimAtPath(common)
    while not merged.IsA(UsdGeom.Xformable) and merged != prim:  # a Scope can't be a body
        merged = merged.GetParent()
    UsdPhysics.RigidBodyAPI.Apply(merged)
    return merged


def _convex_colliders(prim) -> None:
    """Collide every mesh as its convex hull, contacts from 5 mm out. The meshes are instanced (read-only), so the
    instances are made local first."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    while instances := [p for p in Usd.PrimRange(prim) if p.IsInstance()]:
        for p in instances:
            p.SetInstanceable(False)
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI(p).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
            contact = PhysxSchema.PhysxCollisionAPI.Apply(p)
            contact.CreateContactOffsetAttr(0.005)
            contact.CreateRestOffsetAttr(0.0)


def _spawn_raw(prim_path: str, cfg, *args, **kwargs):
    from isaaclab.sim.spawners.from_files import spawn_from_usd

    prim = spawn_from_usd.__wrapped__(prim_path, cfg, *args, **kwargs)
    _prepare(prim, _SPECS[prim_path.rstrip("/").rsplit("/", 1)[-1]])
    return prim


def _spawner():
    if "fn" not in _SPAWN:
        from isaaclab.sim.utils import clone

        _SPAWN["fn"] = clone(_spawn_raw)
    return _SPAWN["fn"]


def _arena_object(spec: SimReady):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    usd = fetch(spec.package, spec.usd)
    _SPECS[spec.name] = spec
    half = spec.yaw / 2
    rotation = spec.quat_xyzw or (0.0, 0.0, math.sin(half), math.cos(half))
    return Object(
        name=spec.name,
        usd_path=str(usd),
        object_type=ObjectType.RIGID,
        scale=(float(spec.scale),) * 3,
        initial_pose=Pose(
            position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(float(v) for v in rotation)
        ),
        spawn_cfg_addon={"func": _spawner()},
    )


# -- hooking yam_arena -----------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_simready", False):
        return
    task_object = scene.task_object

    def task_object_hook(spec):
        return _arena_object(spec) if isinstance(spec, SimReady) else task_object(spec)

    scene.task_object = task_object_hook
    scene._simready = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches yam_arena.scene as it is imported (it needs Isaac Sim, which starts after the task file is loaded)."""

    targets = {"yam_arena.scene": _patch_scene}

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
        exec_module = loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            patch(module)

        loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Hook yam_arena now if it is already imported, and on import otherwise. Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
