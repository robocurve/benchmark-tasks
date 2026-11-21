"""rskit: props from USD files and NVIDIA SimReady packages in a yam_arena task file, plus reset hooks.

A yam_arena task file can declare boxes, cylinders, spheres and Arena library assets. This module adds:

- :class:`UsdProp`: a body from a USD file in this folder. Without a ``mass`` it is
  kinematic (it stays where it is placed); with one it is a free rigid body.
- :class:`SimReadyProp`: a prop from NVIDIA's SimReady library on Hugging Face (``nvidia/simready-assets``),
  downloaded once at a pinned revision into ``RSKIT_CACHE`` (default ``~/.cache/rskit``). ``layer`` is optional USD
  text written next to the package's root file and loaded in its place (to add parts to the prop, say).
  A package made of several bodies on joints is made one rigid body: its joints are
  switched off, and the bodies' authored masses, centres of mass and inertias are combined at the prop's root,
  then scaled to the declared ``mass``. ``convex=True`` collides every mesh as one convex hull, ``convex="decomposition"`` as a convex decomposition.
- ``articulated=True`` (on either class): the file's joints are kept and the prop is an articulation (its file must
  carry an ``ArticulationRootAPI``); :func:`joint_positions` reads its joints during a trial.
- :func:`robot_points`: where the arms' links are during a trial (for no-contact rules); :func:`pad_points`: points
  on the finger pads (for let-go rules).
- :func:`on_reset`: registers ``fn(env)`` to run after every reset; :func:`env` returns the running environment.

Both prop classes subclass ``yam_arena.task_file.Box`` so the task-file checks accept them; ``size`` is only a
nominal box. Importing this module changes nothing on disk: it wraps ``yam_arena.scene.task_object`` and
``YamArenaEmbodiment.reset`` when those load. Everything that needs Isaac Sim is imported inside functions.
"""

from __future__ import annotations

import hashlib
import importlib.abc
import importlib.util
import json
import os
import sys
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from yam_arena.task_file import Box

__all__ = ["SIMREADY_REVISION", "SimReadyProp", "UsdProp", "env", "joint_positions", "on_reset", "pad_points", "robot_points", "simready_file"]

FOLDER = Path(__file__).resolve().parent
SIMREADY_REPO = "nvidia/simready-assets"
SIMREADY_REVISION = "99bb6c7c1247ff3a1f63e484996336e9e35b9cf8"  # dataset commit, 2026-11-16
CACHE = Path(os.environ.get("RSKIT_CACHE", "~/.cache/rskit")).expanduser()


@dataclass(frozen=True)
class UsdProp(Box):
    """A body from ``usd`` (a path relative to this folder), its file origin placed at ``pos`` turned by
    ``rot_xyzw``. ``mass=None``: kinematic. ``convex``: every mesh collides as one convex hull."""

    usd: str = ""
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    scale: float = 1.0
    convex: bool | str = False   # True: each mesh one convex hull; "decomposition": convex decomposition
    articulated: bool = False
    friction: tuple[float, float] | None = None   # (static, dynamic); None: the file's own materials if it has
    # any (SimReady props do), else yam_arena's rubber-like grip (1.2, 1.0)

    def _check(self) -> None:
        super()._check()
        if not self.scale > 0:
            raise ValueError(f"{self.name}.scale must be positive")

    def usd_file(self) -> Path:
        path = FOLDER / self.usd
        if not path.is_file():
            raise FileNotFoundError(f"{self.name}: no file {path}")
        return path


@dataclass(frozen=True)
class SimReadyProp(UsdProp):
    """A SimReady prop: ``package`` is its folder in the dataset and ``usd`` its root file in ``simready_usd/``.
    ``layer`` (USD text) and ``layer_name`` replace the root file with a layer written beside it."""

    package: str = ""
    layer: str = ""
    layer_name: str = ""

    def _check(self) -> None:
        super()._check()
        if not self.package or not self.usd.endswith((".usd", ".usda", ".usdc")):
            raise ValueError(f"{self.name}: give the SimReady package and its root USD file")
        if bool(self.layer) != bool(self.layer_name):
            raise ValueError(f"{self.name}: give both layer and layer_name, or neither")

    def usd_file(self) -> Path:
        root = simready_file(self.package, self.usd)
        if not self.layer:
            return root
        path = root.parent / self.layer_name
        if not path.is_file() or path.read_text() != self.layer:
            path.write_text(self.layer)
        return path


# -- SimReady download ---------------------------------------------------------------------------------------------


def _fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "rskit"})
    if os.environ.get("HF_TOKEN"):
        request.add_header("Authorization", f"Bearer {os.environ['HF_TOKEN']}")
    with urllib.request.urlopen(request, timeout=120) as reply:
        return reply.read()


def simready_file(package: str, usd: str) -> Path:
    """The local path of ``package``'s ``simready_usd/<usd>``, downloading the package's USD folder if needed."""
    folder = CACHE / SIMREADY_REVISION / package
    target, stamp = folder / "simready_usd" / usd, folder / ".done"
    if stamp.is_file() and target.is_file():
        return target
    base = "https://huggingface.co"
    entries = json.loads(_fetch(f"{base}/api/datasets/{SIMREADY_REPO}/tree/{SIMREADY_REVISION}/{package}?recursive=true"))
    wanted = [e for e in entries if e.get("type") == "file" and e["path"].startswith(f"{package}/simready_usd/")]
    if not wanted:
        raise FileNotFoundError(f"{SIMREADY_REPO}@{SIMREADY_REVISION[:8]}: no package {package!r}")
    for entry in wanted:
        dest = CACHE / SIMREADY_REVISION / entry["path"]
        if dest.is_file() and dest.stat().st_size == entry.get("size"):
            continue
        data = _fetch(f"{base}/datasets/{SIMREADY_REPO}/resolve/{SIMREADY_REVISION}/{entry['path']}")
        digest = (entry.get("lfs") or {}).get("oid")
        if digest and hashlib.sha256(data).hexdigest() != digest:
            raise OSError(f"checksum mismatch: {entry['path']}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        partial = dest.with_name(dest.name + ".part")
        partial.write_bytes(data)
        partial.replace(dest)
    if not target.is_file():
        raise FileNotFoundError(f"{package}: no simready_usd/{usd}")
    stamp.write_text(SIMREADY_REVISION)
    return target


# -- spawning ------------------------------------------------------------------------------------------------------

_PROPS: dict[str, UsdProp] = {}


def _one_body(root, spec: UsdProp) -> None:
    """Make the spawned prop one rigid body at ``root``: guide prims and joints off, bodies merged, mass set."""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    while True:                                            # SimReady meshes are instanced: un-instance them so the
        shared = [p for p in Usd.PrimRange(root) if p.IsInstance()]   # bodies and colliders below can be edited
        if not shared:
            break
        for prim in shared:
            prim.SetInstanceable(False)
    for prim in [p for p in Usd.PrimRange(root) if p.GetName() == "Grasp"]:
        prim.SetActive(False)                              # invisible grasp annotations: keep them out of the bounds
    _set_friction(root, spec)
    for prim in [p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]:
        prim.SetActive(False)
    cache = UsdGeom.XformCache()
    to_root = cache.GetLocalToWorldTransform(root).GetInverse()
    masses, centres, inertias = [], [], []
    for prim in [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.MassAPI)]:   # on bodies or on colliders
        api = UsdPhysics.MassAPI(prim)
        m = api.GetMassAttr().Get() or 0.0
        c = api.GetCenterOfMassAttr().Get()
        i = api.GetDiagonalInertiaAttr().Get()
        if m > 0:
            known = c is not None and all(abs(v) < 1e6 for v in c)
            place = cache.GetLocalToWorldTransform(prim) * to_root
            masses.append(m)
            centres.append(np.array(place.Transform(Gf.Vec3d(*(c if known else (0.0, 0.0, 0.0))))))
            inertias.append(np.array(i if i is not None else (0.0, 0.0, 0.0)))
        prim.RemoveAPI(UsdPhysics.MassAPI)
    for body in [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI) and p != root]:
        body.RemoveAPI(UsdPhysics.RigidBodyAPI)
    UsdPhysics.RigidBodyAPI.Apply(root)
    shape = (UsdPhysics.Tokens.convexDecomposition if spec.convex == "decomposition"
             else UsdPhysics.Tokens.convexHull)
    for part in Usd.PrimRange(root):
        if spec.convex and part.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI(part).GetApproximationAttr().Set(shape)
        elif part.IsA(UsdGeom.Mesh) and part.HasAPI(UsdPhysics.CollisionAPI) and not part.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(part).CreateApproximationAttr(shape)
    if spec.mass is None:
        return
    api = UsdPhysics.MassAPI.Apply(root)
    api.CreateMassAttr(float(spec.mass))
    if masses:
        total = float(sum(masses))
        centre = sum(m * c for m, c in zip(masses, centres)) / total
        api.CreateCenterOfMassAttr(Gf.Vec3f(*centre.tolist()))
        diag = sum(inertias) * (float(spec.mass) / total)  # the authored inertia, scaled to the declared mass
        if np.all(diag > 0):
            api.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag.tolist()))


def _set_friction(root, spec: UsdProp) -> None:
    """SimReady files bind their own physics materials on the meshes, which override a material bound at the root:
    with ``friction`` declared, those materials take it."""
    if spec.friction is None:
        return
    from pxr import Usd, UsdPhysics

    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.MaterialAPI):
            api = UsdPhysics.MaterialAPI(prim)
            api.CreateStaticFrictionAttr().Set(float(spec.friction[0]))
            api.CreateDynamicFrictionAttr().Set(float(spec.friction[1]))


def _keep_joints(root, spec: UsdProp) -> None:
    """An articulated prop: un-instance it and switch its meshes to convex hulls if asked; joints stay as authored."""
    from pxr import Usd, UsdPhysics

    while True:
        shared = [p for p in Usd.PrimRange(root) if p.IsInstance()]
        if not shared:
            break
        for prim in shared:
            prim.SetInstanceable(False)
    for prim in [p for p in Usd.PrimRange(root) if p.GetName() == "Grasp"]:
        prim.SetActive(False)
    _set_friction(root, spec)
    if spec.convex:
        for part in Usd.PrimRange(root):
            if part.HasAPI(UsdPhysics.MeshCollisionAPI):
                UsdPhysics.MeshCollisionAPI(part).GetApproximationAttr().Set(UsdPhysics.Tokens.convexHull)


def joint_positions(name: str) -> dict:
    """The named articulated prop's joint positions (rad or m) in env 0, by joint name; {} outside a trial."""
    live = env()
    if live is None:
        return {}
    art = live.unwrapped.scene[name]
    values = _torch(art.data.joint_pos)[0].detach().cpu().numpy()
    return {n: float(v) for n, v in zip(art.joint_names, values)}


def _torch(value):
    """Isaac Lab 3 data fields are warp arrays with a ``.torch`` view; older ones are torch tensors."""
    return getattr(value, "torch", value)


_ARTICULATED: dict = {}


def _articulated_states(env, names) -> dict:
    """ObjectStates for articulated props, which Arena's geometry queries don't cover: the root link's pose, with the
    centroid and bounds of the prop's nominal ``size`` box (its root body; joints' moving parts aren't included).
    The box's origin is its bottom centre, as for the root body in the prop's file. Velocities are reported as 0."""
    import numpy as np
    from yam_arena import task_file

    scene = env.unwrapped.scene
    origin = _torch(scene.env_origins)[0].detach().cpu().numpy().astype(np.float64)
    out = {}
    for name in names:
        spec = _ARTICULATED[name]
        pose = _torch(scene[name].data.root_pose_w)[0].detach().cpu().numpy().astype(np.float64)
        pos, (qx, qy, qz, qw) = pose[:3] - origin, pose[3:]
        rot = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        half = np.asarray(spec.size, float) / 2 * spec.scale
        centre = pos + rot @ np.array([0.0, 0.0, half[2]])
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
        world = corners @ rot.T + centre
        out[name] = task_file.ObjectState(pos=centre, quat_xyzw=pose[3:].copy(), lin_vel=np.zeros(3),
                                          ang_vel=np.zeros(3), low=world.min(0), high=world.max(0))
    return out


def robot_points() -> dict:
    """Both arms' link origins in env 0's frame (m), as {"left": {link: xyz}, "right": {...}}; {} outside a trial.
    Fingertips are the links ``tip_left`` and ``tip_right``."""
    live = env()
    if live is None:
        return {}
    scene = live.unwrapped.scene
    origin = _torch(scene.env_origins)[0]
    out = {}
    for arm in ("left", "right"):
        robot = scene[f"{arm}_robot"]
        pose = robot.data.body_link_pose_w if hasattr(robot.data, "body_link_pose_w") else robot.data.body_pos_w
        pos = (_torch(pose)[0, :, :3] - origin).detach().cpu().numpy()
        out[arm] = {n: pos[i] for i, n in enumerate(robot.body_names)}
    return out


def _quat_matrix(qx, qy, qz, qw):
    import numpy as np

    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def pad_points() -> list:
    """Points on both arms' finger pads in env 0's frame (m); [] outside a trial.

    The links ``tip_left``/``tip_right`` sit at the fingers' roots, 5.7 cm along the gripper's axis; the pads run
    from about 12 cm to the pad tips at 14.7 cm (yam_arena's TCP is at 13.4 cm, the tips 13 mm past it). Each
    finger's frame is carried along the gripper's axis to 12, 13.4 and 14.7 cm."""
    import numpy as np
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    live = env()
    if live is None:
        return []
    scene = live.unwrapped.scene
    origin = _torch(scene.env_origins)[0].detach().cpu().numpy().astype(np.float64)
    tcp = float(TCP_OFFSET[2])
    out = []
    for arm in ("left", "right"):
        robot = scene[f"{arm}_robot"]
        poses = _torch(robot.data.body_link_pose_w)[0].detach().cpu().numpy().astype(np.float64)
        names = list(robot.body_names)
        g = poses[names.index("gripper")]
        rot, at = _quat_matrix(*g[3:]), g[:3] - origin
        for tip in ("tip_left", "tip_right"):
            local = rot.T @ (poses[names.index(tip)][:3] - origin - at)
            for z in (tcp - 0.014, tcp, tcp + PAD_TIP_PAST_TCP):
                out.append(at + rot @ np.array([local[0], local[1], z]))
    return out


def _arena_object(spec: UsdProp):
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg
    from yam_arena import scene as ys

    @clone
    def spawn(prim_path, cfg, *args, **kwargs):
        prim = spawn_from_usd.__wrapped__(prim_path, cfg, *args, **kwargs)
        if spec.articulated:
            _keep_joints(prim, spec)
        else:
            _one_body(prim, spec)
        ccd = prim.GetAttribute("physxRigidBody:enableCCD")
        if spec.mass is None and ccd:
            ccd.Set(False)                                 # PhysX rejects CCD on kinematic bodies
        return prim

    material = ys._grippy()
    if spec.friction is not None:
        import isaaclab.sim as sim_utils

        material = sim_utils.RigidBodyMaterialCfg(static_friction=float(spec.friction[0]),
                                                  dynamic_friction=float(spec.friction[1]))
    addon = {"func": spawn, "physics_material": material}
    if spec.mass is None and not spec.articulated:
        addon["rigid_props"] = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
    else:
        addon.update(rigid_props=PhysxRigidBodyPropertiesCfg(), collision_props=ys._contact_props())
    return Object(
        name=spec.name,
        usd_path=str(spec.usd_file()),
        object_type=ObjectType.ARTICULATION if spec.articulated else ObjectType.RIGID,
        scale=(float(spec.scale),) * 3,
        initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(spec.rot_xyzw)),
        spawn_cfg_addon=addon,
    )


# -- hooks ---------------------------------------------------------------------------------------------------------

_RESET: list = []
_LIVE: dict = {"embodiment": None}


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset of the environment (decorator)."""
    _RESET.append(fn)
    return fn


def env():
    """The running Isaac Lab environment (or None before the first reset)."""
    body = _LIVE["embodiment"]
    return None if body is None else getattr(body, "_env", None)


def _wrap_scene(module) -> None:
    if getattr(module, "_rskit", False):
        return
    plain = module.task_object
    plain_states = module.object_states

    def task_object(spec):
        if isinstance(spec, UsdProp) and spec.articulated:
            _ARTICULATED[spec.name] = spec
        return _arena_object(spec) if isinstance(spec, UsdProp) else plain(spec)

    def object_states(env, names):
        names = list(names)
        out = plain_states(env, [n for n in names if n not in _ARTICULATED])
        out.update(_articulated_states(env, [n for n in names if n in _ARTICULATED]))
        return {n: out[n] for n in names}

    module.task_object, module.object_states, module._rskit = task_object, object_states, True


def _wrap_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_rskit", False):
        return
    plain = cls.reset

    def reset(self, *args, **kwargs):
        observation = plain(self, *args, **kwargs)
        _LIVE["embodiment"] = self
        for fn in _RESET:
            fn(self._env)
        return observation

    cls.reset, cls._rskit = reset, True


_WRAPPERS = {"yam_arena.scene": _wrap_scene, "yam_arena.inspect_embodiment": _wrap_embodiment}


class _AfterImport(importlib.abc.MetaPathFinder):
    """Apply a wrapper to a module right after it is first imported."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _WRAPPERS:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        inner, wrap = spec.loader, _WRAPPERS[fullname]

        class Loader(importlib.abc.Loader):
            def create_module(self, s):
                return inner.create_module(s)

            def exec_module(self, module):
                inner.exec_module(module)
                wrap(module)

        spec.loader = Loader()
        return spec


def _install() -> None:
    for name, wrap in _WRAPPERS.items():
        if name in sys.modules:
            wrap(sys.modules[name])
    if not any(isinstance(f, _AfterImport) for f in sys.meta_path):
        sys.meta_path.insert(0, _AfterImport())


_install()
