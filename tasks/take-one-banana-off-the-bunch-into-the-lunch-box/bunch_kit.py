"""A hand of fruit whose stems snap: separate rigid bodies (a crown and its fruit) joined by native PhysX fixed joints
that break for good past a set torque or force. Reusable for any fruit hand or cluster (bananas, a truss of
tomatoes, grapes on a stalk) and any part that must be snapped off a base.

What it models. Each body is a :class:`BunchPart`: a free compound body of convex mesh pieces (the kit's MeshPart),
spawned with 32 solver position iterations, 1 m/s maximum depenetration speed and a matte look. Each stem is a
:class:`Stem`: a USD ``PhysicsFixedJoint`` between the crown (body 0) and one fruit (body 1) at the stem's root, with
``breakTorque`` and ``breakForce``, collision between the pair off and the joint kept out of any articulation.
PhysX alone decides when a stem lets go (the bending moment or pull at the joint exceeds its limit); nothing in the
task ever moves, welds or releases a body while an episode runs.

Resets. The bunch's bodies are shifted together, never one by one: the harness's per-object jitter skips them and
this module shifts the whole hand by one random x/y offset of the same size (seeded resets only). A stem that broke
in the last episode (seen as a native JOINT_BREAK event or by pose) is switched off and a fresh joint prim is authored
beside it after the bodies are back in place, so every episode starts with the whole hand joined.

Fruit shape. :func:`fruit_meshes` lofts a curved fruit: stations (distance along the centre line, diameter) on a
circular arc of radius ``bend_r`` in the fruit's y-z plane, concave up, lowest at ``s_low``; the fruit leaves its
stem root (its frame's origin) along -y; ``roll`` turns the curve's plane about the fruit's heading (a hand lying
partly on its side shows its curve from above). Cross-sections are rounded polygons with shallow ridges (a banana's
angles). Every piece is one convex hull; its offline stand-in is a box of the piece's true section.

Tuned numbers used by the banana task (estimates, not measurements): break torque 0.6 N m and break force 80 N at
the stem root; fruit friction 0.7/0.6; contact offset 1.5 mm, rest offset 0.
"""
from __future__ import annotations

import functools
import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass

import numpy as np
import yam_parts as parts
import yam_mesh as ymesh
from yam_mesh import Mesh, MeshPart

TAU = 2 * math.pi
DETACH_MM, DETACH_DEG = 3.0, 3.0  # a fruit this far off its joined pose has come off its stem


# ---------------------------------------------------------------- geometry
def _closed_loft(name, rings, color, proxy):
    """A closed mesh through rings of equal point count (same order), with flat end caps, outward faces."""
    rings = [np.asarray(r, float) for r in rings]
    k = len(rings[0])
    points = np.concatenate(rings)
    faces = []
    for i in range(len(rings) - 1):
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k
            c, d = a + k, b + k
            faces += [(a, b, d), (a, d, c)]
    faces += [(0, j + 1, j) for j in range(1, k - 1)]
    last = (len(rings) - 1) * k
    faces += [(last, last + j, last + j + 1) for j in range(1, k - 1)]
    faces = np.asarray(faces, int)
    tri = (points - points.mean(axis=0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, points, faces, collision="convex", color=tuple(color), proxy=tuple(proxy))


def section(n=14, ridges=5, depth=0.035):
    """Unit cross-section: a circle with `ridges` shallow flats (relative depth `depth`), as (cos, sin) * radius."""
    a = np.linspace(0.0, TAU, n, endpoint=False) + TAU / (2 * n)
    r = 1.0 - depth * (0.5 - 0.5 * np.cos(ridges * a))
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def arc_frame(s, bend_r, s_low):
    """Centre point, tangent and in-plane normal at distance s along the fruit (frame: origin at the stem root,
    the fruit heading -y, curving up in the y-z plane, lowest at s_low)."""
    p, p0 = (s - s_low) / bend_r, -s_low / bend_r
    centre = np.array([0.0, -(bend_r * math.sin(p) - bend_r * math.sin(p0)), bend_r * (math.cos(p0) - math.cos(p))])
    tangent = np.array([0.0, -math.cos(p), math.sin(p)])
    normal = np.array([0.0, math.sin(p), math.cos(p)])  # in the y-z plane, pointing up-ish
    return centre, tangent, normal


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rolled_frame(s, bend_r, s_low, roll=0.0):
    """arc_frame with the curve's plane rolled `roll` (rad) about the fruit's heading (its y axis)."""
    R = rot_y(roll)
    return tuple(R @ v for v in arc_frame(s, bend_r, s_low))


def fruit_ring(s, diameter, bend_r, s_low, unit, roll=0.0):
    c, _, n = arc_frame(s, bend_r, s_low)
    lat = np.array([1.0, 0.0, 0.0])
    r = diameter / 2
    return np.array([c + r * (u * lat + v * n) for u, v in unit]) @ rot_y(roll).T


def fruit_meshes(stations, bend_r, s_low, colors, grip_s=None, grip_len=0.036, name="fruit", roll=0.0):
    """Convex pieces between consecutive stations [(s, diameter)], coloured colors[i]; returns (meshes, grip index
    among the proxies, or None). Each proxy is a box of the piece's section on its centre line (85 % of its width,
    so neighbours in a hand do not overlap on paper; the stem's starts 7 mm off the crown); the piece that
    holds grip_s gets one more proxy, the pinched section, which the offline checker uses as the grip collider."""
    unit = section()
    meshes, grip, count = [], None, 0
    for i, ((s0, d0), (s1, d1)) in enumerate(zip(stations, stations[1:])):
        r0, r1 = fruit_ring(s0, d0, bend_r, s_low, unit, roll), fruit_ring(s1, d1, bend_r, s_low, unit, roll)
        c0, c1 = rolled_frame(s0, bend_r, s_low, roll)[0], rolled_frame(s1, bend_r, s_low, roll)[0]
        mid, dm = (c0 + c1) / 2, max(d0, d1)
        length = float(np.linalg.norm(c1 - c0))
        if i == 0:  # the stem: its stand-in starts 7 mm out from the crown it is joined to
            c0 = c0 + (c1 - c0) * min(0.007 / max(length, 1e-9), 0.9)
            mid, length = (c0 + c1) / 2, float(np.linalg.norm(c1 - c0))
        proxy = [parts.box(tuple(mid), (0.85 * dm, max(length, 0.002), dm))]
        if grip_s is not None and s0 <= grip_s < s1:
            d = float(np.interp(grip_s, [s0, s1], [d0, d1]))
            proxy.append(parts.box(tuple(rolled_frame(grip_s, bend_r, s_low, roll)[0]), (d, grip_len, d)))
            grip = count + 1
        count += len(proxy)
        meshes.append(_closed_loft(f"{name}_{i}", [r0, r1], colors[i], proxy))
    return meshes, grip


def fruit_axis(stations, bend_r, s_low, roll=0.0):
    """The fruit's stem root and tip on its centre line (own frame)."""
    return rolled_frame(stations[0][0], bend_r, s_low, roll)[0], rolled_frame(stations[-1][0], bend_r, s_low, roll)[0]


def fruit_points(stations, bend_r, s_low, n=10, roll=0.0):
    """Surface sample of a fruit in its own frame (scoring: where the fruit lies)."""
    unit = section(n)
    return np.concatenate([fruit_ring(s, d, bend_r, s_low, unit, roll) for s, d in stations])


def rounded_block(name, size, radius, color, z0=0.0, n=5):
    """A box with rounded vertical edges (x, y, z size; base at z0), as one convex mesh."""
    sx, sy, sz = size
    hx, hy = sx / 2 - radius, sy / 2 - radius
    ring = []
    for cx, cy, a0 in ((hx, hy, 0.0), (-hx, hy, 90.0), (-hx, -hy, 180.0), (hx, -hy, 270.0)):
        for k in range(n):
            a = math.radians(a0 + 90.0 * k / (n - 1))
            ring.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    ring = np.asarray(ring)
    lo = np.c_[ring, np.full(len(ring), z0)]
    hi = np.c_[ring, np.full(len(ring), z0 + sz)]
    proxy = parts.box((0.0, 0.0, z0 + sz / 2), (sx, sy, sz))
    return _closed_loft(name, [lo, hi], color, [proxy])


# ---------------------------------------------------------------- bodies and stems
class BunchPart(MeshPart):
    """One body of a hand of fruit (the crown or one fruit). It is free (it falls, can be grasped and pushed); at a
    seeded reset the hand's bodies move together (this module), not one by one (the harness)."""


@dataclass(frozen=True)
class Stem:
    """A breakable stem: `fruit`'s frame origin is the stem root, fixed to `crown` until the bending moment there
    exceeds `break_torque` (N m) or the pull `break_force` (N). The fruit starts turned `yaw` (rad) about z."""

    name: str
    crown: str
    fruit: str
    anchor: tuple[float, float, float]  # world, at the start (the fruit's pos)
    crown_pos: tuple[float, float, float]
    yaw: float
    break_torque: float
    break_force: float

    def joined_pose(self, crown_origin, crown_rot):
        """Where the fruit's frame is while the stem holds (origin, rotation), from the crown's pose."""
        local = np.asarray(self.anchor, float) - np.asarray(self.crown_pos, float)
        return np.asarray(crown_origin, float) + crown_rot @ local, crown_rot @ rot_z(self.yaw)


STEMS: list[Stem] = []


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def stem(name, crown, fruit, break_torque, break_force):
    """Register the stem joining `fruit` (a BunchPart whose pos is its stem root) to `crown` (unturned)."""
    s = Stem(name, crown.name, fruit.name, tuple(fruit.pos), tuple(crown.pos), float(fruit.yaw), float(break_torque),
             float(break_force))
    STEMS[:] = [t for t in STEMS if t.name != name] + [s]
    return s


def stem_error(stem_spec, crown_state, fruit_state):
    """(mm, deg) between the fruit's pose and where the stem would hold it."""
    o, R = stem_spec.joined_pose(crown_state.origin, crown_state.rotation())
    d = float(np.linalg.norm(np.asarray(fruit_state.origin, float) - o)) * 1000
    c = (np.trace(R.T @ fruit_state.rotation()) - 1) / 2
    return d, math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def detached(stem_spec, crown_state, fruit_state):
    d, a = stem_error(stem_spec, crown_state, fruit_state)
    return d > DETACH_MM or a > DETACH_DEG


def member_names(objects):
    return tuple(o.name for o in objects if any(c.__name__ == "BunchPart" for c in type(o).__mro__))


# ---------------------------------------------------------------- runtime (Isaac Sim only; imported lazily)
_PATHS: dict[str, str] = {}  # body name -> prim path
_STEM_PATHS: dict[str, str] = {}  # stem name -> current joint prim
_GEN: dict[str, int] = {}
_BROKEN: set[str] = set()  # stems PhysX reported broken this episode
EVENTS: list[tuple[str, str]] = []  # (stem, joint path) of every break, for diagnostics
_SUB = []
_CFG = []


def _subscribe():
    if _SUB:
        return
    try:
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import SimulationEvent
        from pxr import PhysicsSchemaTools
    except Exception as e:  # noqa: BLE001
        print(f"BUNCH_KIT no joint-break events: {e!r}", flush=True)
        _SUB.append(None)
        return

    def on_event(event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            path = str(PhysicsSchemaTools.decodeSdfPath(*event.payload["jointPath"]))
            for name, p in _STEM_PATHS.items():
                if p == path:
                    _BROKEN.add(name)
                    EVENTS.append((name, path))
                    print(f"BUNCH_BREAK {name} {path}", flush=True)

    _SUB.append(get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_event))


def _author_stem(stage, s):
    from pxr import Gf, UsdPhysics

    gen = _GEN.get(s.name, 0)
    path = f"{_PATHS[s.crown]}/stem_{s.name}" + (f"_{gen}" if gen else "")
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([_PATHS[s.crown]])
    joint.CreateBody1Rel().SetTargets([_PATHS[s.fruit]])
    a0 = np.asarray(s.anchor, float) - np.asarray(s.crown_pos, float)
    half = s.yaw / 2
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in a0)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(math.cos(half)), Gf.Vec3f(0.0, 0.0, float(math.sin(half)))))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBreakForceAttr().Set(s.break_force)
    joint.CreateBreakTorqueAttr().Set(s.break_torque)
    joint.CreateJointEnabledAttr().Set(True)
    _STEM_PATHS[s.name] = path


def _spawn_body(prim_path, cfg, *args, **kwargs):
    import isaaclab.sim as sim_utils
    from pxr import PhysxSchema
    from yam_arena.station import linear_rgb

    spec = parts._SPECS[cfg.part_name]
    prim = ymesh._spawn_mesh_part(prim_path, cfg, *args, **kwargs)
    stage = kwargs.get("stage") or sim_utils.get_current_stage()
    for mesh in spec.meshes:  # matte peel and crown instead of the kit's metallic mesh look
        look_path = f"{prim_path}/mesh_{mesh.name}_matte"
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(mesh.color or spec.color), roughness=0.6,
                                           metallic=0.0)
        look.func(look_path, look)
        sim_utils.bind_visual_material(f"{prim_path}/mesh_{mesh.name}", look_path, stage=stage)
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rigid.CreateSolverPositionIterationCountAttr().Set(32)
    rigid.CreateSolverVelocityIterationCountAttr().Set(4)
    rigid.CreateMaxDepenetrationVelocityAttr().Set(1.0)
    rigid.CreateLinearDampingAttr().Set(0.05)
    rigid.CreateAngularDampingAttr().Set(0.05)
    _PATHS[spec.name] = str(prim_path)
    _subscribe()
    for s in STEMS:
        if spec.name in (s.crown, s.fruit) and s.crown in _PATHS and s.fruit in _PATHS:
            _author_stem(stage, s)
    return prim


def _bunch_object(spec):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass

    obj = ymesh._mesh_part_object(spec)
    if not _CFG:
        _CFG.append((ymesh.spawner_config_type(sim_utils.SpawnerCfg, configclass), clone(_spawn_body)))
    cfg_type, spawn = _CFG[0]
    obj.spawner_cfg = obj.object_cfg.spawn = cfg_type(func=spawn, part_name=spec.name)
    _PATHS.pop(spec.name, None)
    return obj


def _poses(base, names):
    def arr(t):
        t = getattr(t, "torch", t)
        return t.detach().cpu().numpy().astype(float)

    origin = arr(base.scene.env_origins)[0]
    out = {}
    for n in names:
        pose = arr(base.scene[n].data.root_pose_w)[0]
        out[n] = (pose[:3] - origin, pose[3:])
    return out


class _Pose:
    def __init__(self, origin, quat_xyzw):
        self.origin = np.asarray(origin, float)
        self.quat_xyzw = np.asarray(quat_xyzw, float)

    def rotation(self):
        x, y, z, w = self.quat_xyzw / np.linalg.norm(self.quat_xyzw)
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _around_env_reset(base, env_reset, seed, jitter, members, args, kwargs):
    """Inside the env's reset: note broken stems, reset, shift the hand as one, replace broken stems."""
    broken = set(_BROKEN)
    try:
        poses = {n: _Pose(*p) for n, p in _poses(base, members).items()}
        broken |= {s.name for s in STEMS if s.crown in poses and s.fruit in poses
                   and detached(s, poses[s.crown], poses[s.fruit])}
    except Exception as e:  # noqa: BLE001
        print(f"BUNCH_KIT pose read before reset failed: {e!r}", flush=True)
    result = env_reset(*args, **kwargs)
    if seed is not None and jitter > 0:
        import torch

        d = np.random.default_rng([int(seed), 7019]).uniform(-jitter, jitter, size=2)
        for n in members:
            obj = base.scene[n]
            pose = obj.data.default_root_pose.torch.clone()
            pose[:, :3] += base.scene.env_origins
            pose[:, :2] += torch.tensor(d, dtype=pose.dtype, device=pose.device)
            obj.write_root_pose_to_sim_index(root_pose=pose)
            obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
        print(f"BUNCH_JITTER seed={seed} dx_mm={d[0] * 1000:.1f} dy_mm={d[1] * 1000:.1f}", flush=True)
    if broken:
        import isaaclab.sim as sim_utils

        from pxr import UsdPhysics

        stage = sim_utils.get_current_stage()
        for s in STEMS:
            if s.name in broken and s.name in _STEM_PATHS:
                UsdPhysics.Joint(stage.GetPrimAtPath(_STEM_PATHS[s.name])).GetJointEnabledAttr().Set(False)
                _GEN[s.name] = _GEN.get(s.name, 0) + 1
                _author_stem(stage, s)
                print(f"BUNCH_REARM {s.name} -> {_STEM_PATHS[s.name]}", flush=True)
    _BROKEN.clear()
    return result


def _patch_scene(scene):
    ymesh._patch_scene(scene)
    scene._bunch_kit_impl = sys.modules[__name__]
    if getattr(scene, "_bunch_kit_patched", False):
        return
    inner = scene.task_object

    @functools.wraps(inner)
    def task_object(spec):
        if any(c.__name__ == "BunchPart" for c in type(spec).__mro__):
            return scene._bunch_kit_impl._bunch_object(spec)
        return inner(spec)

    scene.task_object = task_object
    scene._bunch_kit_patched = True


class _SceneFinder(importlib.abc.MetaPathFinder):
    _bunch_kit_finder = True

    def __init__(self, patch):
        self.patch = patch

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is not None and spec.loader is not None:
            spec.loader = parts._SceneLoader(spec.loader, self.patch)
        return spec


def _install_embodiment():
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except Exception:  # noqa: BLE001  (offline: no simulator)
        return
    cls._bunch_kit_impl = sys.modules[__name__]
    if getattr(cls, "_bunch_kit_hooked", False):
        return
    ensure0, reset0 = cls._ensure_sim, cls.reset

    def _ensure_sim(self, goal):
        ensure0(self, goal)
        names = cls._bunch_kit_impl.member_names(self.task.objects) if self.task is not None else ()
        if names and hasattr(self, "_free_objects"):
            self._free_objects = tuple(n for n in self._free_objects if n not in names)

    def reset(self, scene, *, seed=None):
        impl = cls._bunch_kit_impl
        members = impl.member_names(self.task.objects) if self.task is not None else ()
        if not members:
            return reset0(self, scene, seed=seed)
        self._ensure_sim(None)
        base = self._env.unwrapped
        had, saved = "reset" in vars(base), vars(base).get("reset")
        env_reset = base.reset
        jitter = self.object_jitter
        base.reset = lambda *a, **kw: impl._around_env_reset(base, env_reset, seed, jitter, members, a, kw)
        try:
            return reset0(self, scene, seed=seed)
        finally:
            if had:
                base.reset = saved
            else:
                del base.reset

    cls._ensure_sim, cls.reset = _ensure_sim, reset
    cls._bunch_kit_hooked = True


def install():
    finder = next((f for f in sys.meta_path if getattr(f, "_bunch_kit_finder", False)), None)
    if finder is None:
        finder = _SceneFinder(_patch_scene)
    else:
        sys.meta_path.remove(finder)
        finder.patch = _patch_scene
    sys.meta_path.insert(0, finder)
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    _install_embodiment()


# ---------------------------------------------------------------- the hands' squeeze (scoring: dents)
def finger_squeeze(env):
    """Per arm: (pads' gap m, squeeze N) from the finger drive: stiffness x how far the closing target is past the
    fingers (both fingers press equally on a held part), capped at the drive's effort."""
    from yam_arena.constants import FINGER_JOINTS, GRIPPER

    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0][ids]
        qt = getattr(art.data.joint_pos_target, "torch", art.data.joint_pos_target)[0][ids]
        q, qt = q.detach().cpu().numpy().astype(float), qt.detach().cpu().numpy().astype(float)
        force = np.clip(GRIPPER["stiffness"] * (qt - q), 0.0, GRIPPER["effort"])
        out[arm] = (float(-2 * np.mean(q)), float(np.mean(force)))
    return out


install()
