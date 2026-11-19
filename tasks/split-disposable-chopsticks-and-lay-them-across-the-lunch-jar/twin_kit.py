"""Two bodies joined by a bond that snaps: a pair of sticks that splits, a tab that breaks off a card, a wafer.

What it models. Each half is a :class:`TwinLeg`: an ordinary free compound part of the parts kit (boxes, cylinders),
spawned with 32 solver position iterations, 1 m/s maximum depenetration speed and light damping, so a body of a few
grams keeps still between pads. A :class:`Bond` is one USD ``PhysicsFixedJoint`` between two halves at a stated
point, with ``breakTorque`` and ``breakForce``; contact between the two halves is off while it holds and the joint is
kept out of any articulation. PhysX alone decides when the bond lets go: the moment (or pull) carried through the
joint at its anchor exceeds the limit. Nothing here moves, welds or pushes a body while an episode runs. Once a bond
has broken, :func:`release_broken` switches the dead joint prim off, which only gives the two halves back their
contact with each other (two split sticks can then lie against one another).

Resets. The halves of a bond move together: the harness's per-object start shift skips them and this module shifts
both by one random x/y offset of the same size (seeded resets only). A bond that broke in the last episode is
switched off and a fresh joint prim is authored beside it once the halves are back in place.

Reading the hands (for scripted demos and diagnostics, never for moving objects): :func:`hand_force` gives the force
an arm applies at its tool point from its joints' position error (drive stiffness times error, through the arm's
Jacobian); :func:`finger_state` gives the pads' opening and squeeze from the finger drive.

Tuned numbers used by the chopsticks task (all estimates): break torque 0.8 N m about the root of the slit (about
4 N at the open ends of a 24 cm pair), break force 100 N; stick friction 0.8/0.7; contact offset 1 mm; a pose net of
8 mm / 8 degrees behind the native break (never needed in the recorded runs).
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
from yam_parts import Part

SPLIT_MM, SPLIT_DEG = 8.0, 8.0  # two halves this far off their joined pose have come apart (a held bond keeps them under 1)


class TwinLeg(Part):
    """One half of a bonded pair. Free (it falls, can be held and pushed). Both halves of a bond must be given the
    same `pos` (one shared frame), so the joined pose is simply `same origin, same rotation`."""


@dataclass(frozen=True)
class Bond:
    """A breakable bond between halves `a` and `b` at `anchor` (their shared part frame, metres): it holds until the
    moment at the anchor exceeds `break_torque` (N m) or the force `break_force` (N)."""

    name: str
    a: str
    b: str
    anchor: tuple[float, float, float]
    break_torque: float
    break_force: float


BONDS: list[Bond] = []


def bond(name, a, b, anchor, break_torque, break_force):
    """Register the bond joining TwinLegs `a` and `b` (same pos) at `anchor` in their shared frame."""
    if tuple(a.pos) != tuple(b.pos) or a.yaw != b.yaw:
        raise ValueError("the two halves of a bond must share one frame (same pos and yaw)")
    s = Bond(name, a.name, b.name, tuple(float(v) for v in anchor), float(break_torque), float(break_force))
    BONDS[:] = [t for t in BONDS if t.name != name] + [s]
    return s


def bond_error(state_a, state_b):
    """(mm, deg) between half b's pose and half a's (zero while the bond holds)."""
    d = float(np.linalg.norm(np.asarray(state_b.origin, float) - np.asarray(state_a.origin, float))) * 1000
    c = (np.trace(state_a.rotation().T @ state_b.rotation()) - 1) / 2
    return d, math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def apart(state_a, state_b):
    d, a = bond_error(state_a, state_b)
    return d > SPLIT_MM or a > SPLIT_DEG


def member_names(objects):
    return tuple(o.name for o in objects if any(c.__name__ == "TwinLeg" for c in type(o).__mro__))


# ---------------------------------------------------------------- runtime (Isaac Sim only; imported lazily)
_PATHS: dict[str, str] = {}  # body name -> prim path
_BOND_PATHS: dict[str, str] = {}  # bond name -> current joint prim
_GEN: dict[str, int] = {}
_BROKEN: set[str] = set()  # bonds PhysX reported broken this episode
_RELEASED: set[str] = set()  # broken bonds whose dead joint prim has been switched off
EVENTS: list[tuple[str, str]] = []
_SUB = []
_SPAWN = []


def _subscribe():
    if _SUB:
        return
    try:
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import SimulationEvent
        from pxr import PhysicsSchemaTools
    except Exception as e:  # noqa: BLE001
        print(f"TWIN_KIT no joint-break events: {e!r}", flush=True)
        _SUB.append(None)
        return

    def on_event(event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            path = str(PhysicsSchemaTools.decodeSdfPath(*event.payload["jointPath"]))
            for name, p in _BOND_PATHS.items():
                if p == path:
                    _BROKEN.add(name)
                    EVENTS.append((name, path))
                    print(f"TWIN_BREAK {name} {path}", flush=True)

    _SUB.append(get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_event))


def _author_bond(stage, s):
    from pxr import Gf, UsdPhysics

    gen = _GEN.get(s.name, 0)
    path = f"{_PATHS[s.a]}/bond_{s.name}" + (f"_{gen}" if gen else "")
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([_PATHS[s.a]])
    joint.CreateBody1Rel().SetTargets([_PATHS[s.b]])
    anchor = Gf.Vec3f(*(float(v) for v in s.anchor))
    joint.CreateLocalPos0Attr().Set(anchor)
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(anchor)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBreakForceAttr().Set(s.break_force)
    joint.CreateBreakTorqueAttr().Set(s.break_torque)
    joint.CreateJointEnabledAttr().Set(True)
    _BOND_PATHS[s.name] = path


def _spawn_leg(prim_path, cfg, *args, **kwargs):
    import isaaclab.sim as sim_utils
    from pxr import PhysxSchema

    spec = parts._SPECS[cfg.part_name]
    prim = parts._spawn_part(prim_path, cfg, *args, **kwargs)
    stage = kwargs.get("stage") or sim_utils.get_current_stage()
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rigid.CreateSolverPositionIterationCountAttr().Set(32)
    rigid.CreateSolverVelocityIterationCountAttr().Set(4)
    rigid.CreateMaxDepenetrationVelocityAttr().Set(1.0)
    rigid.CreateLinearDampingAttr().Set(0.05)
    rigid.CreateAngularDampingAttr().Set(0.05)
    _PATHS[spec.name] = str(prim_path)
    _subscribe()
    for s in BONDS:
        if spec.name in (s.a, s.b) and s.a in _PATHS and s.b in _PATHS:
            _author_bond(stage, s)
    return prim


def _leg_object(spec):
    from isaaclab.sim.utils import clone

    obj = parts._part_object(spec)
    if not _SPAWN:
        _SPAWN.append(clone(_spawn_leg))
    obj.spawner_cfg = obj.object_cfg.spawn = parts._SPAWNER_CFG_TYPE(func=_SPAWN[0], part_name=spec.name)
    parts._PART_NAMES.add(spec.name)
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
        out[n] = Pose(pose[:3] - origin, pose[3:])
    return out


class Pose:
    """A body frame: origin and quaternion (x, y, z, w), with the parts kit's rotation()."""

    def __init__(self, origin, quat_xyzw=(0.0, 0.0, 0.0, 1.0)):
        self.origin = np.asarray(origin, float)
        self.quat_xyzw = np.asarray(quat_xyzw, float)

    def rotation(self):
        x, y, z, w = self.quat_xyzw / np.linalg.norm(self.quat_xyzw)
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def broken(name):
    """True once PhysX has reported this bond broken in the current episode."""
    return name in _BROKEN


def release_broken(env, states=None):
    """Switch off the joint prim of every bond that has broken (seen as a PhysX break event, or by the two halves'
    poses in `states`), once. It is already dead; this only restores contact between its two halves."""
    if env is None:
        return
    for s in BONDS:
        if s.name in _RELEASED or s.name not in _BOND_PATHS:
            continue
        gone = s.name in _BROKEN
        if not gone and states is not None and s.a in states and s.b in states:
            gone = apart(states[s.a], states[s.b])
            if gone:
                _BROKEN.add(s.name)
        if gone:
            try:
                import isaaclab.sim as sim_utils
                from pxr import UsdPhysics

                stage = sim_utils.get_current_stage()
                UsdPhysics.Joint(stage.GetPrimAtPath(_BOND_PATHS[s.name])).GetJointEnabledAttr().Set(False)
                print(f"TWIN_RELEASE {s.name} {_BOND_PATHS[s.name]}", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"TWIN_KIT release failed: {e!r}", flush=True)
            _RELEASED.add(s.name)


def _around_env_reset(base, env_reset, seed, jitter, members, args, kwargs):
    """Inside the env's reset: note broken bonds, reset, shift each pair as one, replace broken bonds."""
    was_broken = set(_BROKEN)
    try:
        poses = _poses(base, members)
        was_broken |= {s.name for s in BONDS if s.a in poses and s.b in poses and apart(poses[s.a], poses[s.b])}
    except Exception as e:  # noqa: BLE001
        print(f"TWIN_KIT pose read before reset failed: {e!r}", flush=True)
    if was_broken:
        import isaaclab.sim as sim_utils
        from pxr import UsdPhysics

        stage = sim_utils.get_current_stage()
        for s in BONDS:
            if s.name in was_broken and s.name in _BOND_PATHS:
                UsdPhysics.Joint(stage.GetPrimAtPath(_BOND_PATHS[s.name])).GetJointEnabledAttr().Set(False)
    result = env_reset(*args, **kwargs)
    if seed is not None and jitter > 0:
        import torch

        for k, s in enumerate(BONDS):
            d = np.random.default_rng([int(seed), 7019, k]).uniform(-jitter, jitter, size=2)
            for n in (s.a, s.b):
                obj = base.scene[n]
                pose = obj.data.default_root_pose.torch.clone()
                pose[:, :3] += base.scene.env_origins
                pose[:, :2] += torch.tensor(d, dtype=pose.dtype, device=pose.device)
                obj.write_root_pose_to_sim_index(root_pose=pose)
                obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
            print(f"TWIN_JITTER {s.name} seed={seed} dx_mm={d[0] * 1000:.1f} dy_mm={d[1] * 1000:.1f}", flush=True)
    if was_broken:
        import isaaclab.sim as sim_utils

        stage = sim_utils.get_current_stage()
        for s in BONDS:
            if s.name in was_broken and s.name in _BOND_PATHS:
                _GEN[s.name] = _GEN.get(s.name, 0) + 1
                _author_bond(stage, s)
                print(f"TWIN_REARM {s.name} -> {_BOND_PATHS[s.name]}", flush=True)
    _BROKEN.clear()
    _RELEASED.clear()
    return result


def _patch_scene(scene):
    parts._patch_scene(scene)
    for other in ("yam_joints", "yam_mesh", "yam_linkage"):
        mod = sys.modules.get(other)
        if mod is not None and hasattr(mod, "_patch_scene"):
            mod._patch_scene(scene)
    scene._twin_kit_impl = sys.modules[__name__]
    if getattr(scene, "_twin_kit_patched", False):
        return
    inner = scene.task_object

    @functools.wraps(inner)
    def task_object(spec):
        if any(c.__name__ == "TwinLeg" for c in type(spec).__mro__):
            return scene._twin_kit_impl._leg_object(spec)
        return inner(spec)

    scene.task_object = task_object
    scene._twin_kit_patched = True


class _SceneFinder(importlib.abc.MetaPathFinder):
    _twin_kit_finder = True

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
    cls._twin_kit_impl = sys.modules[__name__]
    if getattr(cls, "_twin_kit_hooked", False):
        return
    ensure0, reset0 = cls._ensure_sim, cls.reset

    def _ensure_sim(self, goal):
        ensure0(self, goal)
        names = cls._twin_kit_impl.member_names(self.task.objects) if self.task is not None else ()
        if names and hasattr(self, "_free_objects"):
            self._free_objects = tuple(n for n in self._free_objects if n not in names)

    def reset(self, scene, *, seed=None):
        impl = cls._twin_kit_impl
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
    cls._twin_kit_hooked = True


def install():
    finder = next((f for f in sys.meta_path if getattr(f, "_twin_kit_finder", False)), None)
    if finder is None:
        finder = _SceneFinder(_patch_scene)
    else:
        sys.meta_path.remove(finder)
        finder.patch = _patch_scene
    sys.meta_path.insert(0, finder)
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    _install_embodiment()


# ---------------------------------------------------------------- reading the hands
def _np(t):
    t = getattr(t, "torch", t)
    return t.detach().cpu().numpy().astype(float)


def arm_joints(env, arm):
    """(measured arm joints, their targets) for one arm, radians."""
    from yam_arena.constants import ARM_JOINTS

    art = env.unwrapped.scene[arm + "_robot"]
    ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
    return _np(art.data.joint_pos)[0][ids], _np(art.data.joint_pos_target)[0][ids]


def hand_force(env, arm, base):
    """Force (N, world x, y, z) the arm applies at its tool point, from its joints' position error: the drives hold
    each joint with stiffness K, so their torques are K (target - measured) and J^T F equals them. It includes the
    arm's own weight, so read it once before contact and subtract."""
    from yam_arena import ik
    from yam_arena.constants import SHOULDER, WRIST

    k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    q, qt = arm_joints(env, arm)
    tau = k * (qt - q)
    p0, r0 = ik.tcp_pose(q, base)
    jac = np.zeros((6, 6))
    for j in range(6):
        dq = q.copy()
        dq[j] += 1e-5
        p1, r1 = ik.tcp_pose(dq, base)
        w = r1 @ r0.T
        jac[:3, j] = (p1 - p0) / 1e-5
        jac[3:, j] = np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]]) / 2 / 1e-5
    return np.linalg.lstsq(jac.T, tau, rcond=None)[0][:3]


def finger_state(env, arm):
    """(pads' opening m, squeeze N per pad) from the finger drive: stiffness times how far the closing target is
    past the fingers, capped at the drive's effort."""
    from yam_arena.constants import FINGER_JOINTS, GRIPPER

    art = env.unwrapped.scene[arm + "_robot"]
    ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
    q, qt = _np(art.data.joint_pos)[0][ids], _np(art.data.joint_pos_target)[0][ids]
    force = np.clip(GRIPPER["stiffness"] * (qt - q), 0.0, GRIPPER["effort"])
    return float(-2 * np.mean(q)), float(np.mean(force))


install()
