"""debriskit: debris as real rigid bodies (chips, flakes, crusts), loose or bonded, for a yam_arena task file.

A task declares its debris as `Chip`s (a thin box: center, size, yaw, mass, color) and calls `setup(chips, ...)` at
import. When the simulator builds the scene, each chip is spawned as its own free rigid body (a box collider and the
same visible box), with its own friction, contact offset and solver settings (no CCD: the chips are at least 2 mm
thick and move under 1 m/s, under 4 mm per 240 Hz physics step). The chips are not in the task's OBJECTS:
yam_arena's object jitter moves every free object independently by up to 3 cm, which would pile a dense layer of chips
into each other, so they keep the layout given here (the scene's reset puts them back).

A chip with a `bond` is stuck to a fixed anchor body (`setup(..., anchor=(name, pos))`) by a breakable PhysX fixed
joint at the middle of its underside, authored with the chip before the simulation starts. The solver breaks the joint
for good the first time holding the chip takes more than its break force or torque; from then on the chip is free.
Collisions between the chip and the anchor stay on. A broken joint is not restored by the scene's reset (one trial per
run). `Physics.break_scale` multiplies the values written to PhysX (calibrated by the task against a measured push).

After every control step `objects["debris"]` holds a (N, 7) array: each chip's position (x, y, z, in the same frame as
the other objects) and orientation (quaternion x, y, z, w), in the order of `chips`. The scorer reads only that: where
the chips really are. Nothing is drawn or moved by this module; the physics engine holds, frees and pushes the chips.

Nothing on disk changes. The module hooks yam_arena.scene when the simulator imports it (as yamkit does), wrapping
task_room (to add the chips to the scene) and object_states (to read them). Plain Python and NumPy at import time.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np

KEY = "debris"


@dataclass(frozen=True)
class Chip:
    """One loose piece: a box of `size` (x, y, z before `yaw`, m) whose center is at `pos` (world frame, m)."""

    name: str
    pos: tuple
    size: tuple
    yaw: float
    mass: float  # kg
    color: tuple  # sRGB 0..1
    bond: tuple | None = None  # (break force N, break torque N m): stuck to the anchor body; None: loose


@dataclass(frozen=True)
class Physics:
    """The chips' contact properties. Friction is combined with "min" against other surfaces (the lower of the two
    applies), so a chip on a greased plate slides with the plate's friction."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    contact_offset: float = 0.002  # m: contacts are generated from this far out
    rest_offset: float = 0.0
    solver_position_iterations: int = 16
    solver_velocity_iterations: int = 2
    max_depenetration_velocity: float = 0.3  # m/s: a chip pinched into a collider is pushed out no faster
    break_scale: float = 1.0  # the break force/torque written to PhysX = the chip's bond x this (calibration: see task)


_SPEC: dict = {"chips": (), "physics": Physics(), "anchor": None}


def setup(chips, physics: Physics = Physics(), anchor: tuple | None = None) -> None:
    """`anchor` = (name, pos): the fixed (kinematic, unturned) body in the task's OBJECTS that bonded chips stick to,
    and its position (the same frame as the chips)."""
    names = [c.name for c in chips]
    if len(set(names)) != len(names):
        raise ValueError("debris chip names must be unique")
    if any(c.bond is not None for c in chips) and anchor is None:
        raise ValueError("bonded chips need an anchor body")
    _SPEC["chips"], _SPEC["physics"], _SPEC["anchor"] = tuple(chips), physics, anchor


def bond_frames(c: Chip) -> tuple:
    """Where a chip's bond sits: the middle of its underside, given in the anchor's frame (position, quaternion x, y,
    z, w: the chip's yaw) and in the chip's own frame (position; no turn)."""
    _, apos = _SPEC["anchor"]
    p0 = tuple(float(c.pos[i] - apos[i]) for i in range(2)) + (float(c.pos[2] - c.size[2] / 2 - apos[2]),)
    q0 = (0.0, 0.0, math.sin(c.yaw / 2), math.cos(c.yaw / 2))
    return p0, q0, (0.0, 0.0, -float(c.size[2]) / 2)


def _author_bond(prim, prim_path: str) -> None:
    """A breakable fixed joint from the anchor body to the chip just spawned at `prim_path`. PhysX breaks it, for good,
    the first time the force (or torque) it needs to hold the chip passes the chip's break force (or torque); from then
    on the chip is a free body. Collisions between the chip and the anchor stay on (a USD joint turns them off by
    default), so a freed chip still rests on and slides over the plate."""
    from pxr import Gf, UsdPhysics

    parent, name = prim_path.rsplit("/", 1)
    chip = next((c for c in _SPEC["chips"] if c.name == name), None)
    if chip is None or chip.bond is None:
        return
    stage = prim.GetStage()
    anchor = f"{parent}/{_SPEC['anchor'][0]}"
    p0, q0, p1 = bond_frames(chip)
    j = UsdPhysics.FixedJoint.Define(stage, f"{parent}/bond_{name}")
    j.CreateBody0Rel().SetTargets([anchor])
    j.CreateBody1Rel().SetTargets([prim_path])
    j.CreateLocalPos0Attr(Gf.Vec3f(*p0))
    j.CreateLocalRot0Attr(Gf.Quatf(q0[3], q0[0], q0[1], q0[2]))
    j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateCollisionEnabledAttr(True)
    k = _SPEC["physics"].break_scale
    j.CreateBreakForceAttr(float(chip.bond[0] * k))
    j.CreateBreakTorqueAttr(float(chip.bond[1] * k))


_SPAWN: dict = {}


def _bonded_spawner():
    """Isaac Lab's cuboid spawner, then the chip's bond (authored before the simulation starts, so PhysX parses it)."""
    if "f" not in _SPAWN:
        from isaaclab.sim.spawners.shapes import spawn_cuboid
        from isaaclab.sim.utils import clone

        @clone
        def spawn(prim_path, cfg, *args, **kwargs):
            prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
            _author_bond(prim, str(prim_path))
            return prim

        _SPAWN["f"] = spawn
    return _SPAWN["f"]


def chips() -> tuple:
    return _SPEC["chips"]


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _objects(scene) -> list:
    ph = _SPEC["physics"]
    su = scene.sim_utils
    out = []
    for c in _SPEC["chips"]:
        cfg = su.CuboidCfg(
            size=tuple(float(v) for v in c.size),
            rigid_props=scene.PhysxRigidBodyPropertiesCfg(
                solver_position_iteration_count=ph.solver_position_iterations,
                solver_velocity_iteration_count=ph.solver_velocity_iterations,
                max_depenetration_velocity=ph.max_depenetration_velocity),
            mass_props=su.MassPropertiesCfg(mass=float(c.mass)),
            collision_props=scene.PhysxCollisionPropertiesCfg(contact_offset=ph.contact_offset,
                                                              rest_offset=ph.rest_offset),
            physics_material=su.RigidBodyMaterialCfg(static_friction=ph.static_friction,
                                                     dynamic_friction=ph.dynamic_friction,
                                                     friction_combine_mode="min"),
            visual_material=su.PreviewSurfaceCfg(diffuse_color=tuple(_linear(v) for v in c.color), roughness=0.9),
        )
        if c.bond is not None:
            cfg.func = _bonded_spawner()
        q = (0.0, 0.0, math.sin(c.yaw / 2), math.cos(c.yaw / 2))
        out.append(scene.Object(name=c.name, object_type=scene.ObjectType.RIGID, spawner_cfg=cfg,
                                initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in c.pos),
                                                        rotation_xyzw=tuple(float(v) for v in q))))
    return out


def read(env) -> np.ndarray:
    """The chips' poses now: (N, 7), position minus the env origin, then quaternion x, y, z, w."""
    import torch

    names = [c.name for c in _SPEC["chips"]]
    if not names:
        return np.zeros((0, 7))
    base = env.unwrapped
    world = base.arena_world
    origin = base.scene.env_origins[0]
    t = torch.stack([world.get_pose_w(n)[0] for n in names])
    t = torch.cat([t[:, :3] - origin[None, :], t[:, 3:]], dim=1)
    return t.detach().cpu().numpy().astype(np.float64)


def _patch(scene) -> None:
    if getattr(scene, "_debriskit", False):
        return
    scene._debriskit = True
    room, states = scene.task_room, scene.object_states

    def task_room(*a, **k):
        return [*room(*a, **k), *_objects(scene)]

    def object_states(env, names):
        out = states(env, [n for n in names if n != KEY])
        if _SPEC["chips"]:
            out[KEY] = read(env)
        return out

    scene.task_room, scene.object_states = task_room, object_states


class _Hook(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Patches yam_arena.scene as soon as it is imported (after any other hook's patch)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None:
            return None
        self._inner = spec.loader
        spec.loader = self
        return spec

    def create_module(self, spec):
        return self._inner.create_module(spec)

    def exec_module(self, module):
        self._inner.exec_module(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _Hook) for f in sys.meta_path):
    sys.meta_path.insert(0, _Hook())
