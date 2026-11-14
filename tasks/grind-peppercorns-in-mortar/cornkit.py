"""cornkit: many small loose bodies (peppercorns and their cracked pieces) for a yam_arena task file, with a hidden
pool that pieces are moved in from and parked back into.

A task declares its bodies as `Body`s (a sphere: radius, mass, color, a `home` where the scene starts and resets it,
and a `park` spot out of sight) and calls `setup(bodies, ...)` at import. When the simulator builds the scene, each
body is spawned as its own free rigid body. The bodies are not in the task's OBJECTS: yam_arena's object jitter moves
every free object independently by up to 3 cm, which would pile a layer of corns into each other and into the
bowl's wall, so they keep the layout given here (the scene's reset puts them back at `home`).

After every control step `objects["pepper"]` holds a (N, 7) array: each body's position (x, y, z, in the same frame as
the other objects) and orientation (quaternion x, y, z, w), in the order of `bodies`. The task may call `move(i, pos)`
to put body i at `pos` (same frame, at rest), or `move(i, None)` to park it; the moves are made at the next control
step, before the bodies are read. That is the only thing this module does to the bodies; the physics engine moves
them otherwise.

Nothing on disk changes. The module hooks yam_arena.scene when the simulator imports it (as yamkit does), wrapping
task_room (to add the bodies to the scene) and object_states (to make the moves and read the bodies). Plain Python
and NumPy at import time.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

import numpy as np

KEY = "pepper"


@dataclass(frozen=True)
class Body:
    name: str
    radius: float  # m
    mass: float  # kg
    color: tuple  # sRGB 0..1
    home: tuple  # m: where the scene starts and resets it
    park: tuple  # m: its spot in the hidden pool


@dataclass(frozen=True)
class Physics:
    """The bodies' contact properties. Friction is combined with "min" against other surfaces."""

    static_friction: float = 0.35
    dynamic_friction: float = 0.3
    contact_offset: float = 0.001  # m
    rest_offset: float = 0.0
    solver_position_iterations: int = 16
    solver_velocity_iterations: int = 2
    max_depenetration_velocity: float = 0.3  # m/s: a body pinched into a collider is pushed out no faster
    angular_damping: float = 0.0  # 1/s
    torsion: tuple = (0.0, 0.0)  # (patch radius, minimum patch radius) for torsional friction, m


_SPEC: dict = {"bodies": (), "physics": Physics()}
_MOVES: list = []


def setup(bodies, physics: Physics = Physics()) -> None:
    names = [b.name for b in bodies]
    if len(set(names)) != len(names):
        raise ValueError("cornkit body names must be unique")
    _SPEC["bodies"], _SPEC["physics"] = tuple(bodies), physics
    _MOVES.clear()


def bodies() -> tuple:
    return _SPEC["bodies"]


def move(i: int, pos) -> None:
    """Put body i at `pos` (at rest, unturned) at the next control step; None parks it in the pool."""
    _MOVES.append((int(i), None if pos is None else tuple(float(v) for v in pos)))


def clear() -> None:
    """Drop the moves not made yet (a new trial)."""
    _MOVES.clear()


def pending() -> list:
    return list(_MOVES)


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _objects(scene) -> list:
    ph = _SPEC["physics"]
    su = scene.sim_utils
    out = []
    for b in _SPEC["bodies"]:
        cfg = su.SphereCfg(
            radius=float(b.radius),
            rigid_props=scene.PhysxRigidBodyPropertiesCfg(
                solver_position_iteration_count=ph.solver_position_iterations,
                solver_velocity_iteration_count=ph.solver_velocity_iterations,
                max_depenetration_velocity=ph.max_depenetration_velocity,
                angular_damping=float(ph.angular_damping), sleep_threshold=0.0),
            mass_props=su.MassPropertiesCfg(mass=float(b.mass)),
            collision_props=scene.PhysxCollisionPropertiesCfg(
                contact_offset=ph.contact_offset, rest_offset=ph.rest_offset,
                torsional_patch_radius=float(ph.torsion[0]), min_torsional_patch_radius=float(ph.torsion[1])),
            physics_material=su.RigidBodyMaterialCfg(static_friction=ph.static_friction,
                                                     dynamic_friction=ph.dynamic_friction,
                                                     friction_combine_mode="min"),
            visual_material=su.PreviewSurfaceCfg(diffuse_color=tuple(_linear(v) for v in b.color), roughness=0.8),
        )
        out.append(scene.Object(name=b.name, object_type=scene.ObjectType.RIGID, spawner_cfg=cfg,
                                initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in b.home),
                                                        rotation_xyzw=(0.0, 0.0, 0.0, 1.0))))
    return out


def _apply(env) -> None:
    """Make the queued moves: each body put at rest where asked (or at its park spot)."""
    import torch

    if not _MOVES:
        return
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    for i, pos in list(_MOVES):
        b = _SPEC["bodies"][i]
        p = b.park if pos is None else pos
        obj = base.scene.rigid_objects[b.name]
        pose = torch.tensor([[*p, 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=origin.device)
        pose[0, :3] += origin
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                       device=origin.device))
    _MOVES.clear()


def read(env) -> np.ndarray:
    """The bodies' poses now: (N, 7), position minus the env origin, then quaternion x, y, z, w."""
    import torch

    names = [b.name for b in _SPEC["bodies"]]
    if not names:
        return np.zeros((0, 7))
    base = env.unwrapped
    world = base.arena_world
    origin = base.scene.env_origins[0]
    t = torch.stack([world.get_pose_w(n)[0] for n in names])
    t = torch.cat([t[:, :3] - origin[None, :], t[:, 3:]], dim=1)
    return t.detach().cpu().numpy().astype(np.float64)


_EP = {"last": None}


def _patch(scene) -> None:
    if getattr(scene, "_cornkit", False):
        return
    scene._cornkit = True
    room, states = scene.task_room, scene.object_states

    def task_room(*a, **k):
        return [*room(*a, **k), *_objects(scene)]

    def object_states(env, names):
        buf = getattr(env.unwrapped, "episode_length_buf", None)
        if buf is not None:  # a new trial (the env's step count dropped): the scene reset put every body home
            n = int(buf[0].item())
            if _EP["last"] is not None and n < _EP["last"]:
                _MOVES.clear()
            _EP["last"] = n
        if _SPEC["bodies"]:
            _apply(env)
        out = states(env, [n for n in names if n != KEY])
        if _SPEC["bodies"]:
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
