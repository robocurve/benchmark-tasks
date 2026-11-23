"""soilkit: loose soil as a few thousand small rigid grains, for a yam_arena task file (with ``scenekit``).

Isaac Lab tracks each scene asset as one body; a pot of soil is thousands. A :class:`Soil` spawns as one fixed
(non-physics) asset whose spawner authors every grain straight into USD as its own rigid sphere, simulated by PhysX
like any other body. After every control step all grain centres are read through one PhysX rigid-body view
(:func:`grains`); at every reset after the first they are put back where they started.

The grains start from ``start_file`` (an ``.npz`` of settled centres and radii; ``SOIL_SAVE=<path>`` writes one every
20 control steps) or, without one, from a loose jittered lattice filling the box ``pos`` +- ``size``/2, which settles
in the first second. Pure Python at import time.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

import scenekit

HERE = Path(__file__).resolve().parent
Color = tuple[float, float, float]


@dataclass(frozen=True)
class Soil(Box):
    """Grains filling the box ``pos`` +- ``size``/2 (station frame, m)."""

    r_min: float = 0.002
    r_max: float = 0.003
    density: float = 1000.0  # kg/m^3 of one grain
    friction: tuple[float, float] = (1.0, 0.9)
    angular_damping: float = 8.0  # stands in for rolling resistance (PhysX spheres roll freely)
    linear_damping: float = 0.1
    colors: tuple[Color, ...] = ((0.22, 0.15, 0.10), (0.28, 0.19, 0.12), (0.18, 0.12, 0.08), (0.33, 0.23, 0.15))
    seed: int = 0
    start_file: str | None = None
    solver_iters: int = 8

    def _check(self) -> None:
        super()._check()
        if not 0 < self.r_min <= self.r_max:
            raise ValueError(f"{self.name}: need 0 < r_min <= r_max")


_STATE: dict[str, dict] = {}


def lattice(s: Soil) -> tuple[np.ndarray, np.ndarray]:
    """(centres N x 3, radii N): a loose staggered lattice filling the soil's box, jittered."""
    rng = np.random.default_rng(s.seed)
    d = 2 * s.r_max * 1.02
    lo = np.asarray(s.pos, float) - np.asarray(s.size, float) / 2 + s.r_max
    hi = np.asarray(s.pos, float) + np.asarray(s.size, float) / 2 - s.r_max
    pts = []
    for k, z in enumerate(np.arange(lo[2], hi[2] + 1e-9, d * 0.82)):
        off = (k % 2) * d / 4
        for x in np.arange(lo[0] + off, hi[0] + 1e-9, d):
            for y in np.arange(lo[1] + off, hi[1] + 1e-9, d):
                pts.append((x, y, z))
    p = np.array(pts) + rng.uniform(-1, 1, (len(pts), 3)) * (d / 2 - s.r_max)
    return p, rng.uniform(s.r_min, s.r_max, len(p))


def start_layout(s: Soil) -> tuple[np.ndarray, np.ndarray]:
    if s.start_file and (HERE / s.start_file).is_file():
        data = np.load(HERE / s.start_file)
        return data["pos"].astype(float), data["radius"].astype(float)
    return lattice(s)


def _author(prim_path: str, s: Soil) -> None:
    """Author the grains under ``prim_path`` (one env's soil prim), in that env's frame."""
    import isaaclab.sim as sim_utils
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    from yam_arena.station import linear_rgb

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, prim_path)
    pos, rad = start_layout(s)
    mat_path = f"{prim_path}/physics"
    pm = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, mat_path).GetPrim())
    pm.CreateStaticFrictionAttr(float(s.friction[0]))
    pm.CreateDynamicFrictionAttr(float(s.friction[1]))
    pm.CreateRestitutionAttr(0.0)
    looks = []
    for k, c in enumerate(s.colors):
        cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(c), roughness=0.95)
        cfg.func(f"{prim_path}/look{k}", cfg)
        looks.append(f"{prim_path}/look{k}")
    shade = np.random.default_rng(s.seed + 1).integers(0, len(looks), len(pos))
    layer = stage.GetEditTarget().GetLayer()
    with Sdf.ChangeBlock():
        for i, (p, r) in enumerate(zip(pos, rad)):
            prim = Sdf.CreatePrimInLayer(layer, f"{prim_path}/g{i:05d}")
            prim.specifier = Sdf.SpecifierDef
            prim.typeName = "Sphere"
            prim.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(
                ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI",
                 "PhysxCollisionAPI", "MaterialBindingAPI"]))

            def attr(name, tname, value, prim=prim):
                Sdf.AttributeSpec(prim, name, tname).default = value

            r = float(r)
            attr("radius", Sdf.ValueTypeNames.Double, r)
            attr("extent", Sdf.ValueTypeNames.Float3Array, [Gf.Vec3f(-r, -r, -r), Gf.Vec3f(r, r, r)])
            attr("xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*map(float, p)))
            attr("xformOpOrder", Sdf.ValueTypeNames.TokenArray, ["xformOp:translate"])
            attr("physics:mass", Sdf.ValueTypeNames.Float, float(s.density * 4 / 3 * np.pi * r ** 3))
            attr("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, float(s.angular_damping))
            attr("physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float, float(s.linear_damping))
            attr("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float, 0.3)
            attr("physxRigidBody:maxLinearVelocity", Sdf.ValueTypeNames.Float, 1.0)
            attr("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, int(s.solver_iters))
            attr("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1)
            attr("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 5.0e-5)
            attr("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, 0.001)
            attr("physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.0)
            rel = Sdf.RelationshipSpec(prim, "material:binding", False)
            rel.targetPathList.explicitItems = [Sdf.Path(looks[shade[i]])]
            rel = Sdf.RelationshipSpec(prim, "material:binding:physics", False)
            rel.targetPathList.explicitItems = [Sdf.Path(mat_path)]
    _STATE[s.name] = {"root": prim_path, "start": pos.copy(), "radius": rad.copy(), "now": pos.copy(),
                      "view": None, "steps": 0, "episodes": 0}
    print(f"[soil] {s.name}: {len(pos)} grains under {prim_path}", flush=True)


def _build(s: Soil):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        _author(prim_path, s)
        import omni.usd

        return omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)

    @configclass
    class SoilCfg(sim_utils.SpawnerCfg):
        func: object = spawn

    obj = Object(name=s.name, object_type=ObjectType.BASE, spawner_cfg=SoilCfg())
    obj.set_initial_pose(Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))
    return obj


def _view(name: str):
    st = _STATE[name]
    if st["view"] is None:
        from isaaclab_physx.physics import PhysxManager

        st["view"] = PhysxManager.get_physics_sim_view().create_rigid_body_view(f"{st['root']}/g*")
        if st["view"].count != len(st["start"]):
            raise RuntimeError(f"soil {name}: the view found {st['view'].count} of {len(st['start'])} grains")
    return st["view"]


def grains(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(centres N x 3 in the station frame, radii N) after the last control step."""
    st = _STATE[name]
    return st["now"], st["radius"]


def _state(environment, s: Soil) -> ObjectState:
    p, r = grains(s.name)
    low, high = (p - r[:, None]).min(0), (p + r[:, None]).max(0)
    return ObjectState(pos=(low + high) / 2, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=low, high=high)


scenekit.KINDS[Soil] = (_build, _state)


@scenekit.on_step
def _read(environment) -> None:
    try:
        _read_now(environment)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


def _read_now(environment) -> None:
    import warp as wp

    origin = scenekit._np(environment.unwrapped.scene.env_origins[0])
    for name, st in _STATE.items():
        view = _view(name)
        st["now"] = wp.to_torch(view.get_transforms()).cpu().numpy()[:, :3].astype(float) - origin
        st["steps"] += 1
        save = os.environ.get("SOIL_SAVE")
        if save and st["steps"] % 20 == 0:
            v = np.linalg.norm(wp.to_torch(view.get_velocities()).cpu().numpy()[:, :3], axis=1)
            np.savez(save, pos=st["now"], radius=st["radius"])
            print(f"[soil] step {st['steps']}: saved; top {st['now'][:, 2].max():.4f}, max speed {v.max():.4f} m/s",
                  flush=True)


@scenekit.on_reset
def _put_back(environment) -> None:
    try:
        _put_back_now(environment)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


def _put_back_now(environment) -> None:
    import torch
    import warp as wp

    for st in _STATE.values():
        st["episodes"] += 1
        st["steps"] = 0
        if st["episodes"] == 1:  # the first episode: they are where they were authored
            continue
        view = _view_of(st)
        n = len(st["start"])
        origin = scenekit._np(environment.unwrapped.scene.env_origins[0])
        pose = torch.zeros((n, 7), dtype=torch.float32, device="cuda:0")
        pose[:, :3] = torch.tensor(st["start"] + origin, dtype=torch.float32)
        pose[:, 6] = 1.0
        idx = wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32, device="cuda:0")
        view.set_transforms(wp.from_torch(pose), indices=idx)
        view.set_velocities(wp.from_torch(torch.zeros((n, 6), dtype=torch.float32, device="cuda:0")), indices=idx)
        st["now"] = st["start"].copy()


def _view_of(st):
    return _view(next(k for k, v in _STATE.items() if v is st))
