"""soilkit: loose soil as a few thousand small rigid grains, for yam_arena task files (with ``bodykit``).

Isaac Lab tracks every scene asset as one rigid body; a bed of soil is thousands. :func:`soil` registers a
:class:`Soil` whose grains are authored straight into USD, next to the ``host`` body when that body is spawned (so they
need no asset of their own), simulated by PhysX like any other rigid body, read back through one PhysX rigid-body view
after every control step (:func:`grains`) and put back where they started at every reset, before the arms settle.

Each grain is a sphere or a convex icosahedron with its own bounding radius
(drawn between ``r_min`` and ``r_max``), coloured from ``colors``. The original
preview uses spheres with strong angular damping. The digging task uses angular
grains, low damping and exact polyhedral masses. Neither model is calibrated
against physical soil.

The grains start from ``start_file`` (an ``.npz`` of settled centres, radii and orientations, which ``SOIL_SAVE=<path>`` writes at
the end of an episode), or, without one, from a loose jittered lattice filling the region, which settles in the
first second. Pure Python at import time.
"""

from __future__ import annotations

import os
import itertools
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import bodykit

Color = tuple[float, float, float]


@dataclass(frozen=True)
class Soil:
    name: str
    host: str  # the Body whose spawning authors the grains (they are put next to it in the env)
    low: tuple[float, float, float]  # the region the lattice fills (world, m), when there is no start file
    high: tuple[float, float, float]
    r_min: float = 0.004
    r_max: float = 0.0055
    density: float = 1800.0  # kg/m^3 of a grain (loose soil of such grains weighs about 1.1 g/cm^3)
    friction: tuple[float, float] = (1.0, 0.9)
    angular_damping: float = 6.0
    linear_damping: float = 0.1
    colors: tuple[Color, ...] = ((0.24, 0.16, 0.10), (0.30, 0.20, 0.12), (0.20, 0.13, 0.08), (0.35, 0.24, 0.15))
    roughness: float = 0.95
    seed: int = 0
    start_file: str | None = None
    keep_out: tuple = ()  # (centre, axis, radius) cylinders the lattice leaves empty (a buried pipe, say)
    solver_iterations: int = 6
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0  # low/high are in this local frame; saved layouts remain world-space
    grain_shape: str = "sphere"


def grain_volume(radius, shape="sphere"):
    factor = 4*np.pi/3 if shape == "sphere" else 5*(3+np.sqrt(5))/12*(4/np.sqrt(10+2*np.sqrt(5)))**3
    return factor*np.asarray(radius)**3


def _icosahedron():
    phi = (1+np.sqrt(5))/2
    points = np.array([(0,a,b*phi) for a in (-1,1) for b in (-1,1)]+
                      [(a,b*phi,0) for a in (-1,1) for b in (-1,1)]+
                      [(b*phi,0,a) for a in (-1,1) for b in (-1,1)], float)
    faces = []
    for a,b,c in itertools.combinations(range(12),3):
        if all(abs(np.linalg.norm(points[i]-points[j])-2)<1e-8 for i,j in ((a,b),(b,c),(c,a))):
            if np.dot(np.cross(points[b]-points[a],points[c]-points[a]),points[a]) < 0:
                b,c=c,b
            faces.append((a,b,c))
    return points/np.sqrt(1+phi**2), faces


_SOILS: dict[str, Soil] = {}
_STATE: dict[str, dict] = {}


def soil(*args, **kwargs) -> Soil:
    s = Soil(*args, **kwargs)
    if s.grain_shape not in ("sphere", "icosahedron"):
        raise ValueError(s.grain_shape)
    _SOILS[s.name] = s
    return s


def _lattice(s: Soil) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(s.seed)
    d = 2 * s.r_max * 1.02
    lo, hi = np.asarray(s.low, float) + s.r_max, np.asarray(s.high, float) - s.r_max
    pts = []
    for k, z in enumerate(np.arange(lo[2], hi[2] + 1e-9, d * 0.82)):
        off = (k % 2) * d / 2
        for x in np.arange(lo[0] + off, hi[0] + 1e-9, d):
            for y in np.arange(lo[1] + off, hi[1] + 1e-9, d):
                pts.append((x, y, z))
    p = np.array(pts) + rng.uniform(-0.15, 0.15, (len(pts), 3)) * (d - 2 * s.r_max + 0.001)
    c, sn = np.cos(s.yaw), np.sin(s.yaw)
    p = p @ np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]]).T + s.origin
    keep = np.ones(len(p), bool)
    for centre, axis, radius in s.keep_out:
        a = np.asarray(axis, float) / np.linalg.norm(axis)
        v = p - np.asarray(centre, float)
        dist = np.linalg.norm(v - np.outer(v @ a, a), axis=1)
        keep &= dist > radius + s.r_max
    p = p[keep]
    r = rng.uniform(s.r_min, s.r_max, len(p))
    return p, r


def start_layout(s: Soil) -> tuple[np.ndarray, np.ndarray]:
    """(centres N x 3, radii N): the settled layout from ``start_file`` if it exists, else the lattice."""
    if s.start_file and Path(s.start_file).is_file():
        data = np.load(s.start_file)
        return data["pos"].astype(float), data["radius"].astype(float)
    return _lattice(s)


def _author(stage, prim_path: str, spec) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    for s in _SOILS.values():
        if s.host != spec.name:
            continue
        parent = prim_path.rsplit("/", 1)[0]
        root = f"{parent}/{s.name}"
        pos, rad = start_layout(s)
        quats = np.tile([0.,0.,0.,1.], (len(pos),1))
        if s.grain_shape != "sphere":
            quats = np.random.default_rng(s.seed+19).normal(size=(len(pos),4))
            quats /= np.linalg.norm(quats,axis=1,keepdims=True)
        if s.start_file and Path(s.start_file).is_file():
            data = np.load(s.start_file)
            if "quat" in data:
                quats = data["quat"].copy()
        ico_points, ico_faces = _icosahedron()
        UsdGeom.Scope.Define(stage, root)
        # one physics material, and one look per colour
        mat_path = f"{root}/physics"
        mat = UsdShade.Material.Define(stage, mat_path)
        pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pm.CreateStaticFrictionAttr(float(s.friction[0]))
        pm.CreateDynamicFrictionAttr(float(s.friction[1]))
        pm.CreateRestitutionAttr(0.0)
        import isaaclab.sim as sim_utils

        from yam_arena.station import linear_rgb

        looks = []
        for k, c in enumerate(s.colors):
            path = f"{root}/look{k}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(c), roughness=s.roughness)
            cfg.func(path, cfg)
            looks.append(path)
        rng = np.random.default_rng(s.seed + 1)
        shade = rng.integers(0, len(looks), len(pos))
        layer = stage.GetEditTarget().GetLayer()
        paths = []
        with Sdf.ChangeBlock():
            for i, (p, r) in enumerate(zip(pos, rad)):
                path = f"{root}/g{i:05d}"
                paths.append(path)
                prim = Sdf.CreatePrimInLayer(layer, path)
                prim.specifier = Sdf.SpecifierDef
                prim.typeName = "Sphere" if s.grain_shape == "sphere" else "Mesh"
                prim.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(
                    ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI",
                     "PhysxCollisionAPI", "PhysxContactReportAPI", "MaterialBindingAPI"]+
                    (["PhysicsMeshCollisionAPI"] if s.grain_shape != "sphere" else [])))

                def attr(name, tname, value, prim=prim):
                    a = Sdf.AttributeSpec(prim, name, tname)
                    a.default = value

                if s.grain_shape == "sphere":
                    attr("radius", Sdf.ValueTypeNames.Double, float(r))
                    attr("extent", Sdf.ValueTypeNames.Float3Array, [Gf.Vec3f(-r, -r, -r), Gf.Vec3f(r, r, r)])
                else:
                    attr("points", Sdf.ValueTypeNames.Point3fArray, [Gf.Vec3f(*map(float,p)) for p in ico_points])
                    attr("faceVertexCounts", Sdf.ValueTypeNames.IntArray, [3]*len(ico_faces))
                    attr("faceVertexIndices", Sdf.ValueTypeNames.IntArray, [v for f in ico_faces for v in f])
                    attr("subdivisionScheme", Sdf.ValueTypeNames.Token, "none")
                    attr("physics:approximation", Sdf.ValueTypeNames.Token, "convexHull")
                    attr("xformOp:scale", Sdf.ValueTypeNames.Float3, Gf.Vec3f(r,r,r))
                    attr("extent", Sdf.ValueTypeNames.Float3Array, [Gf.Vec3f(-1,-1,-1),Gf.Vec3f(1,1,1)])
                attr("physxContactReport:threshold", Sdf.ValueTypeNames.Float, 0.0)
                attr("xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*map(float, p)))
                qx,qy,qz,qw = map(float,quats[i])
                attr("xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(qw,qx,qy,qz))
                attr("xformOpOrder", Sdf.ValueTypeNames.TokenArray, ["xformOp:translate", "xformOp:orient"]+
                     (["xformOp:scale"] if s.grain_shape != "sphere" else []))
                attr("physics:mass", Sdf.ValueTypeNames.Float, float(s.density * grain_volume(r,s.grain_shape)))
                attr("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, float(s.angular_damping))
                attr("physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float, float(s.linear_damping))
                attr("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float, 0.3)
                attr("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, int(s.solver_iterations))
                attr("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1)
                attr("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 5.0e-5)
                attr("physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float, 1.0e-5)
                attr("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, 0.0015)
                attr("physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.0)
                rel = Sdf.RelationshipSpec(prim, "material:binding", False)
                rel.targetPathList.explicitItems = [Sdf.Path(looks[shade[i]])]
                rel = Sdf.RelationshipSpec(prim, "material:binding:physics", False)
                rel.targetPathList.explicitItems = [Sdf.Path(mat_path)]
        _STATE[s.name] = {"paths": paths, "start": pos.copy(), "radius": rad.copy(), "view": None, "root": root,
                          "now": pos.copy(), "steps": 0, "start_quat":quats.copy(), "quat":quats.copy()}


bodykit.EXTRA_AUTHORS.append(_author)


def _view(name: str):
    st = _STATE[name]
    if st["view"] is None:
        from isaaclab_physx.physics import PhysxManager

        st["view"] = PhysxManager.get_physics_sim_view().create_rigid_body_view(f"{st['root']}/g*")
        if st["view"].count != len(st["paths"]):
            raise RuntimeError(f"soil {name}: the view found {st['view'].count} of {len(st['paths'])} grains")
    return st["view"]


def grains(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(centres N x 3 world m, radii N) after the last control step (the start layout before the first)."""
    st = _STATE[name]
    return st["now"], st["radius"]


def grain_orientations(name: str) -> np.ndarray:
    """Current grain orientations as XYZW quaternions, in the same order as grains."""
    return _STATE[name]["quat"]


def velocities(name: str) -> np.ndarray:
    import warp as wp

    return wp.to_torch(_view(name).get_velocities()).cpu().numpy()[:, :3].astype(float)


@bodykit.on_step
def _read(env) -> None:
    import warp as wp

    for name, st in _STATE.items():
        origin = bodykit.as_numpy(env.unwrapped.scene.env_origins[0])
        transforms = wp.to_torch(_view(name).get_transforms()).cpu().numpy().astype(float)
        st["now"] = transforms[:, :3] - origin
        st["quat"] = transforms[:, 3:]
        st["steps"] += 1
        save = os.environ.get("SOIL_SAVE")
        if save and st["steps"] % 20 == 0:
            np.savez(save, pos=st["now"], radius=st["radius"], quat=st["quat"])
            v = np.linalg.norm(velocities(name), axis=1)
            print(f"soilkit: step {st['steps']}: saved {name}; grain speed max {v.max():.4f} m/s, "
                  f"mean z {st['now'][:, 2].mean():.4f}", flush=True)


@bodykit.before_reset
def _put_back() -> None:
    import torch
    import warp as wp

    for name, st in _STATE.items():
        if st["view"] is None:  # the first episode: they are where they were authored
            continue
        view = st["view"]
        n = len(st["paths"])
        pose = torch.zeros((n, 7), dtype=torch.float32, device="cuda:0")
        pose[:, :3] = torch.tensor(st["start"], dtype=torch.float32)
        pose[:, 3:] = torch.tensor(st["start_quat"],dtype=torch.float32)
        idx = wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32, device="cuda:0")
        view.set_transforms(wp.from_torch(pose), indices=idx)
        view.set_velocities(wp.from_torch(torch.zeros((n, 6), dtype=torch.float32, device="cuda:0")), indices=idx)
        st["now"] = st["start"].copy()
        st["steps"] = 0
