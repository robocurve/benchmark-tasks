"""clothkit: fabric as PhysX surface deformables (FEM cloth shells), for yam_arena task files with ``bodykit``.

A :class:`Sheet` is one triangle mesh simulated by PhysX's surface-deformable solver, with gravity, in-plane
stretch and shear, bending, friction and self-collision: a T-shirt, a washcloth. It is authored straight into USD
when its ``host`` body is spawned, with its unstressed ``rest`` shape and its starting shape ``start`` (a washcloth
starts folded over its rail; its rest shape is flat). Its vertex positions are read back after every control step
(:func:`points`) through one PhysX surface-deformable view per sheet, and put back to ``start`` at every reset, before
the arms settle.

No software ever moves a vertex during an episode; the only coupling to the arms is ``pinch`` (see pinch.py).
Pure Python at import time: everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import bodykit

Color = tuple[float, float, float]


@dataclass
class Sheet:
    name: str
    host: str  # the Body whose spawning authors this sheet
    rest: np.ndarray  # V x 3 (env frame): unstressed shape
    start: np.ndarray  # V x 3 (env frame): starting shape
    faces: np.ndarray  # F x 3
    face_kind: np.ndarray  # F: index into ``colors``
    colors: tuple  # one colour per face kind
    mass: float  # kg
    thickness: float  # m, the shell thickness (collision rest offset is half of it)
    youngs: float = 5.0e5  # Pa
    poisson: float = 0.3
    bend: float = 0.0  # PhysX surfaceBendStiffness
    friction: tuple[float, float] = (0.6, 0.5)
    iterations: int = 32
    self_collision: bool = True
    roughness: float = 0.9


SHEETS: dict[str, Sheet] = {}
_STATE: dict[str, dict] = {}


def sheet(**kw) -> Sheet:
    s = Sheet(**kw)
    s.rest, s.start = np.asarray(s.rest, float), np.asarray(s.start, float)
    s.faces, s.face_kind = np.asarray(s.faces, int), np.asarray(s.face_kind, int)
    if s.rest.shape != s.start.shape:
        raise ValueError(f"{s.name}: rest and start shapes differ in size")
    SHEETS[s.name] = s
    return s


def _area(p, f) -> float:
    return float(0.5 * np.linalg.norm(np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]]), axis=1).sum())


def _author(stage, prim_path: str, spec) -> None:
    from omni.physx.scripts import deformableUtils, physicsUtils
    from pxr import Gf, PhysxSchema, UsdGeom, UsdShade

    import isaaclab.sim as sim_utils
    from yam_arena.station import linear_rgb

    parent = prim_path.rsplit("/", 1)[0]
    for s in SHEETS.values():
        if s.host != spec.name:
            continue
        path = f"{parent}/{s.name}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in s.rest])
        mesh.CreateFaceVertexCountsAttr([3] * len(s.faces))
        mesh.CreateFaceVertexIndicesAttr([int(v) for v in s.faces.ravel()])
        mesh.CreateSubdivisionSchemeAttr().Set("none")
        mesh.CreateDoubleSidedAttr(True)
        if not deformableUtils.set_physics_surface_deformable_body(stage, path):
            raise RuntimeError(f"could not make {path} a surface deformable")
        mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float, q)) for q in s.start])  # the rest shape stays as authored
        prim = mesh.GetPrim()
        prim.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
        for name, value in (("selfCollision", bool(s.self_collision)),
                            ("solverPositionIterationCount", int(s.iterations)),
                            ("collisionPairUpdateFrequency", 1), ("collisionIterationMultiplier", 2.0),
                            ("enableSpeculativeCCD", True), ("maxDepenetrationVelocity", 0.5)):
            attr = prim.GetAttribute(f"physxDeformableBody:{name}")
            if attr:
                attr.Set(value)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.GetRestOffsetAttr().Set(float(s.thickness / 2))
        col.GetContactOffsetAttr().Set(float(max(0.003, s.thickness)))
        mat = path + "_physics"
        deformableUtils.add_surface_deformable_material(
            stage, mat, density=s.mass / (_area(s.rest, s.faces) * s.thickness), static_friction=s.friction[0],
            dynamic_friction=s.friction[1], youngs_modulus=s.youngs, poissons_ratio=s.poisson,
            surface_thickness=s.thickness, surface_bend_stiffness=s.bend)
        stage.GetPrimAtPath(mat).ApplyAPI("PhysxSurfaceDeformableMaterialAPI")
        physicsUtils.add_physics_material_to_prim(stage, prim, mat)
        for k, color in enumerate(s.colors):
            look = f"{path}_look{k}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=s.roughness, metallic=0.0)
            cfg.func(look, cfg)
            sub = UsdGeom.Subset.CreateGeomSubset(mesh, f"kind{k}", UsdGeom.Tokens.face,
                                                  [int(i) for i in np.nonzero(s.face_kind == k)[0]], "materialBind")
            UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(UsdShade.Material.Get(stage, look))
        UsdShade.MaterialBindingAPI.Apply(prim)
        _STATE[s.name] = {"path": path, "view": None, "now": s.start.copy(), "origin": np.zeros(3)}
    if SHEETS and all(n in _STATE for n in SHEETS):
        for prim in stage.Traverse():
            if prim.GetAttribute("physxScene:gpuMaxDeformableSurfaceContacts"):
                prim.GetAttribute("physxScene:gpuMaxDeformableSurfaceContacts").Set(4 * 1048576)


bodykit.EXTRA_AUTHORS.append(_author)


def _view(name: str):
    st = _STATE[name]
    if st["view"] is None:
        from isaaclab_physx.physics import PhysxManager

        st["view"] = PhysxManager.get_physics_sim_view().create_surface_deformable_body_view(st["path"])
        if st["view"].count != 1:
            raise RuntimeError(f"clothkit {name}: the view found {st['view'].count} bodies")
    return st["view"]


def path(name: str) -> str:
    return _STATE[name]["path"]


def points(name: str) -> np.ndarray:
    """The sheet's vertex positions (env frame, V x 3) after the last control step."""
    return _STATE[name]["now"]


@bodykit.on_step
def _read(env) -> None:
    import warp as wp

    origin = bodykit.as_numpy(env.unwrapped.scene.env_origins[0])
    for name, st in _STATE.items():
        n = len(SHEETS[name].start)
        raw = wp.to_torch(_view(name).get_simulation_nodal_positions()).cpu().numpy().reshape(-1, 3)[:n]
        st["now"] = raw.astype(float) - origin
        st["origin"] = origin


@bodykit.before_reset
def _put_back() -> None:
    import torch
    import warp as wp

    for name, st in _STATE.items():
        if st["view"] is None:  # the first episode: still where it was authored
            continue
        v, s = st["view"], SHEETS[name]
        nmax = v.max_simulation_nodes_per_body
        pos = np.zeros((1, nmax, 3), np.float32)
        pos[0, :len(s.start)] = s.start + st["origin"]
        pos[0, len(s.start):] = pos[0, len(s.start) - 1]
        idx = wp.array(np.zeros(1, np.int32), dtype=wp.int32, device="cuda:0")
        v.set_simulation_nodal_positions(wp.from_torch(torch.tensor(pos, device="cuda:0")), idx)
        v.set_simulation_nodal_velocities(wp.from_torch(torch.zeros((1, nmax, 3), device="cuda:0")), idx)
        st["now"] = s.start.copy()
