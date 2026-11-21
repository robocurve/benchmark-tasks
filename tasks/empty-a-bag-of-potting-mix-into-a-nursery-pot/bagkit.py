"""bagkit: the opened bag of potting mix (one sheet of PhysX film) and the nursery pot. Pure Python at import.

The bag is a PhysX surface deformable (thin cloth-like film): the triangle mesh of ``clothbag`` with polyethylene-like
stretch stiffness and almost no bending stiffness, so it holds the mix like a bag, folds where it is empty and
narrows as it empties. The two front corner seals where the hands hold it are two small rigid tabs (bodykit Bodies,
``TAB_BODIES``), each fixed to the film vertices it encloses by a PhysX deformable attachment.

The film is authored when the second tab spawns, in its filled starting shape (``bag_start.npz``, made by
``prepare_bag.py``), and put back into that shape at every reset (before the arms settle), as soilkit puts back the
mix. :func:`nodes` gives the film's vertex positions after the last control step.

The pot is one free rigid body: a floor, a ring of tapered wall plates and a rolled rim.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import bagplan as p
import bodykit
import clothbag as cb
from bodykit import Body, BoxShape, CylShape, plate

FILM = (0.13, 0.36, 0.17)  # printed green film
PRINT = (0.93, 0.89, 0.78)  # cream label panel
BAND = (0.55, 0.33, 0.16)  # brown printed band (a soil picture)
SEAL = (0.10, 0.30, 0.14)
POT_BLACK = (0.04, 0.04, 0.045)
bodykit.FINISH[FILM] = (0.35, 0.0)
bodykit.FINISH[PRINT] = (0.45, 0.0)
bodykit.FINISH[POT_BLACK] = (0.55, 0.0)

# -- the film ----------------------------------------------------------------------------------------------------------
FILM_E = 2.0e8  # Pa: low-density polyethylene
FILM_T = 1.0e-4  # m: 0.1 mm film
FILM_POISSON = 0.4
FILM_FRICTION = (0.35, 0.30)  # smooth PE film on damp peat
FILM_ITERS = 64  # solver iterations: the film carries 0.8 kg of grains against its own weight
FILM_COLLISION_MULT = 4.0
FILM_REST = p.REST  # contact rest offset (m): the modelled collision thickness of the film
FILM_CONTACT = 0.006
TAB_MASS = 0.020
TAB_FRICTION = (1.5, 1.3)  # a gripper pad on a pinched gather of film

START_FILE = Path(__file__).with_name("bag_start.npz")
FILM_NAME = "bag_film"

_PTS, _FACES, _INFO = cb.mesh()
MOUTH = cb.mouth_rows(_INFO)


def film_density() -> float:
    """kg/m^3 that gives the film ``p.FILM_MASS`` at thickness ``FILM_T``."""
    tri = _PTS[_FACES]
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1).sum()
    return p.FILM_MASS / (area * FILM_T)


def tab_body(name: str, origin=p.BAG_AT) -> Body:
    c, size, yaw = cb.tab_box(name, _PTS, _INFO)
    return Body.build(name, pos=tuple(np.asarray(origin) + c), mass=TAB_MASS,
                      shapes=[BoxShape((0, 0, 0), tuple(size), SEAL, yaw=yaw, friction=TAB_FRICTION)],
                      friction=TAB_FRICTION, solver_iterations=32, solver_velocity_iterations=4, contact_offset=0.002)


TAB_BODIES = [tab_body(name) for name, *_ in cb.TABS]


def start_nodes(origin=p.BAG_AT) -> np.ndarray:
    """The film's starting vertex positions (world): the saved filled shape, else the rest shape."""
    if START_FILE.is_file():
        return np.load(START_FILE)["nodes"] + np.asarray(origin)
    return _PTS + np.asarray(origin)


def _face_colors() -> np.ndarray:
    """Print on the front panel: a cream label above a brown soil-picture band; the rest green."""
    c = _PTS[_FACES].mean(axis=1)
    out = np.zeros(len(_FACES), int)
    front = (c[:, 0] < -0.012) & (c[:, 2] > 0.0005)
    out[front & (c[:, 2] > 0.012) & (c[:, 2] < 0.040)] = 2
    out[front & (c[:, 2] >= 0.048) & (c[:, 2] < 0.100) & (np.abs(c[:, 1]) < 0.055)] = 1
    return out


def author_film(stage, path: str, rest_world: np.ndarray, start_world: np.ndarray, tab_paths=(), looks=True) -> None:
    """Author the film mesh at ``path``: rest shape ``rest_world``, current shape ``start_world`` (both world, V x 3),
    its material, and an attachment to each rigid body in ``tab_paths`` (fixing the film vertices inside it)."""
    from omni.physx.scripts import deformableUtils, physicsUtils
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdShade

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in rest_world])
    mesh.CreateFaceVertexCountsAttr([3] * len(_FACES))
    mesh.CreateFaceVertexIndicesAttr([int(v) for v in _FACES.ravel()])
    # Rendered as a smooth surface (the renderer subdivides the triangles); physics uses the triangles as they are.
    mesh.CreateSubdivisionSchemeAttr().Set("catmullClark")
    mesh.CreateDoubleSidedAttr(True)
    if not deformableUtils.set_physics_surface_deformable_body(stage, path):
        raise RuntimeError(f"could not make {path} a surface deformable")
    mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float, q)) for q in start_world])  # rest shape stays as authored
    prim = mesh.GetPrim()
    prim.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
    for name, value in (("selfCollision", True), ("solverPositionIterationCount", FILM_ITERS),
                        ("collisionPairUpdateFrequency", 1), ("collisionIterationMultiplier", FILM_COLLISION_MULT),
                        ("enableSpeculativeCCD", True)):
        prim.GetAttribute(f"physxDeformableBody:{name}").Set(value)
    collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.GetRestOffsetAttr().Set(FILM_REST)
    collision.GetContactOffsetAttr().Set(FILM_CONTACT)
    mat = path + "_physics"
    deformableUtils.add_surface_deformable_material(stage, mat, density=film_density(),
                                                    static_friction=FILM_FRICTION[0],
                                                    dynamic_friction=FILM_FRICTION[1], youngs_modulus=FILM_E,
                                                    poissons_ratio=FILM_POISSON, surface_thickness=FILM_T)
    stage.GetPrimAtPath(mat).ApplyAPI("PhysxSurfaceDeformableMaterialAPI")
    physicsUtils.add_physics_material_to_prim(stage, prim, mat)
    if looks:
        import isaaclab.sim as sim_utils

        from yam_arena.station import linear_rgb

        kinds = _face_colors()
        for k, color in enumerate((FILM, PRINT, BAND)):
            look = f"{path}_look{k}"
            rough, metal = bodykit.FINISH.get(color, (0.5, 0.0))
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            cfg.func(look, cfg)
            subset = UsdGeom.Subset.CreateGeomSubset(mesh, f"print{k}", UsdGeom.Tokens.face,
                                                     [int(i) for i in np.nonzero(kinds == k)[0]],
                                                     "materialBind")
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(UsdShade.Material.Get(stage, look))
        UsdShade.MaterialBindingAPI.Apply(prim)
    for tab in tab_paths:
        att = Sdf.Path(f"{path}_to_{tab.rstrip('/').split('/')[-2 if tab.endswith('/body') else -1]}")
        if not deformableUtils.create_auto_deformable_attachment(stage, att, Sdf.Path(path), Sdf.Path(tab)):
            raise RuntimeError(f"could not attach {path} to {tab}")


def set_deformable_contacts(stage, count: int = 4 * 1048576) -> None:
    from pxr import PhysxSchema, UsdPhysics

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            PhysxSchema.PhysxSceneAPI.Apply(prim)
            attr = prim.GetAttribute("physxScene:gpuMaxDeformableSurfaceContacts")
            if attr:
                attr.Set(count)


_STATE: dict = {"path": None, "view": None, "now": None, "start": None, "origin": np.zeros(3)}


def _author(stage, prim_path: str, spec) -> None:
    """When the last tab spawns, author the film next to it, attached to both tabs."""
    if spec.name != TAB_BODIES[-1].name:
        return
    parent = prim_path.rsplit("/", 1)[0]
    path = f"{parent}/{FILM_NAME}"
    start = start_nodes()
    author_film(stage, path, _PTS + p.BAG_AT, start, [f"{parent}/{b.name}/body" for b in TAB_BODIES])
    set_deformable_contacts(stage)
    _STATE.update(path=path, view=None, now=start.copy(), start=start.copy())


bodykit.EXTRA_AUTHORS.append(_author)


def _view():
    if _STATE["view"] is None:
        from isaaclab_physx.physics import PhysxManager

        _STATE["view"] = PhysxManager.get_physics_sim_view().create_surface_deformable_body_view(_STATE["path"])
        if _STATE["view"].count != 1:
            raise RuntimeError(f"bagkit: the film view found {_STATE['view'].count} films")
    return _STATE["view"]


def nodes() -> np.ndarray:
    """The film's vertex positions (env frame, V x 3) after the last control step (the start before the first)."""
    return _STATE["now"]


@bodykit.on_step
def _read(env) -> None:
    import warp as wp

    origin = bodykit.as_numpy(env.unwrapped.scene.env_origins[0])
    _STATE["origin"] = origin
    _STATE["now"] = wp.to_torch(_view().get_simulation_nodal_positions()).cpu().numpy()[0].astype(float) - origin


@bodykit.before_reset
def _put_back() -> None:
    import torch
    import warp as wp

    if _STATE["view"] is None:  # the first episode: it is where it was authored
        return
    view = _STATE["view"]
    dev = "cuda:0"
    pos = torch.tensor((_STATE["start"] + _STATE["origin"])[None], dtype=torch.float32, device=dev)
    idx = wp.array(np.zeros(1, np.int32), dtype=wp.int32, device=dev)
    view.set_simulation_nodal_positions(wp.from_torch(pos), idx)
    view.set_simulation_nodal_velocities(wp.from_torch(torch.zeros_like(pos)), idx)
    _STATE["now"] = _STATE["start"].copy()


def in_bag(points, film=None) -> np.ndarray:
    """Which points (env frame, N x 3) are inside the bag: inside the film closed by a lid across the mouth."""
    film = nodes() if film is None else film
    points = np.asarray(points, float)
    lo, hi = film.min(axis=0) - 1e-3, film.max(axis=0) + 1e-3
    out = np.zeros(len(points), bool)
    near = np.nonzero(((points > lo) & (points < hi)).all(axis=1))[0]
    if len(near):
        out[near] = cb.inside(points[near], film, _FACES, MOUTH)
    return out


# -- the pot -----------------------------------------------------------------------------------------------------------


def pot_body(segments: int = 24) -> Body:
    """The black 1-gallon nursery pot: base at ``POT_AT``, open top, tapered wall, rolled rim, drain-ring foot."""
    shapes = []
    r0, r1, h, t = p.POT_R_BASE, p.POT_R_TOP, p.POT_H, p.POT_WALL
    shapes.append(CylShape((0, 0, 0.006), r0 - 0.004, 0.004, POT_BLACK))  # floor, raised on a foot ring
    shapes.append(CylShape((0, 0, 0.002), r0 - 0.006, 0.004, None))  # foot (collision only, hidden under it)
    for i in range(segments):
        a = 2 * math.pi * (i + 0.5) / segments
        c, s = math.cos(a), math.sin(a)
        width = 2 * math.tan(math.pi / segments) * r1 + 0.002
        lo = np.array([c * (r0 - t / 2), s * (r0 - t / 2), 0.004])
        hi = np.array([c * (r1 - t / 2), s * (r1 - t / 2), h - 0.004])
        shapes.append(plate(lo, hi, width, t, (c, s, 0.0), POT_BLACK))
        rim = np.array([c * (r1 + 0.001), s * (r1 + 0.001), h - 0.003])
        shapes.append(plate(rim - [0, 0, 0.004], rim + [0, 0, 0.003], width + 0.001, p.POT_RIM, (c, s, 0.0),
                            POT_BLACK))
    return Body.build("pot", pos=tuple(p.POT_AT), mass=p.POT_MASS, shapes=shapes, friction=(0.7, 0.6),
                      solver_iterations=16)
