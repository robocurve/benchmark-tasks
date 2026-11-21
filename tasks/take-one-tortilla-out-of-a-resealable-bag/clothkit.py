"""clothkit: thin deformable sheets (PhysX surface deformables) for yam_arena task files, with ``bodykit``.

A :class:`Sheet` is one triangle mesh simulated by PhysX's surface-deformable solver (a cloth-like FEM shell with
stretch, shear and bend stiffness): a film of a bag, a tortilla. It is authored straight into USD next to its
``host`` body when that body is spawned (the way ``soilkit`` authors grains), read back after every control step
through one PhysX surface-deformable view per sheet (:func:`points`) and put back at its start shape at every reset.

Two sheets can be joined vertex to vertex (:func:`link`, a PhysX vertex-vertex attachment): the welded seams of a bag,
the cells of its zip. A link can be cut while the episode runs (:func:`cut`) and is restored at every reset. A
sheet's vertices can also be fixed to a rigid body while the episode runs (:func:`hold`, a vertex-xform attachment
authored on the fly) and let go again (:func:`release`): ``gripkit`` uses this for what a finger pad holds by
friction.

Pure Python at import time: everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

import bodykit

Color = tuple[float, float, float]


@dataclass
class Sheet:
    name: str
    host: str  # the Body whose spawning authors this sheet
    points: np.ndarray  # V x 3, env frame (m): the rest shape
    faces: np.ndarray  # F x 3
    color: Color = (0.9, 0.9, 0.9)
    mass: float = 0.01  # kg, the whole sheet
    thickness: float = 0.001  # collision thickness (m): contacts keep the mid-surface half of this from others
    stretch: float = 1.0e4  # PhysX surfaceStretchStiffness
    shear: float = 1.0e3  # PhysX surfaceShearStiffness
    bend: float = 1.0e-3  # PhysX surfaceBendStiffness
    rest_bend: str = "flatDefault"  # or "restShapeDefault": the rest shape's folds are its rest bend angles
    friction: tuple[float, float] = (0.5, 0.5)
    youngs: float = 1.0e6
    poisson: float = 0.3
    self_collision: bool = False
    iterations: int = 32
    damping: float = 0.0  # linear damping (1/s)
    opacity: float = 1.0
    roughness: float = 0.6
    double_sided: bool = True
    uv: np.ndarray | None = None  # V x 2 texture coordinates
    texture: str | None = None  # an image file: its RGB is the colour and, with ``alpha_opacity``, its A the opacity
    alpha_opacity: bool = False
    cast_shadows: bool = True
    looks: list | None = None  # [(face indices, look)]: a look for some faces (see _look); the rest get the base look
    base_look: dict | None = None  # the base look, if not ``texture``/``color``
    start: np.ndarray | None = None  # V x 3 start positions, if not the rest shape (a settled layout, say)


@dataclass
class Link:
    name: str
    sheet0: str
    v0: tuple
    sheet1: str
    v1: tuple
    stiffness: float = float("inf")
    damping: float = 0.0


_SHEETS: dict[str, Sheet] = {}
_LINKS: dict[str, Link] = {}
_STATE: dict[str, dict] = {}
_RUNTIME: dict = {"parent": None, "cut": set(), "holds": {}}


def sheet(*args, **kwargs) -> Sheet:
    s = Sheet(*args, **kwargs)
    s.points = np.asarray(s.points, float)
    s.faces = np.asarray(s.faces, int)
    if s.start is not None:
        s.start = np.asarray(s.start, float)
    _SHEETS[s.name] = s
    return s


def link(*args, **kwargs) -> Link:
    """Join vertices ``v0`` of ``sheet0`` to vertices ``v1`` of ``sheet1`` (pairwise), both sheets on one host."""
    k = Link(*args, **kwargs)
    _LINKS[k.name] = k
    return k


def _author(stage, prim_path: str, spec) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim import schemas
    from isaaclab.sim.spawners.materials.physics_materials import spawn_deformable_body_material
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, Vt

    from yam_arena.station import linear_rgb

    parent = prim_path.rsplit("/", 1)[0]
    _RUNTIME["parent"] = parent
    try:  # clear film is glass: RTX renders it see-through only with translucency on
        import carb

        carb.settings.get_settings().set_bool("/rtx/translucency/enabled", True)
        carb.settings.get_settings().set_bool("/rtx/raytracing/fractionalCutoutOpacity", True)
        import yam_arena.embodiment as E

        gs = E.RENDERER.global_settings
        gs.enable_translucency = True
        gs.carb_settings = {**(gs.carb_settings or {}), "/rtx/raytracing/fractionalCutoutOpacity": True}
    except Exception as e:  # noqa: BLE001 -- rendering only
        print("clothkit: could not enable translucency:", e)
    hosted = [s for s in _SHEETS.values() if s.host == spec.name]
    for s in hosted:
        root = f"{parent}/{s.name}"
        UsdGeom.Xform.Define(stage, root)
        mesh = UsdGeom.Mesh.Define(stage, f"{root}/mesh")
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in s.points]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in s.faces.reshape(-1)]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(s.faces)))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(bool(s.double_sided))
        if not s.cast_shadows:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        if s.uv is not None:
            pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                         UsdGeom.Tokens.vertex)
            pv.Set(Vt.Vec2fArray([Gf.Vec2f(*map(float, t)) for t in s.uv]))
        props = PhysxDeformableBodyPropertiesCfg(
            mass=float(s.mass), self_collision=bool(s.self_collision),
            solver_position_iteration_count=int(s.iterations), linear_damping=float(s.damping),
            max_depenetration_velocity=0.5)
        schemas.define_deformable_body_properties(root, props, stage=stage, deformable_type="surface")
        sim_mesh = stage.GetPrimAtPath(f"{root}/sim_mesh")
        sim_mesh.GetAttribute("omniphysics:restBendAnglesDefault").Set(s.rest_bend)
        col = PhysxSchema.PhysxCollisionAPI.Apply(sim_mesh)
        col.CreateContactOffsetAttr(float(max(0.002, s.thickness)))
        col.CreateRestOffsetAttr(float(s.thickness / 2))
        mat = PhysxSurfaceDeformableBodyMaterialCfg(
            density=float(s.mass / max(1e-9, _area(s.points, s.faces) * s.thickness)),
            static_friction=float(s.friction[0]), dynamic_friction=float(s.friction[1]),
            youngs_modulus=float(s.youngs), poissons_ratio=float(s.poisson), surface_thickness=float(s.thickness),
            surface_stretch_stiffness=float(s.stretch), surface_shear_stiffness=float(s.shear),
            surface_bend_stiffness=float(s.bend))
        spawn_deformable_body_material(f"{root}/physics", mat)
        bind_physics_material(root, f"{root}/physics", stage=stage)
        if s.base_look is not None:
            _look(stage, f"{root}/look", s.base_look)
        elif s.texture:
            _textured_look(stage, f"{root}/look", s)
        else:
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(s.color), roughness=s.roughness,
                                               opacity=float(s.opacity))
            look.func(f"{root}/look", look)
        bind_visual_material(f"{root}/mesh", f"{root}/look", stage=stage)
        for i, (faces, spec_) in enumerate(s.looks or []):
            sub = UsdGeom.Subset.CreateGeomSubset(mesh, f"part{i}", UsdGeom.Tokens.face,
                                                  Vt.IntArray([int(f) for f in faces]), "materialBind")
            _look(stage, f"{root}/look{i}", spec_)
            bind_visual_material(sub.GetPath().pathString, f"{root}/look{i}", stage=stage)
        start = s.points if s.start is None else s.start
        _STATE[s.name] = {"root": root, "view": None, "now": start.copy(), "vel": np.zeros_like(start),
                          "placed": s.start is None}
    names = {s.name for s in hosted}
    for k in _LINKS.values():
        if k.sheet0 in names and k.sheet1 in names:
            _author_link(stage, k)


def _author_link(stage, k: Link) -> None:
    from pxr import Sdf, Vt

    parent = _RUNTIME["parent"]
    path = f"{parent}/{k.sheet0}/link_{k.name}"
    prim = stage.DefinePrim(path, "OmniPhysicsVtxVtxAttachment")
    prim.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(f"{parent}/{k.sheet0}/sim_mesh")])
    prim.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(f"{parent}/{k.sheet1}/sim_mesh")])
    prim.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set(
        Vt.IntArray([int(v) for v in k.v0]))
    prim.CreateAttribute("omniphysics:vtxIndicesSrc1", Sdf.ValueTypeNames.IntArray).Set(
        Vt.IntArray([int(v) for v in k.v1]))
    prim.CreateAttribute("omniphysics:stiffness", Sdf.ValueTypeNames.Float).Set(float(k.stiffness))
    prim.CreateAttribute("omniphysics:damping", Sdf.ValueTypeNames.Float).Set(float(k.damping))
    prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool).Set(True)


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def cut(name: str) -> None:
    """Remove link ``name`` for the rest of the episode (a zip cell pulled open, say)."""
    if name in _RUNTIME["cut"]:
        return
    k = _LINKS[name]
    _stage().RemovePrim(f"{_RUNTIME['parent']}/{k.sheet0}/link_{name}")
    _RUNTIME["cut"].add(name)


def is_cut(name: str) -> bool:
    return name in _RUNTIME["cut"]


def hold(key: str, sheet_name: str, vertices, body_path: str, body_pos, body_quat_xyzw) -> None:
    """Fix ``vertices`` of a sheet where they are now to the rigid body at USD path ``body_path`` (whose pose is
    ``body_pos``/``body_quat_xyzw`` in the env frame), until :func:`release`."""
    from pxr import Gf, Sdf, Vt

    release(key)
    vertices = np.asarray(vertices, int)
    if len(vertices) == 0:
        return
    r = bodykit.rotation(tuple(body_quat_xyzw))
    local = (points(sheet_name)[vertices] - np.asarray(body_pos)) @ r
    path = f"{_RUNTIME['parent']}/{sheet_name}/hold_{key.replace(':', '_')}"
    prim = _stage().DefinePrim(path, "OmniPhysicsVtxXformAttachment")
    prim.CreateRelationship("omniphysics:src0").SetTargets(
        [Sdf.Path(f"{_RUNTIME['parent']}/{sheet_name}/sim_mesh")])
    prim.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(body_path)])
    prim.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set(
        Vt.IntArray([int(v) for v in vertices]))
    prim.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in local]))
    prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    _RUNTIME["holds"][key] = (path, sheet_name, vertices)


def release(key: str) -> None:
    h = _RUNTIME["holds"].pop(key, None)
    if h is not None:
        _stage().RemovePrim(h[0])


def holds() -> dict:
    """{key: (sheet name, vertex indices)} of what is held now."""
    return {k: (v[1], v[2]) for k, v in _RUNTIME["holds"].items()}


def _look(stage, path: str, spec: dict) -> None:
    """A material from a look: {"glass": tint, "frost": 0..1} (thin-walled clear plastic), {"texture": file} or
    {"color": rgb, "roughness": r}."""
    import isaaclab.sim as sim_utils

    from yam_arena.station import linear_rgb

    if "glass" in spec:
        cfg = sim_utils.GlassMdlCfg(glass_color=tuple(spec["glass"]), frosting_roughness=float(spec.get("frost", 0.0)),
                                    thin_walled=True, glass_ior=float(spec.get("ior", 1.5)))
        cfg.func(path, cfg)
    elif "texture" in spec:
        s = Sheet("_", "_", np.zeros((1, 3)), np.zeros((1, 3), int), texture=spec["texture"],
                  roughness=float(spec.get("roughness", 0.5)))
        _textured_look(stage, path, s)
    else:
        cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(tuple(spec["color"])),
                                          roughness=float(spec.get("roughness", 0.5)))
        cfg.func(path, cfg)


def _textured_look(stage, path, s: Sheet) -> None:
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(s.roughness))
    sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set((0.04, 0.04, 0.04))
    reader = UsdShade.Shader.Define(stage, f"{path}/st")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, f"{path}/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(s.texture)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    if s.alpha_opacity:
        tex.CreateOutput("a", Sdf.ValueTypeNames.Float)
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(), "a")
        sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    else:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(s.opacity))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")


def _area(p, f):
    a = p[f[:, 1]] - p[f[:, 0]]
    b = p[f[:, 2]] - p[f[:, 0]]
    return float(0.5 * np.linalg.norm(np.cross(a, b), axis=1).sum())


bodykit.EXTRA_AUTHORS.append(_author)


def _view(name: str):
    st = _STATE[name]
    if st["view"] is None:
        from isaaclab_physx.physics import PhysxManager

        st["view"] = PhysxManager.get_physics_sim_view().create_surface_deformable_body_view(st["root"])
        if st["view"].count != 1:
            raise RuntimeError(f"clothkit {name}: view found {st['view'].count} bodies")
    return st["view"]


def points(name: str) -> np.ndarray:
    """The sheet's vertex positions (V x 3, env frame) after the last control step."""
    return _STATE[name]["now"]


def velocities(name: str) -> np.ndarray:
    return _STATE[name]["vel"]


def names() -> list[str]:
    return list(_SHEETS)


def get(name: str) -> Sheet:
    return _SHEETS[name]


@bodykit.on_step
def _read(env) -> None:
    import warp as wp

    origin = bodykit.as_numpy(env.unwrapped.scene.env_origins[0])
    for name, st in _STATE.items():
        v = _view(name)
        n = len(_SHEETS[name].points)
        if not st["placed"]:  # the first step of the first episode: move to the start layout
            _place(name)
            st["placed"] = True
        pos = wp.to_torch(v.get_simulation_nodal_positions()).cpu().numpy().reshape(-1, 3)[:n].astype(float)
        vel = wp.to_torch(v.get_simulation_nodal_velocities()).cpu().numpy().reshape(-1, 3)[:n].astype(float)
        st["now"] = pos - origin
        st["vel"] = vel


def _place(name: str) -> None:
    import torch
    import warp as wp

    st = _STATE[name]
    v = _view(name)
    s = _SHEETS[name]
    start = s.points if s.start is None else s.start
    nmax = v.max_simulation_nodes_per_body
    pos = np.zeros((1, nmax, 3), np.float32)
    pos[0, :len(start)] = start  # env 0 sits at the world origin in these tasks
    pos[0, len(start):] = start[-1]
    idx = wp.array(np.arange(1, dtype=np.int32), dtype=wp.int32, device="cuda:0")
    v.set_simulation_nodal_positions(wp.from_torch(torch.tensor(pos, device="cuda:0")), indices=idx)
    v.set_simulation_nodal_velocities(wp.from_torch(torch.zeros((1, nmax, 3), device="cuda:0")), indices=idx)
    st["now"] = start.copy()


@bodykit.before_reset
def _put_back() -> None:
    if _RUNTIME["parent"] is None:
        return
    for key in list(_RUNTIME["holds"]):
        release(key)
    stage = _stage()
    for name in list(_RUNTIME["cut"]):
        _author_link(stage, _LINKS[name])
    _RUNTIME["cut"].clear()
    for name, st in _STATE.items():
        if st["view"] is None:
            continue
        _place(name)


# -- meshes ----------------------------------------------------------------------------------------------------------

def disc(radius: float, edge: float, centre=(0.0, 0.0, 0.0)):
    """A flat disc in the xy plane: (points, faces, uv). Rings of vertices, about ``edge`` apart."""
    rings = max(2, int(round(radius / edge)))
    pts = [np.zeros(3)]
    uv = [(0.5, 0.5)]
    faces = []
    prev = [0]
    for k in range(1, rings + 1):
        r = radius * k / rings
        n = max(6, int(round(2 * np.pi * r / edge)))
        idx = list(range(len(pts), len(pts) + n))
        ang0 = (k % 2) * np.pi / n
        for i in range(n):
            a = ang0 + 2 * np.pi * i / n
            pts.append(np.array([r * np.cos(a), r * np.sin(a), 0.0]))
            uv.append((0.5 + 0.5 * r / radius * np.cos(a), 0.5 + 0.5 * r / radius * np.sin(a)))
        faces += _stitch_rings(prev, idx, pts)
        prev = idx
    p = np.array(pts) + np.asarray(centre, float)
    return p, np.array(faces, int), np.array(uv)


def _stitch_rings(inner, outer, pts):
    if len(inner) == 1:
        c = inner[0]
        return [(c, outer[i], outer[(i + 1) % len(outer)]) for i in range(len(outer))]
    P = np.array(pts)
    ai = np.arctan2(P[inner, 1], P[inner, 0]) % (2 * np.pi)
    ao = np.arctan2(P[outer, 1], P[outer, 0]) % (2 * np.pi)
    oi, oo = np.argsort(ai), np.argsort(ao)
    inner = [inner[i] for i in oi]
    outer = [outer[i] for i in oo]
    ai, ao = np.sort(ai), np.sort(ao)
    ni, no = len(inner), len(outer)
    faces = []
    i = j = 0
    while i < ni or j < no:
        ni_ang = ai[i + 1] if i + 1 < ni else ai[0] + 2 * np.pi
        no_ang = ao[j + 1] if j + 1 < no else ao[0] + 2 * np.pi
        if j >= no or (i < ni and ni_ang < no_ang):
            faces.append((inner[i % ni], outer[j % no], inner[(i + 1) % ni]))
            i += 1
        else:
            faces.append((inner[i % ni], outer[j % no], outer[(j + 1) % no]))
            j += 1
    return faces
