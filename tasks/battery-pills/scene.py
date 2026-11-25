"""battery-pills USD authoring (Isaac only, imported after the simulator starts) and the physics constants.

Bodies (no attachments, fixed joints or kinematic objects; every contact is native PhysX, CPU-side rigid bodies on the
GPU-dynamics scene the YAM station uses):
- ``tray``   one dynamic rigid body: floor slab + 4 walls (boxes), black melamine, 900 g (estimate)
- ``dish``   one dynamic rigid body: foot frustum + 32 flared wall segments, blue melamine, 61 g (listing upper bound)
- ``jar``    one dynamic rigid body: floor disc + 32 wall segments, clear plastic rendered pale with a white band, 91 g
- ``cell``   one dynamic rigid body: 24-sided prism 7.9 x 3.6 mm, steel grey, 0.8 g (estimate)
- ``tablet0..4`` dynamic rigid bodies: 24-sided prisms 9.5 x 4.0 mm, white, 0.35 g each (estimate)
Every collider is a convex hull of its piece's points (PhysX convexHull approximation). Rest offset 0 everywhere;
contact offsets 0.5 mm (cell, tablets) and 1 mm (tray, dish, jar) only start contact generation early.
Friction coefficients have no published value for these material pairs: estimates, listed in SOURCED_VALUES.md.
"""
import numpy as np

import geometry as G

PHYSICS_HZ = 240
SPAWN_GAP = 0.0001            # bodies authored 0.1 mm above what they rest on
MU = dict(tray=0.5, dish=0.5, jar=0.4, cell=0.4, tablet=0.5)        # estimates (no published value)
CONTACT_OFFSET = dict(tray=0.001, dish=0.001, jar=0.001, cell=0.0005, tablet=0.0005)
REST_OFFSET = 0.0
ITERATIONS = dict(tray=16, dish=32, jar=16, cell=32, tablet=32)
COLORS = dict(tray=(0.06, 0.06, 0.065), dish=(0.12, 0.30, 0.72), jar=(0.86, 0.88, 0.90), band=(0.97, 0.97, 0.96),
              cell=(0.47, 0.48, 0.50), tablet=(0.97, 0.97, 0.96), label=(0.05, 0.05, 0.05))
MASS_G = dict(tray=G.TRAY_MASS_G, dish=G.DISH_MASS_G, jar=G.JAR_MASS_G, cell=G.CELL_MASS_G, tablet=G.TAB_MASS_G)


def kind_of(name):
    return 'tablet' if name.startswith('tablet') else name


def pieces_for(kind):
    if kind == 'tray':
        return G.tray_pieces()
    if kind == 'dish':
        return G.dish_pieces()
    if kind == 'jar':
        return G.jar_pieces()
    if kind == 'cell':
        return [('body', G.prism_points(G.CELL_R, G.CELL_H))]
    if kind == 'tablet':
        return [('body', G.prism_points(G.TAB_R, G.TAB_H))]
    raise KeyError(kind)


def linear_rgb(srgb):
    return tuple(c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4 for c in srgb)


def _xform(stage, path, translation):
    from pxr import Gf, UsdGeom
    x = UsdGeom.Xform.Define(stage, path)
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in translation]))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    x.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))
    return x


def _physics_material(stage, path, mu):
    from pxr import UsdPhysics, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(mu))
    api.CreateDynamicFrictionAttr(float(mu))
    api.CreateRestitutionAttr(0.0)
    return mat


def _look(stage, path, rgb, roughness=0.5, metallic=0.0, opacity=None):
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + '/Shader')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(rgb)))
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(metallic))
    if opacity is not None:
        sh.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat


def _hull_mesh(stage, path, pts):
    """A UsdGeom.Mesh of the convex hull of ``pts`` (render faces from trimesh), collider = convexHull."""
    import trimesh
    from pxr import Gf, UsdGeom, Vt
    hull = trimesh.convex.convex_hull(np.asarray(pts, float))
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in hull.vertices]))
    m.CreateFaceVertexCountsAttr([3] * len(hull.faces))
    m.CreateFaceVertexIndicesAttr([int(k) for f in hull.faces for k in f])
    m.CreateSubdivisionSchemeAttr('none')
    return m


def author_body(stage, path, translation, name):
    """One dynamic rigid body of the given kind at ``translation`` (world, its body-frame origin). Returns the prim."""
    from pxr import PhysxSchema, UsdPhysics, UsdShade
    kind = kind_of(name)
    root = _xform(stage, path, translation)
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    r = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    r.CreateSolverPositionIterationCountAttr(int(ITERATIONS[kind]))
    r.CreateSolverVelocityIterationCountAttr(4)
    r.CreateMaxDepenetrationVelocityAttr(0.2)
    r.CreateEnableCCDAttr(False)
    r.CreateSleepThresholdAttr(0.0)
    r.CreateStabilizationThresholdAttr(0.0)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(MASS_G[kind] * 1e-3))
    phys = _physics_material(stage, path + '/Phys', MU[kind])
    look = _look(stage, path + '/Look', COLORS[kind], roughness=0.35 if kind == 'cell' else 0.55,
                 metallic=0.6 if kind == 'cell' else 0.0)
    band = _look(stage, path + '/LookBand', COLORS['band'], roughness=0.8) if kind == 'jar' else None
    for pname, pts in pieces_for(kind):
        m = _hull_mesh(stage, f'{path}/{pname}', pts)
        cp = m.GetPrim()
        UsdPhysics.CollisionAPI.Apply(cp)
        c = PhysxSchema.PhysxCollisionAPI.Apply(cp)
        c.CreateContactOffsetAttr(float(CONTACT_OFFSET[kind]))
        c.CreateRestOffsetAttr(float(REST_OFFSET))
        UsdPhysics.MeshCollisionAPI.Apply(cp).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI.Apply(cp).Bind(phys, UsdShade.Tokens.weakerThanDescendants, 'physics')
        UsdShade.MaterialBindingAPI(cp).Bind(look)
    if kind == 'jar':
        _jar_band(stage, path, band)
    return prim


def _jar_band(stage, path, band):
    """Render-only white paper band (100 mm tall, estimate) round the jar body and the USED / BATTERIES label patch:
    visual meshes with no collider. The label's lettering is not drawn in sim yet (see README: remaining)."""
    from pxr import Gf, UsdGeom, UsdShade, Vt
    n = 48
    lo, hi = 70.0, 170.0                       # band centred on the 120 mm label height, 100 mm tall (estimate)
    r = G.JAR_OUT_R / np.cos(np.pi / G.JAR_SEGMENTS) + 0.3
    pts = G.disc_points(r, r, lo, hi, n)       # n bottom points then n top points: an open tube (side quads only)
    m = UsdGeom.Mesh.Define(stage, f'{path}/band_visual')
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in pts]))
    m.CreateFaceVertexCountsAttr([4] * n)
    m.CreateFaceVertexIndicesAttr([k for i in range(n) for k in (i, (i + 1) % n, n + (i + 1) % n, n + i)])
    m.CreateSubdivisionSchemeAttr('none')
    m.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(band)
