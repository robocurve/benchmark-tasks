"""Native stemware and folded microfiber geometry with removable water marks."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .usd_scene import Writer

GLASS = (.72, .85, .90)
CLOTH = (.20, .49, .53)
BINDING = (.125, .29, .32)
MAT = (.34, .265, .18)


def bench(w):
    s = w.stage("bench")
    w.box(s, "/Asset/Top", (1.06, 1.18, .055), (.343, 0., C.TABLE - .0275), MAT)
    for i, x in enumerate((-.15, .82)):
        for j, y in enumerate((-.53, .53)):
            w.box(s, f"/Asset/Leg{i}{j}", (.055, .055, .69), (x, y, .345), (.11, .13, .14))
    w.box(s, "/Asset/Floor", (8., 8., .020), (0., 0., -.010), (.24, .26, .26))
    return s


def glass_material(w, s):
    from pxr import Sdf

    mat = w.Shade.Material.Define(s, "/Asset/GlassMaterial")
    shader = w.Shade.Shader.Define(s, "/Asset/GlassMaterial/Surface")
    shader.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    shader.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(.97, .99, 1.))
    shader.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.45)
    shader.CreateInput("frosting_roughness", Sdf.ValueTypeNames.Float).Set(.045)
    shader.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return mat


def glazed(w, geom, material):
    from pxr import Sdf

    p = geom.GetPrim()
    w.Shade.MaterialBindingAPI.Apply(p).Bind(material)
    p.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)


def mesh(w, s, path, points, faces, color, collide=False):
    m = w.Geom.Mesh.Define(s, path)
    m.CreatePointsAttr([w.Gf.Vec3f(*map(float, p)) for p in points])
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    if collide:
        w.Phy.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return w._finish(s, m, color, collide)


def bowl_mesh(w, s, root, material):
    rods = 48
    outer = list(zip(C.PROFILE_Z, C.PROFILE_R))
    inner = [(z + (.004 if i == 0 else 0), max(.004, r - C.WALL))
             for i, (z, r) in enumerate(outer)]
    profile = outer + list(reversed(inner))
    points = [(r * cos(2 * pi * j / rods), r * sin(2 * pi * j / rods), z)
              for z, r in profile for j in range(rods)]
    faces = [list(reversed(range(rods)))]
    for i in range(len(profile) - 1):
        for j in range(rods):
            k = (j + 1) % rods
            faces.append([i * rods + j, i * rods + k,
                          (i + 1) * rods + k, (i + 1) * rods + j])
    faces.append(list(range((len(profile) - 1) * rods, len(points))))
    glazed(w, mesh(w, s, root + "/OpenRimBowl", points, faces, GLASS), material)

    # Short convex wedges preserve the concave wall and the change in bowl curvature.
    segments = 24
    for i, (z, r) in enumerate(outer[:-1]):
        nz, nr = outer[i + 1]
        for j in range(segments):
            angles = [2 * pi * (j + k) / segments for k in (0, 1)]
            wedge = [(rr * cos(a), rr * sin(a), zz)
                     for zz, radius in ((z, r), (nz, nr))
                     for rr in (radius, radius - C.WALL)
                     for a in angles]
            collider = mesh(w, s, f"{root}/Wall_{i}_{j}", wedge,
                            [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1],
                             [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]],
                            GLASS, True)
            collider.CreateVisibilityAttr(w.Geom.Tokens.invisible)


def water_marks(w, s, root):
    names = []
    for i, (deg, height) in enumerate(zip(C.MARK_DEGREES, C.MARK_HEIGHTS)):
        center = pi + np.deg2rad(C.CLOTH_YAW + deg)
        name = f"WaterMark{i}"
        names.append(name)
        ring_points = []
        for radius_scale in (.70, 1.):
            for j in range(20):
                a = 2 * pi * j / 20
                roughness = 1 + .095 * cos(3 * a + i)
                offset = C.MARK_RADIUS * roughness * radius_scale
                z = height + offset * sin(a)
                r = np.interp(z, C.PROFILE_Z, C.PROFILE_R) + .0004
                theta = center + offset * cos(a) / r
                ring_points.append((r * cos(theta), r * sin(theta), z))
        faces = [[j, j + 20, ((j + 1) % 20) + 20, (j + 1) % 20] for j in range(20)]
        mark = mesh(w, s, root + "/" + name, ring_points, faces, (.86, .91, .85))
        mark.CreateDoubleSidedAttr(True)
    return names


def stemware(w):
    s = w.stage("glass")
    body = w.body(s, "/Asset/Stemware", .142, ccd=True)
    material = glass_material(w, s)
    w.cylinder(s, body + "/Foot", .035, .0035, (0., 0., .00175), GLASS)
    w.cylinder(s, body + "/Stem", C.STEM_RADIUS, .084, (0., 0., .044), GLASS)
    for part in ("Foot", "Stem"):
        glazed(w, w.Geom.Gprim.Get(s, f"{body}/{part}"), material)
    bowl_mesh(w, s, body, material)
    names = water_marks(w, s, body)
    return s, names


def cloth(w):
    s = w.stage("cloth")
    body = w.body(s, "/Asset/FoldedCloth", .026, articulation=True, ccd=True)
    f = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    f.CreateStaticFrictionAttr(.84)
    f.CreateDynamicFrictionAttr(.63)
    w.rounded_box(s, body + "/Layers", (.089, .064, .008), (-.015, 0., .004),
                  CLOTH, .011, .002)
    w.rounded_box(s, body + "/PinchedFold", (.052, .024, .024), (-.026, 0., .017),
                  CLOTH, .009, .003)
    for lateral in (-.027, .027):
        n = "A" if lateral > 0 else "B"
        w.box(s, body + "/FinishedHem" + n, (.078, .002, .001),
              (-.020, lateral, .009), BINDING, collide=False)
    pad = w.body(s, "/Asset/WorkingFold", .007, ccd=True)
    w.rounded_box(s, pad + "/WipeFace", (.030, C.PAD_WIDTH, C.PAD_HEIGHT),
                  (.040, 0., C.PAD_FACE[2]), CLOTH, .006, .002)
    j = w.Phy.PrismaticJoint.Define(s, "/Asset/FoldCompression")
    j.CreateBody0Rel().SetTargets([body])
    j.CreateBody1Rel().SetTargets([pad])
    j.CreateAxisAttr("X")
    j.CreateCollisionEnabledAttr(False)
    j.CreateLowerLimitAttr(-.007)
    j.CreateUpperLimitAttr(.0006)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(C.FLEX_K)
    drive.CreateDampingAttr(15.)
    drive.CreateMaxForceAttr(30.)
    drive.CreateTargetPositionAttr(0.)
    return s


def create_assets(directory):
    w = Writer(directory)
    glass, marks = stemware(w)
    rows = (("bench", "base", bench(w), (0., 0., 0.)),
            ("glass", "rigid", glass, C.GLASS_START),
            ("cloth", "articulation", cloth(w), C.CLOTH_START))
    assets = [dict(name=name, type=kind, file=w.save(stage), position=list(pos),
                   quaternion=([0., 0., sin(np.deg2rad(C.CLOTH_YAW) / 2.),
                                cos(np.deg2rad(C.CLOTH_YAW) / 2.)]
                               if name == "cloth" else [0., 0., 0., 1.]))
              for name, kind, stage, pos in rows]
    w.inventory()
    return {"assets": assets, "settle_actions": 24, "marks": marks}
