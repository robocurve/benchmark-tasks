"""Native rigid labware and noncolliding optical material records."""

from pathlib import Path

import numpy as np
from yam_arena.ik import matrix_to_quat_xyzw

from . import config as C
from .usd_scene import Writer

GLASS = (.71, .87, .89)
GRAPHITE = (.11, .12, .12)
DAMP = (.56, .78, .82)
DRY = (.91, .92, .86)


def glass_material(w, stage):
    from pxr import Sdf

    mat = w.Shade.Material.Define(stage, "/Asset/GlassMaterial")
    shader = w.Shade.Shader.Define(stage, "/Asset/GlassMaterial/Surface")
    shader.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    shader.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(
        w.Gf.Vec3f(.97, .99, 1.)
    )
    shader.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.10)
    shader.CreateInput("frosting_roughness", Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return mat


def bind_glass(w, stage, geom):
    from pxr import Sdf

    prim = geom.GetPrim()
    material = w.Shade.Material.Get(stage, "/Asset/GlassMaterial")
    w.Shade.MaterialBindingAPI.Apply(prim).Bind(material)
    # RTX raster shadows otherwise treat the clear glass as opaque.
    prim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)


def tank(w):
    s = w.stage("tank")
    glass_material(w, s)
    x, y, z = C.TANK_SIZE
    t = C.WALL
    for name, size, center in (
        ("Floor", (x, y, t), (0., 0., t / 2)),
        ("FrontWall", (t, y, z), (-(x - t) / 2, 0., z / 2)),
        ("BackWall", (t, y, z), ((x - t) / 2, 0., z / 2)),
        ("LeftWall", (x - 2*t, t, z), (0., (y - t) / 2, z / 2)),
        ("RightWall", (x - 2*t, t, z), (0., -(y - t) / 2, z / 2)),
    ):
        bind_glass(w, s, w.box(s, f"/Asset/{name}", size, center, GLASS))
    w.box(s, "/Asset/Saline", (x - 2*t, y - 2*t, C.SOLVENT_DEPTH),
          (0., 0., t + C.SOLVENT_DEPTH / 2), (.37, .58, .63), collide=False)
    # Glass plate support beneath the extraction space, like a TLC tank baffle.
    support_center = (.0175, 0., .066)
    bind_glass(w, s, w.box(s, "/Asset/Baffle", (.004, .065, .008),
                           support_center, GLASS))
    edge = (.55, .67, .67)
    for i, sign in enumerate((-1, 1)):
        w.box(s, f"/Asset/RimX{i}", (.0014, y, .0012),
              (sign * (x - .0014) / 2, 0., z - .0006), edge, collide=False)
        w.box(s, f"/Asset/RimY{i}", (x, .0014, .0012),
              (0., sign * (y - .0014) / 2, z - .0006), edge, collide=False)
    for mm in range(10, 101, 10):
        w.box(s, f"/Asset/Scale{mm}", (.0003, .007, .0008),
              (-x / 2 - .00015, .042, t + mm / 1000),
              (.30, .38, .38), collide=False)
    return s


def lid(w):
    s = w.stage("lid")
    glass_material(w, s)
    root = w.body(s, "/Asset", .162, ccd=True)
    bind_glass(w, s, w.box(s, root + "/GlassCover", C.LID_SIZE, (0., 0., 0.), GLASS))
    x, y, _ = C.TANK_SIZE
    z = C.LID_SKIRT
    side = C.WALL
    center = -C.LID_SIZE[2] / 2 - z / 2
    for name, size, location in (
        ("FrontSkirt", (side, y, z), (-(x - side) / 2, 0., center)),
        ("BackSkirt", (side, y, z), ((x - side) / 2, 0., center)),
        ("LeftSkirt", (x - 2 * side, side, z), (0., (y - side) / 2, center)),
        ("RightSkirt", (x - 2 * side, side, z), (0., -(y - side) / 2, center)),
    ):
        bind_glass(w, s, w.box(s, root + "/" + name, size, location, GLASS))
    w.cylinder(s, root + "/KnobStem", .009, .014, (0., 0., .0095), GRAPHITE)
    w.cylinder(s, root + "/Knob", .014, .018, (0., 0., .0225), GRAPHITE)
    return s


def plate(w):
    s = w.stage("plate")
    root = w.body(s, "/Asset", .025, ccd=True)
    w.box(s, root + "/Backing",
          (C.PLATE_WIDTH, C.PLATE_LENGTH, C.PLATE_THICK), (0., 0., 0.), DRY)
    # Thin surface overlays are rendering only. Marks are authored by material.py.
    w.box(s, root + "/WetFront", (C.PLATE_WIDTH - .001, .001, .00014),
          (0., 0., C.PLATE_THICK / 2 + .0002), DAMP, collide=False)
    w.box(s, root + "/WetSilica", (C.PLATE_WIDTH - .001, .001, .0001),
          (0., 0., C.PLATE_THICK / 2 + .0001), (.75, .85, .81), collide=False)
    w.box(s, root + "/Baseline", (C.PLATE_WIDTH - .005, .0005, .0001),
          (0., C.BASELINE - C.PLATE_LENGTH / 2, .00135), GRAPHITE, collide=False)
    for n, (x, color, fraction) in enumerate((
        (-.013, (.88, .50, .03), .68),
        (.002, (.58, .04, .07), .51),
        (.015, (.04, .20, .62), .45),
    )):
        height = C.BASELINE + fraction * (C.INITIAL_FRONT - C.BASELINE)
        w.cylinder(s, root + f"/Dye{n}", .0024, .00018,
                   (x, height - C.PLATE_LENGTH / 2, .0014), color, collide=False)
    for i in range(C.MAX_MARKS):
        mark = w.box(s, root + f"/Mark{i:02}", (.0018, .00085, .00013),
                     (0., 0., .00155), GRAPHITE, collide=False)
        mark.MakeInvisible()
    return s


def pencil(w):
    s = w.stage("pencil")
    barrel = w.body(s, "/Asset/Barrel", .011, articulation=True, ccd=True)
    w.cylinder(s, barrel + "/PlasticBarrel", .004, .109, (0., .0005, 0.),
               (.12, .26, .52), "Y")
    w.cylinder(s, barrel + "/Grip", .0041, .023, (0., -.043, 0.),
               (.13, .15, .17), "Y")
    w.cylinder(s, barrel + "/Button", .0033, .013, (0., .0615, 0.),
               (.64, .65, .64), "Y")
    cone = w.Geom.Cone.Define(s, barrel + "/Nose")
    cone.CreateAxisAttr("Y")
    cone.CreateRadiusAttr(.0039)
    cone.CreateHeightAttr(.008)
    cone.AddTranslateOp().Set(w.Gf.Vec3d(0., -.058, 0.))
    cone.AddRotateXOp().Set(180.)
    w._finish(s, cone, (.60, .62, .66), False)

    # Production pencils commonly have a rubber finger sleeve.
    grip_material = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    grip_material.GetStaticFrictionAttr().Set(1.05)
    grip_material.GetDynamicFrictionAttr().Set(.82)
    # A mechanical pencil with an axial graphite cushion. The movable graphite
    # collider is the only surface allowed to create the optical trace.
    tip = w.body(s, "/Asset/Graphite", .0007, C.PENCIL_TIP, ccd=True)
    point = w.Geom.Sphere.Define(s, tip + "/Contact")
    point.CreateRadiusAttr(.00045)
    w._finish(s, point, GRAPHITE, True, contact=.00025)
    w.cylinder(s, tip + "/Lead", .0004, .003, (0., .0012, 0.),
               GRAPHITE, axis="Y", collide=False)
    slide = w.Phy.PrismaticJoint.Define(s, "/Asset/LeadTravel")
    slide.CreateBody0Rel().SetTargets([barrel])
    slide.CreateBody1Rel().SetTargets([tip])
    slide.CreateLocalPos0Attr(w.Gf.Vec3f(*C.PENCIL_TIP))
    slide.CreateAxisAttr("Y")
    slide.CreateLowerLimitAttr(0.)
    slide.CreateUpperLimitAttr(.0022)
    drive = w.Phy.DriveAPI.Apply(slide.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.)
    drive.CreateStiffnessAttr(285.)
    drive.CreateDampingAttr(.45)
    drive.CreateMaxForceAttr(2.)
    return s


def pads(w):
    s = w.stage("pads")
    x, y, _ = C.TANK
    w.box(s, "/Asset/TankMat", (.104, .132, .004), (x, y, C.TOP + .002),
          (.24, .29, .31))
    px, py, _ = C.PLACEMENT
    lower = C.PLACEMENT - C.PLATE_R[:, 1] * C.PLATE_LENGTH / 2
    w.box(s, "/Asset/DryingRest/Base", (.061, .085, .014),
          (px + .005, py, C.TOP + .007), (.45, .60, .62))
    w.box(s, "/Asset/DryingRest/Paper", (.018, .061, .0006),
          (lower[0], py, C.TOP + .0143), (.78, .86, .86), collide=False)
    w.box(s, "/Asset/DryingRest/Toe", (.005, .067, .007),
          (lower[0] - .006, py, C.TOP + .0175), (.50, .66, .68))
    w.box(s, "/Asset/DryingRest/AftSupport", (.029, .067, .062),
          (px + .016, py, C.TOP + .045), (.52, .68, .71))
    for i, x in enumerate((C.PENCIL[0] - .047, C.PENCIL[0] + .057)):
        w.box(s, f"/Asset/Rest{i}/Base", (.012, .039, .020),
              (x, C.PENCIL[1], C.TOP + .010), (.25, .30, .32))
        for sign in (-1, 1):
            w.box(s, f"/Asset/Rest{i}/Lip{sign > 0}", (.012, .008, .004),
                  (x, C.PENCIL[1] + sign * .015, C.TOP + .022),
                  (.25, .30, .32))
    return s


def create_assets(directory):
    w = Writer(Path(directory))
    plate_rotation = matrix_to_quat_xyzw(C.PLATE_R)
    assets = []
    for name, make, kind, p, rotation in (
        ("tank", tank, "base", C.TANK, (0., 0., 0., 1.)),
        ("pads", pads, "base", (0., 0., 0.), (0., 0., 0., 1.)),
        ("cap", lid, "rigid", C.LID, (0., 0., 0., 1.)),
        ("plate", plate, "rigid", C.PLATE_CENTER, plate_rotation),
        ("pencil", pencil, "articulation", C.PENCIL, matrix_to_quat_xyzw(C.PENCIL_START)),
    ):
        assets.append(dict(name=name, file=w.save(make(w)), type=kind,
                           position=p, quaternion=rotation))
    w.inventory()
    return {"assets": assets, "settle_actions": 25}
