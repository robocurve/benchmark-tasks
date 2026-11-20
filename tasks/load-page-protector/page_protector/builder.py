"""Original USD geometry for a Letter pocket, paper and the reset supports."""

import math

import numpy as np
from yam_arena import ik

from . import config as c
from .usd_scene import Writer

BINDER_BLUE = (.095, .22, .37)
STEEL = (.64, .69, .73)
FILM = (.76, .9, .95)
SEAM = (.63, .77, .84)
PAPER = (.94, .925, .86)
INK = (.12, .16, .23)


def film(w, stage, path, length, width, translation):
    from pxr import Sdf

    panel = w.Geom.Mesh.Define(stage, path)
    half_x, half_y = length / 2, width / 2
    panel.CreatePointsAttr([
        (-half_x, -half_y, 0.), (half_x, -half_y, 0.),
        (half_x, half_y, 0.), (-half_x, half_y, 0.),
    ])
    panel.CreateFaceVertexCountsAttr([4])
    panel.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    panel.CreateSubdivisionSchemeAttr("none")
    panel.CreateDoubleSidedAttr(True)
    panel.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, translation)))

    material_path = "/Asset/FilmAppearance"
    material = w.Shade.Material.Define(stage, material_path)
    shader = w.Shade.Shader.Define(stage, material_path + "/Preview")
    shader.CreateIdAttr("UsdPreviewSurface")
    tint = w.Gf.Vec3f(.55, .72, .83)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(tint)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.14)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(.18)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))

    # Render the clear pocket as one optically thin layer. The two contact
    # layers are independent and remain present in the native solver.
    mdl = w.Shade.Shader.Define(stage, material_path + "/ThinFilm")
    mdl.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    mdl.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    mdl.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(tint)
    mdl.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(.14)
    mdl.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
    mdl.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(.18)
    mdl.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.)
    material.CreateSurfaceOutput("mdl").ConnectToSource(
        mdl.CreateOutput("out", Sdf.ValueTypeNames.Token))
    material.GetPrim().CreateAttribute(
        "omni:rtx:enableCutoutOpacity", Sdf.ValueTypeNames.Bool).Set(True)
    w.Shade.MaterialBindingAPI.Apply(panel.GetPrim()).Bind(material)


def collider(w, stage, path, size, pos, rotation=None):
    w.box(stage, path, size, pos, FILM, rotation)
    w.Geom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()


def revolve(w, stage, name, parent, child, pos_a, pos_b, limits, stiffness, damping):
    j = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*pos_a))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*pos_b))
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness * math.pi / 180)
    drive.CreateDampingAttr(damping * math.pi / 180)
    drive.CreateTargetPositionAttr(0.)
    return j


def workstation(w):
    stage = w.stage("binder")
    pose = c.SLEEVE_ROTATION
    put = c.sleeve_point
    depth = .284
    binder_x = .151
    w.box(stage, "/Asset/Cover", (depth, .254, .006),
          put(binder_x, -.011, -.014), BINDER_BLUE, pose)
    w.box(stage, "/Asset/FacingCover", (depth, .245, .006),
          put(binder_x, -.274, -.014), BINDER_BLUE, pose)
    w.box(stage, "/Asset/Spine", (depth, .029, .009),
          put(binder_x, -.150, -.012), BINDER_BLUE, pose)
    w.box(stage, "/Asset/FiledPages", (.283, .238, .009),
          put(binder_x + .001, -.008, -.0054), (.74, .77, .78), pose)
    w.box(stage, "/Asset/RingBase", (.265, .013, .005),
          put(binder_x, -.131, -.0085), STEEL, pose)
    for number, x in enumerate((binder_x - .108, binder_x, binder_x + .108)):
        for i in range(22):
            angle = i * math.tau / 22
            center = put(x, -.127 + .017 * math.cos(angle), .009 + .022 * math.sin(angle))
            w.box(stage, f"/Asset/Ring{number}_{i}", (.003, .006, .004), center,
                  STEEL, pose, collide=False)
    w.box(stage, "/Asset/BinderRest", (.288, .501, .009),
          put(binder_x, -.129, -.0215), (.25, .27, .32), pose)
    w.box(stage, "/Asset/BinderStop", (.012, .501, .015),
          put(.300, -.129, -.011), (.25, .27, .32), pose)
    for i, dx in enumerate((.035, .220)):
        for j, dy in enumerate((-.319, .086)):
            contact = put(dx, dy, -.027)
            height = contact[2] - c.TABLE
            w.box(stage, f"/Asset/BinderStandLeg{i}_{j}", (.014, .014, height),
                  (*contact[:2], c.TABLE + height / 2), (.25, .27, .32))

    angle = c.STOCK_ROTATION
    rear = c.STOCK_REAR
    length = c.PAPER_LENGTH - c.STOCK_OVERHANG + .008
    center = rear + angle @ np.array([c.STOCK_OVERHANG + length / 2, 0., -.0035])
    w.box(stage, "/Asset/DocumentStand", (length, c.PAPER_WIDTH + .010, .006),
          center, BINDER_BLUE, angle)
    center = rear + angle @ np.array([c.PAPER_LENGTH + .0035, 0., .0035])
    w.box(stage, "/Asset/StandRetainer", (.005, c.PAPER_WIDTH + .010, .010),
          center, BINDER_BLUE, angle)
    for i, dx in enumerate((.08, .22)):
        for j, dy in enumerate((-.075, .075)):
            contact = rear + angle @ np.array([dx, dy, -.009])
            height = contact[2] - c.TABLE
            w.box(stage, f"/Asset/StandLeg{i}_{j}", (.012, .012, height),
                  (*contact[:2], c.TABLE + height / 2), BINDER_BLUE)
    return w.save(stage)


def sleeve(w):
    stage = w.stage("protector")
    base = w.body(stage, "/Asset/LowerFilm", .65, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(stage, base).CreateEnabledSelfCollisionsAttr(False)
    pose = c.SLEEVE_ROTATION
    orientation = ik.matrix_to_quat_xyzw(pose)
    put = c.sleeve_point
    depth = c.SLEEVE_DEPTH
    half = c.INNER_HALF_WIDTH
    hinge = put(depth, 0., c.LAYERS_GAP)
    upper = w.body(stage, "/Asset/UpperFilm", .006, position=hinge,
                   orientation=orientation)
    collider(w, stage, base + "/LowerContact", (depth, 2 * (half + .002), c.FILM_PROXY),
             put(depth / 2, 0., -c.FILM_PROXY / 2), pose)
    for side, label in [(-1, "RingSide"), (1, "OuterSide")]:
        w.box(stage, base + "/" + label, (depth, .002, .0022),
              put(depth / 2, side * (half + .001), .00025), SEAM, pose)
    # The sides remain joined when the upper film bows. Contact envelopes
    # follow the narrowing space towards the sealed end of the pocket.
    seam_sections = (.008, .076, .147, .218, depth)
    for side in (-1, 1):
        for i, (start, end) in enumerate(zip(seam_sections, seam_sections[1:])):
            height = (depth - start) * math.sin(math.radians(8.6)) + c.LAYERS_GAP
            collider(w, stage, base + f"/WeldedSide{int(side > 0)}_{i}",
                     (end - start, .00075, height),
                     put((start + end) / 2, side * (half + .00045), height / 2), pose)
    w.box(stage, base + "/BottomSeam", (.002, 2 * half, .0022),
          put(depth, 0., .00025), SEAM, pose)
    w.box(stage, base + "/ReinforcedMargin", (depth, .015, .00025),
          put(depth / 2, -half - .009, 0.), SEAM, pose)
    length = depth + c.UPPER_OVERHANG
    collider(w, stage, upper + "/UpperContact", (length, 2 * half, c.FILM_PROXY),
             (-length / 2, 0, 0))
    film(w, stage, upper + "/UpperSurface", length, 2 * half,
         (-length / 2, 0, .00002))
    w.box(stage, upper + "/TopEdge", (.00065, 2 * half, .00011),
          (-length, 0, .00004), SEAM, collide=False)
    mass = w.Phy.MassAPI.Get(stage, upper)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(-length / 2, 0, 0))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(.000025, .000044, .000069))
    joint = revolve(w, stage, "FilmFlex", base, upper, hinge, (0., 0., 0.),
                    (0., 8.6), .013, .005)
    joint.CreateLocalRot0Attr(w.Gf.Quatf(float(orientation[3]), w.Gf.Vec3f(*map(float, orientation[:3]))))
    surface = w.Phy.MaterialAPI.Get(stage, "/Asset/Physics")
    surface.CreateStaticFrictionAttr(.31)
    surface.CreateDynamicFrictionAttr(.21)
    return w.save(stage)


def paper(w):
    stage = w.stage("printed_page")
    width = c.PAPER_WIDTH
    length = c.PANEL_LENGTH
    mass = c.PAPER_MASS / len(c.PAPER_PANELS)
    thickness = c.PAPER_THICKNESS
    links = []
    for i, name in enumerate(c.PAPER_PANELS):
        link = w.body(stage, "/Asset/" + name, mass, position=(i * length, 0., 0.),
                      articulation=(i == 0), ccd=True)
        links.append(link)
        inertia = w.Phy.MassAPI.Get(stage, link)
        inertia.CreateCenterOfMassAttr(w.Gf.Vec3f(0.))
        inertia.CreateDiagonalInertiaAttr(w.Gf.Vec3f(
            mass * (width**2 + thickness**2) / 12,
            mass * (length**2 + thickness**2) / 12,
            mass * (length**2 + width**2) / 12,
        ))
        collider(w, stage, link + "/PaperContact",
                 (length - .00022, width, c.PAPER_PROXY), (0., 0., 0.))
        w.box(stage, link + "/PaperVisual", (length, width, thickness),
              (0., 0., 0.), PAPER, collide=False)
    w.Px.PhysxArticulationAPI.Get(stage, links[0]).CreateEnabledSelfCollisionsAttr(False)
    for i in range(1, len(links)):
        revolve(w, stage, f"PaperFlex{i}", links[i - 1], links[i], (length / 2, 0., 0.),
                (-length / 2, 0., 0.), (-48., 48.), c.PAPER_BEND_NM_RAD,
                c.PAPER_BEND_DAMPING_NMS_RAD)
    for i in range(23):
        x = c.PAPER_LENGTH / 2 + .106 - i * .008
        if i in (3, 11, 17):
            continue
        index = min(int(x / length), len(links) - 1)
        x -= (index + .5) * length
        width = .111 if i == 0 else .17 - (i % 5) * .009
        w.box(stage, links[index] + f"/Ink{i}", (.0028 if i < 2 else .0012, width, .00002),
              (x, -.091 + width / 2, thickness / 2 + .000012), INK, collide=False)
    surface = w.Phy.MaterialAPI.Get(stage, "/Asset/Physics")
    surface.CreateStaticFrictionAttr(.41)
    surface.CreateDynamicFrictionAttr(.29)
    return w.save(stage)


def create_assets(directory):
    w = Writer(directory)
    stock_center = c.STOCK_REAR + c.STOCK_ROTATION @ np.array(
        [c.PANEL_LENGTH / 2, 0., 0.])
    assets = [
        {"name": "workstation", "type": "base", "file": workstation(w)},
        {"name": "sleeve", "type": "articulation", "file": sleeve(w)},
        {"name": "page", "type": "articulation", "file": paper(w),
         "position": tuple(stock_center),
         "quaternion": tuple(ik.matrix_to_quat_xyzw(c.STOCK_ROTATION))},
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 12}
