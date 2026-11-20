"""Original trainer fixtures and articulated contact surfaces."""

import math


from . import config as C
from .usd_scene import Writer

ABS = (.072, .15, .20)
GREY = (.42, .47, .51)
TEAL = (.063, .38, .44)
WET = (.21, .64, .62)
RUBBER = (.17, .21, .23)
GOLD = (.90, .47, .105)
FOAM = (.26, .35, .39)
CELLULOSE = (.72, .75, .73)
GEL = (.10, .42, .52)
COVER = (.23, .35, .42)


def slider(w, s, name, fixed, moving, origin, axis, limits, stiffness, damping, cap):
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([fixed])
    joint.CreateBody1Rel().SetTargets([moving])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.)
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(cap)
    return joint


def cassette(w):
    s = w.stage("cassette")
    base = w.body(s, "/Asset/Base", .9, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, base).CreateEnabledSelfCollisionsAttr(False)
    x, y, z = C.ORIGIN
    w.box(s, base + "/Stand", (.237, .163, z - C.TABLE),
          (x+.023, y-.023, (z+C.TABLE)/2), ABS)
    w.box(s, base + "/CassetteTray", (.139, .106, .009), (x, y, z+.0045), GREY)
    for side in (-1, 1):
        w.box(s, base + f"/HingeBlock{side}", (.016, .011, .022),
              (x+.066, y+side*.052, z+.020), ABS)
    carriage_pos = C.ORIGIN + [0., 0., .010]
    carriage = w.body(s, "/Asset/Carriage", .10, position=carriage_pos)
    slider(w, s, "Shear", base, carriage, carriage_pos, "Y",
           (-C.SHEAR_LIMIT, C.SHEAR_LIMIT), C.SHEAR_STIFFNESS, 3., 4.)
    cushion = w.body(s, "/Asset/Layer", .043, position=carriage_pos)
    w.box(s, cushion + "/Foam", (.106, .082, .002), (0., 0., .001), FOAM, collide=False)
    w.box(s, cushion + "/BackingPaper", (.104, .078, .0008), (0., 0., .0024),
          CELLULOSE, collide=False)
    # The gel and membrane share a compliant support with a native roll contact.
    w.box(s, cushion + "/Gel", (C.SHEET_LENGTH, C.SHEET_WIDTH, .004),
          (0., 0., .0048), GEL)
    w.box(s, cushion + "/Membrane", (C.SHEET_LENGTH, C.SHEET_WIDTH, .0002),
          (0., 0., .0069), WET)
    w.box(s, cushion + "/DryGrip", (.025, .011, .005), (.043, .045, .0045), ABS)
    w.box(s, cushion + "/DryGripOpposite", (.025, .011, .005), (.043, -.045, .0045),
          ABS, collide=False)
    slider(w, s, "Compression", carriage, cushion, (0., 0., 0.), "Z",
           (-.005, .0001), C.CUSHION_STIFFNESS, 5., 18.)
    for k, (gx, gy, radius) in enumerate(C.GAPS):
        mark = w.Geom.Xform.Define(s, cushion + f"/Gap{k}")
        mark.AddTranslateOp().Set(w.Gf.Vec3d(gx, gy, .00712))
        mark.AddScaleOp().Set(w.Gf.Vec3f(1., 1., 1.))
        mark.CreateVisibilityAttr()
        w.cylinder(s, str(mark.GetPath())+"/Rim", radius, .00022, (0., 0., 0.),
                   (.85, .92, .92), collide=False)
        w.cylinder(s, str(mark.GetPath())+"/Highlight", radius*.67, .00025,
                   (0., 0., .00020), (.62, .79, .80), collide=False)
    lid = w.body(s, "/Asset/Cover", .10, position=C.LID_HINGE)
    half = math.radians(C.LID_OPEN_DEG) / 2
    opening = w.Gf.Quatf(math.cos(half), 0., math.sin(half), 0.)
    lid_pose = w.Geom.Xformable.Get(s, lid).GetOrderedXformOps()[1]
    lid_pose.Set(w.Gf.Quatd(math.cos(half), 0., math.sin(half), 0.))
    for side in (-1, 1):
        w.box(s, lid + f"/Side{side}", (.124, .006, .008),
              (-.061, side*.044, 0.), COVER)
    for k in range(7):
        w.box(s, lid + f"/Crossbar{k}", (.005, .095, .006),
              (-.121+k*.0195, 0., 0.), COVER)
    for k in (1, 3, 5):
        w.box(s, lid + f"/FoamStrip{k}", (.004, .075, .007),
              (-.121+k*.0195, 0., -.0065), FOAM, collide=False)
    w.box(s, lid + "/GripWeb", (.016, .058, .005),
          (C.LID_GRIP[0], 0., .0045), COVER, collide=False)
    w.box(s, lid + "/Grip", (.019, .060, .021),
          (C.LID_GRIP[0], 0., C.LID_GRIP[2]), GOLD)
    w.cylinder(s, lid + "/HingePin", .003, .108, (0., 0., 0.), GREY,
               axis="Y", collide=False)
    hinge = w.Phy.RevoluteJoint.Define(s, "/Asset/Closing")
    hinge.CreateBody0Rel().SetTargets([base])
    hinge.CreateBody1Rel().SetTargets([lid])
    hinge.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.LID_HINGE)))
    hinge.CreateLocalRot0Attr(opening)
    hinge.CreateAxisAttr("Y")
    hinge.CreateLowerLimitAttr(-C.LID_OPEN_DEG)
    hinge.CreateUpperLimitAttr(0.)
    hinge.CreateCollisionEnabledAttr(False)
    w.Px.PhysxJointAPI.Apply(hinge.GetPrim()).CreateJointFrictionAttr(.004)
    return w.save(s)


def cradle(w):
    s = w.stage("cradle")
    x, y, z = C.ROLLER_START
    # The two flat seats are entirely outboard of the fork and drum.
    # Their inside edges leave 6.5 mm of lateral clearance to the fork.
    support = z + .0195
    for k, side in enumerate((-1, 1)):
        w.box(s, f"/Asset/Seat{k}", (.009, .034, support-C.TABLE),
              (x+side*.069, y, (C.TABLE+support)/2), ABS)
    return w.save(s)


def roller(w):
    s = w.stage("roller")
    handle = w.body(s, "/Asset/Handle", .11, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(s, handle).CreateEnabledSelfCollisionsAttr(False)
    native = w.Px.PhysxRigidBodyAPI.Get(s, handle)
    native.CreateAngularDampingAttr(.8)
    w.cylinder(s, handle + "/Neck", .008, .049, (0., 0., .047), GREY)
    w.box(s, handle + "/Grip", (.020, .023, .044), (0., 0., C.GRIP_HEIGHT), GOLD)
    w.box(s, handle + "/Bridge", (.142, .017, .009), (0., 0., .0245), GREY)
    for side in (-1, 1):
        w.box(s, handle + f"/Fork{side}", (.006, .011, .027),
              (side*.055, 0., .011), GREY)
    drum = w.body(s, "/Asset/Drum", .065, ccd=True)
    w.cylinder(s, drum + "/Rubber", C.ROLLER_R, C.ROLLER_WIDTH,
               (0., 0., 0.), RUBBER, axis="X")
    joint = w.Phy.RevoluteJoint.Define(s, "/Asset/Spin")
    joint.CreateBody0Rel().SetTargets([handle])
    joint.CreateBody1Rel().SetTargets([drum])
    joint.CreateAxisAttr("X")
    joint.CreateCollisionEnabledAttr(False)
    return w.save(s)


def create_assets(directory):
    writer = Writer(directory)
    return {
        "assets": [
            dict(name="cassette", type="articulation", file=cassette(writer)),
            dict(name="cradle", type="base", file=cradle(writer)),
            dict(name="roller", type="articulation", file=roller(writer), position=C.ROLLER_START),
        ],
        "settle_actions": 12,
    }
