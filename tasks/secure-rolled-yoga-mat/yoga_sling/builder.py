"""Local USD geometry and passive PhysX constraints for mat and elastic sling."""

import math

import numpy as np

from . import config as c
from .usd_scene import Writer

FOAM = (.19, .49, .44)
FOAM_EDGE = (.11, .31, .27)
WEB = (.77, .29, .13)
PAD = (.34, .125, .092)


def spring(w, s, name, a, b, parent_at, child_at, stiffness, damping, lo, hi, target):
    j = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([a])
    j.CreateBody1Rel().SetTargets([b])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, parent_at)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, child_at)))
    j.CreateAxisAttr("Z")
    j.CreateLowerLimitAttr(lo)
    j.CreateUpperLimitAttr(hi)
    j.CreateCollisionEnabledAttr(False)
    d = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateTargetPositionAttr(target)
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)


def arc(w, s, parent, radius, width, thickness, angles, color, collide=True):
    step = angles[1] - angles[0]
    for i, angle in enumerate(angles):
        u, v = math.cos(angle), math.sin(angle)
        frame = np.array([[u, 0, -v], [0, 1, 0], [v, 0, u]])
        w.box(s, parent + f"/Arc{i:02}", (thickness, width, 2 * radius * math.tan(step / 2) + .0006),
              (radius * u, 0., radius * v), color, frame, collide=collide)


def mat(w):
    s = w.stage("mat")
    base = w.body(s, "/Asset/RolledLayers", .73, articulation=True, ccd=True)
    w.cylinder(s, base + "/InnerLayers", .0445, c.MAT_LENGTH, (0,0,0), FOAM, axis="Y")
    # Original nested ends give visible rolled layers without added grasp fixtures.
    for i, radius in enumerate((.013, .020, .027, .034, .041)):
        for k, direction in enumerate((-1, 1)):
            arc(w, s, base + f"/Layer{i}_{k}", radius, c.MAT_LENGTH + .0006 + .00005 * i,
                .00055, np.linspace(0., math.tau, 21)[:-1], FOAM_EDGE, False)
    section_angles = np.radians(np.arange(200, 380, 15))
    arc(w, s, base + "/BottomOuterTurn", c.MAT_RADIUS, c.MAT_LENGTH, .005, section_angles, FOAM)
    # The open outer turn shares its radial movement along the width of the roll.
    leaf = w.body(s, "/Asset/OuterTurn", .17, ccd=True)
    arc(w, s, leaf + "/UpperTurn", c.MAT_RADIUS, c.MAT_LENGTH, .005,
        np.radians(np.arange(15, 198, 15)), FOAM)
    spring(w, s, "Springback", base, leaf, (0., 0., 0.), (0., 0., 0.),
           290., 5.4, 0., .016, .0145)
    return w.save(s)


def flex(w, s, name, a, b, anchor, oa, ob):
    j = w.Phy.SphericalJoint.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([a])
    j.CreateBody1Rel().SetTargets([b])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, anchor - oa)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, anchor - ob)))
    j.CreateCollisionEnabledAttr(False)


def elastic_segment(w, stage, body, radius, height, degrees, origin):
    angles = np.radians(degrees)
    points = np.column_stack((radius * np.cos(angles), np.zeros(len(angles)),
                              height * np.sin(angles)))
    for i, (a, b) in enumerate(zip(points, points[1:])):
        direction = b - a
        length = np.linalg.norm(direction)
        tangent = direction / length
        radial = np.cross([0., 1., 0.], tangent)
        frame = np.column_stack((radial, [0., 1., 0.], tangent))
        w.box(stage, body + f"/Tape{i:02}", (.003, c.BAND_WIDTH, length + .0005),
              (a + b) / 2 - origin, WEB, frame)


def bending_wing(w, stage, parent, name, center, degrees, height):
    angle = math.radians(degrees[0])
    anchor = np.array([c.BAND_RADIUS * math.cos(angle), 0., height * math.sin(angle)])
    wing = w.body(stage, "/Asset/" + name, .006, position=center + anchor, ccd=True)
    elastic_segment(w, stage, wing, c.BAND_RADIUS, height, degrees, anchor)
    j = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name + "Bend")
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([wing])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, anchor)))
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(-12.)
    j.CreateUpperLimitAttr(12.)
    j.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.)
    drive.CreateStiffnessAttr(.75 * math.pi / 180)
    drive.CreateDampingAttr(.038 * math.pi / 180)
    return wing


def sling(w):
    s = w.stage("sling")
    center = np.array([-.14, 0., .005])
    shoulder = w.body(s, "/Asset/ShoulderWebbing", .033, position=center, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(s, shoulder).CreateEnabledSelfCollisionsAttr(False)
    w.box(s, shoulder + "/GripMaterial", (.030, .095, .0035), (0.,0.,0.), PAD)
    for sign, side in ((1, "Left"), (-1, "Right")):
        points = [
            center + [0., sign * .0475, 0.],
            np.array([-.16, sign * .207, .007]),
            np.array([-.088, sign * c.BAND_Y, .017]),
            np.array([0., sign * c.BAND_Y, c.BAND_UPPER_HEIGHT + .002]),
        ]
        parent, origin = shoulder, center
        for i, (u, v) in enumerate(zip(points, points[1:])):
            length = np.linalg.norm(v - u)
            y = (v - u) / length
            x = np.cross(y, [0.,0.,1.])
            x /= np.linalg.norm(x)
            z = np.cross(x, y)
            link_position = (u + v) / 2
            link = w.body(s, f"/Asset/{side}Web{i}", .014, position=link_position)
            w.box(s, link + "/Webbing", (.026, length + .0006, .0035), (0.,0.,0.),
                  WEB, np.column_stack((x, y, z)))
            flex(w, s, f"{side}Swing{i}", parent, link, u, origin, link_position)
            parent, origin = link, link_position
        loop_pos = np.array([0., sign * c.BAND_Y, 0.])
        outer = w.body(s, "/Asset/" + side + "BandTop", .029, position=loop_pos, ccd=True)
        lower = w.body(s, "/Asset/" + side + "BandBottom", .023, position=loop_pos, ccd=True)
        elastic_segment(w, s, outer, c.BAND_RADIUS, c.BAND_UPPER_HEIGHT,
                        np.arange(60, 121, 10), np.zeros(3))
        elastic_segment(w, s, lower, c.BAND_RADIUS, c.BAND_LOWER_HEIGHT,
                        np.arange(240, 301, 10), np.zeros(3))
        upper_positive = bending_wing(w, s, outer, side + "UpperPositive", loop_pos,
                                      np.arange(60, -1, -10), c.BAND_UPPER_HEIGHT)
        upper_negative = bending_wing(w, s, outer, side + "UpperNegative", loop_pos,
                                      np.arange(120, 181, 10), c.BAND_UPPER_HEIGHT)
        bending_wing(w, s, lower, side + "LowerPositive", loop_pos,
                     np.arange(300, 361, 10), c.BAND_LOWER_HEIGHT)
        bending_wing(w, s, lower, side + "LowerNegative", loop_pos,
                     np.arange(240, 179, -10), c.BAND_LOWER_HEIGHT)
        for wing, end in ((upper_positive, 1), (upper_negative, -1)):
            anchor_x = end * .5 * c.BAND_RADIUS
            w.box(s, wing + "/ElasticSplice", (.003, c.BAND_WIDTH, .032),
                  (end * c.BAND_RADIUS - anchor_x, 0., -.007 - c.BAND_UPPER_HEIGHT * math.sqrt(3) / 2), WEB)
        flex(w, s, side + "BandSeam", parent, outer, points[-1], origin, loop_pos)
        spring(w, s, side + "Elastic", outer, lower, (0.,0.,0.), (0.,0.,0.),
               210., 3.5, -.020, .016, c.BAND_CLOSE_TRAVEL)
    return w.save(s)


def riser(w):
    s = w.stage("riser")
    w.box(s, "/Asset/NonslipPad", (.181, .409, .017),
          (c.MAT_X, 0., c.TABLE_TOP + .0085), (.16,.19,.20))
    return w.save(s)


def create_assets(directory):
    w = Writer(directory)
    return {
        "assets": [
            {"name": "mat", "type": "articulation", "file": mat(w),
             "position": (c.MAT_X, 0., c.MAT_Z)},
            {"name": "sling", "type": "articulation", "file": sling(w),
             "position": (c.BAND_X, 0., c.BAND_Z + c.WARMUP_CLEARANCE)},
            {"name": "riser", "type": "base", "file": riser(w)},
        ],
        "settle_actions": 7,
    }
